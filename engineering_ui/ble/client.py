from __future__ import annotations

import asyncio
import json
import subprocess
import sys
import threading
from dataclasses import dataclass

from PySide6.QtCore import QObject, Signal

from ble.protocol import AUTH_UUID, COMMAND_UUID, DEVICE_NAME, SERVICE_UUID, TELEMETRY_UUID

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # pragma: no cover
    BleakClient = None
    BleakScanner = None

SCAN_DISCOVER_TIMEOUT_S = 2.5
SCAN_PROCESS_TIMEOUT_S = 6.0
COMMAND_WRITE_SPACING_S = 0.08
COMMAND_RETRY_BASE_DELAY_S = 0.25


@dataclass
class BleDevice:
    name: str
    address: str
    is_robot: bool = False


class BleRobotClient(QObject):
    devices_changed = Signal(list)
    connected_changed = Signal(bool)
    authenticated_changed = Signal(bool)
    telemetry_packet = Signal(bytes)
    error = Signal(str)
    status = Signal(str)

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self._client: BleakClient | None = None
        self._devices: list[BleDevice] = []
        self._write_lock: asyncio.Lock | None = None
        self._last_command_write_at = 0.0
        self._realtime_packets: dict[int, bytes] = {}
        self._realtime_task_running = False
        self._notifications_started = False
        self._scan_running = False
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, name="ble-asyncio", daemon=True)
        self._thread.start()

    @property
    def devices(self) -> list[BleDevice]:
        return self._devices

    def scan(self) -> None:
        self._submit(self._scan())

    def connect(self, address: str) -> None:
        self._submit(self._connect(address))

    def disconnect(self) -> None:
        self._submit(self._disconnect())

    def authenticate(self, token: str) -> None:
        self._submit(self._authenticate(token))

    def keepalive(self, token: str) -> None:
        self._submit(self._keepalive(token))

    def write_command(self, packet: bytes) -> None:
        if self._is_realtime_motor_command(packet):
            self._loop.call_soon_threadsafe(self._queue_realtime_command, packet)
            return
        self._submit(self._write_command(packet))

    def shutdown(self) -> None:
        future = asyncio.run_coroutine_threadsafe(self._disconnect(), self._loop)
        try:
            future.result(timeout=2.0)
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=2.0)

    async def _scan(self) -> None:
        if BleakScanner is None:
            self.error.emit("Bleak nao esta instalado")
            return
        if self._scan_running:
            self.status.emit("Scan BLE ja esta em andamento")
            return

        self._scan_running = True
        self.status.emit("Escaneando BLE...")
        print("[BLE UI] scan start", flush=True)
        try:
            raw_devices = await asyncio.to_thread(self._scan_in_subprocess)
            devices = []
            for raw in raw_devices:
                name = raw["name"] or "sem nome"
                address = raw["address"]
                service_uuids = {uuid.lower() for uuid in raw.get("service_uuids", [])}
                is_robot = name.startswith(DEVICE_NAME) or SERVICE_UUID.lower() in service_uuids
                devices.append(BleDevice(name, address, is_robot))
            devices.sort(key=lambda d: (not d.is_robot, d.name.lower(), d.address))
            self._devices = devices
            self.devices_changed.emit(devices)
            robot_count = sum(1 for device in devices if device.is_robot)
            self.status.emit(f"Scan concluido: {robot_count} robo(s), {len(devices)} dispositivo(s)")
            print(f"[BLE UI] scan done robots={robot_count} devices={len(devices)}", flush=True)
        except TimeoutError as exc:
            message = f"Falha no scan BLE: {exc}"
            print(f"[BLE UI] scan timeout: {exc}", flush=True)
            self.error.emit(message)
        except Exception as exc:
            print(f"[BLE UI] scan failed: {exc}", flush=True)
            self.error.emit(f"Falha no scan BLE: {exc}")
        finally:
            self._scan_running = False

    @staticmethod
    def _scan_in_subprocess() -> list[dict]:
        script = r"""
import asyncio
import json
from bleak import BleakScanner

async def main():
    found = await BleakScanner.discover(timeout=__SCAN_DISCOVER_TIMEOUT_S__, return_adv=True)
    devices = []
    for device, advertisement in found.values():
        devices.append({
            "name": device.name or advertisement.local_name or "",
            "address": device.address,
            "service_uuids": list(advertisement.service_uuids or []),
        })
    print(json.dumps(devices), flush=True)

asyncio.run(main())
""".replace("__SCAN_DISCOVER_TIMEOUT_S__", f"{SCAN_DISCOVER_TIMEOUT_S:.1f}")
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        try:
            completed = subprocess.run(
                [sys.executable, "-c", script],
                capture_output=True,
                text=True,
                timeout=SCAN_PROCESS_TIMEOUT_S,
                check=False,
                creationflags=creationflags,
            )
        except subprocess.TimeoutExpired as exc:
            raise TimeoutError(f"backend BLE do Windows nao respondeu em {SCAN_PROCESS_TIMEOUT_S:.0f}s") from exc

        if completed.returncode != 0:
            stderr = completed.stderr.strip() or completed.stdout.strip() or f"codigo {completed.returncode}"
            raise RuntimeError(stderr)

        output = completed.stdout.strip()
        if not output:
            return []

        return json.loads(output)

    async def _connect(self, address: str) -> None:
        if BleakClient is None:
            self.error.emit("Bleak nao esta instalado")
            return

        await self._disconnect()
        self.status.emit(f"Conectando BLE: {address}")
        print(f"[BLE UI] connect address={address}", flush=True)
        try:
            self._client = BleakClient(address, disconnected_callback=self._on_disconnect)
            await self._client.connect(timeout=12.0)
            if not self._client.is_connected:
                raise RuntimeError("cliente BLE nao confirmou conexao")
            await self._validate_robot_gatt()
            self.connected_changed.emit(True)
            self.status.emit("BLE conectado; aguardando autenticacao")
            print("[BLE UI] connected notify_started=0", flush=True)
        except Exception as exc:
            print(f"[BLE UI] connect failed: {exc}", flush=True)
            try:
                if self._client and self._client.is_connected:
                    await self._client.disconnect()
            except Exception:
                pass
            self._client = None
            self.error.emit(f"Falha BLE: {exc}")
            self.connected_changed.emit(False)
            self.authenticated_changed.emit(False)

    async def _validate_robot_gatt(self) -> None:
        if not self._client:
            raise RuntimeError("BLE nao conectado")

        services = self._client.services
        if hasattr(self._client, "get_services"):
            try:
                services = await self._client.get_services()
            except Exception:
                services = self._client.services

        service_uuids = {service.uuid.lower() for service in services}
        char_uuids = {
            char.uuid.lower()
            for service in services
            for char in service.characteristics
        }
        required_chars = {AUTH_UUID.lower(), COMMAND_UUID.lower(), TELEMETRY_UUID.lower()}
        missing_chars = sorted(required_chars - char_uuids)
        if SERVICE_UUID.lower() not in service_uuids or missing_chars:
            raise RuntimeError(
                "dispositivo conectado nao expõe o GATT do robo "
                f"(service={SERVICE_UUID}, faltando={', '.join(missing_chars) or 'service'})"
            )

    async def _disconnect(self) -> None:
        try:
            if self._client and self._client.is_connected:
                print("[BLE UI] disconnect requested", flush=True)
                await self._stop_notifications()
                await self._client.disconnect()
        finally:
            self._client = None
            self._notifications_started = False
            self.connected_changed.emit(False)
            self.authenticated_changed.emit(False)

    async def _authenticate(self, token: str) -> None:
        if not self._client or not self._client.is_connected:
            self.error.emit("BLE nao conectado")
            return
        try:
            print(f"[BLE UI] auth write len={len(token)}", flush=True)
            await self._client.write_gatt_char(AUTH_UUID, token.encode("utf-8"), response=True)
            await self._start_notifications()
            self.authenticated_changed.emit(True)
            self.status.emit("BLE autenticado")
        except Exception as exc:
            print(f"[BLE UI] auth failed: {exc}", flush=True)
            self.authenticated_changed.emit(False)
            self.error.emit(f"Autenticacao negada: {exc}")

    async def _keepalive(self, token: str) -> None:
        if not self._client or not self._client.is_connected or not self._notifications_started:
            return
        if self._write_lock is None:
            self._write_lock = asyncio.Lock()
        try:
            async with self._write_lock:
                await self._wait_for_command_slot()
                await self._client.write_gatt_char(AUTH_UUID, token.encode("utf-8"), response=True)
                self._last_command_write_at = self._loop.time()
        except Exception as exc:
            print(f"[BLE UI] keepalive ignored: {exc}", flush=True)

    async def _start_notifications(self) -> None:
        if not self._client or not self._client.is_connected or self._notifications_started:
            return
        await self._client.start_notify(TELEMETRY_UUID, self._on_telemetry)
        self._notifications_started = True
        print("[BLE UI] notifications started", flush=True)

    async def _stop_notifications(self) -> None:
        if not self._client or not self._client.is_connected or not self._notifications_started:
            return
        try:
            await self._client.stop_notify(TELEMETRY_UUID)
        except Exception as exc:
            print(f"[BLE UI] stop notify ignored: {exc}", flush=True)
        finally:
            self._notifications_started = False

    async def _write_command(self, packet: bytes) -> None:
        if not self._client or not self._client.is_connected:
            self.error.emit("BLE nao conectado")
            return
        if self._write_lock is None:
            self._write_lock = asyncio.Lock()
        try:
            async with self._write_lock:
                max_attempts = 2 if self._is_realtime_motor_command(packet) else 6
                for attempt in range(1, max_attempts + 1):
                    if len(packet) >= 4:
                        print(
                            f"[BLE UI] command write class=0x{packet[1]:02x} id=0x{packet[2]:02x} payload={packet[3]} total={len(packet)} attempt={attempt}",
                            flush=True,
                        )
                    try:
                        await self._wait_for_command_slot()
                        await self._client.write_gatt_char(COMMAND_UUID, packet, response=True)
                        self._last_command_write_at = self._loop.time()
                        if len(packet) >= 4:
                            print(f"[BLE UI] command ok id=0x{packet[2]:02x}", flush=True)
                        return
                    except Exception as exc:
                        if attempt >= max_attempts or not self._is_insufficient_resource(exc):
                            raise
                        print(f"[BLE UI] command retry after insufficient resource: {exc}", flush=True)
                        self._last_command_write_at = self._loop.time()
                        await asyncio.sleep(COMMAND_RETRY_BASE_DELAY_S * attempt)
        except Exception as exc:
            if len(packet) >= 4:
                print(f"[BLE UI] command failed id=0x{packet[2]:02x}: {exc}", flush=True)
            else:
                print(f"[BLE UI] command failed: {exc}", flush=True)
            self.error.emit(f"Falha ao enviar comando: {exc}")

    async def _wait_for_command_slot(self) -> None:
        elapsed = self._loop.time() - self._last_command_write_at
        if elapsed < COMMAND_WRITE_SPACING_S:
            await asyncio.sleep(COMMAND_WRITE_SPACING_S - elapsed)

    def _queue_realtime_command(self, packet: bytes) -> None:
        if len(packet) < 4:
            return
        self._realtime_packets[packet[2]] = packet
        if not self._realtime_task_running:
            self._realtime_task_running = True
            self._loop.create_task(self._write_realtime_commands())

    async def _write_realtime_commands(self) -> None:
        try:
            while self._realtime_packets:
                pending = list(self._realtime_packets.values())
                self._realtime_packets.clear()
                for packet in pending:
                    await self._write_command(packet)
        finally:
            self._realtime_task_running = False
            if self._realtime_packets:
                self._realtime_task_running = True
                self._loop.create_task(self._write_realtime_commands())

    @staticmethod
    def _is_realtime_motor_command(packet: bytes) -> bool:
        return len(packet) >= 4 and packet[0] == 0x01 and packet[1] == 0x03 and packet[2] in (0x10, 0x11, 0x12)

    @staticmethod
    def _is_insufficient_resource(exc: Exception) -> bool:
        message = str(exc).lower()
        return "insufficient resource" in message or "0x11" in message

    def _on_disconnect(self, client: BleakClient) -> None:
        self._client = None
        self._notifications_started = False
        self.connected_changed.emit(False)
        self.authenticated_changed.emit(False)
        self.status.emit("BLE desconectado")
        print("[BLE UI] disconnected callback", flush=True)

    def _on_telemetry(self, sender: int, data: bytearray) -> None:
        if len(data) >= 4 and data[1] == 0x04 and data[2] in (0x20, 0x21):
            print(f"[BLE UI] telemetry map id=0x{data[2]:02x} payload={data[3]} total={len(data)}", flush=True)
        self.telemetry_packet.emit(bytes(data))

    def _submit(self, coro) -> None:
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        future.add_done_callback(self._done)

    def _done(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.error.emit(f"Erro BLE interno: {exc}")

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()
