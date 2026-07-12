from __future__ import annotations

import argparse
import asyncio
import math
import time
from collections import Counter
from dataclasses import dataclass, field

from bleak import BleakClient, BleakScanner

from ble.protocol import (
    AUTH_TOKEN,
    AUTH_UUID,
    COMMAND_UUID,
    DEVICE_NAME,
    SERVICE_UUID,
    TELEMETRY_UUID,
    CommandClass,
    ErrorCode,
    TelemetryId,
    pack_read_map_chunk,
    pack_read_map_list,
    pack_save_map_chunk,
    unpack_encoder_telemetry,
    unpack_imu_fast_telemetry,
    unpack_imu_telemetry,
    unpack_telemetry_bundle,
    unpack_map_chunk,
    unpack_map_list,
    unpack_odometry_telemetry,
    unpack_packet,
)
from commands import robot_commands


MAP_COMMAND_CLASSES = {int(CommandClass.SAVE), int(CommandClass.READ)}


@dataclass
class DiagnosticState:
    notifications: int = 0
    packets: Counter[str] = field(default_factory=Counter)
    statuses: list[int] = field(default_factory=list)
    maps: list[dict] = field(default_factory=list)
    map_chunks: dict[tuple[int, int], dict] = field(default_factory=dict)
    odometry_samples: list[dict] = field(default_factory=list)
    encoder_samples: list[dict] = field(default_factory=list)
    imu_samples: list[dict] = field(default_factory=list)


def packet_label(command_class: int, message_id: int) -> str:
    if command_class == int(CommandClass.TELE):
        try:
            return f"TELE.{TelemetryId(message_id).name}"
        except ValueError:
            return f"TELE.0x{message_id:02x}"
    try:
        return f"{CommandClass(command_class).name}.0x{message_id:02x}"
    except ValueError:
        return f"0x{command_class:02x}.0x{message_id:02x}"


def is_insufficient_resource(exc: Exception) -> bool:
    message = str(exc).lower()
    return "insufficient resource" in message or "0x11" in message


def path_distance(points: list[tuple[float, float]]) -> float:
    return sum(math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]) for i in range(1, len(points)))


class BleDiagnostic:
    def __init__(self, address: str | None, verbose: bool) -> None:
        self.address = address
        self.verbose = verbose
        self.state = DiagnosticState()
        self._map_list_event = asyncio.Event()
        self._map_chunk_event = asyncio.Event()
        self._status_event = asyncio.Event()

    async def scan(self, timeout_s: float):
        print(f"[scan] procurando por {DEVICE_NAME} por {timeout_s:.1f}s")
        found = await BleakScanner.discover(timeout=timeout_s, return_adv=True)
        robot_device = None
        for device, adv in found.values():
            name = device.name or adv.local_name or ""
            uuids = {uuid.lower() for uuid in (adv.service_uuids or [])}
            is_robot = name.startswith(DEVICE_NAME) or SERVICE_UUID.lower() in uuids
            print(f"[scan] {'ROBO ' if is_robot else '     '}name={name or '-'} address={device.address} uuids={sorted(uuids)}")
            if self.address and device.address.lower() == self.address.lower():
                robot_device = device
            elif is_robot and robot_device is None:
                robot_device = device
        if robot_device is not None:
            print(f"[scan] alvo={robot_device.address}")
            return robot_device
        if self.address is not None:
            print(f"[scan] alvo explicito nao anunciado, tentando por endereco={self.address}")
            return self.address
        if robot_device is None:
            raise RuntimeError("robo nao encontrado no scan BLE")

    def on_notify(self, sender: int, data: bytearray) -> None:
        self.state.notifications += 1
        raw = bytes(data)
        try:
            command_class, message_id, payload = unpack_packet(raw)
        except Exception as exc:
            print(f"[notify] invalido sender={sender} len={len(raw)} data={raw.hex(' ')} err={exc}")
            return

        label = packet_label(int(command_class), message_id)
        self.state.packets[label] += 1
        if self.verbose or message_id in (TelemetryId.STATUS, TelemetryId.MAP_LIST, TelemetryId.MAP_CHUNK):
            print(f"[notify] {label} payload={len(payload)} total={len(raw)}")

        if command_class != CommandClass.TELE:
            return

        if message_id == TelemetryId.BUNDLE:
            try:
                records = unpack_telemetry_bundle(payload)
            except Exception as exc:
                print(f"[notify] falha parse {label}: {exc}")
                return
            for bundled_message_id, bundled_payload in records:
                bundled_label = packet_label(int(CommandClass.TELE), bundled_message_id)
                self.state.packets[bundled_label] += 1
                self._handle_telemetry_payload(bundled_message_id, bundled_payload, bundled_label)
            return

        self._handle_telemetry_payload(message_id, payload, label)

    def _handle_telemetry_payload(self, message_id: int, payload: bytes, label: str) -> None:
        try:
            if message_id == TelemetryId.STATUS:
                status = payload[0] if payload else -1
                self.state.statuses.append(status)
                status_name = ErrorCode(status).name if status in [int(e) for e in ErrorCode] else f"0x{status:02x}"
                print(f"[status] {status_name}")
                self._status_event.set()
            elif message_id == TelemetryId.MAP_LIST:
                self.state.maps = unpack_map_list(payload)
                print(f"[map-list] count={len(self.state.maps)} maps={self.state.maps}")
                self._map_list_event.set()
            elif message_id == TelemetryId.MAP_CHUNK:
                chunk = unpack_map_chunk(payload)
                key = (int(chunk["slot"]), int(chunk["offset"]))
                self.state.map_chunks[key] = chunk
                print(
                    f"[map-chunk] slot={chunk['slot']} offset={chunk['offset']} "
                    f"count={chunk['point_count']} total={chunk['total_points']}"
                )
                self._map_chunk_event.set()
            elif message_id == TelemetryId.ODOMETRY:
                sample = unpack_odometry_telemetry(payload)
                self.state.odometry_samples.append(sample)
                if self.verbose:
                    print(
                        f"[odom] x={sample['x_m']:.3f} y={sample['y_m']:.3f} "
                        f"fused=({sample['fused_x_m']:.3f},{sample['fused_y_m']:.3f})"
                    )
            elif message_id == TelemetryId.ENCODERS:
                self.state.encoder_samples.append(unpack_encoder_telemetry(payload))
            elif message_id == TelemetryId.IMU:
                self.state.imu_samples.append(unpack_imu_telemetry(payload))
            elif message_id == TelemetryId.IMU_FAST:
                self.state.imu_samples.append(unpack_imu_fast_telemetry(payload))
        except Exception as exc:
            print(f"[notify] falha parse {label}: {exc}")

    async def write_command(self, client: BleakClient, packet: bytes, label: str, attempts: int = 6) -> bool:
        retryable = len(packet) >= 4 and packet[1] in MAP_COMMAND_CLASSES
        max_attempts = attempts if retryable else 2
        for attempt in range(1, max_attempts + 1):
            print(f"[write] {label} len={len(packet)} payload={packet[3] if len(packet) >= 4 else '-'} attempt={attempt}")
            try:
                await client.write_gatt_char(COMMAND_UUID, packet, response=True)
                await asyncio.sleep(0.18)
                return True
            except Exception as exc:
                print(f"[write] falhou {label}: {exc}")
                if attempt >= max_attempts or not (retryable and is_insufficient_resource(exc)):
                    return False
                await asyncio.sleep(0.40 * attempt)
        return False

    async def wait_for(self, event: asyncio.Event, timeout_s: float, label: str) -> bool:
        try:
            await asyncio.wait_for(event.wait(), timeout=timeout_s)
            return True
        except TimeoutError:
            print(f"[wait] timeout aguardando {label}")
            return False
        finally:
            event.clear()

    async def exercise(self, client: BleakClient, write_test: bool) -> None:
        await self.write_command(client, robot_commands.stop(), "stop")
        await self.wait_for(self._status_event, 2.0, "status stop")

        await self.write_command(client, pack_read_map_list(), "read map list")
        await self.wait_for(self._map_list_event, 4.0, "map list")

        if self.state.maps:
            first = self.state.maps[0]
            slot = int(first["slot"])
            expected_total = int(first["point_count"])
            print(f"[load] carregando slot={slot} expected_total={expected_total}")
            offset = 0
            loaded: list[tuple[float, float]] = []
            while offset < expected_total:
                if not await self.write_command(client, pack_read_map_chunk(slot, offset), f"read map chunk slot={slot} offset={offset}"):
                    break
                if not await self.wait_for(self._map_chunk_event, 4.0, "map chunk"):
                    break
                chunk = self.state.map_chunks.get((slot, offset))
                if not chunk:
                    break
                loaded.extend(chunk["points"])
                point_count = int(chunk["point_count"])
                if point_count <= 0:
                    break
                offset += point_count
                await asyncio.sleep(0.25)
            print(f"[load] pontos={len(loaded)}/{expected_total} distancia_calc={path_distance(loaded):.3f}m")

        if write_test:
            points = [(0.0, 0.0), (0.25, 0.0), (0.25, 0.25), (0.0, 0.25)]
            packet = pack_save_map_chunk("diag_ble", len(points), 0, points)
            print(f"[save-test] salvando mapa diag_ble len={len(packet)} distancia={path_distance(points):.3f}m")
            if await self.write_command(client, packet, "save diag_ble"):
                await self.wait_for(self._status_event, 4.0, "status save")
                await asyncio.sleep(1.0)
                await self.write_command(client, pack_read_map_list(), "read map list after save")
                await self.wait_for(self._map_list_event, 4.0, "map list after save")

    async def run(self, scan_timeout_s: float, listen_s: float, write_test: bool) -> None:
        target = await self.scan(scan_timeout_s)
        disconnected = asyncio.Event()

        def on_disconnect(_: BleakClient) -> None:
            print("[conn] desconectado")
            disconnected.set()

        target_label = getattr(target, "address", target)
        print(f"[conn] conectando {target_label}")
        async with BleakClient(target, disconnected_callback=on_disconnect, timeout=12.0) as client:
            print(f"[conn] connected={client.is_connected} mtu={getattr(client, 'mtu_size', 'n/a')}")
            print("[services]")
            for service in client.services:
                print(f"  service {service.uuid}")
                for char in service.characteristics:
                    print(f"    char {char.uuid} handle={char.handle} props={char.properties}")

            print("[auth] escrevendo token antes de habilitar notify")
            await client.write_gatt_char(AUTH_UUID, AUTH_TOKEN.encode("utf-8"), response=True)
            await asyncio.sleep(0.4)
            await client.start_notify(TELEMETRY_UUID, self.on_notify)
            print(f"[listen] coletando telemetria por {listen_s:.1f}s antes dos comandos")
            await asyncio.sleep(listen_s)
            await self.exercise(client, write_test=write_test)
            print("[listen] coletando telemetria final por 2.0s")
            await asyncio.sleep(2.0)
            await client.stop_notify(TELEMETRY_UUID)
            await self.write_command(client, robot_commands.stop(), "final stop")

        print("[summary]")
        print(f"  notifications={self.state.notifications}")
        print(f"  packets={dict(self.state.packets)}")
        print(f"  statuses={self.state.statuses}")
        print(f"  maps={self.state.maps}")
        print(f"  odom_samples={len(self.state.odometry_samples)} encoder_samples={len(self.state.encoder_samples)} imu_samples={len(self.state.imu_samples)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Diagnostico BLE do AspiradorPista-S3")
    parser.add_argument("--address", help="Endereco BLE do robo. Se omitido, usa scan.")
    parser.add_argument("--scan-timeout", type=float, default=6.0)
    parser.add_argument("--listen", type=float, default=2.0)
    parser.add_argument("--write-test", action="store_true", help="Salva um mapa pequeno diag_ble no robo.")
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


async def main() -> None:
    args = parse_args()
    diagnostic = BleDiagnostic(args.address, args.verbose)
    await diagnostic.run(args.scan_timeout, args.listen, args.write_test)


if __name__ == "__main__":
    asyncio.run(main())
