"""Transport regression tests; no Bluetooth adapter or Qt event loop required."""
import asyncio
from collections import deque
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ble.client import BleRobotClient, COMMAND_QUEUE_LIMIT
from commands.robot_commands import stop, set_left_pwm, set_right_pwm, set_aux_pwm


class Signal:
    def __init__(self):
        self.messages = []

    def emit(self, *args):
        self.messages.append(args)


class Transport:
    is_connected = True

    def __init__(self):
        self.packets = []
        self.started = asyncio.Event()
        self.release = asyncio.Event()
        self.block_first = False
        self.resource_error = False

    async def write_gatt_char(self, uuid, packet, response):
        assert response
        self.packets.append(packet)
        self.started.set()
        if self.resource_error:
            self.resource_error = False
            raise RuntimeError('Insufficient resources 0x11')
        if self.block_first and len(self.packets) == 1:
            await self.release.wait()

    async def disconnect(self):
        self.is_connected = False


class QueueTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.client = BleRobotClient.__new__(BleRobotClient)
        c = self.client
        c._loop = asyncio.get_running_loop()
        c._client = self.transport = Transport()
        c._write_lock = None
        c._last_command_write_at = 0
        c._command_queue = deque()
        c._command_worker = None
        c._command_epoch = 0
        c._command_wakeup = None
        c._notifications_started = False
        for name in ('error', 'status', 'connected_changed', 'authenticated_changed'):
            setattr(c, name, Signal())

    async def drain(self):
        await asyncio.wait_for(self.client._command_worker, 1)

    async def test_stop_discards_old_pwm_and_start(self):
        self.transport.block_first = True
        self.client._enqueue_command(set_left_pwm(40))
        await self.transport.started.wait()
        self.client._enqueue_command(set_right_pwm(40))
        self.client._enqueue_command(set_aux_pwm(40))
        self.client._enqueue_command(bytes([1, 3, 0x44, 1, 25]))
        self.client._enqueue_command(stop())
        self.transport.release.set()
        await self.drain()
        self.assertEqual(self.transport.packets, [set_left_pwm(40), stop()])

    async def test_stop_interrupts_resource_backoff(self):
        self.transport.resource_error = True
        self.client._enqueue_command(set_left_pwm(40))
        await self.transport.started.wait()
        # Let wait_for propagate the ATT error into the retry wait.
        while self.client._command_wakeup is None:
            await asyncio.sleep(0)
        self.client._enqueue_command(stop())
        await self.drain()
        self.assertEqual(self.transport.packets, [set_left_pwm(40), stop()])

    async def test_record_stop_discards_queued_record_start_and_reads(self):
        record_stop = bytes([1, 3, 0x23, 0])
        self.transport.block_first = True
        self.client._enqueue_command(set_left_pwm(40))
        await self.transport.started.wait()
        self.client._enqueue_command(bytes([1, 3, 0x22, 0]))
        self.client._enqueue_command(bytes([1, 2, 3, 2, 0, 0]))
        self.client._enqueue_command(record_stop)
        self.transport.release.set()
        await self.drain()
        self.assertEqual(self.transport.packets, [set_left_pwm(40), record_stop])

    async def test_latest_pwm_is_coalesced_without_crossing_start(self):
        start = bytes([1, 3, 0x44, 1, 25])
        for packet in [set_left_pwm(10), set_left_pwm(20), start, set_left_pwm(30)]:
            self.client._enqueue_command(packet)
        await self.drain()
        self.assertEqual(self.transport.packets, [set_left_pwm(20), start, set_left_pwm(30)])

    async def test_full_queue_still_accepts_stop(self):
        for offset in range(COMMAND_QUEUE_LIMIT + 1):
            self.client._enqueue_command(bytes([1, 2, 3, 2, offset, 0]))
        self.assertEqual(len(self.client._command_queue), COMMAND_QUEUE_LIMIT)
        self.assertTrue(self.client.error.messages)
        self.client._enqueue_command(stop())
        await self.drain()
        self.assertEqual(self.transport.packets, [stop()])

    async def test_control_precedes_queued_background_reads(self):
        self.transport.block_first = True
        self.client._enqueue_command(set_left_pwm(0))
        await self.transport.started.wait()
        read = bytes([1, 2, 1, 0])
        self.client._enqueue_command(read)
        self.client._enqueue_command(read)
        self.client._enqueue_command(set_right_pwm(0))
        self.transport.release.set()
        await self.drain()
        self.assertEqual(self.transport.packets, [set_left_pwm(0), set_right_pwm(0), read])

    async def test_disconnect_discards_old_commands(self):
        self.client._enqueue_command(set_left_pwm(40))
        await self.client._disconnect()
        await self.drain()
        self.assertEqual(self.transport.packets, [])

    async def test_stale_disconnect_does_not_clear_new_connection(self):
        self.client._on_disconnect(Transport())
        self.assertIs(self.client._client, self.transport)

    async def test_timeout_disconnects_without_replaying(self):
        self.transport.block_first = True
        with patch('ble.client.COMMAND_WRITE_TIMEOUT_S', 0.01):
            self.client._enqueue_command(set_left_pwm(40))
            self.client._enqueue_command(set_right_pwm(40))
            await self.drain()
        self.assertEqual(self.transport.packets, [set_left_pwm(40)])
        self.assertFalse(self.transport.is_connected)

    async def test_successful_writes_need_no_fixed_sleep(self):
        with patch('ble.client.asyncio.sleep', side_effect=AssertionError('unnecessary pacing')):
            self.client._enqueue_command(set_left_pwm(40))
            self.client._enqueue_command(set_right_pwm(40))
            await self.drain()
        self.assertEqual(len(self.transport.packets), 2)

    async def test_failed_configuration_discards_start(self):
        async def failing_write(*args, **kwargs):
            raise RuntimeError('ATT configuration rejected')
        self.transport.write_gatt_char = failing_write
        self.client._enqueue_command(bytes([1, 3, 0x43, 1, 2]))
        self.client._enqueue_command(bytes([1, 3, 0x44, 1, 25]))
        await self.drain()
        self.assertEqual(len(self.client.error.messages), 1)
        self.assertEqual(len(self.client._command_queue), 0)


if __name__ == '__main__':
    unittest.main()
