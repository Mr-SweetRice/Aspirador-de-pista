from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ble.protocol import CommandClass, SendId, TelemetryId, pack_telemetry_enabled, unpack_packet


class TelemetryConfigProtocolTests(unittest.TestCase):
    def test_enable_packet(self):
        command_class, message_id, payload = unpack_packet(
            pack_telemetry_enabled(TelemetryId.ODOMETRY, True)
        )
        self.assertEqual(command_class, CommandClass.SEND)
        self.assertEqual(message_id, SendId.TELEMETRY_SET_ENABLED)
        self.assertEqual(payload, bytes([TelemetryId.ODOMETRY, 1]))

    def test_disable_packet(self):
        packet = pack_telemetry_enabled(TelemetryId.IMU_FAST, False)
        self.assertEqual(packet, bytes([1, 3, 0x81, 2, TelemetryId.IMU_FAST, 0]))

    def test_essential_response_cannot_be_disabled(self):
        with self.assertRaises(ValueError):
            pack_telemetry_enabled(TelemetryId.STATUS, False)
        with self.assertRaises(ValueError):
            pack_telemetry_enabled(TelemetryId.MAP_CHUNK, False)


if __name__ == "__main__":
    unittest.main()
