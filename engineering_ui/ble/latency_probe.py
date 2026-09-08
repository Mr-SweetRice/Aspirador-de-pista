"""Read-only hardware latency probe; does not start motors or reset odometry."""
import argparse
import asyncio
import json
import time
from pathlib import Path

from bleak import BleakClient
from ble.protocol import (AUTH_UUID, AUTH_TOKEN, COMMAND_UUID, TELEMETRY_UUID,
                          TelemetryId, pack_read_map_list, pack_read_map_chunk,
                          unpack_packet, unpack_telemetry_bundle, unpack_map_list,
                          unpack_control_telemetry)


async def probe(address, output, samples):
    result = {'samples': [], 'counts': {}, 'control': [], 'errors': []}
    events, payloads = {}, {}

    def receive(_, data):
        _, mid, payload = unpack_packet(bytes(data))
        records = unpack_telemetry_bundle(payload) if mid == TelemetryId.BUNDLE else [(mid, payload)]
        for mid, payload in records:
            key = str(int(mid))
            result['counts'][key] = result['counts'].get(key, 0) + 1
            payloads[mid] = payload
            if mid == TelemetryId.CONTROL:
                result['control'].append(unpack_control_telemetry(payload))
            if mid in events:
                events[mid].set()

    try:
        async with BleakClient(address, timeout=12) as client:
            await client.write_gatt_char(AUTH_UUID, AUTH_TOKEN.encode(), response=True)
            result['mtu'] = client.mtu_size

            async def measure(packet, mid=None, phase='telemetry'):
                if mid is not None:
                    events[mid] = asyncio.Event()
                row = {'phase': phase, 'class': packet[1], 'id': packet[2]}
                result['samples'].append(row)
                start = time.perf_counter()
                try:
                    await asyncio.wait_for(client.write_gatt_char(COMMAND_UUID, packet, response=True), 3)
                    row['att_ms'] = round((time.perf_counter() - start) * 1000, 2)
                    if mid is not None:
                        await asyncio.wait_for(events[mid].wait(), 3)
                        row['response_ms'] = round((time.perf_counter() - start) * 1000, 2)
                except Exception as error:
                    row['error'] = type(error).__name__ + ': ' + str(error)
                    raise
                finally:
                    print(json.dumps(row), flush=True)

            for _ in range(5):
                await measure(pack_read_map_list(), phase='no_notifications')
                await asyncio.sleep(.1)
            await client.start_notify(TELEMETRY_UUID, receive)
            await measure(pack_read_map_list(), TelemetryId.MAP_LIST)
            maps = unpack_map_list(payloads[TelemetryId.MAP_LIST])
            result['maps'] = maps
            if maps:
                for _ in range(samples):
                    await measure(pack_read_map_chunk(maps[0]['slot'], 0), TelemetryId.MAP_CHUNK)
                    await asyncio.sleep(.05)
            await asyncio.sleep(2)
    except Exception as error:
        result['errors'].append(type(error).__name__ + ': ' + str(error))
    finally:
        Path(output).write_text(json.dumps(result, indent=2), encoding='utf-8')
        print('Report:', output, 'errors:', result['errors'], flush=True)
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--address', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--samples', type=int, default=20)
    args = parser.parse_args()
    asyncio.run(probe(args.address, args.output, args.samples))
