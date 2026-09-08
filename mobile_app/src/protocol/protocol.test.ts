import assert from 'node:assert/strict';
import test from 'node:test';

import {
  decodeTelemetryPacket,
  packControlPid,
  packControlRacePlan,
  packControlStartAutoTrack,
  packEmergencyStop,
  packRgbLedManual,
} from './protocol';
import { RaceSegmentType } from './types';

const bytes = (hex: string) => Uint8Array.from(hex.match(/../g)!.map((value) => Number.parseInt(value, 16)));
const hex = (value: Uint8Array) => Array.from(value, (byte) => byte.toString(16).padStart(2, '0')).join('');

test('empacota comandos exatamente como o protocolo Python', () => {
  assert.equal(hex(packEmergencyStop()), '01030100');
  assert.equal(hex(packControlStartAutoTrack(2, 70, true)), '01034b03024601');
  assert.equal(hex(packControlPid(1.25, 0.5, 0.125, 80, 0.75)), '010342110000a03f0000003f0000003e500000403f');
  assert.equal(hex(packRgbLedManual(255, 54, 0, 179)), '01036204ff3600b3');
});

test('empacota plano de corrida completo', () => {
  const packets = packControlRacePlan([
    { startIndex: 0, endIndex: 9, type: RaceSegmentType.Straight, speedPercent: 60, maxSpeedPercent: 80, auxPercent: 70, kp: 1.25, ki: 0.5, kd: 0.125 },
    { startIndex: 10, endIndex: 19, type: RaceSegmentType.Curve, speedPercent: 40, maxSpeedPercent: 60, auxPercent: 75, kp: 2, ki: 0.25, kd: 0.5 },
    { startIndex: 20, endIndex: 24, type: RaceSegmentType.Intersection, speedPercent: 30, maxSpeedPercent: 50, auxPercent: 80, kp: 3, ki: 0.125, kd: 0.25 },
    { startIndex: 25, endIndex: 25, type: RaceSegmentType.Stop, speedPercent: 0, maxSpeedPercent: 0, auxPercent: 0, kp: 0, ki: 0, kd: 0 },
  ]);
  assert.equal(packets.length, 1);
  assert.equal(hex(packets[0]!), '01034d540104000400000900003c50460000a03f0000003f0000003e0a00130001283c4b000000400000803e0000003f14001800031e3250000040400000003e0000803e1900190002000000000000000000000000000000');
});

test('decodifica bundle rápido do firmware', () => {
  const decoded = decodeTelemetryPacket(bytes('01040828090c0000c03f000020c00000403f0b0c000020410000a0c00000b4420a0aac0d0103190000fa4346'));
  assert.equal(decoded.records.length, 3);
  const merged = Object.assign({}, ...decoded.patches);
  assert.equal(merged.fusedXM, 1.5);
  assert.equal(merged.fusedYM, -2.5);
  assert.equal(merged.roll, 10);
  assert.equal(merged.pitch, -5);
  assert.equal(merged.yaw, 90);
  assert.equal(merged.linePosition, 3500);
  assert.equal(merged.lineReadHz, 500);
  assert.equal(merged.lineFilterPercent, 70);
});

test('decodifica odometria completa, bateria e segurança', () => {
  const odometry = Object.assign({}, ...decodeTelemetryPacket(bytes('010405390000803f000000c00000003f0000403f000080be0000903f000008c00000c03e0000a03f000010c00000203f0000c03f000020c00000403f01')).patches);
  assert.equal(odometry.encoderXM, 1.125);
  assert.equal(odometry.imuYM, -2.25);
  assert.equal(odometry.fusedHeadingRad, 0.75);
  assert.equal(odometry.odometryImuAvailable, true);

  const battery = decodeTelemetryPacket(bytes('0104060a00004841000096420008')).patches[0]!;
  assert.equal(battery.batteryV, 12.5);
  assert.equal(battery.batteryPercent, 75);
  assert.equal(battery.batteryRaw, 2048);

  const safety = decodeTelemetryPacket(bytes('0104321aa3050000344200007041000048c1000091420000c03f0000803e')).patches[0]!;
  assert.equal(safety.safetyCollisionEnabled, true);
  assert.equal(safety.safetyBleConnected, true);
  assert.equal(safety.safetyCurrentRollDeg, -12.5);
  assert.equal(safety.safetyLineLossElapsedS, 0.25);
});
