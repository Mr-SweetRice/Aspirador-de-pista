import {
  CommandClass,
  ControlMode,
  type DecodedTelemetryPacket,
  LineTrackType,
  type MapChunk,
  type MapPoint,
  type MapRecordChunk,
  OdometrySource,
  RaceSegmentType,
  type RacePlanSegment,
  ReadId,
  RgbLedMode,
  type RobotMap,
  type RobotState,
  SaveId,
  SendId,
  TelemetryId,
  type TelemetryRecord,
} from './types';

export const DEVICE_NAME = 'AspiradorPista-S3';
export const AUTH_TOKEN = 'engineering-token';
export const SERVICE_UUID = '5d7a0000-8f5a-4a7d-9d4f-7a6c2b8d0001';
export const AUTH_UUID = '5d7a0001-8f5a-4a7d-9d4f-7a6c2b8d0001';
export const COMMAND_UUID = '5d7a0002-8f5a-4a7d-9d4f-7a6c2b8d0001';
export const TELEMETRY_UUID = '5d7a0003-8f5a-4a7d-9d4f-7a6c2b8d0001';
export const PROTOCOL_VERSION = 1;
export const RACE_PLAN_MAX_SEGMENTS = 64;
export const RACE_PLAN_CHUNK_MAX_SEGMENTS = 10;

class Writer {
  private bytes: number[] = [];

  u8(value: number) { this.bytes.push(value & 0xff); return this; }
  i8(value: number) { return this.u8(value); }
  u16(value: number) { this.bytes.push(value & 0xff, (value >>> 8) & 0xff); return this; }
  u32(value: number) {
    this.bytes.push(value & 0xff, (value >>> 8) & 0xff, (value >>> 16) & 0xff, (value >>> 24) & 0xff);
    return this;
  }
  f32(value: number) {
    const data = new Uint8Array(4);
    new DataView(data.buffer).setFloat32(0, value, true);
    this.raw(data);
    return this;
  }
  raw(value: Uint8Array | number[]) { this.bytes.push(...value); return this; }
  result() { return Uint8Array.from(this.bytes); }
}

class Reader {
  offset = 0;
  private readonly view: DataView;

  constructor(readonly bytes: Uint8Array) {
    this.view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  }

  ensure(size: number) {
    if (this.offset + size > this.bytes.length) throw new Error('payload truncado');
  }
  u8() { this.ensure(1); return this.view.getUint8(this.offset++); }
  i8() { this.ensure(1); return this.view.getInt8(this.offset++); }
  u16() { this.ensure(2); const v = this.view.getUint16(this.offset, true); this.offset += 2; return v; }
  i32() { this.ensure(4); const v = this.view.getInt32(this.offset, true); this.offset += 4; return v; }
  f32() { this.ensure(4); const v = this.view.getFloat32(this.offset, true); this.offset += 4; return v; }
  raw(size: number) { this.ensure(size); const v = this.bytes.slice(this.offset, this.offset + size); this.offset += size; return v; }
}

const clamp = (value: number, min: number, max: number) => Math.max(min, Math.min(max, value));
const boolByte = (value: boolean) => value ? 1 : 0;

export function packPacket(commandClass: CommandClass, messageId: number, payload = new Uint8Array()): Uint8Array {
  if (payload.length > 255) throw new Error('payload maior que 255 bytes');
  return new Writer().u8(PROTOCOL_VERSION).u8(commandClass).u8(messageId).u8(payload.length).raw(payload).result();
}

export function unpackPacket(data: Uint8Array) {
  if (data.length < 4) throw new Error('pacote curto');
  if (data[0] !== PROTOCOL_VERSION) throw new Error(`versao invalida: ${data[0]}`);
  const payloadLength = data[3] ?? 0;
  if (data.length !== 4 + payloadLength) throw new Error('tamanho de payload invalido');
  return {
    commandClass: data[1] as CommandClass,
    messageId: data[2] ?? 0,
    payload: data.slice(4),
  };
}

export const packEmergencyStop = () => packPacket(CommandClass.Send, SendId.Stop);
export const packControlStop = () => packPacket(CommandClass.Send, SendId.ControlStop);
export const packResetEncoders = () => packPacket(CommandClass.Send, SendId.ResetEncoders);
export const packResetYaw = () => packPacket(CommandClass.Send, SendId.ResetYaw);
export const packSetLeftPwm = (value: number) => packPacket(CommandClass.Send, SendId.SetLeftPwm, new Writer().i8(clamp(Math.round(value), -100, 100)).result());
export const packSetRightPwm = (value: number) => packPacket(CommandClass.Send, SendId.SetRightPwm, new Writer().i8(clamp(Math.round(value), -100, 100)).result());
export const packSetAuxPwm = (value: number) => packPacket(CommandClass.Send, SendId.SetAuxPwm, new Writer().i8(clamp(Math.round(value), -100, 100)).result());
export const packZeroBrakeEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.SetZeroBrakeEnabled, new Writer().u8(boolByte(enabled)).result());

export const packControlStartMap = (slot: number, speedPercent: number) =>
  packPacket(CommandClass.Send, SendId.ControlStartMap, new Writer().u8(slot).i8(clamp(Math.round(speedPercent), -100, 100)).result());
export const packControlStartLine = (speedPercent: number) =>
  packPacket(CommandClass.Send, SendId.ControlStartLine, new Writer().i8(clamp(Math.round(speedPercent), -100, 100)).result());
export const packControlStartAutoTrack = (slot: number, speedPercent: number, useRacePlan = false) =>
  packPacket(CommandClass.Send, SendId.ControlStartAutoTrack, new Writer().u8(slot).i8(clamp(Math.round(speedPercent), -100, 100)).u8(boolByte(useRacePlan)).result());

export function packControlPid(kp: number, ki: number, kd: number, motorLimitPercent: number, alpha = 0.7) {
  const payload = new Writer()
    .f32(clamp(kp, 0, 1000)).f32(clamp(ki, 0, 1000)).f32(clamp(kd, 0, 1000))
    .u8(clamp(Math.round(motorLimitPercent), 0, 100)).f32(clamp(alpha, 0, 1)).result();
  return packPacket(CommandClass.Send, SendId.ControlSetPid, payload);
}

export function packControlSavePid(kp: number, ki: number, kd: number, motorLimitPercent: number, auxPercent: number, alpha = 0.7) {
  const payload = new Writer()
    .f32(clamp(kp, 0, 1000)).f32(clamp(ki, 0, 1000)).f32(clamp(kd, 0, 1000))
    .u8(clamp(Math.round(motorLimitPercent), 0, 100)).u8(clamp(Math.round(auxPercent), 0, 100))
    .f32(clamp(alpha, 0, 1)).result();
  return packPacket(CommandClass.Send, SendId.ControlSavePid, payload);
}

export const packControlAuxPercent = (value: number) => packPacket(CommandClass.Send, SendId.ControlSetAuxPercent, new Writer().u8(clamp(Math.round(value), 0, 100)).result());
export const packControlBatteryCompensationEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.ControlSetBatteryCompensationEnabled, new Writer().u8(boolByte(enabled)).result());
export const packControlAutoTrackConfig = (lineLossOdometryEnabled: boolean) => packPacket(
  CommandClass.Send,
  SendId.ControlSetAutoTrackConfig,
  new Writer().u8(0).u8(0).f32(Math.PI / 2).u8(boolByte(lineLossOdometryEnabled)).result(),
);
export const packOdometrySource = (_source: OdometrySource = OdometrySource.Fused) => packPacket(CommandClass.Send, SendId.OdometrySetSource, new Writer().u8(OdometrySource.Fused).result());
export const packOdometryPosition = (xM: number, yM: number) => packPacket(CommandClass.Send, SendId.OdometrySetPosition, new Writer().f32(xM).f32(yM).result());

export function packControlRacePlan(segments: RacePlanSegment[], enabled = true): Uint8Array[] {
  if (!enabled) return [packPacket(CommandClass.Send, SendId.ControlSetRacePlan, new Writer().u8(0).u8(0).u8(0).u8(0).result())];
  if (!segments.length || segments.length > RACE_PLAN_MAX_SEGMENTS) throw new Error('quantidade de segmentos invalida');
  let previousEnd = -1;
  const clean = segments.map((segment) => {
    const start = clamp(Math.round(segment.startIndex), 0, 2047);
    const end = clamp(Math.round(segment.endIndex), 0, 2047);
    if (start > end || start <= previousEnd) throw new Error('segmentos fora de ordem ou sobrepostos');
    previousEnd = end;
    return { ...segment, startIndex: start, endIndex: end };
  });
  const packets: Uint8Array[] = [];
  for (let offset = 0; offset < clean.length; offset += RACE_PLAN_CHUNK_MAX_SEGMENTS) {
    const chunk = clean.slice(offset, offset + RACE_PLAN_CHUNK_MAX_SEGMENTS);
    const writer = new Writer().u8(1).u8(clean.length).u8(offset).u8(chunk.length);
    for (const segment of chunk) {
      const isStop = segment.type === RaceSegmentType.Stop;
      writer.u16(segment.startIndex).u16(segment.endIndex).u8(segment.type)
        .u8(isStop ? 0 : clamp(Math.round(segment.speedPercent), 0, 100))
        .u8(isStop ? 0 : clamp(Math.round(segment.maxSpeedPercent ?? 100), 0, 100))
        .u8(clamp(Math.round(segment.auxPercent), 0, 100))
        .f32(clamp(segment.kp, 0, 1000)).f32(clamp(segment.ki, 0, 1000)).f32(clamp(segment.kd, 0, 1000));
    }
    packets.push(packPacket(CommandClass.Send, SendId.ControlSetRacePlan, writer.result()));
  }
  return packets;
}

const encodeName = (name: string) => {
  const raw = unescape(encodeURIComponent(name.trim())).split('').map((char) => char.charCodeAt(0)).slice(0, 15);
  return Uint8Array.from([...raw, ...new Array(16 - raw.length).fill(0)]);
};
const decodeName = (raw: Uint8Array) => {
  const zero = raw.indexOf(0);
  const bytes = raw.slice(0, zero < 0 ? raw.length : zero);
  const binary = Array.from(bytes, (value) => String.fromCharCode(value)).join('');
  try { return decodeURIComponent(escape(binary)); } catch { return binary; }
};

export const packReadMapList = () => packPacket(CommandClass.Read, ReadId.MapList);
export const packReadMapChunk = (slot: number, offset = 0) => packPacket(CommandClass.Read, ReadId.MapChunk, new Writer().u8(slot).u16(offset).result());
export const packReadMapRecordChunk = (offset = 0) => packPacket(CommandClass.Read, ReadId.MapRecordChunk, new Writer().u16(offset).result());
export const packMapRecordStart = (name: string) => packPacket(CommandClass.Send, SendId.MapRecordStart, encodeName(name));
export const packMapRecordStop = () => packPacket(CommandClass.Send, SendId.MapRecordStop);
export const packMapRecordSave = (name: string) => packPacket(CommandClass.Send, SendId.MapRecordSave, encodeName(name));
export const packMapDelete = (slot: number) => packPacket(CommandClass.Send, SendId.MapDelete, new Writer().u8(slot).result());

export function packSaveMapChunk(name: string, totalPoints: number, offset: number, points: MapPoint[]) {
  const selected = points.slice(0, 12);
  const writer = new Writer().raw(encodeName(name)).u16(totalPoints).u16(offset).u8(selected.length);
  selected.forEach((point) => writer.f32(point.x).f32(point.y));
  return packPacket(CommandClass.Save, SaveId.MapChunk, writer.result());
}

const durationPacket = (id: SendId, seconds: number, minMs: number, maxMs: number) => packPacket(CommandClass.Send, id, new Writer().u32(clamp(Math.round(seconds * 1000), minMs, maxMs)).result());
export const packImuCalibrateMag = (seconds = 30) => durationPacket(SendId.ImuCalibrateMag, seconds, 5000, 120000);
export const packImuCalibrateAccelGyro = (seconds = 5) => durationPacket(SendId.ImuCalibrateAccelGyro, seconds, 1000, 30000);
export const packImuCalibrateYawDrift = (seconds = 20) => durationPacket(SendId.ImuCalibrateYawDrift, seconds, 1000, 120000);
export const packImuCalibrateAll = (seconds = 45) => durationPacket(SendId.ImuCalibrateAll, seconds, 15000, 180000);
export const packImuMagFilterGain = (gain: number) => packPacket(CommandClass.Send, SendId.ImuSetMagFilterGain, new Writer().f32(clamp(gain, 0, 0.2)).result());
export const packImuMagHeadingMode = (mode: number) => packPacket(CommandClass.Send, SendId.ImuSetMagHeadingMode, new Writer().u8(clamp(Math.round(mode), 0, 2)).result());
export const packImuMagIgnored = (ignored: boolean) => packPacket(CommandClass.Send, SendId.ImuSetMagIgnored, new Writer().u8(boolByte(ignored)).result());
export const packImuYawDriftThreshold = (thresholdDps: number) => packPacket(CommandClass.Send, SendId.ImuSetYawDriftThreshold, new Writer().f32(clamp(thresholdDps, 0, 5)).result());

export const packLineCalibrate = (seconds = 5) => durationPacket(SendId.LineCalibrate, seconds, 1000, 120000);
export const packLineTrackType = (type: LineTrackType) => packPacket(CommandClass.Send, SendId.LineSetTrackType, new Writer().u8(type === LineTrackType.White ? 1 : 0).result());
export const packLineThreshold = (percent: number) => packPacket(CommandClass.Send, SendId.LineSetThreshold, new Writer().u8(clamp(Math.round(percent), 0, 45)).result());
export const packLineFilter = (percent: number) => packPacket(CommandClass.Send, SendId.LineSetFilter, new Writer().u8(clamp(Math.round(percent), 0, 100)).result());

export const packRgbLedEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.RgbLedSetEnabled, new Writer().u8(boolByte(enabled)).result());
export const packRgbLedMode = (mode: RgbLedMode) => packPacket(CommandClass.Send, SendId.RgbLedSetMode, new Writer().u8(clamp(mode, 0, 3)).result());
export const packRgbLedManual = (red: number, green: number, blue: number, intensity: number) => packPacket(CommandClass.Send, SendId.RgbLedSetManual, new Writer().u8(clamp(red, 0, 255)).u8(clamp(green, 0, 255)).u8(clamp(blue, 0, 255)).u8(clamp(intensity, 0, 255)).result());

export const packSafetyCollisionEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.SafetySetCollisionEnabled, new Writer().u8(boolByte(enabled)).result());
export const packSafetyBatteryBlockEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.SafetySetBatteryBlockEnabled, new Writer().u8(boolByte(enabled)).result());
export const packSafetyLineLossEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.SafetySetLineLossEnabled, new Writer().u8(boolByte(enabled)).result());
export const packSafetyBleLossEnabled = (enabled: boolean) => packPacket(CommandClass.Send, SendId.SafetySetBleLossEnabled, new Writer().u8(boolByte(enabled)).result());
export const packSafetyRollLimit = (degrees: number) => packPacket(CommandClass.Send, SendId.SafetySetRollLimit, new Writer().f32(clamp(degrees, 1, 90)).result());
export const packSafetyBatteryBlockPercent = (percent: number) => packPacket(CommandClass.Send, SendId.SafetySetBatteryBlockPercent, new Writer().f32(clamp(percent, 0, 100)).result());
export const packSafetyLineLossTimeout = (seconds: number) => packPacket(CommandClass.Send, SendId.SafetySetLineLossTimeout, new Writer().f32(clamp(seconds, 0.1, 10)).result());

function parseMapList(payload: Uint8Array): RobotMap[] {
  const reader = new Reader(payload);
  const count = reader.u8();
  const entrySize = count ? (payload.length - 1) / count : 23;
  if (entrySize !== 23 && entrySize !== 19) throw new Error('lista de mapas invalida');
  const maps: RobotMap[] = [];
  for (let index = 0; index < count; index++) {
    const slot = reader.u8();
    const pointCount = reader.u16();
    const distanceM = entrySize === 23 ? reader.f32() : 0;
    maps.push({ slot, pointCount, distanceM, name: decodeName(reader.raw(16)) });
  }
  return maps;
}

function parseMapChunk(payload: Uint8Array): MapChunk {
  const reader = new Reader(payload);
  const slot = reader.u8();
  const totalPoints = reader.u16();
  const offset = reader.u16();
  const pointCount = reader.u8();
  const points = Array.from({ length: pointCount }, () => ({ x: reader.f32(), y: reader.f32() }));
  if (reader.offset !== payload.length) throw new Error('chunk de mapa invalido');
  return { slot, totalPoints, offset, pointCount, points };
}

function parseMapRecordChunk(payload: Uint8Array): MapRecordChunk {
  const reader = new Reader(payload);
  const totalPoints = reader.u16();
  const offset = reader.u16();
  const pointCount = reader.u8();
  const active = Boolean(reader.u8());
  const rejectedPoints = reader.u16();
  const distanceM = reader.f32();
  const points = Array.from({ length: pointCount }, () => ({ x: reader.f32(), y: reader.f32() }));
  if (reader.offset !== payload.length) throw new Error('chunk de gravacao invalido');
  return { totalPoints, offset, pointCount, active, rejectedPoints, distanceM, points };
}

function parseControl(payload: Uint8Array): Partial<RobotState> {
  if (payload.length !== 103 && payload.length !== 147) throw new Error(`controle com ${payload.length} bytes`);
  const r = new Reader(payload);
  const controlRunning = Boolean(r.u8());
  const controlMapSlot = r.u8();
  const controlTargetIndex = r.u16();
  const controlPointCount = r.u16();
  const controlSpeedPercent = r.i8();
  const controlTargetXM = r.f32(); const controlTargetYM = r.f32(); const controlDistanceM = r.f32();
  const controlAngleErrorRad = r.f32(); const controlSteerPercent = r.f32();
  const controlKp = r.f32(); const controlKi = r.f32(); const controlKd = r.f32();
  const controlMotorLimitPercent = r.u8();
  const controlLoopHz = r.f32(); const racePlanLoopHz = r.f32(); const trackOdometryLoopHz = r.f32();
  const lineSensorLoopHz = r.f32(); const imuLoopHz = r.f32();
  const controlMode = r.u8() as ControlMode;
  const controlSpeedProfileEnabled = Boolean(r.u8());
  const controlAuxPercent = r.u8(); const controlActiveAuxPercent = r.u8();
  const controlAverageSpeedMps = r.f32(); const controlMaxSpeedMps = r.f32();
  const controlBatteryCompensationEnabled = Boolean(r.u8());
  const controlMapXM = r.f32(); const controlMapYM = r.f32(); const controlMapHeadingRad = r.f32();
  const controlRaceSegmentActive = Boolean(r.u8());
  const controlRaceSegmentType = r.u8() as RaceSegmentType;
  const controlRaceSegmentStartIndex = r.u16(); const controlRaceSegmentEndIndex = r.u16();
  const controlRaceSegmentSpeedPercent = r.u8(); const controlRaceSegmentMaxSpeedPercent = r.u8();
  const controlRaceSegmentAuxPercent = r.u8(); const controlActiveSpeedPercent = r.u8();
  const controlRacePlanAverageSpeedMps = r.f32();
  const controlLineAlpha = payload.length === 103 ? r.f32() : 0.7;
  return {
    controlRunning, controlMapSlot, controlTargetIndex, controlPointCount, controlSpeedPercent,
    controlActiveSpeedPercent, controlTargetXM, controlTargetYM, controlDistanceM, controlAngleErrorRad,
    controlSteerPercent, controlKp, controlKi, controlKd, controlLineAlpha, controlMotorLimitPercent,
    controlLoopHz, racePlanLoopHz, trackOdometryLoopHz, lineSensorLoopHz, imuLoopHz, controlMode,
    controlSpeedProfileEnabled, controlAuxPercent, controlActiveAuxPercent, controlAverageSpeedMps,
    controlMaxSpeedMps, controlRacePlanAverageSpeedMps, controlBatteryCompensationEnabled,
    controlMapPoseValid: true, controlMapXM, controlMapYM, controlMapHeadingRad, controlRaceSegmentActive,
    controlRaceSegmentType, controlRaceSegmentStartIndex, controlRaceSegmentEndIndex,
    controlRaceSegmentSpeedPercent, controlRaceSegmentMaxSpeedPercent, controlRaceSegmentAuxPercent,
  };
}

function parseRecord(messageId: number, payload: Uint8Array): TelemetryRecord {
  const record: TelemetryRecord = { messageId, payload };
  const r = new Reader(payload);
  switch (messageId) {
    case TelemetryId.Status:
      record.status = payload[0];
      record.patch = { lastStatus: payload[0] ?? null };
      break;
    case TelemetryId.Encoders:
      if (payload.length !== 20 && payload.length !== 16) throw new Error('encoder invalido');
      record.patch = { leftEncoder: r.i32(), rightEncoder: r.i32(), leftRpm: r.f32(), rightRpm: r.f32(), linearMps: payload.length === 20 ? r.f32() : 0 };
      break;
    case TelemetryId.Battery:
      if (payload.length !== 10 && payload.length !== 8) throw new Error('bateria invalida');
      record.patch = { batteryV: r.f32(), batteryPercent: r.f32(), batteryRaw: payload.length === 10 ? r.u16() : 0 };
      break;
    case TelemetryId.Imu:
    case TelemetryId.ImuFast:
      if (payload.length < 12) throw new Error('IMU invalida');
      record.patch = { roll: r.f32(), pitch: r.f32(), yaw: r.f32() };
      break;
    case TelemetryId.Pose: {
      if (payload.length !== 12) throw new Error('pose invalida');
      const x = r.f32(); const y = r.f32(); const heading = r.f32();
      record.patch = { xM: x, yM: y, headingRad: heading, fusedXM: x, fusedYM: y, fusedHeadingRad: heading };
      break;
    }
    case TelemetryId.Odometry: {
      if (payload.length !== 57 && payload.length !== 56 && payload.length !== 20) throw new Error('odometria invalida');
      const xM = r.f32(); const yM = r.f32(); const headingRad = r.f32(); const linearMps = r.f32(); const angularRadS = r.f32();
      if (payload.length === 20) {
        record.patch = { xM, yM, headingRad, linearMps, angularRadS, encoderXM: xM, encoderYM: yM, encoderHeadingRad: headingRad, imuXM: xM, imuYM: yM, imuHeadingRad: headingRad, fusedXM: xM, fusedYM: yM, fusedHeadingRad: headingRad, odometryImuAvailable: true };
      } else {
        record.patch = { xM, yM, headingRad, linearMps, angularRadS, encoderXM: r.f32(), encoderYM: r.f32(), encoderHeadingRad: r.f32(), imuXM: r.f32(), imuYM: r.f32(), imuHeadingRad: r.f32(), fusedXM: r.f32(), fusedYM: r.f32(), fusedHeadingRad: r.f32(), odometryImuAvailable: payload.length === 57 ? Boolean(r.u8()) : true };
      }
      break;
    }
    case TelemetryId.Control:
      record.patch = parseControl(payload);
      break;
    case TelemetryId.Line: {
      if (![53, 57, 58].includes(payload.length)) throw new Error('linha invalida');
      const lineRaw = Array.from({ length: 8 }, () => r.u16());
      const lineCalibrated = Array.from({ length: 8 }, () => r.u16());
      const lineValues = Array.from({ length: 8 }, () => r.u16());
      const linePosition = r.u16(); const lineTrackType = r.u8() as LineTrackType; const flags = r.u8();
      const lineThresholdPercent = r.u8(); const lineReadHz = payload.length >= 57 ? r.f32() : 0; const lineFilterPercent = payload.length === 58 ? r.u8() : 100;
      record.patch = { lineRaw, lineCalibrated, lineValues, linePosition, lineTrackType, lineVisible: Boolean(flags & 1), lineCalibratedValid: Boolean(flags & 2), lineCalibrating: Boolean(flags & 4), lineThresholdPercent, lineFilterPercent, lineReadHz };
      break;
    }
    case TelemetryId.LineFast: {
      if (payload.length !== 9 && payload.length !== 10) throw new Error('linha rapida invalida');
      const linePosition = r.u16(); const lineTrackType = r.u8() as LineTrackType; const flags = r.u8();
      const lineThresholdPercent = r.u8(); const lineReadHz = r.f32(); const lineFilterPercent = payload.length === 10 ? r.u8() : 100;
      record.patch = { linePosition, lineTrackType, lineVisible: Boolean(flags & 1), lineCalibratedValid: Boolean(flags & 2), lineCalibrating: Boolean(flags & 4), lineThresholdPercent, lineFilterPercent, lineReadHz };
      break;
    }
    case TelemetryId.RgbLed:
      if (payload.length !== 6) throw new Error('LED invalido');
      record.patch = { rgbLedMode: r.u8() as RgbLedMode, rgbLedRed: r.u8(), rgbLedGreen: r.u8(), rgbLedBlue: r.u8(), rgbLedIntensity: r.u8(), rgbLedEnabled: Boolean(r.u8()) };
      break;
    case TelemetryId.Safety: {
      if (payload.length !== 26 && payload.length !== 25 && payload.length !== 17) throw new Error('seguranca invalida');
      const flags = payload.length === 25 || payload.length === 17 ? r.u8() : r.u16();
      const safetyRollLimitDeg = r.f32(); const safetyBatteryBlockPercent = r.f32();
      const safetyCurrentRollDeg = r.f32(); const safetyCurrentBatteryPercent = r.f32();
      const safetyLineLossTimeoutS = payload.length === 17 ? 1 : r.f32(); const safetyLineLossElapsedS = payload.length === 17 ? 0 : r.f32();
      record.patch = {
        safetyCollisionEnabled: Boolean(flags & (1 << 0)), safetyBatteryBlockEnabled: Boolean(flags & (1 << 1)),
        safetyCollisionActive: Boolean(flags & (1 << 2)), safetyBatteryBlockActive: Boolean(flags & (1 << 3)),
        safetyMotorsBlocked: Boolean(flags & (1 << 4)), safetyLineLossEnabled: Boolean(flags & (1 << 5)),
        safetyLineLossActive: Boolean(flags & (1 << 6)), safetyLineVisible: Boolean(flags & (1 << 7)),
        safetyBleLossEnabled: Boolean(flags & (1 << 8)), safetyBleLossActive: Boolean(flags & (1 << 9)),
        safetyBleConnected: Boolean(flags & (1 << 10)), safetyRollLimitDeg, safetyBatteryBlockPercent,
        safetyCurrentRollDeg, safetyCurrentBatteryPercent, safetyLineLossTimeoutS, safetyLineLossElapsedS,
      };
      break;
    }
    case TelemetryId.System:
      if (payload.length !== 9 && payload.length !== 8) throw new Error('sistema invalido');
      record.patch = { cpu0Percent: r.f32(), cpu1Percent: r.f32(), zeroBrakeEnabled: payload.length === 9 ? Boolean(r.u8() & 1) : true };
      break;
    case TelemetryId.MapList:
      record.maps = parseMapList(payload);
      break;
    case TelemetryId.MapChunk:
      record.mapChunk = parseMapChunk(payload);
      break;
    case TelemetryId.MapRecordChunk:
      record.mapRecordChunk = parseMapRecordChunk(payload);
      break;
    default:
      break;
  }
  return record;
}

export function decodeTelemetryPacket(data: Uint8Array): DecodedTelemetryPacket {
  const packet = unpackPacket(data);
  if (packet.commandClass !== CommandClass.Telemetry) throw new Error('pacote nao e telemetria');
  const rawRecords: { messageId: number; payload: Uint8Array }[] = [];
  if (packet.messageId === TelemetryId.Bundle) {
    const r = new Reader(packet.payload);
    while (r.offset < packet.payload.length) {
      const messageId = r.u8(); const length = r.u8();
      rawRecords.push({ messageId, payload: r.raw(length) });
    }
  } else {
    rawRecords.push({ messageId: packet.messageId, payload: packet.payload });
  }
  const records = rawRecords.map(({ messageId, payload }) => parseRecord(messageId, payload));
  const result: DecodedTelemetryPacket = { packet, records, patches: records.flatMap((record) => record.patch ? [record.patch] : []) };
  for (const record of records) {
    if (record.maps) result.maps = record.maps;
    if (record.mapChunk) result.mapChunk = record.mapChunk;
    if (record.mapRecordChunk) result.mapRecordChunk = record.mapRecordChunk;
    if (record.status !== undefined) result.status = record.status;
  }
  return result;
}
