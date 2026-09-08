export enum CommandClass {
  Save = 0x01,
  Read = 0x02,
  Send = 0x03,
  Telemetry = 0x04,
}

export enum ErrorCode {
  Ok = 0x00,
  AuthRequired = 0x01,
  AuthDenied = 0x02,
  InvalidPacket = 0x03,
  Unsupported = 0x04,
  Internal = 0x05,
}

export enum SaveId {
  MapChunk = 0x01,
}

export enum ReadId {
  MapList = 0x01,
  MapChunk = 0x02,
  MapRecordChunk = 0x03,
}

export enum SendId {
  Stop = 0x01,
  MoveForward = 0x02,
  MoveBackward = 0x03,
  SetLeftPwm = 0x10,
  SetRightPwm = 0x11,
  SetAuxPwm = 0x12,
  SetZeroBrakeEnabled = 0x13,
  ResetEncoders = 0x20,
  ResetYaw = 0x21,
  MapRecordStart = 0x22,
  MapRecordStop = 0x23,
  MapRecordSave = 0x24,
  MapDelete = 0x25,
  ImuCalibrateMag = 0x30,
  ImuCalibrateAll = 0x31,
  ImuCalibrateAccelGyro = 0x32,
  ImuSetMagFilterGain = 0x33,
  ImuSetMagHeadingMode = 0x34,
  ImuSetMagIgnored = 0x35,
  ImuSetYawDriftThreshold = 0x36,
  ImuCalibrateYawDrift = 0x37,
  ControlStartMap = 0x40,
  ControlStop = 0x41,
  ControlSetPid = 0x42,
  OdometrySetSource = 0x43,
  ControlStartLine = 0x44,
  ControlSetSpeedProfile = 0x45,
  ControlSavePid = 0x46,
  ControlSetSpeedProfileEnabled = 0x47,
  ControlSetAuxPercent = 0x48,
  OdometrySetPosition = 0x49,
  ControlSetBatteryCompensationEnabled = 0x4a,
  ControlStartAutoTrack = 0x4b,
  ControlSetAutoTrackConfig = 0x4c,
  ControlSetRacePlan = 0x4d,
  LineCalibrate = 0x50,
  LineSetTrackType = 0x51,
  LineSetThreshold = 0x52,
  LineSetFilter = 0x53,
  RgbLedSetEnabled = 0x60,
  RgbLedSetMode = 0x61,
  RgbLedSetManual = 0x62,
  SafetySetCollisionEnabled = 0x70,
  SafetySetBatteryBlockEnabled = 0x71,
  SafetySetRollLimit = 0x72,
  SafetySetBatteryBlockPercent = 0x73,
  SafetySetLineLossEnabled = 0x74,
  SafetySetLineLossTimeout = 0x75,
  SafetySetBleLossEnabled = 0x76,
}

export enum TelemetryId {
  Status = 0x01,
  Encoders = 0x02,
  Rpm = 0x03,
  Line = 0x04,
  Odometry = 0x05,
  Battery = 0x06,
  Imu = 0x07,
  Bundle = 0x08,
  Pose = 0x09,
  LineFast = 0x0a,
  ImuFast = 0x0b,
  MapList = 0x20,
  MapChunk = 0x21,
  MapRecordChunk = 0x22,
  Control = 0x30,
  RgbLed = 0x31,
  Safety = 0x32,
  System = 0x33,
}

export enum OdometrySource {
  Fused = 2,
}

export enum ControlMode {
  Odometry = 0,
  Line = 1,
  AutoTrack = 2,
}

export enum RaceSegmentType {
  Straight = 0,
  Curve = 1,
  Stop = 2,
  Intersection = 3,
}

export enum LineTrackType {
  Black = 0,
  White = 1,
}

export enum RgbLedMode {
  Disabled = 0,
  Battery = 1,
  Manual = 2,
  RacePlan = 3,
}

export interface ProtocolPacket {
  commandClass: CommandClass;
  messageId: number;
  payload: Uint8Array;
}

export interface MapPoint {
  x: number;
  y: number;
}

export interface RobotMap {
  slot: number;
  name: string;
  pointCount: number;
  distanceM: number;
  points?: MapPoint[];
}

export interface MapChunk {
  slot: number;
  totalPoints: number;
  offset: number;
  pointCount: number;
  points: MapPoint[];
}

export interface MapRecordChunk {
  totalPoints: number;
  offset: number;
  pointCount: number;
  active: boolean;
  rejectedPoints: number;
  distanceM: number;
  points: MapPoint[];
}

export interface RacePlanSegment {
  startIndex: number;
  endIndex: number;
  type: RaceSegmentType;
  speedPercent: number;
  maxSpeedPercent?: number;
  auxPercent: number;
  kp: number;
  ki: number;
  kd: number;
}

export interface SpeedProfilePoint {
  speedPercent: number;
  kp: number;
  ki: number;
  kd: number;
  auxPercent: number;
}

export interface RunPreset {
  id: string;
  name: string;
  mapSlot: number;
  speedPercent: number;
  useRacePlan: boolean;
  mode?: 'race' | 'auto' | 'line' | 'odometry';
  auxPercent?: number;
  autoResetPosition?: boolean;
  racePlan?: RacePlanSegment[];
  lineLossOdometryEnabled?: boolean;
  batteryCompensationEnabled?: boolean;
  updatedAt?: string;
}

export interface RobotState {
  connected: boolean;
  authenticated: boolean;
  lastError: string | null;
  lastStatus: number | null;

  leftEncoder: number;
  rightEncoder: number;
  leftRpm: number;
  rightRpm: number;
  linearMps: number;
  batteryV: number;
  batteryPercent: number;
  batteryRaw: number;
  cpu0Percent: number;
  cpu1Percent: number;
  zeroBrakeEnabled: boolean;

  roll: number;
  pitch: number;
  yaw: number;

  xM: number;
  yM: number;
  headingRad: number;
  angularRadS: number;
  encoderXM: number;
  encoderYM: number;
  encoderHeadingRad: number;
  imuXM: number;
  imuYM: number;
  imuHeadingRad: number;
  fusedXM: number;
  fusedYM: number;
  fusedHeadingRad: number;
  odometryImuAvailable: boolean;

  controlRunning: boolean;
  controlMode: ControlMode;
  controlMapSlot: number;
  controlTargetIndex: number;
  controlPointCount: number;
  controlSpeedPercent: number;
  controlActiveSpeedPercent: number;
  controlTargetXM: number;
  controlTargetYM: number;
  controlDistanceM: number;
  controlAngleErrorRad: number;
  controlSteerPercent: number;
  controlKp: number;
  controlKi: number;
  controlKd: number;
  controlLineAlpha: number;
  controlMotorLimitPercent: number;
  controlLoopHz: number;
  racePlanLoopHz: number;
  trackOdometryLoopHz: number;
  lineSensorLoopHz: number;
  imuLoopHz: number;
  controlSpeedProfileEnabled: boolean;
  controlAuxPercent: number;
  controlActiveAuxPercent: number;
  controlAverageSpeedMps: number;
  controlMaxSpeedMps: number;
  controlRacePlanAverageSpeedMps: number;
  controlBatteryCompensationEnabled: boolean;
  controlMapPoseValid: boolean;
  controlMapXM: number;
  controlMapYM: number;
  controlMapHeadingRad: number;
  controlRaceSegmentActive: boolean;
  controlRaceSegmentType: RaceSegmentType;
  controlRaceSegmentStartIndex: number;
  controlRaceSegmentEndIndex: number;
  controlRaceSegmentSpeedPercent: number;
  controlRaceSegmentMaxSpeedPercent: number;
  controlRaceSegmentAuxPercent: number;

  lineRaw: number[];
  lineCalibrated: number[];
  lineValues: number[];
  linePosition: number;
  lineTrackType: LineTrackType;
  lineVisible: boolean;
  lineCalibratedValid: boolean;
  lineCalibrating: boolean;
  lineThresholdPercent: number;
  lineFilterPercent: number;
  lineReadHz: number;

  rgbLedMode: RgbLedMode;
  rgbLedRed: number;
  rgbLedGreen: number;
  rgbLedBlue: number;
  rgbLedIntensity: number;
  rgbLedEnabled: boolean;

  safetyCollisionEnabled: boolean;
  safetyBatteryBlockEnabled: boolean;
  safetyLineLossEnabled: boolean;
  safetyBleLossEnabled: boolean;
  safetyCollisionActive: boolean;
  safetyBatteryBlockActive: boolean;
  safetyLineLossActive: boolean;
  safetyBleLossActive: boolean;
  safetyMotorsBlocked: boolean;
  safetyLineVisible: boolean;
  safetyBleConnected: boolean;
  safetyRollLimitDeg: number;
  safetyBatteryBlockPercent: number;
  safetyLineLossTimeoutS: number;
  safetyLineLossElapsedS: number;
  safetyCurrentRollDeg: number;
  safetyCurrentBatteryPercent: number;
}

export const DEFAULT_ROBOT_STATE: Readonly<RobotState> = {
  connected: false,
  authenticated: false,
  lastError: null,
  lastStatus: null,
  leftEncoder: 0,
  rightEncoder: 0,
  leftRpm: 0,
  rightRpm: 0,
  linearMps: 0,
  batteryV: 0,
  batteryPercent: 0,
  batteryRaw: 0,
  cpu0Percent: 0,
  cpu1Percent: 0,
  zeroBrakeEnabled: true,
  roll: 0,
  pitch: 0,
  yaw: 0,
  xM: 0,
  yM: 0,
  headingRad: 0,
  angularRadS: 0,
  encoderXM: 0,
  encoderYM: 0,
  encoderHeadingRad: 0,
  imuXM: 0,
  imuYM: 0,
  imuHeadingRad: 0,
  fusedXM: 0,
  fusedYM: 0,
  fusedHeadingRad: 0,
  odometryImuAvailable: false,
  controlRunning: false,
  controlMode: ControlMode.Odometry,
  controlMapSlot: 0,
  controlTargetIndex: 0,
  controlPointCount: 0,
  controlSpeedPercent: 0,
  controlActiveSpeedPercent: 0,
  controlTargetXM: 0,
  controlTargetYM: 0,
  controlDistanceM: 0,
  controlAngleErrorRad: 0,
  controlSteerPercent: 0,
  controlKp: 35,
  controlKi: 0,
  controlKd: 0,
  controlLineAlpha: 0.7,
  controlMotorLimitPercent: 100,
  controlLoopHz: 0,
  racePlanLoopHz: 0,
  trackOdometryLoopHz: 0,
  lineSensorLoopHz: 0,
  imuLoopHz: 0,
  controlSpeedProfileEnabled: true,
  controlAuxPercent: 0,
  controlActiveAuxPercent: 0,
  controlAverageSpeedMps: 0,
  controlMaxSpeedMps: 0,
  controlRacePlanAverageSpeedMps: 0,
  controlBatteryCompensationEnabled: false,
  controlMapPoseValid: false,
  controlMapXM: 0,
  controlMapYM: 0,
  controlMapHeadingRad: 0,
  controlRaceSegmentActive: false,
  controlRaceSegmentType: RaceSegmentType.Straight,
  controlRaceSegmentStartIndex: 0,
  controlRaceSegmentEndIndex: 0,
  controlRaceSegmentSpeedPercent: 0,
  controlRaceSegmentMaxSpeedPercent: 0,
  controlRaceSegmentAuxPercent: 0,
  lineRaw: [0, 0, 0, 0, 0, 0, 0, 0],
  lineCalibrated: [0, 0, 0, 0, 0, 0, 0, 0],
  lineValues: [0, 0, 0, 0, 0, 0, 0, 0],
  linePosition: 0,
  lineTrackType: LineTrackType.Black,
  lineVisible: false,
  lineCalibratedValid: false,
  lineCalibrating: false,
  lineThresholdPercent: 0,
  lineFilterPercent: 100,
  lineReadHz: 0,
  rgbLedMode: RgbLedMode.Battery,
  rgbLedRed: 255,
  rgbLedGreen: 255,
  rgbLedBlue: 255,
  rgbLedIntensity: 32,
  rgbLedEnabled: true,
  safetyCollisionEnabled: true,
  safetyBatteryBlockEnabled: true,
  safetyLineLossEnabled: true,
  safetyBleLossEnabled: true,
  safetyCollisionActive: false,
  safetyBatteryBlockActive: false,
  safetyLineLossActive: false,
  safetyBleLossActive: false,
  safetyMotorsBlocked: false,
  safetyLineVisible: false,
  safetyBleConnected: false,
  safetyRollLimitDeg: 6,
  safetyBatteryBlockPercent: 10,
  safetyLineLossTimeoutS: 1,
  safetyLineLossElapsedS: 0,
  safetyCurrentRollDeg: 0,
  safetyCurrentBatteryPercent: 0,
};

export interface TelemetryRecord {
  messageId: number;
  payload: Uint8Array;
  patch?: Partial<RobotState>;
  maps?: RobotMap[];
  mapChunk?: MapChunk;
  mapRecordChunk?: MapRecordChunk;
  status?: number;
}

export interface DecodedTelemetryPacket {
  packet: ProtocolPacket;
  records: TelemetryRecord[];
  patches: Partial<RobotState>[];
  maps?: RobotMap[];
  mapChunk?: MapChunk;
  mapRecordChunk?: MapRecordChunk;
  status?: number;
}
