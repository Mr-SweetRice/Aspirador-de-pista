import { fromByteArray, toByteArray } from 'base64-js';
import * as SecureStore from 'expo-secure-store';
import { createContext, useCallback, useContext, useEffect, useMemo, useRef, useState, type ReactNode } from 'react';
import { PermissionsAndroid, Platform } from 'react-native';
import { BleManager, type Device, type Subscription } from 'react-native-ble-plx';

import {
  AUTH_TOKEN,
  AUTH_UUID,
  COMMAND_UUID,
  DEVICE_NAME,
  SERVICE_UUID,
  TELEMETRY_UUID,
  decodeTelemetryPacket,
  packControlAutoTrackConfig,
  packControlAuxPercent,
  packControlBatteryCompensationEnabled,
  packControlRacePlan,
  packControlStartAutoTrack,
  packControlStartLine,
  packControlStartMap,
  packControlStop,
  packEmergencyStop,
  packOdometryPosition,
  packOdometrySource,
  packReadMapChunk,
  packReadMapList,
} from '../protocol/protocol';
import { DEFAULT_ROBOT_STATE, OdometrySource, type RacePlanSegment, type RobotMap, type RobotState } from '../protocol/types';
import { CommandQueue, CommandCancelledError } from './CommandQueue';
import { loadLastDeviceId, saveLastDeviceId } from '../storage/presets';

const TOKEN_KEY = 'aspirador-ble-token';
const TELEMETRY_RENDER_MS = 50;

export interface BleDeviceInfo {
  id: string;
  name: string;
  rssi: number | null;
}

export interface PacketStats {
  total: number;
  parseErrors: number;
  lastPacketAt: number | null;
  messageCounts: Record<number, number>;
}

export interface StartRunOptions {
  mode: 'race' | 'auto' | 'line' | 'odometry';
  mapSlot: number;
  speedPercent: number;
  auxPercent: number;
  autoResetPosition: boolean;
  lineLossOdometryEnabled?: boolean;
  batteryCompensationEnabled?: boolean;
  racePlan?: RacePlanSegment[];
}

interface BleContextValue {
  robot: RobotState;
  devices: BleDeviceInfo[];
  maps: RobotMap[];
  mapFirstPoints: Record<number, { x: number; y: number }>;
  packetStats: PacketStats;
  connectionStatus: 'idle' | 'scanning' | 'connecting' | 'connected' | 'error';
  connectedDeviceName: string | null;
  statusMessage: string;
  authToken: string;
  setAuthToken: (token: string) => Promise<void>;
  scan: () => Promise<void>;
  connect: (deviceId: string) => Promise<void>;
  disconnect: () => Promise<void>;
  refreshMaps: () => Promise<void>;
  sendCommand: (packet: Uint8Array) => Promise<void>;
  sendSequence: (packets: Uint8Array[]) => Promise<void>;
  startRun: (options: StartRunOptions) => Promise<void>;
  stopControl: () => Promise<void>;
  emergencyStop: () => Promise<void>;
}

const BleContext = createContext<BleContextValue | null>(null);

const sleep = (milliseconds: number) => new Promise<void>((resolve) => setTimeout(resolve, milliseconds));

export function BleProvider({ children }: { children: ReactNode }) {
  const managerRef = useRef<BleManager | null>(null);
  if (!managerRef.current) managerRef.current = new BleManager();

  const deviceRef = useRef<Device | null>(null);
  const monitorRef = useRef<Subscription | null>(null);
  const disconnectRef = useRef<Subscription | null>(null);
  const scanTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const renderTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null);
  const commandQueueRef = useRef<CommandQueue | null>(null);
  const robotRef = useRef<RobotState>({ ...DEFAULT_ROBOT_STATE });
  const mapFirstPointsRef = useRef<Record<number, { x: number; y: number }>>({});
  const statsRef = useRef<PacketStats>({ total: 0, parseErrors: 0, lastPacketAt: null, messageCounts: {} });
  const statsLastRenderRef = useRef(0);

  const [robot, setRobot] = useState<RobotState>({ ...DEFAULT_ROBOT_STATE });
  const [devices, setDevices] = useState<BleDeviceInfo[]>([]);
  const [maps, setMaps] = useState<RobotMap[]>([]);
  const [mapFirstPoints, setMapFirstPoints] = useState<Record<number, { x: number; y: number }>>({});
  const [packetStats, setPacketStats] = useState<PacketStats>({ ...statsRef.current });
  const [connectionStatus, setConnectionStatus] = useState<BleContextValue['connectionStatus']>('idle');
  const [connectedDeviceName, setConnectedDeviceName] = useState<string | null>(null);
  const [statusMessage, setStatusMessage] = useState('Pronto para procurar o robô');
  const [authTokenState, setAuthTokenState] = useState(AUTH_TOKEN);
  const authTokenRef = useRef(AUTH_TOKEN);

  const flushRobotSoon = useCallback(() => {
    if (renderTimerRef.current) return;
    renderTimerRef.current = setTimeout(() => {
      renderTimerRef.current = null;
      setRobot({ ...robotRef.current });
    }, TELEMETRY_RENDER_MS);
  }, []);

  const updateRobot = useCallback((patch: Partial<RobotState>) => {
    robotRef.current = { ...robotRef.current, ...patch };
    flushRobotSoon();
  }, [flushRobotSoon]);

  useEffect(() => {
    SecureStore.getItemAsync(TOKEN_KEY).then((saved) => {
      if (saved) {
        authTokenRef.current = saved;
        setAuthTokenState(saved);
      }
    }).catch(() => undefined);
    return () => {
      commandQueueRef.current?.cancel();
      if (scanTimerRef.current) clearTimeout(scanTimerRef.current);
      if (renderTimerRef.current) clearTimeout(renderTimerRef.current);
      monitorRef.current?.remove();
      disconnectRef.current?.remove();
      managerRef.current?.stopDeviceScan();
      managerRef.current?.destroy();
      managerRef.current = null;
    };
  }, []);

  const setAuthToken = useCallback(async (token: string) => {
    const clean = token.trim() || AUTH_TOKEN;
    authTokenRef.current = clean;
    setAuthTokenState(clean);
    await SecureStore.setItemAsync(TOKEN_KEY, clean);
  }, []);

  const requestPermissions = useCallback(async () => {
    if (Platform.OS !== 'android') return true;
    const version = typeof Platform.Version === 'number' ? Platform.Version : Number(Platform.Version);
    if (version >= 31) {
      const result = await PermissionsAndroid.requestMultiple([
        PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
        PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
      ]);
      return result[PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN] === PermissionsAndroid.RESULTS.GRANTED &&
        result[PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT] === PermissionsAndroid.RESULTS.GRANTED;
    }
    return (await PermissionsAndroid.request(PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION)) === PermissionsAndroid.RESULTS.GRANTED;
  }, []);

  const stopScan = useCallback(() => {
    managerRef.current?.stopDeviceScan();
    if (scanTimerRef.current) clearTimeout(scanTimerRef.current);
    scanTimerRef.current = null;
  }, []);

  const scan = useCallback(async () => {
    if (!await requestPermissions()) {
      setConnectionStatus('error');
      setStatusMessage('Permissão Bluetooth negada');
      return;
    }
    stopScan();
    setDevices([]);
    setConnectionStatus('scanning');
    setStatusMessage('Procurando AspiradorPista-S3…');
    const found = new Map<string, BleDeviceInfo>();
    managerRef.current?.startDeviceScan(null, { allowDuplicates: false }, (error, device) => {
      if (error) {
        stopScan();
        setConnectionStatus('error');
        setStatusMessage(`Falha no scan: ${error.message}`);
        return;
      }
      if (!device) return;
      const name = device.name ?? device.localName ?? 'Dispositivo BLE';
      const services = (device.serviceUUIDs ?? []).map((uuid) => uuid.toLowerCase());
      if (!name.startsWith(DEVICE_NAME) && !services.includes(SERVICE_UUID.toLowerCase())) return;
      found.set(device.id, { id: device.id, name, rssi: device.rssi ?? null });
      setDevices(Array.from(found.values()).sort((a, b) => (b.rssi ?? -999) - (a.rssi ?? -999)));
    });
    scanTimerRef.current = setTimeout(() => {
      stopScan();
      setConnectionStatus('idle');
      setStatusMessage(found.size ? `${found.size} robô(s) encontrado(s)` : 'Nenhum robô encontrado');
    }, 8000);
  }, [requestPermissions, stopScan]);

  const handleTelemetry = useCallback((base64Value: string) => {
    try {
      const decoded = decodeTelemetryPacket(toByteArray(base64Value));
      statsRef.current.total += 1;
      statsRef.current.lastPacketAt = Date.now();
      for (const record of decoded.records) {
        statsRef.current.messageCounts[record.messageId] = (statsRef.current.messageCounts[record.messageId] ?? 0) + 1;
      }
      for (const patch of decoded.patches) robotRef.current = { ...robotRef.current, ...patch };
      if (decoded.maps) setMaps(decoded.maps);
      if (decoded.mapChunk?.offset === 0 && decoded.mapChunk.points[0]) {
        const point = decoded.mapChunk.points[0];
        mapFirstPointsRef.current = { ...mapFirstPointsRef.current, [decoded.mapChunk.slot]: { x: point.x, y: point.y } };
        setMapFirstPoints(mapFirstPointsRef.current);
      }
      flushRobotSoon();
      const now = Date.now();
      if (now - statsLastRenderRef.current > 250) {
        statsLastRenderRef.current = now;
        setPacketStats({ ...statsRef.current, messageCounts: { ...statsRef.current.messageCounts } });
      }
    } catch (error) {
      statsRef.current.parseErrors += 1;
      updateRobot({ lastError: error instanceof Error ? error.message : String(error) });
    }
  }, [flushRobotSoon, updateRobot]);

  const disconnect = useCallback(async () => {
    stopScan();
    commandQueueRef.current?.cancel();
    monitorRef.current?.remove();
    disconnectRef.current?.remove();
    monitorRef.current = null;
    disconnectRef.current = null;
    const device = deviceRef.current;
    deviceRef.current = null;
    if (device) {
      try { await managerRef.current?.cancelDeviceConnection(device.id); } catch { /* já desconectado */ }
    }
    updateRobot({ connected: false, authenticated: false, controlRunning: false });
    setConnectedDeviceName(null);
    setConnectionStatus('idle');
    setStatusMessage('Desconectado');
  }, [stopScan, updateRobot]);

  const connect = useCallback(async (deviceId: string) => {
    stopScan();
    setConnectionStatus('connecting');
    setStatusMessage('Conectando e autenticando…');
    try {
      if (deviceRef.current) await disconnect();
      let device = await managerRef.current!.connectToDevice(deviceId, { timeout: 12000 });
      device = await device.discoverAllServicesAndCharacteristics();
      if (Platform.OS === 'android') {
        try { device = await device.requestMTU(256); } catch { /* usa MTU negociado */ }
      }
      await device.writeCharacteristicWithResponseForService(SERVICE_UUID, AUTH_UUID, fromByteArray(Uint8Array.from(unescape(encodeURIComponent(authTokenRef.current)), (char) => char.charCodeAt(0))));
      monitorRef.current = device.monitorCharacteristicForService(SERVICE_UUID, TELEMETRY_UUID, (error, characteristic) => {
        if (error) {
          updateRobot({ lastError: error.message });
          return;
        }
        if (characteristic?.value) handleTelemetry(characteristic.value);
      });
      disconnectRef.current = managerRef.current!.onDeviceDisconnected(device.id, () => {
        if (deviceRef.current?.id !== device.id) return;
        commandQueueRef.current?.cancel();
        deviceRef.current = null;
        updateRobot({ connected: false, authenticated: false, controlRunning: false });
        setConnectionStatus('idle');
        setConnectedDeviceName(null);
        setStatusMessage('Robô desconectado; a proteção BLE deve parar os motores');
      });
      commandQueueRef.current?.cancel();
      deviceRef.current = device;
      setConnectedDeviceName(device.name ?? device.localName ?? DEVICE_NAME);
      updateRobot({ connected: true, authenticated: true, lastError: null });
      setConnectionStatus('connected');
      setStatusMessage(`Conectado · MTU ${device.mtu ?? 'automático'}`);
      await saveLastDeviceId(device.id);
      await sleep(120);
      await commandQueueRef.current!.enqueue(packReadMapList());
    } catch (error) {
      updateRobot({ connected: false, authenticated: false, lastError: error instanceof Error ? error.message : String(error) });
      setConnectionStatus('error');
      setStatusMessage(`Falha ao conectar: ${error instanceof Error ? error.message : String(error)}`);
    }
  }, [disconnect, handleTelemetry, stopScan, updateRobot]);

  useEffect(() => {
    loadLastDeviceId().catch(() => null);
  }, []);

  const writePacket = useCallback(async (packet: Uint8Array, current: () => boolean) => {
    const device = deviceRef.current;
    if (!device || !robotRef.current.authenticated) throw new Error('Robô não está autenticado');
    for (let attempt = 1; attempt <= 3; attempt++) {
      if (!current() || device !== deviceRef.current) throw new CommandCancelledError();
      const transactionId = `command-${Date.now()}-${attempt}`;
      let timer: ReturnType<typeof setTimeout> | undefined;
      let timedOut = false;
      try {
        await Promise.race([
          device.writeCharacteristicWithResponseForService(SERVICE_UUID, COMMAND_UUID, fromByteArray(packet), transactionId),
          new Promise<never>((_, reject) => {
            timer = setTimeout(() => { timedOut = true; reject(new Error('Timeout BLE')); }, 2000);
          }),
        ]);
        return;
      } catch (error) {
        if (timedOut) {
          managerRef.current?.cancelTransaction(transactionId);
          if (device === deviceRef.current) await disconnect();
          throw error;
        }
        const resourceError = (error as { attErrorCode?: number })?.attErrorCode === 0x11;
        if (!resourceError || attempt === 3) throw error;
        await sleep(30 * attempt);
      } finally {
        if (timer) clearTimeout(timer);
      }
    }
  }, [disconnect]);

  if (!commandQueueRef.current) commandQueueRef.current = new CommandQueue(writePacket);

  const sendCommand = useCallback((packet: Uint8Array) => {
    const stop = packet.length === 4 && packet[0] === 1 && packet[1] === 3 && (packet[2] === 1 || packet[2] === 0x41);
    const queued = commandQueueRef.current!.enqueue(packet, stop);
    void queued.catch((error) => {
      if (!(error instanceof CommandCancelledError)) updateRobot({ lastError: error instanceof Error ? error.message : String(error) });
    });
    return queued;
  }, [updateRobot]);

  const sendSequence = useCallback(async (packets: Uint8Array[]) => {
    const epoch = commandQueueRef.current!.generation;
    for (const packet of packets) {
      if (epoch !== commandQueueRef.current!.generation) throw new CommandCancelledError();
      await sendCommand(packet);
    }
  }, [sendCommand]);

  const refreshMaps = useCallback(async () => {
    await sendCommand(packReadMapList());
  }, [sendCommand]);

  const loadFirstPoint = useCallback(async (slot: number) => {
    if (mapFirstPointsRef.current[slot]) return mapFirstPointsRef.current[slot];
    await sendCommand(packReadMapChunk(slot, 0));
    for (let attempt = 0; attempt < 12; attempt++) {
      await sleep(60);
      if (mapFirstPointsRef.current[slot]) return mapFirstPointsRef.current[slot];
    }
    return undefined;
  }, [sendCommand]);

  const startRun = useCallback(async (options: StartRunOptions) => {
    const epoch = commandQueueRef.current!.generation;
    const packets: Uint8Array[] = [packControlAuxPercent(options.auxPercent)];
    if (options.mode === 'line') {
      packets.push(packControlStartLine(options.speedPercent));
      await sendSequence(packets);
      return;
    }
    if (options.mode === 'race') {
      packets.push(packControlAutoTrackConfig(options.lineLossOdometryEnabled ?? true));
      packets.push(packControlBatteryCompensationEnabled(options.batteryCompensationEnabled ?? false));
      if (options.racePlan?.length) packets.push(...packControlRacePlan(options.racePlan, true));
    }
    packets.push(packOdometrySource(OdometrySource.Fused));
    if (options.autoResetPosition) {
      const point = await loadFirstPoint(options.mapSlot);
      if (epoch !== commandQueueRef.current!.generation) throw new CommandCancelledError();
      if (point) packets.push(packOdometryPosition(point.x, point.y));
    }
    if (options.mode === 'odometry') packets.push(packControlStartMap(options.mapSlot, options.speedPercent));
    else packets.push(packControlStartAutoTrack(options.mapSlot, options.speedPercent, options.mode === 'race'));
    await sendSequence(packets);
  }, [loadFirstPoint, sendSequence]);

  const stopControl = useCallback(async () => {
    await sendCommand(packControlStop());
  }, [sendCommand]);

  const emergencyStop = useCallback(async () => {
    try {
      await sendCommand(packEmergencyStop());
      setStatusMessage('STOP de emergência enviado');
    } catch (error) {
      updateRobot({ lastError: `Falha no STOP: ${error instanceof Error ? error.message : String(error)}` });
    }
  }, [updateRobot, sendCommand]);

  const value = useMemo<BleContextValue>(() => ({
    robot, devices, maps, mapFirstPoints, packetStats, connectionStatus, connectedDeviceName, statusMessage,
    authToken: authTokenState, setAuthToken, scan, connect, disconnect, refreshMaps, sendCommand, sendSequence,
    startRun, stopControl, emergencyStop,
  }), [robot, devices, maps, mapFirstPoints, packetStats, connectionStatus, connectedDeviceName, statusMessage,
    authTokenState, setAuthToken, scan, connect, disconnect, refreshMaps, sendCommand, sendSequence, startRun,
    stopControl, emergencyStop]);

  return <BleContext.Provider value={value}>{children}</BleContext.Provider>;
}

export function useRobot() {
  const value = useContext(BleContext);
  if (!value) throw new Error('useRobot deve estar dentro de BleProvider');
  return value;
}
