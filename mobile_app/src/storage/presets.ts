import AsyncStorage from '@react-native-async-storage/async-storage';

import type { RunPreset } from '../protocol/types';

const PRESETS_KEY = '@aspirador/presets/v1';
const LAST_DEVICE_KEY = '@aspirador/last-device/v1';

export async function loadRunPresets(): Promise<RunPreset[]> {
  const raw = await AsyncStorage.getItem(PRESETS_KEY);
  if (!raw) {
    return [];
  }
  try {
    const value: unknown = JSON.parse(raw);
    if (!Array.isArray(value)) {
      return [];
    }
    return value.filter(isRunPreset);
  } catch {
    return [];
  }
}

export async function saveRunPresets(presets: RunPreset[]): Promise<void> {
  await AsyncStorage.setItem(PRESETS_KEY, JSON.stringify(presets));
}

export async function loadLastDeviceId(): Promise<string | null> {
  return AsyncStorage.getItem(LAST_DEVICE_KEY);
}

export async function saveLastDeviceId(deviceId: string | null): Promise<void> {
  if (deviceId) {
    await AsyncStorage.setItem(LAST_DEVICE_KEY, deviceId);
  } else {
    await AsyncStorage.removeItem(LAST_DEVICE_KEY);
  }
}

function isRunPreset(value: unknown): value is RunPreset {
  if (!value || typeof value !== 'object') {
    return false;
  }
  const preset = value as Partial<RunPreset>;
  return (
    typeof preset.id === 'string' &&
    typeof preset.name === 'string' &&
    Number.isInteger(preset.mapSlot) &&
    typeof preset.speedPercent === 'number' &&
    typeof preset.useRacePlan === 'boolean'
  );
}
