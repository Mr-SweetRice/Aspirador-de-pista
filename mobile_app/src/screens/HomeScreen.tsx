import { useEffect, useState } from 'react';
import { Alert, StyleSheet, Text, View } from 'react-native';

import { useRobot, type StartRunOptions } from '../ble/BleProvider';
import { Button, Card, LabeledInput, Metric, Screen, SectionTitle, SegmentedControl, StatusPill, Stepper, ToggleRow, ValueRow } from '../components/ui';
import type { RunPreset } from '../protocol/types';
import { loadRunPresets, saveRunPresets } from '../storage/presets';
import { colors, radius, spacing } from '../theme';

const MODE_OPTIONS = [
  { label: 'Plano', value: 'race' as const, color: colors.orange },
  { label: 'Auto pista', value: 'auto' as const, color: colors.green },
  { label: 'Linha', value: 'line' as const, color: colors.cyan },
  { label: 'Odometria', value: 'odometry' as const, color: colors.yellow },
];

export function HomeScreen() {
  const ble = useRobot();
  const [token, setToken] = useState(ble.authToken);
  const [mode, setMode] = useState<StartRunOptions['mode']>('race');
  const [mapSlot, setMapSlot] = useState(0);
  const [speed, setSpeed] = useState(50);
  const [aux, setAux] = useState(0);
  const [autoReset, setAutoReset] = useState(true);
  const [lineFallback, setLineFallback] = useState(true);
  const [batteryCompensation, setBatteryCompensation] = useState(false);
  const [presetName, setPresetName] = useState('Minha corrida');
  const [presets, setPresets] = useState<RunPreset[]>([]);
  const [busy, setBusy] = useState(false);

  useEffect(() => { setToken(ble.authToken); }, [ble.authToken]);
  useEffect(() => { loadRunPresets().then(setPresets).catch(() => undefined); }, []);
  useEffect(() => {
    if (ble.maps.length && !ble.maps.some((map) => map.slot === mapSlot)) setMapSlot(ble.maps[0]!.slot);
  }, [ble.maps, mapSlot]);

  const canStart = ble.robot.authenticated && (mode === 'line' || ble.maps.some((map) => map.slot === mapSlot));

  const start = async () => {
    setBusy(true);
    try {
      await ble.startRun({
        mode,
        mapSlot,
        speedPercent: speed,
        auxPercent: aux,
        autoResetPosition: autoReset,
        lineLossOdometryEnabled: lineFallback,
        batteryCompensationEnabled: batteryCompensation,
      });
    } catch (error) {
      Alert.alert('Não foi possível iniciar', error instanceof Error ? error.message : String(error));
    } finally {
      setBusy(false);
    }
  };

  const savePreset = async () => {
    const name = presetName.trim();
    if (!name) return;
    const preset: RunPreset = {
      id: `${Date.now()}`,
      name,
      mapSlot,
      speedPercent: speed,
      useRacePlan: mode === 'race',
      mode,
      auxPercent: aux,
      autoResetPosition: autoReset,
      lineLossOdometryEnabled: lineFallback,
      batteryCompensationEnabled: batteryCompensation,
      updatedAt: new Date().toISOString(),
    };
    const next = [preset, ...presets].slice(0, 30);
    setPresets(next);
    await saveRunPresets(next);
  };

  const loadPreset = (preset: RunPreset) => {
    setPresetName(preset.name);
    setMapSlot(preset.mapSlot);
    setSpeed(preset.speedPercent);
    setMode(preset.mode ?? (preset.useRacePlan ? 'race' : 'auto'));
    setAux(preset.auxPercent ?? 0);
    setAutoReset(preset.autoResetPosition ?? true);
    setLineFallback(preset.lineLossOdometryEnabled ?? true);
    setBatteryCompensation(preset.batteryCompensationEnabled ?? false);
  };

  const removePreset = async (id: string) => {
    const next = presets.filter((preset) => preset.id !== id);
    setPresets(next);
    await saveRunPresets(next);
  };

  return (
    <Screen>
      <SectionTitle title="Conexão" subtitle="Autenticação automática em até 10 segundos após conectar." />
      <Card accent={ble.robot.authenticated ? colors.green : colors.orange}>
        <View style={styles.row}>
          <StatusPill
            label={ble.robot.authenticated ? 'Autenticado' : ble.connectionStatus === 'scanning' ? 'Procurando' : 'Desconectado'}
            tone={ble.robot.authenticated ? 'good' : ble.connectionStatus === 'error' ? 'danger' : 'warn'}
          />
          <Text style={styles.status}>{ble.statusMessage}</Text>
        </View>
        {!ble.robot.connected ? (
          <>
            <LabeledInput label="Token BLE" value={token} onChangeText={setToken} secureTextEntry />
            <View style={styles.buttonRow}>
              <View style={styles.flex}><Button title="Salvar token" compact onPress={() => ble.setAuthToken(token)} /></View>
              <View style={styles.flex}><Button title={ble.connectionStatus === 'scanning' ? 'Procurando…' : 'Procurar robô'} compact variant="primary" onPress={ble.scan} disabled={ble.connectionStatus === 'scanning'} /></View>
            </View>
            {ble.devices.map((device) => (
              <View key={device.id} style={styles.device}>
                <View style={styles.flex}>
                  <Text style={styles.deviceName}>{device.name}</Text>
                  <Text style={styles.deviceId}>{device.id} · {device.rssi ?? '–'} dBm</Text>
                </View>
                <Button title="Conectar" compact variant="primary" onPress={() => ble.connect(device.id)} />
              </View>
            ))}
          </>
        ) : (
          <View style={styles.buttonRow}>
            <View style={styles.flex}><Button title="Atualizar mapas" compact onPress={ble.refreshMaps} /></View>
            <View style={styles.flex}><Button title="Desconectar" compact variant="ghost" onPress={ble.disconnect} /></View>
          </View>
        )}
      </Card>

      <SectionTitle title="Estado da corrida" />
      <View style={styles.metrics}>
        <Metric label="Bateria" value={ble.robot.batteryPercent.toFixed(0)} unit="%" color={ble.robot.batteryPercent < 15 ? colors.red : colors.green} />
        <Metric label="Velocidade" value={ble.robot.linearMps.toFixed(2)} unit="m/s" />
        <Metric label="Turbina" value={ble.robot.controlActiveAuxPercent} unit="%" color={colors.cyan} />
      </View>
      <Card>
        <ValueRow label="Controle" value={ble.robot.controlRunning ? 'RODANDO' : 'PARADO'} valueColor={ble.robot.controlRunning ? colors.green : colors.textMuted} />
        <ValueRow label="Mapa / alvo" value={`${ble.robot.controlMapSlot} · ${ble.robot.controlTargetIndex}/${ble.robot.controlPointCount}`} />
        <ValueRow label="Trecho" value={ble.robot.controlRaceSegmentActive ? `${ble.robot.controlRaceSegmentType} · ${ble.robot.controlRaceSegmentStartIndex}–${ble.robot.controlRaceSegmentEndIndex}` : 'fora do plano'} />
        <ValueRow label="Segurança" value={ble.robot.safetyMotorsBlocked ? 'MOTORES BLOQUEADOS' : 'liberado'} valueColor={ble.robot.safetyMotorsBlocked ? colors.red : colors.green} />
      </Card>

      <SectionTitle title="Executar" subtitle="Plano usa o último plano persistido no robô; a pré-partida de 2 s é controlada pelo firmware." />
      <Card accent={mode === 'race' ? colors.orange : colors.green}>
        <SegmentedControl options={MODE_OPTIONS} value={mode} onChange={setMode} />
        {mode !== 'line' ? (
          <>
            <Text style={styles.label}>Mapa salvo no robô</Text>
            <View style={styles.mapGrid}>
              {ble.maps.map((map) => (
                <Text
                  key={map.slot}
                  onPress={() => setMapSlot(map.slot)}
                  style={[styles.map, mapSlot === map.slot ? styles.mapSelected : null]}
                >
                  {map.name || `Slot ${map.slot}`}\n{map.pointCount} pts · {map.distanceM.toFixed(2)} m
                </Text>
              ))}
              {!ble.maps.length ? <Text style={styles.muted}>Conecte e atualize os mapas.</Text> : null}
            </View>
          </>
        ) : null}
        <Stepper label="Velocidade" value={speed} onChange={setSpeed} min={-100} max={100} step={5} suffix="%" />
        <Stepper label="Turbina" value={aux} onChange={setAux} min={0} max={100} step={5} suffix="%" />
        {mode === 'race' ? (
          <>
            <ToggleRow label="Odometria ao perder linha" value={lineFallback} onValueChange={setLineFallback} />
            <ToggleRow label="Compensação de bateria" value={batteryCompensation} onValueChange={setBatteryCompensation} />
          </>
        ) : null}
        {mode !== 'line' ? <ToggleRow label="Reposicionar no ponto inicial" value={autoReset} onValueChange={setAutoReset} /> : null}
        <Button title={busy ? 'Enviando configuração…' : ble.robot.controlRunning ? 'CORRIDA EM EXECUÇÃO' : 'START'} variant="primary" onPress={start} disabled={!canStart || busy || ble.robot.controlRunning} />
        <Button title="STOP DO CONTROLE" variant="danger" onPress={ble.stopControl} disabled={!ble.robot.authenticated} />
      </Card>

      <SectionTitle title="Configurações salvas no celular" subtitle="Salva mapa, modo, velocidade, turbina e proteções. O robô mantém apenas um plano de segmentos global." />
      <Card>
        <LabeledInput label="Nome" value={presetName} onChangeText={setPresetName} />
        <Button title="Salvar configuração atual" onPress={savePreset} />
        {presets.map((preset) => (
          <View key={preset.id} style={styles.preset}>
            <View style={styles.flex}>
              <Text style={styles.deviceName}>{preset.name}</Text>
              <Text style={styles.deviceId}>{preset.mode ?? (preset.useRacePlan ? 'race' : 'auto')} · mapa {preset.mapSlot} · {preset.speedPercent}%</Text>
            </View>
            <Button title="Usar" compact onPress={() => loadPreset(preset)} />
            <Text style={styles.remove} onPress={() => Alert.alert('Excluir configuração?', preset.name, [{ text: 'Cancelar' }, { text: 'Excluir', style: 'destructive', onPress: () => removePreset(preset.id) }])}>×</Text>
          </View>
        ))}
      </Card>
    </Screen>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', alignItems: 'center', gap: spacing.md },
  status: { color: colors.textMuted, flex: 1, fontSize: 12 },
  flex: { flex: 1 },
  buttonRow: { flexDirection: 'row', gap: spacing.sm },
  device: { flexDirection: 'row', alignItems: 'center', gap: spacing.sm, paddingTop: spacing.md, borderTopWidth: 1, borderTopColor: colors.border },
  deviceName: { color: colors.text, fontSize: 15, fontWeight: '700' },
  deviceId: { color: colors.textMuted, fontSize: 11, marginTop: 3 },
  metrics: { flexDirection: 'row', gap: spacing.sm },
  label: { color: colors.textMuted, fontSize: 13 },
  mapGrid: { gap: spacing.sm },
  map: { color: colors.textMuted, backgroundColor: colors.surfaceRaised, borderColor: colors.border, borderWidth: 1, borderRadius: radius.sm, padding: spacing.md, lineHeight: 19 },
  mapSelected: { color: colors.green, borderColor: colors.green },
  muted: { color: colors.textMuted, fontSize: 13 },
  preset: { flexDirection: 'row', alignItems: 'center', gap: spacing.sm, paddingTop: spacing.md, borderTopWidth: 1, borderTopColor: colors.border },
  remove: { color: colors.red, fontSize: 28, paddingHorizontal: spacing.sm },
});
