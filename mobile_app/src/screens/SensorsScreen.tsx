import { useState } from 'react';
import { Alert, StyleSheet, Text, View } from 'react-native';

import { useRobot } from '../ble/BleProvider';
import { Button, Card, LabeledInput, Metric, Screen, SectionTitle, SegmentedControl, StatusPill, Stepper, ToggleRow, ValueRow } from '../components/ui';
import {
  packImuCalibrateAccelGyro,
  packImuCalibrateAll,
  packImuCalibrateMag,
  packImuCalibrateYawDrift,
  packImuMagFilterGain,
  packImuMagHeadingMode,
  packImuMagIgnored,
  packImuYawDriftThreshold,
  packLineCalibrate,
  packLineFilter,
  packLineThreshold,
  packLineTrackType,
  packOdometryPosition,
  packResetEncoders,
  packResetYaw,
} from '../protocol/protocol';
import { LineTrackType } from '../protocol/types';
import { colors, radius, spacing } from '../theme';

export function SensorsScreen() {
  const ble = useRobot();
  const state = ble.robot;
  const [calibrationTime, setCalibrationTime] = useState(5);
  const [trackType, setTrackType] = useState<LineTrackType>(state.lineTrackType);
  const [threshold, setThreshold] = useState(state.lineThresholdPercent);
  const [filter, setFilter] = useState(state.lineFilterPercent);
  const [magIgnored, setMagIgnored] = useState(false);
  const [magGain, setMagGain] = useState(0.05);
  const [headingMode, setHeadingMode] = useState(0);
  const [driftThreshold, setDriftThreshold] = useState(0.25);
  const [x, setX] = useState('0');
  const [y, setY] = useState('0');

  const send = (packet: Uint8Array) => ble.sendCommand(packet).catch((error) => Alert.alert('Comando não enviado', String(error)));

  return (
    <Screen>
      <SectionTitle title="Sensor de linha QRE-8D" />
      <Card accent={state.lineVisible ? colors.green : colors.red}>
        <View style={styles.metrics}>
          <Metric label="Posição" value={state.linePosition} unit="/7000" color={state.lineVisible ? colors.green : colors.red} />
          <Metric label="Leitura" value={state.lineReadHz.toFixed(0)} unit="Hz" />
        </View>
        <View style={styles.row}>
          <StatusPill label={state.lineVisible ? 'Linha visível' : 'Linha perdida'} tone={state.lineVisible ? 'good' : 'danger'} />
          <StatusPill label={state.lineCalibrating ? 'Calibrando' : state.lineCalibratedValid ? 'Calibrado' : 'Pendente'} tone={state.lineCalibratedValid ? 'good' : 'warn'} />
        </View>
        <View style={styles.sensorGrid}>
          {Array.from({ length: 8 }, (_, index) => (
            <View key={index} style={styles.sensor}>
              <Text style={styles.sensorName}>QTR {index + 1}</Text>
              <Text style={styles.sensorValue}>{state.lineValues[index] ?? 0}</Text>
              <Text style={styles.sensorRaw}>{state.lineRaw[index] ?? 0} µs</Text>
              <View style={styles.bar}><View style={[styles.barFill, { width: `${Math.min(100, (state.lineCalibrated[index] ?? 0) / 10)}%` }]} /></View>
            </View>
          ))}
        </View>
      </Card>
      <Card>
        <SegmentedControl
          value={trackType}
          onChange={setTrackType}
          options={[
            { label: 'Linha preta', value: LineTrackType.Black, color: colors.text },
            { label: 'Linha branca', value: LineTrackType.White, color: colors.cyan },
          ]}
        />
        <Stepper label="Tempo de calibração" value={calibrationTime} onChange={setCalibrationTime} min={1} max={120} suffix="s" />
        <Stepper label="Limiar" value={threshold} onChange={setThreshold} min={0} max={45} suffix="%" />
        <Stepper label="Filtro" value={filter} onChange={setFilter} min={0} max={100} step={5} suffix="%" />
        <Button title="Calibrar sensor" variant="primary" onPress={() => send(packLineCalibrate(calibrationTime))} />
        <Button title="Aplicar pista, limiar e filtro" onPress={() => ble.sendSequence([packLineTrackType(trackType), packLineThreshold(threshold), packLineFilter(filter)])} />
      </Card>

      <SectionTitle title="IMU" subtitle="O BLE recebe somente roll, pitch e yaw, conforme o pacote reduzido do firmware." />
      <View style={styles.metrics}>
        <Metric label="Roll" value={state.roll.toFixed(1)} unit="°" color={colors.orange} />
        <Metric label="Pitch" value={state.pitch.toFixed(1)} unit="°" color={colors.green} />
        <Metric label="Yaw" value={state.yaw.toFixed(1)} unit="°" color={colors.cyan} />
      </View>
      <Card>
        <View style={styles.row}>
          <View style={styles.flex}><Button title="Acc/Gyro · 5 s" compact onPress={() => send(packImuCalibrateAccelGyro(5))} /></View>
          <View style={styles.flex}><Button title="Drift yaw · 20 s" compact onPress={() => send(packImuCalibrateYawDrift(20))} /></View>
        </View>
        <View style={styles.row}>
          <View style={styles.flex}><Button title="Mag · 30 s" compact onPress={() => send(packImuCalibrateMag(30))} /></View>
          <View style={styles.flex}><Button title="Completa · 45 s" compact variant="primary" onPress={() => send(packImuCalibrateAll(45))} /></View>
        </View>
        <Button title="Zerar yaw" onPress={() => send(packResetYaw())} />
        <ToggleRow label="Ignorar magnetômetro" value={magIgnored} onValueChange={(value) => { setMagIgnored(value); send(packImuMagIgnored(value)); }} />
        <Stepper label="Ganho do filtro magnético" value={magGain} onChange={setMagGain} min={0} max={0.2} step={0.01} decimals={2} />
        <Stepper label="Modo heading" value={headingMode} onChange={setHeadingMode} min={0} max={2} />
        <Stepper label="Limiar de drift yaw" value={driftThreshold} onChange={setDriftThreshold} min={0} max={5} step={0.05} decimals={2} suffix="dps" />
        <Button title="Aplicar ajustes da IMU" onPress={() => ble.sendSequence([packImuMagFilterGain(magGain), packImuMagHeadingMode(headingMode), packImuYawDriftThreshold(driftThreshold)])} />
      </Card>

      <SectionTitle title="Odometria completa" />
      <Card>
        <ValueRow label="Operacional" value={`${state.xM.toFixed(3)}, ${state.yM.toFixed(3)}, ${state.headingRad.toFixed(3)} rad`} />
        <ValueRow label="Encoder" value={`${state.encoderXM.toFixed(3)}, ${state.encoderYM.toFixed(3)}, ${state.encoderHeadingRad.toFixed(3)} rad`} />
        <ValueRow label="IMU" value={`${state.imuXM.toFixed(3)}, ${state.imuYM.toFixed(3)}, ${state.imuHeadingRad.toFixed(3)} rad`} />
        <ValueRow label="Fusão" value={`${state.fusedXM.toFixed(3)}, ${state.fusedYM.toFixed(3)}, ${state.fusedHeadingRad.toFixed(3)} rad`} />
        <ValueRow label="IMU disponível" value={state.odometryImuAvailable ? 'sim' : 'não'} />
        <View style={styles.row}>
          <View style={styles.flex}><LabeledInput label="X (m)" value={x} onChangeText={setX} keyboardType="numeric" /></View>
          <View style={styles.flex}><LabeledInput label="Y (m)" value={y} onChangeText={setY} keyboardType="numeric" /></View>
        </View>
        <Button title="Definir posição e zerar heading" onPress={() => send(packOdometryPosition(Number(x.replace(',', '.')) || 0, Number(y.replace(',', '.')) || 0))} />
        <Button title="Reset encoders" onPress={() => send(packResetEncoders())} />
      </Card>
    </Screen>
  );
}

const styles = StyleSheet.create({
  metrics: { flexDirection: 'row', gap: spacing.sm },
  row: { flexDirection: 'row', gap: spacing.sm, alignItems: 'center' },
  flex: { flex: 1 },
  sensorGrid: { flexDirection: 'row', flexWrap: 'wrap', gap: spacing.sm },
  sensor: { width: '47%', backgroundColor: colors.surfaceRaised, borderRadius: radius.sm, padding: spacing.sm, gap: 3 },
  sensorName: { color: colors.textMuted, fontSize: 11 },
  sensorValue: { color: colors.text, fontSize: 20, fontWeight: '700', fontVariant: ['tabular-nums'] },
  sensorRaw: { color: colors.textMuted, fontSize: 10 },
  bar: { height: 3, backgroundColor: colors.border, borderRadius: 2, overflow: 'hidden' },
  barFill: { height: 3, backgroundColor: colors.green },
});
