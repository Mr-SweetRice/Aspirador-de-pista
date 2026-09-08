import { useEffect, useState } from 'react';
import { Alert, StyleSheet, Text, View } from 'react-native';

import { useRobot } from '../ble/BleProvider';
import { Button, Card, Screen, SectionTitle, SegmentedControl, Stepper, ToggleRow, ValueRow } from '../components/ui';
import {
  packControlAuxPercent,
  packControlBatteryCompensationEnabled,
  packControlPid,
  packControlSavePid,
  packResetEncoders,
  packResetYaw,
  packRgbLedEnabled,
  packRgbLedManual,
  packRgbLedMode,
  packSetAuxPwm,
  packSetLeftPwm,
  packSetRightPwm,
  packZeroBrakeEnabled,
} from '../protocol/protocol';
import { ControlMode, RgbLedMode } from '../protocol/types';
import { colors, spacing } from '../theme';

export function ControlScreen() {
  const ble = useRobot();
  const state = ble.robot;
  const [mode, setMode] = useState<ControlMode>(ControlMode.AutoTrack);
  const [mapSlot, setMapSlot] = useState(0);
  const [speed, setSpeed] = useState(25);
  const [aux, setAux] = useState(0);
  const [kp, setKp] = useState(35);
  const [ki, setKi] = useState(0);
  const [kd, setKd] = useState(0);
  const [alpha, setAlpha] = useState(0.7);
  const [limit, setLimit] = useState(100);
  const [leftPwm, setLeftPwm] = useState(0);
  const [rightPwm, setRightPwm] = useState(0);
  const [manualAux, setManualAux] = useState(0);
  const [red, setRed] = useState(255);
  const [green, setGreen] = useState(255);
  const [blue, setBlue] = useState(255);
  const [intensity, setIntensity] = useState(179);

  useEffect(() => {
    if (!state.authenticated) return;
    setKp(state.controlKp); setKi(state.controlKi); setKd(state.controlKd);
    setAlpha(state.controlLineAlpha); setLimit(state.controlMotorLimitPercent);
  }, [state.authenticated]);
  useEffect(() => {
    if (ble.maps.length && !ble.maps.some((map) => map.slot === mapSlot)) setMapSlot(ble.maps[0]!.slot);
  }, [ble.maps, mapSlot]);

  const send = (packet: Uint8Array) => ble.sendCommand(packet).catch((error) => Alert.alert('Comando não enviado', String(error)));
  const startNavigation = () => ble.startRun({
    mode: mode === ControlMode.Line ? 'line' : mode === ControlMode.Odometry ? 'odometry' : 'auto',
    mapSlot,
    speedPercent: speed,
    auxPercent: aux,
    autoResetPosition: true,
  }).catch((error) => Alert.alert('Falha ao iniciar', String(error)));

  return (
    <Screen>
      <SectionTitle title="Controle manual" subtitle="Aplique os valores somente com o robô suspenso ou em área livre." />
      <Card accent={colors.red}>
        <Stepper label="Motor esquerdo" value={leftPwm} onChange={setLeftPwm} min={-100} max={100} step={5} suffix="%" />
        <Stepper label="Motor direito" value={rightPwm} onChange={setRightPwm} min={-100} max={100} step={5} suffix="%" />
        <Stepper label="Turbina" value={manualAux} onChange={setManualAux} min={0} max={100} step={5} suffix="%" />
        <Button
          title="APLICAR PWM MANUAL"
          variant="danger"
          disabled={!state.authenticated}
          onPress={() => ble.sendSequence([packSetLeftPwm(leftPwm), packSetRightPwm(rightPwm), packSetAuxPwm(manualAux)])}
        />
        <Button title="Zerar motores" onPress={() => ble.sendSequence([packSetLeftPwm(0), packSetRightPwm(0), packSetAuxPwm(0)])} disabled={!state.authenticated} />
        <View style={styles.row}>
          <View style={styles.flex}><Button title="Reset encoders" compact onPress={() => send(packResetEncoders())} /></View>
          <View style={styles.flex}><Button title="Reset yaw" compact onPress={() => send(packResetYaw())} /></View>
        </View>
      </Card>

      <SectionTitle title="Navegação" />
      <Card>
        <SegmentedControl
          value={mode}
          onChange={setMode}
          options={[
            { label: 'Linha', value: ControlMode.Line, color: colors.cyan },
            { label: 'Auto pista', value: ControlMode.AutoTrack, color: colors.green },
            { label: 'Odometria', value: ControlMode.Odometry, color: colors.yellow },
          ]}
        />
        {mode !== ControlMode.Line ? (
          <>
            <Text style={styles.label}>Mapa</Text>
            <SegmentedControl
              value={mapSlot}
              onChange={setMapSlot}
              options={ble.maps.length ? ble.maps.map((map) => ({ label: map.name || `Slot ${map.slot}`, value: map.slot })) : [{ label: 'Sem mapas', value: 0 }]}
            />
          </>
        ) : null}
        <Stepper label="Velocidade" value={speed} onChange={setSpeed} min={-100} max={100} step={5} suffix="%" />
        <Stepper label="Turbina" value={aux} onChange={setAux} min={0} max={100} step={5} suffix="%" />
        <ToggleRow label="Compensação por bateria" value={state.controlBatteryCompensationEnabled} onValueChange={(value) => send(packControlBatteryCompensationEnabled(value))} />
        <ToggleRow label="Freio em 0%" value={state.zeroBrakeEnabled} onValueChange={(value) => send(packZeroBrakeEnabled(value))} />
        <Button title="START NAVEGAÇÃO" variant="primary" disabled={!state.authenticated || state.controlRunning} onPress={startNavigation} />
        <Button title="STOP CONTROLE" variant="danger" disabled={!state.authenticated} onPress={ble.stopControl} />
      </Card>

      <SectionTitle title="PID e limites" />
      <Card>
        <Stepper label="Kp" value={kp} onChange={setKp} min={0} max={1000} step={1} decimals={2} />
        <Stepper label="Ki" value={ki} onChange={setKi} min={0} max={1000} step={0.1} decimals={2} />
        <Stepper label="Kd" value={kd} onChange={setKd} min={0} max={1000} step={0.1} decimals={2} />
        <Stepper label="Alpha P" value={alpha} onChange={setAlpha} min={0} max={1} step={0.05} decimals={2} />
        <Stepper label="Limite dos motores" value={limit} onChange={setLimit} min={0} max={100} step={5} suffix="%" />
        <View style={styles.row}>
          <View style={styles.flex}><Button title="Aplicar" onPress={() => send(packControlPid(kp, ki, kd, limit, alpha))} /></View>
          <View style={styles.flex}><Button title="Salvar no robô" variant="primary" onPress={() => send(packControlSavePid(kp, ki, kd, limit, aux, alpha))} /></View>
        </View>
        <Button title="Aplicar turbina" onPress={() => send(packControlAuxPercent(aux))} />
      </Card>

      <SectionTitle title="Estado do controlador" />
      <Card>
        <ValueRow label="Rodando" value={state.controlRunning ? 'sim' : 'não'} />
        <ValueRow label="Target" value={`${state.controlTargetIndex}/${state.controlPointCount}`} />
        <ValueRow label="Erro angular" value={`${state.controlAngleErrorRad.toFixed(4)} rad`} />
        <ValueRow label="Distância" value={`${state.controlDistanceM.toFixed(3)} m`} />
        <ValueRow label="Correção" value={`${state.controlSteerPercent.toFixed(1)} %`} />
        <ValueRow label="Velocidade alvo / ativa" value={`${state.controlSpeedPercent} / ${state.controlActiveSpeedPercent} %`} />
        <ValueRow label="Turbina alvo / ativa" value={`${state.controlAuxPercent} / ${state.controlActiveAuxPercent} %`} />
        <ValueRow label="Média / máxima" value={`${state.controlAverageSpeedMps.toFixed(3)} / ${state.controlMaxSpeedMps.toFixed(3)} m/s`} />
      </Card>

      <SectionTitle title="LED RGB" />
      <Card>
        <ToggleRow label="LED habilitado" value={state.rgbLedEnabled} onValueChange={(value) => send(packRgbLedEnabled(value))} />
        <SegmentedControl
          value={state.rgbLedMode}
          onChange={(value) => send(packRgbLedMode(value))}
          options={[
            { label: 'Desligado', value: RgbLedMode.Disabled },
            { label: 'Bateria', value: RgbLedMode.Battery, color: colors.green },
            { label: 'Manual', value: RgbLedMode.Manual, color: colors.cyan },
            { label: 'Plano', value: RgbLedMode.RacePlan, color: colors.orange },
          ]}
        />
        <Stepper label="Vermelho" value={red} onChange={setRed} min={0} max={255} step={5} />
        <Stepper label="Verde" value={green} onChange={setGreen} min={0} max={255} step={5} />
        <Stepper label="Azul" value={blue} onChange={setBlue} min={0} max={255} step={5} />
        <Stepper label="Intensidade" value={intensity} onChange={setIntensity} min={0} max={255} step={5} />
        <View style={[styles.preview, { backgroundColor: `rgb(${red},${green},${blue})`, opacity: intensity / 255 }]} />
        <Button title="Aplicar cor manual" onPress={() => send(packRgbLedManual(red, green, blue, intensity))} />
      </Card>
    </Screen>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', gap: spacing.sm },
  flex: { flex: 1 },
  label: { color: colors.textMuted, fontSize: 13 },
  preview: { height: 34, borderRadius: 8, borderWidth: 1, borderColor: colors.border },
});
