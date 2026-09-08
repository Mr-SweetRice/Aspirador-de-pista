import { useState } from 'react';
import { Alert, StyleSheet, View } from 'react-native';

import { useRobot } from '../ble/BleProvider';
import { Button, Card, Metric, Screen, SectionTitle, StatusPill, Stepper, ToggleRow, ValueRow } from '../components/ui';
import {
  packSafetyBatteryBlockEnabled,
  packSafetyBatteryBlockPercent,
  packSafetyBleLossEnabled,
  packSafetyCollisionEnabled,
  packSafetyLineLossEnabled,
  packSafetyLineLossTimeout,
  packSafetyRollLimit,
} from '../protocol/protocol';
import { colors, spacing } from '../theme';

export function SafetyScreen() {
  const ble = useRobot();
  const state = ble.robot;
  const [rollLimit, setRollLimit] = useState(state.safetyRollLimitDeg || 6);
  const [batteryLimit, setBatteryLimit] = useState(state.safetyBatteryBlockPercent || 10);
  const [lineTimeout, setLineTimeout] = useState(state.safetyLineLossTimeoutS || 1);

  const send = (packet: Uint8Array) => ble.sendCommand(packet).catch((error) => Alert.alert('Configuração não enviada', String(error)));
  const guardedToggle = (label: string, enabled: boolean, packet: Uint8Array) => {
    if (enabled) { send(packet); return; }
    Alert.alert(
      `Desativar ${label}?`,
      'Essa ação reduz a proteção do robô durante a corrida.',
      [{ text: 'Cancelar', style: 'cancel' }, { text: 'Desativar', style: 'destructive', onPress: () => send(packet) }],
    );
  };

  const activeAlerts = [
    state.safetyCollisionActive && 'Inclinação',
    state.safetyBatteryBlockActive && 'Bateria',
    state.safetyLineLossActive && 'Linha perdida',
    state.safetyBleLossActive && 'BLE perdido',
  ].filter(Boolean);

  return (
    <Screen>
      <SectionTitle title="Segurança funcional" subtitle="As alterações são salvas no robô. Desativar uma proteção exige confirmação." />
      <View style={styles.metrics}>
        <Metric label="Roll atual" value={state.safetyCurrentRollDeg.toFixed(1)} unit="°" color={Math.abs(state.safetyCurrentRollDeg) >= state.safetyRollLimitDeg ? colors.red : colors.green} />
        <Metric label="Bateria" value={state.safetyCurrentBatteryPercent.toFixed(1)} unit="%" color={state.safetyBatteryBlockActive ? colors.red : colors.green} />
      </View>
      <Card accent={state.safetyMotorsBlocked ? colors.red : colors.green}>
        <View style={styles.pills}>
          <StatusPill label={state.safetyMotorsBlocked ? 'Motores bloqueados' : 'Motores liberados'} tone={state.safetyMotorsBlocked ? 'danger' : 'good'} />
          <StatusPill label={state.safetyBleConnected ? 'BLE presente' : 'BLE ausente'} tone={state.safetyBleConnected ? 'good' : 'danger'} />
          <StatusPill label={state.safetyLineVisible ? 'Linha visível' : 'Sem linha'} tone={state.safetyLineVisible ? 'good' : 'warn'} />
        </View>
        <ValueRow label="Alertas ativos" value={activeAlerts.length ? activeAlerts.join(', ') : 'nenhum'} valueColor={activeAlerts.length ? colors.red : colors.green} />
        <ValueRow label="Tempo sem linha" value={`${state.safetyLineLossElapsedS.toFixed(2)} s`} />
      </Card>

      <SectionTitle title="Parada por inclinação" />
      <Card>
        <ToggleRow
          label="Habilitar parada por roll"
          value={state.safetyCollisionEnabled}
          onValueChange={(value) => guardedToggle('parada por inclinação', value, packSafetyCollisionEnabled(value))}
        />
        <Stepper label="Limite de roll" value={rollLimit} onChange={setRollLimit} min={1} max={90} step={0.5} decimals={1} suffix="°" />
        <Button title="Aplicar limite" onPress={() => send(packSafetyRollLimit(rollLimit))} />
      </Card>

      <SectionTitle title="Bloqueio por bateria" />
      <Card>
        <ToggleRow
          label="Bloquear motores por bateria baixa"
          value={state.safetyBatteryBlockEnabled}
          onValueChange={(value) => guardedToggle('bloqueio por bateria', value, packSafetyBatteryBlockEnabled(value))}
        />
        <Stepper label="Bloquear abaixo de" value={batteryLimit} onChange={setBatteryLimit} min={0} max={100} suffix="%" />
        <Button title="Aplicar limite" onPress={() => send(packSafetyBatteryBlockPercent(batteryLimit))} />
      </Card>

      <SectionTitle title="Perda de linha" />
      <Card>
        <ToggleRow
          label="Parar se perder a linha"
          value={state.safetyLineLossEnabled}
          onValueChange={(value) => guardedToggle('parada por perda de linha', value, packSafetyLineLossEnabled(value))}
        />
        <Stepper label="Tempo sem linha" value={lineTimeout} onChange={setLineTimeout} min={0.1} max={10} step={0.1} decimals={1} suffix="s" />
        <Button title="Aplicar timeout" onPress={() => send(packSafetyLineLossTimeout(lineTimeout))} />
      </Card>

      <SectionTitle title="Perda de BLE" />
      <Card accent={colors.orange}>
        <ToggleRow
          label="Parar se o celular desconectar"
          description="Recomendado para toda corrida controlada pelo aplicativo."
          value={state.safetyBleLossEnabled}
          onValueChange={(value) => guardedToggle('parada por perda de BLE', value, packSafetyBleLossEnabled(value))}
        />
      </Card>
    </Screen>
  );
}

const styles = StyleSheet.create({
  metrics: { flexDirection: 'row', gap: spacing.sm },
  pills: { flexDirection: 'row', flexWrap: 'wrap', gap: spacing.sm },
});
