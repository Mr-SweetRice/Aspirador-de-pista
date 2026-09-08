import { StyleSheet, Text, View } from 'react-native';

import { useRobot } from '../ble/BleProvider';
import { Card, Metric, Screen, SectionTitle, StatusPill, ValueRow } from '../components/ui';
import { ControlMode, RaceSegmentType, RgbLedMode, TelemetryId } from '../protocol/types';
import { colors, radius, spacing } from '../theme';

const yesNo = (value: boolean) => value ? 'sim' : 'não';
const modeLabel = (value: number) => ({ [ControlMode.Odometry]: 'Odometria', [ControlMode.Line]: 'Linha', [ControlMode.AutoTrack]: 'Auto pista' }[value] ?? String(value));
const segmentLabel = (value: number) => ({ [RaceSegmentType.Straight]: 'Reta', [RaceSegmentType.Curve]: 'Curva', [RaceSegmentType.Stop]: 'Parada', [RaceSegmentType.Intersection]: 'Interseção' }[value] ?? String(value));
const ledLabel = (value: number) => ({ [RgbLedMode.Disabled]: 'Desligado', [RgbLedMode.Battery]: 'Bateria', [RgbLedMode.Manual]: 'Manual', [RgbLedMode.RacePlan]: 'Plano' }[value] ?? String(value));

export function TelemetryScreen() {
  const { robot: s, maps, packetStats, statusMessage } = useRobot();
  return (
    <Screen>
      <SectionTitle title="Telemetria" subtitle="Todos os frames conhecidos são decodificados; a tela atualiza a 20 Hz sem descartar a ingestão BLE." />
      <Card accent={s.authenticated ? colors.green : colors.red}>
        <View style={styles.pills}>
          <StatusPill label={s.connected ? 'BLE conectado' : 'BLE desconectado'} tone={s.connected ? 'good' : 'danger'} />
          <StatusPill label={s.authenticated ? 'Autenticado' : 'Sem autenticação'} tone={s.authenticated ? 'good' : 'warn'} />
          <StatusPill label={`STATUS ${s.lastStatus ?? '–'}`} tone={s.lastStatus === 0 ? 'good' : s.lastStatus == null ? 'neutral' : 'danger'} />
        </View>
        <ValueRow label="Conexão" value={statusMessage} />
        <ValueRow label="Último erro" value={s.lastError ?? 'nenhum'} valueColor={s.lastError ? colors.red : colors.green} />
      </Card>

      <View style={styles.metrics}>
        <Metric label="Bateria" value={s.batteryPercent.toFixed(1)} unit="%" color={s.batteryPercent < 15 ? colors.red : colors.green} />
        <Metric label="Velocidade" value={s.linearMps.toFixed(3)} unit="m/s" />
        <Metric label="Yaw" value={s.yaw.toFixed(1)} unit="°" color={colors.cyan} />
      </View>

      <SectionTitle title="Encoders, bateria e sistema" />
      <Card>
        <ValueRow label="Encoder esquerdo / direito" value={`${s.leftEncoder} / ${s.rightEncoder}`} />
        <ValueRow label="RPM esquerdo / direito" value={`${s.leftRpm.toFixed(2)} / ${s.rightRpm.toFixed(2)}`} />
        <ValueRow label="Velocidade linear" value={`${s.linearMps.toFixed(4)} m/s`} />
        <ValueRow label="Tensão" value={`${s.batteryV.toFixed(3)} V`} />
        <ValueRow label="Bateria" value={`${s.batteryPercent.toFixed(2)} % · ADC ${s.batteryRaw}`} />
        <ValueRow label="CPU core 0 / core 1" value={`${s.cpu0Percent.toFixed(1)} / ${s.cpu1Percent.toFixed(1)} %`} />
        <ValueRow label="Freio em zero" value={yesNo(s.zeroBrakeEnabled)} />
      </Card>

      <SectionTitle title="IMU" />
      <Card>
        <ValueRow label="Roll" value={`${s.roll.toFixed(3)} °`} />
        <ValueRow label="Pitch" value={`${s.pitch.toFixed(3)} °`} />
        <ValueRow label="Yaw" value={`${s.yaw.toFixed(3)} °`} />
      </Card>

      <SectionTitle title="Odometria encoder + IMU" />
      <Card>
        <ValueRow label="Pose operacional X / Y" value={`${s.xM.toFixed(4)} / ${s.yM.toFixed(4)} m`} />
        <ValueRow label="Heading / angular" value={`${s.headingRad.toFixed(4)} rad / ${s.angularRadS.toFixed(4)} rad/s`} />
        <ValueRow label="Encoder X / Y / H" value={`${s.encoderXM.toFixed(4)} / ${s.encoderYM.toFixed(4)} / ${s.encoderHeadingRad.toFixed(4)}`} />
        <ValueRow label="IMU X / Y / H" value={`${s.imuXM.toFixed(4)} / ${s.imuYM.toFixed(4)} / ${s.imuHeadingRad.toFixed(4)}`} />
        <ValueRow label="Fusão X / Y / H" value={`${s.fusedXM.toFixed(4)} / ${s.fusedYM.toFixed(4)} / ${s.fusedHeadingRad.toFixed(4)}`} />
        <ValueRow label="IMU disponível" value={yesNo(s.odometryImuAvailable)} />
      </Card>

      <SectionTitle title="Controlador" />
      <Card>
        <ValueRow label="Rodando / modo" value={`${yesNo(s.controlRunning)} · ${modeLabel(s.controlMode)}`} />
        <ValueRow label="Mapa / target / pontos" value={`${s.controlMapSlot} · ${s.controlTargetIndex} / ${s.controlPointCount}`} />
        <ValueRow label="Velocidade solicitada / ativa" value={`${s.controlSpeedPercent} / ${s.controlActiveSpeedPercent} %`} />
        <ValueRow label="Target X / Y" value={`${s.controlTargetXM.toFixed(4)} / ${s.controlTargetYM.toFixed(4)} m`} />
        <ValueRow label="Distância / erro angular" value={`${s.controlDistanceM.toFixed(4)} m / ${s.controlAngleErrorRad.toFixed(4)} rad`} />
        <ValueRow label="Steer" value={`${s.controlSteerPercent.toFixed(2)} %`} />
        <ValueRow label="PID / alpha" value={`${s.controlKp.toFixed(3)} / ${s.controlKi.toFixed(3)} / ${s.controlKd.toFixed(3)} · ${s.controlLineAlpha.toFixed(3)}`} />
        <ValueRow label="Limite motor" value={`${s.controlMotorLimitPercent} %`} />
        <ValueRow label="Perfil de velocidade" value={yesNo(s.controlSpeedProfileEnabled)} />
        <ValueRow label="Turbina configurada / ativa" value={`${s.controlAuxPercent} / ${s.controlActiveAuxPercent} %`} />
        <ValueRow label="Velocidade média / máxima" value={`${s.controlAverageSpeedMps.toFixed(4)} / ${s.controlMaxSpeedMps.toFixed(4)} m/s`} />
        <ValueRow label="Média final da corrida" value={`${s.controlRacePlanAverageSpeedMps.toFixed(4)} m/s`} />
        <ValueRow label="Compensação de bateria" value={yesNo(s.controlBatteryCompensationEnabled)} />
        <ValueRow label="Pose no mapa válida" value={yesNo(s.controlMapPoseValid)} />
        <ValueRow label="Pose mapa X / Y / H" value={`${s.controlMapXM.toFixed(4)} / ${s.controlMapYM.toFixed(4)} / ${s.controlMapHeadingRad.toFixed(4)}`} />
        <ValueRow label="Segmento ativo / tipo" value={`${yesNo(s.controlRaceSegmentActive)} · ${segmentLabel(s.controlRaceSegmentType)}`} />
        <ValueRow label="Segmento início / fim" value={`${s.controlRaceSegmentStartIndex} / ${s.controlRaceSegmentEndIndex}`} />
        <ValueRow label="Segmento vel / máx / turb" value={`${s.controlRaceSegmentSpeedPercent} / ${s.controlRaceSegmentMaxSpeedPercent} / ${s.controlRaceSegmentAuxPercent} %`} />
      </Card>

      <SectionTitle title="Frequência das tasks" />
      <Card>
        <ValueRow label="control_task" value={`${s.controlLoopHz.toFixed(1)} Hz`} />
        <ValueRow label="race_plan" value={`${s.racePlanLoopHz.toFixed(1)} Hz`} />
        <ValueRow label="track_odometry" value={`${s.trackOdometryLoopHz.toFixed(1)} Hz`} />
        <ValueRow label="line_sensor" value={`${s.lineSensorLoopHz.toFixed(1)} Hz`} />
        <ValueRow label="imu_task" value={`${s.imuLoopHz.toFixed(1)} Hz`} />
      </Card>

      <SectionTitle title="Sensor de linha" />
      <Card>
        <ValueRow label="Raw [1..8]" value={s.lineRaw.join(' · ')} />
        <ValueRow label="Calibrado [1..8]" value={s.lineCalibrated.join(' · ')} />
        <ValueRow label="Linha [1..8]" value={s.lineValues.join(' · ')} />
        <ValueRow label="Posição / tipo" value={`${s.linePosition} / ${s.lineTrackType === 0 ? 'preta' : 'branca'}`} />
        <ValueRow label="Visível / calibrado / calibrando" value={`${yesNo(s.lineVisible)} / ${yesNo(s.lineCalibratedValid)} / ${yesNo(s.lineCalibrating)}`} />
        <ValueRow label="Limiar / filtro / frequência" value={`${s.lineThresholdPercent} / ${s.lineFilterPercent} % / ${s.lineReadHz.toFixed(1)} Hz`} />
      </Card>

      <SectionTitle title="LED" />
      <Card>
        <ValueRow label="Habilitado / modo" value={`${yesNo(s.rgbLedEnabled)} · ${ledLabel(s.rgbLedMode)}`} />
        <ValueRow label="R / G / B / intensidade" value={`${s.rgbLedRed} / ${s.rgbLedGreen} / ${s.rgbLedBlue} / ${s.rgbLedIntensity}`} />
        <View style={[styles.led, { backgroundColor: `rgb(${s.rgbLedRed},${s.rgbLedGreen},${s.rgbLedBlue})`, opacity: s.rgbLedEnabled ? s.rgbLedIntensity / 255 : 0.1 }]} />
      </Card>

      <SectionTitle title="Segurança" />
      <Card>
        <ValueRow label="Inclinação habilitada / ativa" value={`${yesNo(s.safetyCollisionEnabled)} / ${yesNo(s.safetyCollisionActive)}`} />
        <ValueRow label="Bateria habilitada / ativa" value={`${yesNo(s.safetyBatteryBlockEnabled)} / ${yesNo(s.safetyBatteryBlockActive)}`} />
        <ValueRow label="Linha habilitada / ativa" value={`${yesNo(s.safetyLineLossEnabled)} / ${yesNo(s.safetyLineLossActive)}`} />
        <ValueRow label="BLE habilitado / ativo" value={`${yesNo(s.safetyBleLossEnabled)} / ${yesNo(s.safetyBleLossActive)}`} />
        <ValueRow label="Motores bloqueados" value={yesNo(s.safetyMotorsBlocked)} valueColor={s.safetyMotorsBlocked ? colors.red : colors.green} />
        <ValueRow label="Linha / BLE presentes" value={`${yesNo(s.safetyLineVisible)} / ${yesNo(s.safetyBleConnected)}`} />
        <ValueRow label="Limite roll / roll atual" value={`${s.safetyRollLimitDeg.toFixed(2)} / ${s.safetyCurrentRollDeg.toFixed(2)} °`} />
        <ValueRow label="Limite bateria / atual" value={`${s.safetyBatteryBlockPercent.toFixed(2)} / ${s.safetyCurrentBatteryPercent.toFixed(2)} %`} />
        <ValueRow label="Timeout / tempo sem linha" value={`${s.safetyLineLossTimeoutS.toFixed(2)} / ${s.safetyLineLossElapsedS.toFixed(2)} s`} />
      </Card>

      <SectionTitle title="Transporte BLE e mapas" />
      <Card>
        <ValueRow label="Pacotes recebidos" value={packetStats.total} />
        <ValueRow label="Erros de parse" value={packetStats.parseErrors} valueColor={packetStats.parseErrors ? colors.red : colors.green} />
        <ValueRow label="Último pacote" value={packetStats.lastPacketAt ? new Date(packetStats.lastPacketAt).toLocaleTimeString() : '–'} />
        {Object.entries(packetStats.messageCounts).sort(([a], [b]) => Number(a) - Number(b)).map(([id, count]) => (
          <ValueRow key={id} label={`0x${Number(id).toString(16).padStart(2, '0')} ${TelemetryId[Number(id)] ?? 'desconhecido'}`} value={count} />
        ))}
        {maps.map((map) => <ValueRow key={map.slot} label={`Mapa ${map.slot}: ${map.name}`} value={`${map.pointCount} pts · ${map.distanceM.toFixed(3)} m`} />)}
      </Card>
    </Screen>
  );
}

const styles = StyleSheet.create({
  pills: { flexDirection: 'row', flexWrap: 'wrap', gap: spacing.sm },
  metrics: { flexDirection: 'row', gap: spacing.sm },
  led: { height: 30, borderRadius: radius.sm, borderColor: colors.border, borderWidth: 1 },
});
