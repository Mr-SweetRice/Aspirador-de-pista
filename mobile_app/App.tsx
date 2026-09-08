import { useKeepAwake } from 'expo-keep-awake';
import { StatusBar } from 'expo-status-bar';
import { useState } from 'react';
import { Pressable, StyleSheet, Text, View } from 'react-native';
import { SafeAreaProvider, SafeAreaView } from 'react-native-safe-area-context';

import { BleProvider, useRobot } from './src/ble/BleProvider';
import { HomeScreen } from './src/screens/HomeScreen';
import { ControlScreen } from './src/screens/ControlScreen';
import { SafetyScreen } from './src/screens/SafetyScreen';
import { SensorsScreen } from './src/screens/SensorsScreen';
import { TelemetryScreen } from './src/screens/TelemetryScreen';
import { colors, radius, spacing } from './src/theme';

type Tab = 'home' | 'telemetry' | 'control' | 'sensors' | 'safety';

const TABS: { key: Tab; label: string; icon: string }[] = [
  { key: 'home', label: 'Início', icon: '▶' },
  { key: 'telemetry', label: 'Telemetria', icon: '⌁' },
  { key: 'control', label: 'Controle', icon: '✥' },
  { key: 'sensors', label: 'Sensores', icon: '◉' },
  { key: 'safety', label: 'Segurança', icon: '◆' },
];

function MobileApp() {
  useKeepAwake('aspirador-corrida');
  const ble = useRobot();
  const [tab, setTab] = useState<Tab>('home');

  const screen = {
    home: <HomeScreen />,
    telemetry: <TelemetryScreen />,
    control: <ControlScreen />,
    sensors: <SensorsScreen />,
    safety: <SafetyScreen />,
  }[tab];

  return (
    <SafeAreaView style={styles.safe} edges={['top', 'left', 'right']}>
      <StatusBar style="light" />
      <View style={styles.header}>
        <View style={styles.brand}>
          <View style={[styles.dot, { backgroundColor: ble.robot.authenticated ? colors.green : ble.connectionStatus === 'connecting' ? colors.yellow : colors.red }]} />
          <View style={styles.brandCopy}>
            <Text style={styles.title}>ASPIRADOR DE PISTA</Text>
            <Text numberOfLines={1} style={styles.subtitle}>{ble.connectedDeviceName ?? 'robô desconectado'}</Text>
          </View>
        </View>
        <Pressable
          accessibilityRole="button"
          accessibilityLabel="STOP de emergência"
          disabled={!ble.robot.authenticated}
          onPress={ble.emergencyStop}
          style={({ pressed }) => [styles.stop, !ble.robot.authenticated ? styles.stopDisabled : null, pressed ? styles.pressed : null]}
        >
          <Text style={styles.stopText}>STOP</Text>
        </Pressable>
      </View>

      <View style={styles.content}>{screen}</View>

      <View style={styles.tabBar}>
        {TABS.map((item) => {
          const active = item.key === tab;
          return (
            <Pressable key={item.key} onPress={() => setTab(item.key)} style={styles.tab} accessibilityRole="tab" accessibilityState={{ selected: active }}>
              <Text style={[styles.tabIcon, active ? styles.tabActive : null]}>{item.icon}</Text>
              <Text numberOfLines={1} style={[styles.tabLabel, active ? styles.tabActive : null]}>{item.label}</Text>
            </Pressable>
          );
        })}
      </View>
    </SafeAreaView>
  );
}

export default function App() {
  return (
    <SafeAreaProvider>
      <BleProvider>
        <MobileApp />
      </BleProvider>
    </SafeAreaProvider>
  );
}

const styles = StyleSheet.create({
  safe: { flex: 1, backgroundColor: colors.background },
  header: { minHeight: 62, paddingHorizontal: spacing.md, paddingVertical: spacing.sm, borderBottomWidth: 1, borderBottomColor: colors.border, backgroundColor: colors.background, flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: spacing.md },
  brand: { flex: 1, flexDirection: 'row', alignItems: 'center', gap: spacing.sm },
  brandCopy: { flex: 1 },
  dot: { width: 9, height: 9, borderRadius: 5 },
  title: { color: colors.text, fontSize: 13, fontWeight: '800', letterSpacing: 1.1 },
  subtitle: { color: colors.textMuted, fontSize: 10, marginTop: 2 },
  stop: { backgroundColor: colors.redDark, borderColor: colors.red, borderWidth: 1, borderRadius: radius.sm, minWidth: 78, minHeight: 42, alignItems: 'center', justifyContent: 'center' },
  stopDisabled: { opacity: 0.3 },
  stopText: { color: colors.red, fontSize: 15, fontWeight: '900', letterSpacing: 1 },
  pressed: { opacity: 0.65 },
  content: { flex: 1, backgroundColor: colors.background },
  tabBar: { height: 66, paddingBottom: 3, borderTopWidth: 1, borderTopColor: colors.border, backgroundColor: colors.background, flexDirection: 'row' },
  tab: { flex: 1, alignItems: 'center', justifyContent: 'center', gap: 3, paddingHorizontal: 2 },
  tabIcon: { color: colors.textMuted, fontSize: 18 },
  tabLabel: { color: colors.textMuted, fontSize: 9, fontWeight: '600' },
  tabActive: { color: colors.green },
});
