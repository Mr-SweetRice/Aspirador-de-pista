import type { ReactNode } from 'react';
import {
  Pressable,
  ScrollView,
  StyleSheet,
  Switch,
  Text,
  TextInput,
  View,
  type KeyboardTypeOptions,
  type StyleProp,
  type ViewStyle,
} from 'react-native';

import { colors, radius, spacing } from '../theme';

export function Screen({ children }: { children: ReactNode }) {
  return (
    <ScrollView
      style={styles.screen}
      contentContainerStyle={styles.screenContent}
      keyboardShouldPersistTaps="handled"
      showsVerticalScrollIndicator={false}
    >
      {children}
    </ScrollView>
  );
}

export function Card({
  children,
  accent,
  style,
}: {
  children: ReactNode;
  accent?: string;
  style?: StyleProp<ViewStyle>;
}) {
  return (
    <View style={[styles.card, accent ? { borderLeftColor: accent, borderLeftWidth: 3 } : null, style]}>
      {children}
    </View>
  );
}

export function SectionTitle({ title, subtitle }: { title: string; subtitle?: string }) {
  return (
    <View style={styles.sectionTitleWrap}>
      <Text style={styles.sectionTitle}>{title}</Text>
      {subtitle ? <Text style={styles.sectionSubtitle}>{subtitle}</Text> : null}
    </View>
  );
}

export function ValueRow({
  label,
  value,
  valueColor,
}: {
  label: string;
  value: ReactNode;
  valueColor?: string;
}) {
  return (
    <View style={styles.valueRow}>
      <Text style={styles.valueLabel}>{label}</Text>
      {typeof value === 'string' || typeof value === 'number' ? (
        <Text style={[styles.valueText, valueColor ? { color: valueColor } : null]}>{value}</Text>
      ) : (
        value
      )}
    </View>
  );
}

export function Metric({
  label,
  value,
  unit,
  color = colors.text,
}: {
  label: string;
  value: string | number;
  unit?: string;
  color?: string;
}) {
  return (
    <View style={styles.metric}>
      <Text style={styles.metricLabel}>{label}</Text>
      <Text numberOfLines={1} adjustsFontSizeToFit style={[styles.metricValue, { color }]}>
        {value}
        {unit ? <Text style={styles.metricUnit}> {unit}</Text> : null}
      </Text>
    </View>
  );
}

export function Button({
  title,
  onPress,
  variant = 'secondary',
  disabled = false,
  compact = false,
}: {
  title: string;
  onPress: () => void;
  variant?: 'primary' | 'secondary' | 'danger' | 'ghost';
  disabled?: boolean;
  compact?: boolean;
}) {
  return (
    <Pressable
      accessibilityRole="button"
      accessibilityLabel={title}
      disabled={disabled}
      onPress={onPress}
      style={({ pressed }) => [
        styles.button,
        styles[`button_${variant}`],
        compact ? styles.buttonCompact : null,
        disabled ? styles.buttonDisabled : null,
        pressed && !disabled ? styles.buttonPressed : null,
      ]}
    >
      <Text style={[styles.buttonText, variant === 'primary' ? styles.buttonTextPrimary : null]}>{title}</Text>
    </Pressable>
  );
}

export function ToggleRow({
  label,
  description,
  value,
  onValueChange,
  disabled = false,
  danger = false,
}: {
  label: string;
  description?: string;
  value: boolean;
  onValueChange: (value: boolean) => void;
  disabled?: boolean;
  danger?: boolean;
}) {
  return (
    <View style={styles.toggleRow}>
      <View style={styles.toggleCopy}>
        <Text style={[styles.toggleLabel, danger ? { color: colors.red } : null]}>{label}</Text>
        {description ? <Text style={styles.toggleDescription}>{description}</Text> : null}
      </View>
      <Switch
        accessibilityLabel={label}
        disabled={disabled}
        value={value}
        onValueChange={onValueChange}
        trackColor={{ false: colors.disabled, true: danger ? colors.redDark : '#1F5B12' }}
        thumbColor={value ? (danger ? colors.red : colors.green) : '#777777'}
      />
    </View>
  );
}

export function Stepper({
  label,
  value,
  onChange,
  min,
  max,
  step = 1,
  decimals = 0,
  suffix,
  disabled = false,
}: {
  label: string;
  value: number;
  onChange: (value: number) => void;
  min: number;
  max: number;
  step?: number;
  decimals?: number;
  suffix?: string;
  disabled?: boolean;
}) {
  const clamp = (next: number) => Math.min(max, Math.max(min, Number(next.toFixed(decimals))));
  return (
    <View style={styles.stepperRow}>
      <Text style={styles.stepperLabel}>{label}</Text>
      <View style={styles.stepperControls}>
        <Pressable
          disabled={disabled || value <= min}
          onPress={() => onChange(clamp(value - step))}
          style={({ pressed }) => [styles.stepButton, pressed ? styles.buttonPressed : null]}
        >
          <Text style={styles.stepButtonText}>−</Text>
        </Pressable>
        <View style={styles.stepValueWrap}>
          <Text style={styles.stepValue}>{value.toFixed(decimals)}</Text>
          {suffix ? <Text style={styles.stepSuffix}>{suffix}</Text> : null}
        </View>
        <Pressable
          disabled={disabled || value >= max}
          onPress={() => onChange(clamp(value + step))}
          style={({ pressed }) => [styles.stepButton, pressed ? styles.buttonPressed : null]}
        >
          <Text style={styles.stepButtonText}>+</Text>
        </Pressable>
      </View>
    </View>
  );
}

export function LabeledInput({
  label,
  value,
  onChangeText,
  placeholder,
  secureTextEntry = false,
  keyboardType = 'default',
}: {
  label: string;
  value: string;
  onChangeText: (value: string) => void;
  placeholder?: string;
  secureTextEntry?: boolean;
  keyboardType?: KeyboardTypeOptions;
}) {
  return (
    <View style={styles.inputWrap}>
      <Text style={styles.inputLabel}>{label}</Text>
      <TextInput
        value={value}
        onChangeText={onChangeText}
        placeholder={placeholder}
        placeholderTextColor={colors.textMuted}
        secureTextEntry={secureTextEntry}
        keyboardType={keyboardType}
        autoCapitalize="none"
        autoCorrect={false}
        style={styles.input}
      />
    </View>
  );
}

export function SegmentedControl<T extends string | number>({
  options,
  value,
  onChange,
}: {
  options: readonly { label: string; value: T; color?: string }[];
  value: T;
  onChange: (value: T) => void;
}) {
  return (
    <View style={styles.segmented}>
      {options.map((option) => {
        const selected = option.value === value;
        return (
          <Pressable
            key={String(option.value)}
            onPress={() => onChange(option.value)}
            style={[
              styles.segmentedButton,
              selected ? { backgroundColor: option.color ?? colors.text, borderColor: option.color ?? colors.text } : null,
            ]}
          >
            <Text style={[styles.segmentedText, selected ? styles.segmentedTextSelected : null]}>{option.label}</Text>
          </Pressable>
        );
      })}
    </View>
  );
}

export function StatusPill({
  label,
  tone = 'neutral',
}: {
  label: string;
  tone?: 'good' | 'warn' | 'danger' | 'neutral' | 'blue';
}) {
  const toneStyle = {
    good: { borderColor: colors.green, color: colors.green },
    warn: { borderColor: colors.yellow, color: colors.yellow },
    danger: { borderColor: colors.red, color: colors.red },
    blue: { borderColor: colors.cyan, color: colors.cyan },
    neutral: { borderColor: colors.border, color: colors.textMuted },
  }[tone];
  return (
    <View style={[styles.pill, { borderColor: toneStyle.borderColor }]}>
      <Text style={[styles.pillText, { color: toneStyle.color }]}>{label}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  screen: { flex: 1, backgroundColor: colors.background },
  screenContent: { padding: spacing.md, paddingBottom: 120, gap: spacing.md },
  card: {
    backgroundColor: colors.surface,
    borderColor: colors.border,
    borderWidth: 1,
    borderRadius: radius.md,
    padding: spacing.lg,
    gap: spacing.md,
  },
  sectionTitleWrap: { gap: 3, marginTop: spacing.sm },
  sectionTitle: { color: colors.text, fontSize: 20, fontWeight: '700' },
  sectionSubtitle: { color: colors.textMuted, fontSize: 13, lineHeight: 18 },
  valueRow: { minHeight: 28, flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: spacing.md },
  valueLabel: { color: colors.textMuted, fontSize: 14, flex: 1 },
  valueText: { color: colors.text, fontSize: 14, fontVariant: ['tabular-nums'], textAlign: 'right' },
  metric: { flex: 1, minWidth: 92, backgroundColor: colors.surfaceRaised, borderRadius: radius.sm, padding: spacing.md, gap: 5 },
  metricLabel: { color: colors.textMuted, fontSize: 11, textTransform: 'uppercase', letterSpacing: 0.7 },
  metricValue: { color: colors.text, fontSize: 22, fontWeight: '700', fontVariant: ['tabular-nums'] },
  metricUnit: { color: colors.textMuted, fontSize: 12, fontWeight: '400' },
  button: {
    minHeight: 48,
    borderRadius: radius.md,
    alignItems: 'center',
    justifyContent: 'center',
    paddingHorizontal: spacing.lg,
    borderWidth: 1,
  },
  buttonCompact: { minHeight: 38, paddingHorizontal: spacing.md },
  button_primary: { backgroundColor: colors.green, borderColor: colors.green },
  button_secondary: { backgroundColor: colors.surfaceRaised, borderColor: colors.border },
  button_danger: { backgroundColor: colors.redDark, borderColor: colors.red },
  button_ghost: { backgroundColor: colors.transparent, borderColor: colors.border },
  buttonText: { color: colors.text, fontWeight: '700', fontSize: 15, letterSpacing: 0.2 },
  buttonTextPrimary: { color: '#051400' },
  buttonDisabled: { opacity: 0.35 },
  buttonPressed: { opacity: 0.72, transform: [{ scale: 0.985 }] },
  toggleRow: { flexDirection: 'row', alignItems: 'center', gap: spacing.md, minHeight: 54 },
  toggleCopy: { flex: 1, gap: 3 },
  toggleLabel: { color: colors.text, fontSize: 15, fontWeight: '600' },
  toggleDescription: { color: colors.textMuted, fontSize: 12, lineHeight: 17 },
  stepperRow: { gap: spacing.sm },
  stepperLabel: { color: colors.textMuted, fontSize: 13 },
  stepperControls: { flexDirection: 'row', alignItems: 'center', gap: spacing.sm },
  stepButton: { width: 44, height: 42, borderRadius: radius.sm, borderWidth: 1, borderColor: colors.border, backgroundColor: colors.surfaceRaised, alignItems: 'center', justifyContent: 'center' },
  stepButtonText: { color: colors.text, fontSize: 22, fontWeight: '600' },
  stepValueWrap: { flex: 1, height: 42, flexDirection: 'row', alignItems: 'baseline', justifyContent: 'center', gap: 4, borderRadius: radius.sm, borderWidth: 1, borderColor: colors.border, backgroundColor: colors.background },
  stepValue: { color: colors.text, fontSize: 17, fontWeight: '700', fontVariant: ['tabular-nums'] },
  stepSuffix: { color: colors.textMuted, fontSize: 12 },
  inputWrap: { gap: spacing.xs },
  inputLabel: { color: colors.textMuted, fontSize: 13 },
  input: { color: colors.text, backgroundColor: colors.background, borderWidth: 1, borderColor: colors.border, borderRadius: radius.sm, minHeight: 44, paddingHorizontal: spacing.md, fontSize: 16 },
  segmented: { flexDirection: 'row', flexWrap: 'wrap', gap: spacing.sm },
  segmentedButton: { minHeight: 38, paddingHorizontal: spacing.md, borderRadius: radius.pill, borderWidth: 1, borderColor: colors.border, alignItems: 'center', justifyContent: 'center', backgroundColor: colors.surfaceRaised },
  segmentedText: { color: colors.textMuted, fontSize: 13, fontWeight: '600' },
  segmentedTextSelected: { color: '#000000' },
  pill: { alignSelf: 'flex-start', borderRadius: radius.pill, borderWidth: 1, paddingVertical: 4, paddingHorizontal: 9 },
  pillText: { fontSize: 11, fontWeight: '700', textTransform: 'uppercase', letterSpacing: 0.6 },
});
