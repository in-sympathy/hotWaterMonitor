/*
  Nano Matter – HeatPipeTemp (DS18B20-only) + Battery monitor + LED alerts
  Update: quick charge poll every 6 s using short slope window (~36 s).

  Calibrations (from your measurements):
    • ADC_VREF = 3.251 V
    • CAL_GAIN = 1.031896 (from Vbat=3.494 V, A0=1.693 V)

  Cadence:
    • Every 3 minutes: read DS18B20 + battery, publish to Matter (delta or force), print detailed block.
    • Every 6 seconds: quick battery-only poll → fast charging/low/full detection & background LEDs.

  LED behavior:
    • 5 s after each temperature reading:
        - HeatPipeTempC <= 28.0 °C → smooth BLUE triple-blink (300/300 ×3, 1.0 s gap).
        - HeatPipeTempC  > 28.0 °C → solid RED.
    • Outside that window (background):
        - Charging & Full (>= 4.18 V): GREEN slow blink (250 ms ON / 1750 ms OFF).
        - Charging (not full): PURPLE slow blink (1 s ON / 1 s OFF).
        - Low/Crit (<= 3.60 / 3.45 V): ORANGE quick triple-blink every 10 s.

  Button:
    • Short press → pairing info to Serial.
    • Long press (>= 6 s) → Serial note + WHITE ×6, Matter.decommission().

  Battery sense wiring (1S pack AFTER BMS, BEFORE booster):
      BAT+ ─ R1=1.0MΩ ── A0 ── R2=1.0MΩ ─ BAT−   (optional 100 nF A0→GND)
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Matter.h>
#include <math.h>

#if !defined(__has_include)
  #define __has_include(x) 0
#endif
#if __has_include(<MatterTemperature.h>)
  #include <MatterTemperature.h>
  #define HAS_MATTER_TEMP 1
#endif

#ifndef BTN_BUILTIN
  #define BTN_BUILTIN 7
#endif

// -------------------- Pins --------------------
const uint8_t DS18B20_PIN   = 5;    // D5 on Nano Matter
const uint8_t BAT_SENSE_PIN = A0;   // Analog sense via divider

// Prefer explicit RGB LEDs (Nano Matter typically has active-LOW RGB)
#if (defined(LED_BLUE) && defined(LED_RED) && defined(LED_GREEN))
  #define BLUE_LED_PIN  LED_BLUE
  #define RED_LED_PIN   LED_RED
  #define GREEN_LED_PIN LED_GREEN
  #ifndef BLUE_LED_ACTIVE_HIGH
    #define BLUE_LED_ACTIVE_HIGH 0
  #endif
  #ifndef RED_LED_ACTIVE_HIGH
    #define RED_LED_ACTIVE_HIGH  0
  #endif
  #ifndef GREEN_LED_ACTIVE_HIGH
    #define GREEN_LED_ACTIVE_HIGH 0
  #endif
  #define HAVE_RGB_3 1
#elif (defined(LEDB) && defined(LEDR) && defined(LEDG))
  #define BLUE_LED_PIN  LEDB
  #define RED_LED_PIN   LEDR
  #define GREEN_LED_PIN LEDG
  #ifndef BLUE_LED_ACTIVE_HIGH
    #define BLUE_LED_ACTIVE_HIGH 0
  #endif
  #ifndef RED_LED_ACTIVE_HIGH
    #define RED_LED_ACTIVE_HIGH  0
  #endif
  #ifndef GREEN_LED_ACTIVE_HIGH
    #define GREEN_LED_ACTIVE_HIGH 0
  #endif
  #define HAVE_RGB_3 1
#elif (defined(LED_BLUE) && defined(LED_RED))
  #define BLUE_LED_PIN  LED_BLUE
  #define RED_LED_PIN   LED_RED
  #ifndef BLUE_LED_ACTIVE_HIGH
    #define BLUE_LED_ACTIVE_HIGH 0
  #endif
  #ifndef RED_LED_ACTIVE_HIGH
    #define RED_LED_ACTIVE_HIGH  0
  #endif
  #define HAVE_RGB_2 1
#elif (defined(LEDB) && defined(LEDR))
  #define BLUE_LED_PIN  LEDB
  #define RED_LED_PIN   LEDR
  #ifndef BLUE_LED_ACTIVE_HIGH
    #define BLUE_LED_ACTIVE_HIGH 0
  #endif
  #ifndef RED_LED_ACTIVE_HIGH
    #define RED_LED_ACTIVE_HIGH  0
  #endif
  #define HAVE_RGB_2 1
#elif defined(LED_BUILTIN) // Fallback: single LED
  #define SINGLE_LED_PIN LED_BUILTIN
  #ifndef SINGLE_LED_ACTIVE_HIGH
    #define SINGLE_LED_ACTIVE_HIGH 0
  #endif
  #define HAVE_SINGLE_LED 1
#else
  #error "No LED pins detected. Define LED pins or LED_BUILTIN."
#endif

// -------------------- Behavior Config --------------------
const float    TEMP_THRESHOLD_C          = 28.0f;
const uint32_t MEASUREMENT_INTERVAL_MS   = 3UL * 60UL * 1000UL;   // every 3 minutes
const uint32_t INDICATE_DURATION_MS      = 5000;                  // temp LED window
const float    DELTA_REPORT_C            = 0.2f;
const uint32_t MAX_REPORT_INTERVAL_MS    = MEASUREMENT_INTERVAL_MS; // force each scheduled read

// Temp window blink: 300ms ON / 300ms OFF ×3, then 1.0 s gap
const uint16_t BLINK_ON_MS               = 300;
const uint16_t BLINK_OFF_MS              = 300;
const uint8_t  BLINK_COUNT               = 3;
const uint16_t BURST_GAP_MS              = 1000;

// Decommission confirm (WHITE ×6)
const uint16_t QUICK_BLINK_ON_MS         = 120;
const uint16_t QUICK_BLINK_OFF_MS        = 120;
const uint8_t  QUICK_BLINK_COUNT         = 6;

// DS18B20 conversion wait (9-bit ≈ 94 ms; use 100 ms)
const uint16_t DS18B20_CONV_MS           = 100;

// -------------------- Battery sensing (1S pack) --------------------
// Divider: BAT+ → R1 → A0 → R2 → BAT−, A0 = Vbat * R2 / (R1+R2)
const float BAT_R1_OHMS = 1000000.0f; // 1.0 MΩ
const float BAT_R2_OHMS = 1000000.0f; // 1.0 MΩ  (½ scaling)
const float DIV_GAIN    = (BAT_R1_OHMS + BAT_R2_OHMS) / BAT_R2_OHMS; // = 2.0
const float ADC_VREF    = 3.251f;     // your measured 3V3
const float CAL_GAIN    = 1.031896f;  // from your Vbat/A0 readings
const int   ADC_BITS    = 12;         // Silabs ADC is 12-bit
const int   BAT_SAMPLES = 8;          // oversampling

// Thresholds & charging detection
const float BAT_LOW_V   = 3.60f;      // warn
const float BAT_CRIT_V  = 3.45f;      // critical
const float BAT_FULL_V  = 4.18f;      // consider "full" near CV taper

// Long-window charging heuristic: Vbat rise ≥ 15 mV over ~3 min
const float    CHG_RISE_V_LONG   = 0.015f;
const uint32_t CHG_WINDOW_MS     = 3UL * 60UL * 1000UL;

// NEW: quick charge poll every 6 s with short slope window (~36 s)
const uint32_t CHG_POLL_INTERVAL_MS = 6000;
const uint8_t  CHG_BUF_N            = 6;       // 6 samples * 6 s ≈ 36 s
const float    CHG_RISE_V_SHORT     = 0.020f;  // 20 mV over ~36 s (filters noise)

// Low-battery periodic alert: ORANGE quick triple-blink every 10 s
const uint32_t LOW_ALERT_PERIOD_MS = 10000;
const uint16_t LOW_BLINK_ON_MS     = 120;
const uint16_t LOW_BLINK_OFF_MS    = 120;

// Charging blink: PURPLE slow (1 s on / 1 s off)
const uint16_t CHG_BLINK_ON_MS     = 1000;
const uint16_t CHG_BLINK_OFF_MS    = 1000;

// Fully-charged blink: GREEN slow so temp window can still show
const uint16_t FULL_BLINK_ON_MS    = 250;
const uint16_t FULL_BLINK_OFF_MS   = 1750;

// -------------------- DS18B20 --------------------
OneWire oneWire(DS18B20_PIN);
DallasTemperature HeatPipeTemp(&oneWire);  // required name
float HeatPipeTempC = NAN;

// -------------------- Matter --------------------
#ifdef HAS_MATTER_TEMP
MatterTemperature matter_heatPipeTemp;
#endif

// -------------------- State / timers --------------------
uint32_t lastMeasurementMs   = 0;
uint32_t indicateUntilMs     = 0;

float    lastPublishedC      = NAN;
uint32_t lastPublishedMs     = 0;

// Battery state
float    lastVbat            = NAN;
float    prevVbatForCharge   = NAN;    // long window anchor
uint32_t prevVbatTimeMs      = 0;
bool     battLow             = false;
bool     battCrit            = false;
bool     battFull            = false;

enum ChargeState { CHG_UNKNOWN, CHG_NO, CHG_YES };
ChargeState chargeState = CHG_UNKNOWN;

// Quick charge poll ring buffer
float    chgBufV[CHG_BUF_N]  = {0};
uint32_t chgBufT[CHG_BUF_N]  = {0};
uint8_t  chgBufCount         = 0;
uint8_t  chgBufHead          = 0;
uint32_t nextChgPollMs       = 0;

// Button
bool     btnLast   = true;   // active LOW
uint32_t btnDownMs = 0;

// Pairing/Thread cache
String pairingPIN;
bool pairedCached     = false;
bool threadConnCached = false;

// Temp window blinker state
enum BlinkState { BLINK_IDLE, BLINK_ON, BLINK_OFF, BLINK_GAP };
BlinkState blinkState = BLINK_IDLE;
uint8_t    blinkIndex = 0;
uint32_t   nextBlinkTransitionMs = 0;

// Background alert blinkers
uint32_t lowAlertCycleStartMs = 0;
uint8_t  lowAlertPulseIndex   = 0;
bool     lowAlertActivePulse  = false;
uint32_t lowAlertNextEdgeMs   = 0;

bool     chgBlinkOn           = false;
uint32_t chgBlinkNextEdgeMs   = 0;

bool     fullBlinkOn          = false;
uint32_t fullBlinkNextEdgeMs  = 0;

// -------------------- LED helpers --------------------
inline void setBlueLed(bool on) {
#if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
  #if BLUE_LED_ACTIVE_HIGH
    digitalWrite(BLUE_LED_PIN, on ? HIGH : LOW);
  #else
    digitalWrite(BLUE_LED_PIN, on ? LOW : HIGH);
  #endif
#else
  (void)on;
#endif
}
inline void setRedLed(bool on) {
#if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
  #if RED_LED_ACTIVE_HIGH
    digitalWrite(RED_LED_PIN, on ? HIGH : LOW);
  #else
    digitalWrite(RED_LED_PIN, on ? LOW : HIGH);
  #endif
#else
  (void)on;
#endif
}
inline void setGreenLed(bool on) {
#if defined(HAVE_RGB_3)
  #if GREEN_LED_ACTIVE_HIGH
    digitalWrite(GREEN_LED_PIN, on ? HIGH : LOW);
  #else
    digitalWrite(GREEN_LED_PIN, on ? LOW : HIGH);
  #endif
#else
  (void)on;
#endif
}
inline void setSingleLed(bool on) {
#if defined(HAVE_SINGLE_LED)
  #if SINGLE_LED_ACTIVE_HIGH
    digitalWrite(SINGLE_LED_PIN, on ? HIGH : LOW);
  #else
    digitalWrite(SINGLE_LED_PIN, on ? LOW : HIGH);
  #endif
#else
  (void)on;
#endif
}
inline void ledsOff() {
#if defined(HAVE_RGB_3)
  setRedLed(false); setGreenLed(false); setBlueLed(false);
#elif defined(HAVE_RGB_2)
  setRedLed(false); setBlueLed(false);
#elif defined(HAVE_SINGLE_LED)
  setSingleLed(false);
#endif
}
inline void setWhiteLed(bool on) {
#if defined(HAVE_RGB_3)
  setRedLed(on); setGreenLed(on); setBlueLed(on);
#elif defined(HAVE_RGB_2)
  setRedLed(on); setBlueLed(on); // pseudo-white
#elif defined(HAVE_SINGLE_LED)
  setSingleLed(on);
#endif
}
inline void setOrangeLed(bool on) { // R+G (fallback to RED)
#if defined(HAVE_RGB_3)
  setRedLed(on); setGreenLed(on); setBlueLed(false);
#elif defined(HAVE_RGB_2)
  setRedLed(on); setBlueLed(false);
#elif defined(HAVE_SINGLE_LED)
  setSingleLed(on);
#endif
}
inline void setPurpleLed(bool on) { // R+B (fallback to RED)
#if defined(HAVE_RGB_3)
  setRedLed(on); setBlueLed(on); setGreenLed(false);
#elif defined(HAVE_RGB_2)
  setRedLed(on); setBlueLed(on);
#elif defined(HAVE_SINGLE_LED)
  setSingleLed(on);
#endif
}
inline void setGreenOnly(bool on) { // convenience
#if defined(HAVE_RGB_3)
  setRedLed(false); setBlueLed(false); setGreenLed(on);
#elif defined(HAVE_RGB_2)
  setRedLed(false); setBlueLed(false); // no green available
#elif defined(HAVE_SINGLE_LED)
  setSingleLed(on);
#endif
}

// -------------------- % estimation from voltage --------------------
uint8_t estimatePercentFromVoltage(float v) {
  const float   V[] = {4.20,4.10,4.00,3.90,3.80,3.75,3.70,3.65,3.55,3.45,3.30};
  const uint8_t P[] = {100,  90,  80,  70,  60,  50,  40,  30,  20,  10,   0};
  if (v >= V[0]) return 100;
  if (v <= V[10]) return 0;
  for (int i = 0; i < 10; ++i) {
    if (v <= V[i] && v >= V[i+1]) {
      float t = (v - V[i+1]) / (V[i] - V[i+1]); // 0..1
      float p = P[i+1] + t * (P[i] - P[i+1]);
      if (p < 0) p = 0; if (p > 100) p = 100;
      return (uint8_t)(p + 0.5f);
    }
  }
  return 0;
}

// -------------------- Pairing / Buttons --------------------
void printPairingInfo() {
  pairingPIN = Matter.getManualPairingCode();
  Serial.println(F("=== Matter Pairing Info ==="));
  Serial.print(F("Commissioned: ")); Serial.println(Matter.isDeviceCommissioned() ? F("yes") : F("no"));
  Serial.print(F("Thread link:  ")); Serial.println(Matter.isDeviceThreadConnected() ? F("connected") : F("not connected"));
  Serial.print(F("Manual code:  ")); Serial.println(pairingPIN);
  Serial.println(F("==========================="));
}
void printDecommissionNotice() {
  Serial.println(F("[Matter] Decommission requested (long press ≥ 6 s)."));
  Serial.println(F("Blinking WHITE 6× fast, then decommissioning..."));
}
void doQuickWhiteBlinkConfirm() {
  for (uint8_t i = 0; i < QUICK_BLINK_COUNT; ++i) {
    setWhiteLed(true);  delay(QUICK_BLINK_ON_MS);
    setWhiteLed(false); delay(QUICK_BLINK_OFF_MS);
  }
}
void handleButton() {
  static bool longPressTriggered = false;
  bool now = (digitalRead(BTN_BUILTIN) == LOW); // active LOW
  uint32_t ms = millis();
  if (now && !btnLast) { btnDownMs = ms; longPressTriggered = false; }
  else if (now && btnLast) {
    if (!longPressTriggered && (ms - btnDownMs) >= 6000) {
      longPressTriggered = true;
      printDecommissionNotice();
      doQuickWhiteBlinkConfirm();
      Matter.decommission(); // will reboot
    }
  } else if (!now && btnLast) {
    if (!longPressTriggered) printPairingInfo(); // short press
  }
  btnLast = now;
}
void logPairingStateIfChanged() {
  bool nowPaired = Matter.isDeviceCommissioned();
  bool nowThread = Matter.isDeviceThreadConnected();
  if (nowPaired != pairedCached || nowThread != threadConnCached) {
    Serial.print(F("[Matter] Commissioned: "));
    Serial.print(nowPaired ? F("yes") : F("no"));
    Serial.print(F(" | Thread: "));
    Serial.println(nowThread ? F("connected") : F("not connected"));
    pairedCached = nowPaired; threadConnCached = nowThread;
    printPairingInfo();
  }
}

// -------------------- Battery sensing --------------------
float readBatteryVolts(float *vA0_out = nullptr, uint16_t *raw_out = nullptr) {
#if defined(analogReadResolution)
  analogReadResolution(ADC_BITS);
#endif
  (void)analogRead(BAT_SENSE_PIN); // settle S/H
  delayMicroseconds(300);

  uint32_t sum = 0;
  for (int i = 0; i < BAT_SAMPLES; ++i) {
    sum += analogRead(BAT_SENSE_PIN);
    delayMicroseconds(200);
  }
  float avgCounts = (float)sum / (float)BAT_SAMPLES;
  float maxCounts = (float)((1UL << ADC_BITS) - 1UL);
  float vA0 = (avgCounts / maxCounts) * ADC_VREF;
  float vBat = (vA0 * DIV_GAIN) * CAL_GAIN;  // apply calibration

  if (vA0_out)  *vA0_out = vA0;
  if (raw_out)  *raw_out = (uint16_t)(avgCounts + 0.5f);

  return vBat;
}

// Long-window update (called during full 3-min cycle; prints detailed block)
void updateBatteryStateLong(float vbat, float vA0, uint16_t raw, uint32_t ms) {
  bool wasLow  = battLow;
  bool wasCrit = battCrit;
  bool wasFull = battFull;
  ChargeState prevChg = chargeState;

  battCrit = (vbat <= BAT_CRIT_V);
  battLow  = (vbat <= BAT_LOW_V);
  battFull = (vbat >= BAT_FULL_V);

  // Charging heuristic (rise over long window)
  if (prevVbatTimeMs == 0 || (ms - prevVbatTimeMs) >= CHG_WINDOW_MS) {
    if (!isnan(prevVbatForCharge)) {
      float dv = vbat - prevVbatForCharge;
      if (dv >= CHG_RISE_V_LONG)       chargeState = CHG_YES;
      else if (dv <= -CHG_RISE_V_LONG) chargeState = CHG_NO;
      else                              chargeState = CHG_UNKNOWN;
    }
    prevVbatForCharge = vbat;
    prevVbatTimeMs    = ms;
  }

  // Serial reporting (detailed)
  uint8_t pct = estimatePercentFromVoltage(vbat);
  Serial.println(F("==========================="));
  Serial.println(F("--- Sensor reading ---"));
  Serial.print  (F("Temperature (HeatPipeTemp / DS18B20): "));
  if (isnan(HeatPipeTempC)) Serial.println(F("--.- C"));
  else { Serial.print(HeatPipeTempC, 2); Serial.println(F(" C")); }

  Serial.print(F("Battery: raw=")); Serial.print(raw);
  Serial.print(F("  V(A0)=")); Serial.print(vA0, 3);
  Serial.print(F(" V  -> Vbat≈")); Serial.print(vbat, 3);
  Serial.print(F(" V (≈")); Serial.print(pct); Serial.print(F("%)  State="));
  if (battCrit) Serial.print(F("CRIT"));
  else if (battLow) Serial.print(F("LOW"));
  else Serial.print(F("OK"));
  Serial.print(F(", Charging="));
  switch (chargeState) { case CHG_YES: Serial.print(F("YES")); break; case CHG_NO: Serial.print(F("NO")); break; default: Serial.print(F("UNKNOWN")); break; }
  if (battFull) Serial.print(F(", FULL"));
  Serial.println();

  if ((!wasCrit && battCrit) || (!wasLow && battLow)) {
    Serial.println(F("[Power] Battery low — charge needed."));
  }
  if (prevChg != chargeState) {
    if (chargeState == CHG_YES) Serial.println(F("[Power] Charging detected (long window)."));
    else if (chargeState == CHG_NO) Serial.println(F("[Power] Discharging (long window)."));
  }
  if (!wasFull && battFull) {
    Serial.println(F("[Power] Battery appears full (>= 4.18 V)."));
  }
}

// Quick poll helper — updates charge/low/full often, minimal serial
void quickChargePoll(uint32_t ms) {
  float v = readBatteryVolts(nullptr, nullptr);

  // Update low/full from quick reading (for responsive LEDs)
  bool wasLow  = battLow;
  bool wasCrit = battCrit;
  bool wasFull = battFull;
  ChargeState prevChg = chargeState;

  battCrit = (v <= BAT_CRIT_V);
  battLow  = (v <= BAT_LOW_V);
  battFull = (v >= BAT_FULL_V);

  // Update short-window slope buffer
  if (chgBufCount < CHG_BUF_N) {
    chgBufV[chgBufHead] = v; chgBufT[chgBufHead] = ms;
    chgBufHead = (chgBufHead + 1) % CHG_BUF_N;
    chgBufCount++;
  } else {
    // Overwrite oldest (head points to next slot; oldest is at head)
    uint8_t oldest = chgBufHead;
    float vold = chgBufV[oldest]; uint32_t told = chgBufT[oldest];
    chgBufV[oldest] = v; chgBufT[oldest] = ms;
    chgBufHead = (chgBufHead + 1) % CHG_BUF_N;

    uint32_t dt = ms - told;
    if (dt >= 20000UL) { // need at least ~20 s baseline
      float dv = v - vold;
      if (dv >= CHG_RISE_V_SHORT)       chargeState = CHG_YES;
      else if (dv <= -CHG_RISE_V_SHORT) chargeState = CHG_NO;
      // else keep prior chargeState (no change)
    }
  }

  // Minimal console notes on state changes (so it’s obvious but not spammy)
  if ((!wasCrit && battCrit) || (!wasLow && battLow)) {
    Serial.println(F("[Power] Battery low — charge needed (quick poll)."));
  }
  if (!wasFull && battFull) {
    Serial.println(F("[Power] Battery appears full (quick poll)."));
  }
  if (prevChg != chargeState) {
    if (chargeState == CHG_YES) Serial.println(F("[Power] Charging detected (quick poll)."));
    else if (chargeState == CHG_NO) Serial.println(F("[Power] Discharging (quick poll)."));
  }
}

// -------------------- Background battery/charging/charged blinkers --------------------
void updateLowBatteryBlinker(uint32_t ms) {
  if (!(battLow || battCrit)) {
    lowAlertPulseIndex  = 0;
    lowAlertActivePulse = false;
    lowAlertCycleStartMs = ms + LOW_ALERT_PERIOD_MS;
    setOrangeLed(false);
    return;
  }
  if (lowAlertCycleStartMs == 0) lowAlertCycleStartMs = ms;
  if (ms < lowAlertCycleStartMs) { setOrangeLed(false); return; }

  if (lowAlertPulseIndex < 3) {
    if (!lowAlertActivePulse) {
      setOrangeLed(true);
      lowAlertActivePulse = true;
      lowAlertNextEdgeMs = ms + LOW_BLINK_ON_MS;
    } else if ((int32_t)(ms - lowAlertNextEdgeMs) >= 0) {
      setOrangeLed(false);
      lowAlertActivePulse = false;
      lowAlertNextEdgeMs = ms + LOW_BLINK_OFF_MS;
      lowAlertPulseIndex++;
    }
  } else {
    if ((int32_t)(ms - lowAlertNextEdgeMs) >= 0) {
      lowAlertCycleStartMs += LOW_ALERT_PERIOD_MS;
      lowAlertPulseIndex = 0;
      lowAlertActivePulse = false;
    }
  }
}
void updateChargingBlinker(uint32_t ms) {
  if (chargeState != CHG_YES || battFull) {
    chgBlinkOn = false; setPurpleLed(false);
    return;
  }
  if ((int32_t)(ms - chgBlinkNextEdgeMs) >= 0) {
    chgBlinkOn = !chgBlinkOn;
    setPurpleLed(chgBlinkOn);
    chgBlinkNextEdgeMs = ms + (chgBlinkOn ? CHG_BLINK_ON_MS : CHG_BLINK_OFF_MS);
  }
}
void updateFullBlinker(uint32_t ms) {
  if (!(chargeState == CHG_YES && battFull)) {
    fullBlinkOn = false; setGreenOnly(false);
    return;
  }
  if ((int32_t)(ms - fullBlinkNextEdgeMs) >= 0) {
    fullBlinkOn = !fullBlinkOn;
    setGreenOnly(fullBlinkOn);
    fullBlinkNextEdgeMs = ms + (fullBlinkOn ? FULL_BLINK_ON_MS : FULL_BLINK_OFF_MS);
  }
}

// -------------------- Temperature publish --------------------
#ifdef HAS_MATTER_TEMP
void publishIfNeeded() {
  bool mustRefresh = (millis() - lastPublishedMs) >= MAX_REPORT_INTERVAL_MS;
  bool bigDelta    = isnan(lastPublishedC) || isnan(HeatPipeTempC)
                     ? true
                     : (fabsf(HeatPipeTempC - lastPublishedC) >= DELTA_REPORT_C);

  if (!isnan(HeatPipeTempC) && (bigDelta || mustRefresh)) {
    matter_heatPipeTemp.set_measured_value_celsius(HeatPipeTempC);
    lastPublishedC  = HeatPipeTempC;
    lastPublishedMs = millis();

    Serial.print(F("[Matter] Published temperature: "));
    Serial.print(HeatPipeTempC, 2);
    Serial.println(F(" °C"));
  } else {
    Serial.println(F("[Matter] Skipped publish (delta < threshold)."));
  }
}
#endif

// -------------------- Sensor reading --------------------
void readSensorOnce() {
  HeatPipeTemp.requestTemperatures();
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < DS18B20_CONV_MS) delay(2);
  float t = HeatPipeTemp.getTempCByIndex(0);
  if (t > -80 && t < 150) HeatPipeTempC = t;

  // Battery (full read + detailed serial + long-window charge)
  float vA0 = NAN; uint16_t raw = 0;
  lastVbat = readBatteryVolts(&vA0, &raw);
  updateBatteryStateLong(lastVbat, vA0, raw, millis());
}

// Temp window blinker (5 s after reading)
enum TWState { TW_IDLE, TW_ON, TW_OFF, TW_GAP };
TWState   twState = TW_IDLE;
uint8_t   twIndex = 0;
uint32_t  twNext  = 0;

void updateTempWindowBlinker(uint32_t ms) {
  if ((int32_t)(ms - indicateUntilMs) > 0) return; // outside temp window

  // Override background while active
  if (!isnan(HeatPipeTempC) && HeatPipeTempC > TEMP_THRESHOLD_C) {
    ledsOff();
#if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
    setRedLed(true);
#elif defined(HAVE_SINGLE_LED)
    setSingleLed(true);
#endif
    twState = TW_IDLE;
    return;
  }

  switch (twState) {
    case TW_IDLE:
      ledsOff();
#if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
      setBlueLed(true);
#elif defined(HAVE_SINGLE_LED)
      setSingleLed(true);
#endif
      twIndex = 0; twState = TW_ON; twNext = ms + BLINK_ON_MS;
      break;

    case TW_ON:
      if ((int32_t)(ms - twNext) >= 0) {
#if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
        setBlueLed(false);
#elif defined(HAVE_SINGLE_LED)
        setSingleLed(false);
#endif
        twState = TW_OFF; twNext = ms + BLINK_OFF_MS;
      }
      break;

    case TW_OFF:
      if ((int32_t)(ms - twNext) >= 0) {
        twIndex++;
        if (twIndex < BLINK_COUNT) {
#if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
          setBlueLed(true);
#elif defined(HAVE_SINGLE_LED)
          setSingleLed(true);
#endif
          twState = TW_ON; twNext = ms + BLINK_ON_MS;
        } else {
          twState = TW_GAP; twNext = ms + BURST_GAP_MS;
        }
      }
      break;

    case TW_GAP:
      if ((int32_t)(ms - twNext) >= 0) { twState = TW_IDLE; }
      break;
  }
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println();
  Serial.println(F("Nano Matter – HeatPipeTemp | Battery monitor + LED alerts (quick charge poll)"));

  pinMode(BTN_BUILTIN, INPUT_PULLUP);
  pinMode(BAT_SENSE_PIN, INPUT);

#if defined(HAVE_RGB_3)
  pinMode(BLUE_LED_PIN, OUTPUT); pinMode(RED_LED_PIN, OUTPUT); pinMode(GREEN_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN,  BLUE_LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(RED_LED_PIN,   RED_LED_ACTIVE_HIGH  ? LOW : HIGH);
  digitalWrite(GREEN_LED_PIN, GREEN_LED_ACTIVE_HIGH? LOW : HIGH);
#elif defined(HAVE_RGB_2)
  pinMode(BLUE_LED_PIN, OUTPUT); pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(BLUE_LED_PIN,  BLUE_LED_ACTIVE_HIGH ? LOW : HIGH);
  digitalWrite(RED_LED_PIN,   RED_LED_ACTIVE_HIGH  ? LOW : HIGH);
#elif defined(HAVE_SINGLE_LED)
  pinMode(SINGLE_LED_PIN, OUTPUT);
  digitalWrite(SINGLE_LED_PIN, SINGLE_LED_ACTIVE_HIGH ? LOW : HIGH);
#endif

  HeatPipeTemp.begin();
  HeatPipeTemp.setResolution(9);
  Serial.println(F("DS18B20 (HeatPipeTemp) initialized (9-bit)."));

  Matter.begin();
#ifdef HAS_MATTER_TEMP
  matter_heatPipeTemp.begin();
#else
  Serial.println(F("[Matter] Temperature cluster not available in this build."));
#endif

  printPairingInfo();
  pairedCached     = Matter.isDeviceCommissioned();
  threadConnCached = Matter.isDeviceThreadConnected();

  lastMeasurementMs = millis() - MEASUREMENT_INTERVAL_MS; // force an immediate measurement
  indicateUntilMs   = 0;
  lastPublishedMs   = 0;

  lowAlertCycleStartMs = millis() + LOW_ALERT_PERIOD_MS;
  chgBlinkNextEdgeMs   = millis() + CHG_BLINK_OFF_MS;
  fullBlinkNextEdgeMs  = millis() + FULL_BLINK_OFF_MS;

  nextChgPollMs        = millis() + 1000; // start quick poll soon after boot
}

void loop() {
  uint32_t ms = millis();

  handleButton();
  logPairingStateIfChanged();

  // Periodic full sensor update (every 3 minutes)
  if ((uint32_t)(ms - lastMeasurementMs) >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurementMs = ms;

    readSensorOnce();

    // Start temp indication window
    indicateUntilMs = ms + INDICATE_DURATION_MS;

#ifdef HAS_MATTER_TEMP
    publishIfNeeded();
#endif
  }

  // Quick battery-only poll every 6 seconds for charging/low/full responsiveness
  if ((int32_t)(ms - nextChgPollMs) >= 0) {
    quickChargePoll(ms);
    nextChgPollMs = ms + CHG_POLL_INTERVAL_MS;
  }

  // LED logic priority:
  // 1) Temp window active → show temp pattern
  // 2) Charging & full → GREEN slow
  // 3) Charging → PURPLE slow
  // 4) Low/Crit → ORANGE triple every 10 s
  // 5) Else off
  if ((int32_t)(ms - indicateUntilMs) <= 0) {
    updateTempWindowBlinker(ms);
  } else {
    setPurpleLed(false);
    setOrangeLed(false);
    setGreenOnly(false);

    if (chargeState == CHG_YES && battFull) {
      updateFullBlinker(ms);
    } else if (chargeState == CHG_YES) {
      updateChargingBlinker(ms);
    } else if (battLow || battCrit) {
      updateLowBatteryBlinker(ms);
    }
  }

  delay(3); // keep Thread/Matter serviced
}