/*
  Nano Matter – HeatPipeTemp (DS18B20-only, battery-lean)

  This build:
    • Manual pairing code in an ASCII frame, one line:
        - 11 digits → "XXXX XXX XXXX"
        - 10 digits → "XXX XXXX XXX"
    • Minimal Serial spam:
        - Boot: one framed pairing block + brief info
        - Short button press: framed pairing block
        - Decommission press: short notice
        - Every 3 minutes: sensor lines (temp + battery + Matter publish/skip)
        - Charging-state transitions are ONLY printed inside the 3-minute cycle
          (no periodic/boot charging spam)
    • Battery sensing via A0 (R1=1M to pack+, R2=1M to GND, C1=100nF A0→GND)
    • Temp LED window 5s: ≤28°C → blue triple-bursts; >28°C → solid red
    • Low batt orange triple-blink every 10s; charging purple slow blink; full green slow blink
    • Long press (≥6s) → white 6×, Matter.decommission()

  Wiring (battery sense):
    Pack+ -- R1 (1.0M) --+-- A0
                          |
                         C1 (100nF) to GND
                          |
                         R2 (1.0M) to GND
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
const uint8_t DS18B20_PIN = 5;   // D5 on Nano Matter
const uint8_t BAT_ADC_PIN = A0;  // battery sense via divider to A0

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

// Fallback: single built-in LED (often active-LOW)
#elif defined(LED_BUILTIN)
  #define SINGLE_LED_PIN LED_BUILTIN
  #ifndef SINGLE_LED_ACTIVE_HIGH
    #define SINGLE_LED_ACTIVE_HIGH 0
  #endif
  #define HAVE_SINGLE_LED 1
#else
  #error "No LED pins detected. Define LED pins or LED_BUILTIN."
#endif

// -------------------- Power/Reporting Config --------------------
const float    TEMP_THRESHOLD_C          = 28.0f;                 // LED logic threshold
const uint32_t MEASUREMENT_INTERVAL_MS   = 3UL * 60UL * 1000UL;   // every 3 minutes
const uint32_t INDICATE_DURATION_MS      = 5000;                  // temp LED window
const float    DELTA_REPORT_C            = 0.2f;                  // publish if Δ ≥ 0.2°C
const uint32_t MAX_REPORT_INTERVAL_MS    = 10UL * 60UL * 1000UL;  // force publish ≤10 min

// BLUE burst (≤ threshold): 300ms on / 300ms off, x3, then 1.0s gap
const uint16_t BLINK_ON_MS               = 300;
const uint16_t BLINK_OFF_MS              = 300;
const uint8_t  BLINK_COUNT               = 3;
const uint16_t BURST_GAP_MS              = 1000;

// Quick white blink (decommission confirm)
const uint16_t QUICK_BLINK_ON_MS         = 120;
const uint16_t QUICK_BLINK_OFF_MS        = 120;
const uint8_t  QUICK_BLINK_COUNT         = 6;

// DS18B20 conversion (9-bit ≈ 94 ms)
const uint16_t DS18B20_CONV_MS           = 100;

// -------------------- Battery Sense / Calibration --------------------
// Divider: Pack+ --R1--> A0 --R2--> GND, with C1 100 nF from A0→GND
const float R1_OHMS = 1'000'000.0f;   // 1.0 MΩ
const float R2_OHMS = 1'000'000.0f;   // 1.0 MΩ
const uint8_t ADC_BITS = 12;          // 12-bit ADC (0..4095)
// Measured 3V3 reference on your board:
const float VREF_3V3 = 3.254f;

// Calibration tuned close to your board (keep if it matches your DMM)
// out = ideal * CAL_SLOPE + CAL_OFFSET_V
const float CAL_SLOPE    = 1.0007225f;
const float CAL_OFFSET_V = 0.0f;

// Battery thresholds (1S2P Li-ion, resting)
const float LOW_BATT_V   = 3.55f;     // needs charge (tune to taste)
const float FULL_BATT_V  = 4.18f;     // consider full near this

// Charging inference
const uint32_t CHARGE_CHECK_INTERVAL_MS = 6000; // re-check every 6 s (LED only; no prints here)
const float CHG_DV_MIN      = 0.015f;  // ≥15 mV rise → charging
const float FULL_DV_MAX     = 0.003f;  // ≤3 mV change near full → treat as FULL

// (We still do a boot assist internally, but do NOT print there)
const uint32_t CHG_BOOT_WINDOW_MS         = 20000; // 20 s after boot
const uint32_t CHG_BOOT_SAMPLE_PERIOD_MS  = 1000;  // 1 s
const float    CHG_BOOT_DV_MIN            = 0.006f; // ≥6 mV cumulative rise → charging

// -------------------- DS18B20 --------------------
OneWire oneWire(DS18B20_PIN);
DallasTemperature HeatPipeTemp(&oneWire);  // requested name
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

// Button
bool     btnLast   = true;   // active LOW
uint32_t btnDownMs = 0;

// Pairing/Thread state cache
String pairingPIN;
bool pairedCached     = false;
bool threadConnCached = false;

// Temperature blinker
enum BlinkState { BLINK_IDLE, BLINK_ON, BLINK_OFF, BLINK_GAP };
BlinkState blinkState = BLINK_IDLE;
uint8_t    blinkIndex = 0;             // 0..BLINK_COUNT-1
uint32_t   nextBlinkTransitionMs = 0;

// Battery status / charging inference
enum ChargeState { CHG_UNKNOWN, CHG_DISCHARGING, CHG_CHARGING, CHG_FULL };
ChargeState chargeState = CHG_UNKNOWN;
float lastVoltCheck = NAN;
uint32_t lastChargeCheckMs = 0;
bool lowBattery = false;

// Boot assist (no prints here)
uint32_t bootMs0 = 0;
bool     bootAssistActive = true;
float    bootVStart = NAN;
float    bootVMax   = NAN;
uint32_t lastBootProbeMs = 0;

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
inline void setWhiteLed(bool on) {
  #if defined(HAVE_RGB_3)
    setRedLed(on); setGreenLed(on); setBlueLed(on);
  #elif defined(HAVE_RGB_2)
    setRedLed(on); setBlueLed(on); // pseudo-white
  #elif defined(HAVE_SINGLE_LED)
    setSingleLed(on);
  #endif
}
inline void setPurpleLed(bool on) { // red + blue
  #if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
    setRedLed(on); setBlueLed(on);
  #elif defined(HAVE_SINGLE_LED)
    setSingleLed(on);
  #endif
}
inline void setOrangeLed(bool on) { // red + (some green)
  #if defined(HAVE_RGB_3)
    setRedLed(on); setGreenLed(on);
  #elif defined(HAVE_RGB_2)
    setRedLed(on); // pseudo-orange
  #elif defined(HAVE_SINGLE_LED)
    setSingleLed(on);
  #endif
}
void ledsOff() {
  #if defined(HAVE_RGB_3)
    setRedLed(false); setGreenLed(false); setBlueLed(false);
  #elif defined(HAVE_RGB_2)
    setRedLed(false); setBlueLed(false);
  #elif defined(HAVE_SINGLE_LED)
    setSingleLed(false);
  #endif
}

// -------------------- Temperature LED blinker (5s window) --------------------
void updateTempBlinker(uint32_t ms) {
  if ((int32_t)(ms - indicateUntilMs) > 0) {
    // window over; keep battery overlays if any
    if (!lowBattery && chargeState != CHG_CHARGING && chargeState != CHG_FULL) {
      ledsOff();
    }
    blinkState = BLINK_IDLE;
    return;
  }

  // Above threshold → solid RED (during indicate window)
  if (!isnan(HeatPipeTempC) && HeatPipeTempC > TEMP_THRESHOLD_C) {
    #if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
      setBlueLed(false);
      setRedLed(true);
    #elif defined(HAVE_SINGLE_LED)
      setSingleLed(true);
    #endif
    blinkState = BLINK_IDLE;
    return;
  }

  // ≤ threshold → repeating smooth 3-blink BLUE burst
  switch (blinkState) {
    case BLINK_IDLE:
      #if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
        setRedLed(false);
        setBlueLed(true);
      #elif defined(HAVE_SINGLE_LED)
        setSingleLed(true);
      #endif
      blinkIndex = 0;
      blinkState = BLINK_ON;
      nextBlinkTransitionMs = ms + BLINK_ON_MS;
      break;

    case BLINK_ON:
      if ((int32_t)(ms - nextBlinkTransitionMs) >= 0) {
        #if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
          setBlueLed(false);
        #elif defined(HAVE_SINGLE_LED)
          setSingleLed(false);
        #endif
        blinkState = BLINK_OFF;
        nextBlinkTransitionMs = ms + BLINK_OFF_MS;
      }
      break;

    case BLINK_OFF:
      if ((int32_t)(ms - nextBlinkTransitionMs) >= 0) {
        blinkIndex++;
        if (blinkIndex < BLINK_COUNT) {
          #if defined(HAVE_RGB_3) || defined(HAVE_RGB_2)
            setBlueLed(true);
          #elif defined(HAVE_SINGLE_LED)
            setSingleLed(true);
          #endif
          blinkState = BLINK_ON;
          nextBlinkTransitionMs = ms + BLINK_ON_MS;
        } else {
          blinkState = BLINK_GAP;
          nextBlinkTransitionMs = ms + BURST_GAP_MS;
        }
      }
      break;

    case BLINK_GAP:
      if ((int32_t)(ms - nextBlinkTransitionMs) >= 0) {
        blinkState = BLINK_IDLE; // next burst
      }
      break;
  }
}

// -------------------- Battery LED overlay --------------------
// Low battery: quick orange triple-blink every 10 s
const uint32_t LOW_ALERT_PERIOD_MS = 10000;
const uint16_t LOW_ALERT_ON_MS     = 120;
const uint16_t LOW_ALERT_OFF_MS    = 120;
const uint8_t  LOW_ALERT_PULSES    = 3;
uint32_t lowAlertCycleStartMs = 0;
uint8_t  lowAlertPulseIndex   = 0;
bool     lowAlertActivePulse  = false;
uint32_t lowAlertNextEdgeMs   = 0;

// Charging: purple slow blink (on 400ms / off 600ms)
bool     chgBlinkOn           = false;
uint32_t chgBlinkNextEdgeMs   = 0;

// Full: green slow blink (on 300ms / off 1200ms) to allow temp LED window
bool     fullBlinkOn          = false;
uint32_t fullBlinkNextEdgeMs  = 0;

void updateBatteryLED(uint32_t ms) {
  // LOW BATTERY pattern
  if (lowBattery) {
    if ((int32_t)(ms - lowAlertCycleStartMs) >= (int32_t)LOW_ALERT_PERIOD_MS) {
      lowAlertCycleStartMs = ms;
      lowAlertPulseIndex = 0;
      lowAlertActivePulse = true;
      lowAlertNextEdgeMs = ms + LOW_ALERT_ON_MS;
      setOrangeLed(true);
    } else if (lowAlertActivePulse) {
      if ((int32_t)(ms - lowAlertNextEdgeMs) >= 0) {
        if ( (lowAlertPulseIndex & 1) == 0 ) {
          setOrangeLed(false);
          lowAlertNextEdgeMs = ms + LOW_ALERT_OFF_MS;
        } else {
          setOrangeLed(true);
          lowAlertNextEdgeMs = ms + LOW_ALERT_ON_MS;
        }
        lowAlertPulseIndex++;
        if (lowAlertPulseIndex >= LOW_ALERT_PULSES * 2) {
          lowAlertActivePulse = false;
          setOrangeLed(false);
        }
      }
    }
  }

  // CHARGING pattern (purple slow blink)
  if (chargeState == CHG_CHARGING) {
    if ((int32_t)(ms - chgBlinkNextEdgeMs) >= 0) {
      chgBlinkOn = !chgBlinkOn;
      setPurpleLed(chgBlinkOn);
      chgBlinkNextEdgeMs = ms + (chgBlinkOn ? 400 : 600);
    }
  } else {
    setPurpleLed(false);
  }

  // FULL pattern (green slow blink)
  if (chargeState == CHG_FULL) {
    if ((int32_t)(ms - fullBlinkNextEdgeMs) >= 0) {
      fullBlinkOn = !fullBlinkOn;
      setGreenLed(fullBlinkOn);
      fullBlinkNextEdgeMs = ms + (fullBlinkOn ? 300 : 1200);
    }
  } else {
    setGreenLed(false);
  }
}

// -------------------- Pairing / Buttons --------------------
static String digitsOnly(String s) {
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] >= '0' && s[i] <= '9') out += s[i];
  }
  return out;
}

static String groupManualOneLine(const String &raw) {
  String d = digitsOnly(raw);
  if (d.length() == 11) {
    // XXXX XXX XXXX
    return d.substring(0,4) + " " + d.substring(4,7) + " " + d.substring(7,11);
  } else if (d.length() == 10) {
    // XXX XXXX XXX
    return d.substring(0,3) + " " + d.substring(3,7) + " " + d.substring(7,10);
  } else {
    // Fallback: print whatever we have
    return d;
  }
}

static void printFramedOneLine(const String &line) {
  // Build a simple box around the line: +---...---+
  const size_t inner = line.length() + 2; // spaces padding
  String top("+");    for (size_t i=0;i<inner;i++) top += "-"; top += "+";
  String mid("| ");   mid += line; mid += " |";
  String bot("+");    for (size_t i=0;i<inner;i++) bot += "-"; bot += "+";

  Serial.println(top);
  Serial.println(mid);
  Serial.println(bot);
}

void printPairingInfo() {
  pairingPIN = Matter.getManualPairingCode();
  String grouped = groupManualOneLine(pairingPIN);

  Serial.println(F("=== Matter Pairing Info ==="));
  Serial.print(F("Commissioned: ")); Serial.println(Matter.isDeviceCommissioned() ? F("yes") : F("no"));
  Serial.print(F("Thread link:  ")); Serial.println(Matter.isDeviceThreadConnected() ? F("connected") : F("not connected"));
  Serial.println(F("Manual pairing code:"));
  printFramedOneLine(grouped);    // <-- ONE LINE in a frame
  Serial.println(F("==========================="));
}

void printDecommissionNotice() {
  Serial.println(F("[Matter] Decommission requested (long press ≥ 6 s)."));
  Serial.println(F("Blinking WHITE 6× fast, then decommissioning..."));
}

void doQuickWhiteBlinkConfirm() {
  for (uint8_t i = 0; i < QUICK_BLINK_COUNT; ++i) {
    setWhiteLed(true);
    delay(QUICK_BLINK_ON_MS);
    setWhiteLed(false);
    delay(QUICK_BLINK_OFF_MS);
  }
}

void handleButton() {
  static bool longPressTriggered = false;

  bool now = (digitalRead(BTN_BUILTIN) == LOW); // active LOW
  uint32_t ms = millis();

  if (now && !btnLast) {
    btnDownMs = ms;
    longPressTriggered = false;
  } else if (now && btnLast) {
    if (!longPressTriggered) {
      uint32_t held = ms - btnDownMs;
      if (held >= 6000) {
        longPressTriggered = true;
        printDecommissionNotice();
        doQuickWhiteBlinkConfirm();
        Matter.decommission(); // will reboot
      }
    }
  } else if (!now && btnLast) {
    if (!longPressTriggered) {
      printPairingInfo(); // short press
    }
  }
  btnLast = now;
}

void logPairingStateIfChanged() {
  bool nowPaired = Matter.isDeviceCommissioned();
  bool nowThread = Matter.isDeviceThreadConnected();
  if (nowPaired != pairedCached || nowThread != threadConnCached) {
    pairedCached = nowPaired;
    threadConnCached = nowThread;
    Serial.print(F("[Matter] Commissioned: "));
    Serial.print(nowPaired ? F("yes") : F("no"));
    Serial.print(F(" | Thread: "));
    Serial.println(nowThread ? F("connected") : F("not connected"));
    printPairingInfo();
  }
}

// -------------------- Battery measurement --------------------
float readPackVoltageRawIdeal() {
  // average a few ADC samples
  const uint8_t N = 8;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < N; ++i) {
    acc += analogRead(BAT_ADC_PIN);
    delay(2);
  }
  const uint32_t raw = acc / N;

  const float kAdcMax = (float)((1u << ADC_BITS) - 1u);
  const float v_adc = (raw * VREF_3V3) / kAdcMax;

  // Undo divider → ideal pack volts (pre-calibration)
  return v_adc * (R1_OHMS + R2_OHMS) / R2_OHMS;
}

float readPackVoltage() {
  float v_pack = readPackVoltageRawIdeal();
  // Apply calibration
  v_pack = v_pack * CAL_SLOPE + CAL_OFFSET_V;
  return v_pack;
}

uint8_t estimatePercent(float v) {
  // Simple 1S2P Li-ion mapping (resting): 3.30V→0%, 4.20V→100%
  const float V_MIN = 3.30f;
  const float V_MAX = 4.20f;
  float p = (v - V_MIN) * 100.0f / (V_MAX - V_MIN);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return (uint8_t)(p + 0.5f);
}

// update charging state w/ option to suppress Serial prints
void updateChargeStateCore(float v_now, uint32_t ms, bool allowPrint, const char* reasonTag) {
  // Low-battery flag
  lowBattery = (v_now <= LOW_BATT_V);

  // Determine from dv/dt + absolute voltage & hysteresis
  ChargeState newState = chargeState;

  if (isnan(lastVoltCheck)) {
    newState = CHG_UNKNOWN;
  } else {
    float dv = v_now - lastVoltCheck;

    if (dv >= CHG_DV_MIN) {
      newState = CHG_CHARGING;
    } else {
      if (v_now >= FULL_BATT_V && fabsf(dv) <= FULL_DV_MAX) {
        newState = CHG_FULL;
      } else {
        newState = (lowBattery ? CHG_DISCHARGING : CHG_UNKNOWN);
      }
    }
  }

  bool changed = (newState != chargeState);
  chargeState = newState;
  lastVoltCheck = v_now;
  lastChargeCheckMs = ms;

  if (changed && allowPrint) {
    Serial.print(F("[Battery] State="));
    switch (chargeState) {
      case CHG_CHARGING:     Serial.print(F("CHARGING")); break;
      case CHG_FULL:         Serial.print(F("FULL")); break;
      case CHG_DISCHARGING:  Serial.print(F("DISCHARGING")); break;
      default:               Serial.print(F("UNKNOWN")); break;
    }
    Serial.print(F("  V=")); Serial.print(v_now, 3);
    Serial.print(F("  (reason: ")); Serial.print(reasonTag); Serial.println(F(")"));
  }
}

void bootChargeAssist(uint32_t ms) {
  if (!bootAssistActive) return;

  // sample ~1 s (no Serial here)
  if ((int32_t)(ms - lastBootProbeMs) >= (int32_t)CHG_BOOT_SAMPLE_PERIOD_MS) {
    float v = readPackVoltage();
    if (isnan(bootVStart)) bootVStart = v;
    if (isnan(bootVMax) || v > bootVMax) bootVMax = v;
    lastBootProbeMs = ms;

    // suppress prints during boot
    updateChargeStateCore(v, ms, /*allowPrint=*/false, "boot-probe");
  }

  if ((int32_t)(ms - bootMs0) >= (int32_t)CHG_BOOT_WINDOW_MS) {
    // find likely state but don't print here
    if (!isnan(bootVStart) && !isnan(bootVMax) && (bootVMax - bootVStart) >= CHG_BOOT_DV_MIN) {
      chargeState = CHG_CHARGING;
    } else if (bootVMax >= FULL_BATT_V) {
      chargeState = CHG_FULL;
    }
    bootAssistActive = false;
  }
}

void periodicChargePoll(uint32_t ms) {
  if ((int32_t)(ms - lastChargeCheckMs) >= (int32_t)CHARGE_CHECK_INTERVAL_MS) {
    float v = readPackVoltage();
    // suppress prints here; LEDs still reflect state
    updateChargeStateCore(v, ms, /*allowPrint=*/false, "periodic");
  }
}

// -------------------- Sensor & Reporting --------------------
void publishIfNeeded() {
  #ifdef HAS_MATTER_TEMP
    bool mustRefresh = (millis() - lastPublishedMs) >= MAX_REPORT_INTERVAL_MS;
    bool bigDelta    = isnan(lastPublishedC) || isnan(HeatPipeTempC) ? true
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
  #endif
}

void readSensorOnce() {
  HeatPipeTemp.requestTemperatures();
  // Short cooperative wait for DS18B20 conversion
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < DS18B20_CONV_MS) {
    delay(2);
  }

  float t = HeatPipeTemp.getTempCByIndex(0);
  if (t > -80 && t < 150) HeatPipeTempC = t;

  // Battery snapshot; allow printing of state transitions ONLY here
  float vbatt = readPackVoltage();
  updateChargeStateCore(vbatt, millis(), /*allowPrint=*/true, "temp-cycle");

  // Output (clean)
  Serial.println(F("--- Sensor reading ---"));
  Serial.print  (F("Temperature (HeatPipeTemp / DS18B20): "));
  if (isnan(HeatPipeTempC)) Serial.println(F("--.- C"));
  else { Serial.print(HeatPipeTempC, 2); Serial.println(F(" C")); }

  uint8_t pct = estimatePercent(vbatt);
  Serial.print(F("Battery (1S2P pack): "));
  Serial.print(vbatt, 3);
  Serial.print(F(" V  ("));
  Serial.print(pct);
  Serial.print(F("%)  -> "));
  switch (chargeState) {
    case CHG_CHARGING:     Serial.println(F("CHARGING")); break;
    case CHG_FULL:         Serial.println(F("FULL")); break;
    case CHG_DISCHARGING:  Serial.println(lowBattery ? F("LOW / DISCHARGING") : F("DISCHARGING")); break;
    default:               Serial.println(F("UNKNOWN")); break;
  }
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println();
  Serial.println(F("Nano Matter – HeatPipeTemp | framed pairing, minimal Serial, 3-min cadence"));

  pinMode(BTN_BUILTIN, INPUT_PULLUP);

  #if defined(HAVE_RGB_3)
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN,  OUTPUT);
    pinMode(GREEN_LED_PIN,OUTPUT);
    // OFF at boot (account for active-LOW default)
    #if BLUE_LED_ACTIVE_HIGH
      digitalWrite(BLUE_LED_PIN, LOW);
    #else
      digitalWrite(BLUE_LED_PIN, HIGH);
    #endif
    #if RED_LED_ACTIVE_HIGH
      digitalWrite(RED_LED_PIN, LOW);
    #else
      digitalWrite(RED_LED_PIN, HIGH);
    #endif
    #if GREEN_LED_ACTIVE_HIGH
      digitalWrite(GREEN_LED_PIN, LOW);
    #else
      digitalWrite(GREEN_LED_PIN, HIGH);
    #endif

  #elif defined(HAVE_RGB_2)
    pinMode(BLUE_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN,  OUTPUT);
    #if BLUE_LED_ACTIVE_HIGH
      digitalWrite(BLUE_LED_PIN, LOW);
    #else
      digitalWrite(BLUE_LED_PIN, HIGH);
    #endif
    #if RED_LED_ACTIVE_HIGH
      digitalWrite(RED_LED_PIN, LOW);
    #else
      digitalWrite(RED_LED_PIN, HIGH);
    #endif

  #elif defined(HAVE_SINGLE_LED)
    pinMode(SINGLE_LED_PIN, OUTPUT);
    #if SINGLE_LED_ACTIVE_HIGH
      digitalWrite(SINGLE_LED_PIN, LOW);
    #else
      digitalWrite(SINGLE_LED_PIN, HIGH);
    #endif
  #endif

  // DS18B20 init
  HeatPipeTemp.begin();
  HeatPipeTemp.setResolution(9); // fastest conversion
  Serial.println(F("DS18B20 (HeatPipeTemp) initialized (9-bit)."));

  // Matter
  Matter.begin();
  #ifdef HAS_MATTER_TEMP
    matter_heatPipeTemp.begin();
  #else
    Serial.println(F("[Matter] Temperature cluster not available in this build."));
  #endif

  // Pairing info on boot (FRAMED + grouped)
  printPairingInfo();
  pairedCached     = Matter.isDeviceCommissioned();
  threadConnCached = Matter.isDeviceThreadConnected();

  // Force immediate first read & LED window
  lastMeasurementMs = millis() - MEASUREMENT_INTERVAL_MS; // read right away
  indicateUntilMs   = 0;
  lastPublishedMs   = 0;

  // Boot assist init (no serial spam)
  bootMs0 = millis();
  bootAssistActive = true;
  bootVStart = NAN;
  bootVMax   = NAN;
  lastBootProbeMs = 0;

  // Seed lastVoltCheck (no print)
  lastVoltCheck = readPackVoltage();
  lastChargeCheckMs = millis();
}

void loop() {
  handleButton();
  logPairingStateIfChanged();

  uint32_t ms = millis();

  // Boot-time assist for charging detection (no prints)
  bootChargeAssist(ms);

  // Periodic charger polling every 6 s (LED-only, no prints)
  periodicChargePoll(ms);

  // Periodic sensor update (every 3 minutes)
  if ((uint32_t)(ms - lastMeasurementMs) >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurementMs = ms;

    readSensorOnce();

    // Start short LED indication window to save power the rest of the time
    indicateUntilMs = ms + INDICATE_DURATION_MS;
    blinkState = BLINK_IDLE; // restart BLUE pattern if needed

    // Publish to Matter with delta/refresh policy
    publishIfNeeded();
  }

  // LED indication (temperature window)
  updateTempBlinker(ms);

  // Battery overlay (low / charging / full)
  updateBatteryLED(ms);

  // Keep loop snappy so Thread/Matter stays serviced
  delay(3);
}
