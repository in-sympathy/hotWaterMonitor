/*
  Nano Matter – HeatPipeTemp (DS18B20-only, battery + temp)

  Noise cleanup:
    • Framed pairing block prints ONCE at boot.
    • Later state changes print only one line: "[Matter] Commissioned: ... | Thread: ..."
    • Sensor print is a single compact line: "Temp: xx.xx °C | Batt: x.xxx V[ | pub]"
    • Voltage reading code kept EXACTLY as before.

  Divider wiring (unchanged):
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

// ---- DallasTemperature portability: some forks only define DEVICE_DISCONNECTED
#ifndef DEVICE_DISCONNECTED_C
  #ifdef DEVICE_DISCONNECTED
    #define DEVICE_DISCONNECTED_C DEVICE_DISCONNECTED
  #else
    #define DEVICE_DISCONNECTED_C (-127.0f)
  #endif
#endif
// --------------------------------------------------------

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
const uint8_t BAT_ADC_PIN = A0;  // A0 for battery divider

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
const float    TEMP_THRESHOLD_C          = 22.0f;                 // trigger temperature for the LED and automation behavior 
const uint32_t MEASUREMENT_INTERVAL_MS   = 3UL * 60UL * 1000UL;   // every 3 minutes
const uint32_t INDICATE_DURATION_MS      = 5000;                  // show LEDs for 5 s after each measurement
const float    DELTA_REPORT_C            = 0.2f;                  // publish if Δ ≥ 0.2°C
const uint32_t MAX_REPORT_INTERVAL_MS    = 10UL * 60UL * 1000UL;  // force publish ≤10 min

// Smooth BLUE 3-blink burst: 300ms on / 300ms off, x3, then 1.0s gap
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
// (KEEP EXACTLY AS BEFORE)
const float R1_OHMS = 1'000'000.0f;   // 1.0 MΩ
const float R2_OHMS = 1'000'000.0f;   // 1.0 MΩ  (equal → ideal pack = 2 * VA0)
const uint8_t ADC_BITS = 12;          // 12-bit ADC (0..4095)
const float VREF_3V3 = 3.254f;        // your measured 3V3
const float CAL_SLOPE    = 1.0000f;   // leave as-is
const float CAL_OFFSET_V = 0.000f;    // leave as-is

// -------------------- DS18B20 --------------------
OneWire oneWire(DS18B20_PIN);
DallasTemperature HeatPipeTemp(&oneWire);  // requested name
float HeatPipeTempC = NAN;
bool ds18b20Present = false;               // presence flag

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
bool     btnLast   = true;   // will be seeded in setup() from real pin state
uint32_t btnDownMs = 0;

// Pairing/Thread state cache + print gate
String pairingPIN;
bool pairedCached     = false;
bool threadConnCached = false;
bool bootPairingPrinted = false;  // blocks duplicate framed print

// Blinker
enum BlinkState { BLINK_IDLE, BLINK_ON, BLINK_OFF, BLINK_GAP };
BlinkState blinkState = BLINK_IDLE;
uint8_t    blinkIndex = 0;             // 0..BLINK_COUNT-1
uint32_t   nextBlinkTransitionMs = 0;

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
inline void ledsOff() {
  #if defined(HAVE_RGB_3)
    setRedLed(false); setGreenLed(false); setBlueLed(false);
  #elif defined(HAVE_RGB_2)
    setRedLed(false); setBlueLed(false);
  #elif defined(HAVE_SINGLE_LED)
    setSingleLed(false);
  #endif
}

// -------------------- Battery measurement (unchanged) --------------------
void primeBatteryNode() {
  pinMode(BAT_ADC_PIN, INPUT);
  delay(220);
  for (uint8_t i = 0; i < 8; ++i) { (void)analogRead(BAT_ADC_PIN); delay(5); }
}
float readPackVoltageRawIdeal() {
  const uint8_t N = 16;
  uint32_t acc = 0;
  for (uint8_t i = 0; i < N; ++i) { acc += analogRead(BAT_ADC_PIN); delay(2); }
  const uint32_t raw = acc / N;
  const float kAdcMax = (float)((1u << ADC_BITS) - 1u); // 4095
  const float v_adc   = (raw * VREF_3V3) / kAdcMax;
  return v_adc * (R1_OHMS + R2_OHMS) / R2_OHMS; // R1==R2 → *2
}
float readPackVoltage() {
  float v_pack = readPackVoltageRawIdeal();
  v_pack = v_pack * CAL_SLOPE + CAL_OFFSET_V;
  return v_pack;
}

// -------------------- DS18B20 helpers --------------------
// Portable helper: avoid isConversionComplete() (not present in some forks)
static float readTempOnceWithTimeout(uint16_t extraWaitMs) {
  HeatPipeTemp.requestTemperatures();            // may block briefly on some libs
  delay(DS18B20_CONV_MS + extraWaitMs);          // ensure conversion window elapsed
  return HeatPipeTemp.getTempCByIndex(0);
}

// -------------------- Temperature LED (only during indicate window) --------------------
void updateBlinker(uint32_t ms) {
  if ((int32_t)(ms - indicateUntilMs) > 0) {
    ledsOff();
    blinkState = BLINK_IDLE;
    return;
  }
  // Above threshold → solid RED
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

// -------------------- Pairing / Buttons --------------------
static String digitsOnly(String s) {
  String out; out.reserve(s.length());
  for (size_t i = 0; i < s.length(); ++i) if (s[i] >= '0' && s[i] <= '9') out += s[i];
  return out;
}
static String groupManualOneLine(const String &raw) {
  String d = digitsOnly(raw);
  if (d.length() == 11) return d.substring(0,4) + " " + d.substring(4,7) + " " + d.substring(7,11);
  if (d.length() == 10) return d.substring(0,3) + " " + d.substring(3,7) + " " + d.substring(7,10);
  return d;
}
static void printFramedOneLine(const String &line) {
  const size_t inner = line.length() + 2;
  String top("+"); for (size_t i=0;i<inner;i++) top += "-"; top += "+";
  String mid("| ");   mid += line; mid += " |";
  String bot("+"); for (size_t i=0;i<inner;i++) bot += "-"; bot += "+";
  Serial.println(top); Serial.println(mid); Serial.println(bot);
}

// framed block ONCE at boot
void printBootPairingBlockOnce() {
  if (bootPairingPrinted) return;
  bootPairingPrinted = true;

  pairingPIN = Matter.getManualPairingCode();
  String grouped = groupManualOneLine(pairingPIN);

  Serial.println(F("=== Matter Pairing Info ==="));
  Serial.print(F("Commissioned: ")); Serial.println(Matter.isDeviceCommissioned() ? F("yes") : F("no"));
  Serial.print(F("Thread link:  ")); Serial.println(Matter.isDeviceThreadConnected() ? F("connected") : F("not connected"));
  Serial.println(F("Manual pairing code:"));
  printFramedOneLine(grouped);
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
      // Short press → show framed block on demand (explicit request only)
      pairingPIN = Matter.getManualPairingCode();
      printFramedOneLine(groupManualOneLine(pairingPIN));
    }
  }
  btnLast = now;
}

void logPairingStateIfChanged() {
  bool nowPaired = Matter.isDeviceCommissioned();
  bool nowThread = Matter.isDeviceThreadConnected();
  if (nowPaired != pairedCached || nowThread != threadConnCached) {
    pairedCached     = nowPaired;
    threadConnCached = nowThread;
    // ONE single line, no frames
    Serial.print(F("[Matter] Commissioned: "));
    Serial.print(nowPaired ? F("yes") : F("no"));
    Serial.print(F(" | Thread: "));
    Serial.println(nowThread ? F("connected") : F("not connected"));
  }
}

// -------------------- Sensor & Reporting --------------------
bool publishIfNeeded() {
  #ifdef HAS_MATTER_TEMP
    bool mustRefresh = (millis() - lastPublishedMs) >= MAX_REPORT_INTERVAL_MS;
    bool bigDelta    = isnan(lastPublishedC) || isnan(HeatPipeTempC) ? true
                         : (fabsf(HeatPipeTempC - lastPublishedC) >= DELTA_REPORT_C);

    if (!isnan(HeatPipeTempC) && (bigDelta || mustRefresh)) {
      matter_heatPipeTemp.set_measured_value_celsius(HeatPipeTempC);
      lastPublishedC  = HeatPipeTempC;
      lastPublishedMs = millis();
      return true;
    }
  #endif
  return false;
}

void readSensorOnce() {
  float t = readTempOnceWithTimeout(60);  // ~160 ms total budget

  // Retry once if disconnected (-127°C) or first-read default (85°C) or out of sane range
  if (t == DEVICE_DISCONNECTED_C || fabsf(t - 85.0f) < 0.01f || t < -80.0f || t > 150.0f) {
    t = readTempOnceWithTimeout(200);     // give it more time
  }

  if (t > -80.0f && t < 150.0f) {
    HeatPipeTempC = t;
  } // else keep previous HeatPipeTempC (may be NAN), which prints "--.-"

  // Battery snapshot (unchanged math)
  float vbatt = readPackVoltage();

  // Publish (if needed) and print ONE compact line
  bool didPub = publishIfNeeded();

  Serial.print(F("Temp: "));
  if (isnan(HeatPipeTempC)) Serial.print(F("--.-"));
  else                      Serial.print(HeatPipeTempC, 2);
  Serial.print(F(" °C | Batt: "));
  Serial.print(vbatt, 3);
  Serial.print(F(" V"));
  if (didPub) Serial.print(F(" | pub"));
  Serial.println();
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println();
  Serial.println(F("Nano Matter – HeatPipeTemp | 3-min cadence, compact prints"));

  // ADC config FIRST, then prime the A0 node so the first reading is correct
  analogReadResolution(ADC_BITS);
  primeBatteryNode();

  pinMode(BTN_BUILTIN, INPUT_PULLUP);
  delay(5);
  // >>> Seed last button state to avoid a phantom 'release' on first loop
  btnLast = (digitalRead(BTN_BUILTIN) == LOW);

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

  // DS18B20
  HeatPipeTemp.begin();
  HeatPipeTemp.setResolution(9); // fastest conversion
  ds18b20Present = (HeatPipeTemp.getDeviceCount() > 0);
  Serial.println(F("DS18B20 ready."));
  if (!ds18b20Present) {
    Serial.println(F("[DS18B20] not detected on D5 (check wiring & 4.7k pull-up)"));
  }

  // Matter
  Matter.begin();
  #ifdef HAS_MATTER_TEMP
    matter_heatPipeTemp.begin();
  #else
    Serial.println(F("[Matter] Temperature cluster not available in this build."));
  #endif

  // Pairing info on boot (framed ONCE)
  printBootPairingBlockOnce();
  pairedCached     = Matter.isDeviceCommissioned();
  threadConnCached = Matter.isDeviceThreadConnected();

  // Force immediate first read & LED window
  lastMeasurementMs = millis() - MEASUREMENT_INTERVAL_MS; // read right away
  indicateUntilMs   = 0;
  lastPublishedMs   = 0;
}

void loop() {
  handleButton();
  logPairingStateIfChanged();

  uint32_t ms = millis();

  // Periodic sensor update (every 3 minutes)
  if ((uint32_t)(ms - lastMeasurementMs) >= MEASUREMENT_INTERVAL_MS) {
    lastMeasurementMs = ms;

    readSensorOnce();

    // Show LED status briefly to save power the rest of the time
    indicateUntilMs = ms + INDICATE_DURATION_MS;
    blinkState = BLINK_IDLE; // restart BLUE pattern if needed
  }

  // LED indication (temperature window)
  updateBlinker(ms);

  // Keep loop snappy so Thread/Matter stays serviced
  delay(3);
}
