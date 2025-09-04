# HeatPipeTemp - a centralised hot water supply monitor

A compact Matter/Thread temperature accessory for **Arduino Nano Matter (MGM240S)** using a **DS18B20** sensor and **analog battery monitoring**. Designed to be battery-friendly while keeping HomeKit responsive.

> ✅ **What you get**
> - Temperature from DS18B20 published to Matter (shows up in Apple Home as a Temperature Sensor).
> - **Battery level** derived from the raw pack voltage via a high-value divider into **A0** (pre-boost, not 5V VIN).
> - **LED status** behavior: solid **RED** when hot, smooth **BLUE bursts** when cool, plus charging/low-battery indicators.
> - **Button**: short press prints pairing info to Serial; long press (≥6 s) **decommissions** with a white confirmation blink ×6.
> - **No display / no QR**: all pairing info is printed to Serial.

---

## Features

- **Sensor**: DS18B20 on D5 (OneWire + DallasTemperature)
- **Matter**: Temperature cluster (via Silicon Labs Arduino Matter library)
- **LED policy** (5 s window after each measurement):
  - `HeatPipeTempC <= 28.0 °C` → smooth **BLUE 3-blink bursts** repeating
  - `HeatPipeTempC  > 28.0 °C` → solid **RED**
- **Battery monitor** on **A0** via high-impedance divider (default 1.0MΩ / 1.0MΩ) with **100 nF** RC filter to ground
- **Low-battery alert** (threshold configurable) via **Serial** + periodic **orange** triple-blink
- **Charging indication** (optional, if you wire a charger-detect pin):
  - **Purple slow blink** while charging
  - **Green slow blink** (or solid) on full charge
- **Pairing & Decommissioning**:
  - **Short press** → Pairing/Thread info to Serial
  - **Long press** (≥6 s) → Serial notice + **WHITE 6× fast blink** → `Matter.decommission()`

> ℹ️ **HomeKit battery tile**: The Arduino wrapper for the **Power Source** Matter cluster isn’t exposed yet, so Home won’t show a built-in battery UI from this sketch. We surface battery% via Serial and LED patterns. When Silicon Labs adds the wrapper, migrate to native Power Source reporting to get Home’s “Low Battery” UI.

---

## Hardware

- **Board**: Arduino **Nano Matter** (MGM240S)
- **Sensor**: DS18B20 (D5)
- **Battery**: 2P 18650 pack via BMS → (optionally) boost to 5V for VIN
- **Battery sense**: tap **raw pack (≈3.0–4.2 V)** **before** the 5V boost and feed to **A0** through a divider.

### Battery sense divider (only the essentials)

```
        R1 = 1.0 MΩ                 C1 = 100 nF
Pack+ ──/\/\/───┐
                ├─── A0  (to Nano Matter)
                ├───||───┐
                │  C1    │
               / \       │
               \ / R2=1MΩ│
                │        │
Pack− (GND) ────┴────────┴── GND
```

- **R1**: from pack+ to **A0**
- **R2**: from **A0** to **GND** (use **1.0 MΩ** to match R1 → ~½ scaling)
- **C1**: 100 nF **from A0 to GND** (noise filter)

> Use high-value resistors (MΩ) to minimize divider drain. If your board’s ADC expects ≤3.3 V, the 1:1 divider keeps A0 ≈ Vpack/2 (max ≈ 2.1 V).

---

## Software Setup

- **Arduino IDE** latest
- **Boards Manager** → install **Silicon Labs** “Arduino Nano Matter” core
- **Libraries** (Library Manager):
  - *OneWire* (Silicon Labs)
  - *DallasTemperature* (aka MAX31850 DallasTemp)
  - *Matter* (Silicon Labs Arduino Matter)

### Board / Port
- **Board**: *Arduino Nano Matter*
- **CPU Freq**: default (39 MHz) is fine
- **Upload**: via USB (Simplicity bootloader)

---

## Configuration

Adjust these in the sketch to suit your build:

```cpp
// Temperature LED threshold
const float    TEMP_THRESHOLD_C        = 28.0f;

// Cadence & publish policy
const uint32_t MEASUREMENT_INTERVAL_MS = 3UL * 60UL * 1000UL; // every 3 min
const float    DELTA_REPORT_C          = 0.2f;                 // publish if Δ ≥ 0.2°C
const uint32_t MAX_REPORT_INTERVAL_MS  = 10UL * 60UL * 1000UL; // force publish ≤10 min

// Divider & ADC calibration
const float R1_OHMS = 1'000'000.0f;
const float R2_OHMS = 1'000'000.0f;
const float VREF_3V3 = 3.251f;      // measure 3V3 on your board and put the real value here
const uint8_t ADC_BITS = 12;        // Nano Matter ADC is 12-bit (0..4095)
```

### Voltage math (what the code does)
1. Read ADC → `v_adc = raw * VREF_3V3 / (2^ADC_BITS - 1)`  
2. Undo divider → `v_pack = v_adc * (R1 + R2) / R2`  
3. Convert to **%** using a Li-ion curve (configurable), apply low-battery threshold.

**Tip:** calibrate with your DMM: measure `Vpack` and `Va0`, set `VREF_3V3` to your board’s real 3.3 V (e.g. 3.251 V).

---

## Pairing & Buttons

- **Short press** (BTN_BUILTIN, active LOW): prints **Manual code**, commissioning, and Thread link status to **Serial**.
- **Long press** (≥6 s): prints notice, blinks **WHITE×6**, then calls `Matter.decommission()` (device reboots → ready to re-pair).

> We don’t draw QR; use the manual code printed to Serial for onboarding.

---

## LED Policy (summary)

- After each measurement, show status for **5 seconds**, then LEDs off to save power.
- **Temp status**:
  - ≤ 28 °C → smooth **BLUE** triple-blink bursts (300ms on, 300ms off, ×3; then 1.0 s gap; repeat)
  - > 28 °C → solid **RED**
- **Battery/Charging** *(if charger detect wired)*:
  - **Orange** fast triple-blink every 10 s when **Low Battery** (needs charge)
  - **Purple** slow blink **while charging**
  - **Green** slow blink when **full**

> If you prefer battery indication to always override temp while active, set the code’s priority to `battery > temp` in the LED update section.

---

## Home / Matter Notes

- The sketch exposes a **Temperature Sensor** endpoint. Apple Home shows a temperature tile.  
- For a native **battery tile** (with “Low Battery” banners), the **Power Source** cluster must be implemented at the platform level (ZAP + generated code). When Silicon Labs adds an Arduino wrapper, migrate to it and map your measured percent/charging state accordingly.

---

## Troubleshooting

- **“Accessory Not Responding”** after sleeps: Keep Thread stack alive. This sketch avoids deep sleep and updates every 3 minutes, which tests well with HomeKit.
- **Wrong voltage on A0**: confirm you’re sensing **pre-boost** pack voltage, not 5V VIN; recheck R1/R2 placement; verify `VREF_3V3` and divider values.
- **No DS18B20**: ensure data pin is D5 with a 4.7 kΩ pull-up to 3V3 (as per Dallas sensors best practice).

---

## License

MIT — use at your own risk; verify thermal and battery thresholds for your application.

---

## Credits

- Silicon Labs Arduino Nano Matter board core and Matter library
- DallasTemperature & OneWire libraries
- You — for tapping that pre-boost point and doing the calibration 😉
