# Nano Matter — HeatPipeTemp (DS18B20-only)

Ultra-quiet, battery + temperature reporter for Arduino Nano Matter using a DS18B20 on 1‑Wire.

- 3-minute measurement cadence
- Compact single-line serial print: `Temp: xx.xx °C | Batt: x.xxx V[ | pub]`
- LED window after each reading (BLUE ≤ 28 °C in 3-blink bursts, solid RED > 28 °C)
- Matter/Thread support (optional Temperature cluster if available)
- Long-press button (≥ 6 s) decommissions; short-press reprints pairing code
- Battery sensing math and calibration kept exactly as in code

---

## Overview

This sketch reads a DS18B20 temperature sensor and a battery voltage divider on the Arduino Nano Matter. It prints extremely compact status lines to Serial and only lights LEDs for a brief “indication window” after each reading to save power. If a Matter Temperature cluster is available in your build, temperature updates are published on significant change or periodically.

---

## Hardware

- Board: Arduino Nano Matter (EFR32)
- Sensor: DS18B20 (powered mode, not parasite)
- Button: Built-in (active-LOW)
- LEDs: Uses board RGB (active-LOW on most Nano Matter boards) or a single built-in LED as fallback

### Battery Divider (unchanged)

```
Pack+ -- R1 (1.0M) --+-- A0
                      |
                     C1 (100nF) to GND
                      |
                     R2 (1.0M) to GND
```

### 1-Wire Wiring (default pin D5)

- DS18B20 DQ → D5
- DS18B20 VDD → 3V3
- DS18B20 GND → GND
- 4.7 kΩ pull-up from D5 → 3V3 (required)

---

## Pinout & Key Constants (from the sketch)

- `DS18B20_PIN = 5` (D5)
- `BAT_ADC_PIN = A0`
- `TEMP_THRESHOLD_C = 28.0`
- `MEASUREMENT_INTERVAL_MS = 180000` (3 minutes)
- `INDICATE_DURATION_MS = 5000`
- `DELTA_REPORT_C = 0.2` (Matter publish threshold)
- `MAX_REPORT_INTERVAL_MS = 600000` (force publish ≤ 10 min)
- `DS18B20_CONV_MS = 100` (9-bit conversion)

Battery sense and calibration (unchanged):
- `ADC_BITS = 12` (0..4095)
- `VREF_3V3 = 3.254`
- `R1 = R2 = 1.0 MΩ` (equal; ideal pack voltage ≈ `2 * V(A0)`)
- `CAL_SLOPE = 1.0000`, `CAL_OFFSET_V = 0.000`

With equal resistors and a 3.3 V ADC, the measurable pack voltage is up to about 6.6 V. Stay within board limits.

---

## Required Libraries

- OneWire
- DallasTemperature (exactly one provider installed)
  - Miles Burton DallasTemperature, or
  - Adafruit MAX31850_DallasTemp (also exposes DallasTemperature.h)
- Matter (bundled with Nano Matter core)

Keep only one DallasTemperature provider installed to avoid header or ABI mismatches. On Nano Matter, prefer the OneWire that ships with the Silicon Labs core. If you also have a user-installed OneWire in your sketchbook libraries, disable or rename it to avoid conflicts so the core’s implementation is used.

---

## Build & Upload

1. Install the Arduino Nano Matter board support in the Arduino IDE.
2. Install one DallasTemperature provider.
3. Ensure the build uses the core’s OneWire; disable any duplicate user copy if necessary.
4. Open `HeatPipeTemp.ino`, select Board: Arduino Nano Matter, and upload.

On first boot you’ll get a framed pairing block once, then compact readings every 3 minutes.

---

## Behavior

- Serial prints
  - On boot: device banner, DS18B20 readiness, framed pairing block printed once
  - On Matter or Thread state changes: single compact line
  - On each reading: `Temp: xx.xx °C | Batt: x.xxx V[ | pub]`
- LEDs (5-second indicate window after each reading)
  - Temperature above 28.0 °C: solid RED
  - Temperature at or below 28.0 °C: smooth BLUE 3‑blink burst pattern
  - Single‑LED boards use the built‑in LED as a simplified indicator
- Matter publishing (if Temperature cluster is present)
  - Publishes when change ≥ 0.2 °C or at least every 10 minutes
- Button
  - Short press: reprints the manual pairing code (framed)
  - Long press (≥ 6 s): fast WHITE blink ×6, then decommission (factory reset)

---

## Example Serial Output

```
=== Matter Pairing Info ===
Commissioned: yes
Thread link:  connected
Manual pairing code:
+---------------+
| 3497 011 2332 |
+---------------+
============================
[Matter] Commissioned: yes | Thread: connected
Temp: 23.62 °C | Batt: 4.018 V | pub
```

If the sensor is missing or wiring is wrong:

```
[DS18B20] not detected on D5 (check wiring & 4.7k pull-up)
Temp: --.- °C | Batt: 4.018 V
```

---

## Troubleshooting

- `Temp: --.- °C` or DS18B20 not detected
  - Confirm the 4.7 kΩ pull-up from D5 to 3V3.
  - Use powered mode (VDD to 3V3), not parasite.
  - Ensure DQ goes to D5, not A5.
  - Verify grounds and connector polarity.
  - Try another GPIO such as D2: change `DS18B20_PIN` in the sketch to `2` and rebuild.
- Linker or ABI errors around DallasTemperature or OneWire
  - Keep only one DallasTemperature provider installed.
  - Prefer the core’s OneWire for Nano Matter; disable any duplicate in your sketchbook.
- Matter Temperature cluster missing
  - The sketch still works; you’ll see a notice that the Temperature cluster isn’t available. Serial prints and battery sensing continue normally.

---

## Configuration Tips

- Change `TEMP_THRESHOLD_C` to adjust the LED threshold.
- Change `MEASUREMENT_INTERVAL_MS` to alter the cadence.
- Keep ADC calibration constants unchanged unless you re-characterize the board’s 3V3 rail.
- LED polarity is abstracted; the sketch auto-handles active‑LOW defaults on Nano Matter.

---

## Notes

- The framed pairing block prints only once per boot to keep the log clean.
- Subsequent state changes use single-line logs.
- Battery reading prime and averaging are preserved to stabilize the first sample.

---

## License

MIT License

Copyright (c) 2025 <Your Name>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the Software), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
