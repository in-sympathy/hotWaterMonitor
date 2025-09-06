# Nano Matter — HeatPipeTemp

Minimal temperature + battery monitor for **Arduino Nano Matter** with a **DS18B20** and a simple 1S Li‑ion divider.  
- Reads temperature every 3 minutes (9‑bit conversion for speed).  
- Publishes to Matter when Δ≥0.2 °C or at least every 10 min.  
- Shows a **5 s LED window** after each reading:  
  - ≤ **30 °C** → smooth **BLUE triple‑blink** bursts (repeat).  
  - > **30 °C** → solid **RED**.  
- Prints one compact line to Serial: `Temp: xx.xx °C | Batt: x.xxx V[ | pub]`  
- Boot prints a single framed **manual pairing code** block.  
- Button: **short press** reprints the framed code, **long press (≥6 s)** decommissions.

---

## Download

**Direct sketch:** [HeatPipeTemp.ino](sandbox:/mnt/data/HeatPipeTemp.ino)

> Save as `HeatPipeTemp/HeatPipeTemp.ino` in your Arduino sketchbook.

---

## Hardware

- **Board:** Arduino Nano Matter (EFR32). See Nano Matter tutorials for setup. [docs link in chat]
- **Sensor:** DS18B20 on **D5** (OneWire).  
- **Battery sense:** A0 via divider  
  ```
  Pack+ ─ R1(1.0 MΩ) ──+── A0
                       │
                     C1 100 nF → GND
                       │
                     R2 1.0 MΩ → GND
  ```
- **LEDs:** Uses the on‑board RGB (active‑LOW on Nano Matter).

## Libraries

Install via Library Manager (Sketch → Include Library → Manage Libraries…):
- **OneWire** (Paul Stoffregen).  
- **DallasTemperature** (Miles Burton & contributors).

## Build

1. In **Tools → Board**, select **Arduino Nano Matter**.
2. Open `HeatPipeTemp.ino` and **Upload**.
3. Serial monitor at **115200 baud**.

> The code uses `analogReadResolution(12)` where supported.

## Behavior details

- **Voltage math**: 12‑bit ADC against your measured **3V3 = 3.254 V**. Divider is 1 MΩ / 1 MΩ → `Vpack ≈ 2 × VA0` (plus calibration constants left at 1.000/0.000).  
- **LEDs** are only active during the 5 s “indicate” window to save power.

## Troubleshooting

- If the **first battery reading** looks off after power‑up, make sure the divider is wired as shown and that A0 is not floating; this sketch **pre‑primes** the ADC node before averaging.
- If Matter doesn’t show updates, confirm Thread is connected; a state change will print one line:  
  `[Matter] Commissioned: yes|no | Thread: connected|not connected`.

---

© You. MIT‑ish—feel free to use and adapt.
