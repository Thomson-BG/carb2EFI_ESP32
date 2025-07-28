# 🛠️ EFI Retrofit HOWTO (ESP32, 7" RA8875 TFT, Optional GPS Speedometer)

## 1. Hardware Setup

### Components
- ESP32 Dev Board (e.g., ESP32-WROOM-32)
- 7" TFT RA8875 display (SPI)
- MAP sensor
- O2/lambda sensor (narrowband or wideband)
- High-impedance injector, EFI pump, pressure regulator
- Optoisolator for points/coil negative RPM
- Power supply (12V to 5/3.3V)
- MOSFET injector driver
- GPS module (e.g., u-blox NEO-6M, NEO-7M, NEO-M8N, etc., 9600 baud, 3.3V or 5V logic)
- Wiring, fuses, project box

### Display Wiring
| Signal    | ESP32 Pin | RA8875 Pin |
|-----------|-----------|------------|
| MOSI      | GPIO23    | SDI        |
| MISO      | GPIO19    | SDO        |
| SCK       | GPIO18    | SCK        |
| CS        | GPIO5     | CS         |
| RST       | GPIO4     | RST        |
| GND/3.3V  | GND/3.3V  | GND/VCC    |

### Sensors
- Connect MAP sensor output to GPIO36 (VP)
- Connect O2 sensor output to GPIO39 (VN)
- Wire injector driver to GPIO18 (MOSFET)
- Coil negative (via optoisolator) to GPIO4

### GPS Module Wiring
| GPS Pin | ESP32 Pin | Note                     |
|---------|-----------|--------------------------|
| TX      | GPIO16    | Connect GPS TX to ESP32  |
| RX      | GPIO17    | (optional, not used)     |
| VCC     | 3.3V/5V   | As per module spec       |
| GND     | GND       |                          |

---

## 2. Software Setup

- Install Arduino-ESP32 core
- Install [Adafruit_RA8875 library](https://github.com/adafruit/Adafruit_RA8875)
- Install [TinyGPSPlus library](https://github.com/mikalhart/TinyGPSPlus)
- Clone/copy this codebase

### Build & Upload
- Select ESP32 board in Arduino IDE or PlatformIO
- Upload to ESP32

---

## 3. Tuning & Operation

- With all wiring complete, power on ESP32
- TFT displays live RPM, MAP, injector pulse, AFR, and GPS speed (if GPS fix)
- If no GPS fix, "GPS: NO FIX" is displayed
- Warnings shown for lean/rich, high RPM, MAP sensor error
- Adjust VE, injector size, or thresholds in code if needed

---

## 4. Expanding Features

- Add Wi-Fi or Bluetooth for wireless logging, tuning, OTA updates.
- Add touch support for on-screen tuning.
- Add SD logging for trip/diagnostic logs.

---

## 5. Troubleshooting

- Blank TFT: Check wiring, power, CS/RST pins, library.
- No RPM: Check points/coil wiring, optoisolator, input pin, interrupt.
- Injector not firing: Check MOSFET driver, PWM channel, power.
- Unstable readings: Check sensor supply voltage and grounds.
- No GPS speed: Check GPS module power, wiring (TX to GPIO16), sky view.

---

**Enjoy your ESP32-powered EFI + 7" dashboard with GPS speedometer!**
