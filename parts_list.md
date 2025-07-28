# 🧾 Parts List: ESP32 Single-Point EFI Retrofit

## Electronics
- ESP32 Dev Board (e.g., ESP32-WROOM-32)
- Logic-level MOSFET (e.g., IRLZ44N) for injector
- Flyback diode (e.g., 1N5408, for injector)
- Optoisolator (e.g., 4N35 or PC817, for points/coil negative to ESP32)
- Voltage divider resistors (for optoisolator input)
- 5V or 3.3V regulator (if needed for ESP32/sensors)
- Automotive wire, connectors, relays, fuses

## Sensors
- MAP Sensor (e.g., GM 1-bar MAP, 3-wire)
- O2 Sensor (narrowband or wideband, Bosch, NTK, etc.) [optional but recommended]

## Fuel System
- High-impedance fuel injector (sized for your engine, e.g., 170cc/min for 5.7L V8)
- High-pressure external or in-tank EFI fuel pump (40-60 psi output)
- EFI fuel pressure regulator (return type, 3 bar typical)
- Fuel filter (EFI rated)
- EFI fuel hose and clamps
- Injector bung (for welding or brazing into intake manifold)

## Hardware & Misc
- Throttle position sensor [not required for this setup, but optional upgrade]
- Intake manifold modification (welding/brazing or adapter for injector)
- Hose fittings, T-fittings, clamps
- Project box for electronics
- Heat shrink, loom, zip ties

## Tools
- Soldering iron, wire strippers
- Multimeter
- Oscilloscope (optional, for signal debugging)
- Drill, welder/brazer for manifold mods

## Example Suppliers
- [Digi-Key](https://www.digikey.com/)
- [Mouser Electronics](https://www.mouser.com/)
- [Summit Racing](https://www.summitracing.com/) (injectors, pumps)
- [EFI System Pro](https://www.efisystempro.com/) (EFI supplies)

---

**Total estimated cost:** $150–$350 USD (depending on injector, pump, and sensor sourcing)
