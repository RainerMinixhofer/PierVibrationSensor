# PierVibrationSensor
Software and Hardware Info Repository for Vibration Sensor to monitor the pier of my private observatory
The sensor system consists of an MEMS 9-axis motion tracker module the [MPU-9250/6500](https://github.com/NelisW/myOpenHab/blob/master/docs/707-MPU-9250-9265%20IMU.md) and two [LGT-4.5 horizontal Geophones](http://longetequ.com/geophone/3.htm) amplified with an [AD620 instrument amplifier](https://protosupplies.com/product/ad620-instrumentation-amplifier-module/) and converted to digital with the [ADS1256 24bit ADC](https://github.com/Arda-Bildik/ADS1256_library).
Both sensor paths are controlled by an [Raspberry Pico 2 W](), programmed in C++. VSCode with PlatformIO Plugin is used.
It is planned to upload key vibration analysis data via MQTT into IOBroker. See [example](https://github.com/mats-bergstrom/DS18B20) for how to do this in MicroPython.
The pico_pdm_spectrogram.ino c++ file is holding an example code for doing a spectrogram with the pico 2 w. The Github repository is under https://github.com/dpwe/pico_pdm_spectrogram/tree/main

## Hardware
Microcontroller is a Raspberry Pico 2 W with a W5500 Ethernet Hat.

## Known Issues and Workarounds

### Adafruit_SPITFT MISO Pin Direction Bug (RP2040/RP2350)

**Problem:**
The Adafruit_SPITFT library (part of Adafruit-GFX-Library) has a bug affecting Raspberry Pi Pico/Pico 2 when using hardware SPI with `readcommand8()` or similar read functions. After calling SPI read commands (e.g., `tft.Adafruit_SPITFT::readcommand8()`), the MISO pin gets stuck configured as an **output** instead of an **input**, breaking subsequent SPI communications.

**Root Cause:**
The library calls `pinMode(swspi._miso, INPUT)` in `Adafruit_SPITFT.cpp` line 677, but this is only executed for software SPI (`TFT_SOFT_SPI`). For hardware SPI (`TFT_HARD_SPI`), the MISO pin direction is never properly restored to input after read operations. On RP2040/RP2350, this leaves the GPIO output enable (OE) bit set, causing the pin to drive the SPI bus instead of listening to it.

**Symptoms:**
- SPI reads return 0x00 or 0xFF after first `readcommand8()` call
- Shared SPI bus peripherals (Wiznet W5500, SD cards, touch controllers) stop working
- Display status registers cannot be read correctly
- `iobank0_hw->io[MISO_PIN].ctrl` shows function set to SPI (correct)
- `sio_hw->gpio_oe` shows MISO pin with output enable set (incorrect - should be 0)

**Workaround:**
After any calls to `readcommand8()` or other Adafruit read functions, manually reset the MISO pin direction:

```cpp
#include "hardware/gpio.h"

// Example: Reading ILI9341 display status
uint8_t stat1 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 0);
uint8_t stat2 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 1);

// WORKAROUND: Force MISO back to input after Adafruit read commands
gpio_set_dir(TFT_MISO, GPIO_IN);  // Reset GPIO0 (MISO) to input
```

**Fix Verification:**
After applying the workaround, verify MISO is correctly configured:
```cpp
uint32_t gpio_oe = (sio_hw->gpio_oe >> TFT_MISO) & 0x1u;
Serial.printf("MISO OE bit: %d (should be 0 for input)\n", gpio_oe);
```

**Impact:**
This bug affects any RP2040/RP2350 project using:
- Adafruit ILI9341 TFT displays with hardware SPI
- Shared SPI bus with multiple peripherals
- Display diagnostic/status reads

**Long-term Solution:**
The proper fix would be to modify `Adafruit_SPITFT.cpp` to restore MISO pin direction after read operations for hardware SPI mode, similar to how it's done for software SPI. This requires patching the Adafruit-GFX-Library.

**References:**
- Affected file: `lib/Adafruit-GFX-Library/Adafruit_SPITFT.cpp`
- Relevant code: Line 677 (software SPI only)
- Test implementation: `test/test_tft/tests.cpp` line 389