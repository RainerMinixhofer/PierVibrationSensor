# DigPot C++ Library

This library migrates the Python tools in `ToDo/DigPotTest/DigPot.py` and `ToDo/DigPotTest/DigPotCLI.py` into reusable C++ classes for Arduino/PlatformIO firmware.

## Classes

- `digpot::TPL0102`: register-level access and command helpers for TPL0102.
- `digpot::AD5142A`: command-level API for AD5142A.

## Included Behaviors

- Address probing and candidate-based discovery.
- TPL0102 commands from CLI (`wiper-read`, `wiper-write`, `nvram-read`, `nvram-write`, `acr-read`, `acr-write`, `shutdown`, `wip-poll`).
- AD5142A commands from CLI (`nop`, `rdac-read`, `rdac-write`, `input-write`, `readback`, `linear-step`, `6db-step`, `input-to-rdac`, `rdac-to-eeprom`, `eeprom-to-rdac`, `eeprom-write`, `top-scale`, `bottom-scale`, `reset`, `shutdown`, `control-write`).
- Voltage divider helper functions (`voltdiv-write` and `voltdiv-read` equivalent calculations).

## Example

```cpp
#include <Wire.h>
#include <DigPot.h>

using namespace digpot;

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  uint8_t adAddr = 0;
  if (discoverSingleAddress(Wire, AD5142A::kCandidateAddresses, AD5142A::kCandidateAddressCount, adAddr)) {
    AD5142A ad(Wire, adAddr);
    ad.writeRdac(0, 0x80);
  }
}
```
