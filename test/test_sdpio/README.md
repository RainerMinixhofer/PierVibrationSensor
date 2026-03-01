# test_sd3 - SD Card Benchmark with PIO SPI

This test directory implements SD card operations using the Raspberry Pi Pico's Programmable I/O (PIO) for hardware-accelerated SPI communication.

## Overview

The implementation uses:
- **PIO-based SPI**: Hardware acceleration through PIO state machines for optimal performance
- **Custom SD driver**: Direct SD card protocol implementation without external libraries
- **Benchmark suite**: Comprehensive speed tests across frequencies from 1-20 MHz
- **Reference implementation**: Based on proven pico_fatfs library architecture

## Architecture

### PIO SPI Implementation

The PIO SPI uses a 2-instruction program running at 4 clock cycles per bit:
1. **Output data** (1 cycle) + SCK low (1 cycle delay)
2. **Input data** (1 cycle) + SCK high (1 cycle delay)

This provides a theoretical maximum speed of:
- Clock: 125 MHz (RP2040 system clock) / 4 = 31.25 MHz SPI clock
- Data rate: 31.25 MHz × 1 bit = 31.25 Mbit/s theoretical max

The implementation includes:
- `spi.pio`: PIO assembly program for CPHA=0 SPI
- `pio_spi.cpp/h`: Transfer functions (write, read, write-read)
- `sd_pio.cpp/h`: SD card driver using PIO SPI backend

### SD Card Driver

Features:
- SD Card v1/v2 support (SDSC, SDHC, SDXC)
- Single and multiple block read/write
- Card identification (OCR, CID, CSD, SSR registers)
- Automatic card size detection
- Block vs byte addressing mode

## Pin Configuration

Uses pins from `pin_config.h`:
- **CS (Chip Select)**: SD_CS (GPIO 16)
- **SCK (Clock)**: SD_CLK (GPIO 18)
- **MOSI (Master Out)**: SD_MOSI (GPIO 22)
- **MISO (Master In)**: SD_MISO (GPIO 19)

Note: These pins don't match hardware SPI0/SPI1 constraints, making PIO the optimal choice.

## Test Suite

### Test 1: SD Card Initialization
- Initializes SD card at 10 MHz
- Identifies card type (SD v1/v2, MMC)
- Verifies successful communication

### Test 2: Card Information
- Reads OCR (Operating Conditions Register)
- Reads CSD (Card Specific Data) - determines capacity
- Reads CID (Card Identification) - manufacturer info
- Calculates and displays card size

### Test 3: Single Block Operations
- Writes a 512-byte block with test pattern
- Reads the block back
- Verifies data integrity

### Test 4: Multiple Block Operations
- Writes 10 consecutive blocks (5 KB)
- Reads them back
- Verifies all data

### Test 5: Write Speed Benchmark
- Writes 1024 blocks (512 KB)
- Measures total time
- Calculates write speed in KB/s and Mbit/s

### Test 6: Read Speed Benchmark
- Reads 1024 blocks (512 KB)
- Measures total time
- Calculates read speed in KB/s and Mbit/s

### Test 7: Multi-Block Speed
- Tests CMD25 (write multiple) and CMD18 (read multiple)
- Compares performance vs single-block operations
- 100 iterations of 10-block transfers (50 KB total)

### Test 8: Frequency Sweep
- Tests speeds from 1 MHz to 20 MHz
- Displays read/write performance at each frequency
- Helps identify optimal operating frequency
- Tests: 1, 2, 4, 5, 8, 10, 12.5, 15, 20 MHz

Frequency | Expected Read | Expected Write
----------|---------------|---------------
1 MHz     | ~0.8 Mbit/s   | ~0.6 Mbit/s
4 MHz     | ~3.0 Mbit/s   | ~2.5 Mbit/s
10 MHz    | ~7.0 Mbit/s   | ~6.0 Mbit/s
20 MHz    | ~12 Mbit/s    | ~10 Mbit/s

## Expected Performance

Based on the reference pico_fatfs implementation:
- **Read speeds**: 12-20 Mbit/s (1.5-2.5 MB/s)
- **Write speeds**: 6-12 Mbit/s (0.8-1.5 MB/s)

This represents a **3-5x improvement** over test_sd2's Fast GPIO implementation (4.3 Mbit/s).

## Comparison with test_sd2

| Feature | test_sd2 | test_sd3 |
|---------|----------|----------|
| Implementation | Software SPI with Fast GPIO | PIO hardware SPI |
| Max Speed | 4.3 Mbit/s | 12-20 Mbit/s |
| CPU Usage | High (software loops) | Low (DMA-like PIO) |
| Frequency Limit | ~10 MHz (unstable) | 20+ MHz |
| Code Complexity | Simple, portable | RP2040-specific |

## Building and Running

```bash
# Build test_sd3
pio test -e pico2w -f test_sd3

# Upload and monitor
pio test -e pico2w -f test_sd3 --verbose
```

## Technical Details

### PIO State Machine Configuration

- **Clock divider**: `clock_get_hz(clk_sys) / (freq_hz * 4.0f)`
- **Autopush/Autopull**: 8-bit threshold
- **Shift direction**: MSB first (SD card standard)
- **Input sync bypass**: Enabled for minimal latency on MISO

### SD Card Protocol

- **Initialization**: 400 kHz for compatibility
- **Operation**: Configurable up to 20 MHz
- **Command format**: 6-byte packets with CRC
- **Data blocks**: 512 bytes with 2-byte CRC
- **Timeout**: 500 ms for write operations

### PIO Resource Usage

- **PIO instance**: pio0
- **State machine**: SM0
- **Program size**: 2 instructions
- **FIFO depth**: 4 entries TX, 4 entries RX (standard)

## Troubleshooting

### Initialization Fails
- Check SD card is inserted properly
- Verify pin connections
- Try lower initial frequency (400 kHz is mandatory)
- Some cards require longer initialization delay

### Speed Lower Than Expected
- SD card speed class affects performance (Class 4 vs Class 10)
- Card type matters (SDSC < SDHC < SDXC typically)
- Verify no compilation optimizations are disabled
- Check for debugger overhead if attached

### Data Corruption
- Verify stable power supply (SD cards draw significant current)
- Check for loose connections
- Try lower frequency
- Some cards are sensitive to signal integrity

## Future Enhancements

1. **DMA Integration**: Use DMA with PIO for zero-CPU transfers
2. **Dual PIO**: Use both TX and RX PIO for full-duplex optimization
3. **CPHA=1 Support**: Add alternate clock phase for compatibility
4. **Hardware CRC**: Implement CRC calculation in PIO
5. **SDIO Mode**: 4-bit parallel SD interface for 4x speed

## References

- [RP2040 Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf) - PIO Chapter
- [SD Card Specification](https://www.sdcard.org/downloads/pls/) - Physical Layer Simplified Specification
- [pico_fatfs](https://github.com/ExtremeElectronics/RC2040/tree/main/pico_fatfs-main) - Reference implementation

## License

This implementation is based on public SD card specifications and the BSD-3-Clause licensed pico_fatfs library.
