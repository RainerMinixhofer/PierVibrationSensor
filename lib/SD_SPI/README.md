# SD Card Test Suite - test_sd

## Overview

This test directory contains a basic SD card test implementation using a custom SPI driver that directly interfaces with the SD card hardware. It provides low-level block read/write operations.

## Pin Configuration

The tests use SD card pin definitions from `pin_config.h`:

- **SD_CS**: GP16 (Chip Select)
- **SD_CLK**: GP18 (SPI Clock)
- **SD_MOSI**: GP22 (Master Out Slave In)
- **SD_MISO**: GP19 (Master In Slave Out)
- **SPI Mode**: Software SPI (SD_SPI_PORT = -1)

## Test Cases

### 1. SD Card Initialization Test
- Initializes the SD card via SPI
- Sends initialization sequence (CMD0, CMD8, ACMD41, etc.)
- Detects card type (SDSC, SDHC/SDXC, or MMC)
- Reports initialization status

### 2. SD Card Block Write Test (Disabled)
- Writes a 512-byte test block to sector 100
- Test data includes a signature string and sequential pattern
- Verifies write operation success

### 3. SD Card Block Read Test (Disabled)
- Reads the 512-byte block from sector 100
- Displays the first 64 bytes in hexadecimal
- Verifies read operation success

### 4. SD Card Read/Write Verify Test (Disabled)
- Compares the written and read data
- Verifies signature string match
- Reports any mismatches in the 512-byte block
- Ensures data integrity

### 5. SD Card Filesystem Information Test
- Reads and parses MBR partition table
- Identifies filesystem type (FAT12/16/32, exFAT)
- Extracts boot sector parameters
- Reports capacity and cluster information
- Displays filesystem details (sectors per cluster, FAT size, root directory location)

### 6. SD Card Filesystem and Directory Test
- Identifies filesystem type
- Locates root directory cluster (FAT32) or region (FAT12/16)
- Reads and parses directory entries
- Supports Long File Names (LFN/VFAT) with Unicode
- Displays directory listing with file names, sizes, attributes, and timestamps

### 7. SD Card File Write Test
- Writes 4KB test file to disk
- Measures write time and calculates speed in kbit/s
- Uses test sectors 100-200 to avoid filesystem corruption
- Verifies bytes written

### 8. SD Card File Read Test
- Reads 4KB test file from disk
- Measures read time and calculates speed in kbit/s
- Displays first 64 bytes of read data
- Verifies bytes read

### 9. SD Card File Write/Read/Verify Test
- Writes 8KB test file with unique pattern
- Reads file back into separate buffer
- Measures both write and read speeds
- Verifies data integrity byte-by-byte
- Reports any mismatches

## Implementation Details

### Custom SPI Driver (`sd_spi.cpp/h`)

The implementation includes:
- Software bit-banged SPI communication with performance optimizations
- SD card initialization sequence
- Block read/write operations (512 bytes per block)
- Support for SDSC, SDHC/SDXC, and MMC cards
- MBR partition table parsing
- FAT12/16/32 and exFAT filesystem detection
- Boot sector parameter extraction
- Root directory reading for all FAT types
- Long File Name (LFN/VFAT) support with Unicode handling
- File write/read/delete operations with speed measurement
- SPI frequency benchmark test (100 kHz to 10 MHz)
- Basic error handling

### Performance Optimizations

**RP2040-Specific Optimizations:**
The code automatically enables fast GPIO access on RP2040 (Raspberry Pi Pico):
- **Direct port manipulation** using `sio_hw` registers instead of `digitalWrite/digitalRead`
- **Manual loop unrolling** eliminates loop overhead in the critical SPI transfer path
- **Inline operations** reduce function call overhead

**Performance Comparison:**
- Standard Arduino digitalWrite: ~512 kbit/s maximum
- Optimized direct port access: 1-2 Mbit/s (2-4x faster)

The `USE_FAST_GPIO` flag is automatically enabled for RP2040 targets. To disable optimizations and use portable code, comment out the `#define USE_FAST_GPIO 1` line in sd_spi.cpp.

### Key Features

1. **Low-Level Control**: Direct SPI bit manipulation for maximum control
2. **No External Dependencies**: Uses only Arduino core and pin_config.h
3. **Portable**: Can be adapted to different pin configurations
4. **Educational**: Shows the SD card protocol and FAT filesystem implementation
5. **Filesystem Support**: 
   - MBR partition table detection and parsing
   - FAT12/16/32 and exFAT identification
   - Long filename support (up to 255 characters)
   - Unicode/UTF-16 to ASCII conversion
6. **Performance Metrics**: Real-time speed measurement for read/write operations

## Building and Running

### Using PlatformIO

```bash
# Build the test
pio test -e test_sd

# Upload and run the test
pio test -e test_sd --upload-port COM3
```

### Expected Output

```
=================================================
SD Card Test Suite - Custom SPI Implementation
Using pin definitions from pin_config.h
=================================================

=== SD Card Initialization Test ===
Pin Configuration:
  SD_CS:   GP16
  SD_CLK:  GP18
  SD_MOSI: GP22
  SD_MISO: GP19
  SPI Mode: Software SPI

Initializing SD card...
SD card initialized successfully!
Card Status: 0x00

=== SD Card Block Write Test ===
Writing test block to sector 100...
Block write successful!

=== SD Card Block Read Test ===
Reading test block from sector 100...
Block read successful!
First 64 bytes of read data:
  50 69 63 6F 53 44 20 54 65 73 74 20 42 6C 6F 63
  6B 11 12 13 14 15 16 17 18 19 1A 1B 1C 1D 1E 1F
  20 21 22 23 24 25 26 27 28 29 2A 2B 2C 2D 2E 2F
  30 31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F

=== SD Card Read/Write Verify Test ===
Verifying read data matches written data...
Signature match: "PicoSD Test Block"
Data verification successful - all 512 bytes match!

=== Test Suite Completed ===
```

## Troubleshooting

### SD Card Not Detected

1. Check physical connections (verify pin wiring)
2. Ensure SD card is properly formatted (FAT16/FAT32)
3. Try a different SD card (some cards are more sensitive)
4. Reduce SPI clock speed if experiencing timing issues

### Read/Write Failures

1. Verify CS (Chip Select) is properly controlled
2. Check for proper voltage levels (3.3V for SD cards)
3. Ensure SD card is not write-protected
4. Try increasing delays in SPI bit-banging if timing is critical

### Data Verification Failures

1. Check for electrical noise or poor connections
2. Verify SD card is not being accessed by other devices
3. Ensure consistent power supply

## Comparison with test_sd

| Feature | test_sd | test_sd2 |
|---------|---------|----------|
| Library | SdFat | Custom SPI |
| Level | High-level file operations | Low-level block + filesystem ops |
| Abstraction | Filesystem (FAT) | Raw sectors + FAT parsing |
| Dependencies | SdFat library | None (Arduino core only) |
| Use Case | File I/O testing | Hardware + filesystem verification |
| Long Filenames | Yes | Yes (VFAT/LFN support) |
| Speed Measurement | No | Yes (read/write kbit/s) |

## Notes

- Tests 2-4 (block operations) are disabled by default to avoid corruption
- File write/read tests use sectors 100-200 as a test area
- The software SPI implementation is slower than hardware SPI but more flexible
- Long filename support handles Windows/Mac style filenames with Unicode
- Speed measurements help identify SD card performance characteristics
- For production use, consider using the SdFat library (test_sd) for filesystem operations
