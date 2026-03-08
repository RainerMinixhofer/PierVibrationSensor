# miniSEED Logger Library

Records sensor data in miniSEED (SEED - Standard for the Exchange of Earthquake Data) format compatible with Raspberry Shake and seismic analysis tools.

## Features

- **Raspberry Shake Compatible**: Data format directly compatible with Raspberry Shake ecosystem
- **SEED Standard**: Implements SEED format for seismic data interchange
- **Multi-channel Recording**: Support for velocity and acceleration channels
- **Steim1 Compression**: Supports Steim1 compressed format for efficient storage
- **SD Card Storage**: Records data to SD card with timestamp-based filenames
- **Metadata**: Includes sampling rate, station code, channel information, and timing data

## Usage

### Initialization

```cpp
#include "miniseed_logger.h"

MinISeedLogger miniSeedLogger;

// In setup():
miniSeedLogger.init(&file, "PIER1", "XX", "/data/miniseed");
```

### Recording Samples

```cpp
// Record single sample
time_t now = time(nullptr);
miniSeedLogger.recordSample("VEL_X", adcValue0, 500, now);

// Record batch of samples (more efficient)
int32_t samples[256];
miniSeedLogger.recordBatch("VEL_X", samples, 256, 500, now);
```

### Enabling/Disabling Recording

```cpp
// Enable recording
miniSeedLogger.setRecordingEnabled(true);

// Check status
if (miniSeedLogger.isRecordingEnabled()) {
  // Recording is active
}

// Flush data to disk
miniSeedLogger.flush();

// Close recording
miniSeedLogger.closeFile();
```

## HTTP Control

Control miniSEED recording via web interface:

```
GET /miniseed?enable=1  // Enable recording
GET /miniseed?enable=0  // Disable recording
```

Returns JSON response:
```json
{"miniSeedRecording": true}
```

## Data Format

### File Naming
- Format: `YYYYMMDD.MSE` (one daily file per day, all channels combined)
- Example: `20260308.MSE`
- Location: `/data/miniseed/` on SD card
- Each daily file contains multiple traces (one per channel)
- New file created automatically at midnight (UTC)

### Channel Codes
- `VEL_X` / `VEL_Y` - Velocity channels (ADS1256) → SEED codes: HPN, HPE
- `ACC_X` / `ACC_Y` / `ACC_Z` - Acceleration channels (ICM20948) → SEED codes: HNN, HNE, HNZ
- `GYRO_X` / `GYRO_Y` / `GYRO_Z` - Gyroscope channels (ICM20948) → SEED codes: HJN, HJE, HJZ

### SEED Header Format
- Record Length: 512 bytes per record (SEED 2.4 standard)
- Samples per record: Up to 116 samples (int32_t format)
- Encoding: Uncompressed INT32 (SEED encoding 3)
- Byte Order: Big-endian (network order)
- Quality Flags: Data quality metadata
- Multiple records appended to daily file as data is recorded

## Compatibility

- **Raspberry Shake**: Direct import of miniSEED files
- **ObsPy**: Python library for seismic data analysis
- **SEED Standard Tools**: Any tool supporting miniSEED format
- **SeisComP**: Seismic data management system

## Notes

- Data is automatically time-stamped based on system time
- Sampling rate must be specified for each channel
- Supports batch recording for improved efficiency on high-speed data
- Data is flushed to disk periodically (every 5 seconds) and when buffers fill (116 samples)
- All channels for a given day are stored in a single file per the miniSEED standard
- Files automatically roll over to a new daily file at midnight UTC
