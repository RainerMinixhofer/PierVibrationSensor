/**
 * @file pin_config.h
 * @brief GPIO Pin Configuration for Pier Vibration Sensor
 * @details Pin assignments for Raspberry Pi Pico 2 W with peripheral direction
 *
 * Pin Direction Legend (relative to Pico):
 * - Output: Pico drives the signal
 * - Input: Pico reads the signal
 * - I/O: Bidirectional (SPI MISO/MOSI, I2C SDA)
 */

#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// ============================================================================
// TFT Display (ILI9341) - spi0 @ 25 MHz
// ============================================================================

#ifndef TFT_SPI_PORT
#define TFT_SPI_PORT 0 // Use SPI0
#endif
#define TFT_SPI_BPS 40 * 1000 * 1000      // 40 MHz SPI clock for ILI9341
#define TFT_SPI_BPS_READ 10 * 1000 * 1000 // 10 MHz SPI clock for ILI9341
#ifndef TFT_SCLK
#define TFT_SCLK 2 // GP2  - Output: SPI Clock
#endif
#ifndef TFT_MOSI
#define TFT_MOSI 3 // GP3  - Output: SPI MOSI (Master Out Slave In)
#endif
#ifndef TFT_MISO
#define TFT_MISO 0 // GP0  - Input:  SPI MISO (Master In Slave Out)
#endif
#ifndef TFT_CS
#define TFT_CS 1 // GP1  - Output: Chip Select
#endif
#ifndef TFT_DC
#define TFT_DC 4 // GP4  - Output: Data/Command Select
#endif
#ifndef TFT_RESET
#define TFT_RESET 6 // GP6  - Output: Reset (active low)
#endif
#ifndef TFT_LED
#define TFT_LED 7 // GP7  - Output: Backlight Control (PWM capable)
#endif

// ============================================================================
// Touchscreen (XPT2046) - spi0 @ 1 MHz
// ============================================================================

#define TOUCH_SPI_PORT TFT_SPI_PORT   // Shared with TFT and Wiznet
#define TOUCH_SPI_BPS 1 * 1000 * 1000 // 1 MHz SPI clock for XPT2046
#define TOUCH_CLK TFT_SCLK            // GP2  - Output: SPI Clock (shared)
#define TOUCH_DIN TFT_MOSI            // GP3  - Output: SPI MOSI (shared)
#define TOUCH_DO TFT_MISO             // GP0  - Input:  SPI MISO (shared)
#ifndef TOUCH_CS
#define TOUCH_CS 26 // GP26 - Output: Chip Select
#endif
#define TOUCH_IRQ 27 // GP27 - Input:  Touch Interrupt (active low)

// ============================================================================
// Wiznet (W5500/W5100S Ethernet) - spi0 @ 20 MHz
// ============================================================================

#define WIZNET_SPI_PORT TFT_SPI_PORT    // Shared with TFT and Touchscreen
#define WIZNET_SPI_BPS 20 * 1000 * 1000 // 20    MHz SPI clock for Wiznet
#define WIZNET_SCLK TFT_SCLK            // GP2  - Output: SPI Clock (shared)
#define WIZNET_MOSI TFT_MOSI            // GP3  - Output: SPI MOSI (shared)
#define WIZNET_MISO TFT_MISO            // GP0  - Input:  SPI MISO (shared)
#define WIZNET_SPI_CS 17                // GP17 - Output: Chip Select
#define WIZNET_RST 20                   // GP20 - Output: Hardware Reset (active low)
#define WIZNET_INT 21                   // GP21 - Input:  Interrupt (active low, e.g., Wake-on-LAN)

// Define SPI_ETHERNET_SETTINGS for Ethernet_Generic library
// (w5100.h will redefine this, so we use a wrapper macro for the intended value)
#define WIZNET_SPI_ETHERNET_SETTINGS SPISettings(WIZNET_SPI_BPS, MSBFIRST, SPI_MODE0)

// ============================================================================
// SD Card Interface - software SPI @ 5 MHz
// ============================================================================

#define SD_SPI_PORT -1             // -1 = software SPI, 0 = hardware SPI0, 1 = hardware SPI1
#define SD_SPI_BPS_INIT 400 * 1000 // 400 kHz for SD card initialization
#define SD_SPI_BPS 5 * 1000 * 1000 // 5 MHz for SD card
#define SD_CLK 18                  // GP18  - Output: SPI Clock (shared)
#define SD_MOSI 22                 // GP22  - Output: SPI MOSI (shared)
#define SD_MISO 19                 // GP19  - Input:  SPI MISO (shared)
#define SD_CS 16                   // GP16 - Output: Chip Select

// ============================================================================
// Analog-to-Digital Converter (ADS1256) - spi1 @ 1.4 MHz
// ============================================================================
// ADS1256 @ 1.4 MHz

// SPI1 Port and Speed
#define ADS1256_SPI_PORT SPI1
#define ADS1256_SPI_BPS 1.4 * 1000 * 1000 // 1.4 MHz SPI clock for ADS1256

#define ADS1256_SCLK 10 // GP10 - Output: SPI Clock
#define ADS1256_MOSI 11 // GP11 - Output: SPI MOSI
#define ADS1256_MISO 12 // GP12 - Input:  SPI MISO
#define ADS1256_CS 13   // GP13 - Output: Chip Select
#define ADS1256_RST -1  // Not used (set to -1)
#define ADS1256_SYNC 14 // GP14 - Output: Sync/Power Down
#define ADS1256_DRDY 15 // GP15 - Input:  Data Ready (active low)

// ============================================================================
// 9D Sensor (ICM20948) - i2c0 @ 400 kHz
// ============================================================================

#define ICM20948_I2C_PORT Wire      // Connected to I2C0
#define ICM20948_I2C_BPS 400 * 1000 // 400 kHz I2C clock
#define ICM20948_I2C_SDA 8          // GP8  - I/O: I2C Data
#define ICM20948_I2C_SCL 9          // GP9  - Output: I2C Clock (with pull-up)
#define ICM20948_INT 5              // GP5  - Input:  Interrupt Output
#define ICM20948_ADDR 0x69          // I2C address with AD0 pin high

// ============================================================================
// Pin Summary by GPIO Number
// ============================================================================
/*
GP0  - TFT_MISO / TOUCH_DO / WIZNET_MISO (Input)
GP1  - TFT_CS (Output)
GP2  - TFT_SCLK / TOUCH_CLK / WIZNET_SCLK (Output)
GP3  - TFT_MOSI / TOUCH_DIN / WIZNET_MOSI (Output)
GP4  - TFT_DC (Output)
GP5  - ICM20948_INT (Input)
GP6  - TFT_RESET (Output)
GP7  - TFT_LED (Output, PWM)
GP8  - ICM20948_I2C_SDA (I/O)
GP9  - ICM20948_I2C_SCL (Output)
GP10 - ADS1256_SCLK (Output)
GP11 - ADS1256_MOSI (Output)
GP12 - ADS1256_MISO (Input)
GP13 - ADS1256_CS (Output)
GP14 - ADS1256_SYNC (Output)
GP15 - ADS1256_DRDY (Input)
GP16 - SD_CS (Output)
GP17 - WIZNET_SPI_CS (Output)
GP18 - SD_CLK (Output)
GP19 - SD_MISO (Input)
GP20 - WIZNET_RST (Output)
GP21 - WIZNET_INT (Input)
GP22 - SD_MOSI (Output)
GP26 - TOUCH_CS (Output)
GP27 - TOUCH_IRQ (Input)
*/

#endif // PIN_CONFIG_H
