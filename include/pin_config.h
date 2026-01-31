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
// SPI0 Bus - Shared by TFT Display, Touchscreen, and SD Card
// ============================================================================
// All devices on SPI0 @ 10 MHz

// TFT Display (ILI9341) Pins
#define TFT_SCLK 2 // GP2  - Output: SPI Clock
#define TFT_MOSI 3 // GP3  - Output: SPI MOSI (Master Out Slave In)
#define TFT_MISO 0 // GP0  - Input:  SPI MISO (Master In Slave Out)
#define TFT_CS 1   // GP1  - Output: Chip Select
#define TFT_DC 4   // GP4  - Output: Data/Command Select
#define TFT_RST 6  // GP6  - Output: Reset (active low)
#define TFT_LED 7  // GP7  - Output: Backlight Control (PWM capable)

// Touchscreen (XPT2046) Pins - Shared SPI0
#define TOUCH_CLK TFT_SCLK // GP2  - Output: SPI Clock (shared)
#define TOUCH_DIN TFT_MOSI // GP3  - Output: SPI MOSI (shared)
#define TOUCH_DO TFT_MISO  // GP0  - Input:  SPI MISO (shared)
#define TOUCH_CS 22        // GP22 - Output: Chip Select
#define TOUCH_IRQ 27       // GP27 - Input:  Touch Interrupt (active low)

// SD Card Pins - Shared SPI0
#define SD_CLK 18  // GP18  - Output: SPI Clock (shared)
#define SD_MOSI 26 // GP26  - Output: SPI MOSI (shared)
#define SD_MISO 19 // GP19  - Input:  SPI MISO (shared)
#define SD_CS 16   // GP16 - Output: Chip Select

// ============================================================================
// SPI1 Bus - ADS1256 ADC
// ============================================================================
// ADS1256 @ 1.4 MHz

#define ADS1256_SPI_SCK 10  // GP10 - Output: SPI Clock
#define ADS1256_SPI_MOSI 11 // GP11 - Output: SPI MOSI
#define ADS1256_SPI_MISO 12 // GP12 - Input:  SPI MISO
#define ADS1256_SPI_CS 13   // GP13 - Output: Chip Select
#define ADS1256_SPI_RST -1  // Not used (set to -1)
#define ADS1256_SYNC 14     // GP14 - Output: Sync/Power Down
#define ADS1256_DRDY 15     // GP15 - Input:  Data Ready (active low)

// ============================================================================
// I2C0 Bus - ICM20948 IMU
// ============================================================================
// ICM20948 @ 400 kHz

#define ICM20948_I2C_SDA 8 // GP8  - I/O: I2C Data
#define ICM20948_I2C_SCL 9 // GP9  - Output: I2C Clock (with pull-up)
#define ICM20948_INT 5     // GP5  - Input:  Interrupt Output

// ============================================================================
// Wiznet W5500/W5100S Ethernet (SPI0)
// ============================================================================
// Wiznet @ 2 MHz

#define WIZNET_SPI_SCK TFT_SCLK  // GP2  - Output: SPI Clock
#define WIZNET_SPI_MOSI TFT_MOSI // GP3  - Output: SPI MOSI
#define WIZNET_SPI_MISO TFT_MISO // GP0  - Input:  SPI MISO
#define WIZNET_SPI_CS 17         // GP17 - Output: Chip Select
#define WIZNET_RST 20            // GP20 - Output: Hardware Reset (active low)
#define WIZNET_INT 21            // GP21 - Input:  Interrupt (active low, e.g., Wake-on-LAN)

// ============================================================================
// Pin Summary by GPIO Number
// ============================================================================
/*
GP0  - TFT_MISO / TOUCH_DO / WIZNET_SPI_MISO (Input)
GP1  - TFT_CS (Output)
GP2  - TFT_SCLK / TOUCH_CLK / WIZNET_SPI_SCK (Output)
GP3  - TFT_MOSI / TOUCH_DIN / WIZNET_SPI_MOSI (Output)
GP4  - TFT_DC (Output)
GP5  - ICM20948_INT (Input)
GP6  - TFT_RST (Output)
GP7  - TFT_LED (Output, PWM)
GP8  - ICM20948_I2C_SDA (I/O)
GP9  - ICM20948_I2C_SCL (Output)
GP10 - ADS1256_SPI_SCK (Output)
GP11 - ADS1256_SPI_MOSI (Output)
GP12 - ADS1256_SPI_MISO (Input)
GP13 - ADS1256_SPI_CS (Output)
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
