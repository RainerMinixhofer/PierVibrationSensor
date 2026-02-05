/**
 * @file tft_setup.h
 * @brief Custom TFT_eSPI setup pulling pin definitions from pin_config.h
 * @details This setup bridges pin_config.h and TFT_eSPI library configuration
 */

// ===========================================================================
// Display Driver Chip
// ===========================================================================
#define ILI9341_DRIVER 1

// ===========================================================================
// Interface mode
// ===========================================================================
#define TFT_PARALLEL_8_BIT 0   // Not using parallel
#define RP2040_PIO_INTERFACE 0 // Not using PIO

// ===========================================================================
// Pin configuration - explicit values from pin_config.h
// ===========================================================================
// TFT Display pins
#define TFT_MISO 0 // GP0  - SPI MISO
#define TFT_MOSI 3 // GP3  - SPI MOSI
#define TFT_SCLK 2 // GP2  - SPI Clock
#define TFT_CS 1   // GP1  - Chip Select
#define TFT_DC 4   // GP4  - Data/Command
#define TFT_RST 6  // GP6  - Reset
#define TFT_BL 7   // GP7  - Backlight

// Touch screen pins
#define TOUCH_CS 22 // GP22 - Touch Chip Select

// ===========================================================================
// SPI Frequencies (Hz)
// ===========================================================================
#define SPI_FREQUENCY 40000000      // Default write frequency: 40 MHz
#define SPI_READ_FREQUENCY 20000000 // Read frequency: 20 MHz
#define SPI_TOUCH_FREQUENCY 2500000 // Touch frequency: 2.5 MHz

// ===========================================================================
// Display configuration
// ===========================================================================
#define TFT_WIDTH 320
#define TFT_HEIGHT 240
#define TFT_ROTATION 1 // Landscape (0=portrait, 1=landscape, 2=reverse portrait, 3=reverse landscape)

// ===========================================================================
// Font selection
// ===========================================================================
#define LOAD_GLCD  // 1. Standard 5x7 pixel font - needs ~1820 bytes in FLASH
#define LOAD_FONT2 // 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4 // 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6 // 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7 // 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8 // 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
#define LOAD_GFXFF // FreeFonts. Use #include <FS.h> and #include <SPIFFS.h> in sketch

// ===========================================================================
// Optional features
// ===========================================================================
#define SMOOTH_FONT