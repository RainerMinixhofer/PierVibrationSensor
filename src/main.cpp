/**
 * @file main.cpp
 * @brief Main application for Pier Vibration Sensor: Reads ADS1256 and MPU-6500, serves web UI, and streams UDP sensor data.
 *
 * @note SPI Bus Configuration:
 * - Display (ILI9341): spi0 @ 10 MHz (max per datasheet p. 242)
 * - Touchscreen (XPT2046): spi0 @ 2.5 MHz (max per datasheet p. 27: 400ns min clock cycle)
 * - SPI clock speed is dynamically switched before each touchscreen access
 */

// #define USE_WIFI // comment out to disable WiFi support
#define USE_WIZNET // comment out to disable WIZNET5K support
// #define WAIT_FOR_TOUCH // comment out to disable waiting for touch input
// #define FORCE_TOUCH_CALIBRATION // comment out to force touch calibration
// #define PRINT_TOUCH_CALIB_FILE // comment out to disable printing touch calibration file to serial
#define USE_HARDWARE_SPI // comment out to use software SPI for TFT display
// #define DEBUG_TOUCH_EVENTS // comment out to disable touch event debug logging
// #define LOG_THRESHOLD_EVENTS // comment out to disable speed threshold event logging

#include <Arduino.h>
#include "pico/stdlib.h" // <-- Add this include for Pico SDK functions
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#ifdef USE_WIZNET
#include <Ethernet.h>
#endif
#ifdef USE_WIFI
#include <WiFi.h>
#endif
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/pwm.h> // Add this include for PWM functions
#include <ADS1256.h>      // Make sure to include the ADS1256 library
// #include <FastIMU.h>
#include <Wire.h>
#include <FS.h>       // Already present
#include <LittleFS.h> // Use LittleFS for file system operations
#include "arm_math.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#ifdef USE_HARDWARE_SPI
#include <XPT2046_Touchscreen.h>
#else
#include <XPT2046_Bitbang.h>
#endif
#include "SdFatConfig.h" // Include SdFatConfig.h for SD card support
#include <SdFat.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>

#define PERFORM_CALIBRATION // Comment to disable startup calibration

// --- Constants and Configuration ---

// WIZNET5K Ethernet settings
#define WIZNET_SPI_PORT spi0
#define WIZNET_SPI_SCK 18
#define WIZNET_SPI_MOSI 19
#define WIZNET_SPI_MISO 16
#define WIZNET_SPI_CS 17
#define WIZNET_SPI_BPS 2000 * 1000
#define WIZNET_RST 20 ///< WIZNET5K HW reset input pin
#define WIZNET_INT 21 ///< WIZNET5K interrupt output pin (goes low when e.g. when wake on LAN is triggered)

// Constants for ADS1256
#define ADS1256_SPI_PORT SPI1
#define ADS1256_SPI_SCK 10
#define ADS1256_SPI_MOSI 11
#define ADS1256_SPI_MISO 12
#define ADS1256_SPI_CS 13
#define ADS1256_SPI_RST 0 // Set to 0 if not used
#define ADS1256_SPI_BPS 1400 * 1000
#define ADS1256_SYNC 14
#define ADS1256_DRDY 15
#define ADS1256_DRATE DRATE_2000SPS // 2000 SPS to get approximately 500 SPS overall
#define ADS1256_CLOCKMHZ 7.68
#define ADS1256_VREF 2.5

// Create ADS1256 object using SPI1
ADS1256 ads(ADS1256_DRDY, ADS1256_SPI_RST, ADS1256_SYNC, ADS1256_SPI_CS, 2.500, &ADS1256_SPI_PORT); // DRDY, RESET, SYNC(PDWN), CS, VREF(float), SPI bus.

// Constants for ICM20948
#define ICM20948_I2C_PORT Wire // Connected to I2C0
#define ICM20948_I2C_SDA 8
#define ICM20948_I2C_SCL 9
#define ICM20948_INT 5
#define ICM20948_I2C_BPS 400 * 1000
#define ICM20948_ADDR 0x69 // AD0 pin high
Adafruit_ICM20948 icm;

// Constants for TFT display with ILI9341 driver
// Hardware pins mapping (Display <-> Raspberry Pi Pico 2 W):
// SDO(MISO) <-> GP0
// SCK <-> GP2
// SDI(MOSI) <-> GP7
// DC <-> GP4
// RESET <-> GP6
// CS <-> GP1

#define TFT_SCLK 2      // GP2 (SCK)
#define TFT_MOSI 7      // GP7 (SDI)
#define TFT_MISO 0      // GP0 (SDO), set to -1 if not used
#define TFT_CS 1        // GP1
#define TFT_DC 4        // GP4
#define TFT_RST 6       // GP6, set to -1 if not used
#define TFT_LED 3       // GP3, Set to -1 if not used
#define TFT_ROTATION 1  // Set the default rotation for the display
#define TFT_TEXT_SIZE 1 // Set the default text size for the display
#ifdef USE_HARDWARE_SPI
// Hardware SPI configuration
#define TFT_SPI_PORT spi0      // Using SPI0
#define TFT_SPI_BPS 10000000   // 20 MHz SPI clock for all devices (TFT, Touch, SD)
#define TOUCH_SPI_BPS 10000000 // Same speed as TFT for consistent SPI bus performance

// Create TFT display object using hardware SPI (3-wire constructor)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
#else
// Software SPI configuration
// Create TFT display object using software SPI
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
#endif

// Custom color constants (RGB565 format)
#define COLOR_GREY 0x7BEF // Medium grey for UI elements

// Constants for XPT2046 TFT touch controller (if used)
#define TOUCH_CLK TFT_SCLK // GP2 (shared with TFT and SD SCK)
#define TOUCH_DIN TFT_MOSI // GP7 (shared with TFT and SD MOSI)
#define TOUCH_DO TFT_MISO  // GP0 (shared with TFT and SD MISO)
#define TOUCH_CS 22        // GP22, Set to -1 if not used
#define TOUCH_IRQ 27       // GP27, Set to -1 if not used

// Create TFT touchscreen object using XPT2046 driver
#ifdef USE_HARDWARE_SPI
#define TOUCH_SPI_PORT spi0 // Using SPI0
XPT2046_Touchscreen touchscreen(TOUCH_CS, TOUCH_IRQ);
#else
XPT2046_Bitbang touchscreen(TOUCH_DIN, TOUCH_DO, TOUCH_CLK, TOUCH_CS);
#endif

// XPT2046 touch controller resolution constants
// XPT2046 uses 12-bit ADC (0-4095 range)
const uint8_t XPT2046_RESOLUTION_BITS = 12;                                   // ADC resolution in bits
const uint16_t XPT2046_YMAX_VALUE = (1 << XPT2046_RESOLUTION_BITS) - 1;       // Maximum raw coordinate value based on resolution (4095)
const uint16_t XPT2046_XMAX_VALUE = (1 << (XPT2046_RESOLUTION_BITS - 1)) - 1; // Due to wiring, X max is half of full range (2047)

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
//
// Chip select pin
#define SD_CS_PIN = 10
//
// Pin numbers in templates must be constants.
#define SD_SCK TFT_SCLK  // GP2 (shared with TFT and Touch SCK)
#define SD_MOSI TFT_MOSI // GP7 (shared with TFT and Touch MOSI)
#define SD_MISO TFT_MISO // GP0 (shared with TFT and Touch MISO)
#define SD_CS 26         // GP26

#ifdef USE_HARDWARE_SPI
// Hardware SPI configuration for SD card
#define SD_SPI_PORT TFT_SPI_PORT // Using same SPI0 as TFT
#define SD_SPI_BPS 20000000      // 20 MHz for SD card (same as TFT and Touch)
#define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(20))
#else
// Software SPI configuration for SD card
#if SPI_DRIVER_SELECT == 2 // Must be set in SdFat/SdFatConfig.h
SoftSpiDriver<SD_MISO, SD_MOSI, SD_SCK> softSpi;
// Speed argument is ignored for software SPI.
#define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(0), &softSpi)
#else
#error SPI_DRIVER_SELECT must be two in SdFat/SdFatConfig.h when USE_HARDWARE_SPI is not defined
#endif // SPI_DRIVER_SELECT
#endif // USE_HARDWARE_SPI

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif // SD_FAT_TYPE

// MMSE-based calibration coefficients
// Transformation: Xd = A*Xt + B*Yt + C, Yd = D*Xt + E*Yt + F
struct CalibrationMatrix
{
  float A, B, C, D, E, F;
};

// NTP server address (local)
#define LOCAL_TIMEZONE "Europe/Vienna" // Set your local timezone (IANA TZ format)
#define NTP_SERVER "10.0.0.1"          // Set your local NTP server IP here
#define NTP_PORT 123
#define NTP_TIMEOUT_MS 3000

#define HTTP_SERVER_PORT 3000
#define UDP_STREAM_PORT 10000
#define UDP_PACKET_SIZE 32 // Set your desired block size

// --- Fallback IP when DHCP is not working ---

IPAddress ip(10, 0, 3, 177);

#ifdef USE_WIZNET
EthernetServer server(HTTP_SERVER_PORT);
EthernetUDP udpStream;
#endif

#ifdef USE_WIFI
WiFiServer server(HTTP_SERVER_PORT);
WiFiUDP udpStream;
#endif

/**
 * @enum NetworkMode
 * @brief Indicates which network interface is active.
 */
enum NetworkMode
{
  NET_NONE,
  NET_WIZNET,
  NET_WIFI
};
NetworkMode netMode = NET_NONE;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

// --- Network Configuration ---
char macString[18] = "--:--:--:--:--:--";
char ssid[64] = {0};
char wifi_password[64] = {0};

// --- Touch Calibration ---
CalibrationMatrix mmseCalibMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}; // Identity by default
int NUM_CALIB_POINTS = 5;                                                 // Change to 5 for faster calibration, 9 for better accuracy
volatile bool trigger_touch_calibration = false;                          // Trigger for remote calibration

// --- Touch Event Handling ---
static volatile bool touch_event_flag = false;
static volatile int irq_save = 0; // Save the IRQ state for touch events
#ifndef USE_HARDWARE_SPI
TouchPoint touch_event_point = {0, 0, 0, 0, 0, 0}; // Initialize touch point structure
#else
TS_Point touch_event_point = {0, 0, 0}; // Initialize touch point structure for XPT2046_Touchscreen
#endif

// --- Sensor Data and Flags ---
volatile bool ads1256_data_ready = false;  ///< Flag set by ADS1256 DRDY interrupt
volatile bool icm20948_data_ready = false; ///< Flag set by ICM-20948 INT interrupt
int adcValue0 = 0.0;                       ///< Latest raw value from ADS1256 channel 0 (Veloc X)
int adcValue7 = 0.0;                       ///< Latest raw value from ADS1256 channel 7 (Veloc Y)
float velocX = 0.0;                        ///< Latest velocity X value in m/s
float velocY = 0.0;                        ///< Latest velocity Y value in m/s
sensors_event_t accelData;                 // accel structure to hold latest accelerometer data
sensors_event_t gyroData;                  // gyro structure to hold latest gyroscope data
sensors_event_t magData;                   // mag structure to hold latest magnetometer data
sensors_event_t tempData;                  // temp structure to hold latest temperature data

// --- UDP Streaming Control ---
volatile bool udpStreamEnable[5] = {false, false, false, false, false};

// --- Display Mode and State ---
volatile int mode = 3;         // -1 = off 0 = text 1 = waveform, 2 = fft, 3 = sgram
volatile int display_axis = 0; // 0 = X axis, 1 = Y axis
bool text_mode_initialized = false;
bool waveform_initialized = false;
bool mode23_screen_cleared = false;

// --- Menu State ---
const int NUM_MENU_BUTTONS = 5;
bool menu_visible = false;

// --- Sample Tracking ---
int total_samples = 0;
int last_report_sample = 0;
const int REPORT_INTERVAL_SAMPLE_S = 1000; // Report every 1000 samples
const int display_n_samples = 256;

// --- Waveform Display ---
const int waveform_x = 0;
const int waveform_w = 320;
const int waveform_y = 0;
const int waveform_h = 240;
const int waveform_mid_y = waveform_y + waveform_h / 2;
const int LAST_POINTS_SIZE = 1024;
short last_points[LAST_POINTS_SIZE];

// --- Spectrogram Display ---
int disp_column = 0;
short spectrogram_min = 32767;  // Current spectrogram scale minimum
short spectrogram_max = -32768; // Current spectrogram scale maximum

// --- Ring Buffer for Data Storage ---
const int RING_BUFFER_SIZE = 32768; // Must be 2^n
int ring_buffer[RING_BUFFER_SIZE];
volatile int ring_buffer_tail = 0;

// --- Real-time Crossing Detection ---
volatile bool crossing_detected = false;
volatile unsigned long crossing_time_out = 0; // When signal went outside
volatile unsigned long crossing_time_in = 0;  // When signal came back inside
volatile bool currently_outside = false;      // Track if currently outside threshold
static short previous_sample = 0;
const int CROSSING_THRESHOLD = 10000;

// --- Event Sample Buffer ---
const int MAX_EVENT_SAMPLES = 4096;
short event_samples[MAX_EVENT_SAMPLES];
volatile int event_sample_count = 0;

// --- FFT Configuration ---
const int FFT_SIZE = 1024;

// --- Magnitude Spectrum ---
const int mag_spec_len = 512;

// ============================================================================
// FUNCTION DECLARATIONS AND INTERRUPT HANDLERS
// ============================================================================

// Interrupt handler for ADS1256 DRDY pin
void ads1256_drdy_isr(uint gpio, uint32_t events)
{
  ads1256_data_ready = true;
}

// Interrupt handler for ICM-20948 INT pin
void icm20948_int_isr(uint gpio, uint32_t events)
{
  icm20948_data_ready = true;
}

// Forward declarations
void drawMenuIndicator(int scroll_offset = 0);
void drawModeMenu();

// Helper functions to get raw touch coordinates (compatible with both libraries)
inline uint16_t getTouchRawX()
{
#ifdef USE_HARDWARE_SPI
  return touch_event_point.x; // XPT2046_Touchscreen: x is already the raw ADC value
#else
  return touch_event_point.xRaw; // XPT2046_Bitbang: use xRaw field
#endif
}

inline uint16_t getTouchRawY()
{
#ifdef USE_HARDWARE_SPI
  return touch_event_point.y; // XPT2046_Touchscreen: y is already the raw ADC value
#else
  return touch_event_point.yRaw; // XPT2046_Bitbang: use yRaw field
#endif
}

#ifdef USE_HARDWARE_SPI
// Helper function to set SPI clock speed for display operations
inline void setDisplaySPISpeed()
{
  spi_set_baudrate(TFT_SPI_PORT, TFT_SPI_BPS);
}

// Helper function to set SPI clock speed for touchscreen operations
inline void setTouchSPISpeed()
{
  spi_set_baudrate(TFT_SPI_PORT, TOUCH_SPI_BPS);
}
#endif

// Interrupt handler for TFT touch controller (XPT2046)
void touch_irq_isr(uint gpio, uint32_t events)
{
  // Set a volatile flag for main loop processing
  gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false); // disable IRQ to prevent triggering during processing
#ifdef USE_HARDWARE_SPI
  // For hardware SPI, just set the flag - don't read in IRQ context
  // Reading will be done in main loop after flag is detected
  touch_event_flag = true;
#else
  // For software SPI, reading in IRQ context works fine
  touch_event_point = touchscreen.getTouch();
  touch_event_flag = touch_event_point.z > 0; // Set flag if touch is detected
  if (!touch_event_flag)
  {
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true); // enable IRQ if no touch detected
  }
#endif
}

void initSDCard()
{
  Serial.println("Initializing SD card...");

  // Initialize LittleFS
  if (!LittleFS.begin())
  {
    Serial.println("Failed to mount LittleFS!");
    return;
  }
  Serial.println("LittleFS mounted successfully.");

#ifdef USE_HARDWARE_SPI
  // Initialize hardware SPI for SD card (shared with TFT on SPI0)
  Serial.println("Using hardware SPI for SD card");
  spi_init(SD_SPI_PORT, SD_SPI_BPS);
  gpio_set_function(SD_SCK, GPIO_FUNC_SPI);
  gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(SD_MISO, GPIO_FUNC_SPI);

  // Configure CS pin as output
  gpio_init(SD_CS);
  gpio_set_dir(SD_CS, GPIO_OUT);
  gpio_put(SD_CS, 1); // Start with CS high (inactive)
#else
  Serial.println("Using software SPI for SD card");
#endif

  // Initialize SD card
  if (!sd.begin(SD_CONFIG))
  {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized successfully.");

  // Check if the root directory exists
  if (!sd.exists("/"))
  {
    Serial.println("Root directory does not exist, creating...");
    if (!sd.mkdir("/"))
    {
      Serial.println("Failed to create root directory!");
      return;
    }
  }
  Serial.println("Root directory exists.");
  // List files in the root directory
  FsFile root = sd.open("/");
  if (!root)
  {
    Serial.println("Failed to open root directory!");
    return;
  }
  Serial.println("Listing files in root directory:");
  FsFile file = root.openNextFile();
  while (file)
  {
    char fname[64] = {0};
    file.getName(fname, sizeof(fname));
    Serial.printf("File: %s, Size: %d bytes\n", fname, file.fileSize());
    file = root.openNextFile();
  }
  root.close();
  Serial.println("SD card initialized and files listed successfully.");
}

void setBacklight(float brightness)
{
  if (TFT_LED != -1)
  {
    if (brightness == 0.0f || brightness == 1.0f)
    {
      gpio_init(TFT_LED);
      gpio_set_dir(TFT_LED, GPIO_OUT);
      gpio_put(TFT_LED, (int)brightness); // Directly set pin state for full off/on
    }
    else
    {
      // brightness: 0.0 (off) to 1.0 (full on)
      static bool initialized = false;
      if (!initialized)
      {
        gpio_set_function(TFT_LED, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(TFT_LED);
        pwm_set_wrap(slice_num, 255); // 8-bit resolution
        pwm_set_enabled(slice_num, true);
        initialized = true;
      }
      uint slice_num = pwm_gpio_to_slice_num(TFT_LED);
      uint level = (uint)(brightness * 255.0f);
      if (level > 255)
        level = 255;
      pwm_set_gpio_level(TFT_LED, level);
    }
  }
}

void tft_read_display_id_and_status()
{
  // ILI9341 responds to 0x04 (Read Display Identification Information)
  // Should return 3 bytes: Manufacturer ID, Module/Driver Version ID, Module/Driver ID
  // Yields 0x00 0x00 0x00 since the IDs seem not to be provided by the display

  // Read 3 bytes (index 1 denotes the second byte, since the first byte read back is a dummy byte)
  uint8_t id1 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDID, 0); // ID1 Status Register
  uint8_t id2 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDID, 1); // ID2 Status Register
  uint8_t id3 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDID, 2); // ID3 Status Register

  // print the IDs for debug
  Serial.printf("ILI9341 ID bytes: 0x%02X 0x%02X 0x%02X\n", id1, id2, id3);

  tft.begin();

  uint8_t stat1 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 0); // #2 D[31:25] status bits
  uint8_t stat2 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 1); // #3 D[22:16] status bits
  uint8_t stat3 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 2); // #4 D[10:8] status bits
  uint8_t stat4 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 3); // #5 D[7:5] status bits
  // print the IDs for debug
  Serial.printf("ILI9341 Status bytes: 0x%02X 0x%02X 0x%02X 0x%02X\n", stat1, stat2, stat3, stat4);
}

void tft_read_test()
{
  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDMODE);
  Serial.printf("Display Power Mode: 0x%02X\n", x);
  x = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDMADCTL);
  Serial.printf("MADCTL Mode: 0x%02X\n", x);
  x = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDPIXFMT);
  Serial.printf("Pixel Format: 0x%02X\n", x);
  x = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDIMGFMT);
  Serial.printf("Image Format: 0x%02X\n", x);
  x = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDSELFDIAG);
  Serial.printf("Self Diagnostic: 0x%02X\n", x);
  x = tft.readcommand8(ILI9341_GMCTRN1, 13);
  Serial.printf("Gamma Control Byte: 0x%02X\n", x);
}

void tftPrint(const char *msg, bool resetline = false)
{
  // Print debug messages to Serial
  Serial.print(msg);

  static int tft_line = 0;
  static const int font_height = 8 * TFT_TEXT_SIZE;
  static int max_lines = 0;

  if (resetline)
  {
    tft_line = 0; // Reset line counter if requested
  }

  // Update max_lines if rotation changed
  int rotation = TFT_ROTATION;
  if (tft.getRotation() != rotation)
  {
    tft.setRotation(rotation);
  }
  int disp_height = tft.height(); // tft.height() accounts for rotation
  max_lines = disp_height / font_height;

  // Split msg into lines
  const char *p = msg;
  while (*p)
  {
    // Find end of line or end of string
    const char *line_end = strchr(p, '\n');
    int len = line_end ? (line_end - p) : strlen(p);

    // If at bottom, wrap to top
    if (tft_line >= max_lines)
    {
      tft.fillRect(0, 0, tft.width(), disp_height, ILI9341_BLACK);
      tft_line = 0;
      drawMenuIndicator(); // Redraw menu indicator after clearing screen
    }

    // Print the line
    tft.setCursor(0, tft_line * font_height);
    for (int i = 0; i < len; ++i)
      tft.write(p[i]);
    if (line_end)
    {
      tft_line++;
      p = line_end + 1;
    }
    else
    {
      p += len;
    }
  }
}

// Helper function to print to Serial and optionally to TFT in mode 0
void debugPrint(const char *msg, uint16_t color = ILI9341_GREEN)
{
  Serial.print(msg);
  if (mode == 0 && text_mode_initialized)
  {
    // Also display on TFT in text mode
    static int tft_line = 3; // Start at line 3 to avoid menu indicator (16 pixels tall = 2 lines)
    static const int font_height = 8 * TFT_TEXT_SIZE;
    int disp_height = tft.height();
    int max_lines = disp_height / font_height;

    // Split msg into lines
    const char *p = msg;
    while (*p)
    {
      const char *line_end = strchr(p, '\n');
      int len = line_end ? (line_end - p) : strlen(p);

      // If at bottom, wrap to top (but skip lines 0-2 for menu indicator)
      if (tft_line >= max_lines)
      {
        // Clear screen except for menu indicator area (first 3 lines = 24 pixels)
        tft.fillRect(0, 3 * font_height, tft.width(), disp_height - 3 * font_height, ILI9341_BLACK);
        tft_line = 3;
      }

      // Print the line
      tft.setCursor(0, tft_line * font_height);
      tft.setTextColor(color);
      tft.setTextSize(TFT_TEXT_SIZE);
      for (int i = 0; i < len; ++i)
        tft.write(p[i]);

      if (line_end)
      {
        tft_line++;
        p = line_end + 1;
      }
      else
      {
        p += len;
      }
    }
  }
}

void debugPrintf(const char *format, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  debugPrint(buffer);
}

void debugPrintfColor(uint16_t color, const char *format, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  debugPrint(buffer, color);
}

void debugPrintln(const char *msg)
{
  char buffer[256];
  snprintf(buffer, sizeof(buffer), "%s\n", msg);
  debugPrint(buffer);
}

void disableTouchController()
{
  Serial.println("Disabling touch controller...");
  // Set CS of Touch high to disable it
  if (TOUCH_CS != -1)
  {
    gpio_init(TOUCH_CS);
    gpio_set_dir(TOUCH_CS, GPIO_OUT);
    gpio_put(TOUCH_CS, 1); // Disable touch controller
  }
}

void disableSDCard()
{
  Serial.println("Disabling SD card...");
  // Set CS of SD card high to disable it
  if (SD_CS != -1)
  {
    gpio_init(SD_CS);
    gpio_set_dir(SD_CS, GPIO_OUT);
    gpio_put(SD_CS, 1); // Disable SD card
  }
}

void resetTFTDisplay()
{
  Serial.println("Resetting TFT display...");
  disableTouchController();
  disableSDCard();
  tft.begin();
  setBacklight(0.25);            // Turn on backlight after initialization
  tft.setRotation(TFT_ROTATION); // Set the display rotation: 0 means portrait with J4 on top, 1 means landscape with J4 on left clockwise rotated from 0 (default), 2 means portrait with J4 on bottom, 3 means landscape with J4 on right
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(TFT_TEXT_SIZE);
  tft.setScrollMargins(0, 0); // Set scroll margins to 0
  tft.setCursor(0, 0);
}

void initTFTDisplay()
{
  Serial.println("TFT display driver initialization");
  // Note: Don't disable touch controller here - let XPT2046_Touchscreen library manage CS pin
  disableSDCard();

#ifdef USE_HARDWARE_SPI
  // Initialize hardware SPI for TFT display
  spi_init(TFT_SPI_PORT, TFT_SPI_BPS);

  // Configure SPI pins
  gpio_set_function(TFT_MISO, GPIO_FUNC_SPI); // GP0 - MISO
  gpio_set_function(TFT_SCLK, GPIO_FUNC_SPI); // GP2 - SCK
  gpio_set_function(TFT_MOSI, GPIO_FUNC_SPI); // GP7 - MOSI
  // Note: CS is handled by the library as a GPIO pin

  Serial.println("Hardware SPI initialized for TFT display");
  Serial.printf("SPI Port: spi0, Speed: %d Hz\n", TFT_SPI_BPS);
  Serial.printf("MISO: GP%d, MOSI: GP%d, SCK: GP%d\n", TFT_MISO, TFT_MOSI, TFT_SCLK);
  Serial.printf("CS: GP%d, DC: GP%d, RST: GP%d\n", TFT_CS, TFT_DC, TFT_RST);
#else
  Serial.println("Software SPI initialized for TFT display");
  Serial.printf("MISO: GP%d, MOSI: GP%d, SCK: GP%d\n", TFT_MISO, TFT_MOSI, TFT_SCLK);
  Serial.printf("CS: GP%d, DC: GP%d, RST: GP%d\n", TFT_CS, TFT_DC, TFT_RST);
#endif

  setBacklight(0.0); // Turn off backlight initially
  tft.begin();
  setBacklight(0.25); // Turn on backlight after initialization
  tft_read_display_id_and_status();
  tft_read_test();               // Read and print display diagnostics
  delay(1000);                   // Allow time for display to stabilize
  tft.setRotation(TFT_ROTATION); // Set the display rotation
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(TFT_TEXT_SIZE);
  tft.setScrollMargins(0, 0); // Set scroll margins to 0
  tft.setCursor(0, 0);
  tftPrint("TFT display driver initialized...\n");
  tftPrint("---------------------------------------------\n");
}

void setupTFTDisplayForSpectrogram()
{
  Serial.println("ILI9341 initialization for spectrogram...");

  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
}

void initTouchController()
{
  // Optionally calibrate or test touch here
  tftPrint("Initializing XPT2046 touch controller...\n");
#ifdef USE_HARDWARE_SPI
  // Note: XPT2046 shares SPI bus (spi0) with display but requires slower clock (max 2.5 MHz)
  // SPI speed is dynamically switched before each touchscreen access
  touchscreen.begin();                   // Uses spi0 which was initialized in initTFTDisplay()
  touchscreen.setRotation(TFT_ROTATION); // Match TFT rotation if supported
  // Initialize the IRQ pin for hardware SPI library
  gpio_init(TOUCH_IRQ);
  gpio_set_dir(TOUCH_IRQ, GPIO_IN);
  gpio_pull_up(TOUCH_IRQ); // Enable pull-up resistor
  // Note: CS pin is managed by the XPT2046_Touchscreen library
#else
  touchscreen.begin();
  // Initialize the IRQ pin
  gpio_init(TOUCH_IRQ);
  gpio_set_dir(TOUCH_IRQ, GPIO_IN);
  gpio_pull_up(TOUCH_IRQ); // Enable pull-up resistor
  // Note: CS pin is managed by the XPT2046_Bitbang library
#endif

#ifdef WAIT_FOR_TOUCH
  gpio_set_irq_enabled_with_callback(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true, &touch_irq_isr);
  tftPrint("Touch to continue...\n");
  // Wait for touch event
  while (!touch_event_flag)
  {
    // Wait for touch event
    tight_loop_contents();
  }
  Serial.printf("Touch event detected!\n");
  Serial.printf("Touch position: x=%d, y=%d, z=%d\n", touch_event_point.x, touch_event_point.y, touch_event_point.z);
#else
  gpio_set_irq_enabled_with_callback(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false, &touch_irq_isr); // Disable IRQ to prevent further triggering
#endif
  touch_event_flag = false; // Reset flag after first touch event
  tftPrint("Touch controller initialized...\n");
  tftPrint("---------------------------------------------\n");
}

void drawCalibrationCross(int x, int y, uint16_t color = ILI9341_RED)
{
  // Draw a cross at the specified position
  tft.drawLine(x - 10, y, x + 10, y, color);
  tft.drawLine(x, y - 10, x, y + 10, color);
}

// MMSE-Based Multipoint Calibration Algorithm (based on AN-1021)
// Implements Minimum Mean Square Error calibration for resistive touchscreens
//
// This algorithm uses least squares regression to calculate a 6-coefficient
// transformation matrix that maps raw touch coordinates to display coordinates.
//
// Transformation equations:
//   Display_X = A*Touch_X + B*Touch_Y + C
//   Display_Y = D*Touch_X + E*Touch_Y + F
//
// The coefficients compensate for:
//   - Non-linearity in the resistive touch layer
//   - Rotation/misalignment between touch and display
//   - Scaling differences between touch ADC range and display pixels
//   - Offset/translation
//
// Using 5+ calibration points provides better accuracy than 3-point methods
// by minimizing the sum of squared errors across all calibration points.
//
bool calculateMMSECalibration(int numPoints,
                              const int *lcd_x, const int *lcd_y,
                              const uint16_t *touch_x, const uint16_t *touch_y,
                              CalibrationMatrix &matrix)
{
  if (numPoints < 3)
  {
    Serial.println("Error: Need at least 3 calibration points");
    return false;
  }

  // Build the system of equations using least squares
  // For X: Xd = A*Xt + B*Yt + C
  // For Y: Yd = D*Xt + E*Yt + F

  // with Xd, Yd = display coordinates
  //      Xt, Yt = touch coordinates

  // Matrix equation: [Xt Yt 1] * [A B C; D E F]^T = [Xd Yd]
  // We solve: A^T * A * x = A^T * b (normal equations)

  // Calculate sums for least squares regression
  float sum_xt = 0, sum_yt = 0, sum_xt2 = 0, sum_yt2 = 0, sum_xt_yt = 0;
  float sum_xd = 0, sum_yd = 0, sum_xd_xt = 0, sum_xd_yt = 0;
  float sum_yd_xt = 0, sum_yd_yt = 0;

  for (int i = 0; i < numPoints; i++)
  {
    // Use touch coordinates directly - matrix will learn the scaling
    float xt = (float)touch_x[i];
    float yt = (float)touch_y[i];
    float xd = (float)lcd_x[i];
    float yd = (float)lcd_y[i];

    sum_xt += xt;
    sum_yt += yt;
    sum_xt2 += xt * xt;
    sum_yt2 += yt * yt;
    sum_xt_yt += xt * yt;
    sum_xd += xd;
    sum_yd += yd;
    sum_xd_xt += xd * xt;
    sum_xd_yt += xd * yt;
    sum_yd_xt += yd * xt;
    sum_yd_yt += yd * yt;
  }

  float n = (float)numPoints;

  // get coefficients of the linear equation systems in x and y direction

  float a[3] = {sum_xt2, sum_xt_yt, sum_xt}, b[3] = {sum_xt_yt, sum_yt2, sum_yt}, c[3] = {sum_xd_xt, sum_xd_yt, sum_xd}, d[3] = {sum_yd_xt, sum_yd_yt, sum_yd};
  a[0] /= sum_xt;
  b[0] /= sum_xt;
  c[0] /= sum_xt;
  d[0] /= sum_xt;
  a[1] /= sum_yt;
  b[1] /= sum_yt;
  c[1] /= sum_yt;
  d[1] /= sum_yt;
  a[2] /= n;
  b[2] /= n;
  c[2] /= n;
  d[2] /= n;

  // Calculate determinant of the coefficient matrix
  float det = (a[0] - a[2]) * (b[1] - b[2]) - (a[1] - a[2]) * (b[0] - b[2]);

  if (fabs(det) < 0.001f)
  {
    Serial.println("Error: Singular matrix - calibration points may be collinear");
    return false;
  }

  // Calculate A using Cramer's rule
  float det_a = (c[0] - c[2]) * (b[1] - b[2]) - (c[1] - c[2]) * (b[0] - b[2]);
  matrix.A = det_a / det;

  // Calculate B
  float det_b = (c[1] - c[2]) * (a[0] - a[2]) - (c[0] - c[2]) * (a[1] - a[2]);
  matrix.B = det_b / det;

  // Calculate C
  float det_c = b[0] * (a[2] * c[1] - a[1] * c[2]) + b[1] * (a[0] * c[2] - a[2] * c[0]) + b[2] * (a[1] * c[0] - a[0] * c[1]);
  matrix.C = det_c / det;

  // Solve for Y direction coefficients (D, E, F) - same process
  // Calculate D using Cramer's rule
  float det_d = (d[0] - d[2]) * (b[1] - b[2]) - (d[1] - d[2]) * (b[0] - b[2]);
  matrix.D = det_d / det;

  // Calculate E
  float det_e = (d[1] - d[2]) * (a[0] - a[2]) - (d[0] - d[2]) * (a[1] - a[2]);
  matrix.E = det_e / det;

  // Calculate F
  float det_f = b[0] * (a[2] * d[1] - a[1] * d[2]) + b[1] * (a[0] * d[2] - a[2] * d[0]) + b[2] * (a[1] * d[0] - a[0] * d[1]);
  matrix.F = det_f / det;

  Serial.println("MMSE Calibration Matrix:");
  Serial.printf("A=%.6f, B=%.6f, C=%.6f\n", matrix.A, matrix.B, matrix.C);
  Serial.printf("D=%.6f, E=%.6f, F=%.6f\n", matrix.D, matrix.E, matrix.F);

  return true;
}

// Apply MMSE calibration to convert touch coordinates to display coordinates
void applyMMSECalibration(uint16_t touch_x, uint16_t touch_y, int &display_x, int &display_y)
{
  // Apply MMSE calibration matrix directly to raw touch coordinates
  // The matrix was calculated from raw coordinates, so it already includes all transformations
  // (rotation, scaling, offset, etc.) - no need to rotate before applying the matrix
  float xt = (float)touch_x;
  float yt = (float)touch_y;

  display_x = (int)(mmseCalibMatrix.A * xt + mmseCalibMatrix.B * yt + mmseCalibMatrix.C);
  display_y = (int)(mmseCalibMatrix.D * xt + mmseCalibMatrix.E * yt + mmseCalibMatrix.F);
}

/**
 * @brief Performs MMSE-based multipoint touch screen calibration for the display controller.
 *
 * This function implements the MMSE (Minimum Mean Square Error) calibration algorithm as described
 * in AN-1021. It guides the user through touching multiple calibration points on the screen,
 * calculates the calibration matrix, saves it to LittleFS, and verifies accuracy with test points.
 *
 * Calibration Process:
 * 1. Displays NUM_CALIB_POINTS crosshairs (5 or 9 points)
 *    - 5-point mode: 4 corners + center
 *    - 9-point mode: 4 corners + 4 edge midpoints + center
 * 2. Waits for user to touch each crosshair using IRQ-based touch detection
 * 3. Compensates raw touch coordinates for current TFT_ROTATION setting
 * 4. Calculates MMSE calibration matrix using least-squares fitting
 * 5. Saves matrix coefficients and raw calibration points to /touch_calib.txt
 * 6. Verifies calibration with 3 additional test points
 * 7. Reports average and maximum calibration error in pixels
 *
 * The calibration compensates for:
 * - Touch sensor orientation relative to display (via TFT_ROTATION)
 * - Non-linear touch response
 * - Coordinate scaling and offset
 *
 * @note Requires global variables:
 *       - mmseCalibMatrix: CalibrationMatrix structure to store results
 *       - touch_event_flag: IRQ flag indicating touch detection
 *       - touch_event_point: Structure containing raw touch coordinates (x, y, z, xRaw, yRaw)
 *       - TFT_ROTATION: Current display rotation (0-3)
 *       - NUM_CALIB_POINTS: Number of calibration points (5 or 9)
 *
 * @note Uses IRQ on TOUCH_IRQ pin (edge falling) to detect touch events
 *
 * @note Calibration file format (/touch_calib.txt):
 *       Line 1: "MMSE" header
 *       Line 2: A,B,C coefficients (x transformation)
 *       Line 3: D,E,F coefficients (y transformation)
 *       Lines 4+: lcd_x,lcd_y,touch_x,touch_y reference points
 *
 * @note Quality thresholds for verification:
 *       - < 5.0 pixels: Excellent accuracy
 *       - < 10.0 pixels: Good accuracy
 *       - >= 10.0 pixels: Consider recalibrating
 *
 * @see calculateMMSECalibration() for matrix calculation
 * @see applyMMSECalibration() for using the calibration
 * @see drawCalibrationCross() for crosshair rendering
 */
void calibrateTouchController()
// MMSE-based multipoint calibration (AN-1021 algorithm)
{
  // Set display rotation
  tft.setRotation(TFT_ROTATION);

  // Initialize touch controller with matching rotation
  touchscreen.begin();
#ifdef USE_HARDWARE_SPI
  touchscreen.setRotation(TFT_ROTATION);
#endif

  tft.scrollTo(0); // Reset hardware scrolling to ensure correct coordinate mapping
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.printf("MMSE Touch Calibration\n");
  tft.printf("%d-point mode\n", NUM_CALIB_POINTS);
  tft.setTextSize(1);
  tft.print("Touch each crosshair\n");
  delay(2000);

  // Calibration points configuration
  // 5-point: 4 corners + center
  // 9-point: 4 corners + 4 edge midpoints + center
  const int numPoints = NUM_CALIB_POINTS;
  const int margin = 30;

  int lcd_x[9]; // Max 9 points
  int lcd_y[9];

  // First 5 points (common to both modes): 4 corners + center
  lcd_x[0] = margin; // Top-left
  lcd_y[0] = margin;
  lcd_x[1] = tft.width() - margin; // Top-right
  lcd_y[1] = margin;
  lcd_x[2] = tft.width() - margin; // Bottom-right
  lcd_y[2] = tft.height() - margin;
  lcd_x[3] = margin; // Bottom-left
  lcd_y[3] = tft.height() - margin;
  lcd_x[4] = tft.width() / 2; // Center
  lcd_y[4] = tft.height() / 2;

  // Additional 4 points for 9-point calibration: edge midpoints
  if (numPoints == 9)
  {
    lcd_x[5] = tft.width() / 2; // Top-center
    lcd_y[5] = margin;
    lcd_x[6] = tft.width() - margin; // Right-center
    lcd_y[6] = tft.height() / 2;
    lcd_x[7] = tft.width() / 2; // Bottom-center
    lcd_y[7] = tft.height() - margin;
    lcd_x[8] = margin; // Left-center
    lcd_y[8] = tft.height() / 2;
  }

  uint16_t touch_x[9], touch_y[9]; // Max 9 points

  // int lcd_x[numPoints] = {3931, 164, 2047, 3931, 164};
  // int lcd_y[numPoints] = {3849, 3849, 2047, 246, 246};
  // uint16_t touch_x[numPoints] = {3927, 193, 2054, 3915, 189};
  // uint16_t touch_y[numPoints] = {3920, 3943, 2127, 331, 371};

  for (int i = 0; i < numPoints; ++i)
  {
    // Clear entire screen before each calibration point
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.printf("Touch cross %d/%d", i + 1, numPoints);
    drawCalibrationCross(lcd_x[i], lcd_y[i]);

    Serial.printf("Waiting for touch at point %d...\n", i + 1);
    bool touch_detected = false;

#ifdef USE_HARDWARE_SPI
    while (!touch_detected)
    {
      if (touchscreen.touched())
      {
        touch_event_point = touchscreen.getPoint();
        if (touch_event_point.z > 0)
        {
          touch_detected = true;
          Serial.printf("Touch detected: x=%d, y=%d, z=%d\n",
                        touch_event_point.x, touch_event_point.y, touch_event_point.z);
        }
      }
      delay(10);
    }

    // Wait for touch release
    delay(100);
    while (touchscreen.touched())
    {
      delay(10);
    }
#else
    // Software SPI - use IRQ
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true);
    while (!touch_event_flag)
    {
      tight_loop_contents();
    }
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false);
    touch_event_flag = false;
    touch_event_point = touchscreen.getTouch();
    Serial.printf("Touch detected: x=%d, y=%d, z=%d\n",
                  touch_event_point.xRaw, touch_event_point.yRaw, touch_event_point.z);
#endif

    drawCalibrationCross(lcd_x[i], lcd_y[i], ILI9341_GREEN); // Draw green cross to indicate touch was registered

    // Store raw coordinates without rotation - let MMSE algorithm handle the transformation
    touch_x[i] = getTouchRawX();
    touch_y[i] = getTouchRawY();

    Serial.printf("Calibration point %d: Expected LCD(%d,%d) <-> Raw Touch(%d,%d)\n",
                  i, lcd_x[i], lcd_y[i], touch_x[i], touch_y[i]);
    tft.fillScreen(ILI9341_BLACK);
  }

  // Calculate MMSE calibration matrix
  if (!calculateMMSECalibration(numPoints, lcd_x, lcd_y, touch_x, touch_y, mmseCalibMatrix))
  {
    tftPrint("Calibration failed!", true);
    delay(2000);
    return;
  }

  // Save calibration matrix to LittleFS
  File f = LittleFS.open("/touch_calib.txt", "w");
  if (f)
  {
    // Save matrix coefficients
    f.printf("MMSE\n"); // Header to identify MMSE calibration
    f.printf("%.8f,%.8f,%.8f\n", mmseCalibMatrix.A, mmseCalibMatrix.B, mmseCalibMatrix.C);
    f.printf("%.8f,%.8f,%.8f\n", mmseCalibMatrix.D, mmseCalibMatrix.E, mmseCalibMatrix.F);
    // Also save raw calibration points for reference
    for (int i = 0; i < numPoints; ++i)
    {
      f.printf("%d,%d,%d,%d\n", lcd_x[i], lcd_y[i], touch_x[i], touch_y[i]);
    }
    f.close();
    Serial.println("MMSE calibration matrix saved to /touch_calib.txt");
  }
  else
  {
    Serial.println("Failed to save touch calibration!");
  }

  // Verify calibration accuracy with 3 test points
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.print("Verifying calibration...\n");
  tft.print("Touch 3 test points\n");

  const int numTestPoints = 4;
  // Use different positions than calibration points
  int test_lcd_x[numTestPoints] = {
      tft.width() / 4,     // Left-center
      tft.width() * 3 / 4, // Right-center
      tft.width() / 2,     // Center-top
      tft.width() / 2      // Center-bottom
  };
  int test_lcd_y[numTestPoints] = {
      tft.height() / 2,    // Left-center
      tft.height() / 2,    // Right-center
      tft.height() / 4,    // Center-top
      tft.height() * 3 / 4 // Center-bottom
  };
  uint16_t test_touch_x[numTestPoints], test_touch_y[numTestPoints];

  for (int i = 0; i < numTestPoints; ++i)
  {
    // Clear the instruction area
    tft.fillRect(0, 0, tft.width(), 30, ILI9341_BLACK);
    tft.setCursor(10, 10);
    tft.printf("Touch test point %d/3", i + 1);
    drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], ILI9341_YELLOW);

#ifdef USE_HARDWARE_SPI
    bool touch_detected = false;
    while (!touch_detected)
    {
      if (touchscreen.touched())
      {
        touch_event_point = touchscreen.getPoint();
        if (touch_event_point.z > 0)
        {
          touch_detected = true;
        }
      }
      delay(10);
    }
    delay(100);
    while (touchscreen.touched())
    {
      delay(10);
    }
#else
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true);
    while (!touch_event_flag)
    {
      tight_loop_contents();
    }
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false);
    touch_event_flag = false;
    touch_event_point = touchscreen.getTouch();
#endif

    // Store raw coordinates without rotation - MMSE matrix handles the transformation
    test_touch_x[i] = getTouchRawX();
    test_touch_y[i] = getTouchRawY();

    // Calculate calibration accuracy for this point
    int calibrated_x, calibrated_y;
    float xt = (float)test_touch_x[i];
    float yt = (float)test_touch_y[i];
    calibrated_x = (int)(mmseCalibMatrix.A * xt + mmseCalibMatrix.B * yt + mmseCalibMatrix.C);
    calibrated_y = (int)(mmseCalibMatrix.D * xt + mmseCalibMatrix.E * yt + mmseCalibMatrix.F);

    float error_x = calibrated_x - test_lcd_x[i];
    float error_y = calibrated_y - test_lcd_y[i];
    float error = sqrtf(error_x * error_x + error_y * error_y);

    drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], ILI9341_GREEN);

    // Display error near the crosshair
    tft.setCursor(test_lcd_x[i] + 15, test_lcd_y[i] - 10);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.printf("Err: %.1fpx", error);

    delay(1500);
    tft.fillScreen(ILI9341_BLACK);
  }

  // Calculate calibration accuracy
  float total_error = 0.0f;
  float max_error = 0.0f;

  Serial.println("\nCalibration Verification Results:");
  Serial.println("Point | Expected (X,Y) | Raw Touch (X,Y) | Calibrated (X,Y) | Error (pixels)");
  Serial.println("------|----------------|-----------------|------------------|---------------");

  for (int i = 0; i < numTestPoints; ++i)
  {
    int calibrated_x, calibrated_y;
    // Apply MMSE matrix directly to raw coordinates (no rotation)
    // The matrix was built from raw coordinates, so apply it directly during verification
    float xt = (float)test_touch_x[i];
    float yt = (float)test_touch_y[i];
    calibrated_x = (int)(mmseCalibMatrix.A * xt + mmseCalibMatrix.B * yt + mmseCalibMatrix.C);
    calibrated_y = (int)(mmseCalibMatrix.D * xt + mmseCalibMatrix.E * yt + mmseCalibMatrix.F);

    float error_x = calibrated_x - test_lcd_x[i];
    float error_y = calibrated_y - test_lcd_y[i];
    float error = sqrtf(error_x * error_x + error_y * error_y);

    Serial.printf("  %d   | (%3d, %3d)     | (%4d, %4d)    | (%4d, %4d)      | %.2f\n",
                  i + 1, test_lcd_x[i], test_lcd_y[i],
                  test_touch_x[i], test_touch_y[i],
                  calibrated_x, calibrated_y, error);

    total_error += error;
    if (error > max_error)
      max_error = error;
  }

  float avg_error = total_error / numTestPoints;

  Serial.println("------|----------------|------------------|---------------");
  Serial.printf("Average error: %.2f pixels\n", avg_error);
  Serial.printf("Maximum error: %.2f pixels\n", max_error);

  // Display results on screen
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, 40);
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.print("Calibration Complete\n\n");

  tft.setTextSize(1);
  tft.printf("Avg Error: %.1f px\n", avg_error);
  tft.printf("Max Error: %.1f px\n\n", max_error);

  if (avg_error < 5.0f)
  {
    tft.setTextColor(ILI9341_GREEN);
    tft.print("Excellent accuracy!");
  }
  else if (avg_error < 10.0f)
  {
    tft.setTextColor(ILI9341_YELLOW);
    tft.print("Good accuracy");
  }
  else
  {
    tft.setTextColor(ILI9341_RED);
    tft.print("Consider recalibrating");
  }

  // Reset text color to white for subsequent output
  tft.setTextColor(ILI9341_WHITE);

  delay(3000);
}

bool loadTouchCalibration()
{
  File f = LittleFS.open("/touch_calib.txt", "r");
  if (!f)
  {
    Serial.println("No touch calibration file found.");
    return false;
  }

  // Read first line to check if this is MMSE calibration
  String header = f.readStringUntil('\n');
  header.trim();

  if (header == "MMSE")
  {
    // Load MMSE calibration matrix
    String line1 = f.readStringUntil('\n');
    String line2 = f.readStringUntil('\n');
    f.close();

    if (sscanf(line1.c_str(), "%f,%f,%f", &mmseCalibMatrix.A, &mmseCalibMatrix.B, &mmseCalibMatrix.C) != 3 ||
        sscanf(line2.c_str(), "%f,%f,%f", &mmseCalibMatrix.D, &mmseCalibMatrix.E, &mmseCalibMatrix.F) != 3)
    {
      Serial.println("Invalid MMSE calibration file format.");
      LittleFS.remove("/touch_calib.txt");
      return false;
    }

    Serial.println("MMSE calibration matrix loaded:");
    Serial.printf("A=%.6f, B=%.6f, C=%.6f\n", mmseCalibMatrix.A, mmseCalibMatrix.B, mmseCalibMatrix.C);
    Serial.printf("D=%.6f, E=%.6f, F=%.6f\n", mmseCalibMatrix.D, mmseCalibMatrix.E, mmseCalibMatrix.F);
    return true;
  }
  else
  {
    // Legacy 3-point calibration - try to convert
    Serial.println("Found legacy 3-point calibration, converting to MMSE...");
    f.close();
    f = LittleFS.open("/touch_calib.txt", "r");

    int lcd_x[3], lcd_y[3];
    uint16_t touch_x[3], touch_y[3];
    bool valid = true;

    for (int i = 0; i < 3; ++i)
    {
      String line = f.readStringUntil('\n');
      if (sscanf(line.c_str(), "%d,%d,%hu,%hu", &lcd_x[i], &lcd_y[i], &touch_x[i], &touch_y[i]) != 4)
      {
        valid = false;
        break;
      }
      if (lcd_x[i] < 0 || lcd_y[i] < 0 || touch_x[i] <= 0 || touch_y[i] <= 0)
      {
        valid = false;
        break;
      }
    }
    f.close();

    if (!valid)
    {
      Serial.println("Invalid legacy calibration file.");
      LittleFS.remove("/touch_calib.txt");
      return false;
    }

    // Convert 3-point to MMSE matrix
    if (calculateMMSECalibration(3, lcd_x, lcd_y, touch_x, touch_y, mmseCalibMatrix))
    {
      Serial.println("Successfully converted to MMSE calibration.");
      // Save as MMSE format
      File fw = LittleFS.open("/touch_calib.txt", "w");
      if (fw)
      {
        fw.printf("MMSE\n");
        fw.printf("%.8f,%.8f,%.8f\n", mmseCalibMatrix.A, mmseCalibMatrix.B, mmseCalibMatrix.C);
        fw.printf("%.8f,%.8f,%.8f\n", mmseCalibMatrix.D, mmseCalibMatrix.E, mmseCalibMatrix.F);
        fw.close();
      }
      return true;
    }
    else
    {
      Serial.println("Failed to convert legacy calibration.");
      LittleFS.remove("/touch_calib.txt");
      return false;
    }
  }
}

// Initialize ADS1256 ADC with SPI1 support
void initADS1256()
{
  tftPrint("Initializing ADC... \n");

  // Initialize SPI1 with ADS1256 pin definitions
  ADS1256_SPI_PORT.setSCK(ADS1256_SPI_SCK);
  ADS1256_SPI_PORT.setTX(ADS1256_SPI_MOSI);
  ADS1256_SPI_PORT.setRX(ADS1256_SPI_MISO);

  // Setting up CS, RESET, SYNC and SPI
  // Assigning default values to: STATUS, MUX, ADCON, DRATE
  // Performing a SYSCAL

  ads.InitializeADC();

  // Set a PGA value
  Serial.println("Setting PGA to 1... ");
  ads.setPGA(PGA_1); // 0b00000000 - DEC: 0
  //--------------------------------------------

  // Set initial input channel
  Serial.println("Setting MUX to single-ended channel 0... ");
  ads.setMUX(SING_0); // 0b00001111 - DEC: 15
  //--------------------------------------------

  // Set DRATE
  Serial.println("Setting DRATE to 500 SPS... ");
  ads.setDRATE(ADS1256_DRATE);
  //--------------------------------------------

  // Read back the above 3 values to check if the writing was succesful
  assert(ads.getPGA() == PGA_1);                        // Check if PGA is set correctly
  assert(ads.readRegister(MUX_REG) == SING_0);          // Check if MUX is set correctly
  assert(ads.readRegister(DRATE_REG) == ADS1256_DRATE); // Check if DRATE is set correctly

  tftPrint("ADC initialized...\n");
  tftPrint("---------------------------------------------\n");
}

/**
 * @brief Read a single-ended channel from ADS1256 using Arduino library.
 * @param channel Channel number (0-7)
 * @return float value in volts
 */
float readADS1256_channel(uint8_t channel)
{
  // Reads the voltage from the specified channel (single-ended)
  ads.setMUX(channel << 4 | 0b00001111); // Set the MUX to the specified single-ended channel
  return ads.readSingle();
}

uint8_t ICM20948_read_whoami()
{
  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(ICM20X_B0_REG_BANK_SEL);
  Wire.write(0x00); // Select Bank 0
  Wire.endTransmission();
  Wire.beginTransmission(ICM20948_ADDR);
  Wire.write(ICM20X_B0_WHOAMI);
  Wire.endTransmission(false);
  Wire.requestFrom(ICM20948_ADDR, 1);
  // Read the WHOAMI value
  if (Wire.available())
  {
    return Wire.read();
  }
  // If no data is available, return 0xFF (error value)
  return 0xFF;
}

void initICM20948()
{
  String resultStr;
  char resultstr[64];
  ICM20948_I2C_PORT.setSDA(ICM20948_I2C_SDA);
  ICM20948_I2C_PORT.setSCL(ICM20948_I2C_SCL);
  ICM20948_I2C_PORT.setClock(ICM20948_I2C_BPS); // 400khz clock
  ICM20948_I2C_PORT.begin();
  bool success = icm.begin_I2C(ICM20948_ADDR, &ICM20948_I2C_PORT);
  if (!success)
  {
    tftPrint("Failed to find ICM20948 chip, halting...\n");
    while (1)
    {
      delay(10);
    }
  }
  tftPrint("ICM20948 Found!\n");

  int result = ICM20948_read_whoami();
  sprintf(resultstr, "ICM20948 Chip ID: 0x%02X\n", result);
  if (result != 0xEA)
  {
    tftPrint("Error: ICM20948 Chip ID mismatch!\n");
    tftPrint(resultstr);
    while (1)
    {
      delay(10);
    }
  }
  tftPrint(resultstr);
  result = icm.readExternalRegister(0x8C, 0x01);
  sprintf(resultstr, "ICM20948 Magnetometer (AK09916) Chip ID: 0x%02X\n", result); // AK09916_CHIP_ID;
  tftPrint(resultstr);
  if (result != 0x09)
  {
    tftPrint("Error: ICM20948 Magnetometer (AK09916) Chip ID mismatch!\n");
    tftPrint(resultstr);
    while (1)
    {
      delay(10);
    }
  }

  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  resultStr = "Accelerometer range set to: ";
  switch (icm.getAccelRange())
  {
  case ICM20948_ACCEL_RANGE_2_G:
    resultStr += " +-2G";
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    resultStr += " +-4G";
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    resultStr += " +-8G";
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    resultStr += " +-16G";
    break;
  }
  resultStr += "\n";
  tftPrint(resultStr.c_str());

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  resultStr = "Gyro range set to: ";
  switch (icm.getGyroRange())
  {
  case ICM20948_GYRO_RANGE_250_DPS:
    resultStr += "250 degrees/s";
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    resultStr += "500 degrees/s";
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    resultStr += "1000 degrees/s";
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    resultStr += "2000 degrees/s";
    break;
  }
  resultStr += "\n";
  tftPrint(resultStr.c_str());

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  resultStr = "Accelerometer data rate divisor set to: " + String(accel_divisor) + "\n";
  tftPrint(resultStr.c_str());
  resultStr = "Accelerometer data rate (Hz) is approximately: " + String(accel_rate) + "\n";
  tftPrint(resultStr.c_str());

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  resultStr = "Gyro data rate divisor set to: " + String(gyro_divisor) + "\n";
  tftPrint(resultStr.c_str());
  resultStr = "Gyro data rate (Hz) is approximately: " + String(gyro_rate) + "\n";
  tftPrint(resultStr.c_str());

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  resultStr = "Magnetometer data rate set to: ";
  switch (icm.getMagDataRate())
  {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    resultStr += "Shutdown";
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    resultStr += "Single/One shot";
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    resultStr += "10 Hz";
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    resultStr += "20 Hz";
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    resultStr += "50 Hz";
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    resultStr += "100 Hz";
    break;
  }
  resultStr += "\n";
  tftPrint(resultStr.c_str());
  gpio_init(ICM20948_INT);
  gpio_set_dir(ICM20948_INT, GPIO_IN);
  gpio_pull_up(ICM20948_INT);
  gpio_set_irq_enabled_with_callback(ICM20948_INT, GPIO_IRQ_EDGE_FALL, true, &icm20948_int_isr);
  tftPrint("IMU initialized...\n");
  tftPrint("---------------------------------------------\n");
}

void readICM20948()
{
  String resultStr;
  //  /* Get a new normalized sensor event */
  icm.getEvent(&accelData, &gyroData, &tempData, &magData);

  // resultStr = "\t\tTemperature: " + String(tempData.temperature) + " deg C\n";
  // tftPrint(resultStr.c_str());

  // /* Display the results (acceleration is measured in m/s^2) */
  // resultStr = "\t\tAccel X: " + String(accelData.acceleration.x) + "\tY: " + String(accelData.acceleration.y) + "\tZ: " + String(accelData.acceleration.z) + " m/s^2\n";
  // tftPrint(resultStr.c_str());

  // resultStr = "\t\tMag X: " + String(magData.magnetic.x) + "\tY: " + String(magData.magnetic.y) + "\tZ: " + String(magData.magnetic.z) + " uT\n";
  // tftPrint(resultStr.c_str());

  // /* Display the results (acceleration is measured in m/s^2) */
  // resultStr = "\t\tGyro X: " + String(gyroData.gyro.x) + "\tY: " + String(gyroData.gyro.y) + "\tZ: " + String(gyroData.gyro.z) + " radians/s\n";
  // tftPrint(resultStr.c_str());

  // delay(100); // Removed to allow full speed sampling

  //  Serial.print(temp.temperature);
  //
  //  Serial.print(",");
  //
  //  Serial.print(accel.acceleration.x);
  //  Serial.print(","); Serial.print(accel.acceleration.y);
  //  Serial.print(","); Serial.print(accel.acceleration.z);
  //
  //  Serial.print(",");
  //  Serial.print(gyro.gyro.x);
  //  Serial.print(","); Serial.print(gyro.gyro.y);
  //  Serial.print(","); Serial.print(gyro.gyro.z);
  //
  //  Serial.print(",");
  //  Serial.print(mag.magnetic.x);
  //  Serial.print(","); Serial.print(mag.magnetic.y);
  //  Serial.print(","); Serial.print(mag.magnetic.z);

  //  Serial.println();
  //
  //  delayMicroseconds(measurement_delay_us);
}
/**
 * @brief Get MAC address for WIZNET (from chip if possible, else fallback).
 * @param mac Pointer to 6-byte array to fill.
 */
void getWiznetMAC(byte *mac)
{
#ifdef USE_WIZNET
  // Read unique 64-bit ID from flash (RP2040)
  uint8_t id[8] = {0};
  uint32_t ints = save_and_disable_interrupts();
  flash_get_unique_id(id);
  restore_interrupts(ints);

  // Locally administered MAC: 02:XX:XX:XX:XX:XX (first byte 0x02)
  mac[0] = 0x02;
  mac[1] = id[0];
  mac[2] = id[1];
  mac[3] = id[2];
  mac[4] = id[3];
  mac[5] = id[4];
#endif
}

/**
 * @brief Get MAC address for WiFi (from chip).
 * @param mac Pointer to 6-byte array to fill.
 */
void getWiFiMAC(byte *mac)
{
#ifdef USE_WIFI
  String macStr = WiFi.macAddress(); // Format: "AA:BB:CC:DD:EE:FF"
  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) == 6)
  {
    for (int i = 0; i < 6; ++i)
      mac[i] = (byte)values[i];
  }
#endif
}

/**
 * @brief Load WiFi credentials from LittleFS file "/wifi.txt".
 * The file should contain two lines: first line is SSID, second line is password.
 * Returns true if loaded successfully, false otherwise.
 */
bool loadWiFiCredentials()
{
  File f = LittleFS.open("/wifi.txt", "r");
  if (!f)
    return false;

  String ssidStr = f.readStringUntil('\n');
  String passStr = f.readStringUntil('\n');
  f.close();

  ssidStr.trim();
  passStr.trim();

  if (ssidStr.length() == 0 || passStr.length() == 0)
    return false;

  strncpy(ssid, ssidStr.c_str(), sizeof(ssid) - 1);
  strncpy(wifi_password, passStr.c_str(), sizeof(wifi_password) - 1);
  return true;
}

#ifdef USE_WIFI
void initWiFi()
{
  if (!loadWiFiCredentials())
  {
    Serial.println("Failed to load WiFi credentials from /wifi.txt");
    netMode = NET_NONE;
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_password);

  Serial.print("Connecting to WiFi");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30)
  {
    sleep_ms(500);
    Serial.print(".");
    retry++;
  }
  Serial.print("\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    byte mac[6];
    getWiFiMAC(mac);
    Serial.printf("WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    server.begin();
    netMode = NET_WIFI;
  }
  else
  {
    Serial.print("WiFi connection failed.\n");
    netMode = NET_NONE;
  }
  Serial.print("WiFi initialized...\n");
}
#endif

#ifdef USE_WIZNET

/**
 * @brief Reset the WIZNET5K chip using the hardware reset pin.
 */
void wiznet5k_reset()
{
#ifdef USE_WIZNET
  gpio_init(WIZNET_RST);
  gpio_set_dir(WIZNET_RST, GPIO_OUT);
  gpio_put(WIZNET_RST, 0); // Assert reset (active low)
  sleep_ms(10);            // Hold reset for 10ms
  gpio_put(WIZNET_RST, 1); // Release reset
  sleep_ms(100);           // Wait for chip to be ready
#endif
}

/**
 * @brief Interrupt handler for WIZNET5K INT pin.
 * You can add custom handling here if needed.
 */
void wiznet5k_int_isr(uint gpio, uint32_t events)
{
  // Example: Print or set a flag (implement as needed)
  Serial.printf("WIZNET5K interrupt detected on GPIO %d\n", gpio);
  // Optionally: set a volatile flag for main loop processing
}

void initWIZNET5K()
{
  byte mac[6];
  char resultStr[32];
  tftPrint("Initializing Ethernet Network...\n");

  getWiznetMAC(mac);

  // --- Reset WIZNET5K chip before SPI init ---
  wiznet5k_reset();

  spi_init(WIZNET_SPI_PORT, WIZNET_SPI_BPS);
  gpio_set_function(WIZNET_SPI_SCK, GPIO_FUNC_SPI);
  gpio_set_function(WIZNET_SPI_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(WIZNET_SPI_MISO, GPIO_FUNC_SPI);
  gpio_init(WIZNET_SPI_CS);
  gpio_set_dir(WIZNET_SPI_CS, GPIO_OUT);
  gpio_put(WIZNET_SPI_CS, 1);

  // --- Setup WIZNET5K INT pin and ISR ---
  gpio_init(WIZNET_INT);
  gpio_set_dir(WIZNET_INT, GPIO_IN);
  gpio_pull_up(WIZNET_INT);
  // gpio_set_irq_enabled_with_callback(WIZNET_INT, GPIO_IRQ_EDGE_FALL, true, &wiznet5k_int_isr);

  Ethernet.init(WIZNET_SPI_CS);

  if (Ethernet.begin(mac) == 0)
  {
    Ethernet.begin(mac, ip);
  }
  sprintf(resultStr, "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  tftPrint(resultStr);
  sprintf(resultStr, "Ethernet IP: %s\n", Ethernet.localIP().toString().c_str());
  tftPrint(resultStr);
  netMode = NET_WIZNET;
  tftPrint("Ethernet Network initialized...\n");
}
#endif

bool setTimeFromNTP()
{
  uint8_t ntp_packet[48] = {0};
  ntp_packet[0] = 0b11100011;

  IPAddress ntpServerIP;
  if (!ntpServerIP.fromString(NTP_SERVER))
  {
    tftPrint("Invalid NTP server IP.\n");
    return false;
  }

#if defined(USE_WIFI)
  WiFiUDP udp;
  udp.begin(2390);
  udp.beginPacket(ntpServerIP, NTP_PORT);
  udp.write(ntp_packet, sizeof(ntp_packet));
  udp.endPacket();

  uint32_t start = millis();
  int packetSize = 0;
  uint8_t recv_buf[48];
  while ((millis() - start) < NTP_TIMEOUT_MS)
  {
    packetSize = udp.parsePacket();
    if (packetSize >= 48)
    {
      udp.read(recv_buf, 48);
      break;
    }
    delay(10);
  }
#elif defined(USE_WIZNET)
  EthernetUDP ethUdp;
  char resultStr[64];
  ethUdp.begin(2390);
  ethUdp.beginPacket(ntpServerIP, NTP_PORT);
  ethUdp.write(ntp_packet, sizeof(ntp_packet));
  ethUdp.endPacket();

  uint32_t start = millis();
  int packetSize = 0;
  uint8_t recv_buf[48];
  while ((millis() - start) < NTP_TIMEOUT_MS)
  {
    packetSize = ethUdp.parsePacket();
    if (packetSize >= 48)
    {
      ethUdp.read(recv_buf, 48);
      break;
    }
    delay(10);
  }
#else
  Serial.print("NTP requires either WiFi or WIZNET enabled.\n");
  return false;
#endif

  if (packetSize < 48)
  {
    tftPrint("NTP response timeout or too short.\n");
    return false;
  }

  uint32_t secs_since_1900 =
      (recv_buf[40] << 24) | (recv_buf[41] << 16) | (recv_buf[42] << 8) | recv_buf[43];
  uint32_t unix_time = secs_since_1900 - 2208988800U;

  struct timeval tv;
  tv.tv_sec = unix_time;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);

  time_t t = unix_time;
  struct tm *tm_info = localtime(&t);

  sprintf(resultStr, "System time set from NTP: %04d-%02d-%02d %02d:%02d:%02d\n",
          tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
          tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  tftPrint(resultStr);

  return true;
}

void serveChartPage(Stream &client)
{
  // Open the HTML file from the LittleFS filesystem first
  File htmlFile = LittleFS.open("/chart.html", "r");

  if (!htmlFile)
  {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>Error: chart.html not found on filesystem.</h2></body></html>");
    return;
  }

  // Send HTTP headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();

  // Read and send file content byte by byte (simple and reliable)
  while (htmlFile.available())
  {
    char c = htmlFile.read();
    client.write(c);
  }

  htmlFile.close();
}

void serveSensorData(Stream &client)
{
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  // Determine network type string
  const char *nettype = "None";
#if defined(USE_WIFI)
  if (netMode == NET_WIFI)
    nettype = "WLAN";
#endif
#if defined(USE_WIZNET)
  if (netMode == NET_WIZNET)
    nettype = "Ethernet";
#endif

  time_t now = time(NULL);
  struct tm *tm_info = localtime(&now);
  long tz_offset_sec = 0;
  // calculate offset manually if tm_gmtoff is not available
  time_t utc = mktime(gmtime(&now));
  time_t local = mktime(tm_info);
  tz_offset_sec = difftime(local, utc);

  // Send JSON in parts to avoid stack issues with large snprintf
  client.print("{");
  client.print("\"velocXraw\":");
  client.print(adcValue0);
  client.print(",\"velocYraw\":");
  client.print(adcValue7);
  client.print(",\"velocX\":");
  client.print(velocX, 5);
  client.print(",\"velocY\":");
  client.print(velocY, 5);
  client.print(",\"accelXraw\":");
  client.print(accelData.data[0]);
  client.print(",\"accelYraw\":");
  client.print(accelData.data[1]);
  client.print(",\"accelZraw\":");
  client.print(accelData.data[2]);
  client.print(",\"accelX\":");
  client.print(accelData.acceleration.x * 9.80665, 5);
  client.print(",\"accelY\":");
  client.print(accelData.acceleration.y * 9.80665, 5);
  client.print(",\"accelZ\":");
  client.print(accelData.acceleration.z * 9.80665, 5);
  client.print(",\"gyroXraw\":");
  client.print(gyroData.data[0]);
  client.print(",\"gyroYraw\":");
  client.print(gyroData.data[1]);
  client.print(",\"gyroZraw\":");
  client.print(gyroData.data[2]);
  client.print(",\"gyroX\":");
  client.print(gyroData.gyro.x, 5);
  client.print(",\"gyroY\":");
  client.print(gyroData.gyro.y, 5);
  client.print(",\"gyroZ\":");
  client.print(gyroData.gyro.z, 5);
  client.print(",\"mac\":\"");
  client.print(macString);
  client.print("\",\"nettype\":\"");
  client.print(nettype);
  client.print("\",\"timestamp\":");
  client.print((long)now);
  client.print(",\"tzoffset\":");
  client.print(tz_offset_sec);
  client.print(",\"mode\":");
  client.print(mode);
  client.print(",\"axis\":\"");
  client.print(display_axis == 0 ? 'X' : 'Y');
  client.print("\"");
  client.print(",\"spectrogramMin\":");
  client.print(spectrogram_min);
  client.print(",\"spectrogramMax\":");
  client.print(spectrogram_max);
  client.println("}");
}

void sendSensorUDPStream(const char *channel, const float *data, size_t count)
{
  if (count == 0)
    return;

  time_t now = time(NULL);

  char packet[128 + UDP_PACKET_SIZE * 16] = {0};
  int len = snprintf(packet, sizeof(packet), "%s,%ld", channel, (long)now);

  for (size_t i = 0; i < count; ++i)
  {
    len += snprintf(packet + len, sizeof(packet) - len, ",%.5f", data[i]);
  }
  snprintf(packet + len, sizeof(packet) - len, "\n");

#if defined(USE_WIFI)
  if (netMode == NET_WIFI)
  {
    udpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    udpStream.write((const uint8_t *)packet, strlen(packet));
    udpStream.endPacket();
  }
#endif
#if defined(USE_WIZNET)
  if (netMode == NET_WIZNET)
  {
    udpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    udpStream.write((const uint8_t *)packet, strlen(packet));
    udpStream.endPacket();
  }
#endif
}

// Overloaded for int data
void sendSensorUDPStream(const char *channel, const int *data, size_t count)
{
  if (count == 0)
    return;

  time_t now = time(NULL);

  char packet[128 + UDP_PACKET_SIZE * 16] = {0};
  int len = snprintf(packet, sizeof(packet), "%s,%ld", channel, (long)now);

  for (size_t i = 0; i < count; ++i)
  {
    len += snprintf(packet + len, sizeof(packet) - len, ",%d", data[i]);
  }
  snprintf(packet + len, sizeof(packet) - len, "\n");

#if defined(USE_WIFI)
  if (netMode == NET_WIFI)
  {
    udpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    udpStream.write((const uint8_t *)packet, strlen(packet));
    udpStream.endPacket();
  }
#endif
#if defined(USE_WIZNET)
  if (netMode == NET_WIZNET)
  {
    udpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    udpStream.write((const uint8_t *)packet, strlen(packet));
    udpStream.endPacket();
  }
#endif
}

void serveUDPStreamControl(Stream &client, const String &req)
{
  int chIdx = -1;
  if (req.indexOf("velocX") != -1)
    chIdx = 0;
  else if (req.indexOf("velocY") != -1)
    chIdx = 1;
  else if (req.indexOf("accelX") != -1)
    chIdx = 2;
  else if (req.indexOf("accelY") != -1)
    chIdx = 3;
  else if (req.indexOf("accelZ") != -1)
    chIdx = 4;

  int enableIdx = req.indexOf("enable=");
  bool enable = false;
  if (enableIdx != -1)
  {
    enable = (req.charAt(enableIdx + 7) == '1');
  }
  if (chIdx >= 0)
    udpStreamEnable[chIdx] = enable;

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.printf("OK %d %d\n", chIdx, enable ? 1 : 0);
}

void serveModeControl(Stream &client, const String &req)
{
  // Parse mode parameter from request: /setmode?mode=X
  int modeIdx = req.indexOf("mode=");
  if (modeIdx != -1)
  {
    // Extract mode value (single digit or negative)
    String modeStr = req.substring(modeIdx + 5); // Skip "mode="
    // Find first non-digit/non-minus character
    int endIdx = 0;
    if (modeStr.charAt(0) == '-')
      endIdx = 1;
    while (endIdx < modeStr.length() && isdigit(modeStr.charAt(endIdx)))
      endIdx++;
    modeStr = modeStr.substring(0, endIdx);

    int newMode = modeStr.toInt();
    // Validate mode range: -1 to 3
    if (newMode >= -1 && newMode <= 3)
    {
      mode = newMode;
      // Reset mode-specific flags when mode changes
      text_mode_initialized = false;
      waveform_initialized = false;
      mode23_screen_cleared = false;

      debugPrintf("Mode changed to: %d\n", mode);
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.printf("OK mode=%d\n", mode);
}

void serveAxisControl(Stream &client, const String &req)
{
  // Parse axis parameter from request: /setaxis?axis=X or /setaxis?axis=Y
  int axisIdx = req.indexOf("axis=");
  if (axisIdx != -1)
  {
    String axisStr = req.substring(axisIdx + 5, axisIdx + 6); // Get single character
    axisStr.toUpperCase();

    if (axisStr == "X")
    {
      display_axis = 0;
      // Reset display flags to force re-initialization with new axis data
      text_mode_initialized = false;
      waveform_initialized = false;
      mode23_screen_cleared = false;
      debugPrintln("Display axis changed to X");
    }
    else if (axisStr == "Y")
    {
      display_axis = 1;
      // Reset display flags to force re-initialization with new axis data
      text_mode_initialized = false;
      waveform_initialized = false;
      mode23_screen_cleared = false;
      debugPrintln("Display axis changed to Y");
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.printf("OK axis=%c\n", display_axis == 0 ? 'X' : 'Y');
}

void serveTouchCalibrate(Stream &client, const String &req)
{
  // Parse points parameter from request: /touchcalibrate?points=5 or points=9
  int pointsIdx = req.indexOf("points=");
  if (pointsIdx != -1)
  {
    String pointsStr = req.substring(pointsIdx + 7, pointsIdx + 8); // Get single digit
    int points = pointsStr.toInt();

    if (points == 5 || points == 9)
    {
      NUM_CALIB_POINTS = points;
      trigger_touch_calibration = true;
      debugPrintf("Touch calibration requested with %d points\n", points);

      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/plain");
      client.println("Connection: close");
      client.println();
      client.printf("OK calibration started with %d points\n", points);
      return;
    }
  }

  client.println("HTTP/1.1 400 Bad Request");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("ERROR: Invalid points parameter (use 5 or 9)");
}

void serveTouchCalibData(Stream &client, const String &req)
{
  File f = LittleFS.open("/touch_calib.txt", "r");

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();

  if (f)
  {
    client.println("=== touch_calib.txt content ===");
    while (f.available())
    {
      client.write(f.read());
    }
    client.println("\n=== End of file ===");
    f.close();

    // Also print to serial
    debugPrintln("Touch calibration data requested via HTTP");
    f = LittleFS.open("/touch_calib.txt", "r");
    if (f)
    {
      Serial.println("\n=== touch_calib.txt content ===");
      while (f.available())
      {
        Serial.write(f.read());
      }
      Serial.println("\n=== End of file ===");
      f.close();
    }
  }
  else
  {
    client.println("touch_calib.txt not found!");
    debugPrintln("touch_calib.txt not found!");
  }
}

// Helper to format MAC address as string
void formatMACString(byte *mac, char *out)
{
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**
 * @brief Set the local timezone for the system.
 * @param tz POSIX timezone string, e.g. "CET-1CEST,M3.5.0,M10.5.0/3" for Central Europe
 */
void setLocalTimezone(const char *tz)
{
  setenv("TZ", tz, 1);
  tzset();
}

void setup_waveform()
{
  // Clear the display
  tft.fillScreen(ILI9341_BLACK);

  // Draw center line
  tft.drawLine(waveform_x, waveform_mid_y,
               waveform_x + waveform_w, waveform_mid_y,
               ILI9341_LIGHTGREY);

  // Calculate trigger level positions
  // Upper trigger at 1/8 from top, lower trigger at 1/8 from bottom
  // Spacing between triggers: 6/8 of screen height
  // Offset from center: 3/8 of screen height = 90 pixels
  int trigger_offset = 3 * waveform_h / 8; // 90 pixels from center
  int upper_trigger_y = waveform_mid_y - trigger_offset;
  int lower_trigger_y = waveform_mid_y + trigger_offset;

  // Draw dashed horizontal lines for trigger levels
  for (int x = waveform_x; x < waveform_x + waveform_w; x += 4)
  {
    tft.drawPixel(x, upper_trigger_y, ILI9341_LIGHTGREY);
    tft.drawPixel(x + 1, upper_trigger_y, ILI9341_LIGHTGREY);
    tft.drawPixel(x, lower_trigger_y, ILI9341_LIGHTGREY);
    tft.drawPixel(x + 1, lower_trigger_y, ILI9341_LIGHTGREY);
  }

  // Initialize last_points to center line
  for (int i = 0; i < LAST_POINTS_SIZE; ++i)
    last_points[i] = waveform_mid_y;
}

void draw_fft_frequency_axis()
{
  // Draw frequency axis at bottom of FFT display
  // Sample rate: 500 SPS, FFT size: 1024, so Nyquist freq = 250 Hz
  // Display width: 320 pixels (showing FFT_SIZE/2 = 512 bins)
  // But waveform displays in compressed form, need to check actual display

  const int axis_y = 228;                       // Position to leave room for labels (text height = 8 pixels)
  const float sample_rate = 500.0;              // SPS
  const float nyquist_freq = sample_rate / 2.0; // 250 Hz

  // Draw axis line
  tft.drawLine(0, axis_y, 320, axis_y, ILI9341_WHITE);

  // Draw frequency labels: 0, 50, 100, 150, 200, 250 Hz
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_WHITE);

  for (int freq = 0; freq <= 250; freq += 50)
  {
    int x_pos = (int)(freq / nyquist_freq * 320);
    // Draw tick mark
    tft.drawLine(x_pos, axis_y, x_pos, axis_y - 3, ILI9341_WHITE);
    // Draw label (positioned above the axis line to stay on screen)
    // Adjust label position based on number of digits and keep rightmost label on screen
    int label_offset;
    if (freq == 0)
      label_offset = 0; // Left-align "0"
    else if (freq == 250)
      label_offset = 18; // Right-align "250" (3 digits × 6 pixels = 18)
    else if (freq < 100)
      label_offset = 6; // Center 2-digit numbers
    else
      label_offset = 9; // Center 3-digit numbers

    tft.setCursor(x_pos - label_offset, axis_y - 10);
    tft.print(freq);
  }

  // Label the axis
  tft.setCursor(280, axis_y - 10);
  tft.print("Hz");
}

void waveform_display(short *samples, int n_samples, int color, bool draw_triggers = true)
{
  // Draw exactly n_samples points (should match screen width)
  int last_x = waveform_x;
  int last_y = waveform_mid_y;
  int last_old_y = last_y;

  // Calculate scale factor so CROSSING_THRESHOLD is at 1/8 from top/bottom
  // 3*waveform_h/8 = 90 pixels from center should correspond to CROSSING_THRESHOLD
  const float scale = (3.0f * waveform_h / 8.0f) / CROSSING_THRESHOLD; // ~0.009

  // Calculate trigger level positions for redrawing
  int trigger_offset = 3 * waveform_h / 8;
  int upper_trigger_y = waveform_mid_y - trigger_offset;
  int lower_trigger_y = waveform_mid_y + trigger_offset;

  for (int i = 0; i < n_samples; ++i)
  {
    int x_val = waveform_x + i;
    // Scale samples so trigger levels appear at 1/8 from edges
    int y_val = waveform_mid_y - (int)(samples[i] * scale);
    int old_y_val = last_points[i];

    // Choose color: red if outside threshold, otherwise use specified color
    bool outside_threshold = (samples[i] > CROSSING_THRESHOLD) || (samples[i] < -CROSSING_THRESHOLD);
    int line_color = outside_threshold ? ILI9341_RED : color;

    if (i > 0)
    {
      tft.drawLine(last_x, last_old_y, x_val, old_y_val, ILI9341_BLACK);
      tft.drawLine(last_x, last_y, x_val, y_val, line_color);

      // Redraw trigger level dashes if waveform passes through them (only in waveform mode)
      if (draw_triggers && ((x_val % 4) == 0 || (x_val % 4) == 1))
      {
        tft.drawPixel(x_val, upper_trigger_y, ILI9341_LIGHTGREY);
        tft.drawPixel(x_val, lower_trigger_y, ILI9341_LIGHTGREY);
      }
    }
    last_x = x_val;
    last_y = y_val;
    last_old_y = old_y_val;
    last_points[i] = y_val;
  }

  // Redraw menu indicator after waveform to keep it visible
  drawMenuIndicator();
}

// ------- spectrogram plot -----------

// from https://github.com/ArmDeveloperEcosystem/audio-spectrogram-example-for-pico

// import numpy as np
// import matplotlib as mpl
//
// cmap = mpl.colormaps['magma']
// s = ''
// for i in range(256):
//   r, g, b, a = cmap(i / 256, bytes=True)
//   val565 = ((r & 0xF8) << (11 - 3)) | ((g & 0xFC) << (5 - 2)) | ((b & 0xF8) >> 3)
//   s += '0x%x, ' % val565
//   if (i + 1) % 9 == 0:
//     print(s)
//     s = ''
// print(s)

// Magma colormap - compact 16-entry lookup table with linear interpolation (RGB565 format)
// Replaces 257-line COLOR_MAP array, saves ~500 bytes of code space
const uint16_t MAGMA_LUT[16] PROGMEM = {
    0x0000, // 0:   Black
    0x0823, // 16:  Dark purple
    0x190e, // 32:  Purple
    0x2994, // 48:  Deep purple
    0x423c, // 64:  Purple-red
    0x5325, // 80:  Red-purple
    0x6c0d, // 96:  Dark red
    0x7d35, // 112: Red
    0x8e7d, // 128: Orange-red
    0x9fc3, // 144: Orange
    0xb0c6, // 160: Light orange
    0xc1c8, // 176: Yellow-orange
    0xd2c7, // 192: Yellow
    0xe3e3, // 208: Light yellow
    0xf4fc, // 224: Pale yellow
    0xfff7  // 240: White-yellow
};

// Fast magma colormap lookup with linear interpolation
inline uint16_t getMagmaColor(uint8_t value)
{
  // Determine which two LUT entries to interpolate between
  uint8_t index = value >> 4;  // Divide by 16 to get LUT index (0-15)
  uint8_t frac = value & 0x0F; // Remainder for interpolation (0-15)

  if (index >= 15)
    return pgm_read_word(&MAGMA_LUT[15]); // Clamp to max

  // Read two adjacent colors from PROGMEM
  uint16_t c1 = pgm_read_word(&MAGMA_LUT[index]);
  uint16_t c2 = pgm_read_word(&MAGMA_LUT[index + 1]);

  // Extract RGB565 components
  uint8_t r1 = (c1 >> 11) & 0x1F;
  uint8_t g1 = (c1 >> 5) & 0x3F;
  uint8_t b1 = c1 & 0x1F;

  uint8_t r2 = (c2 >> 11) & 0x1F;
  uint8_t g2 = (c2 >> 5) & 0x3F;
  uint8_t b2 = c2 & 0x1F;

  // Linear interpolation (frac/16)
  uint8_t r = r1 + ((r2 - r1) * frac >> 4);
  uint8_t g = g1 + ((g2 - g1) * frac >> 4);
  uint8_t b = b1 + ((b2 - b1) * frac >> 4);

  // Pack back to RGB565
  return (r << 11) | (g << 5) | b;
}

// Forward declaration
void drawMenuIndicator(int scroll_offset);

void add_display_column(short *values, int n_values)
{
  // Adaptive scaling for spectrogram: track min/max over time
  static short running_min = 32767;
  static short running_max = -32768;
  static int frame_count = 0;

  // Reset running min/max periodically to adapt to changing signal levels
  if (++frame_count >= 100) // Reset every 100 frames (~10 seconds at 10 Hz)
  {
    running_min = 32767;
    running_max = -32768;
    frame_count = 0;
  }

  // Update running min/max from current column
  for (int i = 0; i < MIN(waveform_h, n_values); ++i)
  {
    if (values[i] < running_min)
      running_min = values[i];
    if (values[i] > running_max)
      running_max = values[i];
  }

  // Update global values for external access (e.g., web UI)
  spectrogram_min = running_min;
  spectrogram_max = running_max;

  // Prevent division by zero
  int range = running_max - running_min;
  if (range < 1)
    range = 1;

  // When menu is visible, pause scrolling to keep menu fixed
  if (menu_visible)
  {
    // Don't scroll, just draw at current position
    int x = waveform_x + disp_column;

    // Only draw outside menu area (x >= 80)
    if (x >= 80)
    {
      for (int i = 0; i < MIN(waveform_h, n_values); ++i)
      {
        int y = waveform_y + waveform_h - i;
        int v = ((long)(values[i] - running_min) * 255) / range;
        v = MAX(0, MIN(255, v));
        int color = getMagmaColor(v);
        tft.drawPixel(x, y, color);
      }
    }
    disp_column = (disp_column + 1) % waveform_w;
    return;
  }

  // Normal scrolling mode when menu is not visible
  int x = waveform_x + disp_column;
  for (int i = 0; i < MIN(waveform_h, n_values); ++i)
  {
    int y = waveform_y + waveform_h - i;

    // Scale value from [running_min, running_max] to [0, 255]
    int v = ((long)(values[i] - running_min) * 255) / range;
    v = MAX(0, MIN(255, v)); // Clamp to valid range

    int color = getMagmaColor(v);
    tft.drawPixel(x, y, color);
  }
  disp_column = (disp_column + 1) % waveform_w;
  tft.scrollTo(disp_column);

  drawMenuIndicator(disp_column);
}

void setup_buffer()
{
  for (int i = 0; i < RING_BUFFER_SIZE; ++i)
  {
    ring_buffer[i] = 0;
  }
}

void buffer_feed(int sample)
{
  ring_buffer[ring_buffer_tail++] = sample;
  ring_buffer_tail &= RING_BUFFER_SIZE - 1;
}

enum CrossingDirection
{
  CROSSING_POSITIVE = 0, // Positive-going crossing only
  CROSSING_NEGATIVE = 1, // Negative-going crossing only
  CROSSING_BOTH = 2      // Either direction
};

int find_crossing(int min_size, int threshold, CrossingDirection direction)
{
  int tail = ring_buffer_tail; // Snapshot volatile.

  // Check if we have enough samples in the buffer
  if (tail < min_size + 1)
  {
    static unsigned long last_warn = 0;
    if (millis() - last_warn > 2000)
    {
      Serial.print("find_crossing: Not enough samples yet. tail=");
      Serial.print(tail);
      Serial.print(", need at least ");
      Serial.println(min_size + 1);
      last_warn = millis();
    }
    return 0; // Not enough data yet
  }

  short last_sample = ring_buffer[(tail - min_size) & (RING_BUFFER_SIZE - 1)];

  // Only scan recent history (4096 samples) for faster detection
  int max_scan = min_size + 4096;
  if (max_scan > RING_BUFFER_SIZE)
    max_scan = RING_BUFFER_SIZE;

  for (int i = min_size + 1; i < max_scan; ++i)
  {
    // Scan backwards, so we find the most recent transition with at least min_size.
    int index = (tail - i) & (RING_BUFFER_SIZE - 1);
    short next_sample = ring_buffer[index];

    // Check if signal exits the interval [-threshold, +threshold]
    bool last_inside = (last_sample >= -threshold) && (last_sample <= threshold);

    if (direction == CROSSING_POSITIVE || direction == CROSSING_BOTH)
    {
      // Looking for signal going from inside interval to outside positive
      if (last_inside && (next_sample > threshold))
      {
        return index;
      }
    }

    if (direction == CROSSING_NEGATIVE || direction == CROSSING_BOTH)
    {
      // Looking for signal going from inside interval to outside negative
      if (last_inside && (next_sample < -threshold))
      {
        return index;
      }
    }

    last_sample = next_sample;
  }

  // Didn't find a crossing.
  return 0;
}

void buffer_get_latest(short *buffer, int n_samples, int scalebits)
{
  // Copy most recent values into output buffer.
  int from = (ring_buffer_tail - n_samples) & (RING_BUFFER_SIZE - 1);
  for (int i = 0; i < n_samples; ++i)
  {
    *buffer++ = ring_buffer[from++] << scalebits;
    from &= RING_BUFFER_SIZE - 1;
  }
}

// ----- dsp ------
// #define USE_FIXEDPOINT

#ifdef USE_FIXEDPOINT

#define SAMPLE_ q15_t
#define float_to_sample(from, to, len) arm_float_to_q15(from, to, len)
#define short_to_sample(from, to, len) bcopy(from, to, len * sizeof(short))
#define sample_to_short(from, to, len) bcopy(from, to, len * sizeof(short))

#define FFT_STRUCT arm_rfft_instance_q15
#define rfft_init(fft_struct, fft_size) arm_rfft_init_q15(fft_struct, fft_size, 0, 1)
#define mult(in1, in2, out, len) arm_mult_q15(in1, in2, out, len)
#define rfft(fft_struct, in, out) arm_rfft_q15(fft_struct, in, out)
#define cmplx_mag(in, out, len) arm_cmplx_mag_q15(in, out, len)
#define vlog(in, out, len)            /* not implemented */
#define vscale(in, scale, out, len)   /* not implemented */
#define voffset(in, offset, out, len) /* not implemented */

#else // floating point

#define SAMPLE_ float32_t
#define float_to_sample(from, to, len) bcopy(from, to, len * sizeof(float32_t))
#define short_to_sample(from, to, len) arm_q15_to_float(from, to, len)
#define sample_to_short(from, to, len) arm_float_to_q15(from, to, len)

#define FFT_STRUCT arm_rfft_fast_instance_f32
#define rfft_init(fft_struct, fft_size) arm_rfft_fast_init_f32(fft_struct, fft_size)
#define mult(in1, in2, out, len) arm_mult_f32(in1, in2, out, len)
#define rfft(fft_struct, in, out) arm_rfft_fast_f32(fft_struct, in, out, 0)
#define cmplx_mag(in, out, len) arm_cmplx_mag_f32(in, out, len)
#define vlog(in, out, len) arm_vlog_f32(in, out, len)
#define vscale(in, scale, out, len) arm_scale_f32(in, scale, out, len)
#define voffset(in, offset, out, len) arm_offset_f32(in, offset, out, len)

#endif

SAMPLE_ input_buf[FFT_SIZE];
SAMPLE_ window_buf[FFT_SIZE];
SAMPLE_ windowed_input_buf[FFT_SIZE];

FFT_STRUCT S_;

SAMPLE_ fft_buf[FFT_SIZE * 2];
SAMPLE_ fft_mag_buf[FFT_SIZE / 2];
SAMPLE_ mag_spec[mag_spec_len];
short mag_spec_short[mag_spec_len];
short input_shorts_buf[FFT_SIZE];

void hanning_window_init(SAMPLE_ *window, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    window[i] = 0.5 * (1.0 - arm_cos_f32(2 * PI * i / FFT_SIZE));
  }
}

void setup_dsp()
{
  // initialize the hanning window and RFFT instance
  hanning_window_init(window_buf, FFT_SIZE);
  rfft_init(&S_, FFT_SIZE);
}

int magnitude_spectrum(SAMPLE_ *in_buf, SAMPLE_ *out_buf, int len)
{
  int n_out_points = FFT_SIZE / 2;
  assert(len >= n_out_points);

  // mult(window_buf, in_buf, out_buf, FFT_SIZE);
  // return FFT_SIZE / 2;

  // apply the DSP pipeline: Hanning Window + FFT
  // remove DC
  SAMPLE_ sum = 0;
  for (int i = 0; i < FFT_SIZE; ++i)
    sum += in_buf[i];
  voffset(in_buf, -sum / FFT_SIZE, in_buf, FFT_SIZE);
  // window & fft & mag
  mult(window_buf, in_buf, windowed_input_buf, FFT_SIZE);

  rfft(&S_, windowed_input_buf, fft_buf);
  cmplx_mag(fft_buf, out_buf, n_out_points);

  vlog(out_buf, out_buf, n_out_points);

  // Scale log magnitude for FFT display from bottom of screen
  // After log: typical range 6-13 for varying amplitudes
  // Map to 0 (bottom) to +0.4 (peaks going up ~100 pixels from mid_y=120)
  voffset(out_buf, (float32_t)-6.0, out_buf, n_out_points);       // Shift: log(6)=6 -> 0, log(50000)=10.8 -> 4.8
  vscale(out_buf, (float32_t)(0.4 / 8.0), out_buf, n_out_points); // Scale: 0-8 range -> 0 to 0.4
  arm_clip_f32(out_buf, out_buf, 0.0, 0.4, n_out_points);         // Clip to prevent overflow

  // for debug: short-circuit the output.
  // bcopy(windowed_input_buf, out_buf, n_out_points);

  return n_out_points;
}

// ------- Touch Menu System -------

// Menu button definitions
struct MenuButton
{
  int x, y, w, h;
  const char *label;
  int mode_value;
  uint16_t color;
};

// Define menu buttons for mode selection
MenuButton menu_buttons[NUM_MENU_BUTTONS] = {
    {10, 40, 60, 40, "OFF", -1, ILI9341_RED},
    {10, 90, 60, 40, "TEXT", 0, ILI9341_BLUE},
    {10, 140, 60, 40, "WAVE", 1, ILI9341_GREEN},
    {10, 190, 60, 40, "FFT", 2, ILI9341_CYAN},
    {10, 240, 60, 40, "SGRAM", 3, ILI9341_MAGENTA}};

unsigned long menu_show_time = 0;
const unsigned long MENU_TIMEOUT_MS = 5000; // Menu auto-hides after 5 seconds

// Draw the mode selection menu
void drawModeMenu()
{
  // Draw semi-transparent background (simulate with black box)
  tft.fillRect(0, 20, 80, 280, ILI9341_BLACK);

  // Draw title
  tft.setCursor(10, 25);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.print("MODE");

  // Draw buttons
  for (int i = 0; i < NUM_MENU_BUTTONS; i++)
  {
    MenuButton &btn = menu_buttons[i];
    tft.fillRect(btn.x, btn.y, btn.w, btn.h, btn.color);
    tft.drawRect(btn.x, btn.y, btn.w, btn.h, ILI9341_WHITE);

    // Center text in button
    tft.setCursor(btn.x + 5, btn.y + 15);
    tft.setTextColor(ILI9341_WHITE, btn.color);
    tft.setTextSize(1);
    tft.print(btn.label);
  }

  menu_visible = true;
  menu_show_time = millis();
}

// Draw a small menu indicator in the top-left corner
// scroll_offset: current scroll position to account for scrolling
void drawMenuIndicator(int scroll_offset)
{
  // Account for different rotation settings
  // Rotation 0: Portrait (240w x 320h) - scrollTo scrolls vertically
  // Rotation 1: Landscape (320w x 240h) - scrollTo scrolls horizontally
  // Rotation 2: Portrait inverted (240w x 320h) - scrollTo scrolls vertically
  // Rotation 3: Landscape inverted (320w x 240h) - scrollTo scrolls horizontally

  int x_base = 2;
  int y_base = 0;

  switch (TFT_ROTATION)
  {
  case 0: // Portrait - scroll affects y
    y_base = scroll_offset % 320;
    break;
  case 1: // Landscape - scroll affects x
    x_base = scroll_offset % 320 + 2;
    break;
  case 2: // Portrait inverted - scroll affects y
    y_base = scroll_offset % 320;
    break;
  case 3: // Landscape inverted - scroll affects x
    x_base = scroll_offset % 320 + 2;
    break;
  }

  // Clear background with a slightly larger area to remove old border pixels from scrolling
  tft.fillRect(x_base - 1, y_base, 18, 16, ILI9341_BLACK);

  // Draw a small "hamburger" menu icon (3 horizontal lines)
  tft.fillRect(x_base + 3, y_base + 2, 10, 2, ILI9341_WHITE);
  tft.fillRect(x_base + 3, y_base + 7, 10, 2, ILI9341_WHITE);
  tft.fillRect(x_base + 3, y_base + 12, 10, 2, ILI9341_WHITE);
  // Add a small border
  tft.drawRect(x_base, y_base, 16, 16, COLOR_GREY);
}

// Check if a touch point is within a button
bool isTouchInButton(int touch_x, int touch_y, MenuButton &btn)
{
  return (touch_x >= btn.x && touch_x <= (btn.x + btn.w) &&
          touch_y >= btn.y && touch_y <= (btn.y + btn.h));
}

// Handle touch input for menu and mode selection
// Returns true if mode was changed
bool handleTouchInput(volatile int &current_mode)
{
  static unsigned long last_touch_time = 0;
  static bool touch_was_pressed = false;
  const unsigned long DEBOUNCE_MS = 200;

#ifdef DEBUG_TOUCH_EVENTS
  static unsigned long last_poll_debug = 0;
  if (millis() - last_poll_debug > 5000)
  {
    Serial.println("handleTouchInput: polling...");
    last_poll_debug = millis();
  }
#endif

  // Check if enough time has passed since last touch (debounce)
  if (millis() - last_touch_time < DEBOUNCE_MS)
  {
    return false;
  }

  // Get touch input
#ifdef USE_HARDWARE_SPI
  setTouchSPISpeed(); // Set SPI to touchscreen speed before reading

  // Check if touchscreen is being touched first
  bool is_touched = touchscreen.touched();
  TS_Point tp;
  if (is_touched)
  {
    tp = touchscreen.getPoint();
  }
  else
  {
    // When not touched, still call getPoint() to keep touchscreen active
    tp = touchscreen.getPoint();
  }

  setDisplaySPISpeed(); // Restore SPI to display speed
#else
  TouchPoint tp = touchscreen.getTouch();
  bool is_touched = (tp.z > 0);
#endif

#ifdef DEBUG_TOUCH_EVENTS
  // Log raw touch data for debugging
  static unsigned long last_debug_time = 0;
  static int debug_counter = 0;
  if (millis() - last_debug_time > 1000)
  {
    Serial.printf("Touch poll #%d: touched=%d, x=%d, y=%d, z=%d\n",
                  debug_counter++, is_touched, tp.x, tp.y, tp.z);
    last_debug_time = millis();
  }
#endif

  // Check for touch press (rising edge detection)
  if (tp.z > 0)
  {
    if (!touch_was_pressed)
    {
      // New touch detected
      touch_was_pressed = true;
      last_touch_time = millis();

      // Convert touch coordinates to display coordinates
      int display_x, display_y;
      applyMMSECalibration(tp.x, tp.y, display_x, display_y);

#ifdef DEBUG_TOUCH_EVENTS
      Serial.print("Touch event detected at display coordinates: x=");
      Serial.print(display_x);
      Serial.print(", y=");
      Serial.println(display_y);
#endif

      // Check if touch is in the menu toggle area (top-left corner)
      if (!menu_visible && display_x < 80 && display_y < 20)
      {
#ifdef DEBUG_TOUCH_EVENTS
        Serial.println("Menu symbol pressed - showing menu");
#endif
        // Show menu
        drawModeMenu();
        return false;
      }

#ifdef DEBUG_TOUCH_EVENTS
      if (!menu_visible)
      {
        Serial.println("Touch outside menu symbol area");
      }
#endif

      // If menu is visible, check button touches
      if (menu_visible)
      {
        for (int i = 0; i < NUM_MENU_BUTTONS; i++)
        {
          if (isTouchInButton(display_x, display_y, menu_buttons[i]))
          {
#ifdef DEBUG_TOUCH_EVENTS
            Serial.print("Menu button pressed: ");
            Serial.print(menu_buttons[i].label);
            Serial.print(" (mode ");
            Serial.print(menu_buttons[i].mode_value);
            Serial.println(")");
#endif
            // Mode button pressed
            int new_mode = menu_buttons[i].mode_value;
            if (new_mode != current_mode)
            {
              current_mode = new_mode;
              menu_visible = false;

              // Clear screen for new mode
              tft.fillScreen(ILI9341_BLACK);

              debugPrint("Mode changed to: ");
              debugPrintf("%d\n", current_mode);
              return true;
            }
          }
        }
#ifdef DEBUG_TOUCH_EVENTS
        Serial.println("Touch in menu area but outside buttons");
#endif
      }
    }
  }
  else
  {
    // No touch detected
    touch_was_pressed = false;
  }

  // Auto-hide menu after timeout
  if (menu_visible && (millis() - menu_show_time > MENU_TIMEOUT_MS))
  {
    menu_visible = false;
    tft.fillScreen(ILI9341_BLACK);
    // Redraw menu indicator after timeout
    if (current_mode > 0)
    {
      drawMenuIndicator();
    }
  }

  return false;
}

void setup()
{
  arduino::String resultStr;

  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  sleep_ms(1000);
  initTFTDisplay(); // Initialize TFT display early to show debug messages
  tftPrint("Starting Pier Vibration Sensor...\n");
  LittleFS.begin(); // Initialize LittleFS filesystem
  tftPrint("Filesystem initialized...\n");

#ifdef PRINT_TOUCH_CALIB_FILE
  // Print touch calibration file content to serial for backup
  File f = LittleFS.open("/touch_calib.txt", "r");
  if (f)
  {
    Serial.println("\n=== touch_calib.txt content ===");
    while (f.available())
    {
      Serial.write(f.read());
    }
    Serial.println("\n=== End of file ===");
    f.close();
  }
  else
  {
    Serial.println("touch_calib.txt not found!");
  }
#endif
  tftPrint("---------------------------------------------\n");
  tftPrint("Initializing XPT2046 touch controller...\n");
  tftPrint("Start touch controller calibration...\n");

  bool calibrationPerformed = false;

  if (!loadTouchCalibration())
  {
    tftPrint("No touch calibration found, starting calibration...\n");
    // calibrateTouchController() will initialize the touchscreen
    calibrateTouchController();
    calibrationPerformed = true;
  }
  else
  {
#ifdef FORCE_TOUCH_CALIBRATION
    tftPrint("Forcing touch calibration...\n");
    calibrateTouchController();
    calibrationPerformed = true;
#else
    tftPrint("Touch calibration loaded from /touch_calib.txt...\n");
    // Initialize touchscreen the same way as calibrateTouchController() does
    Serial.println("Initializing touchscreen with loaded calibration...");

    // Set display rotation
    tft.setRotation(TFT_ROTATION);

    // Reset hardware scrolling to ensure correct coordinate mapping
    tft.scrollTo(0);

#ifdef USE_HARDWARE_SPI
    // Set SPI speed to touchscreen speed before initialization
    Serial.println("Setting SPI speed for touchscreen...");
    setTouchSPISpeed();
#endif

    // Initialize touch controller - begin() will handle CS pin setup
    touchscreen.begin();
#ifdef USE_HARDWARE_SPI
    touchscreen.setRotation(TFT_ROTATION);
#endif

    tftPrint("Touch controller initialized...\n");
    tft.setCursor(0, 0);
#endif
  }

  initADS1256();
  // initMPU6500();
  initICM20948();

  // Set your local timezone here (example: Central Europe)
  setLocalTimezone(LOCAL_TIMEZONE);

#if defined(USE_WIFI)
  initWiFi();
  if (netMode == NET_WIFI)
  {
    byte mac[6];
    getWiFiMAC(mac);
    formatMACString(mac, macString);
  }
#elif defined(USE_WIZNET)
  initWIZNET5K();
  if (netMode == NET_WIZNET)
  {
    byte mac[6];
    getWiznetMAC(mac);
    formatMACString(mac, macString);
  }
#endif

  if (!setTimeFromNTP())
  {
    tftPrint("Failed to set system time from NTP server.\n");
  }

  server.begin();
  tftPrint("HTTP server started...\n");
  resultStr = "Serving chart page at http://" + Ethernet.localIP().toString() + ":" + String(HTTP_SERVER_PORT) + "\n";
  tftPrint(resultStr.c_str());
  resultStr = "UDP stream port: " + String(UDP_STREAM_PORT) + "\n";
  tftPrint(resultStr.c_str());
  resultStr = "UDP packet size: " + String(UDP_PACKET_SIZE) + "\n";
  tftPrint(resultStr.c_str());
  tftPrint("---------------------------------------------\n");
  // setupTFTDisplayForSpectrogram();
  // setup_waveform();
  setup_dsp();

  tftPrint("---------------------------------------------\n");
  tftPrint("System initialized.\n");

  // Re-initialize touchscreen after all other peripherals to ensure it's in correct state
#ifdef USE_HARDWARE_SPI
  setTouchSPISpeed();
  touchscreen.begin();
  touchscreen.setRotation(TFT_ROTATION);
  delay(1000); // Wait for chip to stabilize

  // Drain any phantom touches
  for (int i = 0; i < 10; i++)
  {
    touchscreen.getPoint();
    delay(10);
  }
#endif

  tftPrint("Starting Main Loop...\n");
  Serial.println("Main loop started - collecting sensor data");
}

// Ring buffer sensor selection constants
#define RINGBUFFER_VELOCX_RAW 0
#define RINGBUFFER_VELOCY_RAW 1
#define RINGBUFFER_ACCELX_RAW 2
#define RINGBUFFER_ACCELY_RAW 3
#define RINGBUFFER_ACCELZ_RAW 4

void loop()
{
  static int velocXraw_block[UDP_PACKET_SIZE];
  static int velocYraw_block[UDP_PACKET_SIZE];
  static float velocX_block[UDP_PACKET_SIZE];
  static float velocY_block[UDP_PACKET_SIZE];
  static int accelXraw_block[UDP_PACKET_SIZE];
  static int accelYraw_block[UDP_PACKET_SIZE];
  static int accelZraw_block[UDP_PACKET_SIZE];
  static float accelX_block[UDP_PACKET_SIZE];
  static float accelY_block[UDP_PACKET_SIZE];
  static float accelZ_block[UDP_PACKET_SIZE];
  static int gyroXraw_block[UDP_PACKET_SIZE];
  static int gyroYraw_block[UDP_PACKET_SIZE];
  static int gyroZraw_block[UDP_PACKET_SIZE];
  static float gyroX_block[UDP_PACKET_SIZE];
  static float gyroY_block[UDP_PACKET_SIZE];
  static float gyroZ_block[UDP_PACKET_SIZE];
  static size_t block_idx = 0;
  static uint8_t icm_read_counter = 0; // Counter to reduce ICM20948 read frequency

  const uint8_t myMuxList[] = {SING_0, SING_7};                                     // or any list of SING_x constants
  adcValue0 = ads.cycleSingle(myMuxList, sizeof(myMuxList) / sizeof(myMuxList[0])); // Read single-ended channel 0 (Veloc X) selecting channel 7 as next channel
  velocX = ads.convertToVoltage(adcValue0) * 1000000;                               // Convert raw ADC value to uV

  adcValue7 = ads.cycleSingle(nullptr, 0);            // Read single-ended channel 7 (Veloc Y) selecting channel 0 as next channel
  velocY = ads.convertToVoltage(adcValue7) * 1000000; // Convert raw ADC value to uV

  // Read ICM20948 only every 5th iteration to improve overall sampling rate
  // This gives ~40 Hz for IMU data which is sufficient for slow vibrations
  icm_read_counter++;
  if (icm_read_counter >= 5)
  {
    readICM20948();
    icm_read_counter = 0;
  }

  velocXraw_block[block_idx] = adcValue0;
  velocYraw_block[block_idx] = adcValue7;
  velocX_block[block_idx] = velocX;
  velocY_block[block_idx] = velocY;
  accelX_block[block_idx] = accelData.acceleration.x;
  accelY_block[block_idx] = accelData.acceleration.y;
  accelZ_block[block_idx] = accelData.acceleration.z;
  accelXraw_block[block_idx] = accelData.data[0];
  accelYraw_block[block_idx] = accelData.data[1];
  accelZraw_block[block_idx] = accelData.data[2];
  gyroXraw_block[block_idx] = gyroData.data[0];
  gyroYraw_block[block_idx] = gyroData.data[1];
  gyroZraw_block[block_idx] = gyroData.data[2];
  gyroX_block[block_idx] = gyroData.gyro.x;
  gyroY_block[block_idx] = gyroData.gyro.y;
  gyroZ_block[block_idx] = gyroData.gyro.z;

  // Feed selected sensor data into ring buffer for display/analysis modes
  if (mode >= 0)
  {
    int current_sample = 0;
    // Use display_axis to determine which sensor to feed into ring buffer
    int sensor_select = (display_axis == 0) ? RINGBUFFER_VELOCX_RAW : RINGBUFFER_VELOCY_RAW;
    switch (sensor_select)
    {
    case RINGBUFFER_VELOCX_RAW:
      current_sample = adcValue0;
      buffer_feed(current_sample);
      break;
    case RINGBUFFER_VELOCY_RAW:
      current_sample = adcValue7;
      buffer_feed(current_sample);
      break;
    case RINGBUFFER_ACCELX_RAW:
      current_sample = accelData.data[0];
      buffer_feed(current_sample);
      break;
    case RINGBUFFER_ACCELY_RAW:
      current_sample = accelData.data[1];
      buffer_feed(current_sample);
      break;
    case RINGBUFFER_ACCELZ_RAW:
      current_sample = accelData.data[2];
      buffer_feed(current_sample);
      break;
    default:
      current_sample = adcValue0;
      buffer_feed(current_sample);
      break;
    }

    // Real-time crossing detection: check if signal crosses threshold
    bool prev_inside = (previous_sample >= -CROSSING_THRESHOLD) && (previous_sample <= CROSSING_THRESHOLD);
    bool curr_outside_threshold = (current_sample > CROSSING_THRESHOLD) || (current_sample < -CROSSING_THRESHOLD);
    bool curr_inside = !curr_outside_threshold;

    if (prev_inside && curr_outside_threshold)
    {
      // Signal just went outside threshold - start event collection
      crossing_time_out = millis();
      currently_outside = true;
      event_sample_count = 0;
      // Add first sample
      if (event_sample_count < MAX_EVENT_SAMPLES)
      {
        event_samples[event_sample_count++] = current_sample;
      }
    }
    else if (!prev_inside && curr_inside)
    {
      // Signal came back inside threshold - mark event complete
      crossing_time_in = millis();
      crossing_detected = true; // Flag that we have complete event data
      currently_outside = false;
    }
    else if (currently_outside)
    {
      // Collect samples while outside threshold
      if (event_sample_count < MAX_EVENT_SAMPLES)
      {
        event_samples[event_sample_count++] = current_sample;
      }
    }

    previous_sample = current_sample;
  }

  block_idx++;
  total_samples++;

  // Report buffer statistics periodically
  if (total_samples - last_report_sample >= REPORT_INTERVAL_SAMPLE_S)
  {
    static unsigned long last_report_time = 0;
    unsigned long current_time = millis();

    float samples_per_sec = 0;
    if (last_report_time > 0)
    {
      unsigned long time_diff = current_time - last_report_time;
      if (time_diff > 0)
      {
        samples_per_sec = (float)REPORT_INTERVAL_SAMPLE_S * 1000.0 / time_diff;
      }
    }

    last_report_sample = total_samples;
    last_report_time = current_time;
  }

  if (block_idx >= UDP_PACKET_SIZE)
  {
    if (udpStreamEnable[0])
    {
      Serial.printf("Sending UDP stream for velocXraw, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("velocX", velocXraw_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[1])
    {
      Serial.printf("Sending UDP stream for velocYraw, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("velocY", velocYraw_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[2])
    {
      Serial.printf("Sending UDP stream for accelX, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("accelX", accelX_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[3])
    {
      Serial.printf("Sending UDP stream for accelY, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("accelY", accelY_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[4])
    {
      Serial.printf("Sending UDP stream for accelZ, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("accelZ", accelZ_block, UDP_PACKET_SIZE);
    }
    block_idx = 0;
  }

#if defined(USE_WIZNET)
  if (netMode == NET_WIZNET)
  {
    EthernetClient client = server.accept();
    if (client)
    {
      while (client.connected() && !client.available())
        tight_loop_contents();
      String req = "";
      if (client.available())
      {
        req = client.readStringUntil('\r');
        // Consume the rest of the HTTP headers
        while (client.available())
        {
          String line = client.readStringUntil('\n');
          if (line.length() <= 1)
            break; // Empty line marks end of headers
          tight_loop_contents();
        }

        if (req.indexOf("GET /udpstream") == 0)
        {
          serveUDPStreamControl(client, req);
        }
        else if (req.indexOf("GET /setmode") == 0)
        {
          serveModeControl(client, req);
        }
        else if (req.indexOf("GET /setaxis") == 0)
        {
          serveAxisControl(client, req);
        }
        else if (req.indexOf("GET /touchcalibrate") == 0)
        {
          serveTouchCalibrate(client, req);
        }
        else if (req.indexOf("GET /touchcalibdata") == 0)
        {
          serveTouchCalibData(client, req);
        }
        else if (req.indexOf("GET /data") == 0)
        {
          serveSensorData(client);
        }
        else
        {
          serveChartPage(client);
        }
      }
      delay(1);
      client.stop();
    }
  }
#endif
#if defined(USE_WIFI)
  if (netMode == NET_WIFI)
  {
    WiFiClient client = server.accept();
    if (client)
    {
      while (client.connected() && !client.available())
        tight_loop_contents();
      String req = "";
      if (client.available())
      {
        req = client.readStringUntil('\r');
        // Consume the rest of the HTTP headers
        while (client.available())
        {
          String line = client.readStringUntil('\n');
          if (line.length() <= 1)
            break; // Empty line marks end of headers
          tight_loop_contents();
        }

        if (req.indexOf("GET /udpstream") == 0)
        {
          serveUDPStreamControl(client, req);
        }
        else if (req.indexOf("GET /setmode") == 0)
        {
          serveModeControl(client, req);
        }
        else if (req.indexOf("GET /setaxis") == 0)
        {
          serveAxisControl(client, req);
        }
        else if (req.indexOf("GET /touchcalibrate") == 0)
        {
          serveTouchCalibrate(client, req);
        }
        else if (req.indexOf("GET /touchcalibdata") == 0)
        {
          serveTouchCalibData(client);
        }
        else if (req.indexOf("GET /data") == 0)
        {
          serveSensorData(client);
        }
        else
        {
          serveChartPage(client);
        }
      }
      delay(1);
      client.stop();
    }
  }
#endif

  // Handle touch input for mode selection menu
  bool mode_changed = handleTouchInput(mode);
  if (mode_changed)
  {
    // Reset mode-specific flags when mode changes
    text_mode_initialized = false;
    waveform_initialized = false;
    mode23_screen_cleared = false;
  }

  // Track mode transitions for display control
  static int previous_mode = 3; // Initialize to default mode

  // Handle touch calibration trigger from web interface
  if (trigger_touch_calibration)
  {
    trigger_touch_calibration = false;
    debugPrintf("Starting touch calibration with %d points...\\n", NUM_CALIB_POINTS);
    calibrateTouchController();
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    // Reset display mode flags
    text_mode_initialized = false;
    waveform_initialized = false;
    mode23_screen_cleared = false;
  }

  // Entering mode -1: turn off display
  if (mode == -1 && previous_mode != -1)
  {
    debugPrintln("Switching to mode -1: turning off display");
    tft.fillScreen(ILI9341_BLACK);
    tft.scrollTo(0);
    setBacklight(0.0); // Turn off backlight
  }

  // Leaving mode -1: turn on display
  if (mode != -1 && previous_mode == -1)
  {
    debugPrintf("Leaving mode -1, switching to mode %d: turning on display\n", mode);
    setBacklight(1.0); // Use full brightness first to ensure GPIO is set
    delay(10);
    setBacklight(0.25); // Then set to desired brightness
  }

  previous_mode = mode;

  if (mode == -1)
  {
    // Mode -1: Display off mode - do nothing
  }
  else
  {
    static int last_trigger_sample = -10000; // Track when we last triggered

    // Check if crossing was detected in real-time during data acquisition
    if (mode >= 0 && crossing_detected)
    {
      crossing_detected = false; // Clear the flag

      // Calculate mean and RMS only on samples collected during the event
      if (event_sample_count > 0)
      {
        int n = event_sample_count;
        long sum = 0;
        long long sumsq = 0; // Use long long to prevent overflow
        for (int i = 0; i < n; ++i)
        {
          short sample = event_samples[i];
          sum += sample;
          sumsq += (long long)sample * sample;
        }

#ifdef LOG_THRESHOLD_EVENTS
        // Output statistics with timing information (in yellow)
        debugPrintfColor(ILI9341_YELLOW, "Event: out=%lums in=%lums\n", crossing_time_out, crossing_time_in);
        debugPrintfColor(ILI9341_YELLOW, "Duration=%lums samples=%d\n", crossing_time_in - crossing_time_out, n);
        debugPrintfColor(ILI9341_YELLOW, "Mean=%d RMS=%d\n", sum / n, (int)sqrtf(((float)sumsq) / n));
#endif
      }
    }

    if (mode == 0)
    {
      // Mode 0: Statistics text mode
      if (!text_mode_initialized)
      {
        tft.fillScreen(ILI9341_BLACK);
        tft.scrollTo(0); // Reset scroll position
        tft.setTextColor(ILI9341_GREEN);
        tft.setTextSize(TFT_TEXT_SIZE);
        tft.setCursor(0, 0);
        drawMenuIndicator(); // Draw menu access indicator
        text_mode_initialized = true;
      }

      // Display statistics periodically
      static unsigned long last_stats_update = 0;
      if (millis() - last_stats_update >= 1000)
      {
        char buf[128];

        // Get latest sensor data
        buffer_get_latest(input_shorts_buf, 100, 0);

        // Calculate statistics
        long sum = 0;
        long long sumsq = 0;
        short min_val = 32767;
        short max_val = -32768;
        for (int i = 0; i < 100; i++)
        {
          short val = input_shorts_buf[i];
          sum += val;
          sumsq += (long long)val * val;
          if (val < min_val)
            min_val = val;
          if (val > max_val)
            max_val = val;
        }
        int mean = sum / 100;
        int rms = (int)sqrtf(((float)sumsq) / 100);

        snprintf(buf, sizeof(buf), "Samples: %d\n", total_samples);
        debugPrint(buf);
        snprintf(buf, sizeof(buf), "Mean: %d\n", mean);
        debugPrint(buf);
        snprintf(buf, sizeof(buf), "RMS: %d\n", rms);
        debugPrint(buf);
        snprintf(buf, sizeof(buf), "Min: %d\n", min_val);
        debugPrint(buf);
        snprintf(buf, sizeof(buf), "Max: %d\n", max_val);
        debugPrint(buf);
        snprintf(buf, sizeof(buf), "Range: %d\n\n", max_val - min_val);
        debugPrint(buf);

        last_stats_update = millis();
      }
    }
    else if (mode == 1)
    {
      // Initialize waveform display on first entry to mode 1
      if (!waveform_initialized)
      {
        setup_waveform();
        tft.scrollTo(0);     // Reset scroll position
        drawMenuIndicator(); // Draw menu access indicator
        waveform_initialized = true;
      }

      // Throttle display updates to ~10 Hz to allow fast data acquisition
      static unsigned long last_display_update_mode1 = 0;
      if (millis() - last_display_update_mode1 >= 100)
      {
        // Get data for display
        buffer_get_latest(input_shorts_buf, waveform_w, 0);

        waveform_display(input_shorts_buf, waveform_w, ILI9341_YELLOW);

        last_display_update_mode1 = millis();
      }
    }
    else if (mode == 2 || mode == 3)
    {
      // Clear screen once when entering FFT mode and draw frequency axis
      if (!mode23_screen_cleared)
      {
        tft.fillScreen(ILI9341_BLACK);
        tft.scrollTo(0); // Reset scroll position when switching to mode 2 or 3
        disp_column = 0; // Reset spectrogram column position
        if (mode == 2)
        {
          draw_fft_frequency_axis(); // Draw frequency labels for FFT mode
        }
        drawMenuIndicator(); // Draw menu access indicator
        mode23_screen_cleared = true;
      }

      // Throttle display updates to ~10 Hz to allow fast data acquisition
      static unsigned long last_display_update_mode23 = 0;
      if (millis() - last_display_update_mode23 >= 100)
      {
        // Get data for display
        buffer_get_latest(input_shorts_buf, FFT_SIZE, /* lshift_bits */ 0);

        // Convert shorts to float with proper scaling for ADC values
        // ADC values are in range ~±2000, scale to make better use of float range
        for (int i = 0; i < FFT_SIZE; i++)
        {
          input_buf[i] = (float)input_shorts_buf[i];
        }

        int npts = magnitude_spectrum(input_buf, mag_spec, mag_spec_len);

        sample_to_short(mag_spec, mag_spec_short, npts);
        if (mode == 2)
        {
          waveform_display(mag_spec_short, npts, ILI9341_YELLOW, false); // Don't draw triggers in FFT mode
        }
        else
        {
          add_display_column(mag_spec_short, npts);
        }

        last_display_update_mode23 = millis();
      }
    }
  }
}