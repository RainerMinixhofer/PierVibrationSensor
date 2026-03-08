/**
 * @file main.cpp
 * @brief Main application for Pier Vibration Sensor: Reads ADS1256 and MPU-6500, serves web UI, and streams UDP sensor data.
 *
 * @note SPI Bus Configuration:
 * - All devices on SPI0: Display (ILI9341), Touchscreen (XPT2046) and WIZNET Ethernet (W5500/W5100S) share SPI0 with separate chip selects.
 * - SPI1: ADS1256 @ 1.4 MHz
 * - I2C0: ICM20948 9-DOF sensor @ 400 kHz
 * - GPIO: SD card uses software SPI on separate GPIO pins to avoid conflicts with SPI0 devices.
 * - Fixed clock speeds have proven to be most stable
 */

// Networking is now runtime-controlled via use_networking global variable
// WIZNET-first-then-WiFi-fallback logic implemented in initNetworking()
// #define WAIT_FOR_TOUCH // comment out to disable waiting for touch input
#define ENABLE_SD_CARD 1 // Set to 1 to enable SD card support (causes 10-30s delay if no card inserted)
// #define TESTING_WEBSERVER // Uncomment to use simple counter page webserver from test_wiznet_counter_button for testing

// Global networking control variable
bool use_networking = true; // Set to true to enable networking (WIZNET first, then WiFi fallback)

// --- Logging and Debug Output ---
enum LogLevel
{
  LOG_NONE = 0,
  LOG_ERROR = 1,
  LOG_WARN = 2,
  LOG_INFO = 3,
  LOG_DEBUG = 4
};
int logVerbosity = LOG_INFO; // Set global log verbosity here

#include <Arduino.h>
#include "pico/stdlib.h" // <-- Add this include for Pico SDK functions
// #include "hardware/spi.h"
// #include "hardware/i2c.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
// Rename lwIP Ethernet enum and its values to prevent conflict with Ethernet library
#define EthernetLinkStatus EthernetLinkStatus_LWIP
#define Unknown Unknown_LWIP
#define LinkON LinkON_LWIP
#define LinkOFF LinkOFF_LWIP
#define DHCP_TIMEOUT 10000 // 10 seconds for DHCP
#define LINK_TIMEOUT 5000  // 5 seconds for link detection
#include <WiFi.h>
#undef EthernetLinkStatus
#undef Unknown
#undef LinkON
#undef LinkOFF
#ifdef UDP_TX_PACKET_MAX_SIZE
#undef UDP_TX_PACKET_MAX_SIZE
#endif
// Now include the Ethernet library which defines its own EthernetLinkStatus and enum values
#include <Ethernet_Generic.h>
#include <hardware/flash.h>
// #include <hardware/sync.h>
#include <hardware/pwm.h> // Add this include for PWM functions
#include <ADS1256.h>      // Make sure to include the ADS1256 library
#include <Wire.h>
#include <FS.h>       // Already present
#include <LittleFS.h> // Use LittleFS for file system operations
#include "arm_math.h"
#include <TFT_eSPI.h> // TFT_eSPI supports both display and touch
#include "sd_spi.h"   // Low-level SD card SPI interface
// #include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include "version.h"
#include "pin_config.h"      // GPIO pin assignments
#include "miniseed_logger.h" // miniSEED data logging for seismic data compatibility

#define PERFORM_CALIBRATION // Comment to disable startup calibration

// --- Constants and Configuration ---

// Constants for ADS1256
#define ADS1256_DRATE DRATE_2000SPS // 2000 SPS to get approximately 500 SPS overall
#define ADS1256_CLOCKMHZ 7.68
#define ADS1256_VREF 2.5

// Create ADS1256 object using SPI1
ADS1256 ads(ADS1256_DRDY, ADS1256_RST, ADS1256_SYNC, ADS1256_CS, 2.500, &ADS1256_SPI_PORT); // DRDY, RESET, SYNC(PDWN), CS, VREF(float), SPI bus.

// Create ICM20948 object using I2C0
Adafruit_ICM20948 icm;

// Ring buffer sensor selection constants
#define RINGBUFFER_VELOCX_RAW 0
#define RINGBUFFER_VELOCY_RAW 1
#define RINGBUFFER_VELOCR_RAW 2
#define RINGBUFFER_ACCELX_RAW 3
#define RINGBUFFER_ACCELY_RAW 4
#define RINGBUFFER_ACCELZ_RAW 5

// Constants for TFT display with TFT_eSPI
#define TFT_TEXT_SIZE 1             // Set the default text size for the display
#define TFT_DISPLAY_REFRESH_RATE 10 // Set a refresh rate of 20 FPS for the display modes 1,2 and 3
#define TEXT_DISPLAY_REFRESH_RATE 1 // Set a refresh rate of 1 FPS for the text display in mode 0

// Create TFT_eSPI object (handles both display and touch via eSPI_Setup.h)
TFT_eSPI tft = TFT_eSPI();

// ILI9341 hardware scrolling command constants
#define ILI9341_VSCRDEF 0x33  // Vertical Scroll Definition
#define ILI9341_VSCRSADD 0x37 // Vertical Scrolling Start Address

// Hardware scrolling configuration (landscape mode 320x240)
#define SCROLL_TOP_FIXED_AREA 0    // Lines fixed at top
#define SCROLL_BOTTOM_FIXED_AREA 0 // Lines fixed at bottom
static uint16_t scroll_x_start = SCROLL_TOP_FIXED_AREA;

// Custom color constants (RGB565 format)
#define COLOR_GREY 0x7BEF // Medium grey for UI elements

// Touch controller calibration settings
// Set FORCETOUCHCALIBRATION to true to force touch calibration at every startup.
// You can get the resulting calibration parameters from the webpage with 'Show Calibration Data' button.
#define FORCETOUCHCALIBRATION false // Set to true to force touch calibration at every startup

// Touch calibration threshold for TFT_eSPI getTouch()
#define TOUCH_THRESHOLD 600 // Pressure threshold for valid touch

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
#define NTP_TIMEOUT_MS 2000

#define HTTP_SERVER_PORT 3001
#define UDP_STREAM_PORT 10000
#define UDP_PACKET_SIZE 32 // Set your desired block size

// --- Fallback IP when DHCP is not working ---

IPAddress ip(10, 0, 3, 177);

// DHCP-assigned network parameters
IPAddress assignedIP;
IPAddress assignedGateway;
IPAddress assignedSubnet;
IPAddress assignedDNS;

// Networking servers and UDP streams
EthernetServer ethServer(HTTP_SERVER_PORT);
WiFiServer wifiServer(HTTP_SERVER_PORT);
EthernetUDP ethUdpStream;
WiFiUDP wifiUdpStream;

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
volatile NetworkMode netMode = NET_NONE;

// Global flag to prevent duplicate server info printing across resets
static bool serverInfoPrinted = false;

/* DSP/FFT Implementation:
 * - The code supports both fixed-point and floating-point DSP pipelines for FFT and signal processing.
 * - To enable fixed-point DSP (using CMSIS-DSP q15_t types), define USE_FIXEDPOINT.
 * - By default, floating-point DSP is used for higher accuracy and easier debugging.
 * - Fixed-point DSP is more efficient on MCUs without FPU but requires careful scaling and management of numerical ranges.
 */

// #define USE_FIXEDPOINT // Uncomment to use fixed-point DSP (q15_t), comment out to use floating-point DSP (float)

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
static volatile bool forceTouchCalibration = FORCETOUCHCALIBRATION; // Set to true if force touch calibration should be enforced at startup ignoring any calibration file data
static volatile bool touch_event_flag = false;
static volatile int irq_save = 0;             // Save the IRQ state for touch events
static volatile uint32_t touch_irq_count = 0; // Debug: number of touch IRQ triggers
// Note: TFT_eSPI uses getTouch(&x, &y) instead of TS_Point structure

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

// --- miniSEED Recording Control (Raspberry Shake compatible data format) ---
MinISeedLogger miniSeedLogger;                  ///< miniSEED logger for Raspberry Shake compatible data storage
fs_info_t sd_fs_info;                           ///< Global filesystem info for miniSEED file I/O
volatile bool miniSeedRecordingEnabled = false; ///< Global flag to control miniSEED recording
volatile bool sd_card_ready = false;            ///< Global flag indicating SD card is mounted and ready

static void stopMiniSeedRecording(const char *reason)
{
  if (reason)
  {
    Serial.print("miniSEED stop requested: ");
    Serial.println(reason);
  }

  // Disable first to prevent new writes, then close buffered data immediately.
  miniSeedRecordingEnabled = false;
  miniSeedLogger.setRecordingEnabled(false);
  miniSeedLogger.closeFile();
}

// --- Display Mode and State ---
volatile int mode = 3;         // -1 = off 0 = text 1 = waveform, 2 = fft, 3 = sgram
volatile int display_axis = 0; // 0 = X axis, 1 = Y axis, 2 = radial velocity
// --- Radial velocity calculation ---
float velocR = 0.0; // Latest radial velocity value in m/s
bool mode_initialized = false;

// --- Highcharts Status Tracking ---
volatile bool highchartsWorking = false; // Track if Highcharts loaded successfully
unsigned long highchartsLastReport = 0;  // Timestamp of last Highcharts status report

// --- Menu State ---
const int NUM_MENU_BUTTONS = 5;
bool menu_visible = false;
int saved_scroll_offset = 0;     // Remember current scroll offset while menu is shown
bool text_block_refresh = false; // Request to clear and overwrite text output block

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
enum CrossingDirection
{
  CROSSING_POSITIVE = 0, // Positive-going crossing only
  CROSSING_NEGATIVE = 1, // Negative-going crossing only
  CROSSING_BOTH = 2      // Either direction
};

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

// --- Menu Button Definition ---
struct MenuButton
{
  int x;
  int y;
  int w;
  int h;
  const char *label;
  int mode_value;
  uint16_t color;
};

/* Define menu buttons for mode selection
 *
 * Each button has position (x, y), size (w, h), label text, associated mode value,
 * and a color for visual distinction.
 */
MenuButton menu_buttons[NUM_MENU_BUTTONS] = {
    {10, 30, 60, 32, "OFF", -1, TFT_RED},
    {10, 66, 60, 32, "TEXT", 0, TFT_BLUE},
    {10, 102, 60, 32, "WAVE", 1, TFT_GREEN},
    {10, 138, 60, 32, "FFT", 2, TFT_CYAN},
    {10, 174, 60, 32, "SGRAM", 3, TFT_MAGENTA}};

unsigned long menu_show_time = 0;
const unsigned long MENU_TIMEOUT_MS = 15000; // Menu auto-hides after 15 seconds

// ============================================================================
// FORWARD DECLARATIONS
// ============================================================================

void drawModeMenu();
bool fsCreateDirectory(const char *path);
bool fsDirectoryExists(const char *path);

// ============================================================================
// INTERRUPT HANDLERS
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

// Interrupt handler for TFT touch controller (XPT2046)
void touch_irq_isr(uint gpio, uint32_t events)
{
  (void)gpio;
  (void)events;
  // Set a volatile flag for main loop processing
  gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false); // disable IRQ to prevent triggering during processing
  // For hardware SPI, just set the flag - don't read in IRQ context
  // Reading will be done in main loop after flag is detected
  touch_event_flag = true;
  touch_irq_count++;
}

#ifdef DEBUG_BUILD
inline void debugSerialPrint(const char *msg) { Serial.print(msg); }
inline void debugSerialPrint(int val) { Serial.print(val); }
inline void debugSerialPrint(unsigned int val) { Serial.print(val); }
inline void debugSerialPrint(long val) { Serial.print(val); }
inline void debugSerialPrint(unsigned long val) { Serial.print(val); }
inline void debugSerialPrint(float val) { Serial.print(val); }
inline void debugSerialPrint(double val) { Serial.print(val); }
inline void debugSerialPrint(char val) { Serial.print(val); }
inline void debugSerialPrintln() { Serial.println(); }
inline void debugSerialPrintln(const char *msg) { Serial.println(msg); }
inline void debugSerialPrintln(int val) { Serial.println(val); }
inline void debugSerialPrintln(unsigned int val) { Serial.println(val); }
inline void debugSerialPrintln(long val) { Serial.println(val); }
inline void debugSerialPrintln(unsigned long val) { Serial.println(val); }
inline void debugSerialPrintln(float val) { Serial.println(val); }
inline void debugSerialPrintln(double val) { Serial.println(val); }
inline void debugSerialPrintln(char val) { Serial.println(val); }
inline void debugSerialPrintf(const char *fmt, ...)
{
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  Serial.print(buffer);
}
#else
inline void debugSerialPrint(const char *) {}
inline void debugSerialPrint(int) {}
inline void debugSerialPrint(unsigned int) {}
inline void debugSerialPrint(long) {}
inline void debugSerialPrint(unsigned long) {}
inline void debugSerialPrint(float) {}
inline void debugSerialPrint(double) {}
inline void debugSerialPrint(char) {}
inline void debugSerialPrintln() {}
inline void debugSerialPrintln(const char *) {}
inline void debugSerialPrintln(int) {}
inline void debugSerialPrintln(unsigned int) {}
inline void debugSerialPrintln(long) {}
inline void debugSerialPrintln(unsigned long) {}
inline void debugSerialPrintln(float) {}
inline void debugSerialPrintln(double) {}
inline void debugSerialPrintln(char) {}
inline void debugSerialPrintf(const char *, ...) {}
#endif

// ============================================================================
// GPIO INITIALIZATION
// ============================================================================

/**
 * @brief Initialize all GPIO pins according to pin_config.h
 * @details Sets pin direction and default values for all peripherals:
 *          - TFT Display (ILI9341)
 *          - Touchscreen (XPT2046)
 *          - Wiznet Ethernet (W5500/W5100S)
 *          - SD Card
 *          - ADS1256 ADC
 *          - ICM20948 9-DOF sensor
 *
 * @note Chip select pins are set HIGH (inactive) by default
 * @note I2C pins (SDA/SCL) are left for Wire library to initialize
 * @note SPI pins (MOSI/MISO/SCLK) are initialized but may be reconfigured by SPI library
 */
void initAllGPIOs()
{
  static bool initialized = false;
  if (initialized)
  {
    debugSerialPrintln("  GPIOs already initialized, skipping...");
    return;
  }
  debugSerialPrintln("Initializing all GPIOs...");
  initialized = true;

  // -------------------------------------------------------------------------
  // TFT Display (ILI9341) - SPI0
  // -------------------------------------------------------------------------
  // SPI pins (will be properly configured by SPI library, but set safe defaults)
  pinMode(TFT_MISO, INPUT);  // GP0 - SPI MISO (input)
  pinMode(TFT_MOSI, OUTPUT); // GP3 - SPI MOSI (output)
  pinMode(TFT_SCLK, OUTPUT); // GP2 - SPI Clock (output)
  digitalWrite(TFT_MOSI, LOW);
  digitalWrite(TFT_SCLK, LOW);

  // Control pins
  pinMode(TFT_CS, OUTPUT);    // GP1 - Chip Select
  digitalWrite(TFT_CS, HIGH); // Deselect TFT (active low)

  pinMode(TFT_DC, OUTPUT);    // GP4 - Data/Command
  digitalWrite(TFT_DC, HIGH); // Default to data mode

  pinMode(TFT_RST, OUTPUT);    // GP6 - Reset
  digitalWrite(TFT_RST, HIGH); // Inactive (active low)

  pinMode(TFT_BL, OUTPUT);   // GP7 - Backlight
  digitalWrite(TFT_BL, LOW); // Start with backlight off (will be enabled later)

  debugSerialPrintln("  TFT Display pins initialized");

  // -------------------------------------------------------------------------
  // Touchscreen (XPT2046) - SPI0 (shared with TFT)
  // -------------------------------------------------------------------------
  pinMode(TOUCH_CS, OUTPUT);    // GP26 - Chip Select
  digitalWrite(TOUCH_CS, HIGH); // Deselect touch controller (active low)

  gpio_init(TOUCH_IRQ); // Ensure proper initialization
  gpio_set_dir(TOUCH_IRQ, GPIO_IN);
  gpio_pull_up(TOUCH_IRQ);

  debugSerialPrintln("  Touchscreen pins initialized");

  // -------------------------------------------------------------------------
  // Wiznet Ethernet (W5500/W5100S) - SPI0 (shared with TFT)
  // -------------------------------------------------------------------------
  // SPI pins shared with TFT (already initialized above)

  pinMode(WIZNET_SPI_CS, OUTPUT);    // GP17 - Chip Select
  digitalWrite(WIZNET_SPI_CS, HIGH); // Deselect Wiznet (active low)

  pinMode(WIZNET_RST, OUTPUT);    // GP20 - Hardware Reset
  digitalWrite(WIZNET_RST, HIGH); // Inactive (active low)

  pinMode(WIZNET_INT, INPUT); // GP21 - Interrupt (active low)
  gpio_pull_up(WIZNET_INT);

  debugSerialPrintln("  Wiznet Ethernet pins initialized");

  // -------------------------------------------------------------------------
  // SD Card - Software SPI
  // -------------------------------------------------------------------------
  pinMode(SD_CLK, OUTPUT); // GP18 - SPI Clock
  digitalWrite(SD_CLK, LOW);

  pinMode(SD_MOSI, OUTPUT); // GP22 - SPI MOSI
  digitalWrite(SD_MOSI, LOW);

  pinMode(SD_MISO, INPUT); // GP19 - SPI MISO

  pinMode(SD_CS, OUTPUT);    // GP16 - Chip Select
  digitalWrite(SD_CS, HIGH); // Deselect SD card (active low)

  debugSerialPrintln("  SD Card pins initialized");

  // -------------------------------------------------------------------------
  // ADS1256 ADC - SPI1
  // -------------------------------------------------------------------------
  pinMode(ADS1256_SCLK, OUTPUT); // GP10 - SPI Clock
  digitalWrite(ADS1256_SCLK, LOW);

  pinMode(ADS1256_MOSI, OUTPUT); // GP11 - SPI MOSI
  digitalWrite(ADS1256_MOSI, LOW);

  pinMode(ADS1256_MISO, INPUT); // GP12 - SPI MISO

  pinMode(ADS1256_CS, OUTPUT);    // GP13 - Chip Select
  digitalWrite(ADS1256_CS, HIGH); // Deselect ADS1256 (active low)

  pinMode(ADS1256_SYNC, OUTPUT);    // GP14 - Sync/Power Down
  digitalWrite(ADS1256_SYNC, HIGH); // Normal operation (active low for power down)

  pinMode(ADS1256_DRDY, INPUT); // GP15 - Data Ready (active low)

  debugSerialPrintln("  ADS1256 ADC pins initialized");

  // -------------------------------------------------------------------------
  // ICM20948 9-DOF Sensor - I2C0
  // -------------------------------------------------------------------------
  // I2C pins (SDA/SCL) will be initialized by Wire library
  // GP8 - ICM20948_I2C_SDA (I/O)
  // GP9 - ICM20948_I2C_SCL (Output with pull-up)

  pinMode(ICM20948_INT, INPUT); // GP5 - Interrupt Output

  debugSerialPrintln("  ICM20948 sensor pins initialized");

  // -------------------------------------------------------------------------
  // Summary
  // -------------------------------------------------------------------------
  debugSerialPrintln("All GPIOs initialized successfully");
  debugSerialPrintln("  Output pins: All set to default safe states");
  debugSerialPrintln("  Input pins: Configured for reading");
  debugSerialPrintln("  Chip selects: All set HIGH (inactive)");
  debugSerialPrintln("  I2C pins: Will be configured by Wire library");
}

// ============================================================================
// SPI HELPER FUNCTIONS
// ============================================================================

/**
 * @brief Release SPI bus
 * Sets all SPI device CS pins high to release the bus and prevent conflicts
 * Call this before switching operation between shared SPI devices
 */
void releaseSPIBus()
{
  // Release TFT from SPI bus
#if defined(TFT_CS) && TFT_CS != -1
  digitalWrite(TFT_CS, HIGH);
#endif

  // Release touch controller from SPI bus
#if defined(TOUCH_CS) && TOUCH_CS != -1
  digitalWrite(TOUCH_CS, HIGH);
#endif

  // Wiznet CS is managed by Ethernet library, but we can ensure it's available
#if defined(WIZNET_SPI_CS) && WIZNET_SPI_CS != -1
  digitalWrite(WIZNET_SPI_CS, HIGH);
#endif
}

/**
 * @brief Get SPI baudrate for TFT display (SPI0)
 * @param spi SPI instance (spi0 for TFT)
 * @return Actual baudrate in Hz
 */
static uint32_t get_tft_spi_baudrate(spi_inst_t *spi)
{
  spi_hw_t *hw = spi_get_hw(spi);
  uint32_t cpsr = hw->cpsr & 0xFFu;
  uint32_t scr = (hw->cr0 >> 8) & 0xFFu; // CR0 bits 15:8 contain SCR
  uint32_t clk = clock_get_hz(clk_peri);
  if (cpsr == 0)
    return 0;
  return clk / (cpsr * (scr + 1u));
}

/**
 * @brief Log TFT SPI speed with tag for debugging
 * @param tag Description tag for the log message
 */
static void log_tft_spi_speed(const char *tag)
{
  spi_inst_t *spi = TFT_SPI_PORT;
  spi_hw_t *hw = spi_get_hw(spi);
  uint32_t cpsr = hw->cpsr & 0xFFu;
  uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
  uint32_t baud = get_tft_spi_baudrate(spi);

  debugSerialPrintf("%s SPI baud: %.2f MHz (cpsr=%lu, scr=%lu)\n", tag, baud / 1000000.0, cpsr, scr);
}

/**
 * @brief Get SPI baudrate for Wiznet adapter
 * @param spi SPI instance
 * @return Actual baudrate in Hz
 */
static uint32_t get_wiznet_spi_baudrate(spi_inst_t *spi)
{
  spi_hw_t *hw = spi_get_hw(spi);
  uint32_t cpsr = hw->cpsr & 0xFFu;
  uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
  uint32_t clk = clock_get_hz(clk_peri);
  if (cpsr == 0)
    return 0;
  return clk / (cpsr * (scr + 1u));
}

/**
 * @brief Log Wiznet SPI speed with tag for debugging
 * @param tag Description tag for the log message
 */
static void log_wiznet_spi_speed(const char *tag)
{
  spi_hw_t *hw = spi_get_hw(WIZNET_SPI_PORT);
  uint32_t cpsr = hw->cpsr & 0xFFu;
  uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
  uint32_t baud = get_wiznet_spi_baudrate(WIZNET_SPI_PORT);

  debugSerialPrintf("%s SPI baud: %lu Hz (cpsr=%lu, scr=%lu)\n", tag, baud, cpsr, scr);
}

// ============================================================================
// SD CARD FUNCTIONS
// ============================================================================

void initSDCard()
{
  Serial.println("\n=== SD Card Initialization ===");
  debugSerialPrintln("Initializing SD card...");

  // Initialize LittleFS
  if (!LittleFS.begin())
  {
    debugSerialPrintln("Failed to mount LittleFS!");
    return;
  }
  debugSerialPrintln("LittleFS mounted successfully.");

  // Initialize SD card via low-level SPI driver (proven working method from test suite)
  Serial.println("Attempting SD card initialization...");
  Serial.println("NOTE: This may take 10-30 seconds if no SD card is inserted.");
  Serial.println("Please wait - system will continue automatically after timeout.");
  Serial.flush();

  unsigned long sd_start = millis();
  bool sd_card_available = sd_spi_init();
  unsigned long init_time = millis() - sd_start;

  if (!sd_card_available)
  {
    debugSerialPrintf("SD card initialization failed after %lu ms (no card inserted or card error)\n", init_time);
    debugSerialPrintln("Continuing without SD card support...");
    Serial.flush();
    return;
  }

  debugSerialPrintf("SD card detected in %lu ms\n", init_time);

  // Get card status
  uint8_t status = sd_spi_get_status();
  Serial.print("Card Status: 0x");
  Serial.println(status, HEX);

  // Get and display card type
  const char *card_type_str = sd_spi_get_card_type_string();
  Serial.print("Card Type: ");
  Serial.println(card_type_str);

  // Read OCR for card capacity information
  uint32_t ocr = 0;
  if (sd_spi_read_ocr(&ocr))
  {
    Serial.println("\nOCR (Operating Conditions Register):");
    Serial.print("  Power Up Status: ");
    Serial.println((ocr & 0x80000000) ? "Complete" : "Busy");

    // Determine card capacity type
    Serial.print("  Card Capacity Status: ");
    if (ocr & 0x40000000)
    {
      uint64_t capacity = sd_spi_get_capacity();
      if (capacity > (32LL * 1024 * 1024 * 1024))
      {
        Serial.println("SDXC (>32GB)");
      }
      else
      {
        Serial.println("SDHC (2GB-32GB)");
      }
    }
    else
    {
      Serial.println("SDSC (<2GB)");
    }
  }

  // Identify filesystem
  fs_info_t fs_info;
  bool fs_identified = sd_spi_identify_filesystem(&fs_info);

  if (!fs_identified)
  {
    Serial.println("Failed to identify filesystem!");
    return;
  }

  // Display filesystem information
  const char *fs_type = sd_spi_get_fs_type_string(fs_info.type);
  Serial.print("\nFile System Type: ");
  Serial.println(fs_type);

  Serial.println("Filesystem Details:");
  Serial.print("  Bytes per Sector: ");
  Serial.println(fs_info.bytes_per_sector);
  Serial.print("  Sectors per Cluster: ");
  Serial.println(fs_info.sectors_per_cluster);
  Serial.print("  FAT Count: ");
  Serial.println(fs_info.fat_count);
  Serial.print("  Cluster Count: ");
  Serial.println(fs_info.cluster_count);

  // Calculate and display filesystem size
  uint64_t fs_size = (uint64_t)fs_info.total_sectors * fs_info.bytes_per_sector;
  Serial.print("  Total Size: ");
  if (fs_size >= (1024LL * 1024 * 1024))
  {
    Serial.print(fs_size / (1024LL * 1024 * 1024));
    Serial.println(" GB");
  }
  else if (fs_size >= (1024 * 1024))
  {
    Serial.print(fs_size / (1024 * 1024));
    Serial.println(" MB");
  }
  else
  {
    Serial.print(fs_size / 1024);
    Serial.println(" KB");
  }

  // Read and display root directory
  Serial.println("\nReading Root Directory:");

  dirent_t directory[32];
  int entry_count = sd_spi_read_directory(&fs_info, directory, 32);

  if (entry_count < 0)
  {
    Serial.println("Failed to read directory!");
    return;
  }

  Serial.print("Found ");
  Serial.print(entry_count);
  Serial.println(" entries:");

  if (entry_count == 0)
  {
    Serial.println("  (empty directory)");
  }
  else
  {
    // Display directory entries
    for (int i = 0; i < entry_count && i < 32; i++)
    {
      Serial.print("  ");
      Serial.print(i + 1);
      Serial.print(". ");
      Serial.print(directory[i].filename);

      // Pad filename for alignment
      int name_len = strlen(directory[i].filename);
      if (name_len < 35)
      {
        for (int j = name_len; j < 35; j++)
          Serial.print(" ");
      }
      else
      {
        Serial.println();
        Serial.print("     ");
      }

      // Display attributes and size
      Serial.print(" [");
      if (directory[i].attributes & 0x10)
        Serial.print("DIR");
      else
      {
        Serial.print("FILE ");
        Serial.print(directory[i].file_size);
        Serial.print("B");
      }
      Serial.println("]");
    }
  }

  debugSerialPrintln("SD card initialized successfully.");

  // Save filesystem info to global for miniSEED logger to use
  sd_fs_info = fs_info;
  sd_card_ready = true;

  // Create directory structure for miniSEED data logging
  Serial.println("\nCreating directory structure for miniSEED logging...");

  // Create /data directory
  if (sd_spi_directory_exists(&fs_info, "/data"))
  {
    Serial.println("  /data directory already exists");
  }
  else
  {
    if (sd_spi_create_directory(&fs_info, "/data"))
    {
      Serial.println("  Created /data directory");
    }
    else
    {
      Serial.println("  WARNING: Failed to create /data directory");
    }
  }

  // Create /data/miniseed directory
  if (sd_spi_directory_exists(&fs_info, "/data/miniseed"))
  {
    Serial.println("  /data/miniseed directory already exists");
  }
  else
  {
    if (sd_spi_create_directory(&fs_info, "/data/miniseed"))
    {
      Serial.println("  Created /data/miniseed directory");
    }
    else
    {
      Serial.println("  WARNING: Failed to create /data/miniseed directory");
    }
  }

  // Initialize miniSEED logger with hierarchical directory support
  miniSeedLogger.init(&sd_fs_info, "PIER1", "XX", "/data/miniseed");

  Serial.println("miniSEED recording will write files to /data/miniseed directory");
  Serial.println("=== SD Card and Filesystem Ready ===");
}

void disableSDCard()
{
  debugSerialPrintln("Disabling SD card...");
  // Set CS of SD card high to disable it
  if (SD_CS != -1)
  {
    digitalWrite(SD_CS, HIGH); // Disable SD card
  }
}

void disableTouch()
{
  if (TOUCH_CS != -1)
  {
    digitalWrite(TOUCH_CS, HIGH); // Deselect touch
  }
}

// ============================================================================
// TFT DISPLAY FUNCTIONS
// ============================================================================

/**
 * @brief Enable TFT backlight (from test_all)
 */
static void enableBacklight()
{
  if (TFT_BL != -1)
  {
    digitalWrite(TFT_BL, HIGH);
  }
}

/**
 * @brief Disable TFT backlight (from test_all)
 */
static void disableBacklight()
{
  if (TFT_BL != -1)
  {
    digitalWrite(TFT_BL, LOW); // Turn off backlight
  }
}

void setBacklight(float brightness)
{
  if (TFT_BL != -1)
  {
    if (brightness == 0.0f || brightness == 1.0f)
    {
      digitalWrite(TFT_BL, (int)brightness); // Directly set pin state for full off/on
    }
    else
    {
      // brightness: 0.0 (off) to 1.0 (full on)
      static bool initialized = false;
      if (!initialized)
      {
        gpio_set_function(TFT_BL, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(TFT_BL);
        pwm_set_wrap(slice_num, 255); // 8-bit resolution
        pwm_set_enabled(slice_num, true);
        initialized = true;
      }
      uint slice_num = pwm_gpio_to_slice_num(TFT_BL);
      uint level = (uint)(brightness * 255.0f);
      if (level > 255)
        level = 255;
      pwm_set_gpio_level(TFT_BL, level);
    }
  }
}

/**
 * @brief Draws a menu indicator ("hamburger" icon) on the TFT display
 *
 * This function renders a small menu indicator icon on the top left corner of the TFT display.
 * It clears the background area to prevent artifacts from scrolling,
 * then draws a three-line "hamburger" menu icon with a border.
 *
 * @param scroll_offset The current scroll offset, used to position the menu indicator appropriately based on rotation.
 */
void drawMenuIndicator(int scroll_offset)
{

  int x_base = (2 + scroll_offset) % 320; // Adjust x position based on scroll offset for landscape modes
  int y_base = 0;

  int w = tft.width();
  int h = tft.height();

  // Clear background with a slightly larger area to remove old border pixels from scrolling
  tft.fillRect(x_base - 1, y_base, 18, 16, TFT_BLACK);

  // Draw a small "hamburger" menu icon (3 horizontal lines)
  tft.fillRect(x_base + 3, y_base + 2, 10, 2, TFT_WHITE);
  tft.fillRect(x_base + 3, y_base + 7, 10, 2, TFT_WHITE);
  tft.fillRect(x_base + 3, y_base + 12, 10, 2, TFT_WHITE);
  // Add a small border
  tft.drawRect(x_base, y_base, 16, 16, COLOR_GREY);
}

/**
 * @brief Draws a menu indicator ("hamburger" icon) on the TFT display, adjusting for screen rotation and scroll offset.
 *
 * This function renders a small menu indicator icon at a position determined by the current scroll offset and
 * the TFT display's rotation setting. It clears the background area to prevent artifacts from scrolling,
 * then draws a three-line "hamburger" menu icon with a border.
 *
 * @param scroll_offset The current scroll offset, used to position the menu indicator appropriately based on rotation.
 */
void drawMenuIndicatorOld(int scroll_offset)
{
  // Account for different rotation settings
  // Rotation 0: Portrait (240w x 320h) - scrollTo scrolls vertically
  // Rotation 1: Landscape (320w x 240h) - scrollTo scrolls horizontally
  // Rotation 2: Portrait inverted (240w x 320h) - scrollTo scrolls vertically
  // Rotation 3: Landscape inverted (320w x 240h) - scrollTo scrolls horizontally

  int x_base = 2;
  int y_base = 0;

  // Use current scroll offset for positioning; when menu is visible use the saved offset so icon stays at physical top-left
  int effective_offset = menu_visible ? saved_scroll_offset : scroll_offset;
  int w = tft.width();
  int h = tft.height();

  switch (TFT_ROTATION)
  {
  case 0: // Portrait - scroll affects y
    y_base = effective_offset % h;
    break;
  case 1: // Landscape - scroll affects x
    x_base = effective_offset % w + 2;
    break;
  case 2: // Portrait inverted - scroll affects y
    y_base = effective_offset % h;
    break;
  case 3: // Landscape inverted - scroll affects x
    x_base = effective_offset % w + 2;
    break;
  }

  // Clear background with a slightly larger area to remove old border pixels from scrolling
  tft.fillRect(x_base - 1, y_base, 18, 16, TFT_BLACK);

  // Draw a small "hamburger" menu icon (3 horizontal lines)
  tft.fillRect(x_base + 3, y_base + 2, 10, 2, TFT_WHITE);
  tft.fillRect(x_base + 3, y_base + 7, 10, 2, TFT_WHITE);
  tft.fillRect(x_base + 3, y_base + 12, 10, 2, TFT_WHITE);
  // Add a small border
  tft.drawRect(x_base, y_base, 16, 16, COLOR_GREY);
}

void tftPrint(const char *msg, bool resetline = false)
{
  // Print debug messages to Serial
  debugSerialPrint(msg);

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
      tft.fillRect(0, 0, tft.width(), disp_height, TFT_BLACK);
      tft_line = 0;
      drawMenuIndicator(disp_column); // Redraw menu indicator after clearing screen
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

// ============================================================================
// Hardware Scrolling Functions (ILI9341)
// ============================================================================

/**
 * @brief Set hardware scroll address (vertical scrolling start position)
 * @param VSP Vertical Scrolling Start Address (0 to scroll_height-1)
 */
void scrollAddress(uint16_t VSP)
{
  tft.writecommand(ILI9341_VSCRSADD); // Vertical scrolling start address
  tft.writedata(VSP >> 8);
  tft.writedata(VSP & 0xFF);
}

/**
 * @brief Setup hardware scroll area for ILI9341 display
 * @param TFA Top Fixed Area (lines at top that don't scroll)
 * @param BFA Bottom Fixed Area (lines at bottom that don't scroll)
 * @note Hardware scrolling is only supported in portrait mode thus we ignore TFT_ROTATION for determining scroll height.
 * The scroll height is always the long side of the display (320 lines) regardless of rotation.
 * The TFT_eSPI library does not provide built-in support for hardware scrolling, so we send the ILI9341 commands directly to configure the scroll area.
 */
void setupScrollArea(uint16_t TFA, uint16_t BFA)
{
  uint16_t scroll_height = 320; // ILI9341 scroll height is always 320 lines in portrait mode regardless of rotation

  tft.writecommand(ILI9341_VSCRDEF); // Vertical scroll definition
  tft.writedata(TFA >> 8);
  tft.writedata(TFA & 0xFF);
  tft.writedata((scroll_height - TFA - BFA) >> 8);
  tft.writedata((scroll_height - TFA - BFA) & 0xFF);
  tft.writedata(BFA >> 8);
  tft.writedata(BFA & 0xFF);
  scrollAddress(TFA);
}

/**
 * @brief Scroll display by incrementing scroll position
 * @param offset Scroll offset (pixels to scroll)
 */
void scrollDisplay(int offset)
{
  if (offset == 0)
  {
    scroll_x_start = 0;
    scrollAddress(scroll_x_start);
    return;
  }
  uint16_t scroll_height = 320; // ILI9341 scroll height is always 320 lines in portrait mode regardless of rotation

  scroll_x_start = (scroll_x_start + offset) % scroll_height;
  if (scroll_x_start < SCROLL_TOP_FIXED_AREA)
    scroll_x_start = SCROLL_TOP_FIXED_AREA;

  scrollAddress(scroll_x_start);
}

// ============================================================================
// TFT Display Initialization
// ============================================================================

static void initTFTDisplay(uint8_t rotation)
{
  static bool tft_initialized = false;
  debugSerialPrintln("TFT display driver initialization");

  // Deselect other SPI0 devices before TFT init
  // and turn off backlight before initialization

  releaseSPIBus();
  disableBacklight();
  if (!tft_initialized)
  {
    tft.init();
    log_tft_spi_speed("After TFT begin");
    tft_initialized = true;
  }
  // Read display status
  // TFT_eSPI provides basic diagnostics via getSetup()
  debugSerialPrintln("TFT_eSPI setup information:");
  debugSerialPrintf("  Width: %d, Height: %d\n", tft.width(), tft.height());
  debugSerialPrintf("  Rotation: %d\n", tft.getRotation());
  // Configure display settings
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(rotation);
  tft.setTextSize(TFT_TEXT_SIZE);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  enableBacklight();

  delay(100); // Brief stabilization delay

  // Setup hardware scrolling area (uses ILI9341 commands directly)
  setupScrollArea(SCROLL_TOP_FIXED_AREA, SCROLL_BOTTOM_FIXED_AREA);
  debugSerialPrintln("Hardware scrolling configured");

  tftPrint("TFT display driver initialized...\n");
  tftPrint("---------------------------------------------\n");
}

// ============================================================================
// Logging and Debug Output Functions
// ============================================================================

/*
Handler for setting log level via HTTP
*/
void serveSetLogLevel(Stream &client, const String &req)
{
  int levelIdx = req.indexOf("level=");
  int newLevel = -1;
  if (levelIdx != -1)
  {
    String levelStr = req.substring(levelIdx + 6);
    // Only take first digit (0-4)
    int endIdx = 0;
    while (endIdx < levelStr.length() && isdigit(levelStr.charAt(endIdx)))
      endIdx++;
    levelStr = levelStr.substring(0, endIdx);
    newLevel = levelStr.toInt();
    if (newLevel >= 0 && newLevel <= 4)
    {
      logVerbosity = newLevel;
    }
  }
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.printf("OK logVerbosity=%d\n", logVerbosity);
}

/**
 * @brief Handler for enabling/disabling miniSEED recording via HTTP
 * @param client Output stream for HTTP response
 * @param req Request string containing the query parameters
 */
void serveMinISeedControl(Stream &client, const String &req)
{
  int enableIdx = req.indexOf("enable=");
  int formatIdx = req.indexOf("format=");
  bool newState = miniSeedRecordingEnabled;

  if (formatIdx != -1)
  {
    String formatStr = req.substring(formatIdx + 7);
    int endIdx = 0;
    while (endIdx < formatStr.length() && isalnum(formatStr.charAt(endIdx)))
      endIdx++;
    formatStr = formatStr.substring(0, endIdx);
    formatStr.toLowerCase();

    if (formatStr == "v3" || formatStr == "3")
    {
      miniSeedLogger.setFormat(MinISeedLogger::FORMAT_V3);
      debugSerialPrintln("miniSEED format set to v3");
    }
    else if (formatStr == "v2" || formatStr == "2")
    {
      miniSeedLogger.setFormat(MinISeedLogger::FORMAT_V2);
      debugSerialPrintln("miniSEED format set to v2");
    }
  }

  if (enableIdx != -1)
  {
    String enableStr = req.substring(enableIdx + 7);
    int endIdx = 0;
    while (endIdx < enableStr.length() && (isdigit(enableStr.charAt(endIdx)) || enableStr.charAt(endIdx) == '-'))
      endIdx++;
    enableStr = enableStr.substring(0, endIdx);
    int value = enableStr.toInt();
    newState = (value != 0);

    if (newState && !sd_card_ready)
    {
      debugSerialPrintln("miniSEED Recording: request denied, SD card not ready");
      newState = false;
    }

    if (newState)
    {
      miniSeedRecordingEnabled = true;
      miniSeedLogger.setRecordingEnabled(true);
      debugSerialPrintln("miniSEED Recording: ENABLED");
    }
    else
    {
      debugSerialPrintln("miniSEED Recording: DISABLED");
      stopMiniSeedRecording("HTTP disable request");
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.printf("{\"miniSeedRecording\":%s,\"miniSeedFormat\":\"%s\",\"sdCardReady\":%s}\n",
                miniSeedRecordingEnabled ? "true" : "false",
                miniSeedLogger.getFormatString(),
                sd_card_ready ? "true" : "false");
}

/**
 * @brief Handler for safely unmounting SD card via HTTP
 * @param client Output stream for HTTP response
 * @param req Request string (unused)
 */
void serveSDUnmount(Stream &client, const String &req)
{
  debugSerialPrintln("HTTP: SD Card unmount requested");

  bool success = true;
  String message = "";

  // Always stop recording and close immediately before unmount.
  stopMiniSeedRecording("SD unmount request");

  // Stop SD-backed operations. SD_SPI is stateless, so no explicit unmount call is needed.
  debugSerialPrintln("  Releasing SD card for safe removal...");

  // Small delay to ensure all operations complete
  delay(100);

  sd_card_ready = false;

  debugSerialPrintln("  SD card safely unmounted");
  message = "SD card safely unmounted. You can now remove it.";

  // Send JSON response
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();
  client.print("{\"success\":");
  client.print(success ? "true" : "false");
  client.print(",\"message\":\"");
  client.print(message);
  client.print("\",\"miniSeedRecording\":false,\"sdCardReady\":false,\"miniSeedFormat\":\"");
  client.print(miniSeedLogger.getFormatString());
  client.println("\"}");
}

/**
 * @brief Handler for listing mseed files via HTTP
 * @param client Output stream for HTTP response
 * @param req Request string (unused)
 */
void serveMseedFileList(Stream &client, const String &req)
{
  debugSerialPrintln("HTTP: Listing mseed files");

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Connection: close");
  client.println();

  if (!sd_card_ready)
  {
    client.println("{\"error\":\"SD card not ready\",\"files\":[]}");
    return;
  }

  // Check and create directory if it doesn't exist
  if (!fsDirectoryExists("/data/miniseed"))
  {
    debugSerialPrintln("  /data/miniseed doesn't exist, creating...");

    // Create /data directory if needed
    if (!fsDirectoryExists("/data"))
    {
      debugSerialPrintln("  Creating /data directory...");
      if (!fsCreateDirectory("/data"))
      {
        debugSerialPrintln("  Failed to create /data directory");
        client.println("{\"error\":\"Could not create /data directory\",\"files\":[]}");
        return;
      }
      debugSerialPrintln("  Created /data directory");
    }

    // Create /data/miniseed directory
    debugSerialPrintln("  Creating /data/miniseed directory...");
    if (!fsCreateDirectory("/data/miniseed"))
    {
      debugSerialPrintln("  Failed to create /data/miniseed directory");
      client.println("{\"error\":\"Could not create /data/miniseed directory\",\"files\":[]}");
      return;
    }
    debugSerialPrintln("  Created /data/miniseed directory");
  }

  client.print("{\"files\":[");

  dirent_t entries[128];
  int count = sd_spi_read_directory_path(&sd_fs_info, "/data/miniseed", entries, 128);
  if (count < 0)
  {
    debugSerialPrintln("  Failed to open /data/miniseed directory");
    client.println("],\"error\":\"Could not open /data/miniseed directory\"}");
    return;
  }

  bool first = true;
  for (int i = 0; i < count; i++)
  {
    // Skip directories
    if (entries[i].attributes & 0x10)
      continue;

    const char *filename = entries[i].filename;
    int len = strlen(filename);
    bool isMiniSeedFile = false;
    if (len > 6 && (strcmp(filename + len - 6, ".mseed") == 0 ||
                    strcmp(filename + len - 6, ".MSEED") == 0))
    {
      isMiniSeedFile = true;
    }
    else if (len > 4 && (strcmp(filename + len - 4, ".mse") == 0 ||
                         strcmp(filename + len - 4, ".MSE") == 0))
    {
      isMiniSeedFile = true;
    }

    if (isMiniSeedFile)
    {
      if (!first)
        client.print(",");
      first = false;

      client.print("{\"name\":\"");
      client.print(filename);
      client.print("\",\"size\":");
      client.print((unsigned long)entries[i].file_size);
      client.print("}");
    }
  }

  client.println("]}");
}

/**
 * @brief Handler for serving a specific mseed file via HTTP
 * @param client Output stream for HTTP response
 * @param req Request string containing filename parameter
 */
void serveMseedFile(Stream &client, const String &req)
{
  debugSerialPrintln("HTTP: Serving mseed file");

  // Extract filename parameter
  int fileIdx = req.indexOf("file=");
  if (fileIdx == -1)
  {
    client.println("HTTP/1.1 400 Bad Request");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>Error: Missing file parameter</h2></body></html>");
    return;
  }

  String filename = req.substring(fileIdx + 5);
  int endIdx = filename.indexOf('&');
  if (endIdx != -1)
    filename = filename.substring(0, endIdx);
  int spaceIdx = filename.indexOf(' ');
  if (spaceIdx != -1)
    filename = filename.substring(0, spaceIdx);

  // URL decode filename (basic implementation)
  filename.replace("%20", " ");
  filename.replace("%2F", "/");

  debugSerialPrintf("  Requested file: %s\n", filename.c_str());

  if (!sd_card_ready)
  {
    client.println("HTTP/1.1 503 Service Unavailable");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>Error: SD card not ready</h2></body></html>");
    return;
  }

  // Build full path
  String fullPath = "/data/miniseed/" + filename;

  uint32_t fileSize = 0;
  if (!sd_spi_get_file_size(&sd_fs_info, fullPath.c_str(), &fileSize))
  {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>Error: File not found</h2></body></html>");
    return;
  }

  // Send file with appropriate headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/octet-stream");
  client.print("Content-Disposition: attachment; filename=\"");
  client.print(filename);
  client.println("\"");
  client.print("Content-Length: ");
  client.println((unsigned long)fileSize);
  client.println("Connection: close");
  client.println();

  // Stream file content in chunks via SD_SPI
  const uint32_t bufSize = 512;
  uint8_t buf[bufSize];
  uint32_t offset = 0;

  while (offset < fileSize)
  {
    int n = sd_spi_read_file_chunk(&sd_fs_info, fullPath.c_str(), offset, buf, bufSize);
    if (n <= 0)
      break;
    client.write(buf, n);
    offset += (uint32_t)n;
  }

  if (offset != fileSize)
  {
    debugSerialPrintf("  WARNING: Sent %lu/%lu bytes for %s\n", (unsigned long)offset,
                      (unsigned long)fileSize, fullPath.c_str());
  }

  debugSerialPrintln("  File sent successfully");
}

/**
 * @brief Handler for serving the mseed viewer HTML page
 * @param client Output stream for HTTP response
 */
void serveMseedViewer(Stream &client)
{
  debugSerialPrintln("HTTP: Serving mseed viewer page");

  // Open the HTML file from the LittleFS filesystem
  File htmlFile = LittleFS.open("/mseedviewer.html", "r");

  if (!htmlFile)
  {
    client.println("HTTP/1.1 404 Not Found");
    client.println("Content-Type: text/html");
    client.println("Connection: close");
    client.println();
    client.println("<html><body><h2>Error: mseedviewer.html not found on filesystem.</h2></body></html>");
    return;
  }

  // Send HTTP headers
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();

  // Read and send file content in chunks
  const size_t bufSize = 512;
  char buf[bufSize];

  while (htmlFile.available())
  {
    size_t n = htmlFile.readBytes(buf, bufSize - 1);
    if (n == 0)
    {
      break;
    }
    buf[n] = '\0'; // Null terminate

    client.print(buf);
  }

  client.flush();
  htmlFile.close();
}

void debugPrint(const char *msg, uint16_t color = TFT_GREEN, int level = LOG_INFO)
{
  if (logVerbosity < level)
    return;
  debugSerialPrint(msg);
  if (mode == 0 && mode_initialized)
  {
    static int tft_line = 3;
    static const int font_height = 8 * TFT_TEXT_SIZE;
    int disp_height = tft.height();
    int max_lines = disp_height / font_height;

    // When requested, clear the text area and start over to overwrite instead of scrolling
    if (text_block_refresh)
    {
      // Avoid erasing the left-side menu overlay if visible; clear only the text region
      if (menu_visible)
      {
        int clear_x = 80; // menu occupies left ~80px
        int clear_w = tft.width() - clear_x;
        tft.fillRect(clear_x, 3 * font_height, clear_w, disp_height - 3 * font_height, TFT_BLACK);
      }
      else
      {
        tft.fillRect(0, 3 * font_height, tft.width(), disp_height - 3 * font_height, TFT_BLACK);
      }
      tft_line = 3;
      text_block_refresh = false;
    }

    const char *p = msg;
    while (*p)
    {
      const char *line_end = strchr(p, '\n');
      int len = line_end ? (line_end - p) : strlen(p);
      if (tft_line >= max_lines)
      {
        tft.fillRect(0, 3 * font_height, tft.width(), disp_height - 3 * font_height, TFT_BLACK);
        tft_line = 3;
      }
      tft.setTextColor(color);
      tft.setTextSize(TFT_TEXT_SIZE);
      tft.setTextDatum(TL_DATUM); // Ensure left-to-right text alignment
      // Create a temporary string for this line
      char line_buf[256];
      if (len >= sizeof(line_buf))
        len = sizeof(line_buf) - 1;
      memcpy(line_buf, p, len);
      line_buf[len] = '\0';
      // Position text to the right of menu if visible
      int text_x = menu_visible ? 80 : 0;
      tft.drawString(line_buf, text_x, tft_line * font_height);
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

void debugPrintln(const char *msg, int level = LOG_INFO)
{
  if (logVerbosity < level)
    return;
  char buffer[256];
  snprintf(buffer, sizeof(buffer), "%s\n", msg);
  debugPrint(buffer);
}

// ============================================================================
// XPT2046 FUNCTIONS
// ============================================================================

/**
 * @brief Initialize touch controller (improved from test_all)
 * TFT_eSPI handles touch initialization via eSPI_Setup.h configuration
 * Touch is enabled automatically when tft.begin() is called
 */
void initTouchController()
{
  debugSerialPrintln("Initializing touch controller via TFT_eSPI...");
  debugSerialPrintln("Touch support configured in eSPI_Setup.h");

  // TFT_eSPI touch is initialized automatically with tft.begin()
  // Touch pins are configured in eSPI_Setup.h:
  // - TOUCH_CS is configured automatically
  // - Touch calibration is stored in eSPI_Setup.h

  debugSerialPrintf("Touch CS pin: GP%d (managed by TFT_eSPI)\n", TOUCH_CS);
  if (TOUCH_IRQ != -1)
  {
    debugSerialPrintf("Touch IRQ pin: GP%d (optional for TFT_eSPI)\n", TOUCH_IRQ);
  }

  debugSerialPrintln("Touch controller ready (TFT_eSPI)");
}

void drawCalibrationCross(int x, int y, uint16_t color = TFT_RED)
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
    debugSerialPrintln("Error: Need at least 3 calibration points");
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
    debugSerialPrintln("Error: Singular matrix - calibration points may be collinear");
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

  debugSerialPrintln("MMSE Calibration Matrix:");
  debugSerialPrintf("A=%.6f, B=%.6f, C=%.6f\n", matrix.A, matrix.B, matrix.C);
  debugSerialPrintf("D=%.6f, E=%.6f, F=%.6f\n", matrix.D, matrix.E, matrix.F);

  return true;
}

/* Apply MMSE calibration to convert touch coordinates to display coordinates
 *
 * @param touch_x Raw touch X coordinate from touchscreen
 * @param touch_y Raw touch Y coordinate from touchscreen
 * @param display_x Output calibrated display X coordinate
 * @param display_y Output calibrated display Y coordinate
 */
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
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.printf("MMSE Touch Calibration\n");
  tft.printf("%d-point mode\n", NUM_CALIB_POINTS);
  tft.setTextSize(1);
  tft.print("Touch each crosshair\n");
  delay(2000);

  // Calibration points configuration
  // 5-point: 4 corners + center
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

  // Local structure to hold touch event data
  struct
  {
    uint16_t x, y, z;
  } touch_event_point;

  for (int i = 0; i < numPoints; ++i)
  {
    // Clear entire screen before each calibration point
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.printf("Touch cross %d/%d", i + 1, numPoints);
    drawCalibrationCross(lcd_x[i], lcd_y[i]);

    debugSerialPrintf("Waiting for touch at point %d...\n", i + 1);
    bool touch_detected = false;

    while (!touch_detected)
    {
      uint16_t tx = 0, ty = 0;
      if (tft.getTouch(&tx, &ty, TOUCH_THRESHOLD))
      {
        touch_event_point.x = tx;
        touch_event_point.y = ty;
        touch_event_point.z = 1000; // Simulate pressure value
        touch_detected = true;
        debugSerialPrintf("Touch detected: x=%d, y=%d, z=%d\n",
                          touch_event_point.x, touch_event_point.y, touch_event_point.z);
      }
      delay(10);
    }

    // Wait for touch release
    delay(100);
    while (tft.getTouch(&touch_event_point.x, &touch_event_point.y, TOUCH_THRESHOLD))
    {
      delay(10);
    }

    drawCalibrationCross(lcd_x[i], lcd_y[i], TFT_GREEN); // Draw green cross to indicate touch was registered

    // Store raw coordinates without rotation - let MMSE algorithm handle the transformation
    touch_x[i] = touch_event_point.x;
    touch_y[i] = touch_event_point.y;

    debugSerialPrintf("Calibration point %d: Expected LCD(%d,%d) <-> Raw Touch(%d,%d)\n",
                      i, lcd_x[i], lcd_y[i], touch_x[i], touch_y[i]);
    tft.fillScreen(TFT_BLACK);
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
    debugSerialPrintln("MMSE calibration matrix saved to /touch_calib.txt");
  }
  else
  {
    debugSerialPrintln("Failed to save touch calibration!");
  }

  // Verify calibration accuracy with 3 test points
  tft.fillScreen(TFT_BLACK);
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
    tft.fillRect(0, 0, tft.width(), 30, TFT_BLACK);
    tft.setCursor(10, 10);
    tft.printf("Touch test point %d/3", i + 1);
    drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], TFT_YELLOW);

    bool touch_detected = false;
    uint16_t tx = 0, ty = 0;
    while (!touch_detected)
    {
      if (tft.getTouch(&tx, &ty, TOUCH_THRESHOLD))
      {
        test_touch_x[i] = tx;
        test_touch_y[i] = ty;
        touch_detected = true;
      }
      delay(10);
    }
    delay(100);
    // Wait for touch release
    while (tft.getTouch(&tx, &ty, TOUCH_THRESHOLD))
    {
      delay(10);
    }

    // Calculate calibration accuracy for this point
    int calibrated_x, calibrated_y;
    float xt = (float)test_touch_x[i];
    float yt = (float)test_touch_y[i];
    calibrated_x = (int)(mmseCalibMatrix.A * xt + mmseCalibMatrix.B * yt + mmseCalibMatrix.C);
    calibrated_y = (int)(mmseCalibMatrix.D * xt + mmseCalibMatrix.E * yt + mmseCalibMatrix.F);

    float error_x = calibrated_x - test_lcd_x[i];
    float error_y = calibrated_y - test_lcd_y[i];
    float error = sqrtf(error_x * error_x + error_y * error_y);

    drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], TFT_GREEN);

    // Display error near the crosshair
    tft.setCursor(test_lcd_x[i] + 15, test_lcd_y[i] - 10);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(1);
    tft.printf("Err: %.1fpx", error);

    delay(1500);
    tft.fillScreen(TFT_BLACK);
  }

  // Calculate calibration accuracy
  float total_error = 0.0f;
  float max_error = 0.0f;

  debugSerialPrintln("\nCalibration Verification Results:");
  debugSerialPrintln("Point | Expected (X,Y) | Raw Touch (X,Y) | Calibrated (X,Y) | Error (pixels)");
  debugSerialPrintln("------|----------------|-----------------|------------------|---------------");

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

    debugSerialPrintf("  %d   | (%3d, %3d)     | (%4d, %4d)    | (%4d, %4d)      | %.2f\n",
                      i + 1, test_lcd_x[i], test_lcd_y[i],
                      test_touch_x[i], test_touch_y[i],
                      calibrated_x, calibrated_y, error);

    total_error += error;
    if (error > max_error)
      max_error = error;
  }

  float avg_error = total_error / numTestPoints;

  debugSerialPrintln("------|----------------|------------------|---------------");
  debugSerialPrintf("Average error: %.2f pixels\n", avg_error);
  debugSerialPrintf("Maximum error: %.2f pixels\n", max_error);

  // Display results on screen
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(10, 40);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.print("Calibration Complete\n\n");

  tft.setTextSize(1);
  tft.printf("Avg Error: %.1f px\n", avg_error);
  tft.printf("Max Error: %.1f px\n\n", max_error);

  if (avg_error < 5.0f)
  {
    tft.setTextColor(TFT_GREEN);
    tft.print("Excellent accuracy!");
  }
  else if (avg_error < 10.0f)
  {
    tft.setTextColor(TFT_YELLOW);
    tft.print("Good accuracy");
  }
  else
  {
    tft.setTextColor(TFT_RED);
    tft.print("Consider recalibrating");
  }

  // Reset text color to white for subsequent output
  tft.setTextColor(TFT_WHITE);

  delay(3000);

  // Clear screen to prevent cluttering of subsequent output
  tft.fillScreen(TFT_BLACK);
}

bool loadTouchCalibration()
{
  File f = LittleFS.open("/touch_calib.txt", "r");
  if (!f)
  {
    debugSerialPrintln("No touch calibration file found.");
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
      debugSerialPrintln("Invalid MMSE calibration file format.");
      LittleFS.remove("/touch_calib.txt");
      return false;
    }

    debugSerialPrintln("MMSE calibration matrix loaded:");
    debugSerialPrintf("A=%.6f, B=%.6f, C=%.6f\n", mmseCalibMatrix.A, mmseCalibMatrix.B, mmseCalibMatrix.C);
    debugSerialPrintf("D=%.6f, E=%.6f, F=%.6f\n", mmseCalibMatrix.D, mmseCalibMatrix.E, mmseCalibMatrix.F);
    return true;
  }
  else
  {
    // Legacy 3-point calibration - try to convert
    debugSerialPrintln("Found legacy 3-point calibration, converting to MMSE...");
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
      debugSerialPrintln("Invalid legacy calibration file.");
      LittleFS.remove("/touch_calib.txt");
      return false;
    }

    // Convert 3-point to MMSE matrix
    if (calculateMMSECalibration(3, lcd_x, lcd_y, touch_x, touch_y, mmseCalibMatrix))
    {
      debugSerialPrintln("Successfully converted to MMSE calibration.");
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
      debugSerialPrintln("Failed to convert legacy calibration.");
      LittleFS.remove("/touch_calib.txt");
      return false;
    }
  }
}

// ============================================================================
// ADS1256 FUNCTIONS
// ============================================================================

/*
Initialize ADS1256 ADC with SPI1 support
*/
void initADS1256()
{
  tftPrint("Initializing ADC... \n");

  // Initialize SPI1 with ADS1256 pin definitions
  debugSerialPrintln("DEBUG: Setting ADS1256 SPI pins...");
  ADS1256_SPI_PORT.setSCK(ADS1256_SCLK);
  ADS1256_SPI_PORT.setTX(ADS1256_MOSI);
  ADS1256_SPI_PORT.setRX(ADS1256_MISO);
  debugSerialPrintln("DEBUG: SPI pins configured");

  // Setting up CS, RESET, SYNC and SPI
  // Assigning default values to: STATUS, MUX, ADCON, DRATE
  // Performing a SYSCAL

  debugSerialPrintln("DEBUG: Calling ads.InitializeADC()...");
  ads.InitializeADC();
  debugSerialPrintln("DEBUG: ads.InitializeADC() completed");

  // Set a PGA value
  debugSerialPrintln("Setting PGA to 1... ");
  ads.setPGA(PGA_1); // 0b00000000 - DEC: 0
  //--------------------------------------------

  // Set initial input channel
  debugSerialPrintln("Setting MUX to single-ended channel 0... ");
  ads.setMUX(SING_0); // 0b00001111 - DEC: 15
  //--------------------------------------------

  // Set DRATE
  debugSerialPrintln("Setting DRATE to 500 SPS... ");
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

// ============================================================================
// ICM20948 FUNCTIONS
// ============================================================================

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
  // Assign IRQ handler and enable in one call (test code style)
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

  //  debugSerialPrint(temp.temperature);
  //
  //  debugSerialPrint(",");
  //
  //  debugSerialPrint(accel.acceleration.x);
  //  debugSerialPrint(","); debugSerialPrint(accel.acceleration.y);
  //  debugSerialPrint(","); debugSerialPrint(accel.acceleration.z);
  //
  //  debugSerialPrint(",");
  //  debugSerialPrint(gyro.gyro.x);
  //  debugSerialPrint(","); debugSerialPrint(gyro.gyro.y);
  //  debugSerialPrint(","); debugSerialPrint(gyro.gyro.z);
  //
  //  debugSerialPrint(",");
  //  debugSerialPrint(mag.magnetic.x);
  //  debugSerialPrint(","); debugSerialPrint(mag.magnetic.y);
  //  debugSerialPrint(","); debugSerialPrint(mag.magnetic.z);

  //  debugSerialPrintln();
  //
  //  delayMicroseconds(measurement_delay_us);
}

// ============================================================================
// NETWORKING FUNCTIONS
// ============================================================================

/**
 * @brief Get MAC address for WIZNET (from chip if possible, else fallback).
 * @param mac Pointer to 6-byte array to fill.
 */
void getWiznetMAC(byte *mac)
{
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
}

/**
 * @brief Get MAC address for WiFi (from chip).
 * @param mac Pointer to 6-byte array to fill.
 */
void getWiFiMAC(byte *mac)
{
  String macStr = WiFi.macAddress(); // Format: "AA:BB:CC:DD:EE:FF"
  int values[6];
  if (sscanf(macStr.c_str(), "%x:%x:%x:%x:%x:%x",
             &values[0], &values[1], &values[2], &values[3], &values[4], &values[5]) == 6)
  {
    for (int i = 0; i < 6; ++i)
      mac[i] = (byte)values[i];
  }
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

NetworkMode initWiFi()
{
  if (!loadWiFiCredentials())
  {
    debugSerialPrintln("Failed to load WiFi credentials from /wifi.txt");
    return NET_NONE;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, wifi_password);

  debugSerialPrint("Connecting to WiFi");
  int retry = 0;
  while ((WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0)) && retry < 30)
  {
    sleep_ms(500);
    debugSerialPrint(".");
    retry++;
  }
  debugSerialPrint("\n");

  if (WiFi.status() == WL_CONNECTED)
  {
    IPAddress wifiIP = WiFi.localIP();
    if (wifiIP == IPAddress(0, 0, 0, 0) || wifiIP[0] > 255 || wifiIP[1] > 255 || wifiIP[2] > 255 || wifiIP[3] > 255)
    {
      debugSerialPrint("WiFi connected but invalid IP address.\n");
      return NET_NONE;
    }
    byte mac[6];
    getWiFiMAC(mac);
    debugSerialPrintf("WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    debugSerialPrintf("WiFi connected, IP: %d.%d.%d.%d\n", wifiIP[0], wifiIP[1], wifiIP[2], wifiIP[3]);
    int rssi = WiFi.RSSI();
    if (rssi < 0 && rssi > -100)
    {
      debugSerialPrintf("WiFi RSSI: %d dBm\n", rssi);
    }
    else
    {
      debugSerialPrint("WiFi RSSI: Invalid\n");
    }
    wifiServer.begin();
    return NET_WIFI;
  }
  else
  {
    debugSerialPrint("WiFi connection failed.\n");
    return NET_NONE;
  }
  debugSerialPrint("WiFi initialized...\n");
}

/**
 * @brief Reset the WIZNET5K chip using the hardware reset pin (from test_all)
 * Improved implementation with proper timing and debug output
 */
void wiznet5k_reset()
{
  debugSerialPrintln("Resetting WIZNET5K chip...");
  digitalWrite(WIZNET_RST, LOW);  // Assert reset (active low)
  sleep_ms(10);                   // Hold reset for 10ms
  digitalWrite(WIZNET_RST, HIGH); // Release reset
  sleep_ms(100);                  // Wait for chip to stabilize
  debugSerialPrintln("WIZNET5K reset complete");
}

/**
 * @brief Initialize SPI bus and pins for WIZNET5K (improved from test_all)
 * This follows the proven initialization pattern from test_wiznet suite
 */
void wiznet_init_spi()
{
  debugSerialPrintln("Initializing WIZNET SPI...");
  debugSerialPrintln("Using improved initialization strategy from test_all");

  // Reinitialize SPI using Arduino SPI API to keep it in sync with Ethernet library
  // This pattern is proven to work reliably from test_all
  SPI.end();
  SPI.setSCK(WIZNET_SCLK);
  SPI.setTX(WIZNET_MOSI);
  SPI.setRX(WIZNET_MISO);
  SPI.begin();
  SPI.beginTransaction(SPISettings(WIZNET_SPI_BPS, MSBFIRST, SPI_MODE0));
  SPI.endTransaction();

  debugSerialPrintln("Wiznet SPI initialized");
  debugSerialPrintf("MISO: GP%d, MOSI: GP%d, SCK: GP%d, CS: GP%d\n",
                    WIZNET_MISO, WIZNET_MOSI, WIZNET_SCLK, WIZNET_SPI_CS);
  debugSerialPrintf("RST: GP%d, INT: GP%d\n", WIZNET_RST, WIZNET_INT);
  log_wiznet_spi_speed("After Wiznet SPI init");
}

/**
 * @brief Configure interrupt pin for WIZNET5K (from test_all)
 * Sets up INT pin with proper pull-up configuration
 */
void wiznet_init_interrupt()
{
  debugSerialPrintln("Configuring WIZNET INT pin...");
  gpio_init(WIZNET_INT);
  gpio_set_dir(WIZNET_INT, GPIO_IN);
  gpio_pull_up(WIZNET_INT);
  debugSerialPrintf("Wiznet INT pin (GP%d) configured with pull-up\n", WIZNET_INT);
}

/**
 * @brief Interrupt handler for WIZNET5K INT pin.
 * You can add custom handling here if needed.
 */
void wiznet5k_int_isr(uint gpio, uint32_t events)
{
  // Example: Print or set a flag (implement as needed)
  debugSerialPrintf("WIZNET5K interrupt detected on GPIO %d\n", gpio);
  // Optionally: set a volatile flag for main loop processing
}

NetworkMode initNetworking()
{
  byte mac[6];
  char resultStr[32];
  // Fallback static IP parameters if DHCP fails (must be in same subnet as gateway)
  IPAddress staticCounter(10, 0, 3, 46);
  IPAddress staticGateway(10, 0, 0, 1);
  IPAddress staticSubnet(255, 255, 252, 0);
  IPAddress dnsServer(8, 8, 8, 8);

  if (!use_networking)
  {
    tftPrint("Networking disabled, running unconnected.\n");
    return NET_NONE;
  }

  tftPrint("Initializing Networking...\n");

  // First try WIZNET (Ethernet) with 10 second timeout
  tftPrint("Trying WIZNET (Ethernet)...\n");
  unsigned long wiznetStartTime = millis();

  // Ensure other SPI0 devices are deselected before Wiznet init
  releaseSPIBus();

  // --- Reset WIZNET5K chip before SPI init ---
  wiznet5k_reset();

  // --- Initialize SPI ---
  wiznet_init_spi();
  Ethernet.init(WIZNET_SPI_CS);
  getWiznetMAC(mac);

  // Try DHCP first
  tftPrint("Trying DHCP...\n");
  int dhcpResult = Ethernet.begin(mac, DHCP_TIMEOUT);

  if (dhcpResult == 0)
  {
    // DHCP failed, try static IP
    tftPrint("DHCP failed, trying static IP...\n");

    Ethernet.begin(mac, staticCounter, dnsServer, staticGateway, staticSubnet);
    delay(100);
    assignedIP = staticCounter;
  }
  else
  {
    // DHCP succeeded, check if we got a valid IP
    assignedIP = Ethernet.localIP();
    if (assignedIP[0] == 0 || assignedIP == IPAddress(0, 0, 0, 0))
    {
      tftPrint("DHCP gave invalid IP, trying static IP...\n");
      Ethernet.begin(mac, staticCounter, dnsServer, staticGateway, staticSubnet);
      delay(100);
      assignedIP = staticCounter;
    }
    else
    {
      debugSerialPrintf("DHCP successful, IP: %d.%d.%d.%d\n", assignedIP[0], assignedIP[1], assignedIP[2], assignedIP[3]);
      // Store DHCP-assigned network parameters
      assignedGateway = Ethernet.gatewayIP();
      assignedSubnet = Ethernet.subnetMask();
      assignedDNS = Ethernet.dnsServerIP();
    }
  }

  // Wait a bit for link status to stabilize
  delay(100);

  // Check if we have a valid IP address
  assignedIP = Ethernet.localIP();
  if (assignedIP[0] != 0 && assignedIP != IPAddress(0, 0, 0, 0))
  {
    // WIZNET connected successfully
    sprintf(resultStr, "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    tftPrint(resultStr);
    sprintf(resultStr, "Ethernet IP: %s\n", assignedIP.toString().c_str());
    tftPrint(resultStr);

    ethServer.begin();
    tftPrint("Ethernet Network initialized successfully!\n");
    return NET_WIZNET;
  }
  else
  {
    tftPrint("No valid IP address, trying WiFi as fallback...\n");
  }

  // WIZNET failed, check if 10 seconds have passed
  if (millis() - wiznetStartTime < 10000)
  {
    // Wait for remaining time
    delay(10000 - (millis() - wiznetStartTime));
  }

  // Try WiFi
  NetworkMode wifiResult = initWiFi();
  if (wifiResult == NET_WIFI)
  {
    tftPrint("WiFi connected successfully!\n");
    return NET_WIFI;
  }

  // Both failed
  tftPrint("All networking attempts failed, running unconnected.\n");
  return NET_NONE;
}

// TODO: Review NTP connection when using WiFi. Due to metal case the WiFi connection is bad could be the reason for the problems

bool setTimeFromNTP(NetworkMode currentNetMode)
{
  // Pass netMode as parameter to avoid volatile variable stack corruption
  // Variables declared on stack for minimal memory footprint
  IPAddress ntpServerIP;
  uint8_t ntp_packet[48] = {0};
  uint8_t recv_buf[48];
  char resultStr[64];
  uint32_t start = millis();
  const uint32_t timeout_ms = 10000; // 10 second overall timeout
  int packetSize = 0;
  uint32_t secs_since_1900;
  uint32_t unix_time;

  debugSerialPrintf("setTimeFromNTP: netMode=%d (WIFI=%d, WIZNET=%d)\n", currentNetMode, NET_WIFI, NET_WIZNET);

  if (!ntpServerIP.fromString(NTP_SERVER))
  {
    tftPrint("Invalid NTP server IP.\n");
    return false;
  }

  // Check timeout before network operations
  if (millis() - start > timeout_ms)
  {
    tftPrint("NTP timeout before network init.\n");
    return false;
  }

  // Initialize NTP packet
  ntp_packet[0] = 0b11100011;

  if (currentNetMode == NET_WIFI)
  {
    // Only create WiFi UDP object when needed
    WiFiUDP udp;

    if (udp.begin(0) == 0)
    {
      tftPrint("Failed to open UDP socket for NTP.\n");
      return false;
    }
    udp.beginPacket(ntpServerIP, NTP_PORT);
    udp.write(ntp_packet, sizeof(ntp_packet));
    udp.endPacket();
    debugSerialPrint("NTP packet sent, waiting for response...\n");

    int attempts = 0;
    while (attempts < 1000) // Check more frequently with 10ms intervals
    {
      // Check overall timeout
      if (millis() - start > timeout_ms)
      {
        udp.stop();
        tftPrint("NTP overall timeout (10s) exceeded.\n");
        return false;
      }

      packetSize = udp.parsePacket();
      if (packetSize == 48)
      {
        debugSerialPrintf("NTP response received: %d bytes\n", packetSize);
        udp.read(recv_buf, 48);
        udp.stop();
        break;
      }
      delay(10);
      attempts++;
    }
  }
  else if (currentNetMode == NET_WIZNET)
  {
    // Only create Ethernet UDP object when needed
    EthernetUDP ethUdp;

    if (ethUdp.begin(0) == 0)
    {
      tftPrint("Failed to open UDP socket for NTP.\n");
      return false;
    }
    ethUdp.beginPacket(ntpServerIP, NTP_PORT);
    ethUdp.write(ntp_packet, sizeof(ntp_packet));
    ethUdp.endPacket();
    debugSerialPrint("NTP packet sent, waiting for response...\n");

    int attempts = 0;
    while (attempts < 1000) // Check more frequently with 10ms intervals
    {
      // Check overall timeout
      if (millis() - start > timeout_ms)
      {
        ethUdp.stop();
        tftPrint("NTP overall timeout (10s) exceeded.\n");
        return false;
      }

      packetSize = ethUdp.parsePacket();
      if (packetSize == 48)
      {
        debugSerialPrintf("NTP response received: %d bytes\n", packetSize);
        ethUdp.read(recv_buf, 48);
        ethUdp.stop();
        break;
      }
      delay(10);
      attempts++;
    }
  }
  else
  {
    debugSerialPrint("NTP requires network connection.\n");
    return false;
  }

  if (packetSize < 48)
  {
    tftPrint("NTP response timeout or too short.\n");
    return false;
  }

  secs_since_1900 = (recv_buf[40] << 24) | (recv_buf[41] << 16) | (recv_buf[42] << 8) | recv_buf[43];
  unix_time = secs_since_1900 - 2208988800U;

  struct timeval tv;
  tv.tv_sec = unix_time;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);

  time_t t = unix_time;
  tm *tm_info = localtime(&t); // <-- Now 'tm' is fully defined from <time.h>

  sprintf(resultStr, "System time set from NTP: %04d-%02d-%02d %02d:%02d:%02d\n",
          tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday,
          tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
  tftPrint(resultStr);
  return true;
}

void setLocalTimezone(const char *tz)
{
  setenv("TZ", tz, 1);
  tzset();
}

/** Helper to format MAC address as string
 * out must be at least 18 bytes to hold the formatted string.
 */
void formatMACString(byte *mac, char *out)
{
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ============================================================================
// WEBSERVER FUNCTIONS
// ============================================================================

#ifdef TESTING_WEBSERVER
void serveSimpleCounterPage(Stream &client, uint32_t counter, uint32_t reqCount)
{
  const int DATA_POINTS = 200;
  String chartDataJSON = "[";
  for (int i = 0; i < DATA_POINTS; i++)
  {
    float x = i * 1.0f;
    float y = sinf(x * 0.1f) * sinf(x * 0.01f);
    chartDataJSON += "[";
    chartDataJSON += String(x, 1);
    chartDataJSON += ",";
    chartDataJSON += String(y, 3);
    chartDataJSON += "]";
    if (i < DATA_POINTS - 1)
      chartDataJSON += ",";
  }
  chartDataJSON += "]";

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  client.println("<!DOCTYPE html>");
  client.println("<html><head>");
  client.println("<title>Counter Button Test</title>");
  client.println("<script src=\"https://code.highcharts.com/highcharts.js\"></script>");
  client.println("<style>");
  client.println("body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }");
  client.println("button { font-size: 24px; padding: 20px 40px; margin: 20px; background: #4CAF50; color: white; border: none; border-radius: 8px; cursor: pointer; }");
  client.println("button:hover { background: #45a049; }");
  client.println(".counter { font-size: 48px; color: #ff6600; margin: 20px; }");
  client.println("#chart-container { width: 90%; height: 400px; margin: 20px auto; border: 2px solid #ddd; border-radius: 8px; }");
  client.println(".info { font-size: 18px; color: #666; margin: 10px; }");
  client.println("</style>");
  client.println("</head><body>");
  client.println("<h1>Counter Button Test</h1>");
  client.printf("<div class='counter'>Counter: %lu</div>", (unsigned long)counter);
  client.printf("<div class='info'>Requests: %lu</div>", (unsigned long)reqCount);
  client.println("<button onclick=\"incrementCounter()\">Increment Counter</button>");
  client.println("<div id=\"chart-container\"></div>");
  client.println("<script>");
  client.println("let chart;");
  client.println("let dataIndex = 0;");
  client.println("const chartData = ");
  client.println(chartDataJSON);
  client.println(";");
  client.println("function createChart() {");
  client.println("  chart = Highcharts.chart('chart-container', {");
  client.println("    chart: { type: 'line', animation: false, marginRight: 10 },");
  client.println("    title: { text: 'Modulated Sine Wave (Calculated Server-Side)' },");
  client.println("    xAxis: { type: 'linear', tickPixelInterval: 150 },");
  client.println("    yAxis: { title: { text: 'Amplitude' }, plotLines: [{ value: 0, width: 1, color: '#808080' }] },");
  client.println("    tooltip: { formatter: function() { return '<b>' + this.series.name + '</b><br/>' + 'Time: ' + this.x.toFixed(1) + '<br/>' + 'Value: ' + this.y.toFixed(3); } },");
  client.println("    legend: { enabled: false },");
  client.println("    exporting: { enabled: false },");
  client.println("    series: [{ name: 'Modulated Sine', data: [] }]");
  client.println("  });");
  client.println("}");
  client.println("function updateChart() {");
  client.println("  if (dataIndex < chartData.length) {");
  client.println("    chart.series[0].addPoint(chartData[dataIndex], true, false);");
  client.println("    dataIndex++;");
  client.println("  }");
  client.println("}");
  client.println("function incrementCounter() {");
  client.println("  fetch('/increment').then(() => {");
  client.println("    fetch('/').then(response => response.text()).then(html => {");
  client.println("      const parser = new DOMParser();");
  client.println("      const doc = parser.parseFromString(html, 'text/html');");
  client.println("      const counterDiv = doc.querySelector('.counter');");
  client.println("      const infoDiv = doc.querySelector('.info');");
  client.println("      if (counterDiv) document.querySelector('.counter').innerHTML = counterDiv.innerHTML;");
  client.println("      if (infoDiv) document.querySelector('.info').innerHTML = infoDiv.innerHTML;");
  client.println("    });");
  client.println("  });");
  client.println("}");
  client.println("createChart();");
  client.println("setInterval(updateChart, 100);");
  client.println("</script>");
  client.println("</body></html>");
}
#endif

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
  client.println("Content-Type: text/html; charset=utf-8");
  client.println("Connection: close");
  client.println();

  // Read and send file content in chunks
  const size_t bufSize = 512;
  char buf[bufSize];

  while (htmlFile.available())
  {
    size_t n = htmlFile.readBytes(buf, bufSize - 1);
    if (n == 0)
    {
      break;
    }
    buf[n] = '\0'; // Null terminate

    client.print(buf);
  }

  client.flush();
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
  if (netMode == NET_WIFI)
    nettype = "WLAN";
  else if (netMode == NET_WIZNET)
    nettype = "Ethernet";

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
  char axisChar = (display_axis == 0) ? 'X' : (display_axis == 1) ? 'Y'
                                                                  : 'R';
  client.print(axisChar);
  client.print("\"");
  client.print(",\"spectrogramMin\":");
  client.print(spectrogram_min);
  client.print(",\"spectrogramMax\":");
  client.print(spectrogram_max);
  client.print(",\"highchartsOk\":");
  client.print(highchartsWorking ? "true" : "false");
  client.print(",\"highchartsLastReport\":");
  client.print((unsigned long)highchartsLastReport);
  client.print(",\"miniSeedRecording\":");
  client.print(miniSeedRecordingEnabled ? "true" : "false");
  client.print(",\"miniSeedFormat\":\"");
  client.print(miniSeedLogger.getFormatString());
  client.print("\"");
  client.print(",\"sdCardReady\":");
  client.print(sd_card_ready ? "true" : "false");
  client.println("}");
  client.flush();
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

  if (netMode == NET_WIZNET)
  {
    ethUdpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    ethUdpStream.write((const uint8_t *)packet, strlen(packet));
    ethUdpStream.endPacket();
  }
  else if (netMode == NET_WIFI)
  {
    wifiUdpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    wifiUdpStream.write((const uint8_t *)packet, strlen(packet));
    wifiUdpStream.endPacket();
  }

  // Release SPI bus after UDP transmission to prevent state corruption
  releaseSPIBus();
}

void sendSensorUDPStream(const char *channel, const int *data, size_t count)
// Overloaded for int data
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

  if (netMode == NET_WIZNET)
  {
    ethUdpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    ethUdpStream.write((const uint8_t *)packet, strlen(packet));
    ethUdpStream.endPacket();
  }
  else if (netMode == NET_WIFI)
  {
    wifiUdpStream.beginPacket("255.255.255.255", UDP_STREAM_PORT);
    wifiUdpStream.write((const uint8_t *)packet, strlen(packet));
    wifiUdpStream.endPacket();
  }

  // Release SPI bus after UDP transmission to prevent state corruption
  releaseSPIBus();
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
      mode_initialized = false;

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
      mode_initialized = false;
      debugPrintln("Display axis changed to X");
    }
    else if (axisStr == "Y")
    {
      display_axis = 1;
      // Reset display flags to force re-initialization with new axis data
      mode_initialized = false;
      debugPrintln("Display axis changed to Y");
    }
    else if (axisStr == "R")
    {
      display_axis = 2;
      mode_initialized = false;
      debugPrintln("Display axis changed to R (radial)");
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  char axisChar = (display_axis == 0) ? 'X' : (display_axis == 1) ? 'Y'
                                                                  : 'R';
  client.printf("OK axis=%c\n", axisChar);
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

    // Also print to serial if in debug mode
#ifdef DEBUG_BUILD
    debugPrintln("Touch calibration data requested via HTTP");
    f = LittleFS.open("/touch_calib.txt", "r");
    if (f)
    {
      debugSerialPrintln("\n=== touch_calib.txt content ===");
      while (f.available())
      {
        Serial.write(f.read());
      }
      debugSerialPrintln("\n=== End of file ===");
      f.close();
    }
#endif
  }
  else
  {
    client.println("touch_calib.txt not found!");
    debugPrintln("touch_calib.txt not found!");
  }
}

void serveHighchartsStatus(Stream &client, const String &req)
{
  // Parse status parameter from request: /highcharts?status=ok or status=error
  int statusIdx = req.indexOf("status=");
  if (statusIdx != -1)
  {
    String statusStr = req.substring(statusIdx + 7);
    // Extract until space or end of line
    int endIdx = statusStr.indexOf(' ');
    if (endIdx != -1)
      statusStr = statusStr.substring(0, endIdx);

    highchartsWorking = (statusStr == "ok");
    highchartsLastReport = millis();

    if (highchartsWorking)
    {
      debugPrintln("Highcharts: Loaded successfully");
    }
    else
    {
      debugPrintf("Highcharts: Error reported - %s\n", statusStr.c_str());
    }
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println("OK");
}

// ============================================================================
// PLOT FUNCTIONS
// ============================================================================

void setup_waveform()
{
  // Clear the display
  tft.fillScreen(TFT_BLACK);

  // Draw center line
  tft.drawLine(waveform_x, waveform_mid_y,
               waveform_x + waveform_w, waveform_mid_y,
               TFT_LIGHTGREY);

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
    tft.drawPixel(x, upper_trigger_y, TFT_LIGHTGREY);
    tft.drawPixel(x + 1, upper_trigger_y, TFT_LIGHTGREY);
    tft.drawPixel(x, lower_trigger_y, TFT_LIGHTGREY);
    tft.drawPixel(x + 1, lower_trigger_y, TFT_LIGHTGREY);
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
  tft.drawLine(0, axis_y, 320, axis_y, TFT_WHITE);

  // Draw frequency labels: 0, 50, 100, 150, 200, 250 Hz
  tft.setTextSize(1);
  tft.setTextColor(TFT_WHITE);

  for (int freq = 0; freq <= 250; freq += 50)
  {
    int x_pos = (int)(freq / nyquist_freq * 320);
    // Draw tick mark
    tft.drawLine(x_pos, axis_y, x_pos, axis_y - 3, TFT_WHITE);
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
  // For vr (radial), the max value is always positive and can be much higher, so use a different scale
  float scale;
  if (display_axis == 2)
  {
    // Estimate max expected vr (e.g., sqrt(2) * threshold for two maxed channels)
    scale = (3.0f * waveform_h / 8.0f) / (CROSSING_THRESHOLD * 1.5f);
  }
  else
  {
    scale = (3.0f * waveform_h / 8.0f) / CROSSING_THRESHOLD;
  }

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
    int line_color = outside_threshold ? TFT_RED : color;

    if (i > 0)
    {
      tft.drawLine(last_x, last_old_y, x_val, old_y_val, TFT_BLACK);
      tft.drawLine(last_x, last_y, x_val, y_val, line_color);

      // Redraw trigger level dashes if waveform passes through them (only in waveform mode)
      if (draw_triggers && ((x_val % 4) == 0 || (x_val % 4) == 1))
      {
        tft.drawPixel(x_val, upper_trigger_y, TFT_LIGHTGREY);
        tft.drawPixel(x_val, lower_trigger_y, TFT_LIGHTGREY);
      }
    }
    last_x = x_val;
    last_y = y_val;
    last_old_y = old_y_val;
    last_points[i] = y_val;
  }

  // Redraw menu indicator after waveform to keep it visible
  drawMenuIndicator(disp_column);
}

// ============================================================================
// FFT FOR SPECTROGRAM PLOT
// from https://github.com/ArmDeveloperEcosystem/audio-spectrogram-example-for-pico
// ============================================================================

/*
Fast magma colormap lookup with linear interpolation
*/
inline uint16_t getMagmaColor(uint8_t value)
{
  // Replaces 257-line COLOR_MAP array, saves ~500 bytes of code space
  static const uint16_t MAGMA_LUT[16] = {
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

  // Determine which two LUT entries to interpolate between
  uint8_t index = value >> 4;  // Divide by 16 to get LUT index (0-15)
  uint8_t frac = value & 0x0F; // Remainder for interpolation (0-15)

  if (index >= 15)
    return MAGMA_LUT[15]; // Clamp to max

  // Read two adjacent colors
  uint16_t c1 = MAGMA_LUT[index];
  uint16_t c2 = MAGMA_LUT[index + 1];

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

  int range = running_max - running_min;
  if (range < 1)
    range = 1;

  // When menu is visible, pause scrolling to keep menu fixed
  if (menu_visible)
  {
    // Keep the display static while the menu is open
    return;
  }

  // Hardware scrolling for downward movement (new data at top):

  int x = scroll_x_start; // Draw at line that's currently at right corner of display
  scrollDisplay(1);       // Scroll down by 1 line (content moves down, new line appears at top)

  // Draw horizontal row of frequency data (240 frequency bins across width)
  for (int y = 0; y < MIN(240, n_values); ++y)
  {

    // Scale value from [running_min, running_max] to [0, 255]
    int v = ((long)(values[y] - running_min) * 255) / range;
    v = MAX(0, MIN(255, v)); // Clamp to valid range

    int color = getMagmaColor(v);
    tft.drawPixel(x, 240 - y, color);
  }

  // Next time we'll draw at the new scroll_x_start (new top line)
  disp_column = scroll_x_start;

  drawMenuIndicator(scroll_x_start);
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

void buffer_feed(int sample)
{
  ring_buffer[ring_buffer_tail++] = sample;
  ring_buffer_tail &= RING_BUFFER_SIZE - 1;
}

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

// ============================================================================
// TOUCH MENU SYSTEM
// ============================================================================

/**
 * @brief Draws the mode selection menu on the TFT display.
 *
 * This function renders a semi-transparent background, a title, and buttons for each mode option.
 * The buttons are drawn with their respective colors and labels.
 * The menu visibility state and show time are updated accordingly.
 */
void drawModeMenu()
{
  // Freeze scrolling and reset origin so the menu is always drawn at the top-left
  saved_scroll_offset = disp_column;
  // Do not change scroll position; compensate with offsets instead to avoid shifting content

  // Determine the physical top-left coordinates under current scroll/rotation
  int origin_x = 0;
  int origin_y = 0;
  if (TFT_ROTATION == 1 || TFT_ROTATION == 3)
  {
    origin_x = saved_scroll_offset % tft.width();
  }
  else
  {
    origin_y = saved_scroll_offset % tft.height();
  }

  // Clear old menu indicator artifacts before drawing the menu
  tft.fillRect(origin_x, origin_y, 24, 20, TFT_BLACK);

  // Draw semi-transparent background (simulate with black box)
  tft.fillRect(origin_x, origin_y + 20, 80, 220, TFT_BLACK);

  // Draw title
  tft.setCursor(origin_x + 10, origin_y + 25);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.print("MODE");

  // Draw buttons
  for (int i = 0; i < NUM_MENU_BUTTONS; i++)
  {
    MenuButton &btn = menu_buttons[i];
    tft.fillRect(origin_x + btn.x, origin_y + btn.y, btn.w, btn.h, btn.color);
    tft.drawRect(origin_x + btn.x, origin_y + btn.y, btn.w, btn.h, TFT_WHITE);

    // Center text in button
    tft.setCursor(origin_x + btn.x + 5, origin_y + btn.y + 15);
    tft.setTextColor(TFT_WHITE, btn.color);
    tft.setTextSize(1);
    tft.print(btn.label);
  }

  menu_visible = true;
  menu_show_time = millis();
}

/**
 * @brief Checks if the given touch coordinates are within the bounds of a button.
 *
 * @param touch_x The x-coordinate of the touch event.
 * @param touch_y The y-coordinate of the touch event.
 * @param btn Reference to the MenuButton object to check against.
 * @return true if the touch is inside the button's area, false otherwise.
 */
bool isTouchInButton(int touch_x, int touch_y, MenuButton &btn)
{
  return (touch_x >= btn.x && touch_x <= (btn.x + btn.w) &&
          touch_y >= btn.y && touch_y <= (btn.y + btn.h));
}

/**
 * @brief Handles touchscreen input and manages mode switching via a menu.
 *
 * This function polls the touchscreen, applies debouncing, and detects touch events.
 * If a touch is detected, it checks if the touch is within the menu toggle area or on a menu button.
 * When a menu button is pressed, it updates the current mode and hides the menu.
 * The function also manages menu visibility and auto-hides the menu after a timeout.
 * Debug information is printed based on the log verbosity level.
 *
 * @param current_mode Reference to the current mode variable; may be updated if a menu button is pressed.
 * @return true if the mode was changed by a menu button press, false otherwise.
 */
bool handleTouchInput(volatile int &current_mode)
{
  static unsigned long last_touch_time = 0;
  static bool touch_was_pressed = false;
  const unsigned long DEBOUNCE_MS = 200;

  // Only poll for touch events when IRQ flag is set
  if (!touch_event_flag)
    return false;

  // Clear the flag and re-enable IRQ for next touch
  touch_event_flag = false;
  gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true);

  // Debounce after IRQ flag is set (test code style)
  if (millis() - last_touch_time < DEBOUNCE_MS)
  {
    return false;
  }

  uint16_t tx = 0, ty = 0;
  bool is_touched = tft.getTouch(&tx, &ty, TOUCH_THRESHOLD);

  if (!is_touched)
  {
    // False trigger - no actual touch detected
    return false;
  }

  // Create a structure to hold touch coordinates (compatible with existing code)
  struct
  {
    uint16_t x, y, z;
  } tp;
  tp.x = tx;
  tp.y = ty;
  tp.z = 1000; // Pressure value for touched state

  // Log touch data for debugging
  if (logVerbosity >= LOG_INFO)
  {
    debugPrintf("Touch event (IRQ): x=%d, y=%d\n", tp.x, tp.y);
  }

  // Process touch event (same logic as before)
  touch_was_pressed = true;
  last_touch_time = millis();

  // Convert touch coordinates to display coordinates
  int display_x, display_y;
  applyMMSECalibration(tp.x, tp.y, display_x, display_y);

  if (logVerbosity >= LOG_INFO)
  {
    debugPrintf("Touch event detected at display coordinates: x=%d, y=%d\n", display_x, display_y);
  }

  // Wake from display-off mode (-1) on any touch: resume to spectrogram (mode 3)
  if (current_mode == -1)
  {
    current_mode = 3;
    menu_visible = false;
    scrollAddress(saved_scroll_offset); // Restore hardware scroll position
    disp_column = saved_scroll_offset % waveform_w;
    tft.fillScreen(TFT_BLACK);
    debugPrintln("Waking display to mode 3 (spectrogram) via touch");
    return true;
  }

  // Check if touch is in the menu toggle area (top-left corner)
  if (!menu_visible && display_x < 80 && display_y < 20)
  {
    if (logVerbosity >= LOG_INFO)
      debugPrintln("Menu symbol pressed - showing menu", LOG_INFO);
    drawModeMenu();
    return false;
  }

  if (!menu_visible && logVerbosity >= LOG_INFO)
    debugPrintln("Touch outside menu symbol area", LOG_INFO);

  // If menu is visible, check button touches
  if (menu_visible)
  {
    // Calculate the same offset compensation used in drawModeMenu()
    int touch_origin_x = 0;
    int touch_origin_y = 0;
    if (TFT_ROTATION == 1 || TFT_ROTATION == 3)
    {
      touch_origin_x = saved_scroll_offset % tft.width();
    }
    else
    {
      touch_origin_y = saved_scroll_offset % tft.height();
    }

    // Adjust touch coordinates to account for menu offset
    int adjusted_touch_x = display_x - touch_origin_x;
    int adjusted_touch_y = display_y - touch_origin_y;

    for (int i = 0; i < NUM_MENU_BUTTONS; i++)
    {
      if (isTouchInButton(adjusted_touch_x, adjusted_touch_y, menu_buttons[i]))
      {
        if (logVerbosity >= LOG_DEBUG)
          debugPrintf("Menu button pressed: %s (mode %d)\n", menu_buttons[i].label, menu_buttons[i].mode_value);
        // Mode button pressed
        int new_mode = menu_buttons[i].mode_value;
        if (new_mode != current_mode)
        {
          current_mode = new_mode;
          menu_visible = false;

          // Restore scroll position and keep drawing aligned after closing the menu
          scrollAddress(saved_scroll_offset); // Restore hardware scroll position
          disp_column = saved_scroll_offset % waveform_w;

          // Clear screen for new mode
          tft.fillScreen(TFT_BLACK);

          debugPrint("Mode changed to: ");
          debugPrintf("%d\n", current_mode);
          return true;
        }
      }
    }
    if (logVerbosity >= LOG_DEBUG)
      debugPrintln("Touch in menu area but outside buttons", LOG_DEBUG);
  }

  // Auto-hide menu after timeout
  if (menu_visible && (millis() - menu_show_time > MENU_TIMEOUT_MS))
  {
    menu_visible = false;
    scrollAddress(saved_scroll_offset); // Restore hardware scroll position
    disp_column = saved_scroll_offset % waveform_w;
    tft.fillScreen(TFT_BLACK);
    // Redraw menu indicator after timeout
    if (current_mode > 0)
    {
      drawMenuIndicator(disp_column);
    }
  }

  return false;
}

// ============================================================================
// FILESYSTEM UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Creates a directory path on the SD card (supports nested directory creation)
 * @param path Directory path to create (e.g., "/data/miniseed")
 * @return true if directory was created or already exists, false on error
 */
bool fsCreateDirectory(const char *path)
{
  if (!sd_card_ready || !path)
    return false;

  return sd_spi_create_directory(&sd_fs_info, path);
}

/**
 * @brief Checks if a directory exists on the SD card
 * @param path Directory path to check
 * @return true if directory exists, false otherwise
 */
bool fsDirectoryExists(const char *path)
{
  if (!sd_card_ready || !path)
    return false;

  return sd_spi_directory_exists(&sd_fs_info, path);
}

/**
 * @brief Checks if a file exists on the SD card
 * @param path File path to check
 * @return true if file exists, false otherwise
 */
bool fsFileExists(const char *path)
{
  if (!sd_card_ready || !path)
    return false;

  return sd_spi_file_exists(&sd_fs_info, path);
}

/**
 * @brief Writes data to a file on the SD card
 * @param path File path (including parent directories which will be created if needed)
 * @param data Pointer to data buffer to write
 * @param size Number of bytes to write
 * @return Number of bytes written, or -1 on error
 */
int fsWriteFile(const char *path, const uint8_t *data, uint32_t size)
{
  if (!sd_card_ready || !path || !data || size == 0)
    return -1;

  return sd_spi_write_file(&sd_fs_info, path, data, size);
}

/**
 * @brief Reads data from a file on the SD card
 * @param path File path to read
 * @param buffer Pointer to buffer where data will be stored
 * @param max_size Maximum bytes to read
 * @return Number of bytes read, or -1 on error
 */
int fsReadFile(const char *path, uint8_t *buffer, uint32_t max_size)
{
  if (!sd_card_ready || !path || !buffer || max_size == 0)
    return -1;

  return sd_spi_read_file(&sd_fs_info, path, buffer, max_size);
}

/**
 * @brief Deletes a file from the SD card
 * @param path File path to delete
 * @return true if file was deleted successfully, false on error
 */
bool fsDeleteFile(const char *path)
{
  if (!sd_card_ready || !path)
    return false;

  return sd_spi_delete_file(&sd_fs_info, path);
}

/**
 * @brief Lists files and directories in a directory
 * @param path Directory path to list (default is root "/" if NULL)
 * @param entries Pointer to array of dirent_t structures to populate
 * @param max_entries Maximum number of entries to return
 * @return Number of entries found, or -1 on error
 */
int fsListDirectory(const char *path, dirent_t *entries, int max_entries)
{
  if (!sd_card_ready || !entries || max_entries <= 0)
    return -1;

  // For now, only root directory is supported by sd_spi_read_directory
  // This can be extended in the future if needed
  return sd_spi_read_directory(&sd_fs_info, entries, max_entries);
}

/**
 * @brief Gets the total size and free space on the SD card
 * @param total_size Pointer to store total size in bytes
 * @param free_size Pointer to store free space in bytes
 * @return true if successful, false on error
 */
bool fsGetSpaceInfo(uint64_t *total_size, uint64_t *free_size)
{
  if (!sd_card_ready || !total_size || !free_size)
    return false;

  // Calculate total filesystem size
  *total_size = (uint64_t)sd_fs_info.total_sectors * sd_fs_info.bytes_per_sector;

  // Estimate free space based on free clusters
  // (This is approximate as FAT table needs to be scanned for accurate count)
  uint32_t estimated_free_clusters = sd_fs_info.cluster_count / 2; // Rough estimate
  *free_size = (uint64_t)estimated_free_clusters * sd_fs_info.bytes_per_sector * sd_fs_info.sectors_per_cluster;

  return true;
}

/**
 * @brief Gets filesystem type string
 * @return Pointer to static string describing filesystem type
 */
const char *fsGetTypeString()
{
  if (!sd_card_ready)
    return "Unknown";

  return sd_spi_get_fs_type_string(sd_fs_info.type);
}

// ============================================================================
// SETUP AT START
// ============================================================================

void setup()
{
  arduino::String resultStr;
// only wait for Serial in debug builds
#ifdef DEBUG_BUILD
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
#endif
  // Initialize all GPIO pins to safe default states before peripheral initialization
  initAllGPIOs();

  initTFTDisplay(TFT_ROTATION); // Initialize TFT display early to show debug messages
  tftPrint("Starting Pier Vibration Sensor...\n");
  char versionStr[100];
  sprintf(versionStr, "Version: %s (%s)\n", GIT_COMMIT_SHORT, GIT_COMMIT_DATE);
  tftPrint(versionStr);

  mode = 3; // Start in spectrogram mode (mode 3) which has the most SPI activity, so we can test it right away

  // Initialize networking after TFT
  netMode = initNetworking();

  debugSerialPrintf("setup(): netMode set to %d (NONE=%d, WIZNET=%d, WIFI=%d)\n", netMode, NET_NONE, NET_WIZNET, NET_WIFI);

#ifdef DEBUG_BUILD
  debugSerialPrintln("Git Version: " GIT_COMMIT_SHORT);
  debugSerialPrintln("Full Commit: " GIT_COMMIT_FULL);
  debugSerialPrintln("Commit Date: " GIT_COMMIT_DATE);
#endif

  LittleFS.begin(); // Initialize LittleFS filesystem
  tftPrint("Filesystem initialized...\n");

#if ENABLE_SD_CARD
  initSDCard(); // Initialize SD card - also initializes miniSEED logger
  tftPrint("miniSEED logger initialized (sd_spi backend)...\n");
#else
  debugSerialPrintln("SD card support disabled (ENABLE_SD_CARD=0)");
#endif

#ifdef DEBUG_BUILD
  if (logVerbosity >= LOG_DEBUG)
  {
    // Print touch calibration file content to serial for backup
    File f = LittleFS.open("/touch_calib.txt", "r");
    if (f)
    {
      debugSerialPrintln("\n=== touch_calib.txt content ===");
      while (f.available())
      {
        Serial.write(f.read());
      }
      debugSerialPrintln("\n=== End of file ===");
      f.close();
    }
    else
    {
      debugSerialPrintln("touch_calib.txt not found!");
    }
  }
#endif

  // Initialize touch controller with improved strategy from test_all
  tftPrint("---------------------------------------------\n");
  tftPrint("Initializing XPT2046 touch controller...\n");

  bool calibrationPerformed = false;

  if (!loadTouchCalibration() || forceTouchCalibration)
  {
    tftPrint("No touch calibration found or force calibration enabled, starting calibration...\n");
    // calibrateTouchController() will initialize the touchscreen
    calibrateTouchController();
    calibrationPerformed = true;
  }
  else
  {
    tftPrint("Touch calibration loaded from /touch_calib.txt...\n");
  }

  // Initialize touch controller using improved pattern from test_all
  // This should be done BEFORE other SPI devices to establish baseline
  initTouchController();
  tftPrint("Touch controller initialized...\n");

  tftPrint("---------------------------------------------\n");
  initADS1256();
  // initMPU6500();
  initICM20948();

  // Set your local timezone here (example: Central Europe)
  setLocalTimezone(LOCAL_TIMEZONE);

  // Networking already initialized above
  if (netMode == NET_WIFI)
  {
    byte mac[6];
    getWiFiMAC(mac);
    formatMACString(mac, macString);
  }
  else if (netMode == NET_WIZNET)
  {
    byte mac[6];
    getWiznetMAC(mac);
    formatMACString(mac, macString);
  }

  // Auto-correct netMode if it was reset but we have a valid network connection
  // (Same protection as in loop() - needed because of known netMode reset issue)
  debugSerialPrintf("Before NTP: netMode=%d, assignedIP=%d.%d.%d.%d\n", netMode, assignedIP[0], assignedIP[1], assignedIP[2], assignedIP[3]);

  // Check both assignedIP and Ethernet.localIP() as double protection
  IPAddress currentIP = Ethernet.localIP();
  debugSerialPrintf("Ethernet.localIP() = %d.%d.%d.%d\n", currentIP[0], currentIP[1], currentIP[2], currentIP[3]);

  if (netMode == NET_NONE && ((assignedIP[0] != 0) || (currentIP[0] != 0)))
  {
    debugSerialPrintln("WARNING: netMode was reset to NET_NONE, restoring to NET_WIZNET");
    netMode = NET_WIZNET;
    // Update assignedIP if it was cleared
    if (assignedIP[0] == 0 && currentIP[0] != 0)
    {
      assignedIP = currentIP;
    }
  }

  debugSerialPrintf("After auto-correct: netMode=%d\n", netMode);

  // NTP time sync enabled with 10-second timeout
  // Capture netMode here to avoid stack corruption inside function
  NetworkMode netModeForNTP = netMode;
  if (!setTimeFromNTP(netModeForNTP))
  {
    tftPrint("Failed to set system time from NTP server.\n");
  }

  // Server is already started in initNetworking()
  if (!serverInfoPrinted)
  {
    tftPrint("HTTP server started...\n");
    if (netMode == NET_WIFI)
    {
      resultStr = "Serving chart page at http://" + WiFi.localIP().toString() + ":" + String(HTTP_SERVER_PORT) + "\n";
    }
    else if (netMode == NET_WIZNET)
    {
      resultStr = "Serving chart page at http://" + assignedIP.toString() + ":" + String(HTTP_SERVER_PORT) + "\n";
    }
    else
    {
      resultStr = "No network connection - server not available\n";
    }
    tftPrint(resultStr.c_str());
    resultStr = "UDP stream port: " + String(UDP_STREAM_PORT) + "\n";
    tftPrint(resultStr.c_str());
    resultStr = "UDP packet size: " + String(UDP_PACKET_SIZE) + "\n";
    tftPrint(resultStr.c_str());
    tftPrint("---------------------------------------------\n");
    serverInfoPrinted = true;
  }
  setup_dsp();
  gpio_set_irq_enabled_with_callback(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true, touch_irq_isr);

  tftPrint("---------------------------------------------\n");
  tftPrint("System initialized.\n");
  tftPrint("---------------------------------------------\n");

  debugSerialPrintln("Setup complete - entering main loop");
  tftPrint("Starting Main Loop...\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop()
{
  // Auto-correct netMode if it was reset but we have a valid network connection
  if (netMode == NET_NONE && (assignedIP[0] != 0))
  {
    netMode = NET_WIZNET; // Restore to NET_WIZNET since DHCP succeeded
  }

  // ----------------------------------------------------------------------------
  // LOOP INITIALIZATION AND TIMING
  // ----------------------------------------------------------------------------
  static unsigned long loop_start_time = 0;
  static unsigned long block_start_time = 0;
  static unsigned long timing_report_interval = 5000; // Report every 5 seconds
  static unsigned long last_timing_report = 0;

  // TIMING BLOCK 1: Loop Inititalization

  loop_start_time = micros();

  // Ensure all SPI0 CS pins are deselected at loop start
  releaseSPIBus();

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

  // Debug: report touch IRQ triggers periodically
  static uint32_t last_touch_irq_count = 0;
  static unsigned long last_touch_irq_log = 0;
  if (logVerbosity >= LOG_DEBUG && (millis() - last_touch_irq_log) > 1000)
  {
    uint32_t current_count = touch_irq_count;
    if (current_count != last_touch_irq_count)
    {
      debugSerialPrintf("Touch IRQ count: %lu (delta %lu)\n",
                        (unsigned long)current_count,
                        (unsigned long)(current_count - last_touch_irq_count));
      last_touch_irq_count = current_count;
    }
    last_touch_irq_log = millis();
  }

  // ----------------------------------------------------------------------------
  // SENSOR DATA ACQUISITION (ADS1256 & ICM20948)
  // ----------------------------------------------------------------------------
  // TIMING BLOCK 2: Sensor Data Acquisition
  block_start_time = micros();
  const uint8_t myMuxList[] = {SING_0, SING_7};                                     // or any list of SING_x constants
  adcValue0 = ads.cycleSingle(myMuxList, sizeof(myMuxList) / sizeof(myMuxList[0])); // Read single-ended channel 0 (Veloc X) selecting channel 7 as next channel
  velocX = ads.convertToVoltage(adcValue0) * 1000000;                               // Convert raw ADC value to uV

  adcValue7 = ads.cycleSingle(nullptr, 0);            // Read single-ended channel 7 (Veloc Y) selecting channel 0 as next channel
  velocY = ads.convertToVoltage(adcValue7) * 1000000; // Convert raw ADC value to uV
  if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
  {
    debugSerialPrintf("TIMING: ADS1256 Read: %lu us\n", micros() - block_start_time);
  }

  // Read ICM20948 only every 5th iteration to improve overall sampling rate
  // This gives ~40 Hz for IMU data which is sufficient for slow vibrations
  icm_read_counter++;
  if (icm_read_counter >= 5)
  {
    block_start_time = micros();
    readICM20948();
    if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
    {
      debugSerialPrintf("TIMING: ICM20948 Read: %lu us\n", micros() - block_start_time);
    }
    icm_read_counter = 0;
  }

  // ----------------------------------------------------------------------------
  // DATA STORAGE AND BUFFER OPERATIONS
  // ----------------------------------------------------------------------------
  // TIMING BLOCK 3: Data Storage and Buffer Operations
  block_start_time = micros();
  velocXraw_block[block_idx] = adcValue0;
  velocYraw_block[block_idx] = adcValue7;
  velocX_block[block_idx] = velocX;
  velocY_block[block_idx] = velocY;
  velocR = sqrtf(velocX * velocX + velocY * velocY);
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

  // Record to miniSEED logger if recording is enabled (Raspberry Shake compatible format)
  // Data format: int32_t with standardized units for seismic compatibility
  if (miniSeedRecordingEnabled && sd_card_ready)
  {
    time_t now = time(nullptr);

    static uint32_t write_error_count = 0;
    static unsigned long last_error_log = 0;
    bool write_ok = true;

    // VEL channels: Store raw ADC counts (int32_t) at 500 Hz
    // Scaling: 1 count ≈ 0.596 nV per LSB (from ADS1256 datasheet with Vref=2.5V, PGA=1)
    // Raw counts preserve full ADC resolution for post-processing calibration
    if (!miniSeedLogger.recordSample("VEL_X", (int32_t)adcValue0, 500, now))
      write_ok = false;
    if (!miniSeedLogger.recordSample("VEL_Y", (int32_t)adcValue7, 500, now))
      write_ok = false;

    if (icm_read_counter == 0) // Record IMU data every 5th sample (40 Hz)
    {
      // ACC channels: Convert from m/s² to micro-g (µg) for seismic compatibility
      // Standard conversion: 1 g = 9.80665 m/s²
      // Store in units of micro-g (µg) for int32_t precision
      int32_t acc_x_ug = (int32_t)(accelData.acceleration.x * 1000000.0f / 9.80665f);
      int32_t acc_y_ug = (int32_t)(accelData.acceleration.y * 1000000.0f / 9.80665f);
      int32_t acc_z_ug = (int32_t)(accelData.acceleration.z * 1000000.0f / 9.80665f);

      if (!miniSeedLogger.recordSample("ACC_X", acc_x_ug, 100, now))
        write_ok = false;
      if (!miniSeedLogger.recordSample("ACC_Y", acc_y_ug, 100, now))
        write_ok = false;
      if (!miniSeedLogger.recordSample("ACC_Z", acc_z_ug, 100, now))
        write_ok = false;

      // GYRO channels: Convert from rad/s to milli-degrees/s (mdps)
      // Standard conversion: 1 rad/s = 57.2958 deg/s = 57295.8 mdps
      int32_t gyro_x_mdps = (int32_t)(gyroData.gyro.x * 57295.8f);
      int32_t gyro_y_mdps = (int32_t)(gyroData.gyro.y * 57295.8f);
      int32_t gyro_z_mdps = (int32_t)(gyroData.gyro.z * 57295.8f);

      if (!miniSeedLogger.recordSample("GYRO_X", gyro_x_mdps, 100, now))
        write_ok = false;
      if (!miniSeedLogger.recordSample("GYRO_Y", gyro_y_mdps, 100, now))
        write_ok = false;
      if (!miniSeedLogger.recordSample("GYRO_Z", gyro_z_mdps, 100, now))
        write_ok = false;
    }

    // Track and log write errors
    if (!write_ok)
    {
      write_error_count++;
      // Log errors every 5 seconds to avoid spam
      if (millis() - last_error_log > 5000)
      {
        debugSerialPrintf("WARNING: miniSEED write errors detected (total: %lu). Check SD card/directory.\n",
                          (unsigned long)write_error_count);
        last_error_log = millis();
      }
    }

    // Intentionally avoid periodic forced flush here.
    // High-rate channels flush on buffer-full, and recording stop triggers close/flush.
    // Avoiding synchronous 5-second flush reduces acquisition stalls and trace splitting.
  }
  else if (miniSeedRecordingEnabled && !sd_card_ready)
  {
    stopMiniSeedRecording("SD card not ready during data loop");
  }

  // Feed selected sensor data into ring buffer for display/analysis modes
  if (mode >= 0)
  {
    int current_sample = 0;
    // Use display_axis to determine which sensor to feed into ring buffer
    int sensor_select = display_axis;
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
    case RINGBUFFER_VELOCR_RAW:
      current_sample = (int16_t)sqrtf((float)(adcValue0 * adcValue0 + adcValue7 * adcValue7));
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
  if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
  {
    debugSerialPrintf("TIMING: Data Storage/Buffer: %lu us\n", micros() - block_start_time);
  }

  // ----------------------------------------------------------------------------
  // UDP STREAMING (when buffer is full)
  // ----------------------------------------------------------------------------
  if (block_idx >= UDP_PACKET_SIZE)
  {
    // TIMING BLOCK 4: UDP Streaming
    block_start_time = micros();
    if (udpStreamEnable[0])
    {
      debugSerialPrintf("Sending UDP stream for velocXraw, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("velocX", velocXraw_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[1])
    {
      debugSerialPrintf("Sending UDP stream for velocYraw, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("velocY", velocYraw_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[2])
    {
      debugSerialPrintf("Sending UDP stream for accelX, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("accelX", accelX_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[3])
    {
      debugSerialPrintf("Sending UDP stream for accelY, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("accelY", accelY_block, UDP_PACKET_SIZE);
    }
    if (udpStreamEnable[4])
    {
      debugSerialPrintf("Sending UDP stream for accelZ, count: %d\n", UDP_PACKET_SIZE);
      sendSensorUDPStream("accelZ", accelZ_block, UDP_PACKET_SIZE);
    }
    block_idx = 0;
    if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
    {
      debugSerialPrintf("TIMING: UDP Streaming: %lu us\n", micros() - block_start_time);
    }
  }

  // Release SPI bus immediately before webserver operations to prevent socket state corruption
  // This ensures both UDP streaming and subsequent TCP socket operations don't interfere
  releaseSPIBus();

  // ----------------------------------------------------------------------------
  // HTTP/WEBSERVER NETWORKING
  // ----------------------------------------------------------------------------
  if (netMode == NET_WIZNET || netMode == NET_WIFI)
  {
#ifdef TESTING_WEBSERVER
    static uint32_t simpleCounter = 0;
    static uint32_t simpleReqCount = 0;
#endif
    static uint32_t httpAcceptedClients = 0;
    static uint32_t httpSoftResyncCount = 0;
    static uint32_t httpHardResyncCount = 0;
    static uint32_t httpResyncFailStreak = 0;
    static uint32_t httpMaxNoClientLoops = 0;

    // TIMING BLOCK 5: HTTP/WEBSERVER NETWORKING
    block_start_time = micros();
    unsigned long http_accept_start = micros();
    if (netMode == NET_WIZNET)
    {
      static uint32_t wiznetNoClientLoops = 0;
      static unsigned long lastWiznetResyncMs = 0;

      if (wiznetNoClientLoops > 5000 && (millis() - lastWiznetResyncMs) > 10000)
      {
        bool useHardReset = (httpResyncFailStreak >= 2);

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintf("HTTP: WIZNET %s resync after %lu no-client loops\n",
                            useHardReset ? "HARD" : "SOFT",
                            (unsigned long)wiznetNoClientLoops);
        }

        releaseSPIBus();
        if (useHardReset)
        {
          wiznet5k_reset();
          httpHardResyncCount++;
        }
        else
        {
          httpSoftResyncCount++;
        }
        wiznet_init_spi();
        Ethernet.init(WIZNET_SPI_CS);
        ethServer.begin();

        httpResyncFailStreak++;
        lastWiznetResyncMs = millis();
        wiznetNoClientLoops = 0;
      }

      EthernetClient client = ethServer.available();
      if (client)
      {
        wiznetNoClientLoops = 0;
        httpAcceptedClients++;
        httpResyncFailStreak = 0;

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: Ethernet client accepted");
        }

        client.setTimeout(250);

        if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
        {
          debugSerialPrintf("TIMING: HTTP Accept: %lu us\n", micros() - http_accept_start);
        }

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: Ethernet client connected");
        }

        // Request handling pattern analogous to test_wiznet_counter_button
        unsigned long http_parse_start = micros();
        String req = client.readStringUntil('\r');
        client.readStringUntil('\n'); // Skip rest of request line
        req.trim();

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintf("HTTP: Ethernet request line: '%s'\n", req.c_str());
        }

        unsigned long http_header_read_start = micros();
        while (client.available() && client.readStringUntil('\n') != "\r")
          ;

        if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
        {
          debugSerialPrintf("TIMING: HTTP Header Read: %lu us\n", micros() - http_header_read_start);
        }

#ifdef TESTING_WEBSERVER
        simpleReqCount++;
        if (req.indexOf("GET /increment") != -1)
        {
          simpleCounter++;
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintf("Simple counter incremented to: %lu\n", (unsigned long)simpleCounter);
          }
        }
        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: Ethernet route -> simple counter page");
        }
        serveSimpleCounterPage(client, simpleCounter, simpleReqCount);
#else
        unsigned long http_handler_start = micros();
        if (req.indexOf("GET /data") == 0)
        {
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintln("HTTP: Ethernet route -> /data");
          }
          serveSensorData(client);
        }
        else if (req.indexOf("GET /udpstream") == 0)
        {
          debugSerialPrintln("HTTP: /udpstream");
          serveUDPStreamControl(client, req);
        }
        else if (req.indexOf("GET /setmode") == 0)
        {
          debugSerialPrintln("HTTP: /setmode");
          serveModeControl(client, req);
        }
        else if (req.indexOf("GET /setaxis") == 0)
        {
          debugSerialPrintln("HTTP: /setaxis");
          serveAxisControl(client, req);
        }
        else if (req.indexOf("GET /setloglevel") == 0)
        {
          debugSerialPrintln("HTTP: /setloglevel");
          serveSetLogLevel(client, req);
        }
        else if (req.indexOf("GET /touchcalibrate") == 0)
        {
          debugSerialPrintln("HTTP: /touchcalibrate");
          serveTouchCalibrate(client, req);
        }
        else if (req.indexOf("GET /touchcalibdata") == 0)
        {
          debugSerialPrintln("HTTP: /touchcalibdata");
          serveTouchCalibData(client, req);
        }
        else if (req.indexOf("GET /miniseed") == 0)
        {
          debugSerialPrintln("HTTP: /miniseed");
          serveMinISeedControl(client, req);
        }
        else if (req.indexOf("GET /listmseed") == 0)
        {
          debugSerialPrintln("HTTP: /listmseed");
          serveMseedFileList(client, req);
        }
        else if (req.indexOf("GET /mseedfile") == 0)
        {
          debugSerialPrintln("HTTP: /mseedfile");
          serveMseedFile(client, req);
        }
        else if (req.indexOf("GET /mseedviewer") == 0)
        {
          debugSerialPrintln("HTTP: /mseedviewer");
          serveMseedViewer(client);
        }
        else if (req.indexOf("GET /sdunmount") == 0)
        {
          debugSerialPrintln("HTTP: /sdunmount");
          serveSDUnmount(client, req);
        }
        else if (req.indexOf("GET /highcharts") == 0)
        {
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintln("HTTP: /highcharts");
          }
          serveHighchartsStatus(client, req);
        }
        else
        {
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintln("HTTP: Ethernet route -> chart page");
          }
          serveChartPage(client);
        }

        if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
        {
          debugSerialPrintf("TIMING: HTTP Handler: %lu us\n", micros() - http_handler_start);
          debugSerialPrintf("TIMING: HTTP Parse+Process: %lu us\n", micros() - http_parse_start);
        }
#endif

        delay(10);
        client.stop();
        releaseSPIBus();
        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: Ethernet client closed");
        }
      }
      else
      {
        wiznetNoClientLoops++;
        if (wiznetNoClientLoops > httpMaxNoClientLoops)
        {
          httpMaxNoClientLoops = wiznetNoClientLoops;
        }
      }
    }
    else if (netMode == NET_WIFI)
    {
      WiFiClient client = wifiServer.accept();
      if (client)
      {
        httpAcceptedClients++;

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: WiFi client accepted");
        }

        client.setTimeout(250);

        if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
        {
          debugSerialPrintf("TIMING: HTTP Accept: %lu us\n", micros() - http_accept_start);
        }

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: WiFi client connected");
        }

        // Request handling pattern analogous to test_wiznet_counter_button
        unsigned long http_parse_start = micros();
        String req = client.readStringUntil('\r');
        client.readStringUntil('\n'); // Skip rest of request line
        req.trim();

        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintf("HTTP: WiFi request line: '%s'\n", req.c_str());
        }

        unsigned long http_header_read_start = micros();
        while (client.available() && client.readStringUntil('\n') != "\r")
          ;

        if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
        {
          debugSerialPrintf("TIMING: HTTP Header Read: %lu us\n", micros() - http_header_read_start);
        }

#ifdef TESTING_WEBSERVER
        simpleReqCount++;
        if (req.indexOf("GET /increment") != -1)
        {
          simpleCounter++;
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintf("Simple counter incremented to: %lu\n", (unsigned long)simpleCounter);
          }
        }
        if (logVerbosity >= LOG_DEBUG)
        {
          debugSerialPrintln("HTTP: WiFi route -> simple counter page");
        }
        serveSimpleCounterPage(client, simpleCounter, simpleReqCount);
#else
        unsigned long http_handler_start = micros();
        if (req.indexOf("GET /data") == 0)
        {
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintln("HTTP: WiFi route -> /data");
          }
          serveSensorData(client);
        }
        else if (req.indexOf("GET /udpstream") == 0)
        {
          debugSerialPrintln("HTTP: /udpstream");
          serveUDPStreamControl(client, req);
        }
        else if (req.indexOf("GET /setmode") == 0)
        {
          debugSerialPrintln("HTTP: /setmode");
          serveModeControl(client, req);
        }
        else if (req.indexOf("GET /setaxis") == 0)
        {
          debugSerialPrintln("HTTP: /setaxis");
          serveAxisControl(client, req);
        }
        else if (req.indexOf("GET /setloglevel") == 0)
        {
          debugSerialPrintln("HTTP: /setloglevel");
          serveSetLogLevel(client, req);
        }
        else if (req.indexOf("GET /touchcalibrate") == 0)
        {
          debugSerialPrintln("HTTP: /touchcalibrate");
          serveTouchCalibrate(client, req);
        }
        else if (req.indexOf("GET /touchcalibdata") == 0)
        {
          debugSerialPrintln("HTTP: /touchcalibdata");
          serveTouchCalibData(client, req);
        }
        else if (req.indexOf("GET /miniseed") == 0)
        {
          debugSerialPrintln("HTTP: /miniseed");
          serveMinISeedControl(client, req);
        }
        else if (req.indexOf("GET /listmseed") == 0)
        {
          debugSerialPrintln("HTTP: /listmseed");
          serveMseedFileList(client, req);
        }
        else if (req.indexOf("GET /mseedfile") == 0)
        {
          debugSerialPrintln("HTTP: /mseedfile");
          serveMseedFile(client, req);
        }
        else if (req.indexOf("GET /mseedviewer") == 0)
        {
          debugSerialPrintln("HTTP: /mseedviewer");
          serveMseedViewer(client);
        }
        else if (req.indexOf("GET /sdunmount") == 0)
        {
          debugSerialPrintln("HTTP: /sdunmount");
          serveSDUnmount(client, req);
        }
        else if (req.indexOf("GET /highcharts") == 0)
        {
          if (logVerbosity >= LOG_DEBUG)
          {
            debugSerialPrintln("HTTP: /highcharts");
          }
          serveHighchartsStatus(client, req);
        }
        else
        {
          if (logVerbosity >= LOG_INFO)
          {
            debugSerialPrintln("HTTP: WiFi route -> chart page");
          }
          serveChartPage(client);
        }

        if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
        {
          debugSerialPrintf("TIMING: HTTP Handler: %lu us\n", micros() - http_handler_start);
          debugSerialPrintf("TIMING: HTTP Parse+Process: %lu us\n", micros() - http_parse_start);
        }
#endif

        delay(10);
        client.stop();
        releaseSPIBus();
        if (logVerbosity >= LOG_INFO)
        {
          debugSerialPrintln("HTTP: WiFi client closed");
        }
      }
    }

    if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
    {
      debugSerialPrintf("TIMING: HTTP Networking: %lu us\n", micros() - block_start_time);
      debugSerialPrintf("HTTP Stats: accepted=%lu softResync=%lu hardResync=%lu maxNoClientLoops=%lu\n",
                        (unsigned long)httpAcceptedClients,
                        (unsigned long)httpSoftResyncCount,
                        (unsigned long)httpHardResyncCount,
                        (unsigned long)httpMaxNoClientLoops);
    }
  }

  // ----------------------------------------------------------------------------------
  // TOUCH INPUTE CONTROL
  // Handle touch input for mode changes and menu interactions, with timing measurement
  // ----------------------------------------------------------------------------------
  // TIMING BLOCK 6: TOUCH INPUT
  unsigned long touch_input_start = micros();
  bool mode_changed = handleTouchInput(mode);
  if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
  {
    debugSerialPrintf("TIMING: Touch Input: %lu us\n", micros() - touch_input_start);
  }
  // ----------------------------------------------------------------------------
  // MODE CONTROL
  // Update display mode based on touch input or webpage selection and manage
  // mode-specific initialization and state
  // ----------------------------------------------------------------------------
  // TIMING BLOCK 7: MODE CONTROL
  unsigned long mode_control_start = micros();
  if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
  {
    debugSerialPrintf("TIMING: Mode Control: %lu us\n", micros() - mode_control_start);
  }
  if (mode_changed)
  {
    // Reset mode-specific flags when mode changes
    mode_initialized = false;
  }

  // Track mode transitions for display control
  static int previous_mode = 3; // Initialize to default mode

  // Handle touch calibration trigger from web interface
  if (trigger_touch_calibration)
  {
    trigger_touch_calibration = false;
    debugPrintf("Starting touch calibration with %d points...\\n", NUM_CALIB_POINTS);
    calibrateTouchController();
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    // Reset display mode flags
    mode_initialized = false;
  }

  // Mode has changed, reset scrolling and manage display power based on mode
  if (mode != previous_mode)
  {
    scrollDisplay(0); // Reset scrolling
    if (mode == -1)
    {
      // Entering mode -1: turn off display
      debugPrintln("Switching to mode -1: turning off display");
      tft.fillScreen(TFT_BLACK);
      disableBacklight();
    }
    else
    {
      // Leaving mode -1: turn on display
      unsigned long backlight_start = micros();
      debugPrintf("Leaving mode -1, switching to mode %d: turning on display\n", mode);
      /*     setBacklight(1.0); // Use full brightness first to ensure GPIO is set
          delay(10);
          setBacklight(0.25); // Then set to desired brightness
       */
      enableBacklight();
      if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
      {
        debugSerialPrintf("TIMING: Backlight Setup: %lu us\n", micros() - backlight_start);
      }
    }
  }

  previous_mode = mode;

  // ----------------------------------------------------------------------------
  // DISPLAY RENDERING AND UPDATE
  // ----------------------------------------------------------------------------
  if (mode != -1)
  {
    // TIMING BLOCK 8: Display Updates
    block_start_time = micros();
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

        // Output statistics with timing information (in yellow) if log level is DEBUG
        if (logVerbosity >= LOG_DEBUG)
        {
          debugPrintfColor(TFT_YELLOW, "Event: out=%lums in=%lums\n", crossing_time_out, crossing_time_in);
          debugPrintfColor(TFT_YELLOW, "Duration=%lums samples=%d\n", crossing_time_in - crossing_time_out, n);
          debugPrintfColor(TFT_YELLOW, "Mean=%d RMS=%d\n", sum / n, (int)sqrtf(((float)sumsq) / n));
        }
      }
    }

    if (mode == 0)
    {
      // Mode 0: Statistics text mode
      if (!mode_initialized)
      {
        tft.fillScreen(TFT_BLACK);
        disp_column = 0;         // Reset scroll position tracking
        scrollDisplay(0);        // Reset scrolling
        saved_scroll_offset = 0; // Keep menu/icon anchored
        tft.setTextColor(TFT_GREEN);
        tft.setTextSize(TFT_TEXT_SIZE);
        tft.setCursor(0, 0);
        drawMenuIndicator(disp_column); // Draw menu access indicator at top-left
        mode_initialized = true;
      }

      // Display statistics periodically
      static unsigned long last_stats_update = 0;
      if (millis() - last_stats_update >= 1000 / TEXT_DISPLAY_REFRESH_RATE)
      {
        char buf[128];

        // Prepare text area to overwrite instead of scrolling
        text_block_refresh = true;

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
      if (!mode_initialized)
      {
        scrollDisplay(0); // Reset scrolling
        setup_waveform();
        drawMenuIndicator(disp_column); // Draw menu access indicator
        mode_initialized = true;
      }

      // Throttle display updates to ~10 Hz to allow fast data acquisition
      static unsigned long last_display_update_mode1 = 0;
      if (millis() - last_display_update_mode1 >= 1000 / TFT_DISPLAY_REFRESH_RATE)
      {
        // Get data for display
        buffer_get_latest(input_shorts_buf, waveform_w, 0);

        waveform_display(input_shorts_buf, waveform_w, TFT_YELLOW);

        last_display_update_mode1 = millis();
      }
    }
    else if (mode == 2)
    {
      // Clear screen once when entering FFT mode and draw frequency axis
      if (!mode_initialized)
      {
        tft.fillScreen(TFT_BLACK);
        disp_column = SCROLL_TOP_FIXED_AREA; // Reset spectrogram column position
        scrollDisplay(0);                    // Reset scrolling
        draw_fft_frequency_axis();           // Draw frequency labels for FFT mode
        drawMenuIndicator(disp_column);      // Draw menu access indicator
        mode_initialized = true;
      }

      // Throttle display updates to ~10 Hz to allow fast data acquisition
      static unsigned long last_display_update_mode2 = 0;
      if (millis() - last_display_update_mode2 >= 1000 / TFT_DISPLAY_REFRESH_RATE)
      {
        // Get data for display
        buffer_get_latest(input_shorts_buf, FFT_SIZE, /* lshift_bits */ 0);

        // Convert shorts to float with proper scaling for ADC values
        for (int i = 0; i < FFT_SIZE; i++)
        {
          input_buf[i] = (float)input_shorts_buf[i];
        }

        int npts = magnitude_spectrum(input_buf, mag_spec, mag_spec_len);

        sample_to_short(mag_spec, mag_spec_short, npts);
        waveform_display(mag_spec_short, npts, TFT_YELLOW, false); // Don't draw triggers in FFT mode

        last_display_update_mode2 = millis();
      }
    }
    else if (mode == 3)
    {
      // Clear screen once when entering spectrogram mode
      if (!mode_initialized)
      {
        tft.fillScreen(TFT_BLACK);
        disp_column = SCROLL_TOP_FIXED_AREA; // Reset spectrogram column position
        scrollDisplay(0);                    // Reset scrolling
        drawMenuIndicator(disp_column);      // Draw menu access indicator
        mode_initialized = true;
      }

      // Throttle display updates to ~10 Hz to allow fast data acquisition
      static unsigned long last_display_update_mode3 = 0;
      if (millis() - last_display_update_mode3 >= 1000 / TFT_DISPLAY_REFRESH_RATE)
      {
        // Get data for display
        buffer_get_latest(input_shorts_buf, FFT_SIZE, /* lshift_bits */ 0);

        // Convert shorts to float with proper scaling for ADC values
        for (int i = 0; i < FFT_SIZE; i++)
        {
          input_buf[i] = (float)input_shorts_buf[i];
        }

        int npts = magnitude_spectrum(input_buf, mag_spec, mag_spec_len);

        sample_to_short(mag_spec, mag_spec_short, npts);
        add_display_column(mag_spec_short, npts);

        last_display_update_mode3 = millis();
      }
    }
    if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
    {
      debugSerialPrintf("TIMING: Display Updates: %lu us\n", micros() - block_start_time);
    }
  }

  // ----------------------------------------------------------------------------
  // LOOP TIMING AND REPORTING
  // ----------------------------------------------------------------------------
  // TIMING REPORT: Total loop time
  if (logVerbosity >= LOG_INFO && (millis() - last_timing_report) > timing_report_interval)
  {
    debugSerialPrintf("------\n");
    debugSerialPrintf("TIMING: Total Loop: %lu us\n", micros() - loop_start_time);
    debugSerialPrintf("======\n");
    last_timing_report = millis();
  }

  // Release SPI bus at end of loop after all display operations to prevent socket corruption
  releaseSPIBus();
}
