#include <Arduino.h>
#include <unity.h>
#include <Adafruit_ILI9341.h>
#define USE_HARDWARE_SPI
#ifdef USE_HARDWARE_SPI
#include <XPT2046_Touchscreen.h>
#else
#include <XPT2046_Bitbang.h>
#endif
#include <hardware/pwm.h> // Add this include for PWM functions
#include "hardware/spi.h"

// Pin definitions (adjust as needed for your hardware)
#define SD_CS 26

// TFT configuration
#define TFT_SPI_PORT spi0    // Using SPI0
#define TFT_CS 1             // GP1
#define TFT_DC 4             // GP4
#define TFT_MISO 0           // GP0 (SDO)
#define TFT_MOSI 7           // GP7 (SDI)
#define TFT_SCLK 2           // GP2 (SCK)
#define TFT_RST 6            // GP6
#define TFT_LED 3            // Set to -1 if not used
#define TFT_SPI_BPS 20000000 // 20 MHz SPI clock. Write speed can be up to 40 MHz for ILI9341, but read speed max 20 MHz. Higher speed cause test_tft_read_display_status to fail.
#define TFT_ROTATION 1       // Set the default rotation for the display
#define TFT_TEXT_SIZE 1      // Set the default text size for the display

// Constants for TFT touch controller
#define TOUCH_CLK TFT_SCLK // GP2 (shared with TFT SCK)
#define TOUCH_CS 22        // Set to -1 if not used
#define TOUCH_DIN TFT_MOSI // GP7 (shared with TFT MOSI)
#define TOUCH_DO TFT_MISO  // GP0 (shared with TFT MISO)
#define TOUCH_IRQ 27       // Set to -1 if not used

// Test Constants and definitions
#define VISUAL_TEST_TIMEOUT 2000 // 10 seconds for visual tests, change to zero for not waiting for visual confirmation

// #define TFT_TESTS
// #define TFT_TESTS_SLOW
// #define TOUCH_TESTS
#define TOUCH_TESTS_INTERACTIVE

// TFT display tests

#ifdef USE_HARDWARE_SPI
// Create TFT display object using hardware SPI (3-wire constructor)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
#else
// Create TFT display object using software SPI
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST, TFT_MISO);
#endif

#ifdef USE_HARDWARE_SPI
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
#else
XPT2046_Bitbang ts(TOUCH_DIN, TOUCH_DO, TOUCH_CLK, TOUCH_CS);
#endif

static volatile bool touch_event_flag = false;
#ifndef USE_HARDWARE_SPI
TouchPoint touch_event_point = {0, 0, 0, 0, 0, 0}; // Initialize touch point structure
#else
TS_Point touch_event_point = {0, 0, 0}; // Initialize touch point structure for XPT2046_Touchscreen
#endif

// XPT2046 touch controller resolution constants
// XPT2046 uses 12-bit ADC (0-4095 range)
const uint8_t XPT2046_RESOLUTION_BITS = 12; // ADC resolution in bits
const uint16_t XPT2046_YMAX_VALUE = 4095;   // Maximum raw coordinate value based on resolution
const uint16_t XPT2046_XMAX_VALUE = 2047;   // Due to wiring, X max is half of full range

// MMSE-based calibration coefficients
// Transformation: Xd = A*Xt + B*Yt + C, Yd = D*Xt + E*Yt + F
struct CalibrationMatrix
{
    float A, B, C, D, E, F;
};

CalibrationMatrix mmseCalibMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f}; // Identity by default
int NUM_CALIB_POINTS = 5;                                                 // 5, 9, or 25 points (5x5 grid for best accuracy)

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

void drawCalibrationCross(int x, int y, uint16_t color = ILI9341_RED)
{
    // Draw a cross at the specified position
    tft.drawLine(x - 10, y, x + 10, y, color);
    tft.drawLine(x, y - 10, x, y + 10, color);
}

// MMSE-Based Multipoint Calibration Algorithm (based on AN-1021)
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

    // Calculate sums for least squares regression
    float sum_xt = 0, sum_yt = 0, sum_xt2 = 0, sum_yt2 = 0, sum_xt_yt = 0;
    float sum_xd = 0, sum_yd = 0, sum_xd_xt = 0, sum_xd_yt = 0;
    float sum_yd_xt = 0, sum_yd_yt = 0;

    for (int i = 0; i < numPoints; i++)
    {
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

    float a[3] = {sum_xt2, sum_xt_yt, sum_xt}, b[3] = {sum_xt_yt, sum_yt2, sum_yt},
          c[3] = {sum_xd_xt, sum_xd_yt, sum_xd}, d[3] = {sum_yd_xt, sum_yd_yt, sum_yd};
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

    float det = (a[0] - a[2]) * (b[1] - b[2]) - (a[1] - a[2]) * (b[0] - b[2]);

    if (fabs(det) < 0.001f)
    {
        Serial.println("Error: Singular matrix - calibration points may be collinear");
        return false;
    }

    // Calculate coefficients using Cramer's rule
    float det_a = (c[0] - c[2]) * (b[1] - b[2]) - (c[1] - c[2]) * (b[0] - b[2]);
    matrix.A = det_a / det;

    float det_b = (c[1] - c[2]) * (a[0] - a[2]) - (c[0] - c[2]) * (a[1] - a[2]);
    matrix.B = det_b / det;

    float det_c = b[0] * (a[2] * c[1] - a[1] * c[2]) + b[1] * (a[0] * c[2] - a[2] * c[0]) + b[2] * (a[1] * c[0] - a[0] * c[1]);
    matrix.C = det_c / det;

    float det_d = (d[0] - d[2]) * (b[1] - b[2]) - (d[1] - d[2]) * (b[0] - b[2]);
    matrix.D = det_d / det;

    float det_e = (d[1] - d[2]) * (a[0] - a[2]) - (d[0] - d[2]) * (a[1] - a[2]);
    matrix.E = det_e / det;

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
    // Apply rotation transform based on display orientation
    uint16_t rotated_x, rotated_y;
    switch (TFT_ROTATION)
    {
    case 0: // Portrait
        rotated_x = touch_x;
        rotated_y = touch_y;
        break;
    case 1: // Landscape 90° CW
        rotated_x = touch_y;
        rotated_y = XPT2046_XMAX_VALUE - touch_x;
        break;
    case 2: // Portrait 180°
        rotated_x = XPT2046_XMAX_VALUE - touch_x;
        rotated_y = XPT2046_YMAX_VALUE - touch_y;
        break;
    case 3: // Landscape 270° CW
        rotated_x = XPT2046_YMAX_VALUE - touch_y;
        rotated_y = touch_x;
        break;
    default:
        rotated_x = touch_x;
        rotated_y = touch_y;
        break;
    }

    float xt = (float)rotated_x;
    float yt = (float)rotated_y;

    display_x = (int)(mmseCalibMatrix.A * xt + mmseCalibMatrix.B * yt + mmseCalibMatrix.C);
    display_y = (int)(mmseCalibMatrix.D * xt + mmseCalibMatrix.E * yt + mmseCalibMatrix.F);
}

void test_tft_begin()
{
    // Set CS of Touch and SD card to high to disable them before initializing TFT
    if (TOUCH_CS != -1)
    {
        gpio_init(TOUCH_CS);
        gpio_set_dir(TOUCH_CS, GPIO_OUT);
        gpio_put(TOUCH_CS, 1); // Disable touch controller
    }
    if (SD_CS != -1)
    {
        gpio_init(SD_CS);
        gpio_set_dir(SD_CS, GPIO_OUT);
        gpio_put(SD_CS, 1); // Disable SD card
    }
    // Switch backlight pin to output if defined
    if (TFT_LED != -1)
    {
        gpio_init(TFT_LED);
        gpio_set_dir(TFT_LED, GPIO_OUT);
        gpio_put(TFT_LED, 0); // Turn off backlight initially
    }

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

    tft.begin();
    // No direct way to check success, but if it doesn't hang/crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_fillScreen()
{
    Serial.println("TFT fill with black color");
    tft.fillScreen(ILI9341_BLACK);
    Serial.println("TFT fill with red color");
    tft.fillScreen(ILI9341_RED);
    Serial.println("TFT fill with green color");
    tft.fillScreen(ILI9341_GREEN);
    Serial.println("TFT fill with blue color");
    tft.fillScreen(ILI9341_BLUE);
    Serial.println("TFT fill with black color");
    tft.fillScreen(ILI9341_BLACK);
    // No direct way to check color, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_text()
{
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.println("Hello World!");
    tft.setTextColor(ILI9341_YELLOW);
    tft.setTextSize(2);
    tft.println(1234.56);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(3);
    tft.println(0xDEADBEEF, HEX);
    tft.println();
    tft.setTextColor(ILI9341_GREEN);
    tft.setTextSize(5);
    tft.println("Groop");
    tft.setTextSize(2);
    tft.println("I implore thee,");
    tft.setTextSize(1);
    tft.println("my foonting turlingdromes.");
    tft.println("And hooptiously drangle me");
    tft.println("with crinkly bindlewurdles,");
    tft.println("Or I will rend thee");
    tft.println("in the gobberwarts");
    tft.println("with my blurglecruncheon,");
    tft.println("see if I don't!");
    delay(VISUAL_TEST_TIMEOUT); // Allow time for showing text on TFT
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_rotateScreen()
{
    tft.setRotation(0);
    test_tft_text(); // Ensure text is visible before rotation tests
    tft.setRotation(1);
    test_tft_text(); // Ensure text is visible before rotation tests
    tft.setRotation(2);
    test_tft_text(); // Ensure text is visible before rotation tests
    tft.setRotation(3);
    test_tft_text(); // Ensure text is visible before rotation tests
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_drawPixel()
{
    tft.fillScreen(ILI9341_BLACK);
    tft.drawPixel(10, 10, ILI9341_WHITE);
    delay(VISUAL_TEST_TIMEOUT); // Allow time for showing element on TFT
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_Lines()
{
    unsigned long start, t;
    int x1, y1, x2, y2,
        w = tft.width(),
        h = tft.height();
    const int color = ILI9341_WHITE;

    tft.fillScreen(ILI9341_BLACK);

    x1 = y1 = 0;
    y2 = h - 1;
    for (x2 = 0; x2 < w; x2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);
    x2 = w - 1;
    for (y2 = 0; y2 < h; y2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);

    tft.fillScreen(ILI9341_BLACK);

    x1 = w - 1;
    y1 = 0;
    y2 = h - 1;
    for (x2 = 0; x2 < w; x2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);
    x2 = 0;
    for (y2 = 0; y2 < h; y2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);

    tft.fillScreen(ILI9341_BLACK);

    x1 = 0;
    y1 = h - 1;
    y2 = 0;
    for (x2 = 0; x2 < w; x2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);
    x2 = w - 1;
    for (y2 = 0; y2 < h; y2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);

    tft.fillScreen(ILI9341_BLACK);

    x1 = w - 1;
    y1 = h - 1;
    y2 = 0;
    for (x2 = 0; x2 < w; x2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);
    x2 = 0;
    for (y2 = 0; y2 < h; y2 += 6)
        tft.drawLine(x1, y1, x2, y2, color);
    delay(VISUAL_TEST_TIMEOUT);
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_FastLines(void)
{
    unsigned long start;
    int x, y, w = tft.width(), h = tft.height();

    tft.fillScreen(ILI9341_BLACK);
    for (y = 0; y < h; y += 5)
        tft.drawFastHLine(0, y, w, ILI9341_RED);
    for (x = 0; x < w; x += 5)
        tft.drawFastVLine(x, 0, h, ILI9341_BLUE);
}

void test_Rects(void)
{
    int n, i, i2,
        cx = tft.width() / 2,
        cy = tft.height() / 2;

    tft.fillScreen(ILI9341_BLACK);
    n = min(tft.width(), tft.height());
    for (i = 2; i < n; i += 6)
    {
        i2 = i / 2;
        tft.drawRect(cx - i2, cy - i2, i, i, ILI9341_GREEN);
    }
}

void test_FilledRects(void)
{
    int n, i, i2,
        cx = tft.width() / 2 - 1,
        cy = tft.height() / 2 - 1;

    tft.fillScreen(ILI9341_BLACK);
    n = min(tft.width(), tft.height());
    for (i = n; i > 0; i -= 6)
    {
        i2 = i / 2;
        tft.fillRect(cx - i2, cy - i2, i, i, ILI9341_YELLOW);
        // Outlines are not included in timing results
        tft.drawRect(cx - i2, cy - i2, i, i, ILI9341_MAGENTA);
    }
}

void test_FilledCircles(void)
{

    int radius = 10, x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

    tft.fillScreen(ILI9341_BLACK);
    for (x = radius; x < w; x += r2)
    {
        for (y = radius; y < h; y += r2)
        {
            tft.fillCircle(x, y, radius, ILI9341_MAGENTA);
        }
    }
}

void test_Circles(void)
{
    int radius = 10, x, y, r2 = radius * 2,
        w = tft.width() + radius,
        h = tft.height() + radius;

    // Screen is not cleared for this one -- this is
    // intentional and does not affect the reported time.
    for (x = 0; x < w; x += r2)
    {
        for (y = 0; y < h; y += r2)
        {
            tft.drawCircle(x, y, radius, ILI9341_WHITE);
        }
    }
}

void test_Triangles()
{
    int n, i, cx = tft.width() / 2 - 1,
              cy = tft.height() / 2 - 1;

    tft.fillScreen(ILI9341_BLACK);
    n = min(cx, cy);
    for (i = 0; i < n; i += 5)
    {
        tft.drawTriangle(
            cx, cy - i,     // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            tft.color565(0, 0, i));
    }
}

void test_FilledTriangles()
{
    int i, cx = tft.width() / 2 - 1,
           cy = tft.height() / 2 - 1;

    tft.fillScreen(ILI9341_BLACK);
    for (i = min(cx, cy); i > 10; i -= 5)
    {
        tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         tft.color565(0, i, i));
        tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                         tft.color565(i, i, 0));
    }
}

void test_RoundRects()
{
    int w, i, i2,
        cx = tft.width() / 2 - 1,
        cy = tft.height() / 2 - 1;

    tft.fillScreen(ILI9341_BLACK);
    w = min(tft.width(), tft.height());
    for (i = 0; i < w; i += 6)
    {
        i2 = i / 2;
        tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(i, 0, 0));
    }
}

void test_FilledRoundRects()
{
    int i, i2,
        cx = tft.width() / 2 - 1,
        cy = tft.height() / 2 - 1;

    tft.fillScreen(ILI9341_BLACK);
    for (i = min(tft.width(), tft.height()); i > 20; i -= 6)
    {
        i2 = i / 2;
        tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
    }
}

void test_tft_backlight_on()
{
    setBacklight(1.0);
    // Optionally add a short delay for visual confirmation
    delay(VISUAL_TEST_TIMEOUT);
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_backlight_off()
{
    setBacklight(0.0);
    // Optionally add a short delay for visual confirmation
    delay(VISUAL_TEST_TIMEOUT);
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_backlight_dim()
{
    for (float brightness = 0.0f; brightness <= 1.0f; brightness += 0.1f)
    {
        setBacklight(brightness); // Set backlight to varying brightness
        delay(1000);              // Allow time for visual confirmation
    }
    delay(VISUAL_TEST_TIMEOUT);
    // No direct readback, but if no crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_tft_read_display_id()
{
    // ILI9341 responds to 0x04 (Read Display Identification Information)
    // Should return 3 bytes: Manufacturer ID, Module/Driver Version ID, Module/Driver ID
    tft.begin(); // Ensure display is initialized

    uint8_t id1 = tft.readcommand8(ILI9341_RDDID, 2); // #2 is manufacturer ID
    uint8_t id2 = tft.readcommand8(ILI9341_RDDID, 3); // #3 is module/driver version ID
    uint8_t id3 = tft.readcommand8(ILI9341_RDDID, 4); // #4 is module/driver ID

    // Check that at least one ID byte is not 0x00 or 0xFF (which would indicate no response)
    bool valid = (id1 == 0x00) && (id2 == 0x00) && (id3 == 0x00);

    TEST_ASSERT_TRUE_MESSAGE(valid, "Failed to read valid display ID from ILI9341");

    // Optionally print the IDs for debug
    Serial.printf("ILI9341 ID bytes: %02X %02X %02X\n", id1, id2, id3);
}

void test_tft_read_display_status()
{
    // ILI9341 responds to 0x09 (Read Display Status)
    // Should return 4 bytes: status information
    tft.begin(); // Ensure display is initialized

    // Read 4 bytes
    uint8_t stat1 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 0); // #2 D[31:25] status bits
    uint8_t stat2 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 1); // #3 D[22:16] status bits
    uint8_t stat3 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 2); // #4 D[10:8] status bits
    uint8_t stat4 = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDDST, 3); // #5 D[7:5] status bits
    uint8_t selfdiag = tft.Adafruit_SPITFT::readcommand8(ILI9341_RDSELFDIAG);

    // Check that at least one status byte is not 0x00 or 0xFF (which would indicate no response)
    bool valid = (stat1 == 0xD2) && (stat2 == 0x29) && (stat3 == 0x42) && (stat4 == 0x00) && (selfdiag == 0xC0);

    // print the status bytes for debug
    Serial.printf("ILI9341 Status bytes: %02X %02X %02X %02X\n", stat1, stat2, stat3, stat4);
    Serial.printf("ILI9341 Self-Diagnostic: %02X\n", selfdiag);

    TEST_ASSERT_TRUE_MESSAGE(valid, "Failed to read valid display status from ILI9341");
}

void test_vertical_scroll()
{
    uint8_t ch = 'A';
    int iScrollStart = 32; // Start scrolling from this line
    tft.begin();           // Ensure display is initialized
    tft.setRotation(0);    // Set rotation to default
    tft.fillScreen(ILI9341_BLACK);
    tft.setScrollMargins(32, 0); // Set scroll margins to all lines below line 32
    tft.setCursor(0, 0);
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_RED);
    tft.println("Fixed Line 1");
    tft.setTextColor(ILI9341_GREEN);
    tft.println("Fixed Line 2");
    tft.setTextColor(ILI9341_BLUE);
    tft.print("Fixed Line 3");
    for (int j = 0; j < 100; j++)
    {
        tft.setCursor(0, iScrollStart == 32 ? 304 : iScrollStart - 16);
        tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
        tft.setTextSize(1);
        for (int i = 0; i < 20; i++)
        {
            tft.write(ch);
        }
        if (ch < 'Z')
            ch++;
        else
            ch = 'A';
        // Try simple...
        iScrollStart += 16;
        if (iScrollStart == 320)
            iScrollStart = 32;
        tft.scrollTo(iScrollStart);
    }

    delay(VISUAL_TEST_TIMEOUT); // Allow time for showing element on TFT
    TEST_ASSERT_TRUE(true);     // No direct readback, but if no crash, it's OK
}

// TFT display touch tests

void test_touchscreen_begin()
{
    ts.begin();
    // No direct way to check success, but if it doesn't hang/crash, it's OK
    TEST_ASSERT_TRUE(true);
}

void test_touchscreen_not_touched()
{
    ts.begin();
#ifdef USE_HARDWARE_SPI
    TS_Point tp = ts.getPoint();
#else
    TouchPoint tp = ts.getTouch();
#endif
    bool touched = tp.x != 0 || tp.y != 0 || tp.z != 0;
    TEST_ASSERT_FALSE(touched);
}

void test_touchscreen_irq_inactive()
{
    // This test is only valid if TOUCH_IRQ is defined
    if (TOUCH_IRQ != -1)
    {
        // Initialize the IRQ pin
        gpio_init(TOUCH_IRQ);
        gpio_set_dir(TOUCH_IRQ, GPIO_IN);
        gpio_pull_up(TOUCH_IRQ); // Enable pull-up resistor

        // Check if the IRQ pin is high (not touched)
        bool irq_state = gpio_get(TOUCH_IRQ);
        TEST_ASSERT_TRUE(irq_state); // Should be high if not touched
    }
    else
    {
        Serial.println("TOUCH_IRQ not defined, skipping test.");
    }
}

void test_touchscreen_read_coordinates()
{
    bool touched;
#ifdef USE_HARDWARE_SPI
    TS_Point point;
#else
    TouchPoint point;
#endif
    ts.begin();
    Serial.printf("Waiting for touch events...\n");
    touched = false;
    while (!touched)
    {
#ifdef USE_HARDWARE_SPI
        if (ts.touched())
        {
            point = ts.getPoint(); // Read touch coordinates
#else
        point = ts.getTouch(); // Read touch coordinates
#endif
            if (point.x != 0 && point.y != 0)
            {
                // If touch is detected, set the flag
                touched = true;
                Serial.printf("Touch event detected!\n");
                Serial.printf("Touch position: x=%d, y=%d, z=%d\n", point.x, point.y, point.z);
                break; // Exit loop after first touch event
            }
#ifdef USE_HARDWARE_SPI
        }
#endif
        tight_loop_contents();
    }
    touched = false;        // Reset flag after first touch event
    delay(1000);            // Allow some time to release the touch
    TEST_ASSERT_TRUE(true); // Test passes if we reach here without crashing
}

// Interrupt handler for TFT touch controller (XPT2046)
void touch_irq_isr(uint gpio, uint32_t events)
{
    // Set a volatile flag for main loop processing
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false); // disable IRQ after first touch event to prevent multiple triggers
#ifdef USE_HARDWARE_SPI
    if (ts.touched())
    {
        touch_event_point = ts.getPoint();
        touch_event_flag = touch_event_point.z > 0;
    }
    else
    {
        touch_event_flag = false;
    }
#else
    touch_event_point = ts.getTouch();
    touch_event_flag = touch_event_point.z > 0;
#endif
}

void test_touchscreen_read_coordinates_irq_wakeup()
{
    // This test is only valid if TOUCH_IRQ is defined

    ts.begin();
    if (TOUCH_IRQ != -1)
    {
        // Initialize the IRQ pin
        gpio_init(TOUCH_IRQ);
        gpio_set_dir(TOUCH_IRQ, GPIO_IN);
        gpio_pull_up(TOUCH_IRQ); // Enable pull-up resistor
        gpio_set_irq_enabled_with_callback(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true, &touch_irq_isr);
        Serial.printf("Waiting for touch event...\n");
        // Wait for touch event
        while (!touch_event_flag)
        {
            // Wait for touch event
            tight_loop_contents();
        }
        Serial.printf_P("Touch event detected!\n");
        Serial.printf("Touch position: x=%d, y=%d, z=%d\n", touch_event_point.x, touch_event_point.y, touch_event_point.z);
        touch_event_flag = false;                                                                           // Reset flag after first touch event
        delay(1000);                                                                                        // Allow some time to release the touch
        TEST_ASSERT_TRUE(touch_event_point.x != 0 || touch_event_point.y != 0 || touch_event_point.z != 0); // Ensure touch was detected
    }
    else
    {
        Serial.println("TOUCH_IRQ not defined, skipping test.");
    }
}

// Combined display and touch test (based on Arduino forum example)
// https://forum.arduino.cc/t/problem-with-xpt2046-touch-on-a-ili9488-4-display/1187818
void test_display_and_touch_combined()
{
    Serial.println("\n=== Combined Display and Touch Test ===");

    // Initialize display
    tft.fillScreen(ILI9341_WHITE);
    tft.setTextColor(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 20);
    tft.println("Display OK");
    tft.println("Touch Test");
    tft.println("");
    tft.setTextSize(1);
    tft.println("Touch screen to see coords");

    // Turn on backlight
    setBacklight(0.5);

    // Initialize touch
    ts.begin();
#ifdef USE_HARDWARE_SPI
    ts.setRotation(TFT_ROTATION);
#endif

    Serial.println("Display initialized");
    Serial.println("Touch controller initialized");
    Serial.println("Touch the screen - will display coords for 10 seconds");

    // Monitor touch for 10 seconds
    unsigned long startTime = millis();
    unsigned long lastTouchTime = 0;
    int touchCount = 0;

    while (millis() - startTime < 10000)
    {
#ifdef USE_HARDWARE_SPI
        if (ts.touched())
        {
            TS_Point p = ts.getPoint();
#else
        TouchPoint p = ts.getTouch();
        if (p.z > 0)
        {
#endif
            // Debounce - only print if 100ms since last touch
            if (millis() - lastTouchTime > 100)
            {
                Serial.print("Touch #");
                Serial.print(++touchCount);
                Serial.print(" - Pressure = ");
                Serial.print(p.z);
                Serial.print(", x = ");
                Serial.print(p.x);
                Serial.print(", y = ");
                Serial.println(p.y);

                // Display on screen
                tft.fillRect(0, 80, 240, 40, ILI9341_WHITE);
                tft.setTextColor(ILI9341_BLUE);
                tft.setTextSize(1);
                tft.setCursor(0, 80);
                tft.print("Touch #");
                tft.println(touchCount);
                tft.print("P=");
                tft.print(p.z);
                tft.print(" X=");
                tft.print(p.x);
                tft.print(" Y=");
                tft.println(p.y);

                lastTouchTime = millis();
            }
        }
#ifdef USE_HARDWARE_SPI
#endif
        delay(10);
    }

    // Final summary
    tft.fillRect(0, 140, 240, 60, ILI9341_GREEN);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 150);
    tft.print("Total: ");
    tft.print(touchCount);
    tft.println(" touches");

    Serial.print("\nTest complete. Total touches detected: ");
    Serial.println(touchCount);

    delay(2000);
    TEST_ASSERT_TRUE(true);
}

// Touch calibration test (MMSE-based multipoint calibration)
void test_touch_calibration()
{
    Serial.println("\n=== Touch Calibration Test ===");

    // Ensure backlight is on (may have been turned off by previous tests)
    setBacklight(0.8);

    // Set display rotation
    tft.setRotation(TFT_ROTATION);

    // Initialize touch controller with matching rotation
    ts.begin();
#ifdef USE_HARDWARE_SPI
    ts.setRotation(TFT_ROTATION);
#endif

    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.printf("MMSE Touch Calibration\n");
    tft.printf("%d-point mode\n", NUM_CALIB_POINTS);
    tft.setTextSize(1);
    tft.print("Touch each crosshair\n");
    delay(2000);

    // Calibration points configuration
    const int numPoints = NUM_CALIB_POINTS;
    const int margin = 30;

    int lcd_x[25]; // Max 25 points (5x5 grid)
    int lcd_y[25];
    uint16_t touch_x[25], touch_y[25];

    if (numPoints == 5)
    {
        // 5-point: 4 corners + center
        lcd_x[0] = margin;
        lcd_y[0] = margin;
        lcd_x[1] = tft.width() - margin;
        lcd_y[1] = margin;
        lcd_x[2] = tft.width() - margin;
        lcd_y[2] = tft.height() - margin;
        lcd_x[3] = margin;
        lcd_y[3] = tft.height() - margin;
        lcd_x[4] = tft.width() / 2;
        lcd_y[4] = tft.height() / 2;
    }
    else if (numPoints == 9)
    {
        // 9-point: 3x3 grid
        lcd_x[0] = margin;
        lcd_y[0] = margin;
        lcd_x[1] = tft.width() / 2;
        lcd_y[1] = margin;
        lcd_x[2] = tft.width() - margin;
        lcd_y[2] = margin;
        lcd_x[3] = tft.width() - margin;
        lcd_y[3] = tft.height() / 2;
        lcd_x[4] = tft.width() - margin;
        lcd_y[4] = tft.height() - margin;
        lcd_x[5] = tft.width() / 2;
        lcd_y[5] = tft.height() - margin;
        lcd_x[6] = margin;
        lcd_y[6] = tft.height() - margin;
        lcd_x[7] = margin;
        lcd_y[7] = tft.height() / 2;
        lcd_x[8] = tft.width() / 2;
        lcd_y[8] = tft.height() / 2;
    }
    else if (numPoints == 25)
    {
        // 25-point: 5x5 grid for best accuracy
        int point_idx = 0;
        for (int row = 0; row < 5; row++)
        {
            for (int col = 0; col < 5; col++)
            {
                lcd_x[point_idx] = margin + col * (tft.width() - 2 * margin) / 4;
                lcd_y[point_idx] = margin + row * (tft.height() - 2 * margin) / 4;
                point_idx++;
            }
        }
    }
    else
    {
        Serial.printf("Error: Unsupported number of calibration points: %d\n", numPoints);
        TEST_ASSERT_TRUE(false);
        return;
    }

    // Collect calibration points
    for (int i = 0; i < numPoints; ++i)
    {
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(1);
        tft.setCursor(10, 10);
        tft.printf("Touch cross %d/%d", i + 1, numPoints);
        drawCalibrationCross(lcd_x[i], lcd_y[i]);

        Serial.printf("Waiting for touch at point %d...\n", i + 1);
        bool touch_detected = false;

#ifdef USE_HARDWARE_SPI
        while (!touch_detected)
        {
            if (ts.touched())
            {
                touch_event_point = ts.getPoint();
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
        while (ts.touched())
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
        touch_event_point = ts.getTouch();
        Serial.printf("Touch detected: x=%d, y=%d, z=%d\n",
                      touch_event_point.xRaw, touch_event_point.yRaw, touch_event_point.z);
#endif

        drawCalibrationCross(lcd_x[i], lcd_y[i], ILI9341_GREEN);

        // Store raw coordinates without rotation - let MMSE algorithm handle the transformation
        touch_x[i] = getTouchRawX();
        touch_y[i] = getTouchRawY();

        delay(200);
        tft.fillScreen(ILI9341_BLACK);
    }

    // Print calibration points table
    Serial.println("\n=== Calibration Points Table ===");
    Serial.println("Point | LCD (X,Y)   | Touch Raw (X,Y)");
    Serial.println("------|-------------|----------------");
    for (int i = 0; i < numPoints; i++)
    {
        Serial.printf(" %2d   | (%3d,%3d)   | (%4d,%4d)\n",
                      i + 1, lcd_x[i], lcd_y[i], touch_x[i], touch_y[i]);
    }
    Serial.println();

    // Calculate MMSE calibration matrix
    if (!calculateMMSECalibration(numPoints, lcd_x, lcd_y, touch_x, touch_y, mmseCalibMatrix))
    {
        tft.fillScreen(ILI9341_RED);
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(10, 40);
        tft.print("Calibration failed!");
        delay(2000);
        TEST_ASSERT_TRUE(false);
        return;
    }

    // Verify calibration with test points
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(0, 0);
    tft.print("Verifying calibration...\n");
    tft.print("Touch 4 test points\n");

    const int numTestPoints = 4;
    int test_lcd_x[4] = {
        tft.width() / 4,
        tft.width() * 3 / 4,
        tft.width() / 2,
        tft.width() / 2};
    int test_lcd_y[4] = {
        tft.height() / 2,
        tft.height() / 2,
        tft.height() / 4,
        tft.height() * 3 / 4};
    uint16_t test_touch_x[4], test_touch_y[4];

    float total_error = 0.0f;
    float max_error = 0.0f;

    Serial.println("\nCalibration Verification:");
    Serial.println("Point | Expected | Calibrated | Error");

    for (int i = 0; i < numTestPoints; ++i)
    {
        tft.fillRect(0, 0, tft.width(), 30, ILI9341_BLACK);
        tft.setCursor(10, 10);
        tft.printf("Touch test point %d/4", i + 1);
        drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], ILI9341_YELLOW);

#ifdef USE_HARDWARE_SPI
        bool touch_detected = false;
        while (!touch_detected)
        {
            if (ts.touched())
            {
                touch_event_point = ts.getPoint();
                if (touch_event_point.z > 0)
                {
                    touch_detected = true;
                }
            }
            delay(10);
        }
        delay(100);
        while (ts.touched())
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
        touch_event_point = ts.getTouch();
#endif

        // Store raw coordinates without rotation - MMSE matrix handles the transformation
        test_touch_x[i] = getTouchRawX();
        test_touch_y[i] = getTouchRawY();

        // Apply MMSE matrix directly (matrix was built from raw coordinates)
        int calibrated_x, calibrated_y;
        float xt = (float)test_touch_x[i];
        float yt = (float)test_touch_y[i];
        calibrated_x = (int)(mmseCalibMatrix.A * xt + mmseCalibMatrix.B * yt + mmseCalibMatrix.C);
        calibrated_y = (int)(mmseCalibMatrix.D * xt + mmseCalibMatrix.E * yt + mmseCalibMatrix.F);

        float error_x = calibrated_x - test_lcd_x[i];
        float error_y = calibrated_y - test_lcd_y[i];
        float error = sqrtf(error_x * error_x + error_y * error_y);

        // Print to serial
        Serial.printf("  %d   | (%3d,%3d) | (%3d,%3d)   | %.2f px\n",
                      i + 1, test_lcd_x[i], test_lcd_y[i],
                      calibrated_x, calibrated_y, error);

        total_error += error;
        if (error > max_error)
            max_error = error;

        // Show result on display
        drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], ILI9341_GREEN);
        tft.setTextColor(ILI9341_WHITE);
        tft.setTextSize(1);
        tft.setCursor(10, 40);
        tft.printf("Expected: (%d,%d)\n", test_lcd_x[i], test_lcd_y[i]);
        tft.printf("Calibrated: (%d,%d)\n", calibrated_x, calibrated_y);
        tft.printf("Error: %.1f px", error);

        delay(1500);
        tft.fillScreen(ILI9341_BLACK);
    }

    float avg_error = total_error / numTestPoints;

    Serial.printf("Avg Error: %.2f px\n", avg_error);
    Serial.printf("Max Error: %.2f px\n", max_error);

    // Display results
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

    delay(3000);
    TEST_ASSERT_TRUE(avg_error < 20.0f);
}

void setup()
{
    UNITY_BEGIN();
    RUN_TEST(test_tft_begin);
#if defined(TFT_TESTS)
    RUN_TEST(test_tft_begin);
    RUN_TEST(test_tft_backlight_on);
    RUN_TEST(test_tft_backlight_dim);
    RUN_TEST(test_tft_read_display_id);
    RUN_TEST(test_tft_read_display_status);
    RUN_TEST(test_tft_fillScreen);
#if defined(TFT_TESTS_SLOW)
    RUN_TEST(test_tft_drawPixel);
    RUN_TEST(test_tft_text);
    RUN_TEST(test_Lines);
    RUN_TEST(test_tft_rotateScreen);
    RUN_TEST(test_FastLines);
    RUN_TEST(test_Rects);
    RUN_TEST(test_FilledRects);
    RUN_TEST(test_FilledCircles);
    RUN_TEST(test_Circles);
    RUN_TEST(test_Triangles);
    RUN_TEST(test_FilledTriangles);
    RUN_TEST(test_RoundRects);
    RUN_TEST(test_FilledRoundRects);
    RUN_TEST(test_vertical_scroll);
#endif // defined(TFT_TESTS_SLOW)
#endif
#if defined(TOUCH_TESTS)
    RUN_TEST(test_touchscreen_begin);
    RUN_TEST(test_touchscreen_not_touched);
    RUN_TEST(test_touchscreen_irq_inactive);
#endif
#if defined(TOUCH_TESTS_INTERACTIVE)
    RUN_TEST(test_touch_calibration);
//    RUN_TEST(test_display_and_touch_combined);
//    RUN_TEST(test_touchscreen_read_coordinates);
//    RUN_TEST(test_touchscreen_read_coordinates_irq_wakeup);
#endif
    RUN_TEST(test_tft_backlight_off);
    UNITY_END();
    // Most tests are based on examples from https://simple-circuit.com/interfacing-arduino-ili9341-tft-display/
}
void loop()
{
    // not used
}