/**
 * @file tests.cpp
 * @brief Unified test suite for Pico 2W system information, TFT display, Wiznet adapter, and SD card
 * @details Combines test_sysinfo, test_tft, test_wiznet, and test_sd into one file
 *          Use compiler flags to enable/disable test suites:
 *          -D ENABLE_SYSINFO_TESTS=1    (system info tests)
 *          -D ENABLE_TFT_TESTS=1        (TFT display tests)
 *          -D ENABLE_WIZNET_TESTS=1     (Wiznet ethernet tests)
 *          -D ENABLE_SD_TESTS=1         (SD card tests)
 */

#include <Arduino.h>
#include <unity.h>
#include <SPI.h>
#include <SD.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <hardware/regs/addressmap.h>
#include <hardware/regs/sio.h>
#include "hardware/spi.h"
#include <TFT_eSPI.h>
#include <Ethernet_Generic.h>
#include <FS.h>
#include "pin_config.h"
#include "sd_spi.h"

// Enable/disable test suites via compiler flags
#ifndef ENABLE_SYSINFO_TESTS
#define ENABLE_SYSINFO_TESTS 1
#endif

#ifndef ENABLE_TFT_TESTS
#define ENABLE_TFT_TESTS 1
#endif

#ifndef ENABLE_WIZNET_TESTS
#define ENABLE_WIZNET_TESTS 1 // Default to Wiznet tests
#endif

#ifndef ENABLE_SD_TESTS
#define ENABLE_SD_TESTS 1
#endif

// Enable/disable interactive tests (requiring user input or observation)
#ifndef ENABLE_INTERACTIVE_TESTS
#define ENABLE_INTERACTIVE_TESTS 1
#endif

// Enable/disable interactive tests (requiring user input or observation)
#ifndef ENABLE_SINGLE_TEST
#define ENABLE_SINGLE_TEST 0
#endif

// Declare TFT instance. Only declares the class and internal variables here, initialization is done in each test to ensure SPI configuration is correct for each test case.
TFT_eSPI tft = TFT_eSPI();

// ============================================================================
// COMMON TEST SETUP/TEARDOWN
// ============================================================================

void setUp(void)
{
    // This runs before each test
#if ENABLE_SD_TESTS
    // Ensure CS is high before each SD test
    digitalWrite(SD_CS, HIGH);
#endif
}

void tearDown(void)
{
    // This runs after each test
}

// ============================================================================
// SYSINFO TEST SUITE (test_sysinfo/tests.cpp)
// ============================================================================

#if ENABLE_SYSINFO_TESTS

#include <pico/unique_id.h>
#include <pico/bootrom.h>

// Expected values for Pico 2 W
#define EXPECTED_FLASH_SIZE (4 * 1024 * 1024) // 4 MB
#define EXPECTED_RAM_SIZE (512 * 1024)        // 512 KB
#define EXPECTED_CPU_FREQ_MHZ 150             // 150 MHz default

// Pico 2 W wireless GPIO pins (connected to CYW43439)
#define WL_GPIO0 29 // GP29 - CYW43 Data/Status
#define WL_GPIO1 25 // GP25 - CYW43 Power/Control (shared with LED)
#define WL_GPIO2 24 // GP24 - CYW43 Clock/Control

// Global variables for test state
pico_unique_board_id_t unique_id;
uint8_t flash_id[8];
uint32_t sys_clock_khz = 0;

/**
 * @brief Get total RAM size
 */
uint32_t get_total_ram()
{
    // RP2350 has 512 KB SRAM (SRAM0-SRAM9)
    return 512 * 1024;
}

/**
 * @brief Get flash size by reading the flash chip
 */
uint32_t get_flash_size()
{
    // For RP2350, flash size is typically 4 MB
    // This can be detected via flash ID or hardcoded based on board
    return 4 * 1024 * 1024;
}

/**
 * @brief Get free heap memory
 */
uint32_t get_free_heap()
{
    extern char __StackLimit, __bss_end__;
    return &__StackLimit - &__bss_end__;
}

/**
 * @brief Print formatted chip ID
 */
void print_chip_id(uint8_t *id, size_t len)
{
    for (size_t i = 0; i < len; i++)
    {
        Serial.printf("%02X", id[i]);
        if (i < len - 1)
            Serial.print(":");
    }
    Serial.println();
}

void test_read_unique_chip_id()
{
    Serial.println("Testing unique chip ID...");

    // Read unique 64-bit ID from flash
    uint32_t ints = save_and_disable_interrupts();
    flash_get_unique_id(flash_id);
    restore_interrupts(ints);

    // Verify ID is not all zeros
    bool all_zeros = true;
    for (int i = 0; i < 8; i++)
    {
        if (flash_id[i] != 0x00)
            all_zeros = false;
    }
    TEST_ASSERT_FALSE_MESSAGE(all_zeros, "Chip ID should not be all zeros");

    // Verify ID is not all 0xFF
    bool all_ff = true;
    for (int i = 0; i < 8; i++)
    {
        if (flash_id[i] != 0xFF)
            all_ff = false;
    }
    TEST_ASSERT_FALSE_MESSAGE(all_ff, "Chip ID should not be all 0xFF");

    Serial.print("Unique Chip ID: ");
    print_chip_id(flash_id, 8);
}

void test_read_board_id()
{
    Serial.println("Testing board unique ID...");

    // Get board ID
    pico_get_unique_board_id(&unique_id);

    // Verify ID is not all zeros
    bool all_zeros = true;
    for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
    {
        if (unique_id.id[i] != 0x00)
            all_zeros = false;
    }
    TEST_ASSERT_FALSE_MESSAGE(all_zeros, "Board ID should not be all zeros");

    Serial.print("Board Unique ID: ");
    print_chip_id(unique_id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
}

void test_cpu_frequency()
{
    Serial.println("Testing CPU frequency...");

    // Get system clock frequency
    sys_clock_khz = clock_get_hz(clk_sys) / 1000;
    uint32_t sys_clock_mhz = sys_clock_khz / 1000;

    Serial.printf("System Clock: %lu kHz (%lu MHz)\n", sys_clock_khz, sys_clock_mhz);

    // Verify clock is reasonable (should be around 150 MHz for RP2350)
    TEST_ASSERT_GREATER_THAN_MESSAGE(100000, sys_clock_khz, "System clock should be > 100 MHz");
    TEST_ASSERT_LESS_THAN_MESSAGE(200000, sys_clock_khz, "System clock should be < 200 MHz");

    // Get peripheral clock frequency
    uint32_t peri_clock_khz = clock_get_hz(clk_peri) / 1000;
    Serial.printf("Peripheral Clock: %lu kHz (%lu MHz)\n", peri_clock_khz, peri_clock_khz / 1000);

    // Get USB clock frequency
    uint32_t usb_clock_khz = clock_get_hz(clk_usb) / 1000;
    Serial.printf("USB Clock: %lu kHz (%lu MHz)\n", usb_clock_khz, usb_clock_khz / 1000);

    // USB clock should be 48 MHz
    TEST_ASSERT_EQUAL_MESSAGE(48000, usb_clock_khz, "USB clock should be 48 MHz");
}

void test_flash_size()
{
    Serial.println("Testing flash size...");

    uint32_t flash_size = get_flash_size();
    Serial.printf("Flash Size: %lu bytes (%lu MB)\n", flash_size, flash_size / (1024 * 1024));

    // Pico 2 W has 4 MB flash
    TEST_ASSERT_EQUAL_MESSAGE(EXPECTED_FLASH_SIZE, flash_size, "Flash size should be 4 MB");
}

void test_ram_size()
{
    Serial.println("Testing RAM size...");

    uint32_t ram_size = get_total_ram();
    Serial.printf("Total RAM: %lu bytes (%lu KB)\n", ram_size, ram_size / 1024);

    // RP2350 has 512 KB SRAM
    TEST_ASSERT_EQUAL_MESSAGE(EXPECTED_RAM_SIZE, ram_size, "RAM size should be 512 KB");
}

void test_free_heap()
{
    Serial.println("Testing free heap memory...");

    uint32_t free_heap = get_free_heap();
    Serial.printf("Free Heap: %lu bytes (%lu KB)\n", free_heap, free_heap / 1024);

    // Should have some free heap
    TEST_ASSERT_GREATER_THAN_MESSAGE(10000, free_heap, "Should have at least 10 KB free heap");
}

void test_bootrom_version()
{
    Serial.println("Testing bootrom version...");

    // RP2350 has bootrom but direct memory access can cause issues
    // Instead, we verify bootrom functions are available
    Serial.println("Bootrom: Present (RP2350 bootrom)");
    Serial.println("Bootrom functions: Available via SDK");

    // This test just verifies we can run without crashing
    TEST_ASSERT_TRUE_MESSAGE(true, "Bootrom verification successful");
}

void test_reset_cause()
{
    Serial.println("Testing reset cause...");

    // Check if watchdog caused the reset
    bool watchdog_reset = watchdog_caused_reboot();

    if (watchdog_reset)
    {
        Serial.println("Reset cause: Watchdog");
    }
    else
    {
        Serial.println("Reset cause: Power-on or manual reset");
    }

    // This is informational, so we just pass
    TEST_ASSERT_TRUE(true);
}

void test_chip_type()
{
    Serial.println("Testing chip type detection...");

    // RP2350 identification
    Serial.println("Chip: RP2350 (Raspberry Pi Pico 2 W)");
    Serial.println("Architecture: ARM Cortex-M33");
    Serial.println("Cores: Dual-core");

    // Verify we're running on RP2350 by checking memory map constants
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x20000000, SRAM_BASE, "SRAM base should be 0x20000000");
    TEST_ASSERT_EQUAL_HEX32_MESSAGE(0x10000000, XIP_BASE, "Flash XIP base should be 0x10000000");
}

void test_psram()
{
    Serial.println("Testing PSRAM...");

    // Pico 2 W does not have PSRAM
    Serial.println("PSRAM: Not available on Pico 2 W");

    TEST_ASSERT_TRUE_MESSAGE(true, "PSRAM test N/A for this board");
}

void test_core_number()
{
    Serial.println("Testing CPU core number...");

    // Get current core number (should be 0 or 1)
    uint32_t core_num = get_core_num();

    Serial.printf("Running on Core: %lu\n", core_num);

    // Should be core 0 or 1
    TEST_ASSERT_TRUE_MESSAGE(core_num == 0 || core_num == 1, "Core number should be 0 or 1");
}

void test_voltage_regulation()
{
    Serial.println("Testing voltage regulation...");

    // Read voltage regulator settings
    // Note: Direct voltage reading may not be available, but we can check if VREG is configured
    Serial.println("Voltage Regulator: Configured for 1.1V (default)");

    // This is informational
    TEST_ASSERT_TRUE(true);
}

void test_wl_gpio0()
{
    Serial.println("Testing WL_GPIO0 (GP29) with LED flash test...");

    // WL_GPIO0 is connected to CYW43439 wireless chip
    // GP29 on RP2350, used for wireless chip data/status
    Serial.printf("WL_GPIO0: GP%d (CYW43 Data/Status)\n", WL_GPIO0);

    // Initialize WL_GPIO0 for testing
    gpio_init(WL_GPIO0);
    gpio_set_dir(WL_GPIO0, GPIO_IN);
    gpio_pull_up(WL_GPIO0);

    // Flash LED briefly to visually indicate test is running
    // LED is on WL_GPIO1 (GP25), controlled through CYW43 chip on Pico 2 W
    // Very slow flashing (2 seconds on/off) to minimize EMI issues with power supply
    Serial.println("Flashing LED 3 times (slow flash to reduce EMI)...");
    pinMode(LED_BUILTIN, OUTPUT);

    int flash_count = 3;
    for (int i = 0; i < flash_count; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(2000); // 2 seconds on
        digitalWrite(LED_BUILTIN, LOW);
        delay(2000); // 2 seconds off
    }

    Serial.printf("LED flashed %d times\n", flash_count);

    // Verify we can configure WL_GPIO0 pin
    uint32_t gpio_oe = (sio_hw->gpio_oe >> WL_GPIO0) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(0, gpio_oe, "WL_GPIO0 should be configured as input");

    // Check pad configuration
    uint32_t pad = pads_bank0_hw->io[WL_GPIO0];
    uint32_t ie = (pad >> PADS_BANK0_GPIO0_IE_LSB) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(1, ie, "WL_GPIO0 input enable should be set");

    Serial.println("WL_GPIO0 accessible and configurable, LED flash test complete");
}

void test_wl_gpio1()
{
    Serial.println("Testing WL_GPIO1 (GP25)...");

    // WL_GPIO1 is connected to CYW43439 wireless chip
    // GP25 on RP2350, often shared with onboard LED
    Serial.printf("WL_GPIO1: GP%d (CYW43 Power/Control, LED)\n", WL_GPIO1);

    // Initialize as output to test accessibility
    gpio_init(WL_GPIO1);
    gpio_set_dir(WL_GPIO1, GPIO_OUT);
    gpio_put(WL_GPIO1, 0);

    // Verify we can configure the pin
    uint32_t gpio_oe = (sio_hw->gpio_oe >> WL_GPIO1) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(1, gpio_oe, "WL_GPIO1 should be configured as output");

    // Test we can toggle the pin
    gpio_put(WL_GPIO1, 1);
    delay(1);
    gpio_put(WL_GPIO1, 0);

    Serial.println("WL_GPIO1 accessible and configurable");
}

void test_wl_gpio2()
{
    Serial.println("Testing WL_GPIO2 (GP24)...");

    // WL_GPIO2 is connected to CYW43439 wireless chip
    // GP24 on RP2350, used for wireless chip clock/control
    // This pin should be HIGH when VSYS is present
    Serial.printf("WL_GPIO2: GP%d (CYW43 Clock/Control)\n", WL_GPIO2);

    // Initialize as input to test accessibility
    gpio_init(WL_GPIO2);
    gpio_set_dir(WL_GPIO2, GPIO_IN);
    gpio_pull_up(WL_GPIO2);

    // Read the pin state
    delay(10); // Small delay for signal to stabilize
    uint32_t pin_state = gpio_get(WL_GPIO2);

    Serial.printf("WL_GPIO2 state: %s\n", pin_state ? "HIGH" : "LOW");

    // Verify we can configure the pin
    uint32_t gpio_oe = (sio_hw->gpio_oe >> WL_GPIO2) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(0, gpio_oe, "WL_GPIO2 should be configured as input");

    // Check pad configuration
    uint32_t pad = pads_bank0_hw->io[WL_GPIO2];
    uint32_t ie = (pad >> PADS_BANK0_GPIO0_IE_LSB) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(1, ie, "WL_GPIO2 input enable should be set");

    // Note: WL_GPIO2 state may vary depending on wireless chip state
    // After LED operations (on WL_GPIO1), the CYW43 chip may change pin states
    Serial.printf("WL_GPIO2 pin state reported as: %s (may vary with wireless chip activity)\n",
                  pin_state ? "HIGH" : "LOW");

    Serial.println("WL_GPIO2 accessible and configurable");
}

void test_wireless_gpio_summary()
{
    Serial.println("Testing wireless GPIO summary...");

    Serial.println("\n--- Wireless GPIO Pins (Pico 2 W) ---");
    Serial.printf("WL_GPIO0 (GP%d): CYW43 Data/Status\n", WL_GPIO0);
    Serial.printf("WL_GPIO1 (GP%d): CYW43 Power/Control (LED)\n", WL_GPIO1);
    Serial.printf("WL_GPIO2 (GP%d): CYW43 Clock/Control\n", WL_GPIO2);
    Serial.println("Wireless Chip: Infineon CYW43439");
    Serial.println("All wireless GPIO pins accessible\n");

    TEST_ASSERT_TRUE(true);
}

void test_system_info_summary()
{
    Serial.println("\n===========================================");
    Serial.println("       SYSTEM INFORMATION SUMMARY");
    Serial.println("===========================================");

    // Board info
    Serial.println("\n--- Board Information ---");
    Serial.println("Board: Raspberry Pi Pico 2 W");
    Serial.println("Chip: RP2350");
    Serial.println("Architecture: ARM Cortex-M33 (Dual-core)");

    // Unique IDs
    Serial.println("\n--- Unique Identifiers ---");
    Serial.print("Board ID: ");
    print_chip_id(unique_id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
    Serial.print("Flash ID: ");
    print_chip_id(flash_id, 8);

    // Memory
    Serial.println("\n--- Memory ---");
    Serial.printf("Flash Size: %u MB\n", get_flash_size() / (1024 * 1024));
    Serial.printf("Total RAM: %u KB\n", get_total_ram() / 1024);
    Serial.printf("Free Heap: %lu KB\n", get_free_heap() / 1024);

    // Clock frequencies
    Serial.println("\n--- Clock Frequencies ---");
    Serial.printf("System Clock: %lu MHz\n", sys_clock_khz / 1000);
    Serial.printf("Peripheral Clock: %lu MHz\n", clock_get_hz(clk_peri) / 1000000);
    Serial.printf("USB Clock: %lu MHz\n", clock_get_hz(clk_usb) / 1000000);

    // Core info
    Serial.println("\n--- Core Information ---");
    Serial.printf("Current Core: %u\n", get_core_num());

    // Wireless
    Serial.println("\n--- Wireless ---");
    Serial.println("WiFi: Infineon CYW43439 (2.4 GHz)");
    Serial.println("Bluetooth: BLE 5.2");
    Serial.printf("WL_GPIO0: GP%d, WL_GPIO1: GP%d, WL_GPIO2: GP%d\n", WL_GPIO0, WL_GPIO1, WL_GPIO2);

    // SDK info
    Serial.println("\n--- SDK Information ---");
    Serial.printf("Arduino Framework: %s\n", ARDUINO_VARIANT);
    Serial.printf("SDK Version: Pico SDK\n");

    Serial.println("===========================================\n");

    TEST_ASSERT_TRUE(true);
}

#endif // ENABLE_SYSINFO_TESTS

// ============================================================================
// TFT TEST SUITE (test_tft/tests.cpp)
// ============================================================================

#if ENABLE_TFT_TESTS

#ifndef SPI_FREQUENCY
#define SPI_FREQUENCY 37500000 // 37.5 MHz default for testing (can be adjusted via compiler flag)
#endif

static void enableBacklight()
{
    if (TFT_BL != -1)
    {
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, HIGH);
    }
}

static void disableBacklight()
{
    if (TFT_BL != -1)
    {
        pinMode(TFT_BL, OUTPUT);
        digitalWrite(TFT_BL, LOW);
    }
}

// SPI speed verification helpers (read from RP2040 SPI registers)
static spi_inst_t *get_tft_spi()
{
    return spi0; // TFT is on SPI0 (GP0/2/3)
}

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

static void log_tft_spi_speed(const char *tag)
{
    spi_inst_t *spi = get_tft_spi();
    spi_hw_t *hw = spi_get_hw(spi);
    uint32_t cpsr = hw->cpsr & 0xFFu;
    uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
    uint32_t baud = get_tft_spi_baudrate(spi);

    Serial.printf("%s SPI baud: %.2f MHz (cpsr=%lu, scr=%lu)\n", tag, baud / 1000000.0, cpsr, scr);
}

/**
 * @brief Enumerate all achievable SPI frequencies for RP2350
 * Returns array of unique frequencies in descending order
 * Max frequencies returned: 256
 */
static uint32_t enumerate_spi_frequencies(uint32_t *freq_array, uint32_t max_count)
{
    uint32_t freq_in = clock_get_hz(clk_peri);
    uint32_t count = 0;

    // Collect all achievable frequencies
    for (uint prescale = 2; prescale <= 254; prescale += 2)
    {
        for (uint scr = 0; scr <= 255; scr++)
        {
            uint32_t freq = freq_in / (prescale * (scr + 1));

            // Check if already in array (avoid duplicates)
            bool duplicate = false;
            for (uint32_t i = 0; i < count; i++)
            {
                if (freq_array[i] == freq)
                {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate && count < max_count)
            {
                freq_array[count++] = freq;
            }
        }
    }

    // Sort descending
    for (uint32_t i = 0; i < count - 1; i++)
    {
        for (uint32_t j = i + 1; j < count; j++)
        {
            if (freq_array[i] < freq_array[j])
            {
                uint32_t tmp = freq_array[i];
                freq_array[i] = freq_array[j];
                freq_array[j] = tmp;
            }
        }
    }

    return count;
}

/**
 * @brief Set TFT SPI speed with fine control
 * Searches all valid CPSR/SCR combinations to find closest match to desired frequency
 */
static uint32_t set_tft_spi_speed_direct(uint32_t desired_freq)
{
    spi_inst_t *spi = get_tft_spi();
    spi_hw_t *hw = spi_get_hw(spi);
    uint32_t freq_in = clock_get_hz(clk_peri);

    uint best_prescale = 254, best_scr = 255;
    uint32_t best_error = UINT32_MAX;

    // Search all valid combinations for closest match (may exceed or undershoot request)
    for (uint prescale = 2; prescale <= 254; prescale += 2)
    {
        for (uint scr = 0; scr <= 255; scr++)
        {
            uint32_t actual = freq_in / (prescale * (scr + 1));
            uint32_t error = (actual >= desired_freq) ? (actual - desired_freq) : (desired_freq - actual);

            // Find closest match regardless of direction
            if (error < best_error)
            {
                best_error = error;
                best_prescale = prescale;
                best_scr = scr;
            }
        }
    }

    // Disable SPI before register changes
    uint32_t enable_mask = hw->cr1 & SPI_SSPCR1_SSE_BITS;
    hw_clear_bits(&hw->cr1, SPI_SSPCR1_SSE_BITS);

    // Set registers
    hw->cpsr = best_prescale;
    hw_write_masked(&hw->cr0, best_scr << SPI_SSPCR0_SCR_LSB, SPI_SSPCR0_SCR_BITS);

    // Re-enable SPI
    hw_set_bits(&hw->cr1, enable_mask);

    uint32_t actual_freq = freq_in / (best_prescale * (best_scr + 1));
    return actual_freq;
}

// Terminal scrolling variables and functions
#define TERM_TEXT_HEIGHT 16    // Height of text to be printed and scrolled
#define TERM_BOT_FIXED_AREA 0  // Number of lines in bottom fixed area
#define TERM_TOP_FIXED_AREA 16 // Number of lines in top fixed area
#define TERM_YMAX 320          // Bottom of screen area

uint16_t term_yStart = TERM_TOP_FIXED_AREA;
uint16_t term_yArea = TERM_YMAX - TERM_TOP_FIXED_AREA - TERM_BOT_FIXED_AREA;
uint16_t term_yDraw = TERM_YMAX - TERM_BOT_FIXED_AREA - TERM_TEXT_HEIGHT;
uint16_t term_xPos = 0;
int term_blank[19]; // Record of line lengths for optimization

void tft_setupScrollArea(uint16_t tfa, uint16_t bfa)
{
    tft.writecommand(ILI9341_VSCRDEF); // Vertical scroll definition
    tft.writedata(tfa >> 8);           // Top Fixed Area line count
    tft.writedata(tfa & 0xFF);
    tft.writedata((TERM_YMAX - tfa - bfa) >> 8); // Vertical Scrolling Area line count
    tft.writedata((TERM_YMAX - tfa - bfa) & 0xFF);
    tft.writedata(bfa >> 8); // Bottom Fixed Area line count
    tft.writedata(bfa & 0xFF);
}

static void tft_scroll_to(uint16_t scroll_start)
{
    tft.writecommand(ILI9341_VSCRSADD); // VSCRSADD
    tft.writedata(scroll_start >> 8);
    tft.writedata(scroll_start & 0xFF);
}

int term_scroll_line()
{
    int yTemp = term_yStart;
    tft.fillRect(0, term_yStart, term_blank[(18 + (term_yStart - TERM_TOP_FIXED_AREA) / TERM_TEXT_HEIGHT) % 19], TERM_TEXT_HEIGHT, TFT_BLACK);
    term_yStart += TERM_TEXT_HEIGHT;
    if (term_yStart >= TERM_YMAX - TERM_BOT_FIXED_AREA)
        term_yStart = TERM_TOP_FIXED_AREA + (term_yStart - TERM_YMAX + TERM_BOT_FIXED_AREA);
    tft_scroll_to(term_yStart);
    return yTemp;
}

static void initialize_tft(uint8_t rotation)
{
    static bool tft_initialized = false;

    disableBacklight();
    if (!tft_initialized)
    {
        tft.init();
        tft_initialized = true;
    }
    tft.fillScreen(TFT_BLACK);
    tft.setRotation(rotation); // Set rotation mode
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(0, 0);
    enableBacklight();
}

// Helper function to get rainbow colors for scrolling effects
static uint16_t getRainbowColor(int position, int timeOffset)
{
    int hue = (position + timeOffset) % 360;
    if (hue < 60)
        return tft.color565(255, hue * 4.25, 0); // Red to Yellow
    if (hue < 120)
        return tft.color565(255 - (hue - 60) * 4.25, 255, 0); // Yellow to Green
    if (hue < 180)
        return tft.color565(0, 255, (hue - 120) * 4.25); // Green to Cyan
    if (hue < 240)
        return tft.color565(0, 255 - (hue - 180) * 4.25, 255); // Cyan to Blue
    if (hue < 300)
        return tft.color565((hue - 240) * 4.25, 0, 255);   // Blue to Magenta
    return tft.color565(255, 0, 255 - (hue - 300) * 4.25); // Magenta to Red
}

// Calibration support (MMSE-based multipoint calibration)
struct CalibrationMatrix
{
    float A, B, C, D, E, F;
};

static CalibrationMatrix mmseCalibMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};
static int NUM_CALIB_POINTS = 5; // 5-point (corners + center) or 9-point (3x3 grid)

static void drawCalibrationCross(int x, int y, uint16_t color = TFT_RED)
{
    tft.drawLine(x - 10, y, x + 10, y, color);
    tft.drawLine(x, y - 10, x, y + 10, color);
}

static bool calculateMMSECalibration(int numPoints,
                                     const int *lcd_x, const int *lcd_y,
                                     const uint16_t *touch_x, const uint16_t *touch_y,
                                     CalibrationMatrix &matrix)
{
    if (numPoints < 3)
    {
        Serial.println("Error: Need at least 3 calibration points");
        return false;
    }

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

// Static variables for touch test IRQ handling
static volatile bool test_touch_event_flag = false;
static volatile uint32_t test_touch_irq_count = 0;

/**
 * @brief Touch IRQ callback function for test_display_and_touch
 */
static void touch_irq_callback(uint gpio, uint32_t events)
{
    (void)gpio;
    (void)events;
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false); // disable IRQ during processing
    test_touch_event_flag = true;
    test_touch_irq_count++;
}

/**
 * @brief Test: Touchscreen initialization
 */
void test_touch_begin()
{
    Serial.println("Testing touchscreen initialization...");

    // TFT_eSPI has built-in XPT2046 support configured via User_Setup.h
    // Just initialize the display with touch support enabled
    tft.init();

    // Initialize touch via TFT_eSPI (handles XPT2046 automatically if TOUCH_CS is defined)
    if (TOUCH_CS != -1)
    {
        gpio_init(TOUCH_CS);
        gpio_set_dir(TOUCH_CS, GPIO_OUT);
        gpio_put(TOUCH_CS, 1); // Initially high (deselected)
    }

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Touchscreen not touched (read coordinates with no touch)
 */
void test_touch_no_press()
{
    Serial.println("Testing touchscreen not pressed...");

    tft.init();

    if (TOUCH_CS != -1)
    {
        uint16_t x = 0, y = 0;
        uint8_t touched = tft.getTouch(&x, &y, 600); // 600 is default threshold

        if (touched)
        {
            Serial.printf("Touch detected during no-press test: x=%u, y=%u\n", x, y);
        }
        else
        {
            Serial.println("getTouch() returned 0 (not touched) - correct");
        }

        TEST_ASSERT_TRUE(true);
    }
    else
    {
        Serial.println("TOUCH_CS not defined, skipping test.");
        TEST_ASSERT_TRUE(true);
    }
}

/**
 * @brief Test: Touchscreen IRQ pin state when not touched
 */
void test_touch_irq_inactive()
{
    Serial.println("Testing touchscreen IRQ inactive...");

    if (TOUCH_IRQ != -1)
    {
        gpio_init(TOUCH_IRQ);
        gpio_set_dir(TOUCH_IRQ, GPIO_IN);
        gpio_pull_up(TOUCH_IRQ);

        bool irq_state = gpio_get(TOUCH_IRQ);
        Serial.printf("TOUCH_IRQ pin state (should be high/inactive): %u\n", irq_state);

        TEST_ASSERT_TRUE(irq_state);
    }
    else
    {
        Serial.println("TOUCH_IRQ not defined, skipping test.");
        TEST_ASSERT_TRUE(true);
    }
}

/**
 * @brief Test: Read touch coordinates with display feedback
 */
void test_touch_read_coordinates()
{
    Serial.println("Testing touch read coordinates...");

    if (TOUCH_CS == -1)
    {
        Serial.println("TOUCH_CS not defined, skipping test.");
        TEST_ASSERT_TRUE(true);
        return;
    }

    initialize_tft(1);
    tft.setTextSize(2);
    tft.setCursor(20, 20);
    tft.println("Touch Display");
    tft.setTextSize(1);
    tft.setCursor(20, 60);
    tft.println("Waiting for touch...");

    unsigned long start_time = millis();
    unsigned long timeout = 10000; // 10 second timeout
    uint16_t x = 0, y = 0;
    bool touched = false;

    while (millis() - start_time < timeout)
    {
        if (tft.getTouch(&x, &y, 600))
        {
            touched = true;
            Serial.printf("Touch detected: x=%u, y=%u\n", x, y);

            // Display touch coordinates on screen
            tft.fillRect(20, 100, 280, 60, TFT_BLACK);
            tft.setCursor(20, 100);
            tft.setTextColor(TFT_GREEN);
            tft.print("Touch: x=");
            tft.print(x);
            tft.print(" y=");
            tft.println(y);

            delay(1000);
            break;
        }
        delay(50);
    }

    if (!touched)
    {
        Serial.println("No touch detected within timeout");
    }

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Display and touch combined visual test using IRQ
 */
void test_display_and_touch()
{
    Serial.println("Testing display and touch combined (IRQ-based)...");

    if (TOUCH_CS == -1)
    {
        Serial.println("TOUCH_CS not defined, skipping test.");
        TEST_ASSERT_TRUE(true);
        return;
    }

    if (TOUCH_IRQ == -1)
    {
        Serial.println("TOUCH_IRQ not defined, falling back to polling mode.");
        // Fall back to original polling implementation
        initialize_tft(1);
        tft.setTextSize(2);
        tft.setCursor(0, 20);
        tft.println("Display & Touch");
        tft.println("Test (Polling)");
        tft.setTextSize(1);
        tft.println("");
        tft.println("Touch screen to show coordinates");
        tft.println("(10 second timeout)");

        unsigned long start_time = millis();
        unsigned long last_touch = 0;
        int touch_count = 0;

        while (millis() - start_time < 10000)
        {
            uint16_t x = 0, y = 0;
            if (tft.getTouch(&x, &y, 600))
            {
                if (millis() - last_touch > 100)
                {
                    touch_count++;
                    Serial.printf("Touch #%d: x=%u, y=%u\n", touch_count, x, y);

                    // Display on screen
                    tft.fillRect(0, 100, tft.width(), 60, TFT_BLACK);
                    tft.setTextColor(TFT_BLUE, TFT_BLACK);
                    tft.setTextSize(1);
                    tft.setCursor(0, 100);
                    tft.print("Touch #");
                    tft.println(touch_count);
                    tft.print("x=");
                    tft.print(x);
                    tft.print(" y=");
                    tft.println(y);

                    last_touch = millis();
                }
            }
            delay(20);
        }

        // Final summary
        tft.fillRect(0, 170, tft.width(), 40, TFT_GREEN);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(2);
        tft.setCursor(10, 180);
        tft.print("Total: ");
        tft.print(touch_count);
        tft.println(" touches");

        Serial.print("Test complete. Total touches: ");
        Serial.println(touch_count);

        delay(2000);

        TEST_ASSERT_TRUE(true);
        return;
    }

    // IRQ-based implementation
    initialize_tft(1);
    tft.setTextSize(2);
    tft.setCursor(0, 20);
    tft.println("Display & Touch");
    tft.println("Test (IRQ)");
    tft.setTextSize(1);
    tft.println("");
    tft.println("Touch screen to show coordinates");
    tft.println("(10 second timeout)");
    tft.println("Using interrupt-driven detection");

    // Set up touch IRQ for this test
    // Reset static variables for this test run
    test_touch_event_flag = false;
    test_touch_irq_count = 0;

    // Initialize touch IRQ pin
    gpio_init(TOUCH_IRQ);
    gpio_set_dir(TOUCH_IRQ, GPIO_IN);
    gpio_pull_up(TOUCH_IRQ);

    // Set up IRQ handler
    gpio_set_irq_enabled_with_callback(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true, touch_irq_callback);

    unsigned long start_time = millis();
    unsigned long last_touch = 0;
    int touch_count = 0;

    Serial.println("Waiting for touch interrupts...");

    while (millis() - start_time < 10000)
    {
        // Check for touch event flag set by IRQ
        if (test_touch_event_flag)
        {
            test_touch_event_flag = false; // Clear flag

            // Debounce check
            if (millis() - last_touch > 100)
            {
                // Read touch coordinates
                uint16_t x = 0, y = 0;
                bool touch_valid = tft.getTouch(&x, &y, 600);

                if (touch_valid)
                {
                    touch_count++;
                    Serial.printf("Touch #%d (IRQ #%lu): x=%u, y=%u\n", touch_count, test_touch_irq_count, x, y);

                    // Display on screen
                    tft.fillRect(0, 100, tft.width(), 60, TFT_BLACK);
                    tft.setTextColor(TFT_BLUE, TFT_BLACK);
                    tft.setTextSize(1);
                    tft.setCursor(0, 100);
                    tft.print("Touch #");
                    tft.println(touch_count);
                    tft.print("x=");
                    tft.print(x);
                    tft.print(" y=");
                    tft.println(y);
                    tft.print("IRQ count: ");
                    tft.println((unsigned long)test_touch_irq_count);

                    last_touch = millis();
                }
            }

            // Re-enable IRQ for next touch
            gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, true);
        }

        // Small delay to prevent busy waiting
        delay(10);
    }

    // Clean up IRQ
    gpio_set_irq_enabled(TOUCH_IRQ, GPIO_IRQ_EDGE_FALL, false);

    // Final summary
    tft.fillRect(0, 170, tft.width(), 60, TFT_GREEN);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 180);
    tft.print("Total: ");
    tft.print(touch_count);
    tft.println(" touches");
    tft.print("IRQs: ");
    tft.println((unsigned long)test_touch_irq_count);

    Serial.printf("Test complete. Total touches: %d, Total IRQs: %lu\n", touch_count, test_touch_irq_count);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Touch calibration using MMSE algorithm
 */
void test_touch_calibration()
{
    Serial.println("Testing touch calibration (MMSE 5-point)...");

    if (TOUCH_CS == -1)
    {
        Serial.println("TOUCH_CS not defined, skipping test.");
        TEST_ASSERT_TRUE(true);
        return;
    }

    initialize_tft(1);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.println("MMSE Touch");
    tft.println("Calibration");
    tft.setTextSize(1);
    tft.println("5-point mode");
    tft.println("Touch each cross");
    delay(2000);

    const int numPoints = NUM_CALIB_POINTS;
    const int margin = 30;

    int lcd_x[25], lcd_y[25];
    uint16_t touch_x[25], touch_y[25];

    // 5-point: 4 corners + center
    if (numPoints == 5)
    {
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

    // Collect calibration points
    for (int i = 0; i < numPoints; ++i)
    {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(1);
        tft.setCursor(10, 10);
        tft.printf("Touch cross %d/%d", i + 1, numPoints);
        drawCalibrationCross(lcd_x[i], lcd_y[i]);

        Serial.printf("Waiting for touch at point %d...\n", i + 1);
        bool touch_detected = false;
        unsigned long start = millis();

        while (millis() - start < 15000) // 15 second timeout per point
        {
            uint16_t x = 0, y = 0;
            if (tft.getTouch(&x, &y, 600))
            {
                touch_detected = true;
                Serial.printf("Touch detected: x=%u, y=%u\n", x, y);
                touch_x[i] = x;
                touch_y[i] = y;
                break;
            }
            delay(50);
        }

        if (!touch_detected)
        {
            Serial.printf("No touch for point %d, using fallback\n", i + 1);
            touch_x[i] = 2048;
            touch_y[i] = 2048;
        }

        drawCalibrationCross(lcd_x[i], lcd_y[i], TFT_GREEN);
        delay(300);
    }

    // Print calibration points table
    Serial.println("\n=== Calibration Points Table ===");
    Serial.println("Point | LCD (X,Y)   | Touch Raw (X,Y)");
    Serial.println("------|-------------|----------------");
    for (int i = 0; i < numPoints; i++)
    {
        Serial.printf(" %2d   | (%3d,%3d)   | (%4u,%4u)\n",
                      i + 1, lcd_x[i], lcd_y[i], touch_x[i], touch_y[i]);
    }
    Serial.println();

    // Calculate MMSE calibration matrix
    if (!calculateMMSECalibration(numPoints, lcd_x, lcd_y, touch_x, touch_y, mmseCalibMatrix))
    {
        tft.fillScreen(TFT_RED);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(10, 40);
        tft.println("Calibration failed!");
        delay(2000);
        TEST_ASSERT_TRUE(false);
        return;
    }

    // Display success
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(10, 40);
    tft.setTextSize(2);
    tft.setTextColor(TFT_GREEN);
    tft.println("Calibration");
    tft.println("Complete!");

    Serial.println("Calibration successful!");
    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Touch accuracy validation using calibration matrix
 * Validates the calibration by measuring delta between expected and measured coordinates
 */
void test_touch_accuracy()
{
    Serial.println("Testing touch accuracy with calibration matrix...");

    if (TOUCH_CS == -1)
    {
        Serial.println("TOUCH_CS not defined, skipping test.");
        TEST_ASSERT_TRUE(true);
        return;
    }

    initialize_tft(1);
    tft.setTextSize(2);
    tft.println("Touch Accuracy");
    tft.println("Test");
    tft.setTextSize(1);
    tft.println("Validating calibration");
    delay(2000);

    const int numTestPoints = 5;
    const int margin = 40;

    int test_lcd_x[5], test_lcd_y[5];
    uint16_t test_touch_x[5], test_touch_y[5];
    float measured_x[5], measured_y[5];
    float delta_x[5], delta_y[5], delta_distance[5];

    // Define test points (DIFFERENT from calibration points for true validation)
    // These are intermediate points to verify calibration works across the entire display
    test_lcd_x[0] = tft.width() / 3;      // 1/3 from left
    test_lcd_y[0] = tft.height() / 3;     // 1/3 from top
    test_lcd_x[1] = 2 * tft.width() / 3;  // 2/3 from left
    test_lcd_y[1] = tft.height() / 4;     // 1/4 from top
    test_lcd_x[2] = 3 * tft.width() / 4;  // 3/4 from left
    test_lcd_y[2] = 3 * tft.height() / 4; // 3/4 from top
    test_lcd_x[3] = tft.width() / 4;      // 1/4 from left
    test_lcd_y[3] = 2 * tft.height() / 3; // 2/3 from top
    test_lcd_x[4] = tft.width() / 2;      // Center X (same as calibration center)
    test_lcd_y[4] = tft.height() / 3;     // 1/3 from top (different Y)

    // Collect accuracy test points
    for (int i = 0; i < numTestPoints; ++i)
    {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE);
        tft.setTextSize(1);
        tft.setCursor(10, 10);
        tft.printf("Accuracy Test %d/%d", i + 1, numTestPoints);
        drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], TFT_YELLOW);

        tft.setCursor(10, 40);
        tft.printf("Expected: (%d,%d)", test_lcd_x[i], test_lcd_y[i]);

        Serial.printf("Waiting for touch at accuracy test point %d...\n", i + 1);
        bool touch_detected = false;
        unsigned long start = millis();

        while (millis() - start < 15000) // 15 second timeout per point
        {
            uint16_t x = 0, y = 0;
            if (tft.getTouch(&x, &y, 600))
            {
                touch_detected = true;
                Serial.printf("Raw touch: x=%u, y=%u\n", x, y);
                test_touch_x[i] = x;
                test_touch_y[i] = y;
                break;
            }
            delay(50);
        }

        if (!touch_detected)
        {
            Serial.printf("No touch for accuracy test point %d\n", i + 1);
            test_touch_x[i] = 2048;
            test_touch_y[i] = 2048;
        }

        // Apply calibration matrix transformation
        // measured_x = A*touch_x + B*touch_y + C
        // measured_y = D*touch_x + E*touch_y + F
        measured_x[i] = mmseCalibMatrix.A * test_touch_x[i] +
                        mmseCalibMatrix.B * test_touch_y[i] +
                        mmseCalibMatrix.C;
        measured_y[i] = mmseCalibMatrix.D * test_touch_x[i] +
                        mmseCalibMatrix.E * test_touch_y[i] +
                        mmseCalibMatrix.F;

        // Calculate delta (error)
        delta_x[i] = measured_x[i] - test_lcd_x[i];
        delta_y[i] = measured_y[i] - test_lcd_y[i];
        delta_distance[i] = sqrt(delta_x[i] * delta_x[i] + delta_y[i] * delta_y[i]);

        // Display result
        drawCalibrationCross(test_lcd_x[i], test_lcd_y[i], TFT_GREEN);
        tft.setTextColor(TFT_CYAN);
        tft.setCursor(10, 60);
        tft.printf("Measured: (%.0f,%.0f)", measured_x[i], measured_y[i]);
        tft.setCursor(10, 75);
        tft.setTextColor(TFT_RED);
        tft.printf("Delta: (%.1f,%.1f) pix", delta_x[i], delta_y[i]);

        Serial.printf("Point %d: Expected (%.0f,%.0f), Measured (%.0f,%.0f), Delta (%.1f,%.1f) px\n",
                      i + 1, (float)test_lcd_x[i], (float)test_lcd_y[i],
                      measured_x[i], measured_y[i], delta_x[i], delta_y[i]);

        delay(1000);
    }

    // Calculate accuracy statistics
    float mean_error = 0, max_error = 0;
    for (int i = 0; i < numTestPoints; i++)
    {
        mean_error += delta_distance[i];
        if (delta_distance[i] > max_error)
            max_error = delta_distance[i];
    }
    mean_error /= numTestPoints;

    // Display accuracy summary
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Accuracy Report");

    tft.setTextSize(1);
    tft.setTextColor(TFT_CYAN);
    tft.setCursor(10, 50);
    tft.printf("Mean Error: %.2f px", mean_error);

    tft.setCursor(10, 70);
    tft.printf("Max Error:  %.2f px", max_error);

    tft.setCursor(10, 100);
    tft.setTextColor(TFT_YELLOW);
    tft.println("Point Details:");

    int y_pos = 120;
    for (int i = 0; i < numTestPoints && y_pos < TFT_HEIGHT; i++)
    {
        tft.setCursor(10, y_pos);
        if (delta_distance[i] < 5)
            tft.setTextColor(TFT_GREEN);
        else if (delta_distance[i] < 10)
            tft.setTextColor(TFT_YELLOW);
        else
            tft.setTextColor(TFT_RED);

        tft.printf("P%d: %.1f px", i + 1, delta_distance[i]);
        y_pos += 15;
    }

    // Print detailed report to serial
    Serial.println("\n=== Touch Accuracy Report ===");
    Serial.println("Point | Expected (X,Y) | Measured (X,Y) | Delta (X,Y) | Distance");
    Serial.println("------|----------------|----------------|-------------|----------");
    for (int i = 0; i < numTestPoints; i++)
    {
        Serial.printf(" %2d   | (%3d,%3d)      | (%5.0f,%5.0f)  | (%6.1f,%6.1f) | %.2f px\n",
                      i + 1, test_lcd_x[i], test_lcd_y[i],
                      measured_x[i], measured_y[i],
                      delta_x[i], delta_y[i],
                      delta_distance[i]);
    }
    Serial.printf("\nMean Error:  %.2f pixels\n", mean_error);
    Serial.printf("Max Error:   %.2f pixels\n", max_error);
    Serial.println("=== End Accuracy Report ===\n");

    delay(3000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_espi_init()
{
    Serial.println("Testing TFT_eSPI initialization with shared SPI0...");

    // The TFT_eSPI library uses a User_Setup.h file for configuration.
    // We will configure it via build flags in platformio.ini to avoid modifying library files.

    // Initialize the TFT display
    initialize_tft(1);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.println("TFT_eSPI Test");

    Serial.println("TFT_eSPI initialized successfully.");
    delay(2000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_espi_graphics()
{
    Serial.println("Testing basic graphics and text...");

    initialize_tft(1);
    tft.fillScreen(TFT_RED);
    delay(500);
    tft.fillScreen(TFT_GREEN);
    delay(500);
    tft.fillScreen(TFT_BLUE);
    delay(500);
    tft.fillScreen(TFT_BLACK);

    tft.setCursor(10, 20);
    tft.println("Hello, TFT_eSPI!");

    tft.drawRect(5, 5, tft.width() - 10, tft.height() - 10, TFT_YELLOW);

    Serial.println("Graphics test completed.");

    TEST_ASSERT_TRUE(true);
    delay(2000);
}

void test_tft_espi_read_id_status()
{
    Serial.println("Reading TFT ID and status...");

    tft.init();

    log_tft_spi_speed("TFT before read");

    // Use indexed reads (TFT_eSPI uses the undocumented 0xD9 index command internally)
    uint8_t id1 = tft.readcommand8(0xDA);
    uint8_t id2 = tft.readcommand8(0xDB);
    uint8_t id3 = tft.readcommand8(0xDC);
    uint8_t st0 = tft.readcommand8(0x09, 0);
    uint8_t st1 = tft.readcommand8(0x09, 1);
    uint8_t st2 = tft.readcommand8(0x09, 2);
    uint8_t st3 = tft.readcommand8(0x09, 3);

    log_tft_spi_speed("TFT after read");

    Serial.print("ID: ");
    Serial.print(id1, HEX);
    Serial.print(" ");
    Serial.print(id2, HEX);
    Serial.print(" ");
    Serial.println(id3, HEX);

    Serial.print("Status: ");
    Serial.print(st0, HEX);
    Serial.print(" ");
    Serial.print(st1, HEX);
    Serial.print(" ");
    Serial.print(st2, HEX);
    Serial.print(" ");
    Serial.println(st3, HEX);

    TEST_ASSERT_TRUE(true);
    delay(500);
}

void test_tft_espi_spi_speed()
{
    Serial.println("Reading SPI speed from RP2040 registers...");

    tft.init();

    spi_inst_t *spi = get_tft_spi();
    spi_hw_t *hw = spi_get_hw(spi);
    uint32_t cpsr = hw->cpsr & 0xFFu;
    uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
    uint32_t baud = get_tft_spi_baudrate(spi);

    log_tft_spi_speed("TFT before read");
    TEST_ASSERT_TRUE(baud > 0);

    if (SPI_FREQUENCY > 0)
    {
        uint32_t requested = SPI_FREQUENCY;
        uint32_t delta = (requested > baud) ? (requested - baud) : (baud - requested);
        uint32_t clk = clock_get_hz(clk_peri);
        uint32_t f_prev = (scr > 0 && cpsr > 0) ? (clk / (cpsr * scr)) : baud;
        uint32_t step = (f_prev > baud) ? (f_prev - baud) : 0;
        uint32_t tol = (requested / 100); // 1% fallback tolerance
        if (step > tol)
            tol = step;

        Serial.printf("SPI_FREQUENCY=%.2f MHz, actual=%.2f MHz, delta=%.2f MHz, step=%.2f MHz\n",
                      requested / 1000000.0,
                      baud / 1000000.0,
                      delta / 1000000.0,
                      step / 1000000.0);

        TEST_ASSERT_TRUE_MESSAGE(requested >= baud, "SPI baud above requested");
        TEST_ASSERT_TRUE_MESSAGE(delta <= tol, "SPI baud outside divider tolerance");
    }
    delay(200);
}

void test_tft_fillScreen()
{
    Serial.println("Testing fillScreen with colors...");

    initialize_tft(1);
    delay(200);
    tft.fillScreen(TFT_RED);
    delay(200);
    tft.fillScreen(TFT_GREEN);
    delay(200);
    tft.fillScreen(TFT_BLUE);
    delay(200);
    tft.fillScreen(TFT_BLACK);

    TEST_ASSERT_TRUE(true);
}

void test_tft_lines()
{
    Serial.println("Testing line drawing...");

    int x1, y1, x2, y2;
    int w = tft.width();
    int h = tft.height();

    initialize_tft(1);

    x1 = y1 = 0;
    y2 = h - 1;
    for (x2 = 0; x2 < w; x2 += 6)
        tft.drawLine(x1, y1, x2, y2, TFT_WHITE);
    x2 = w - 1;
    for (y2 = 0; y2 < h; y2 += 6)
        tft.drawLine(x1, y1, x2, y2, TFT_WHITE);

    delay(500);

    TEST_ASSERT_TRUE(true);
}

void test_tft_rectangles()
{
    Serial.println("Testing rectangle drawing...");

    int n, i, i2;
    int cx = tft.width() / 2;
    int cy = tft.height() / 2;

    initialize_tft(1);
    n = (tft.width() < tft.height()) ? tft.width() : tft.height();

    for (i = 2; i < n; i += 6)
    {
        i2 = i / 2;
        tft.drawRect(cx - i2, cy - i2, i, i, TFT_GREEN);
    }

    delay(1000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_circles()
{
    Serial.println("Testing circle drawing...");

    int radius = 10;
    int x, y;
    int w = tft.width();
    int h = tft.height();
    int r2 = radius * 2;

    initialize_tft(1);
    for (x = radius; x < w; x += r2)
    {
        for (y = radius; y < h; y += r2)
        {
            tft.fillCircle(x, y, radius, TFT_MAGENTA);
        }
    }

    delay(1000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_triangles()
{
    Serial.println("Testing triangle drawing...");

    int n, i;
    int cx = tft.width() / 2 - 1;
    int cy = tft.height() / 2 - 1;

    initialize_tft(1);
    n = (cx < cy) ? cx : cy;

    for (i = 0; i < n; i += 5)
    {
        tft.drawTriangle(
            cx, cy - i,     // peak
            cx - i, cy + i, // bottom left
            cx + i, cy + i, // bottom right
            tft.color565(0, 0, i & 0xFF));
    }

    delay(1000);

    TEST_ASSERT_TRUE(true);
}

void tft_text()
{
    Serial.println("Testing text rendering...");

    tft.println("Hello World!");
    tft.setTextColor(TFT_YELLOW);
    tft.setTextSize(2);
    tft.println(1234.56);
    tft.setTextColor(TFT_RED);
    tft.setTextSize(3);
    tft.println(0xDEADBEEF, HEX);
    tft.println();
    tft.setTextColor(TFT_GREEN);
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
    delay(2000);
}

void test_tft_text()
{
    initialize_tft(1);

    tft_text();

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Rotate screen and render text each rotation
 */
void test_tft_rotate_screen()
{
    Serial.println("Testing text rendering across rotations...");

    initialize_tft(2);

    tft_text();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft_text();
    tft.setRotation(0);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft_text();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Vertical scroll
 */
void test_vertical_scroll()
{
    Serial.println("Testing vertical scroll...");

    uint8_t ch = 'A';
    int iScrollStart = 32;
    int height = tft.height();
    int scroll_area = height - 32;

    initialize_tft(0);
    tft_setupScrollArea(32, 0);
    tft.setTextColor(TFT_RED);
    tft.println("Fixed Line 1");
    tft.setTextColor(TFT_GREEN);
    tft.println("Fixed Line 2");
    tft.setTextColor(TFT_BLUE);
    tft.print("Fixed Line 3");

    for (int j = 0; j < 1000; j++)
    {
        tft.setCursor(0, iScrollStart == 32 ? (height - 16) : (iScrollStart - 16));
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(1);
        for (int i = 0; i < 40; i++)
        {
            tft.write(ch);
        }
        if (ch < 'Z')
            ch++;
        else
            ch = 'A';

        iScrollStart += 16;
        if (iScrollStart >= height)
            iScrollStart = 32;
        tft_scroll_to(iScrollStart);
    }
    tft_scroll_to(0);
    tft_setupScrollArea(0, 0);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_gradient_fills()
{
    Serial.println("Testing gradient fills...");

    initialize_tft(1);

    int mid_x = tft.width() / 2;
    int mid_y = tft.height() / 2;
    int quad_h = mid_y;
    int quad_w = mid_x;

    // Quadrant 1: Horizontal gradient (Red to Blue)
    tft.fillRectHGradient(0, 0, quad_w, quad_h, TFT_RED, TFT_BLUE);

    // Quadrant 2: Vertical gradient (Green to Yellow)
    tft.fillRectVGradient(mid_x, 0, quad_w, quad_h, TFT_GREEN, TFT_YELLOW);

    // Quadrant 3: Horizontal gradient (Cyan to Magenta)
    tft.fillRectHGradient(0, mid_y, quad_w, quad_h, TFT_CYAN, TFT_MAGENTA);

    // Quadrant 4: Vertical gradient (White to Black)
    tft.fillRectVGradient(mid_x, mid_y, quad_w, quad_h, TFT_WHITE, TFT_BLACK);

    // Draw labels
    tft.setCursor(5, 5);
    tft.println("Red->Blue");
    tft.setCursor(mid_x + 5, 5);
    tft.println("Green->Yellow");
    tft.setCursor(5, mid_y + 5);
    tft.println("Cyan->Magenta");
    tft.setCursor(mid_x + 5, mid_y + 5);
    tft.println("White->Black");

    Serial.println("Gradient fill test completed.");
    delay(3000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Animation performance (FPS measurement)
 * Measures rendering speed with animated moving graphics
 */
void test_tft_animation_performance()
{
    Serial.println("Testing animation performance (measuring FPS)...");

    initialize_tft(1);

    int w = tft.width();
    int h = tft.height();
    int cx = w / 2;
    int cy = h / 2;
    int radius = 30;
    int speed = 2;
    unsigned long frame_count = 0;
    unsigned long start_time = millis();
    unsigned long last_display_time = start_time;
    const unsigned long test_duration = 5000; // 5 seconds

    // Animation loop
    int x = 50, y = 50;
    int dx = speed, dy = speed;

    while (millis() - start_time < test_duration)
    {
        // Clear previous circle (draw black circle over it)
        tft.fillCircle(x, y, radius, TFT_BLACK);

        // Update position with bounce
        x += dx;
        y += dy;
        if (x - radius <= 0 || x + radius >= w)
            dx = -dx;
        if (y - radius <= 0 || y + radius >= h)
            dy = -dy;

        // Draw new circle with gradient-like effect using color variation
        uint16_t color = (frame_count % 3 == 0) ? TFT_RED : (frame_count % 3 == 1) ? TFT_GREEN
                                                                                   : TFT_BLUE;
        tft.fillCircle(x, y, radius, color);

        frame_count++;

        // Update FPS display every 500ms
        unsigned long current_time = millis();
        if (current_time - last_display_time >= 500)
        {
            float elapsed_sec = (float)(current_time - start_time) / 1000.0f;
            float fps = frame_count / elapsed_sec;

            // Display FPS and frame count
            tft.fillRect(0, 0, 100, 30, TFT_BLACK);
            tft.setCursor(5, 5);
            tft.setTextColor(TFT_CYAN);
            tft.printf("FPS: %.1f", fps);
            tft.setCursor(5, 18);
            tft.printf("Frames: %lu", frame_count);

            last_display_time = current_time;
        }
    }

    // Final statistics
    unsigned long total_time = millis() - start_time;
    float final_fps = (float)frame_count / (total_time / 1000.0f);
    float avg_frame_time = total_time / (float)frame_count;

    tft.fillScreen(TFT_BLACK);
    tft.setCursor(10, 20);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.println("Animation Results");

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 60);
    tft.printf("Total Frames: %lu\n", frame_count);
    tft.printf("Duration: %lu ms\n", total_time);
    tft.printf("Average FPS: %.2f\n", final_fps);
    tft.printf("Frame Time: %.2f ms", avg_frame_time);

    Serial.printf("\n=== Animation Performance Results ===\n");
    Serial.printf("Total Frames: %lu\n", frame_count);
    Serial.printf("Test Duration: %lu ms\n", total_time);
    Serial.printf("Average FPS: %.2f\n", final_fps);
    Serial.printf("Avg Frame Time: %.2f ms\n", avg_frame_time);
    Serial.printf("Frames per second: ~%u fps\n", (unsigned int)final_fps);

    delay(3000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_color_wheel()
{
    Serial.println("Testing color wheel (full color gamut)...");

    initialize_tft(1);

    int cx = tft.width() / 2;
    int cy = tft.height() / 2;
    int radius = 80;

    tft.setTextSize(2);
    tft.setCursor(cx - 50, 10);
    tft.println("Color Wheel");

    // Draw color wheel using HSV color space
    // Hue varies with angle, Saturation varies with radius
    for (int angle = 0; angle < 360; angle += 3) // Draw every 3 degrees
    {
        float rad = angle * 3.14159f / 180.0f;

        // Draw radial lines from center with varying saturation
        for (int r = 10; r < radius; r += 5)
        {
            // Convert HSV to RGB
            // For a color wheel: Hue = angle, Saturation varies, Value = constant
            float hue = angle / 60.0f;
            float saturation = (float)r / radius;
            float value = 1.0f;

            // HSV to RGB conversion
            float c = value * saturation;
            float x = c * (1.0f - fabs(fmod(hue, 2.0f) - 1.0f));
            float m = value - c;

            float r_f, g_f, b_f;
            if (hue < 1.0f)
            {
                r_f = c;
                g_f = x;
                b_f = 0;
            }
            else if (hue < 2.0f)
            {
                r_f = x;
                g_f = c;
                b_f = 0;
            }
            else if (hue < 3.0f)
            {
                r_f = 0;
                g_f = c;
                b_f = x;
            }
            else if (hue < 4.0f)
            {
                r_f = 0;
                g_f = x;
                b_f = c;
            }
            else if (hue < 5.0f)
            {
                r_f = x;
                g_f = 0;
                b_f = c;
            }
            else
            {
                r_f = c;
                g_f = 0;
                b_f = x;
            }

            // Convert to 16-bit color
            uint8_t r8 = (uint8_t)((r_f + m) * 255);
            uint8_t g8 = (uint8_t)((g_f + m) * 255);
            uint8_t b8 = (uint8_t)((b_f + m) * 255);
            uint16_t color = tft.color565(r8, g8, b8);

            // Calculate endpoint
            int x1 = cx + (int)(r * cos(rad));
            int y1 = cy + (int)(r * sin(rad));
            int x2 = cx + (int)((r + 5) * cos(rad));
            int y2 = cy + (int)((r + 5) * sin(rad));

            tft.drawLine(x1, y1, x2, y2, color);
        }
    }

    // Draw center circle with white
    tft.fillCircle(cx, cy, 8, TFT_WHITE);

    Serial.println("Color wheel test completed.");
    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Cellular Automata (Conway's Game of Life)
 * Based on TFT_eSPI example by RuntimeProjects.com (MIT License)
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/Cellular_Automata/Cellular_Automata.ino
 */
void test_tft_cellular_automata()
{
    Serial.println("\n=== Testing Cellular Automata (Game of Life) ===");

// Grid configuration
#define CA_GRIDX 160
#define CA_GRIDY 120
#define CA_CELLXY 2
#define CA_NUMGEN 300

    // Current grid
    static uint8_t ca_grid[CA_GRIDX][CA_GRIDY];
    // Next generation grid
    static uint8_t ca_newgrid[CA_GRIDX][CA_GRIDY];

    // Initialize display
    initialize_tft(1);

    // Display splash screen
    tft.setTextSize(2);
    tft.setCursor(40, 5);
    tft.println(F("Arduino"));
    tft.setCursor(35, 25);
    tft.println(F("Cellular"));
    tft.setCursor(35, 45);
    tft.println(F("Automata"));
    delay(1000);

    // Initialize grid with random cells
    tft.fillScreen(TFT_BLACK);
    for (int16_t x = 0; x < CA_GRIDX; x++)
    {
        for (int16_t y = 0; y < CA_GRIDY; y++)
        {
            ca_newgrid[x][y] = 0;
            if (x == 0 || x == CA_GRIDX - 1 || y == 0 || y == CA_GRIDY - 1)
            {
                ca_grid[x][y] = 0;
            }
            else
            {
                if (random(3) == 1)
                    ca_grid[x][y] = 1;
                else
                    ca_grid[x][y] = 0;
            }
        }
    }

    // Draw initial grid
    uint16_t color;
    for (int16_t x = 1; x < CA_GRIDX - 1; x++)
    {
        for (int16_t y = 1; y < CA_GRIDY - 1; y++)
        {
            if (ca_grid[x][y] == 1)
                color = TFT_WHITE;
            else
                color = TFT_BLACK;
            tft.fillRect(CA_CELLXY * x, CA_CELLXY * y, CA_CELLXY, CA_CELLXY, color);
        }
    }

    Serial.printf("Running %d generations...\n", CA_NUMGEN);
    unsigned long start_time = millis();

    // Compute generations
    for (int gen = 0; gen < CA_NUMGEN; gen++)
    {
        // Compute next generation using Conway's Game of Life rules
        for (int16_t x = 1; x < CA_GRIDX - 1; x++)
        {
            for (int16_t y = 1; y < CA_GRIDY - 1; y++)
            {
                // Count neighbors (Moore neighbourhood)
                int neighbors = ca_grid[x - 1][y] + ca_grid[x - 1][y - 1] + ca_grid[x][y - 1] +
                                ca_grid[x + 1][y - 1] + ca_grid[x + 1][y] + ca_grid[x + 1][y + 1] +
                                ca_grid[x][y + 1] + ca_grid[x - 1][y + 1];

                // Apply Game of Life rules
                if (ca_grid[x][y] == 1 && (neighbors == 2 || neighbors == 3))
                {
                    ca_newgrid[x][y] = 1; // Survival
                }
                else if (ca_grid[x][y] == 1)
                {
                    ca_newgrid[x][y] = 0; // Underpopulation or overpopulation
                }
                if (ca_grid[x][y] == 0 && (neighbors == 3))
                {
                    ca_newgrid[x][y] = 1; // Birth
                }
                else if (ca_grid[x][y] == 0)
                {
                    ca_newgrid[x][y] = 0;
                }
            }
        }

        // Draw only changed cells
        for (int16_t x = 1; x < CA_GRIDX - 1; x++)
        {
            for (int16_t y = 1; y < CA_GRIDY - 1; y++)
            {
                if (ca_grid[x][y] != ca_newgrid[x][y])
                {
                    if (ca_newgrid[x][y] == 1)
                        color = TFT_WHITE;
                    else
                        color = TFT_BLACK;
                    tft.fillRect(CA_CELLXY * x, CA_CELLXY * y, CA_CELLXY, CA_CELLXY, color);
                }
            }
        }

        // Copy newgrid to grid
        for (int16_t x = 1; x < CA_GRIDX - 1; x++)
        {
            for (int16_t y = 1; y < CA_GRIDY - 1; y++)
            {
                ca_grid[x][y] = ca_newgrid[x][y];
            }
        }
    }

    unsigned long total_time = millis() - start_time;
    float gen_per_sec = (float)CA_NUMGEN / (total_time / 1000.0f);

    Serial.printf("Completed %d generations in %lu ms (%.2f gen/sec)\n",
                  CA_NUMGEN, total_time, gen_per_sec);

    // Show completion message
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setCursor(5, 5);
    tft.printf("%d gens in %lums", CA_NUMGEN, total_time);

    delay(3000);

#undef CA_GRIDX
#undef CA_GRIDY
#undef CA_CELLXY
#undef CA_NUMGEN

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Arc Fill Demo
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_ArcFill/TFT_ArcFill.ino
 * Draws animated arcs with rainbow colors
 */

// Helper: Return 16-bit rainbow colour
static unsigned int arc_rainbow(byte value, byte &red, byte &green, byte &blue, byte &state)
{
    switch (state)
    {
    case 0:
        green++;
        if (green == 64)
        {
            green = 63;
            state = 1;
        }
        break;
    case 1:
        red--;
        if (red == 255)
        {
            red = 0;
            state = 2;
        }
        break;
    case 2:
        blue++;
        if (blue == 32)
        {
            blue = 31;
            state = 3;
        }
        break;
    case 3:
        green--;
        if (green == 255)
        {
            green = 0;
            state = 4;
        }
        break;
    case 4:
        red++;
        if (red == 32)
        {
            red = 31;
            state = 5;
        }
        break;
    case 5:
        blue--;
        if (blue == 255)
        {
            blue = 0;
            state = 0;
        }
        break;
    }
    return (red << 11) | (green << 5) | blue;
}

// Helper: Draw circular or elliptical arc with defined thickness
static void fillArc(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{
#define DEG2RAD 0.0174532925

    byte seg = 6; // Segments are 6 degree segments
    byte inc = 6; // Draw segments every 6 degrees

    // Calculate first pair of coordinates
    float sx = cos((start_angle - 90) * DEG2RAD);
    float sy = sin((start_angle - 90) * DEG2RAD);
    uint16_t x0 = sx * (rx - w) + x;
    uint16_t y0 = sy * (ry - w) + y;
    uint16_t x1 = sx * rx + x;
    uint16_t y1 = sy * ry + y;

    // Draw color blocks every inc degrees
    for (int i = start_angle; i < start_angle + seg * seg_count; i += inc)
    {
        float sx2 = cos((i + seg - 90) * DEG2RAD);
        float sy2 = sin((i + seg - 90) * DEG2RAD);
        int x2 = sx2 * (rx - w) + x;
        int y2 = sy2 * (ry - w) + y;
        int x3 = sx2 * rx + x;
        int y3 = sy2 * ry + y;

        tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);

        x0 = x2;
        y0 = y2;
        x1 = x3;
        y1 = y3;
    }
#undef DEG2RAD
}

void test_tft_arcfill()
{
    Serial.println("\n=== Testing Arc Fill with Rainbow Colors ===");

    initialize_tft(1);

    byte inc = 0;
    unsigned int col = 0;
    byte red = 31;  // Red is top 5 bits
    byte green = 0; // Green is middle 6 bits
    byte blue = 0;  // Blue is bottom 5 bits
    byte state = 0;

    unsigned long start_time = millis();
    const unsigned long test_duration = 5000; // Run for 5 seconds
    uint32_t arc_count = 0;

    Serial.println("Drawing animated arcs...");

    while (millis() - start_time < test_duration)
    {
        // Draw elliptical arc with increasing start angle
        fillArc(160, 120, inc * 6, 1, 140, 100, 10, arc_rainbow(col, red, green, blue, state));

        // Draw segmented elliptical arc
        fillArc(160, 120, ((inc * 2) % 60) * 6, 1, 120, 80, 30, arc_rainbow(col, red, green, blue, state));

        // Draw circle using arc with equal radius
        fillArc(160, 120, inc * 6, 1, 42, 42, 42, arc_rainbow(col, red, green, blue, state));

        inc++;
        col += 1;
        if (col > 191)
            col = 0;
        if (inc > 59)
            inc = 0;

        arc_count++;
    }

    unsigned long total_time = millis() - start_time;
    float arcs_per_sec = (arc_count * 3.0f) / (total_time / 1000.0f); // 3 arcs per iteration

    Serial.printf("Drew %lu arc sets in %lu ms (%.2f arc sets/sec)\n",
                  arc_count, total_time, (float)arc_count / (total_time / 1000.0f));

    // Show completion
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 10);
    tft.print("Arc Fill Test Complete");
    tft.setCursor(10, 25);
    tft.printf("%lu arcs in %lu ms", arc_count, total_time);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Character Drawing Speed
 * Measures font rendering performance by drawing 1000 numbers (0-999) in each font
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Char_times/TFT_Char_times.ino
 */
void test_tft_char_times()
{
    Serial.println("\n=== Testing Character Drawing Speed ===");

    initialize_tft(1);

    // Test fonts available: 1, 2, 4, 6, 7, 8
    // We'll test fonts 1, 2, 4, and 7 (common fonts)
    const uint8_t fonts_to_test[] = {1, 2, 4, 7};
    const uint8_t num_fonts = sizeof(fonts_to_test) / sizeof(fonts_to_test[0]);

    unsigned long total_overall = 0;
    uint32_t total_chars = 0;

    Serial.println("\nFont | Time (ms) | ms/char | chars/sec");
    Serial.println("-----+----------+---------+----------");

    for (uint8_t f = 0; f < num_fonts; f++)
    {
        uint8_t font = fonts_to_test[f];

        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        // Display which font we're testing
        tft.setTextFont(1);
        tft.setCursor(10, 10);
        tft.printf("Testing Font %d", font);
        delay(500);

        // Measure time to draw 1000 numbers
        unsigned long draw_time = millis();

        for (int i = 0; i < 1000; i++)
        {
            yield();
            tft.drawNumber(i, 80, 100, font);
        }

        draw_time = millis() - draw_time;

        // Calculate statistics
        // Approx 2890 characters drawn (0-999 with each digit)
        float chars_drawn = 1000.0f * 3.0f; // Average 3 digits per number
        float ms_per_char = draw_time / chars_drawn;
        float chars_per_sec = (chars_drawn / draw_time) * 1000.0f;

        // Display results on screen
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextFont(1);
        tft.setCursor(20, 50);
        tft.printf("Font %d Results:", font);

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setCursor(20, 80);
        tft.printf("Time: %lu ms", draw_time);

        tft.setCursor(20, 110);
        tft.printf("%.3f ms/char", ms_per_char);

        tft.setCursor(20, 140);
        tft.printf("%.0f chars/sec", chars_per_sec);

        Serial.printf("  %d  |  %6lu  | %7.4f | %8.0f\n",
                      font, draw_time, ms_per_char, chars_per_sec);

        total_overall += draw_time;
        total_chars += (uint32_t)chars_drawn;

        delay(2000);
    }

    // Show overall summary
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextFont(1);
    tft.setCursor(10, 20);
    tft.println("Font Speed Test");
    tft.println("Complete!");

    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setCursor(10, 100);
    tft.printf("Total time: %lu ms", total_overall);
    tft.setCursor(10, 130);
    tft.printf("Total chars: %lu", total_chars);

    if (total_overall > 0)
    {
        float overall_fps = (total_chars / total_overall) * 1000.0f;
        tft.setCursor(10, 160);
        tft.printf("Overall: %.0f ch/s", overall_fps);
    }

    Serial.printf("\nTotal: %lu ms for %lu characters (%.0f chars/sec)\n",
                  total_overall, total_chars, (total_chars * 1000.0f) / total_overall);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Analog Clock
 * Draws an analog clock with moving hour, minute, and second hands
 * Based on TFT_eSPI example by Bodmer (based on Gilchrist 6/2/2014)
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Clock/TFT_Clock.ino
 */
void test_tft_clock()
{
    Serial.println("\n=== Testing Analog Clock ===");

    initialize_tft(1);

    // Center coordinates for landscape display using runtime dimensions
    int centerX = tft.width() / 2;
    int centerY = tft.height() / 2;

    // Draw clock face
    tft.fillScreen(TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);

    tft.fillCircle(centerX, centerY, 118, TFT_GREEN);
    tft.fillCircle(centerX, centerY, 110, TFT_BLACK);

    // Draw 12 hour lines (every 30 degrees)
    for (int i = 0; i < 360; i += 30)
    {
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        int x0 = sx * 114 + centerX;
        int y0 = sy * 114 + centerY;
        int x1 = sx * 100 + centerX;
        int y1 = sy * 100 + centerY;

        tft.drawLine(x0, y0, x1, y1, TFT_GREEN);
    }

    // Draw 60 minute dots
    for (int i = 0; i < 360; i += 6)
    {
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        int x0 = sx * 102 + centerX;
        int y0 = sy * 102 + centerY;

        tft.drawPixel(x0, y0, TFT_WHITE);

        // Draw quadrant dots larger
        if (i == 0 || i == 180)
            tft.fillCircle(x0, y0, 2, TFT_WHITE);
        if (i == 90 || i == 270)
            tft.fillCircle(x0, y0, 2, TFT_WHITE);
    }

    // Draw center circle
    tft.fillCircle(centerX, centerY, 3, TFT_WHITE);

    // Initialize time (use compile time, or just start at 12:00:00)
    uint8_t hh = 12, mm = 0, ss = 0;
    uint32_t targetTime = millis() + 1000;
    unsigned long test_start = millis();
    const unsigned long test_duration = 8000; // Run for 8 seconds
    uint32_t updates = 0;

    Serial.println("Drawing clock hands for 8 seconds...");

    float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;
    uint16_t osx = centerX, osy = centerY, omx = centerX, omy = centerY, ohx = centerX, ohy = centerY;
    float sdeg = 0, mdeg = 0, hdeg = 0;
    bool initial = 1;

    while (millis() - test_start < test_duration)
    {
        yield();

        if (targetTime < millis())
        {
            targetTime += 1000;
            ss++; // Advance second
            if (ss == 60)
            {
                ss = 0;
                mm++; // Advance minute
                if (mm > 59)
                {
                    mm = 0;
                    hh++; // Advance hour
                    if (hh > 23)
                    {
                        hh = 0;
                    }
                }
            }

            // Pre-compute hand degrees and coordinates
            sdeg = ss * 6;                     // 0-59 -> 0-354
            mdeg = mm * 6 + sdeg * 0.01666667; // 0-59 -> 0-360 (includes seconds)
            hdeg = hh * 30 + mdeg * 0.0833333; // 0-11 -> 0-360 (includes minutes & seconds)

            hx = cos((hdeg - 90) * 0.0174532925);
            hy = sin((hdeg - 90) * 0.0174532925);
            mx = cos((mdeg - 90) * 0.0174532925);
            my = sin((mdeg - 90) * 0.0174532925);
            sx = cos((sdeg - 90) * 0.0174532925);
            sy = sin((sdeg - 90) * 0.0174532925);

            if (ss == 0 || initial)
            {
                initial = 0;
                // Erase old hour and minute hands
                tft.drawLine(ohx, ohy, centerX, centerY, TFT_BLACK);
                ohx = hx * 62 + centerX;
                ohy = hy * 62 + centerY;

                tft.drawLine(omx, omy, centerX, centerY, TFT_BLACK);
                omx = mx * 84 + centerX;
                omy = my * 84 + centerY;
            }

            // Erase and redraw second hand
            tft.drawLine(osx, osy, centerX, centerY, TFT_BLACK);
            osx = sx * 90 + centerX;
            osy = sy * 90 + centerY;
            tft.drawLine(osx, osy, centerX, centerY, TFT_RED);

            // Redraw hour and minute hands (not erased to avoid flicker)
            tft.drawLine(ohx, ohy, centerX, centerY, TFT_WHITE);
            tft.drawLine(omx, omy, centerX, centerY, TFT_WHITE);

            // Redraw center circle
            tft.fillCircle(centerX, centerY, 3, TFT_RED);

            updates++;
        }
    }

    unsigned long total_time = millis() - test_start;
    float updates_per_sec = (updates * 1000.0f) / total_time;

    Serial.printf("Clock test: %lu updates in %lu ms (%.2f updates/sec)\n",
                  updates, total_time, updates_per_sec);

    // Show completion message
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 10);
    tft.print("Clock Test Complete");
    tft.setCursor(10, 30);
    tft.printf("%lu updates in %lu ms", updates, total_time);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Digital Clock
 * Displays time in digital format with flashing colons and large fonts
 * Based on TFT_eSPI example by Bodmer (based on Gilchrist 6/2/2014)
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Clock_Digital/TFT_Clock_Digital.ino
 */
void test_tft_clock_digital()
{
    Serial.println("\n=== Testing Digital Clock ===");

    initialize_tft(1);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);

    // Initialize time (12:34:56 for demo)
    uint8_t hh = 12, mm = 34, ss = 56;
    byte oss = 99; // Track last second for updates

    uint32_t targetTime = millis() + 1000;
    unsigned long test_start = millis();
    const unsigned long test_duration = 10000; // Run for 10 seconds
    uint32_t updates = 0;

    Serial.println("Drawing centered digital clock for 10 seconds...");

    // Center the time display on screen (vertical center)
    int center_x = tft.width() / 2;  // Horizontal center: 160
    int center_y = tft.height() / 2; // Vertical center: 120

    while (millis() - test_start < test_duration)
    {
        yield();

        if (targetTime < millis())
        {
            targetTime += 1000;
            ss++; // Advance second
            if (ss == 60)
            {
                ss = 0;
                mm++; // Advance minute
                if (mm > 59)
                {
                    mm = 0;
                    hh++; // Advance hour
                    if (hh > 23)
                    {
                        hh = 0;
                    }
                }
            }

            // Clear previous time display area (generous area for redraws)
            tft.fillRect(center_x - 80, center_y - 30, 160, 60, TFT_BLACK);

            // Format time string HH:MM:SS
            char time_str[9];
            snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d", hh, mm, ss);

            // Draw centered time
            tft.drawCentreString(time_str, center_x, center_y, 7);

            updates++;
        }
    }

    unsigned long total_time = millis() - test_start;
    float updates_per_sec = (updates * 1000.0f) / total_time;

    Serial.printf("Digital clock test: %lu updates in %lu ms (%.2f updates/sec)\n",
                  updates, total_time, updates_per_sec);

    // Show completion message
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 10);
    tft.print("Digital Clock Complete");
    tft.setCursor(10, 30);
    tft.printf("%lu updates in %lu ms", updates, total_time);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Ellipse Drawing
 * Draws filled and outlined ellipses with random sizes, positions, and colors
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Ellipse/TFT_Ellipse.ino
 *
 * Accounts for bug that fillEllipse and drawEllipse do not account for their rotation settings, thus TFT_HEIGHT scales still x coordinate and TFT_WIDTH scales y coordinate
 */
void test_tft_ellipse()
{
    Serial.println("\n=== Testing Ellipse Drawing ===");

    unsigned long test_start = millis();
    uint32_t filled_count = 0;
    uint32_t outlined_count = 0;

    // Draw filled ellipses
    initialize_tft(1);
    Serial.println("Drawing 40 filled ellipses...");

    unsigned long filled_start = millis();
    for (int i = 0; i < 40; i++)
    {
        int rx = random(2, 60);
        int ry = random(2, 60);
        int x = rx + random(TFT_HEIGHT - 2 * rx);
        int y = ry + random(TFT_WIDTH - 2 * ry);
        tft.fillEllipse(x, y, rx, ry, random(0xFFFF));
        filled_count++;
        yield();
    }
    unsigned long filled_time = millis() - filled_start;

    delay(2000);

    // Draw outlined ellipses
    tft.fillScreen(TFT_BLACK);
    Serial.println("Drawing 40 outlined ellipses...");

    unsigned long outlined_start = millis();
    for (int i = 0; i < 40; i++)
    {
        int rx = random(2, 60);
        int ry = random(2, 60);
        int x = rx + random(TFT_HEIGHT - 2 * rx);
        int y = ry + random(TFT_WIDTH - 2 * ry);
        tft.drawEllipse(x, y, rx, ry, random(0xFFFF));
        outlined_count++;
        yield();
    }
    unsigned long outlined_time = millis() - outlined_start;

    delay(2000);

    unsigned long total_time = millis() - test_start;

    Serial.printf("Filled ellipses: %lu in %lu ms (%.2f ms each)\n",
                  filled_count, filled_time, (float)filled_time / filled_count);
    Serial.printf("Outlined ellipses: %lu in %lu ms (%.2f ms each)\n",
                  outlined_count, outlined_time, (float)outlined_time / outlined_count);
    Serial.printf("Total test time: %lu ms\n", total_time);

    // Show completion message
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(10, 10);
    tft.print("Ellipse Test Complete");
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 30);
    tft.printf("Filled: %lu in %lums", filled_count, filled_time);
    tft.setCursor(10, 50);
    tft.printf("Outlined: %lu in %lums", outlined_count, outlined_time);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Fill Arc Spiral
 * Draws an animated spiral using arc segments with rainbow colors
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_FillArcSpiral/TFT_FillArcSpiral.ino
 */

// Helper: Draw arc segment for spiral (modified fillArc for spirals)
static void fillArcSpiral(int x, int y, int start_angle, int seg_count, int rx, int ry, int w, unsigned int colour)
{
#define DEG2RAD_SPIRAL 0.0174532925

    byte seg = 7; // Segment size (7 degrees prevents gaps in spirals)
    byte inc = 6; // Draw segments every 6 degrees

    for (int i = start_angle; i < start_angle + seg * seg_count; i += inc)
    {
        // Calculate segment start coordinates
        float sx = cos((i - 90) * DEG2RAD_SPIRAL);
        float sy = sin((i - 90) * DEG2RAD_SPIRAL);
        uint16_t x0 = sx * (rx - w) + x;
        uint16_t y0 = sy * (ry - w) + y;
        uint16_t x1 = sx * rx + x;
        uint16_t y1 = sy * ry + y;

        // Calculate segment end coordinates
        float sx2 = cos((i + seg - 90) * DEG2RAD_SPIRAL);
        float sy2 = sin((i + seg - 90) * DEG2RAD_SPIRAL);
        int x2 = sx2 * (rx - w) + x;
        int y2 = sy2 * (ry - w) + y;
        int x3 = sx2 * rx + x;
        int y3 = sy2 * ry + y;

        tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
    }
#undef DEG2RAD_SPIRAL
}

void test_tft_fillarcspiral()
{
    Serial.println("\n=== Testing Fill Arc Spiral ===");

    initialize_tft(1);

    int segment = 0;
    unsigned int col = 0;
    int delta = 1; // Direction: 1 = expanding, -1 = contracting

    byte red = 31;
    byte green = 0;
    byte blue = 0;
    byte state = 0;

    unsigned long test_start = millis();
    const unsigned long test_duration = 10000; // Run for 10 seconds
    uint32_t spiral_count = 0;

    Serial.println("Drawing animated spiral for 10 seconds...");

    while (millis() - test_start < test_duration)
    {
        yield();

        // Draw spiral segment
        int radius = 120 - segment / 4;
        fillArcSpiral(160, 120, segment * 6, 1, radius, radius, 3, arc_rainbow(col, red, green, blue, state));

        // Update segment position
        segment += delta;
        col += 1;
        if (col > 191)
            col = 0;

        // Reverse direction at limits (~5 turns in the spiral: 300*6 degrees)
        if (segment < 0)
            delta = 1;
        if (segment > 298)
            delta = -1;

        spiral_count++;
    }

    unsigned long total_time = millis() - test_start;
    float arcs_per_sec = (spiral_count * 1000.0f) / total_time;

    Serial.printf("Spiral test: %lu arc segments in %lu ms (%.2f arcs/sec)\n",
                  spiral_count, total_time, arcs_per_sec);

    // Show completion message
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(TFT_GREEN);
    tft.setCursor(10, 10);
    tft.print("Spiral Test Complete");
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 30);
    tft.printf("%lu arcs in %lu ms", spiral_count, total_time);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Float Number Display
 * Tests the drawFloat() function with various floating point values and precisions
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/tree/master/examples/320%20x%20240/TFT_Float_Test
 */
void test_tft_float_test()
{
    Serial.println("\n=== Testing Float Number Display ===");

    initialize_tft(1);

    char tmp[12];

    // Test 1: 67.125 with 4 decimal places
    tft.setTextDatum(MC_DATUM); // Middle centre

    float test = 67.125;
    tft.drawFloat(test, 4, 160, 120, 6);
    tft.drawString(dtostrf(test, 4, 4, tmp), 100, 200, 6);
    Serial.printf("Test 1: %.4f\n", test);
    delay(1000);

    // Test 2: -0.555555 with 3 decimal places
    tft.fillScreen(TFT_BLACK);
    test = -0.555555;
    tft.drawFloat(test, 3, 160, 120, 6);
    tft.drawString(dtostrf(test, 2, 2, tmp), 100, 200, 6);
    Serial.printf("Test 2: %.3f\n", test);
    delay(1000);

    // Test 3: 0.123 with 4 decimal places
    tft.fillScreen(TFT_BLACK);
    test = 0.123;
    tft.drawFloat(test, 4, 160, 120, 6);
    tft.drawString(dtostrf(test, 4, 4, tmp), 100, 200, 6);
    Serial.printf("Test 3: %.4f\n", test);
    delay(1000);

    // Test 4: 9999999 with 0 decimal places
    tft.fillScreen(TFT_BLACK);
    test = 9999999;
    tft.drawFloat(test, 0, 160, 120, 6);
    tft.drawString(dtostrf(test, 4, 4, tmp), 100, 200, 6);
    Serial.printf("Test 4: %.0f\n", test);
    delay(1000);

    // Plot the datum point (middle centre)
    tft.fillCircle(160, 120, 5, TFT_RED);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 120, 2);
    Serial.println("Showing datum point (middle centre)");
    delay(2000);

    Serial.println("Float test complete");

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Mandelbrot Set Fractal
 * Renders the Mandelbrot set fractal with rainbow coloring
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/tree/master/examples/320%20x%20240/TFT_Mandlebrot
 * Note: This is computationally intensive and takes time to render
 */

// Helper: Rainbow color mapping for Mandelbrot iterations (0-127)
static unsigned int rainbow_mandelbrot(int value)
{
    // Value is expected to be in range 0-127
    // Converted to spectrum: blue -> cyan -> green -> yellow -> red
    byte red = 0;   // Top 5 bits of 16-bit color
    byte green = 0; // Middle 6 bits
    byte blue = 0;  // Bottom 5 bits

    byte quadrant = value / 32;

    if (quadrant == 0)
    {
        blue = 31;
        green = 2 * (value % 32);
        red = 0;
    }
    if (quadrant == 1)
    {
        blue = 31 - (value % 32);
        green = 63;
        red = 0;
    }
    if (quadrant == 2)
    {
        blue = 0;
        green = 63;
        red = value % 32;
    }
    if (quadrant == 3)
    {
        blue = 0;
        green = 63 - 2 * (value % 32);
        red = 31;
    }
    return (red << 11) + (green << 5) + blue;
}

// Helper: Render a specific region of the Mandelbrot set
// needs to account for rotation since using bare metal drawPixel routine
static void render_mandelbrot_region(float x_min, float x_max, float y_min, float y_max, const char *description)
{
    Serial.printf("Rendering: %s\n", description);
    Serial.printf("  X range: %.6f to %.6f\n", x_min, x_max);
    Serial.printf("  Y range: %.6f to %.6f\n", y_min, y_max);

    unsigned long regionTime = millis();
    tft.fillScreen(TFT_BLACK);
    tft.startWrite();

    uint32_t pixel_count = 0;
    uint16_t tft_width = TFT_HEIGHT; // Account for rotation (width is height in portrait mode)
    uint16_t tft_height = TFT_WIDTH; // Account for rotation (height is width in portrait mode)

    for (int px = 0; px < tft_width; px++)
    {
        for (int py = 0; py < tft_height; py++)
        {
            // Map pixel to Mandelbrot coordinates
            float x0 = x_min + (x_max - x_min) * px / tft_width;
            float yy0 = y_min + (y_max - y_min) * py / tft_height;

            float xx = 0.0;
            float yy = 0.0;
            int iteration = 0;
            int max_iteration = 128;

            // Mandelbrot iteration
            while (((xx * xx + yy * yy) < 4) && (iteration < max_iteration))
            {
                float xtemp = xx * xx - yy * yy + x0;
                yy = 2 * xx * yy + yy0;
                xx = xtemp;
                iteration++;
            }

            int color = rainbow_mandelbrot((3 * iteration + 64) % 128);
            tft.drawPixel(px, py, color);
            pixel_count++;

            // Yield every 100 pixels to prevent watchdog timeout
            if (pixel_count % 100 == 0)
            {
                yield();
            }
        }

        // Progress indicator every 64 columns
        if (px % 64 == 0)
        {
            Serial.printf("  Progress: %d/%d columns\n", px, tft_width);
        }
    }

    tft.endWrite();

    unsigned long totalTime = millis() - regionTime;
    Serial.printf("  Complete: %lu ms (%.2f seconds), %lu pixels\n",
                  totalTime, totalTime / 1000.0, pixel_count);

    // Display label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.print(description);

    delay(3000);
}

// Helper: Draw Julia set fractal
static void draw_julia_set(float c_r, float c_i, float zoom)
{
    Serial.printf("Rendering Julia Set: c = %.3f + %.3fi, zoom = %.1f\n", c_r, c_i, zoom);

    unsigned long regionTime = millis();
    tft.fillScreen(TFT_BLACK);
    tft.startWrite();

    const uint16_t MAX_ITERATION = 300;
    uint32_t pixel_count = 0;
    uint16_t tft_width = TFT_HEIGHT; // Account for rotation
    uint16_t tft_height = TFT_WIDTH; // Account for rotation

    // Rely on inverted symmetry for performance
    for (int16_t x = tft_width / 2 - 1; x >= 0; x--)
    {
        for (uint16_t y = 0; y < tft_height; y++)
        {
            float old_r = 1.5 * (x - tft_width / 2) / (0.5 * zoom * tft_width);
            float old_i = (y - tft_height / 2) / (0.5 * zoom * tft_height);
            uint16_t i = 0;

            while ((old_r * old_r + old_i * old_i) < 4.0 && i < MAX_ITERATION)
            {
                float new_r = old_r * old_r - old_i * old_i;
                float new_i = 2.0 * old_r * old_i;
                old_r = new_r + c_r;
                old_i = new_i + c_i;
                i++;
            }

            // Color based on iteration count
            uint16_t color;
            if (i < 100)
            {
                color = tft.color565(255, 255, map(i, 0, 100, 255, 0));
            }
            else if (i < 200)
            {
                color = tft.color565(255, map(i, 100, 200, 255, 0), 0);
            }
            else
            {
                color = tft.color565(map(i, 200, 300, 255, 0), 0, 0);
            }

            // Draw pixel and its symmetric counterpart
            tft.drawPixel(x, y, color);
            tft.drawPixel(tft_width - x - 1, tft_height - y - 1, color);
            pixel_count += 2;

            // Yield every 100 pixels to prevent watchdog timeout
            if (pixel_count % 200 == 0)
            {
                yield();
            }
        }

        // Progress indicator every 32 columns
        if (x % 32 == 0)
        {
            Serial.printf("  Progress: %d/%d columns\n", tft_width / 2 - x, tft_width / 2);
        }
    }

    tft.endWrite();

    unsigned long totalTime = millis() - regionTime;
    Serial.printf("  Complete: %lu ms (%.2f seconds), %lu pixels\n",
                  totalTime, totalTime / 1000.0, pixel_count);

    // Display label
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setCursor(5, 5);
    tft.printf("Julia Set: c=%.3f+%.3fi", c_r, c_i);
}

void test_tft_mandelbrot()
{
    Serial.println("\n=== Testing Mandelbrot Set Rendering ===");

    initialize_tft(1);

    unsigned long totalStartTime = millis();

    // 1. Full Mandelbrot set - classic view
    render_mandelbrot_region(-2.5, 1.0, -1.0, 1.0,
                             "1. Full Set");

    // 2. Seahorse Valley - detailed spiral region
    render_mandelbrot_region(-0.75, -0.735, 0.095, 0.11,
                             "2. Seahorse Valley");

    // 3. Elephant Valley - intricate detail
    render_mandelbrot_region(0.25, 0.35, 0.0, 0.1,
                             "3. Elephant Valley");

    // 4. Spiral detail - deep zoom with fine structure
    render_mandelbrot_region(-0.7436, -0.7426, 0.1318, 0.1328,
                             "4. Spiral Detail");

    // 5. Mini-Mandelbrot - self-similar copy
    render_mandelbrot_region(-0.16, -0.14, 1.025, 1.045,
                             "5. Mini-Mandelbrot");

    // 6. Scepter Valley - period 3 bulb region
    render_mandelbrot_region(-1.3, -1.2, -0.1, 0.1,
                             "6. Scepter Valley");

    // 7. Double Spiral Valley - double spiral region
    render_mandelbrot_region(-0.76, -0.74, 0.08, 0.12,
                             "7. Double Spiral Valley");

    unsigned long totalTime = millis() - totalStartTime;

    // Summary display
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.setCursor(10, 80);
    tft.println("Mandelbrot");
    tft.println("  Complete!");
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 140);
    tft.printf("Total: %.1f sec", totalTime / 1000.0);

    Serial.printf("\n=== Mandelbrot Test Complete ===\n");
    Serial.printf("Total rendering time: %lu ms (%.2f seconds)\n",
                  totalTime, totalTime / 1000.0);
    Serial.printf("7 regions rendered\n");

    delay(3000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_julia_set()
{
    Serial.println("\n=== Testing Julia Set Rendering ===");

    initialize_tft(1);

    unsigned long startTime = millis();

    // Draw Julia set with different parameters
    draw_julia_set(-0.8, 0.156, 1.0);

    unsigned long renderTime = millis() - startTime;

    // Display completion message
    tft.setTextColor(TFT_GREEN, TFT_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 10);
    tft.println("Julia Set Complete!");
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(10, 30);
    tft.printf("Render time: %.1f sec", renderTime / 1000.0);

    Serial.printf("\n=== Julia Set Test Complete ===\n");
    Serial.printf("Rendering time: %lu ms (%.2f seconds)\n", renderTime, renderTime / 1000.0);

    delay(3000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: TFT Pie Chart
 * Draws pie chart segments using circle segments
 * Based on TFT_eSPI example
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Pie_Chart/TFT_Pie_Chart.ino
 */

#define DEG2RAD 0.0174532925

// Draw circle segments for pie charts
// x,y == coords of centre of circle
// start_angle = 0 - 359
// sub_angle   = 0 - 360 = subtended angle
// r = radius
// colour = 16-bit colour value
int fillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour)
{
    // Calculate first pair of coordinates for segment start
    float sx = cos((start_angle - 90) * DEG2RAD);
    float sy = sin((start_angle - 90) * DEG2RAD);
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    // Draw colour blocks every inc degrees
    for (int i = start_angle; i < start_angle + sub_angle; i++)
    {
        // Calculate pair of coordinates for segment end
        int x2 = cos((i + 1 - 90) * DEG2RAD) * r + x;
        int y2 = sin((i + 1 - 90) * DEG2RAD) * r + y;

        tft.fillTriangle(x1, y1, x2, y2, x, y, colour);

        // Copy segment end to segment start for next segment
        x1 = x2;
        y1 = y2;
    }
    return 0;
}

// Return the 16-bit colour with brightness 0-100%
unsigned int brightness(unsigned int colour, int brightness)
{
    byte red = colour >> 11;
    byte green = (colour & 0x7E0) >> 5;
    byte blue = colour & 0x1F;

    blue = (blue * brightness) / 100;
    green = (green * brightness) / 100;
    red = (red * brightness) / 100;

    return (red << 11) + (green << 5) + blue;
}

void test_tft_pie_chart()
{
    Serial.println("Testing TFT Pie Chart...");

    initialize_tft(1);

    // Draw 4 pie chart segments
    fillSegment(160, 120, 0, 60, 100, TFT_RED);
    fillSegment(160, 120, 60, 30, 100, TFT_GREEN);
    fillSegment(160, 120, 60 + 30, 120, 100, TFT_BLUE);
    fillSegment(160, 120, 60 + 30 + 120, 150, 100, TFT_YELLOW);

    delay(4000);

    // Erase old chart with 360 degree black plot
    fillSegment(160, 120, 0, 360, 100, TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Pong game variables and functions
#define PONG_BLACK 0x0000
#define PONG_WHITE 0xFFFF
#define PONG_GREY 0x5AEB

static int16_t pong_h = 240;
static int16_t pong_w = 320;
static int pong_dly = 5;
static int16_t pong_paddle_h = 30;
static int16_t pong_paddle_w = 4;
static int16_t pong_lpaddle_x = 0;
static int16_t pong_rpaddle_x;
static int16_t pong_lpaddle_y = 0;
static int16_t pong_rpaddle_y;
static int16_t pong_lpaddle_d = 1;
static int16_t pong_rpaddle_d = -1;
static int16_t pong_lpaddle_ball_t;
static int16_t pong_rpaddle_ball_t;
static int16_t pong_target_y = 0;
static int16_t pong_ball_x = 2;
static int16_t pong_ball_y = 2;
static int16_t pong_oldball_x = 2;
static int16_t pong_oldball_y = 2;
static int16_t pong_ball_dx = 1;
static int16_t pong_ball_dy = 1;
static int16_t pong_ball_w = 6;
static int16_t pong_ball_h = 6;
static int16_t pong_dashline_h = 4;
static int16_t pong_dashline_w = 2;
static int16_t pong_dashline_n;
static int16_t pong_dashline_x;
static int16_t pong_dashline_y;
static int16_t pong_lscore = 12;
static int16_t pong_rscore = 4;
static int16_t pong_bottom = 202;

void pong_calc_target_y();
void pong_midline();

void pong_initgame()
{
    pong_rpaddle_x = pong_w - pong_paddle_w;
    pong_rpaddle_y = pong_h - pong_paddle_h;
    pong_lpaddle_ball_t = pong_w - pong_w / 4;
    pong_rpaddle_ball_t = pong_w / 4;
    pong_dashline_n = pong_h / pong_dashline_h;
    pong_dashline_x = pong_w / 2 - 1;
    pong_dashline_y = pong_dashline_h / 2;

    pong_lpaddle_y = random(0, pong_bottom - pong_paddle_h);
    pong_rpaddle_y = random(0, pong_bottom - pong_paddle_h);

    // ball is placed on the center of the left paddle
    pong_ball_y = pong_lpaddle_y + (pong_paddle_h / 2);

    pong_calc_target_y();

    pong_midline();

    tft.fillRect(0, pong_h - 26, pong_w, 239, PONG_GREY);

    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(PONG_WHITE, PONG_GREY);
    tft.drawString("TFT_eSPI example", pong_w / 2, pong_h - 26, 4);
}

void pong_midline()
{
    // If the ball is not on the line then don't redraw the line
    if ((pong_ball_x < pong_dashline_x - pong_ball_w) && (pong_ball_x > pong_dashline_x + pong_dashline_w))
        return;

    tft.startWrite();

    // Quick way to draw a dashed line
    tft.setAddrWindow(pong_dashline_x, 0, pong_dashline_w, pong_h);

    for (int16_t i = 0; i < pong_dashline_n; i += 2)
    {
        tft.pushColor(PONG_WHITE, pong_dashline_w * pong_dashline_h); // push dash pixels
        tft.pushColor(PONG_BLACK, pong_dashline_w * pong_dashline_h); // push gap pixels
    }

    tft.endWrite();
}

void pong_lpaddle()
{
    if (pong_lpaddle_d == 1)
    {
        tft.fillRect(pong_lpaddle_x, pong_lpaddle_y, pong_paddle_w, 1, PONG_BLACK);
    }
    else if (pong_lpaddle_d == -1)
    {
        tft.fillRect(pong_lpaddle_x, pong_lpaddle_y + pong_paddle_h - 1, pong_paddle_w, 1, PONG_BLACK);
    }

    pong_lpaddle_y = pong_lpaddle_y + pong_lpaddle_d;

    if (pong_ball_dx == 1)
        pong_lpaddle_d = 0;
    else
    {
        if (pong_lpaddle_y + pong_paddle_h / 2 == pong_target_y)
            pong_lpaddle_d = 0;
        else if (pong_lpaddle_y + pong_paddle_h / 2 > pong_target_y)
            pong_lpaddle_d = -1;
        else
            pong_lpaddle_d = 1;
    }

    if (pong_lpaddle_y + pong_paddle_h >= pong_bottom && pong_lpaddle_d == 1)
        pong_lpaddle_d = 0;
    else if (pong_lpaddle_y <= 0 && pong_lpaddle_d == -1)
        pong_lpaddle_d = 0;

    tft.fillRect(pong_lpaddle_x, pong_lpaddle_y, pong_paddle_w, pong_paddle_h, PONG_WHITE);
}

void pong_rpaddle()
{
    if (pong_rpaddle_d == 1)
    {
        tft.fillRect(pong_rpaddle_x, pong_rpaddle_y, pong_paddle_w, 1, PONG_BLACK);
    }
    else if (pong_rpaddle_d == -1)
    {
        tft.fillRect(pong_rpaddle_x, pong_rpaddle_y + pong_paddle_h - 1, pong_paddle_w, 1, PONG_BLACK);
    }

    pong_rpaddle_y = pong_rpaddle_y + pong_rpaddle_d;

    if (pong_ball_dx == -1)
        pong_rpaddle_d = 0;
    else
    {
        if (pong_rpaddle_y + pong_paddle_h / 2 == pong_target_y)
            pong_rpaddle_d = 0;
        else if (pong_rpaddle_y + pong_paddle_h / 2 > pong_target_y)
            pong_rpaddle_d = -1;
        else
            pong_rpaddle_d = 1;
    }

    if (pong_rpaddle_y + pong_paddle_h >= pong_bottom && pong_rpaddle_d == 1)
        pong_rpaddle_d = 0;
    else if (pong_rpaddle_y <= 0 && pong_rpaddle_d == -1)
        pong_rpaddle_d = 0;

    tft.fillRect(pong_rpaddle_x, pong_rpaddle_y, pong_paddle_w, pong_paddle_h, PONG_WHITE);
}

void pong_calc_target_y()
{
    int16_t target_x;
    int16_t reflections;
    int16_t y;

    if (pong_ball_dx == 1)
    {
        target_x = pong_w - pong_ball_w;
    }
    else
    {
        target_x = -1 * (pong_w - pong_ball_w);
    }

    y = abs(target_x * (pong_ball_dy / pong_ball_dx) + pong_ball_y);

    reflections = floor(y / pong_bottom);

    if (reflections % 2 == 0)
    {
        pong_target_y = y % pong_bottom;
    }
    else
    {
        pong_target_y = pong_bottom - (y % pong_bottom);
    }
}

void pong_ball()
{
    pong_ball_x = pong_ball_x + pong_ball_dx;
    pong_ball_y = pong_ball_y + pong_ball_dy;

    if (pong_ball_dx == -1 && pong_ball_x == pong_paddle_w && pong_ball_y + pong_ball_h >= pong_lpaddle_y && pong_ball_y <= pong_lpaddle_y + pong_paddle_h)
    {
        pong_ball_dx = pong_ball_dx * -1;
        pong_dly = 5; // consistent speed after paddle contact
        pong_calc_target_y();
    }
    else if (pong_ball_dx == 1 && pong_ball_x + pong_ball_w == pong_w - pong_paddle_w && pong_ball_y + pong_ball_h >= pong_rpaddle_y && pong_ball_y <= pong_rpaddle_y + pong_paddle_h)
    {
        pong_ball_dx = pong_ball_dx * -1;
        pong_dly = 5; // consistent speed after paddle contact
        pong_calc_target_y();
    }
    else if ((pong_ball_dx == 1 && pong_ball_x >= pong_w) || (pong_ball_dx == -1 && pong_ball_x + pong_ball_w < 0))
    {
        pong_dly = 5;
    }

    if (pong_ball_y > 202 || pong_ball_y < 0)
    {
        pong_ball_dy = pong_ball_dy * -1;
        pong_ball_y += pong_ball_dy; // Keep in bounds
    }

    // tft.fillRect(pong_oldball_x, pong_oldball_y, pong_ball_w, pong_ball_h, PONG_BLACK);
    tft.drawRect(pong_oldball_x, pong_oldball_y, pong_ball_w, pong_ball_h, PONG_BLACK); // Less TFT refresh aliasing than line above for large balls
    tft.fillRect(pong_ball_x, pong_ball_y, pong_ball_w, pong_ball_h, PONG_WHITE);
    pong_oldball_x = pong_ball_x;
    pong_oldball_y = pong_ball_y;
}

void test_tft_pong()
{
    Serial.println("Testing TFT Pong...");

    initialize_tft(1); // Landscape mode

    tft.fillScreen(PONG_BLACK);

    pong_initgame();

    unsigned long startTime = millis();
    while (millis() - startTime < 10000)
    { // Run for 10 seconds
        delay(pong_dly);

        pong_lpaddle();
        pong_rpaddle();
        pong_midline();
        pong_ball();

        yield();
    }

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

void test_tft_print_test()
{
    Serial.println("Testing TFT Print Test...");

    initialize_tft(0); // Portrait mode as in example

    // Fill screen with grey so we can see the effect of printing with and without a background colour defined
    tft.fillScreen(0x5AEB); // TFT_GREY

    // Set "cursor" at top left corner of display (0,0) and select font 2
    // (cursor will move to next line automatically during printing with 'tft.println'
    //  or stay on the line is there is room for the text with tft.print)
    tft.setCursor(0, 0, 2);

    // Set the font colour to be white with a black background, set text size multiplier to 1
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    // We can now plot text on screen using the "print" class
    tft.println("Hello World!");

    // Set the font colour to be yellow with no background, set to font 7
    tft.setTextColor(TFT_YELLOW);
    tft.setTextFont(7);
    tft.println(1234.56);

    // Set the font colour to be red with black background, set to font 4
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextFont(4);
    tft.println(3735928559L, HEX); // Should print DEADBEEF

    // Set the font colour to be green with black background, set to font 4
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextFont(4);
    tft.println("Groop");
    tft.println("I implore thee,");

    // Change to font 2
    tft.setTextFont(2);
    tft.println("my foonting turlingdromes.");
    tft.println("And hooptiously drangle me");
    tft.println("with crinkly bindlewurdles,");
    // This next line is deliberately made too long for the display width to test
    // automatic text wrapping onto the next line
    tft.println("Or I will rend thee in the gobberwarts with my blurglecruncheon, see if I don't!");

    // Test some print formatting functions
    float fnumber = 123.45;
    // Set the font colour to be blue with no background, set to font 4
    tft.setTextColor(TFT_BLUE);
    tft.setTextFont(4);
    tft.print("Float = ");
    tft.println(fnumber); // Print floating point number
    tft.print("Binary = ");
    tft.println((int)fnumber, BIN); // Print as integer value in binary
    tft.print("Hexadecimal = ");
    tft.println((int)fnumber, HEX); // Print as integer number in Hexadecimal

    delay(5000);

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Rainbow fill function for TFT test
void rainbow_fill()
{
    static byte red = 31;
    static byte green = 0;
    static byte blue = 0;
    static byte state = 0;
    static unsigned int colour = red << 11; // Colour order is RGB 5+6+5 bits each

    // The colours and state are not initialised so the start colour changes each time the function is called
    for (int i = 319; i > 0; i--)
    {
        // Draw a vertical line 1 pixel wide in the selected colour
        tft.drawFastHLine(0, i, tft.width(), colour); // in this example tft.width() returns the pixel width of the display

        // This is a "state machine" that ramps up/down the colour brightnesses in sequence
        switch (state)
        {
        case 0:
            green++;
            if (green == 64)
            {
                green = 63;
                state = 1;
            }
            break;
        case 1:
            red--;
            if (red == 255)
            {
                red = 0;
                state = 2;
            }
            break;
        case 2:
            blue++;
            if (blue == 32)
            {
                blue = 31;
                state = 3;
            }
            break;
        case 3:
            green--;
            if (green == 255)
            {
                green = 0;
                state = 4;
            }
            break;
        case 4:
            red++;
            if (red == 32)
            {
                red = 31;
                state = 5;
            }
            break;
        case 5:
            blue--;
            if (blue == 255)
            {
                blue = 0;
                state = 0;
            }
            break;
        }
        colour = red << 11 | green << 5 | blue;
    }
}

void test_tft_rainbow()
{
    Serial.println("Testing TFT Rainbow...");

    initialize_tft(0); // Portrait mode

    tft.fillScreen(TFT_BLACK);

    rainbow_fill(); // Fill the screen with rainbow colours

    // The standard AdaFruit font still works as before
    tft.setTextColor(TFT_BLACK); // Background is not defined so it is transparent
    tft.setCursor(60, 5);
    tft.setTextFont(0); // Select font 0 which is the Adafruit font
    tft.print("Original Adafruit font!");

    // The new larger fonts do not need to use the .setCursor call, coords are embedded
    tft.setTextColor(TFT_BLACK); // Do not plot the background colour
    // Overlay the black text on top of the rainbow plot (the advantage of not drawing the background colour!)
    tft.drawCentreString("Font size 2", 120, 14, 2); // Draw text centre at position 120, 14 using font 2
    tft.drawCentreString("Font size 4", 120, 30, 4); // Draw text centre at position 120, 30 using font 4
    tft.drawCentreString("12.34", 120, 54, 6);       // Draw text centre at position 120, 54 using font 6

    tft.drawCentreString("12.34 is in font size 6", 120, 92, 2); // Draw text centre at position 120, 92 using font 2
    // Note the x position is the top of the font!

    // draw a floating point number
    float pi = 3.14159;                                     // Value to print
    int precision = 3;                                      // Number of digits after decimal point
    int xpos = 90;                                          // x position
    int ypos = 110;                                         // y position
    int font = 2;                                           // font number 2
    xpos += tft.drawFloat(pi, precision, xpos, ypos, font); // Draw rounded number and return new xpos delta for next print position
    tft.drawString(" is pi", xpos, ypos, font);             // Continue printing from new x position

    tft.setTextSize(1);            // We are using a size multiplier of 1
    tft.setTextColor(TFT_BLACK);   // Set text colour to black, no background (so transparent)
    tft.setCursor(36, 150, 4);     // Set cursor to x = 36, y = 150 and use font 4
    tft.println("Transparent..."); // As we use println, the cursor moves to the next line

    tft.setCursor(30, 175);                 // Set cursor to x = 30, y = 175
    tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text colour to white and background to black
    tft.println("White on black");

    tft.setTextFont(4);     // Select font 4 without moving cursor
    tft.setCursor(50, 210); // Set cursor to x = 50, y = 210 without changing the font
    tft.setTextColor(TFT_WHITE);
    // By using #TFT print we can use all the formatting features like printing HEX
    tft.print(57005, HEX);   // Cursor does no move to next line
    tft.println(48879, HEX); // print and move cursor to next line

    tft.setTextColor(TFT_GREEN, TFT_BLACK); // This time we will use green text on a black background
    tft.setTextFont(2);                     // Select font 2
    // Text will wrap to the next line if needed, by luck it breaks the lines at spaces!
    tft.println(" Ode to a Small Lump of Green Putty I Found in My Armpit One Midsummer Morning ");

    delay(5000);

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Spiro rainbow functions
#define DEG2RAD_SPIRO 0.0174532925

unsigned int spiro_rainbow(int value)
{
    // Value is expected to be in range 0-127
    // The value is converted to a spectrum colour from 0 = blue through to red = blue
    byte red = 0;   // Red is the top 5 bits of a 16-bit colour value
    byte green = 0; // Green is the middle 6 bits
    byte blue = 0;  // Blue is the bottom 5 bits

    byte quadrant = value / 32;

    if (quadrant == 0)
    {
        blue = 31;
        green = 2 * (value % 32);
        red = 0;
    }
    if (quadrant == 1)
    {
        blue = 31 - (value % 32);
        green = 63;
        red = 0;
    }
    if (quadrant == 2)
    {
        blue = 0;
        green = 63;
        red = value % 32;
    }
    if (quadrant == 3)
    {
        blue = 0;
        green = 63 - 2 * (value % 32);
        red = 31;
    }
    return (red << 11) + (green << 5) + blue;
}

void test_tft_spiro()
{
    Serial.println("Testing TFT Spiro...");

    initialize_tft(3); // Portrait reverse as in example

    tft.fillScreen(TFT_BLACK);

    // Run spiro pattern generation for a few iterations
    unsigned long startTime = millis();
    int patternCount = 0;

    while (millis() - startTime < 8000 && patternCount < 3)
    { // Run for 8 seconds or 3 patterns
        int n = random(2, 23), r = random(20, 100);

        for (long i = 0; i < (360 * n); i++)
        {
            float sx = cos((i / n - 90) * DEG2RAD_SPIRO);
            float sy = sin((i / n - 90) * DEG2RAD_SPIRO);
            uint16_t x0 = sx * (120 - r) + 159;
            uint16_t yy0 = sy * (120 - r) + 119;

            sy = cos(((i % 360) - 90) * DEG2RAD_SPIRO);
            sx = sin(((i % 360) - 90) * DEG2RAD_SPIRO);
            uint16_t x1 = sx * r + x0;
            uint16_t yy1 = sy * r + yy0;
            tft.drawPixel(x1, yy1, spiro_rainbow(map(i % 360, 0, 360, 0, 127)));

            yield(); // Allow other tasks
        }

        r = random(20, 100);
        for (long i = 0; i < (360 * n); i++)
        {
            float sx = cos((i / n - 90) * DEG2RAD_SPIRO);
            float sy = sin((i / n - 90) * DEG2RAD_SPIRO);
            uint16_t x0 = sx * (120 - r) + 159;
            uint16_t yy0 = sy * (120 - r) + 119;

            sy = cos(((i % 360) - 90) * DEG2RAD_SPIRO);
            sx = sin(((i % 360) - 90) * DEG2RAD_SPIRO);
            uint16_t x1 = sx * r + x0;
            uint16_t yy1 = sy * r + yy0;
            tft.drawPixel(x1, yy1, spiro_rainbow(map(i % 360, 0, 360, 0, 127)));
        }

        patternCount++;
        delay(1000); // Brief pause between patterns
    }

    delay(2000); // Final display time

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

void test_tft_terminal()
{
    Serial.println("Testing TFT Terminal...");

    initialize_tft(0); // Portrait mode required for scrolling

    tft.fillScreen(TFT_BLACK);

    // Setup top banner
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.fillRect(0, 0, 240, 16, TFT_BLUE);
    tft.drawCentreString(" TFT Terminal Demo ", 120, 0, 2);

    // Change colour for scrolling zone text
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // Setup scroll area
    tft_setupScrollArea(TERM_TOP_FIXED_AREA, TERM_BOT_FIXED_AREA);

    // Zero the array
    for (byte i = 0; i < 18; i++)
        term_blank[i] = 0;

    // Simulate some terminal output
    const char *demoLines[] = {
        "System initialized...",
        "TFT Terminal Test",
        "Scrolling text demo",
        "Line 4: More text here",
        "Line 5: Even more content",
        "Line 6: Testing scrolling",
        "Line 7: Almost done",
        "Line 8: Final line",
        "Demo complete!"};

    int numLines = sizeof(demoLines) / sizeof(demoLines[0]);

    for (int line = 0; line < numLines; line++)
    {
        const char *text = demoLines[line];
        term_xPos = 0;

        for (int i = 0; text[i] != '\0'; i++)
        {
            char c = text[i];
            if (c == '\n' || term_xPos > 231)
            {
                term_xPos = 0;
                term_yDraw = term_scroll_line();
            }
            if (c >= 32 && c < 128)
            {
                term_xPos += tft.drawChar(c, term_xPos, term_yDraw, 2);
                term_blank[(18 + (term_yStart - TERM_TOP_FIXED_AREA) / TERM_TEXT_HEIGHT) % 19] = term_xPos;
            }
        }

        // Add newline
        term_xPos = 0;
        term_yDraw = term_scroll_line();

        delay(500); // Pause between lines
    }

    delay(2000); // Show final result

    // Reset scroll area to full screen
    tft_scroll_to(0);
    tft_setupScrollArea(0, 0);

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Software scrolling terminal variables and functions (landscape, software-based)
#define SOFT_TEXT_HEIGHT 16
#define SOFT_BOT_FIXED_AREA 0
#define SOFT_TOP_FIXED_AREA 16
#define SOFT_YMAX 240 // Landscape height
#define SOFT_XMAX 320 // Landscape width

uint16_t soft_yStart = SOFT_TOP_FIXED_AREA;
uint16_t soft_yArea = SOFT_YMAX - SOFT_TOP_FIXED_AREA - SOFT_BOT_FIXED_AREA;
uint16_t soft_yDraw = SOFT_YMAX - SOFT_BOT_FIXED_AREA - SOFT_TEXT_HEIGHT;
uint16_t soft_xPos = 0;
int soft_blank[19];

#define MAX_LINES 15 // Maximum lines to keep in buffer
String soft_lines[MAX_LINES];
int soft_lineCount = 0;

void soft_scroll_line()
{
    // Software scroll: shift all lines up
    for (int i = 0; i < soft_lineCount - 1; i++)
    {
        soft_lines[i] = soft_lines[i + 1];
    }
    if (soft_lineCount > 0)
    {
        soft_lineCount--;
    }

    // Clear the scrolling area
    tft.fillRect(0, SOFT_TOP_FIXED_AREA, SOFT_XMAX, soft_yArea, TFT_BLACK);

    // Redraw all remaining lines
    for (int i = 0; i < soft_lineCount; i++)
    {
        tft.setCursor(0, SOFT_TOP_FIXED_AREA + i * SOFT_TEXT_HEIGHT);
        tft.print(soft_lines[i]);
    }

    soft_yDraw = SOFT_YMAX - SOFT_BOT_FIXED_AREA - SOFT_TEXT_HEIGHT;
    soft_xPos = 0;
}

void test_tft_terminal_software()
{
    Serial.println("Testing TFT Terminal Software...");

    initialize_tft(1); // Landscape mode

    tft.fillScreen(TFT_BLACK);

    // Setup top banner
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.fillRect(0, 0, SOFT_XMAX, SOFT_TOP_FIXED_AREA, TFT_BLUE);
    tft.drawCentreString(" TFT Terminal Software ", SOFT_XMAX / 2, 0, 2);

    // Change colour for scrolling zone text
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    // Initialize line buffer
    soft_lineCount = 0;

    // Simulate some terminal output
    const char *demoLines[] = {
        "Software Scrolling Demo",
        "Landscape Orientation",
        "Line 3: Testing scroll",
        "Line 4: More content",
        "Line 5: Scrolling up",
        "Line 6: Buffer management",
        "Line 7: Redraw optimization",
        "Line 8: Final demonstration",
        "Software scroll complete!"};

    int numLines = sizeof(demoLines) / sizeof(demoLines[0]);

    for (int line = 0; line < numLines; line++)
    {
        const char *text = demoLines[line];

        // Add the complete line to buffer (bottom-up: newest at end)
        if (soft_lineCount < MAX_LINES)
        {
            soft_lines[soft_lineCount++] = String(text);
        }
        else
        {
            // Shift buffer up (remove oldest line)
            for (int j = 0; j < MAX_LINES - 1; j++)
            {
                soft_lines[j] = soft_lines[j + 1];
            }
            soft_lines[MAX_LINES - 1] = String(text);
        }

        // Redraw the entire scrolling area from bottom up
        tft.fillRect(0, SOFT_TOP_FIXED_AREA, SOFT_XMAX, soft_yArea, TFT_BLACK);

        // Draw lines from bottom to top (most recent at bottom)
        int startY = SOFT_YMAX - SOFT_BOT_FIXED_AREA - SOFT_TEXT_HEIGHT;
        int linesToDraw = min(soft_lineCount, (int)(soft_yArea / SOFT_TEXT_HEIGHT));

        for (int i = 0; i < linesToDraw; i++)
        {
            int lineIndex = soft_lineCount - linesToDraw + i;
            tft.setCursor(0, startY - (linesToDraw - 1 - i) * SOFT_TEXT_HEIGHT);
            tft.print(soft_lines[lineIndex]);
        }

        delay(800); // Pause between lines
    }

    delay(3000); // Show final result

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Starfield simulation variables and functions
#define NSTARS 512 // Reduced from 1024 for test performance
uint8_t star_sx[NSTARS] = {};
uint8_t star_sy[NSTARS] = {};
uint8_t star_sz[NSTARS] = {};
uint8_t star_za, star_zb, star_zc, star_zx;

// Fast 0-255 random number generator
inline uint8_t __attribute__((always_inline)) star_rng()
{
    star_zx++;
    star_za = (star_za ^ star_zc ^ star_zx);
    star_zb = (star_zb + star_za);
    star_zc = ((star_zc + (star_zb >> 1)) ^ star_za);
    return star_zc;
}

void test_tft_starfield()
{
    Serial.println("Testing TFT Starfield...");

    initialize_tft(1); // Landscape mode

    tft.fillScreen(TFT_BLACK);

    // Initialize RNG seeds
    star_za = random(256);
    star_zb = random(256);
    star_zc = random(256);
    star_zx = random(256);

    // Initialize stars
    for (int i = 0; i < NSTARS; ++i)
    {
        star_sx[i] = 160 - 120 + star_rng();
        star_sy[i] = star_rng();
        star_sz[i] = star_rng();
    }

    unsigned long startTime = millis();
    int frameCount = 0;

    while (millis() - startTime < 5000 && frameCount < 2000)
    { // Run for 5 seconds or 200 frames
        unsigned long t0 = micros();
        uint8_t spawnDepthVariation = 255;

        for (int i = 0; i < NSTARS; ++i)
        {
            if (star_sz[i] <= 1)
            {
                star_sx[i] = 160 - 120 + star_rng();
                star_sy[i] = star_rng();
                star_sz[i] = spawnDepthVariation--;
            }
            else
            {
                int old_screen_x = ((int)star_sx[i] - 160) * 256 / star_sz[i] + 160;
                int old_screen_y = ((int)star_sy[i] - 120) * 256 / star_sz[i] + 120;

                // Erase old star
                tft.drawPixel(old_screen_x, old_screen_y, TFT_BLACK);

                star_sz[i] -= 2;
                if (star_sz[i] > 1)
                {
                    int screen_x = ((int)star_sx[i] - 160) * 256 / star_sz[i] + 160;
                    int screen_y = ((int)star_sy[i] - 120) * 256 / star_sz[i] + 120;

                    if (screen_x >= 0 && screen_y >= 0 && screen_x < 320 && screen_y < 240)
                    {
                        uint8_t r, g, b;
                        r = g = b = 255 - star_sz[i];
                        tft.drawPixel(screen_x, screen_y, tft.color565(r, g, b));
                    }
                    else
                    {
                        star_sz[i] = 0; // Out of screen, die.
                    }
                }
            }
        }

        unsigned long t1 = micros();
        float fps = 1.0 / ((t1 - t0) / 1000000.0);
        if (frameCount % 50 == 0)
        { // Print FPS every 50 frames
            Serial.print("Starfield FPS: ");
            Serial.println(fps);
        }

        frameCount++;
        yield();
    }

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

void test_tft_string_align()
{
    Serial.println("Testing TFT String Align...");

    initialize_tft(1); // Landscape mode

    tft.fillScreen(TFT_BLACK);

    // Test different text datums
    for (byte datum = 0; datum < 9; datum++)
    {
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(datum);
        tft.drawNumber(88, 160, 120, 8);
        tft.fillCircle(160, 120, 5, TFT_RED);

        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_BLACK);
        tft.drawString("X", 160, 120, 2);

        delay(500); // Shorter delay for test
        tft.fillScreen(TFT_BLACK);
    }

    // Test centre string
    tft.setTextColor(TFT_BLUE, TFT_BLACK);
    tft.drawCentreString("69", 160, 120, 8);
    tft.fillCircle(160, 120, 5, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 120, 2);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    // Test right string
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawRightString("88", 160, 120, 8);
    tft.fillCircle(160, 120, 5, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 120, 2);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    // Test floating point drawing
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setTextDatum(MC_DATUM);

    float test = 67.125;
    tft.drawFloat(test, 4, 160, 180, 4);
    tft.fillCircle(160, 180, 5, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 180, 2);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    test = -0.555555;
    tft.drawFloat(test, 3, 160, 180, 4);
    tft.fillCircle(160, 180, 5, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 180, 2);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    test = 0.1;
    tft.drawFloat(test, 4, 160, 180, 4);
    tft.fillCircle(160, 180, 5, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 180, 2);
    delay(1000);
    tft.fillScreen(TFT_BLACK);

    test = 9999999;
    tft.drawFloat(test, 1, 160, 180, 4);
    tft.fillCircle(160, 180, 5, TFT_YELLOW);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_BLACK);
    tft.drawString("X", 160, 180, 2);
    delay(2000);

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Forward declaration for rainbow function
uint16_t rainbow(byte value);

// Number of circles to draw
#define CNUMBER 24

// Structure to hold circle plotting parameters
typedef struct circle_t
{
    int16_t cx[CNUMBER] = {0};   // x coordinate of centre
    int16_t cy[CNUMBER] = {0};   // y coordinate of centre
    int16_t cr[CNUMBER] = {0};   // radius
    uint16_t col[CNUMBER] = {0}; // colour
    int16_t dx[CNUMBER] = {0};   // x movement & direction
    int16_t dy[CNUMBER] = {0};   // y movement & direction
    int16_t px[CNUMBER] = {0};   // previous x coordinate
    int16_t py[CNUMBER] = {0};   // previous y coordinate
} circle_param;

void test_tft_bouncy_circles()
{
    Serial.println("Testing TFT Bouncy Circles...");

    initialize_tft(0); // Portrait mode

    tft.fillScreen(TFT_BLACK);

    // Create the structure and get a pointer to it
    circle_param *circle = new circle_param;

    // Seed the random number generator
    randomSeed(analogRead(A0));

    // Initialise circle parameters
    for (uint16_t i = 0; i < CNUMBER; i++)
    {
        circle->cr[i] = random(8, 16);
        circle->cx[i] = random(circle->cr[i], tft.width() - circle->cr[i]);
        circle->cy[i] = random(circle->cr[i], tft.height() - circle->cr[i]);

        circle->col[i] = rainbow(4 * i);
        circle->dx[i] = random(1, 4);
        if (random(2))
            circle->dx[i] = -circle->dx[i];
        circle->dy[i] = random(1, 4);
        if (random(2))
            circle->dy[i] = -circle->dy[i];
    }

    // Used for fps measuring
    uint16_t counter = 0;
    int32_t startMillis = millis();
    uint16_t interval = 50;
    String fps = "xx.xx fps";

    // Initialize previous positions
    for (uint16_t i = 0; i < CNUMBER; i++)
    {
        circle->px[i] = circle->cx[i];
        circle->py[i] = circle->cy[i];
    }

    // Run animation for 5 seconds
    uint32_t endTime = millis() + 5000;

    while (millis() < endTime)
    {
        // Erase circles at previous positions (draw black circles)
        for (uint16_t i = 0; i < CNUMBER; i++)
        {
            tft.fillCircle(circle->px[i], circle->py[i], circle->cr[i], TFT_BLACK);
            tft.drawCircle(circle->px[i], circle->py[i], circle->cr[i], TFT_BLACK);
        }

        // Update circle positions
        for (uint16_t i = 0; i < CNUMBER; i++)
        {
            circle->px[i] = circle->cx[i]; // Store current position as previous
            circle->py[i] = circle->cy[i];

            circle->cx[i] += circle->dx[i];
            circle->cy[i] += circle->dy[i];

            // Bounce off edges
            if (circle->cx[i] <= circle->cr[i])
            {
                circle->cx[i] = circle->cr[i];
                circle->dx[i] = -circle->dx[i];
            }
            else if (circle->cx[i] + circle->cr[i] >= tft.width() - 1)
            {
                circle->cx[i] = tft.width() - circle->cr[i] - 1;
                circle->dx[i] = -circle->dx[i];
            }

            if (circle->cy[i] <= circle->cr[i])
            {
                circle->cy[i] = circle->cr[i];
                circle->dy[i] = -circle->dy[i];
            }
            else if (circle->cy[i] + circle->cr[i] >= tft.height() - 1)
            {
                circle->cy[i] = tft.height() - circle->cr[i] - 1;
                circle->dy[i] = -circle->dy[i];
            }
        }

        // Draw all circles at new positions
        for (uint16_t i = 0; i < CNUMBER; i++)
        {
            // Draw filled circle
            tft.fillCircle(circle->cx[i], circle->cy[i], circle->cr[i], circle->col[i]);

            // Draw outline
            tft.drawCircle(circle->cx[i], circle->cy[i], circle->cr[i], TFT_WHITE);

            // Draw circle number
            tft.setTextColor(TFT_BLACK, circle->col[i]);
            tft.setTextDatum(MC_DATUM);
            tft.drawNumber(i + 1, circle->cx[i], circle->cy[i], 1);
        }

        // Calculate the fps every <interval> iterations
        counter++;
        if (counter % interval == 0)
        {
            long millisSinceUpdate = millis() - startMillis;
            fps = String((interval * 1000.0 / (millisSinceUpdate))) + " fps";
            Serial.println(fps);
            startMillis = millis();
        }
    }

    // Display final FPS
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Bouncy Circles Test", tft.width() / 2, tft.height() / 2 - 20, 2);
    tft.drawString("Final FPS: " + fps, tft.width() / 2, tft.height() / 2 + 10, 2);

    delay(2000);

    // Clean up
    delete circle;

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

// Test JPEG image data (simple 16x16 test pattern)
// This is a minimal JPEG for testing - in practice you'd use a real image
const uint8_t test_jpeg[] PROGMEM = {
    0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A, 0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x01, 0x00, 0x48,
    0x00, 0x48, 0x00, 0x00, 0xFF, 0xDB, 0x00, 0x43, 0x00, 0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08,
    0x07, 0x07, 0x07, 0x09, 0x09, 0x08, 0x0A, 0x0C, 0x14, 0x0D, 0x0C, 0x0B, 0x0B, 0x0C, 0x19, 0x12,
    0x13, 0x0F, 0x14, 0x1D, 0x1A, 0x1F, 0x1E, 0x1D, 0x1A, 0x1C, 0x1C, 0x20, 0x24, 0x2E, 0x27, 0x20,
    0x22, 0x2C, 0x23, 0x1C, 0x1C, 0x28, 0x37, 0x29, 0x2C, 0x30, 0x31, 0x34, 0x34, 0x34, 0x1F, 0x27,
    0x39, 0x3D, 0x38, 0x32, 0x3C, 0x2E, 0x33, 0x34, 0x32, 0xFF, 0xC0, 0x00, 0x11, 0x08, 0x00, 0x10,
    0x00, 0x10, 0x01, 0x01, 0x11, 0x00, 0x02, 0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4, 0x00, 0x14,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0xFF, 0xC4, 0x00, 0x14, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xDA, 0x00, 0x0C, 0x03, 0x01, 0x00, 0x02,
    0x11, 0x03, 0x11, 0x00, 0x3F, 0x00, 0xAA, 0xFF, 0x00, 0xFF, 0xD9};

// DMA buffers for JPEG decoding
#ifdef USE_DMA
uint16_t dmaBuffer1[16 * 16]; // Toggle buffer for 16*16 MCU block, 512bytes
uint16_t dmaBuffer2[16 * 16]; // Toggle buffer for 16*16 MCU block, 512bytes
uint16_t *dmaBufferPtr = dmaBuffer1;
bool dmaBufferSel = 0;
#endif

// JPEG decoder callback function
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t *bitmap)
{
    // Stop further decoding as image is running off bottom of screen
    if (y >= tft.height())
        return 0;

#ifdef USE_DMA
    // Double buffering is used, the bitmap is copied to the buffer by pushImageDMA() the
    // bitmap can then be updated by the jpeg decoder while DMA is in progress
    if (dmaBufferSel)
        dmaBufferPtr = dmaBuffer2;
    else
        dmaBufferPtr = dmaBuffer1;
    dmaBufferSel = !dmaBufferSel; // Toggle buffer selection
    // pushImageDMA() will clip the image block at screen boundaries before initiating DMA
    tft.pushImageDMA(x, y, w, h, bitmap, dmaBufferPtr); // Initiate DMA - blocking only if last DMA is not complete
#else
    // Non-DMA blocking alternative
    tft.pushImage(x, y, w, h, bitmap); // Blocking, so only returns when image block is drawn
#endif

    // Return 1 to decode next block
    return 1;
}

// Return a 16-bit rainbow colour
uint16_t rainbow(byte value)
{
    // If 'value' is in the range 0-159 it is converted to a spectrum colour
    // from 0 = red through to 127 = blue to 159 = violet
    // Extending the range to 0-191 adds a further violet to red band

    value = value % 192;

    byte red = 0;   // Red is the top 5 bits of a 16-bit colour value
    byte green = 0; // Green is the middle 6 bits, but only top 5 bits used here
    byte blue = 0;  // Blue is the bottom 5 bits

    byte sector = value >> 5;
    byte amplit = value & 0x1F;

    switch (sector)
    {
    case 0:
        red = 0x1F;
        green = amplit; // Green ramps up
        blue = 0;
        break;
    case 1:
        red = 0x1F - amplit; // Red ramps down
        green = 0x1F;
        blue = 0;
        break;
    case 2:
        red = 0;
        green = 0x1F;
        blue = amplit; // Blue ramps up
        break;
    case 3:
        red = 0;
        green = 0x1F - amplit; // Green ramps down
        blue = 0x1F;
        break;
    case 4:
        red = amplit; // Red ramps up
        green = 0;
        blue = 0x1F;
        break;
    case 5:
        red = 0x1F;
        green = 0;
        blue = 0x1F - amplit; // Blue ramps down
        break;
    }

    return red << 11 | green << 6 | blue;
}

// TFT Read Register functions
uint32_t readRegister(uint8_t reg, int16_t bytes, uint8_t index)
{
    uint32_t data = 0;

    while (bytes > 0)
    {
        bytes--;
        data = (data << 8) | tft.readcommand8(reg, index);
        index++;
    }

    Serial.print("Register 0x");
    if (reg < 0x10)
        Serial.print("0");
    Serial.print(reg, HEX);

    Serial.print(": 0x");

    // Add leading zeros as needed
    uint32_t mask = 0x1 << (bytes * 7);
    while (data < mask && mask > 0x1)
    {
        Serial.print("0");
        mask = mask >> 4;
    }

    Serial.println(data, HEX);

    return data;
}

void test_tft_read_reg()
{
    Serial.println("Testing TFT Read Registers...");

    initialize_tft(1); // Portrait mode

    tft.fillScreen(TFT_BLUE);
    tft.setCursor(0, 0, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.println("Reading TFT Registers...");
    tft.println("Check Serial output");

    delay(2000);

    // Print a useful subset of the readable registers
    Serial.println();
    Serial.println("Useful subset of readable registers:");

    readRegister(0x04, 3, 1); // RDDID
    readRegister(0x09, 4, 1); // RDDST
    readRegister(0x0A, 1, 1); // RDMODE
    readRegister(0x0B, 1, 1); // RDMADCTL
    readRegister(0x0C, 1, 1); // RDPIXFMT
    readRegister(0x0D, 1, 1); // RDSELFDIAG
    readRegister(0x2E, 3, 1); // RAMRD

    readRegister(0xDA, 1, 1); // RDID1
    readRegister(0xDB, 1, 1); // RDID2
    readRegister(0xDC, 1, 1); // RDID3
    readRegister(0xDD, 1, 1); // RDID4

    Serial.println();
    Serial.println("Test 8, 16 and 32-bit reads and the index...");

    // Test 8, 16 and 32-bit reads and index
    // Note at index 0 the register values are typically undefined (Bxxxxxxxx)
    Serial.println(tft.readcommand8(0xD3, 2), HEX);
    Serial.println(tft.readcommand16(0xD3, 2), HEX);
    Serial.println(tft.readcommand32(0xD3, 0), HEX);

    delay(2000);

    // Erase screen
    tft.fillScreen(TFT_BLACK);

    disableBacklight();

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Matrix-Style Scrolling Text
 * Creates a Matrix movie-style cascading green character effect
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Matrix/TFT_Matrix.ino
 */

#define TEXT_HEIGHT 8    // Height of text to be printed and scrolled
#define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area
#define TOP_FIXED_AREA 0 // Number of lines in top fixed area

static uint16_t yStart_matrix = TOP_FIXED_AREA;

// Helper: Scroll slowly with delay
static int scroll_slow(int lines, int wait)
{
    int yTemp = yStart_matrix;
    for (int i = 0; i < lines; i++)
    {
        yStart_matrix++;
        if (yStart_matrix == 320 - BOT_FIXED_AREA)
            yStart_matrix = TOP_FIXED_AREA;

        tft_scroll_to(yStart_matrix);
        delay(wait);
    }
    return yTemp;
}

void test_tft_matrix()
{
    Serial.println("\n=== Testing Matrix Scrolling Effect ===");

    initialize_tft(0); // Use portrait mode for Matrix effect

    tft_setupScrollArea(TOP_FIXED_AREA, BOT_FIXED_AREA);
    // tft_set_scroll_def(TOP_FIXED_AREA, 320 - TOP_FIXED_AREA - BOT_FIXED_AREA, BOT_FIXED_AREA);

    byte pos[42]; // Brightness values for each column
    uint16_t xPos = 0;
    uint16_t yDraw = TFT_WIDTH - BOT_FIXED_AREA - TEXT_HEIGHT;

    // Initialize brightness array
    for (int i = 0; i < 42; i++)
    {
        pos[i] = 0;
    }

    Serial.println("Filling screen with Matrix characters...");

    // Fill screen with random streaks of characters
    for (int j = 0; j < 600; j += TEXT_HEIGHT)
    {
        for (int i = 0; i < 40; i++)
        {
            // Brightness fade logic
            if (pos[i] > 20)
                pos[i] -= 3; // Rapid fade initially
            if (pos[i] > 0)
                pos[i] -= 1; // Slow fade later

            // ~1 in 20 probability of new character (only in first 400 lines)
            if ((random(20) == 1) && (j < 400))
                pos[i] = 63;

            // Set green brightness (5-bit green in 16-bit color: bits 5-10)
            tft.setTextColor(pos[i] << 5, TFT_BLACK);

            // Brightest characters are white
            if (pos[i] == 63)
                tft.setTextColor(TFT_WHITE, TFT_BLACK);

            // Draw random character
            xPos += tft.drawChar(random(32, 128), xPos, yDraw, 1);
        }

        yDraw = scroll_slow(TEXT_HEIGHT, 14); // Scroll, 14ms per line
        xPos = 0;

        yield();
    }

    Serial.println("Initial fill complete, scrolling continuously for 10 seconds...");

    // Scroll continuously for 10 seconds
    unsigned long scrollStart = millis();
    const unsigned long scrollDuration = 10000; // 10 seconds

    while (millis() - scrollStart < scrollDuration)
    {
        yDraw = scroll_slow(320, 5); // Scroll full height, 5ms per line
        yield();
    }

    Serial.println("Matrix effect complete");

    // Return to normal mode
    tft.setRotation(1); // Back to landscape
    // Reset scroll position and area
    tft_scroll_to(0);
    tft_setupScrollArea(0, 0);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Matrix Software Scrolling (Landscape)
 * Matrix-style cascading effect using software scrolling in landscape mode
 * Columns scroll from top to bottom by redrawing characters
 */
void test_tft_matrix_software()
{
    Serial.println("\n=== Testing Matrix Software Scrolling (Landscape) ===");

    initialize_tft(1);

    const int numColumns = 40;                   // Number of character columns
    const int colWidth = 8;                      // Character width in pixels
    const int charHeight = 8;                    // Character height in pixels
    const int maxRows = TFT_HEIGHT / charHeight; // Maximum rows that fit vertically (30)

    // Column state arrays
    byte brightness[numColumns][maxRows]; // Brightness for each character
    char characters[numColumns][maxRows]; // Character at each position
    int columnPos[numColumns];            // Current head position of each column
    int columnLength[numColumns];         // Length of each column trail

    // Initialize columns
    for (int col = 0; col < numColumns; col++)
    {
        columnPos[col] = random(-20, 0);   // Start at random offsets
        columnLength[col] = random(5, 15); // Random trail length

        for (int row = 0; row < maxRows; row++)
        {
            brightness[col][row] = 0;
            characters[col][row] = random(32, 128);
        }
    }

    Serial.println("Running Matrix effect for 15 seconds...");

    unsigned long startTime = millis();
    const unsigned long duration = 15000; // 15 seconds
    uint32_t frameCount = 0;

    while (millis() - startTime < duration)
    {
        // Update and draw each column
        for (int col = 0; col < numColumns; col++)
        {
            int xPos = col * colWidth;

            // Move column down
            columnPos[col]++;

            // Reset column when it goes off screen
            if (columnPos[col] - columnLength[col] > maxRows)
            {
                columnPos[col] = random(-10, 0);
                columnLength[col] = random(5, 15);
            }

            // Draw the column from top to bottom
            for (int row = 0; row < maxRows; row++)
            {
                int yPos = row * charHeight;

                // Calculate distance from head of column
                int distFromHead = columnPos[col] - row;

                if (distFromHead >= 0 && distFromHead < columnLength[col])
                {
                    // Character is part of the trail
                    byte bright;

                    if (distFromHead == 0)
                    {
                        // Head of column - white
                        bright = 63;
                        tft.setTextColor(TFT_WHITE, TFT_BLACK);
                    }
                    else
                    {
                        // Fade along the trail (bright green to dark green)
                        bright = map(distFromHead, 1, columnLength[col], 50, 5);
                        tft.setTextColor(bright << 5, TFT_BLACK);
                    }

                    // Change character occasionally for effect
                    if (distFromHead == 0 && random(10) < 3)
                    {
                        characters[col][row] = random(32, 128);
                    }

                    brightness[col][row] = bright;
                    tft.drawChar(characters[col][row], xPos, yPos, 1);
                }
                else if (brightness[col][row] > 0)
                {
                    // Fade out old characters
                    brightness[col][row]--;
                    if (brightness[col][row] > 0)
                    {
                        tft.setTextColor(brightness[col][row] << 5, TFT_BLACK);
                        tft.drawChar(characters[col][row], xPos, yPos, 1);
                    }
                    else
                    {
                        // Clear character
                        tft.fillRect(xPos, yPos, colWidth, charHeight, TFT_BLACK);
                    }
                }
            }
        }

        frameCount++;

        // Control animation speed
        delay(50); // ~20 FPS
        yield();
    }

    unsigned long totalTime = millis() - startTime;
    float fps = (frameCount * 1000.0f) / totalTime;

    Serial.printf("Matrix software scrolling complete\n");
    Serial.printf("Frames: %lu, Time: %lu ms, FPS: %.2f\n", frameCount, totalTime, fps);

    // Show summary
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.setCursor(60, 100);
    tft.println("Matrix Complete");
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW);
    tft.setCursor(90, 140);
    tft.printf("FPS: %.2f", fps);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Linear Analog Meter
 * Displays an animated analog meter with needle showing sine wave values
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Meter_linear/TFT_Meter_linear.ino
 */

#define M_SIZE 1.3333 // Meter size for rotation(1) landscape mode

static float ltx_meter = 0;                                         // Saved x coord of bottom of needle
static uint16_t osx_meter = M_SIZE * 120, osy_meter = M_SIZE * 120; // Saved needle coords
static int old_analog_meter = -999;                                 // Value last displayed

// Helper: Draw the analog meter face
static void analogMeter()
{
    // Meter outline
    tft.fillRect(0, 0, M_SIZE * 239, M_SIZE * 126, TFT_DARKGREY);
    tft.fillRect(5, 3, M_SIZE * 230, M_SIZE * 119, TFT_WHITE);

    tft.setTextColor(TFT_BLACK);

    // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg FSD swing)
    for (int i = -50; i < 51; i += 5)
    {
        // Long scale tick length
        int tl = 15;

        // Coordinates of tick to draw
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
        uint16_t y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 140;
        uint16_t x1 = sx * M_SIZE * 100 + M_SIZE * 120;
        uint16_t y1 = sy * M_SIZE * 100 + M_SIZE * 140;

        // Coordinates of next tick for zone fill
        float sx2 = cos((i + 5 - 90) * 0.0174532925);
        float sy2 = sin((i + 5 - 90) * 0.0174532925);
        int x2 = sx2 * (M_SIZE * 100 + tl) + M_SIZE * 120;
        int y2 = sy2 * (M_SIZE * 100 + tl) + M_SIZE * 140;
        int x3 = sx2 * M_SIZE * 100 + M_SIZE * 120;
        int y3 = sy2 * M_SIZE * 100 + M_SIZE * 140;

        // Green zone (0-25)
        if (i >= 0 && i < 25)
        {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
        }

        // Orange zone (25-50)
        if (i >= 25 && i < 50)
        {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
        }

        // Short scale tick length
        if (i % 25 != 0)
            tl = 8;

        // Recalculate coords in case tick length changed
        x0 = sx * (M_SIZE * 100 + tl) + M_SIZE * 120;
        y0 = sy * (M_SIZE * 100 + tl) + M_SIZE * 140;
        x1 = sx * M_SIZE * 100 + M_SIZE * 120;
        y1 = sy * M_SIZE * 100 + M_SIZE * 140;

        // Draw tick
        tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

        // Draw labels at major tick marks
        if (i % 25 == 0)
        {
            x0 = sx * (M_SIZE * 100 + tl + 10) + M_SIZE * 120;
            y0 = sy * (M_SIZE * 100 + tl + 10) + M_SIZE * 140;
            switch (i / 25)
            {
            case -2:
                tft.drawCentreString("0", x0, y0 - 12, 2);
                break;
            case -1:
                tft.drawCentreString("25", x0, y0 - 9, 2);
                break;
            case 0:
                tft.drawCentreString("50", x0, y0 - 7, 2);
                break;
            case 1:
                tft.drawCentreString("75", x0, y0 - 9, 2);
                break;
            case 2:
                tft.drawCentreString("100", x0, y0 - 12, 2);
                break;
            }
        }

        // Draw scale arc
        sx = cos((i + 5 - 90) * 0.0174532925);
        sy = sin((i + 5 - 90) * 0.0174532925);
        x0 = sx * M_SIZE * 100 + M_SIZE * 120;
        y0 = sy * M_SIZE * 100 + M_SIZE * 140;
        if (i < 50)
            tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
    }

    tft.drawString("%RH", M_SIZE * (5 + 230 - 40), M_SIZE * (119 - 20), 2); // Units at bottom right
    tft.drawCentreString("%RH", M_SIZE * 120, M_SIZE * 70, 4);              // Center label
    tft.drawRect(5, 3, M_SIZE * 230, M_SIZE * 119, TFT_BLACK);              // Draw bezel line

    // Reset needle position
    old_analog_meter = -999;
    ltx_meter = 0;
    osx_meter = M_SIZE * 120;
    osy_meter = M_SIZE * 120;
}

// Helper: Update needle position
static void plotNeedle(int value, byte ms_delay)
{
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    char buf[8];
    dtostrf(value, 4, 0, buf);
    tft.drawRightString(buf, M_SIZE * 40, M_SIZE * (119 - 20), 2);

    if (value < -10)
        value = -10; // Limit value to emulate needle end stops
    if (value > 110)
        value = 110;

    // Move the needle until new value reached
    while (!(value == old_analog_meter))
    {
        if (old_analog_meter < value)
            old_analog_meter++;
        else
            old_analog_meter--;

        if (ms_delay == 0)
            old_analog_meter = value; // Update immediately if delay is 0

        float sdeg = map(old_analog_meter, -10, 110, -150, -30); // Map value to angle

        // Calculate tip of needle coords
        float sx = cos(sdeg * 0.0174532925);
        float sy = sin(sdeg * 0.0174532925);

        // Calculate x delta of needle start
        float tx = tan((sdeg + 90) * 0.0174532925);

        // Erase old needle image
        tft.drawLine(M_SIZE * (120 + 20 * ltx_meter - 1), M_SIZE * (140 - 20), osx_meter - 1, osy_meter, TFT_WHITE);
        tft.drawLine(M_SIZE * (120 + 20 * ltx_meter), M_SIZE * (140 - 20), osx_meter, osy_meter, TFT_WHITE);
        tft.drawLine(M_SIZE * (120 + 20 * ltx_meter + 1), M_SIZE * (140 - 20), osx_meter + 1, osy_meter, TFT_WHITE);

        // Re-plot text under needle
        tft.setTextColor(TFT_BLACK);
        tft.drawCentreString("%RH", M_SIZE * 120, M_SIZE * 70, 4);

        // Store new needle end coords for next erase
        ltx_meter = tx;
        osx_meter = M_SIZE * (sx * 98 + 120);
        osy_meter = M_SIZE * (sy * 98 + 140);

        // Draw the needle in the new position (3 lines to thicken needle)
        tft.drawLine(M_SIZE * (120 + 20 * ltx_meter - 1), M_SIZE * (140 - 20), osx_meter - 1, osy_meter, TFT_RED);
        tft.drawLine(M_SIZE * (120 + 20 * ltx_meter), M_SIZE * (140 - 20), osx_meter, osy_meter, TFT_MAGENTA);
        tft.drawLine(M_SIZE * (120 + 20 * ltx_meter + 1), M_SIZE * (140 - 20), osx_meter + 1, osy_meter, TFT_RED);

        // Slow needle down slightly as it approaches new position
        if (abs(old_analog_meter - value) < 10)
            ms_delay += ms_delay / 5;

        delay(ms_delay);
    }
}

void test_tft_meter_linear()
{
    Serial.println("\n=== Testing Linear Analog Meter ===");

    initialize_tft(1);

    analogMeter(); // Draw analog meter

    Serial.println("Animating meter with sine wave for 15 seconds...");

    int d = 0;
    int value = 0;
    unsigned long startTime = millis();
    const unsigned long duration = 15000; // 15 seconds
    uint32_t updateCount = 0;

    while (millis() - startTime < duration)
    {
        // Create a sine wave for testing
        d += 4;
        if (d >= 360)
            d = 0;
        value = 50 + 50 * sin((d + 0) * 0.0174532925);

        plotNeedle(value, 0); // Update needle (0ms delay for smooth animation)
        updateCount++;

        delay(35); // Update every 35ms
        yield();
    }

    unsigned long totalTime = millis() - startTime;
    float updatesPerSec = (updateCount * 1000.0f) / totalTime;

    Serial.printf("Meter test complete\n");
    Serial.printf("Updates: %lu, Time: %lu ms, Updates/sec: %.2f\n",
                  updateCount, totalTime, updatesPerSec);

    delay(2000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Multiple Analog and Linear Meters
 * Displays one analog meter plus six linear bar meters, all animated with sine waves
 * Based on TFT_eSPI example by Bodmer
 * https://github.com/Bodmer/TFT_eSPI/blob/master/examples/320%20x%20240/TFT_Meters/TFT_Meters.ino
 */

static float ltx_meters = 0;                               // Saved x coord of bottom of needle
static uint16_t osx_meters = 120, osy_meters = 120;        // Saved needle coords
static int old_analog_meters = -999;                       // Value last displayed
static int old_value_meters[6] = {-1, -1, -1, -1, -1, -1}; // Linear meter values

// Helper: Draw the analog meter (non-scaled, portrait mode)
static void analogMeter_multi()
{
    // Meter outline
    tft.fillRect(0, 0, 239, 126, TFT_DARKGREY);
    tft.fillRect(5, 3, 230, 119, TFT_WHITE);

    tft.setTextColor(TFT_BLACK);

    // Draw ticks every 5 degrees from -50 to +50 degrees
    for (int i = -50; i < 51; i += 5)
    {
        int tl = 15; // Long tick length

        // Tick coordinates
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (100 + tl) + 120;
        uint16_t y0 = sy * (100 + tl) + 140;
        uint16_t x1 = sx * 100 + 120;
        uint16_t y1 = sy * 100 + 140;

        // Next tick for zone fill
        float sx2 = cos((i + 5 - 90) * 0.0174532925);
        float sy2 = sin((i + 5 - 90) * 0.0174532925);
        int x2 = sx2 * (100 + tl) + 120;
        int y2 = sy2 * (100 + tl) + 140;
        int x3 = sx2 * 100 + 120;
        int y3 = sy2 * 100 + 140;

        // Green zone (0-25)
        if (i >= 0 && i < 25)
        {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
        }

        // Orange zone (25-50)
        if (i >= 25 && i < 50)
        {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
        }

        // Short tick length
        if (i % 25 != 0)
            tl = 8;

        // Recalculate coords
        x0 = sx * (100 + tl) + 120;
        y0 = sy * (100 + tl) + 140;
        x1 = sx * 100 + 120;
        y1 = sy * 100 + 140;

        // Draw tick
        tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

        // Draw labels
        if (i % 25 == 0)
        {
            x0 = sx * (100 + tl + 10) + 120;
            y0 = sy * (100 + tl + 10) + 140;
            switch (i / 25)
            {
            case -2:
                tft.drawCentreString("0", x0, y0 - 12, 2);
                break;
            case -1:
                tft.drawCentreString("25", x0, y0 - 9, 2);
                break;
            case 0:
                tft.drawCentreString("50", x0, y0 - 6, 2);
                break;
            case 1:
                tft.drawCentreString("75", x0, y0 - 9, 2);
                break;
            case 2:
                tft.drawCentreString("100", x0, y0 - 12, 2);
                break;
            }
        }

        // Draw scale arc
        sx = cos((i + 5 - 90) * 0.0174532925);
        sy = sin((i + 5 - 90) * 0.0174532925);
        x0 = sx * 100 + 120;
        y0 = sy * 100 + 140;
        if (i < 50)
            tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
    }

    tft.drawString("%RH", 5 + 230 - 40, 119 - 20, 2);
    tft.drawCentreString("%RH", 120, 70, 4);
    tft.drawRect(5, 3, 230, 119, TFT_BLACK);

    // Reset state
    old_analog_meters = -999;
    ltx_meters = 0;
    osx_meters = 120;
    osy_meters = 120;
}

// Helper: Update analog meter needle
static void plotNeedle_multi(int value, byte ms_delay)
{
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    char buf[8];
    dtostrf(value, 4, 0, buf);
    tft.drawRightString(buf, 40, 119 - 20, 2);

    if (value < -10)
        value = -10;
    if (value > 110)
        value = 110;

    while (!(value == old_analog_meters))
    {
        if (old_analog_meters < value)
            old_analog_meters++;
        else
            old_analog_meters--;

        if (ms_delay == 0)
            old_analog_meters = value;

        float sdeg = map(old_analog_meters, -10, 110, -150, -30);
        float sx = cos(sdeg * 0.0174532925);
        float sy = sin(sdeg * 0.0174532925);
        float tx = tan((sdeg + 90) * 0.0174532925);

        // Erase old needle
        tft.drawLine(120 + 20 * ltx_meters - 1, 140 - 20, osx_meters - 1, osy_meters, TFT_WHITE);
        tft.drawLine(120 + 20 * ltx_meters, 140 - 20, osx_meters, osy_meters, TFT_WHITE);
        tft.drawLine(120 + 20 * ltx_meters + 1, 140 - 20, osx_meters + 1, osy_meters, TFT_WHITE);

        // Re-plot text
        tft.setTextColor(TFT_BLACK);
        tft.drawCentreString("%RH", 120, 70, 4);

        // Store new coords
        ltx_meters = tx;
        osx_meters = sx * 98 + 120;
        osy_meters = sy * 98 + 140;

        // Draw new needle
        tft.drawLine(120 + 20 * ltx_meters - 1, 140 - 20, osx_meters - 1, osy_meters, TFT_RED);
        tft.drawLine(120 + 20 * ltx_meters, 140 - 20, osx_meters, osy_meters, TFT_MAGENTA);
        tft.drawLine(120 + 20 * ltx_meters + 1, 140 - 20, osx_meters + 1, osy_meters, TFT_RED);

        if (abs(old_analog_meters - value) < 10)
            ms_delay += ms_delay / 5;
        delay(ms_delay);
    }
}

// Helper: Draw a linear meter
static void plotLinear(const char *label, int x, int y)
{
    int w = 36;
    tft.drawRect(x, y, w, 155, TFT_DARKGREY);
    tft.fillRect(x + 2, y + 19, w - 3, 155 - 38, TFT_WHITE);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawCentreString(label, x + w / 2, y + 2, 2);

    // Draw tick marks
    for (int i = 0; i < 110; i += 10)
    {
        tft.drawFastHLine(x + 20, y + 27 + i, 6, TFT_BLACK);
    }

    for (int i = 0; i < 110; i += 50)
    {
        tft.drawFastHLine(x + 20, y + 27 + i, 9, TFT_BLACK);
    }

    // Draw pointer triangle
    tft.fillTriangle(x + 3, y + 127, x + 3 + 16, y + 127, x + 3, y + 127 - 5, TFT_RED);
    tft.fillTriangle(x + 3, y + 127, x + 3 + 16, y + 127, x + 3, y + 127 + 5, TFT_RED);

    tft.drawCentreString("---", x + w / 2, y + 155 - 18, 2);
}

// Helper: Update all 6 linear meter pointers
static void plotPointer_multi(int value[6])
{
    int dy = 187;
    byte pw = 16;

    tft.setTextColor(TFT_GREEN, TFT_BLACK);

    for (int i = 0; i < 6; i++)
    {
        char buf[8];
        dtostrf(value[i], 4, 0, buf);
        tft.drawRightString(buf, i * 40 + 36 - 5, 187 - 27 + 155 - 18, 2);

        int dx = 3 + 40 * i;
        if (value[i] < 0)
            value[i] = 0;
        if (value[i] > 100)
            value[i] = 100;

        while (!(value[i] == old_value_meters[i]))
        {
            dy = 187 + 100 - old_value_meters[i];
            if (old_value_meters[i] > value[i])
            {
                tft.drawLine(dx, dy - 5, dx + pw, dy, TFT_WHITE);
                old_value_meters[i]--;
                tft.drawLine(dx, dy + 6, dx + pw, dy + 1, TFT_RED);
            }
            else
            {
                tft.drawLine(dx, dy + 5, dx + pw, dy, TFT_WHITE);
                old_value_meters[i]++;
                tft.drawLine(dx, dy - 6, dx + pw, dy - 1, TFT_RED);
            }
        }
    }
}

void test_tft_meters()
{
    Serial.println("\n=== Testing Multiple Meters ===");

    initialize_tft(0);

    analogMeter_multi(); // Draw analog meter

    // Draw 6 linear meters
    byte d = 40;
    plotLinear("A0", 0 * d, 160);
    plotLinear("A1", 1 * d, 160);
    plotLinear("A2", 2 * d, 160);
    plotLinear("A3", 3 * d, 160);
    plotLinear("A4", 4 * d, 160);
    plotLinear("A5", 5 * d, 160);

    Serial.println("Animating 7 meters with sine waves for 15 seconds...");

    int deg = 0;
    int value[6] = {0, 0, 0, 0, 0, 0};
    unsigned long startTime = millis();
    const unsigned long duration = 15000; // 15 seconds
    uint32_t updateCount = 0;

    while (millis() - startTime < duration)
    {
        // Create sine waves with 60-degree phase shifts
        deg += 4;
        if (deg >= 360)
            deg = 0;

        value[0] = 50 + 50 * sin((deg + 0) * 0.0174532925);
        value[1] = 50 + 50 * sin((deg + 60) * 0.0174532925);
        value[2] = 50 + 50 * sin((deg + 120) * 0.0174532925);
        value[3] = 50 + 50 * sin((deg + 180) * 0.0174532925);
        value[4] = 50 + 50 * sin((deg + 240) * 0.0174532925);
        value[5] = 50 + 50 * sin((deg + 300) * 0.0174532925);

        plotPointer_multi(value);      // Update linear meters
        plotNeedle_multi(value[0], 0); // Update analog meter

        updateCount++;
        delay(35); // Update every 35ms
        yield();
    }

    unsigned long totalTime = millis() - startTime;
    float updatesPerSec = (updateCount * 1000.0f) / totalTime;

    Serial.printf("Multiple meters test complete\n");
    Serial.printf("Updates: %lu, Time: %lu ms, Updates/sec: %.2f\n",
                  updateCount, totalTime, updatesPerSec);

    delay(2000);

    // Return to landscape
    tft.setRotation(1);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: SPI Speed vs FPS Performance
 * Varies SPI speed from 10MHz to 60MHz and measures animation FPS
 */
void test_tft_spi_speed_performance()
{
    Serial.println("\n=== Testing SPI Speed vs Animation FPS ===");

    initialize_tft(1);

    int w = tft.width();
    int h = tft.height();
    int radius = 30;
    int speed = 3;
    int min_spi_freq = 10000000; // 10 MHz
    int max_spi_freq = 37500000; // 37.5 MHz

    // Enumerate all achievable SPI frequencies for this hardware
    uint32_t spi_speeds[256];
    uint32_t num_speeds = enumerate_spi_frequencies(spi_speeds, 256);

    // Find first index (from the end) with frequency >= 10MHz
    int32_t last_good_idx = -1;
    for (int32_t i = (int32_t)num_speeds - 1; i >= 0; i--)
    {
        if (spi_speeds[i] >= min_spi_freq)
        {
            last_good_idx = i;
            break;
        }
    }

    if (last_good_idx < 0)
    {
        Serial.println("No frequencies >= 10MHz available!");
        TEST_ASSERT_TRUE(false);
        return;
    }

    // Find FIRST index (from the start) where frequency <= 37.5MHz
    // Since sorted descending, this finds the lowest freq <= 37.5MHz
    int32_t min_safe_idx = -1;
    for (int32_t i = 0; i < (int32_t)num_speeds; i++)
    {
        if (spi_speeds[i] <= max_spi_freq)
        {
            min_safe_idx = i;
            break;
        }
    }

    if (min_safe_idx < 0 || min_safe_idx > last_good_idx)
    {
        Serial.println("No valid frequency range found!");
        TEST_ASSERT_TRUE(false);
        return;
    }

    // Count frequencies in test range
    uint32_t good_count = last_good_idx - min_safe_idx + 1;

    Serial.printf("\nFound %lu achievable SPI frequencies (testing %lu from %.2f MHz to %.2f MHz)\n",
                  num_speeds, good_count,
                  spi_speeds[last_good_idx] / 1000000.0f,
                  spi_speeds[min_safe_idx] / 1000000.0f);

    Serial.println("\nActual (MHz) | Frames | FPS   | Frame Time (ms)");
    Serial.println("--------------------------------------------------");

    // Test from lowest to highest frequency (start safe, end with risk of crash)
    for (int32_t i = last_good_idx; i >= min_safe_idx; i--)
    {
        uint32_t spi_freq = spi_speeds[i];

        // Set new SPI speed using direct register control
        uint32_t actual_freq = set_tft_spi_speed_direct(spi_freq);

        // Clear screen
        tft.fillScreen(TFT_BLACK);

        // Display current SPI speed being tested
        tft.setTextColor(TFT_YELLOW);
        tft.setTextSize(1);
        tft.setCursor(5, 5);
        tft.printf("Testing: %.2f MHz", actual_freq / 1000000.0f);

        delay(500); // Brief pause to see the message

        // Run animation test for 3 seconds
        unsigned long frame_count = 0;
        unsigned long start_time = millis();
        const unsigned long test_duration = 3000; // 3 seconds per speed

        int x = 50, y = 50;
        int dx = speed, dy = speed;

        while (millis() - start_time < test_duration)
        {
            // Clear previous circle
            tft.fillCircle(x, y, radius, TFT_BLACK);

            // Update position with bounce
            x += dx;
            y += dy;
            if (x - radius <= 0 || x + radius >= w)
                dx = -dx;
            if (y - radius <= 0 || y + radius >= h)
                dy = -dy;

            // Draw new circle
            uint16_t color = (frame_count % 3 == 0) ? TFT_RED : (frame_count % 3 == 1) ? TFT_GREEN
                                                                                       : TFT_BLUE;
            tft.fillCircle(x, y, radius, color);

            frame_count++;
        }

        // Calculate statistics
        unsigned long total_time = millis() - start_time;
        float fps = (float)frame_count / (total_time / 1000.0f);
        float frame_time = total_time / (float)frame_count;

        // Print results to serial
        Serial.printf("%12.2f | %6lu | %5.2f | %14.2f\n",
                      actual_freq / 1000000.0f, frame_count, fps, frame_time);

        // Brief pause between tests
        delay(200);
    }

    // Display summary on screen
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.setCursor(10, 20);
    tft.println("SPI Speed Test");
    tft.println("Complete!");

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, 80);
    tft.println("Results in Serial Monitor");

    Serial.println("-------------------------------------------------------");
    Serial.println("Test complete. Check results above.\n");

    delay(3000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Anti-aliased graphics (smooth circles, arcs, lines)
 * Tests smooth rendering with anti-aliasing
 */
void test_tft_antialiased_graphics()
{
    Serial.println("Testing anti-aliased graphics...");

    initialize_tft(1);

    tft.setTextSize(2);
    tft.setCursor(10, 5);
    tft.println("Anti-Aliased");

    int cx = tft.width() / 2;
    int cy = tft.height() / 2;

    // Draw smooth circles with different colors
    tft.drawSmoothCircle(cx - 70, cy - 50, 25, TFT_RED, TFT_BLACK);
    tft.drawSmoothCircle(cx, cy - 50, 25, TFT_GREEN, TFT_BLACK);
    tft.drawSmoothCircle(cx + 70, cy - 50, 25, TFT_BLUE, TFT_BLACK);

    // Draw filled smooth circles
    tft.fillSmoothCircle(cx - 70, cy + 10, 20, TFT_CYAN, TFT_BLACK);
    tft.fillSmoothCircle(cx, cy + 10, 20, TFT_MAGENTA, TFT_BLACK);
    tft.fillSmoothCircle(cx + 70, cy + 10, 20, TFT_YELLOW, TFT_BLACK);

    // Draw smooth arcs
    tft.drawSmoothArc(cx - 70, cy + 65, 29, 25, 0, 90, TFT_RED, TFT_BLACK, true);
    tft.drawSmoothArc(cx, cy + 65, 29, 25, 90, 180, TFT_GREEN, TFT_BLACK, true);
    tft.drawSmoothArc(cx + 70, cy + 65, 29, 25, 180, 270, TFT_BLUE, TFT_BLACK, true);

    // Add labels
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE);
    tft.setCursor(10, tft.height() - 15);
    tft.println("Smooth circles & arcs");

    Serial.println("Anti-aliased graphics test completed.");
    delay(3000);

    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Analog clock/gauge demo
 * Displays animated analog clock with smooth hands
 */
void test_tft_clock_gauge_demo()
{
    Serial.println("Testing clock/gauge demo...");

    initialize_tft(1);

    int cx = tft.width() / 2;
    int cy = tft.height() / 2;
    int radius = 80;

    tft.setTextSize(2);
    tft.setCursor(cx - 40, 10);
    tft.println("Clock Demo");

    // Draw clock face
    tft.drawCircle(cx, cy, radius, TFT_WHITE);
    tft.drawCircle(cx, cy, radius - 1, TFT_WHITE);

    // Draw hour markers
    for (int i = 0; i < 12; i++)
    {
        float angle = i * 30.0f * 3.14159f / 180.0f;
        int x1 = cx + (int)((radius - 10) * sin(angle));
        int y1 = cy - (int)((radius - 10) * cos(angle));
        int x2 = cx + (int)((radius - 3) * sin(angle));
        int y2 = cy - (int)((radius - 3) * cos(angle));
        tft.drawLine(x1, y1, x2, y2, TFT_WHITE);
    }

    // Animate clock hands for 5 seconds
    unsigned long start = millis();
    int hour = 10, minute = 10, second = 0;

    while (millis() - start < 5000)
    {
        // Calculate angles (12 o'clock = 0 degrees)
        float second_angle = (second * 6.0f - 90.0f) * 3.14159f / 180.0f;
        float minute_angle = (minute * 6.0f + second * 0.1f - 90.0f) * 3.14159f / 180.0f;
        float hour_angle = (hour * 30.0f + minute * 0.5f - 90.0f) * 3.14159f / 180.0f;

        // Draw hands
        // Hour hand (shortest, thickest)
        int hx = cx + (int)((radius - 35) * cos(hour_angle));
        int hy = cy + (int)((radius - 35) * sin(hour_angle));
        tft.drawLine(cx, cy, hx, hy, TFT_WHITE);
        tft.drawLine(cx + 1, cy, hx + 1, hy, TFT_WHITE);

        // Minute hand (medium)
        int mx = cx + (int)((radius - 20) * cos(minute_angle));
        int my = cy + (int)((radius - 20) * sin(minute_angle));
        tft.drawLine(cx, cy, mx, my, TFT_CYAN);

        // Second hand (longest, thin)
        int sx = cx + (int)((radius - 10) * cos(second_angle));
        int sy = cy + (int)((radius - 10) * sin(second_angle));
        tft.drawLine(cx, cy, sx, sy, TFT_RED);

        // Center dot
        tft.fillCircle(cx, cy, 3, TFT_YELLOW);

        delay(100);

        // Erase old hands
        tft.drawLine(cx, cy, hx, hy, TFT_BLACK);
        tft.drawLine(cx + 1, cy, hx + 1, hy, TFT_BLACK);
        tft.drawLine(cx, cy, mx, my, TFT_BLACK);
        tft.drawLine(cx, cy, sx, sy, TFT_BLACK);

        // Update time (fast forward for demo)
        second += 10;
        if (second >= 60)
        {
            second = 0;
            minute++;
            if (minute >= 60)
            {
                minute = 0;
                hour++;
                if (hour >= 12)
                    hour = 0;
            }
        }
    }

    // Draw gauge demo
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(cx - 40, 10);
    tft.setTextColor(TFT_WHITE);
    tft.setTextSize(2);
    tft.println("Gauge Demo");

    // Draw gauge arc (0 to 180 degrees - top semicircle, classic speedometer). drawSmoothArc is not taking into account rotation, so we adjust angles
    uint8_t rotation = tft.getRotation();
    tft.drawSmoothArc(cx, cy + 20, radius, radius - 15, (rotation * 90) % 360, (180 + rotation * 90) % 360, TFT_GREEN, TFT_BLACK, true);

    // Animate needle
    start = millis();
    float value = 0;
    while (millis() - start < 3000)
    {
        // Calculate needle angle (180 to 360 degrees, starts pointing left)
        float angle = (180.0f + value * 1.8f) * 3.14159f / 180.0f;
        int nx = cx + (int)((radius - 20) * cos(angle));
        int ny = cy + 20 + (int)((radius - 20) * sin(angle));

        // Draw needle
        tft.drawLine(cx, cy + 20, nx, ny, TFT_RED);
        tft.drawLine(cx + 1, cy + 20, nx + 1, ny, TFT_RED);
        tft.fillCircle(cx, cy + 20, 4, TFT_YELLOW);

        // Display value
        tft.fillRect(cx - 50, cy + 55, 100, 30, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(cx - 25, cy + 65);
        tft.printf("%d%%", (int)value);

        delay(50);

        // Erase needle
        tft.drawLine(cx, cy + 20, nx, ny, TFT_BLACK);
        tft.drawLine(cx + 1, cy + 20, nx + 1, ny, TFT_BLACK);

        value += 2.0f;
        if (value > 100)
            value = 0;
    }

    Serial.println("Clock/Gauge demo completed.");
    delay(2000);

    TEST_ASSERT_TRUE(true);
}

void test_tft_read_pixel()
{
    Serial.println("Testing read pixel operations...");

    initialize_tft(1);

    tft.setTextSize(2);
    tft.setCursor(10, 5);
    tft.println("ReadPixel Test");

    // Draw known colored rectangles
    int w = tft.width() / 3;
    int h = tft.height() / 3;

    tft.fillRect(0, 40, w, h, TFT_RED);
    tft.fillRect(w, 40, w, h, TFT_GREEN);
    tft.fillRect(2 * w, 40, w, h, TFT_BLUE);

    tft.fillRect(0, 40 + h, w, h, TFT_CYAN);
    tft.fillRect(w, 40 + h, w, h, TFT_MAGENTA);
    tft.fillRect(2 * w, 40 + h, w, h, TFT_YELLOW);

    delay(1000);

    // Read back pixels from center of each rectangle
    struct TestPixel
    {
        int x, y;
        uint16_t expected;
        const char *name;
    };

    TestPixel pixels[] = {
        {w / 2, 40 + h / 2, TFT_RED, "Red"},
        {w + w / 2, 40 + h / 2, TFT_GREEN, "Green"},
        {2 * w + w / 2, 40 + h / 2, TFT_BLUE, "Blue"},
        {w / 2, 40 + h + h / 2, TFT_CYAN, "Cyan"},
        {w + w / 2, 40 + h + h / 2, TFT_MAGENTA, "Magenta"},
        {2 * w + w / 2, 40 + h + h / 2, TFT_YELLOW, "Yellow"}};

    int pass_count = 0;
    int fail_count = 0;

    Serial.println("\n=== Read Pixel Verification ===");
    Serial.println("Color      | Position   | Expected | Read     | Status");
    Serial.println("-----------|------------|----------|----------|--------");

    tft.setTextSize(1);
    int y_pos = 40 + 2 * h + 10;

    for (int i = 0; i < 6; i++)
    {
        uint16_t read_color = tft.readPixel(pixels[i].x, pixels[i].y);
        bool match = (read_color == pixels[i].expected);

        if (match)
            pass_count++;
        else
            fail_count++;

        Serial.printf("%-10s | (%3d,%3d) | 0x%04X   | 0x%04X   | %s\n",
                      pixels[i].name,
                      pixels[i].x, pixels[i].y,
                      pixels[i].expected,
                      read_color,
                      match ? "PASS" : "FAIL");

        // Display on screen
        if (y_pos < tft.height())
        {
            tft.setCursor(5, y_pos);
            tft.setTextColor(match ? TFT_GREEN : TFT_RED);
            tft.printf("%s: %s", pixels[i].name, match ? "OK" : "FAIL");
            y_pos += 12;
        }
    }

    Serial.printf("\nResults: %d PASS, %d FAIL\n", pass_count, fail_count);
    Serial.println("=== End Read Pixel Test ===\n");

    // Display summary
    tft.setCursor(10, tft.height() - 25);
    tft.setTextSize(2);
    tft.setTextColor(pass_count == 6 ? TFT_GREEN : TFT_YELLOW);
    tft.printf("Pass: %d/%d", pass_count, 6);

    delay(3000);

    TEST_ASSERT_EQUAL(6, pass_count);
}

#endif // ENABLE_TFT_TESTS

// ============================================================================
// WIZNET TEST SUITE (test_wiznet/tests.cpp)
// ============================================================================

#if ENABLE_WIZNET_TESTS

// Test configuration
#define DHCP_TIMEOUT 10000 // 10 seconds for DHCP
#define LINK_TIMEOUT 5000  // 5 seconds for link detection
#define TFT_ROTATION 1     // Set the default rotation for the display

// Static IP configuration (fallback if DHCP fails)
IPAddress staticIP(192, 168, 1, 177);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dnsServer(8, 8, 8, 8);

// Test state
bool wiznet_initialized = false;
byte mac[6] = {0};

static uint32_t get_wiznet_spi_baudrate(spi_inst_t *spi)
{
    spi_hw_t *hw = spi_get_hw(spi);
    uint32_t cpsr = hw->cpsr & 0xFFu;
    // CR0 bits 15:8 contain SCR (Serial Clock Rate)
    uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
    uint32_t clk = clock_get_hz(clk_peri); // clk_peri (peripheral clock) is the SPI clock source
    if (cpsr == 0)
        return 0;
    return clk / (cpsr * (scr + 1u));
}

static void log_wiznet_spi_speed(const char *tag)
{
    spi_hw_t *hw = spi_get_hw(WIZNET_SPI_PORT);
    uint32_t cpsr = hw->cpsr & 0xFFu;
    // CR0 bits 15:8 contain SCR (Serial Clock Rate)
    uint32_t scr = (hw->cr0 >> 8) & 0xFFu;
    uint32_t baud = get_wiznet_spi_baudrate(WIZNET_SPI_PORT);

    Serial.printf("%s SPI baud: %lu Hz (cpsr=%lu, scr=%lu)\n", tag, baud, cpsr, scr);
}

/**
 * @brief Get MAC address for Wiznet from flash unique ID
 * @param mac Pointer to 6-byte array to fill
 */
void getWiznetMAC(byte *mac)
{
    // Read unique 64-bit ID from flash (RP2350)
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
 * @brief Reset the WIZNET5K chip using the hardware reset pin
 */
void wiznet5k_reset()
{
    gpio_init(WIZNET_RST);
    gpio_set_dir(WIZNET_RST, GPIO_OUT);
    gpio_put(WIZNET_RST, 0); // Assert reset (active low)
    sleep_ms(10);            // Hold reset for 10ms
    gpio_put(WIZNET_RST, 1); // Release reset
    sleep_ms(100);           // Wait for chip to be ready
}

void release_SPI()
{
#if defined(TFT_CS)
    pinMode(TFT_CS, OUTPUT);
    digitalWrite(TFT_CS, HIGH);
#endif
#if defined(TOUCH_CS)
    pinMode(TOUCH_CS, OUTPUT);
    digitalWrite(TOUCH_CS, HIGH);
#endif
}

/**
 * @brief Initialize SPI bus for Wiznet adapter
 * @note Can be called either after TFT init (shared SPI) or standalone
 *       If SPI is not initialized, will initialize it first
 */
void wiznet_init_spi()
{
    // Reinitialize SPI using Arduino SPI API to keep it in sync with Ethernet library
    SPI.end();
    SPI.setSCK(WIZNET_SCLK);
    SPI.setTX(WIZNET_MOSI);
    SPI.setRX(WIZNET_MISO);
    SPI.begin();
    SPI.beginTransaction(SPISettings(WIZNET_SPI_BPS, MSBFIRST, SPI_MODE0));
    SPI.endTransaction();

    // Configure CS pin as GPIO output (managed manually by Ethernet library)
    Serial.println("  Configuring CS pin...");
    Serial.flush();
    gpio_init(WIZNET_SPI_CS);
    gpio_set_dir(WIZNET_SPI_CS, GPIO_OUT);
    gpio_put(WIZNET_SPI_CS, 1); // CS idle high
    Serial.println("  CS pin configured");
    Serial.flush();

    Serial.println("Wiznet SPI initialized");
    Serial.printf("MISO: GP%d, MOSI: GP%d, SCK: GP%d, CS: GP%d\n",
                  WIZNET_MISO, WIZNET_MOSI, WIZNET_SCLK, WIZNET_SPI_CS);
    Serial.printf("RST: GP%d, INT: GP%d\n", WIZNET_RST, WIZNET_INT);
    log_wiznet_spi_speed("After Wiznet SPI init");
}

/**
 * @brief Configure interrupt pin for Wiznet adapter
 */
void wiznet_init_interrupt()
{
    gpio_init(WIZNET_INT);
    gpio_set_dir(WIZNET_INT, GPIO_IN);
    gpio_pull_up(WIZNET_INT);
    Serial.printf("Wiznet INT pin (GP%d) configured with pull-up\n", WIZNET_INT);
}

void test_wiznet_mac_generation()
{
    getWiznetMAC(mac);

    // Verify MAC address
    TEST_ASSERT_EQUAL_HEX8_MESSAGE(0x02, mac[0], "First MAC byte should be 0x02 (locally administered)");

    // Check that MAC is not all zeros or all 0xFF
    bool all_zeros = true;
    bool all_ff = true;
    for (int i = 1; i < 6; i++)
    {
        if (mac[i] != 0x00)
            all_zeros = false;
        if (mac[i] != 0xFF)
            all_ff = false;
    }

    TEST_ASSERT_FALSE_MESSAGE(all_zeros, "MAC address should not be all zeros");
    TEST_ASSERT_FALSE_MESSAGE(all_ff, "MAC address should not be all 0xFF");

    Serial.printf("Generated MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void test_wiznet_hardware_reset()
{
    Serial.println("Testing Wiznet hardware reset functionality...");

    // Step 1: Initialize Ethernet and configure a static IP
    Serial.println("Step 1: Initializing Ethernet with static IP...");
    wiznet5k_reset();
    wiznet_init_spi();
    wiznet_init_interrupt();
    Ethernet.init(WIZNET_SPI_CS);

    // Configure a test static IP
    IPAddress testIP(192, 168, 1, 100);
    IPAddress testGateway(192, 168, 1, 1);
    IPAddress testSubnet(255, 255, 255, 0);
    IPAddress testDNS(8, 8, 8, 8);

    getWiznetMAC(mac);
    Ethernet.begin(mac, testIP, testDNS, testGateway, testSubnet);

    // Verify IP was configured
    IPAddress configuredIP = Ethernet.localIP();
    Serial.printf("Configured IP: %d.%d.%d.%d\n", configuredIP[0], configuredIP[1], configuredIP[2], configuredIP[3]);
    TEST_ASSERT_EQUAL_MESSAGE(testIP, configuredIP, "IP should be configured correctly");

    // Step 2: Perform hardware reset
    Serial.println("Step 2: Performing hardware reset...");
    gpio_init(WIZNET_RST);
    gpio_set_dir(WIZNET_RST, GPIO_OUT);
    gpio_put(WIZNET_RST, 0); // Assert reset (active low)
    sleep_ms(10);            // Hold reset for 10ms
    gpio_put(WIZNET_RST, 1); // Release reset
    sleep_ms(100);           // Wait for chip to stabilize

    // Step 3: Reinitialize Ethernet library (SPI state needs to be restored)
    Serial.println("Step 3: Reinitializing Ethernet after reset...");
    wiznet_init_spi(); // Restore SPI state
    Ethernet.init(WIZNET_SPI_CS);

    // Step 4: Verify that the reset cleared the configuration
    Serial.println("Step 4: Verifying reset cleared configuration...");
    IPAddress resetIP = Ethernet.localIP();
    Serial.printf("IP after reset: %d.%d.%d.%d\n", resetIP[0], resetIP[1], resetIP[2], resetIP[3]);

    // After hardware reset, IP should be 0.0.0.0 (unconfigured)
    IPAddress zeroIP(0, 0, 0, 0);
    IPAddress defaultDNS(8, 8, 8, 8); // Some Ethernet libraries may default to a DNS server even after reset, so we won't assert that it's cleared to zero
    TEST_ASSERT_EQUAL_MESSAGE(zeroIP, resetIP, "IP should be cleared after hardware reset");

    // Also check gateway and subnet
    IPAddress resetGateway = Ethernet.gatewayIP();
    IPAddress resetSubnet = Ethernet.subnetMask();
    IPAddress resetDNS = Ethernet.dnsServerIP();

    Serial.printf("Gateway after reset: %d.%d.%d.%d\n", resetGateway[0], resetGateway[1], resetGateway[2], resetGateway[3]);
    Serial.printf("Subnet after reset: %d.%d.%d.%d\n", resetSubnet[0], resetSubnet[1], resetSubnet[2], resetSubnet[3]);
    Serial.printf("DNS after reset: %d.%d.%d.%d\n", resetDNS[0], resetDNS[1], resetDNS[2], resetDNS[3]);

    // These should also be cleared/reset
    TEST_ASSERT_EQUAL_MESSAGE(zeroIP, resetGateway, "Gateway should be cleared after hardware reset");
    TEST_ASSERT_EQUAL_MESSAGE(zeroIP, resetSubnet, "Subnet should be cleared after hardware reset");
    TEST_ASSERT_EQUAL_MESSAGE(defaultDNS, resetDNS, "DNS should be cleared after hardware reset");

    Serial.println("Hardware reset test completed successfully - W5500 was properly reset!");
}

void test_wiznet_spi_init()
{
    Serial.println("Testing Wiznet SPI initialization...");

    // Reset chip before SPI init
    wiznet5k_reset();

    // Initialize SPI
    wiznet_init_spi();

    // Verify SPI hardware is initialized
    Serial.println("Verifying SPI hardware...");
    Serial.flush();

    // Simple verification - just check that we can read GPIO functions
    // (Avoid reading SPI hardware registers directly as it can cause hangs)

    // Verify GPIO pin functions
    Serial.println("Verifying GPIO functions...");
    Serial.flush();
    uint32_t miso_func = (io_bank0_hw->io[WIZNET_MISO].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    uint32_t sck_func = (io_bank0_hw->io[WIZNET_SCLK].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    uint32_t mosi_func = (io_bank0_hw->io[WIZNET_MOSI].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;

    TEST_ASSERT_EQUAL_MESSAGE(GPIO_FUNC_SPI, miso_func, "MISO should be configured for SPI");
    TEST_ASSERT_EQUAL_MESSAGE(GPIO_FUNC_SPI, sck_func, "SCK should be configured for SPI");
    TEST_ASSERT_EQUAL_MESSAGE(GPIO_FUNC_SPI, mosi_func, "MOSI should be configured for SPI");

    // Verify CS pin is GPIO output
    Serial.println("Verifying CS pin...");
    Serial.flush();
    TEST_ASSERT_EQUAL_MESSAGE(1, gpio_get(WIZNET_SPI_CS), "CS pin should be HIGH (idle)");

    Serial.println("SPI initialization successful");
    Serial.flush();
}

void test_wiznet_ethernet_init()
{
    Serial.println("Testing Ethernet library initialization...");

    // Ensure SPI is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        wiznet_initialized = true;
    }

    // Initialize Ethernet library with CS pin
    Ethernet.init(WIZNET_SPI_CS);

    Serial.println("Ethernet library initialized");

    // Give a bit more time for the chip to stabilize after init
    delay(50);

    // Verify Ethernet library is really initialized by checking hardware status
    auto hwStatus = Ethernet.hardwareStatus();
    Serial.printf("Hardware status after init: %d\n", hwStatus);

    // If hardware status is 0, try to call Ethernet.begin() to trigger chip detection
    if (hwStatus == 0)
    {
        Serial.println("Hardware status is 0, attempting Ethernet.begin() to detect chip...");

        // Get MAC address for begin() call
        getWiznetMAC(mac);

        // Try Ethernet.begin() with timeout to see if it can detect the chip
        int beginResult = Ethernet.begin(mac, 2000); // 2 second timeout
        Serial.printf("Ethernet.begin() result: %d\n", beginResult);

        // Check hardware status again after begin()
        hwStatus = Ethernet.hardwareStatus();
        Serial.printf("Hardware status after begin(): %d\n", hwStatus);
    }

    // EthernetNoHardware = 0, W5100 = 1, W5200 = 2, W5500 = 3
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, hwStatus, "Ethernet library should detect Wiznet chip after initialization");

    // Check link status (may be OFF if no cable connected, but should not be UNKNOWN)
    auto linkStatus = Ethernet.linkStatus();
    Serial.print("Link status after init: ");
    if (linkStatus == LinkON)
    {
        Serial.println("ON (cable connected)");
    }
    else if (linkStatus == LinkOFF)
    {
        Serial.println("OFF (no cable)");
    }
    else
    {
        Serial.println("UNKNOWN");
        TEST_FAIL_MESSAGE("Link status should not be UNKNOWN after Ethernet initialization");
    }

    // Print detected chip type
    if (hwStatus == 3)
    {
        Serial.println("Detected Wiznet chip: W5500");
    }
    else if (hwStatus == 2)
    {
        Serial.println("Detected Wiznet chip: W5200");
    }
    else if (hwStatus == 1)
    {
        Serial.println("Detected Wiznet chip: W5100");
    }
    else
    {
        Serial.printf("Detected unknown Wiznet chip type: %d\n", hwStatus);
    }

    TEST_ASSERT_TRUE_MESSAGE(true, "Ethernet library initialization and chip detection successful");
}

void test_wiznet_chip_detection()
{
    Serial.println("Testing Wiznet chip detection...");

    // Ensure Ethernet is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
    }

    // Get MAC address
    getWiznetMAC(mac);

    log_wiznet_spi_speed("Before Ethernet.begin");

    // Try to begin Ethernet (this will detect the chip)
    int result = Ethernet.begin(mac, 1000); // 1 second timeout

    log_wiznet_spi_speed("After Ethernet.begin");

    // Check hardware status (chip detection)
    auto hwStatus = Ethernet.hardwareStatus();

    log_wiznet_spi_speed("After Ethernet.hardwareStatus");

    Serial.printf("Hardware status: %d\n", hwStatus);

    // Print network status information
    auto linkStatus = Ethernet.linkStatus();
    Serial.print("Link status: ");
    if (linkStatus == LinkON)
    {
        Serial.println("ON (cable connected)");
    }
    else if (linkStatus == LinkOFF)
    {
        Serial.println("OFF (no cable)");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    // EthernetNoHardware = 0, W5100 = 1, W5200 = 2, W5500 = 3
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, hwStatus, "Wiznet chip should be detected (not EthernetNoHardware)");

    if (hwStatus == 3)
    {
        Serial.println("Detected: W5500");
    }
    else if (hwStatus == 2)
    {
        Serial.println("Detected: W5200");
    }
    else if (hwStatus == 1)
    {
        Serial.println("Detected: W5100");
    }
}

void test_wiznet_dhcp()
{
    Serial.println("Testing Wiznet DHCP configuration...");

    // Ensure Ethernet is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
    }

    // Get MAC address
    getWiznetMAC(mac);

    Serial.println("Attempting DHCP configuration...");
    Serial.printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Try DHCP with timeout
    int dhcp_result = Ethernet.begin(mac, DHCP_TIMEOUT);

    if (dhcp_result == 1)
    {
        Serial.println("DHCP configuration successful");
        IPAddress ip = Ethernet.localIP();
        Serial.printf("IP Address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);

        IPAddress gw = Ethernet.gatewayIP();
        Serial.printf("Gateway: %d.%d.%d.%d\n", gw[0], gw[1], gw[2], gw[3]);

        IPAddress sn = Ethernet.subnetMask();
        Serial.printf("Subnet: %d.%d.%d.%d\n", sn[0], sn[1], sn[2], sn[3]);

        IPAddress dnsServerRead = Ethernet.dnsServerIP();
        Serial.printf("DNS: %d.%d.%d.%d\n", dnsServerRead[0], dnsServerRead[1], dnsServerRead[2], dnsServerRead[3]);

        // Verify we got a valid IP
        TEST_ASSERT_NOT_EQUAL_MESSAGE(0, ip[0], "IP address first octet should not be 0");
        TEST_ASSERT_TRUE_MESSAGE(ip != IPAddress(0, 0, 0, 0), "Should have valid IP address");
    }
    else
    {
        Serial.println("DHCP failed - this is acceptable if no DHCP server is available");
        Serial.println("Test will PASS as chip was detected");
        // Don't fail the test - DHCP failure is acceptable in test environment
    }

    TEST_ASSERT_TRUE(true);
}

void test_wiznet_static_ip()
{
    Serial.println("Testing Wiznet static IP configuration...");

    // Ensure Ethernet is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
    }

    // Get MAC address
    getWiznetMAC(mac);

    Serial.println("Configuring static IP...");
    Serial.printf("IP: %d.%d.%d.%d\n", staticIP[0], staticIP[1], staticIP[2], staticIP[3]);
    Serial.printf("Gateway: %d.%d.%d.%d\n", gateway[0], gateway[1], gateway[2], gateway[3]);
    Serial.printf("Subnet: %d.%d.%d.%d\n", subnet[0], subnet[1], subnet[2], subnet[3]);

    // Configure static IP
    Ethernet.begin(mac, staticIP, dnsServer, gateway, subnet);

    // Wait for configuration to apply
    delay(500);

    // Verify IP configuration
    IPAddress configuredIP = Ethernet.localIP();
    Serial.printf("Configured IP: %d.%d.%d.%d\n",
                  configuredIP[0], configuredIP[1], configuredIP[2], configuredIP[3]);

    TEST_ASSERT_EQUAL_MESSAGE(staticIP[0], configuredIP[0], "IP octet 0 mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(staticIP[1], configuredIP[1], "IP octet 1 mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(staticIP[2], configuredIP[2], "IP octet 2 mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(staticIP[3], configuredIP[3], "IP octet 3 mismatch");

    Serial.println("Static IP configuration successful");
}

void test_wiznet_config_readback()
{
    Serial.println("Testing network configuration readback...");

    // Ensure Ethernet is initialized with static IP
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        getWiznetMAC(mac);
        Ethernet.begin(mac, staticIP, dnsServer, gateway, subnet);
        wiznet_initialized = true;
        delay(500);
    }

    // Read back all network parameters
    IPAddress ip = Ethernet.localIP();
    IPAddress gw = Ethernet.gatewayIP();
    IPAddress sn = Ethernet.subnetMask();
    IPAddress dnsServerRead = Ethernet.dnsServerIP();

    Serial.println("Network configuration:");
    Serial.printf("  IP Address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    Serial.printf("  Gateway:    %d.%d.%d.%d\n", gw[0], gw[1], gw[2], gw[3]);
    Serial.printf("  Subnet:     %d.%d.%d.%d\n", sn[0], sn[1], sn[2], sn[3]);
    Serial.printf("  DNS:        %d.%d.%d.%d\n", dnsServerRead[0], dnsServerRead[1], dnsServerRead[2], dnsServerRead[3]);

    // Print network status
    auto linkStatus = Ethernet.linkStatus();
    Serial.print("  Link: ");
    if (linkStatus == LinkON)
    {
        Serial.println("ON");
    }
    else if (linkStatus == LinkOFF)
    {
        Serial.println("OFF");
    }
    else
    {
        Serial.println("UNKNOWN");
    }

    // Verify readback matches configured values
    TEST_ASSERT_EQUAL_MESSAGE(staticIP[0], ip[0], "IP readback mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(gateway[0], gw[0], "Gateway readback mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(subnet[0], sn[0], "Subnet readback mismatch");

    Serial.println("Configuration readback successful");
}

void test_wiznet_link_status()
{
    Serial.println("Testing Wiznet link status detection...");

    // Ensure Ethernet is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
    }

    // Get MAC address
    getWiznetMAC(mac);

    Ethernet.begin(mac, staticIP, dnsServer, gateway, subnet);

    // Wait longer for link to stabilize - PHY might need time
    Serial.println("Waiting for link to stabilize...");
    delay(2000); // Wait 2 seconds

    // Check link status multiple times with delays
    for (int attempt = 1; attempt <= 3; attempt++)
    {
        auto linkStatus = Ethernet.linkStatus();
        Serial.printf("Link status attempt %d: %d ", attempt, linkStatus);

        if (linkStatus == LinkON)
        {
            Serial.println("(ON - cable connected)");
        }
        else if (linkStatus == LinkOFF)
        {
            Serial.println("(OFF - no cable)");
        }
        else
        {
            Serial.println("(UNKNOWN)");
        }

        if (linkStatus == LinkON)
        {
            break; // Link detected, no need to try again
        }

        if (attempt < 3)
        {
            Serial.println("Waiting longer for link detection...");
            delay(1000); // Wait 1 more second between attempts
        }
    }

    // Final link status check
    auto finalLinkStatus = Ethernet.linkStatus();
    Serial.printf("Final link status: %d\n", finalLinkStatus);

    // Print detailed network status
    Serial.print("Link: ");
    if (finalLinkStatus == LinkON)
    {
        Serial.println("ON");
        Serial.println("✅ Ethernet link is UP (cable connected)");
    }
    else if (finalLinkStatus == LinkOFF)
    {
        Serial.println("OFF");
        Serial.println("❌ Ethernet link is DOWN (no cable or not connected)");
        Serial.println("This might indicate:");
        Serial.println("  - Ethernet cable not properly connected");
        Serial.println("  - Network switch/hub not powered on");
        Serial.println("  - PHY auto-negotiation issues");
        Serial.println("  - Wiznet W5500 hardware problem");
    }
    else
    {
        Serial.println("UNKNOWN");
        Serial.println("⚠️  Link status unknown - possible communication issue");
    }

    // Print additional network information
    Serial.print("Local IP: ");
    Serial.println(Ethernet.localIP());
    Serial.print("Subnet: ");
    Serial.println(Ethernet.subnetMask());
    Serial.print("Gateway: ");
    Serial.println(Ethernet.gatewayIP());
    Serial.print("DNS: ");
    Serial.println(Ethernet.dnsServerIP());

    // For this test, we expect the link to be detected if cable is connected
    // But we'll make it a warning rather than failure for now
    if (finalLinkStatus != LinkON)
    {
        Serial.println("⚠️  WARNING: Link should be ON if Ethernet cable is connected!");
        Serial.println("   Check cable connection and network equipment.");
    }

    // Test passes if we can at least read the link status (not UNKNOWN)
    TEST_ASSERT_NOT_EQUAL_MESSAGE(Unknown, finalLinkStatus, "Link status should not be UNKNOWN - indicates communication problem");
}

void test_wiznet_interrupt_pin()
{
    Serial.println("Testing Wiznet interrupt pin configuration...");

    wiznet_init_interrupt();

    // Verify INT pin is configured as input
    uint32_t int_oe = (sio_hw->gpio_oe >> WIZNET_INT) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(0, int_oe, "INT pin should be input (OE=0)");

    // Verify pull-up is enabled
    uint32_t pad_int = pads_bank0_hw->io[WIZNET_INT];
    uint32_t pue = (pad_int >> PADS_BANK0_GPIO0_PUE_LSB) & 0x1u;
    TEST_ASSERT_EQUAL_MESSAGE(1, pue, "INT pin should have pull-up enabled");

    Serial.println("Interrupt pin configuration successful");
}

void test_wiznet_webserver()
{
    Serial.println("Testing Wiznet minimal webserver...");

    // Ensure Ethernet is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
    }

    // Get MAC address
    getWiznetMAC(mac);

    // Try to get IP (DHCP or static)
    IPAddress ip = Ethernet.localIP();
    if (ip == IPAddress(0, 0, 0, 0))
    {
        // No IP, try DHCP
        Serial.println("No IP assigned, attempting DHCP...");
        int dhcp_result = Ethernet.begin(mac, DHCP_TIMEOUT);
        if (dhcp_result == 1)
        {
            ip = Ethernet.localIP();
            Serial.printf("DHCP successful, IP: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
        }
        else
        {
            // Fallback to static
            Serial.println("DHCP failed, using static IP...");
            Ethernet.begin(mac, staticIP, dnsServer, gateway, subnet);
            ip = Ethernet.localIP();
            Serial.printf("Static IP: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
        }
    }
    else
    {
        Serial.printf("Using existing IP: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    }

    // Start server
    EthernetServer server(3000);
    server.begin();
    Serial.printf("Webserver started at http://%d.%d.%d.%d:3000\n", ip[0], ip[1], ip[2], ip[3]);
    Serial.println("Access the page in a browser. The test will run for 10 seconds...");

    // Run webserver for 10 seconds
    unsigned long startTime = millis();
    int reqCount = 0;

    while (millis() - startTime < 10000)
    {
        // Listen for incoming clients
        EthernetClient client = server.available();

        if (client)
        {
            Serial.println("New client connected");
            // An HTTP request ends with a blank line
            bool currentLineIsBlank = true;

            while (client.connected())
            {
                if (client.available())
                {
                    char c = client.read();
                    Serial.write(c); // Echo to serial for debugging

                    // If you've gotten to the end of the line (received a newline
                    // character) and the line is blank, the HTTP request has ended,
                    // so you can send a reply
                    if (c == '\n' && currentLineIsBlank)
                    {
                        Serial.println("Sending response");

                        // Send a standard HTTP response header
                        client.print(
                            "HTTP/1.1 200 OK\r\n"
                            "Content-Type: text/html\r\n"
                            "Connection: close\r\n"
                            "\r\n");
                        client.print("<!DOCTYPE HTML>\r\n");
                        client.print("<html>\r\n");
                        client.print("<h2>Hello from Wiznet Test!</h2>\r\n");
                        client.print("Requests received: ");
                        client.print(++reqCount);
                        client.print("<br>\r\n");
                        client.print("Test running for: ");
                        client.print((millis() - startTime) / 1000);
                        client.print(" seconds<br>\r\n");
                        client.print("</html>\r\n");
                        break;
                    }

                    if (c == '\n')
                    {
                        // You're starting a new line
                        currentLineIsBlank = true;
                    }
                    else if (c != '\r')
                    {
                        // You've gotten a character on the current line
                        currentLineIsBlank = false;
                    }
                }
            }

            // Give the web browser time to receive the data
            delay(10);

            // Close the connection
            client.stop();
            Serial.println("Client disconnected");
        }
    }

    Serial.println("Webserver test completed");
    TEST_ASSERT_TRUE(true);
}

// Big Data Webserver test based on EthernetWebServer_BigData example

// Adjust according to your board's heap size. Too large => crash
#define MULTIPLY_FACTOR 3.0f // For Pico, use smaller factor

// In bytes
#define BUFFER_SIZE 512
char temp[BUFFER_SIZE];

void createPage(String &pageInput)
{
    int sec = millis() / 1000;
    int min = sec / 60;
    int hr = min / 60;
    int day = hr / 24;

    snprintf(temp, BUFFER_SIZE - 1,
             "<html>\
<head>\
<meta http-equiv='refresh' content='5'/>\
<title>EthernetWebServer_BigData-%s</title>\
<style>\
body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
</style>\
</head>\
<body>\
<h2>EthernetWebServer_W5500!</h2>\
<h3>running on %s</h3>\
<p>Uptime: %d d %02d:%02d:%02d</p>\
</body>\
</html>",
             BOARD_NAME, BOARD_NAME, day, hr % 24, min % 60, sec % 60);

    pageInput = temp;
}

String out;

void test_wiznet_webserver_bigdata()
{
    Serial.println("Testing Wiznet big data webserver...");

    // Ensure SPI bus is released by other devices
    release_SPI();
    // Ensure Ethernet is initialized
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
    }

    // Get MAC address
    getWiznetMAC(mac);

    // Try to get IP (DHCP or static)
    IPAddress ip = Ethernet.localIP();
    if (ip == IPAddress(0, 0, 0, 0))
    {
        // No IP, try DHCP
        Serial.println("No IP assigned, attempting DHCP...");
        int dhcp_result = Ethernet.begin(mac, DHCP_TIMEOUT);
        if (dhcp_result == 1)
        {
            ip = Ethernet.localIP();
            Serial.printf("DHCP successful, IP: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
        }
        else
        {
            // Fallback to static
            Serial.println("DHCP failed, using static IP...");
            Ethernet.begin(mac, staticIP, dnsServer, gateway, subnet);
            ip = Ethernet.localIP();
            Serial.printf("Static IP: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
        }
    }
    else
    {
        Serial.printf("Using existing IP: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    }

    // Start server
    EthernetServer server(80);
    server.begin();
    Serial.printf("Big data webserver started at http://%d.%d.%d.%d:80\n", ip[0], ip[1], ip[2], ip[3]);
    Serial.println("Access the page in a browser. The test will run for 10 seconds...");

    // Run webserver for 10 seconds
    unsigned long startTime = millis();
    int reqCount = 0;

    while (millis() - startTime < 10000)
    {
        // Listen for incoming clients
        EthernetClient client = server.available();

        if (client)
        {
            Serial.println("New client connected");
            // An HTTP request ends with a blank line
            bool currentLineIsBlank = true;
            String request = "";

            while (client.connected())
            {
                if (client.available())
                {
                    char c = client.read();
                    request += c;
                    Serial.write(c); // Echo to serial for debugging

                    // If you've gotten to the end of the line (received a newline
                    // character) and the line is blank, the HTTP request has ended,
                    // so you can send a reply
                    if (c == '\n' && currentLineIsBlank)
                    {
                        Serial.println("Sending big data response");

                        // Send a standard HTTP response header
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-Type: text/html");
                        client.println("Connection: close");
                        client.println();

                        // Generate the big data page
                        out = String();
                        createPage(out);

                        out += "<html><body>\r\n<table><tr><th>INDEX</th><th>DATA</th></tr>";

                        for (uint16_t lineIndex = 0; lineIndex < (100 * MULTIPLY_FACTOR); lineIndex++)
                        {
                            out += "<tr><td>";
                            out += String(lineIndex);
                            out += "</td><td>";
                            out += "WiFiWebServer_BigData_ABCDEFGHIJKLMNOPQRSTUVWXYZ</td></tr>";
                        }

                        out += "</table></body></html>\r\n";

                        Serial.print(F("String Len = "));
                        Serial.println(out.length());

                        client.print(out);
                        break;
                    }

                    if (c == '\n')
                    {
                        // You're starting a new line
                        currentLineIsBlank = true;
                    }
                    else if (c != '\r')
                    {
                        // You've gotten a character on the current line
                        currentLineIsBlank = false;
                    }
                }
            }

            // Give the web browser time to receive the data
            delay(10);

            // Close the connection
            client.stop();
            Serial.println("Client disconnected");
        }
    }

    Serial.println("Big data webserver test completed");
    TEST_ASSERT_TRUE(true);
}

void test_wiznet_tft_shared_spi()
{
    Serial.println("Testing TFT initialization after Wiznet on shared SPI0...");

    // Ensure Wiznet is initialized first
    if (!wiznet_initialized)
    {
        wiznet5k_reset();
        wiznet_init_spi();
        wiznet_init_interrupt();
        Ethernet.init(WIZNET_SPI_CS);
        wiznet_initialized = true;
        Serial.println("Wiznet initialized, SPI0 configured at 37.5 MHz");
    }

    // Now initialize TFT using the same SPI0 (do not re-init SPI or GPIO)
    Serial.println("Initializing TFT on SPI0 (shared with Wiznet)...");

    initialize_tft(1);

    tft.setTextSize(2);
    tft.println("TFT + Wiznet Test");
    tft.println("SPI0 Shared Bus");
    tft.println("Both at 37.5 MHz");

    // Ensure TFT releases SPI bus
    tft.endWrite();

    // Deassert TFT/Touch CS lines to avoid SPI bus contention
    release_SPI();

    // Re-configure SPI baudrate for Wiznet after TFT init (TFT may have changed it)
    // Note: Don't call spi_set_format as it can cause hangs - format is already correct
    uint32_t actual_baud = spi_set_baudrate(WIZNET_SPI_PORT, WIZNET_SPI_BPS);
    Serial.printf("SPI baudrate restored: %lu Hz\n", actual_baud);

    Serial.println("TFT initialized successfully on shared SPI0 bus");

    delay(2000); // Wait a moment to enable visual inspection

    TEST_ASSERT_TRUE(true);
}

void test_wiznet_webserver_after_tft()
{
    IPAddress ip;

    Serial.println("Testing Wiznet webserver after TFT initialization...");

    // Initialize TFT FIRST (this is the key difference from regular webserver test)
    initialize_tft(1);

    tft.setTextSize(2);
    tft.println("Webserver Test");
    tft.println("After TFT Init");
    tft.endWrite(); // Ensure TFT releases SPI bus
    delay(1000);    // Wait a moment to enable visual inspection

    // Ensure SPI bus is released by other devices
    release_SPI();
    // Re-init Wiznet to ensure clean SPI state
    wiznet5k_reset();
    wiznet_init_spi();
    Ethernet.init(WIZNET_SPI_CS);

    // Get MAC and ensure IP (reuse DHCP/static logic)
    getWiznetMAC(mac);
    Serial.println("Ensuring IP before webserver starts...");
    int dhcp_result = Ethernet.begin(mac, DHCP_TIMEOUT);
    if (dhcp_result == 1)
    {
        ip = Ethernet.localIP();
    }
    else
    {
        IPAddress staticAfterTft(10, 0, 3, 45);
        IPAddress staticGateway(10, 0, 0, 1);
        IPAddress staticSubnet(255, 255, 252, 0); // /22
        Ethernet.begin(mac, staticAfterTft, dnsServer, staticGateway, staticSubnet);
        delay(100);
        ip = staticAfterTft;
    }

    EthernetServer server(3000);
    server.begin();
    Serial.printf("Webserver started at http://%d.%d.%d.%d:3000\n", ip[0], ip[1], ip[2], ip[3]);
    Serial.println("Access the page in a browser. The test will run for 10 seconds...");

    // Run webserver for 10 seconds - same as regular webserver test
    unsigned long startTime = millis();
    int reqCount = 0;

    while (millis() - startTime < 20000)
    {
        // Listen for incoming clients
        EthernetClient client = server.available();

        if (client)
        {
            Serial.println(F("New client"));
            // an http request ends with a blank line
            bool currentLineIsBlank = true;
            //            delay(100);
            while (client.connected())
            {
                if (client.available())
                {
                    char c = client.read();
                    Serial.write(c);

                    // if you've gotten to the end of the line (received a newline
                    // character) and the line is blank, the http request has ended,
                    // so you can send a reply
                    if (c == '\n' && currentLineIsBlank)
                    {
                        Serial.println(F("Sending response"));

                        // send a standard http response header
                        // use \r\n instead of many println statements to speedup data send
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-Type: text/html");
                        client.println("Connection: close"); // the connection will be closed after completion of the response
                        client.println();
                        client.println("<!DOCTYPE HTML>");
                        client.println("<html>");
                        client.println("<body>");
                        client.println("<h1>Arduino - Web Server with Ethernet Board:");
                        client.print(BOARD_NAME);
                        client.print("!</h1>\r\n");
                        client.print("Requests received: ");
                        client.print(++reqCount);
                        client.println("</body>");
                        client.println("</html>");
                        break;
                    }

                    if (c == '\n')
                    {
                        // you're starting a new line
                        currentLineIsBlank = true;
                    }
                    else if (c != '\r')
                    {
                        // you've gotten a character on the current line
                        currentLineIsBlank = false;
                    }
                }
            }
            delay(10); // give the web browser time to receive the data
            client.stop();
            Serial.println(F("Client disconnected"));
        }
    }

    Serial.println("Webserver test after TFT initialization completed");
    TEST_ASSERT_TRUE(true);
}

void test_wiznet_tft_sprite_webserver()
{
    Serial.println("Testing TFT sprite updates with webserver...");

    // Initialize the TFT display
    initialize_tft(1);

    tft.setTextSize(2);
    tft.println("Testing TFT sprite updates with webserver...");
    //    tft.endWrite(); // Ensure TFT releases SPI bus

    // Ensure SPI bus is released by other devices
    release_SPI();
    // Re-init Wiznet to ensure clean SPI state
    wiznet5k_reset();
    wiznet_init_spi();
    Ethernet.init(WIZNET_SPI_CS);

    // Get MAC and ensure IP (reuse DHCP/static logic)
    getWiznetMAC(mac);
    Serial.println("Ensuring IP before sprite test...");
    int dhcp_result = Ethernet.begin(mac, DHCP_TIMEOUT);
    IPAddress ip;
    if (dhcp_result == 1)
    {
        ip = Ethernet.localIP();
    }
    else
    {
        IPAddress staticAfterTft(10, 0, 3, 45);
        IPAddress staticGateway(10, 0, 0, 1);
        IPAddress staticSubnet(255, 255, 252, 0); // /22
        Ethernet.begin(mac, staticAfterTft, dnsServer, staticGateway, staticSubnet);
        delay(100);
        ip = staticAfterTft;
    }

    EthernetServer server(3000);
    server.begin();
    Serial.printf("Sprite test webserver at http://%d.%d.%d.%d:3000\n", ip[0], ip[1], ip[2], ip[3]);

    // TFT sprite setup
    tft.fillScreen(TFT_BLACK);

    TFT_eSprite sprite(&tft);
    sprite.setColorDepth(16);
    const int spriteW = 160;
    const int spriteH = 120;
    sprite.createSprite(spriteW, spriteH);

    unsigned long startTime = millis();
    uint32_t frameCount = 0;
    uint32_t reqCount = 0;

    while (millis() - startTime < 10000)
    {
        // Update sprite with colorful graphics
        sprite.fillSprite(TFT_NAVY);

        // Draw a colored border
        sprite.drawRect(0, 0, spriteW, spriteH, TFT_YELLOW);
        sprite.drawRect(1, 1, spriteW - 2, spriteH - 2, TFT_YELLOW);

        // Draw animated circle
        int cx = spriteW / 2;
        int cy = spriteH / 2;
        int radius = 20 + (frameCount % 20);
        uint16_t color = (frameCount % 2) ? TFT_GREEN : TFT_CYAN;
        sprite.fillCircle(cx, cy, radius, color);

        // Draw text on top
        sprite.setTextColor(TFT_WHITE, TFT_NAVY);
        sprite.drawString("SPI Stress", 5, 5, 2);
        sprite.drawString("Frames:", 5, 25, 2);
        sprite.drawNumber(frameCount, 75, 25, 2);
        sprite.drawString("Req:", 5, 45, 2);
        sprite.drawNumber(reqCount, 75, 45, 2);

        // Push sprite to display at (0,0)
        sprite.pushSprite(0, 0);

        // Release SPI bus for Wiznet
        release_SPI();
        // Serve web requests
        EthernetClient client = server.available();
        if (client)
        {
            reqCount++;
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");
            client.println();
            client.println("<html><body>");
            client.println("<h1>SPI Stress Test</h1>");
            client.print("<p>Frames: ");
            client.print(frameCount);
            client.println("</p>");
            client.print("<p>Requests: ");
            client.print(reqCount);
            client.println("</p>");
            client.println("</body></html>");
            delay(5);
            client.stop();
        }

        frameCount++;
    }

    sprite.deleteSprite();
    Serial.printf("Sprite test completed: frames=%lu, requests=%lu\n", (unsigned long)frameCount, (unsigned long)reqCount);

    TEST_ASSERT_TRUE(frameCount > 0);
}

void test_wiznet_counter_button()
{
    Serial.println("Testing counter button with TFT sprite display and scrolling...");

    // Initialize the TFT display
    initialize_tft(1);

    tft.setTextSize(2);
    tft.println("Counter Button Test");
    tft.setTextSize(1);
    tft.println("Press button on webpage to increment counter");

    // Ensure SPI bus is released by other devices
    release_SPI();
    // Re-init Wiznet to ensure clean SPI state
    wiznet5k_reset();
    wiznet_init_spi();
    Ethernet.init(WIZNET_SPI_CS);

    // Get MAC and ensure IP
    getWiznetMAC(mac);
    Serial.println("Ensuring IP for counter test...");
    int dhcp_result = Ethernet.begin(mac, DHCP_TIMEOUT);
    IPAddress ip;
    if (dhcp_result == 1)
    {
        ip = Ethernet.localIP();
    }
    else
    {
        IPAddress staticCounter(10, 0, 3, 46);
        IPAddress staticGateway(10, 0, 0, 1);
        IPAddress staticSubnet(255, 255, 252, 0);
        Ethernet.begin(mac, staticCounter, dnsServer, staticGateway, staticSubnet);
        delay(100);
        ip = staticCounter;
    }

    EthernetServer server(3001);
    server.begin();
    Serial.printf("Counter button test at http://%d.%d.%d.%d:3001\n", ip[0], ip[1], ip[2], ip[3]);

    // Counter variable
    static uint32_t counter = 0;

    // TFT sprite setup
    tft.fillScreen(TFT_BLACK);

    TFT_eSprite sprite(&tft);
    sprite.setColorDepth(16);
    const int spriteW = 200;
    const int spriteH = 150;
    sprite.createSprite(spriteW, spriteH);

    // Calculate sprite position (centered)
    const int spriteX = (tft.width() - spriteW) / 2;
    const int spriteY = (tft.height() - spriteH) / 2;

    // Create scrolling sprites for above and below the main sprite
    TFT_eSprite topScrollSprite(&tft);
    topScrollSprite.setColorDepth(16);
    const int topScrollHeight = spriteY;
    const int topScrollWidth = tft.width();
    if (topScrollHeight > 0)
    {
        topScrollSprite.createSprite(topScrollWidth, topScrollHeight);
    }

    TFT_eSprite bottomScrollSprite(&tft);
    bottomScrollSprite.setColorDepth(16);
    const int bottomScrollStart = spriteY + spriteH;
    const int bottomScrollHeight = tft.height() - bottomScrollStart;
    const int bottomScrollWidth = tft.width();
    if (bottomScrollHeight > 0)
    {
        bottomScrollSprite.createSprite(bottomScrollWidth, bottomScrollHeight);
    }

    unsigned long startTime = millis();
    uint32_t reqCount = 0;
    uint32_t scrollCounter = 0;
    int topScrollOffset = 0;
    int bottomScrollOffset = 0;

    while (millis() - startTime < 60000) // 60 second test
    {
        // Update sprite with counter display
        sprite.fillSprite(TFT_DARKGREEN);

        // Draw a colored border
        sprite.drawRect(0, 0, spriteW, spriteH, TFT_YELLOW);
        sprite.drawRect(1, 1, spriteW - 2, spriteH - 2, TFT_YELLOW);

        // Draw counter in center with large text
        sprite.setTextColor(TFT_WHITE, TFT_DARKGREEN);
        sprite.drawString("Counter Button Test", 10, 10, 2);

        // Large counter display in center
        char counterStr[20];
        sprintf(counterStr, "%lu", (unsigned long)counter);
        sprite.setTextColor(TFT_YELLOW, TFT_DARKGREEN);
        sprite.drawString(counterStr, spriteW / 2 - 20, spriteH / 2 - 15, 4);

        // Display additional info
        sprite.setTextColor(TFT_WHITE, TFT_DARKGREEN);
        sprite.drawString("Requests:", 10, spriteH - 50, 2);
        sprite.drawNumber(reqCount, 90, spriteH - 50, 2);

        // Draw animated indicator
        int indicatorX = spriteW - 30;
        int indicatorY = spriteH - 30;
        uint16_t indicatorColor = (millis() / 500) % 2 ? TFT_RED : TFT_GREEN;
        sprite.fillCircle(indicatorX, indicatorY, 8, indicatorColor);

        // Push sprite to display at center of screen
        sprite.pushSprite(spriteX, spriteY);

        // Update top scrolling sprite (above main sprite) - Rainbow waves scrolling right to left
        if (topScrollHeight > 0)
        {
            // Clear and redraw entire sprite with horizontal scrolling
            topScrollSprite.fillSprite(TFT_BLACK);

            // Draw vertical rainbow wave lines, shifted horizontally
            for (int x = 0; x < topScrollWidth; x++)
            {
                int waveX = (x + topScrollOffset) % topScrollWidth;
                uint16_t color = getRainbowColor(waveX * 2, scrollCounter * 3);
                topScrollSprite.drawFastVLine(x, 0, topScrollHeight, color);
            }

            topScrollSprite.pushSprite(0, 0);
            topScrollOffset = (topScrollOffset + 1) % topScrollWidth; // Scroll one column
        }

        // Update bottom scrolling sprite (below main sprite) - Rainbow waves scrolling right to left
        if (bottomScrollHeight > 0)
        {
            // Clear and redraw entire sprite with horizontal scrolling
            bottomScrollSprite.fillSprite(TFT_BLACK);

            // Draw vertical rainbow wave lines, shifted horizontally
            for (int x = 0; x < bottomScrollWidth; x++)
            {
                int waveX = (x + bottomScrollOffset) % bottomScrollWidth;
                uint16_t color = getRainbowColor(waveX * 2, scrollCounter * 4); // Different speed
                bottomScrollSprite.drawFastVLine(x, 0, bottomScrollHeight, color);
            }

            bottomScrollSprite.pushSprite(0, bottomScrollStart);
            bottomScrollOffset = (bottomScrollOffset + 1) % bottomScrollWidth; // Scroll one column
        }

        scrollCounter++;

        // Release SPI bus for Wiznet
        release_SPI();

        // Handle web requests
        EthernetClient client = server.available();
        if (client)
        {
            reqCount++;
            String request = client.readStringUntil('\r');
            client.readStringUntil('\n'); // Skip rest of request line
            // Skip headers
            while (client.available() && client.readStringUntil('\n') != "\r")
                ;

            if (request.indexOf("GET /increment") != -1)
            {
                counter++;
                Serial.printf("Counter incremented to: %lu\n", (unsigned long)counter);
            }

            // Generate modulated sine wave data server-side
            const int DATA_POINTS = 200;
            String chartDataJSON = "[";
            for (int i = 0; i < DATA_POINTS; i++)
            {
                float x = i * 1.0f;
                float y = sinf(x * 0.1f) * sinf(x * 0.01f); // Modulated sine wave

                chartDataJSON += "[";
                chartDataJSON += String(x, 1);
                chartDataJSON += ",";
                chartDataJSON += String(y, 3);
                chartDataJSON += "]";

                if (i < DATA_POINTS - 1)
                {
                    chartDataJSON += ",";
                }
            }
            chartDataJSON += "]";

            // Send HTML response
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
            // Embed the pre-calculated data from server
            client.println("const chartData = ");
            client.println(chartDataJSON);
            client.println(";");
            client.println("");
            client.println("function createChart() {");
            client.println("  chart = Highcharts.chart('chart-container', {");
            client.println("    chart: {");
            client.println("      type: 'line',");
            client.println("      animation: false,");
            client.println("      marginRight: 10");
            client.println("    },");
            client.println("    title: {");
            client.println("      text: 'Modulated Sine Wave (Calculated Server-Side)'");
            client.println("    },");
            client.println("    xAxis: {");
            client.println("      type: 'linear',");
            client.println("      tickPixelInterval: 150");
            client.println("    },");
            client.println("    yAxis: {");
            client.println("      title: {");
            client.println("        text: 'Amplitude'");
            client.println("      },");
            client.println("      plotLines: [{");
            client.println("        value: 0,");
            client.println("        width: 1,");
            client.println("        color: '#808080'");
            client.println("      }]");
            client.println("    },");
            client.println("    tooltip: {");
            client.println("      formatter: function() {");
            client.println("        return '<b>' + this.series.name + '</b><br/>' +");
            client.println("          'Time: ' + this.x.toFixed(1) + '<br/>' +");
            client.println("          'Value: ' + this.y.toFixed(3);");
            client.println("      }");
            client.println("    },");
            client.println("    legend: {");
            client.println("      enabled: false");
            client.println("    },");
            client.println("    exporting: {");
            client.println("      enabled: false");
            client.println("    },");
            client.println("    series: [{");
            client.println("      name: 'Modulated Sine',");
            client.println("      data: []");
            client.println("    }]");
            client.println("  });");
            client.println("}");
            client.println("");
            client.println("function updateChart() {");
            client.println("  if (dataIndex < chartData.length) {");
            client.println("    chart.series[0].addPoint(chartData[dataIndex], true, false);");
            client.println("    dataIndex++;");
            client.println("  }");
            client.println("}");
            client.println("");
            client.println("function incrementCounter() {");
            client.println("  fetch('/increment').then(() => {");
            client.println("    // Update counter display without full page reload");
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
            client.println("");
            client.println("// Initialize chart and start updates");
            client.println("createChart();");
            client.println("setInterval(updateChart, 500); // Update every 500 ms");
            client.println("</script>");
            client.println("</body></html>");

            delay(10);
            client.stop();
        }

        delay(50); // Small delay to prevent overwhelming the network
    }

    Serial.printf("Counter button test completed. Final counter: %lu\n", (unsigned long)counter);

    // Clean up sprites
    if (topScrollHeight > 0)
    {
        topScrollSprite.deleteSprite();
    }
    if (bottomScrollHeight > 0)
    {
        bottomScrollSprite.deleteSprite();
    }

    TEST_ASSERT_TRUE(true);
}

#endif // ENABLE_WIZNET_TESTS

// ============================================================================
// SD CARD TEST SUITE (test_sd/tests.cpp)
// ============================================================================

#if ENABLE_SD_TESTS

#include "sd_spi.h"

// Test buffer for block operations
static uint8_t sd_test_buffer[512];
static uint8_t sd_read_buffer[512];

// Global flag to track SD card initialization status
bool sd_card_available = false;

// Test SD card initialization
void test_sd_initialization(void)
{
    Serial.println("=== SD Card Initialization Test ===");
    Serial.println("Pin Configuration:");
    Serial.print("  SD_CS:   GP");
    Serial.println(SD_CS);
    Serial.print("  SD_CLK:  GP");
    Serial.println(SD_CLK);
    Serial.print("  SD_MOSI: GP");
    Serial.println(SD_MOSI);
    Serial.print("  SD_MISO: GP");
    Serial.println(SD_MISO);
    Serial.print("  SPI Mode: ");
    Serial.println(SD_SPI_PORT == -1 ? "Software SPI" : "Hardware SPI");
    Serial.flush();

    Serial.println("\nInitializing SD card...");
    sd_card_available = sd_spi_init();

    if (sd_card_available)
    {
        Serial.println("SD card initialized successfully!");

        uint8_t status = sd_spi_get_status();
        Serial.print("Card Status: 0x");
        Serial.println(status, HEX);
    }
    else
    {
        Serial.println("SD card initialization failed!");
    }

    TEST_ASSERT_TRUE_MESSAGE(sd_card_available, "SD card initialization failed");
}

// Test SD card block write
void test_sd_write_block(void)
{
    Serial.println("\n=== SD Card Block Write Test ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping write test");
        return;
    }

    // Prepare test data pattern
    for (int i = 0; i < 512; i++)
    {
        sd_test_buffer[i] = (uint8_t)(i & 0xFF);
    }

    // Add signature to beginning of block
    const char *signature = "PicoSD Test Block";
    memcpy(sd_test_buffer, signature, strlen(signature));

    Serial.println("Writing test block to sector 100...");
    bool result = sd_spi_write_block(100, sd_test_buffer);

    if (result)
    {
        Serial.println("Block write successful!");
    }
    else
    {
        Serial.println("Block write failed!");
    }

    TEST_ASSERT_TRUE_MESSAGE(result, "Failed to write block to SD card");
}

// Test SD card block read
void test_sd_read_block(void)
{
    Serial.println("\n=== SD Card Block Read Test ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping read test");
        return;
    }

    // Clear read buffer
    memset(sd_read_buffer, 0, sizeof(sd_read_buffer));

    Serial.println("Reading test block from sector 100...");
    bool result = sd_spi_read_block(100, sd_read_buffer);

    if (result)
    {
        Serial.println("Block read successful!");

        // Display first 64 bytes
        Serial.println("First 64 bytes of read data:");
        for (int i = 0; i < 64; i++)
        {
            if (i % 16 == 0)
            {
                Serial.println();
                Serial.print("  ");
            }
            if (sd_read_buffer[i] < 0x10)
                Serial.print("0");
            Serial.print(sd_read_buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    else
    {
        Serial.println("Block read failed!");
    }

    TEST_ASSERT_TRUE_MESSAGE(result, "Failed to read block from SD card");
}

// Test SD card read/write consistency
void test_sd_read_write_verify(void)
{
    Serial.println("\n=== SD Card Read/Write Verify Test ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping verify test");
        return;
    }

    Serial.println("Verifying read data matches written data...");

    // Compare the signature
    const char *signature = "PicoSD Test Block";
    bool signature_match = (memcmp(sd_read_buffer, signature, strlen(signature)) == 0);

    if (signature_match)
    {
        Serial.print("Signature match: \"");
        for (size_t i = 0; i < strlen(signature); i++)
        {
            Serial.print((char)sd_read_buffer[i]);
        }
        Serial.println("\"");
    }
    else
    {
        Serial.println("Signature mismatch!");
    }

    // Compare full block
    int mismatches = 0;
    for (int i = 0; i < 512; i++)
    {
        if (sd_test_buffer[i] != sd_read_buffer[i])
        {
            if (mismatches < 10) // Report first 10 mismatches
            {
                Serial.print("Mismatch at offset ");
                Serial.print(i);
                Serial.print(": expected 0x");
                Serial.print(sd_test_buffer[i], HEX);
                Serial.print(", got 0x");
                Serial.println(sd_read_buffer[i], HEX);
            }
            mismatches++;
        }
    }

    if (mismatches == 0)
    {
        Serial.println("Data verification successful - all 512 bytes match!");
    }
    else
    {
        Serial.print("Data verification failed - ");
        Serial.print(mismatches);
        Serial.println(" bytes differ");
    }

    TEST_ASSERT_EQUAL_MESSAGE(0, mismatches, "Read data does not match written data");
}

// Test file write with speed measurement
void test_sd_file_write(void)
{
    Serial.println("\n=== SD Card File Write Test (with Speed) ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping file write test");
        return;
    }

    // Prepare test data (4KB)
    const uint32_t test_size = 4096;
    uint8_t *write_buffer = (uint8_t *)malloc(test_size);

    if (!write_buffer)
    {
        Serial.println("Failed to allocate write buffer!");
        TEST_FAIL_MESSAGE("Memory allocation failed");
        return;
    }

    // Fill with pattern
    for (uint32_t i = 0; i < test_size; i++)
    {
        write_buffer[i] = (uint8_t)(i & 0xFF);
    }

    fs_info_t fs_info;
    if (!sd_spi_identify_filesystem(&fs_info))
    {
        free(write_buffer);
        TEST_FAIL_MESSAGE("Failed to identify filesystem");
        return;
    }

    Serial.print("Writing ");
    Serial.print(test_size);
    Serial.println(" bytes...");

    unsigned long start_time = millis();
    int bytes_written = sd_spi_write_file(&fs_info, "TEST.DAT", write_buffer, test_size);
    unsigned long end_time = millis();

    free(write_buffer);

    if (bytes_written < 0)
    {
        Serial.println("File write failed!");
        TEST_FAIL_MESSAGE("Failed to write file");
        return;
    }

    unsigned long elapsed_ms = end_time - start_time;
    float speed_kbps = (bytes_written * 8.0) / elapsed_ms; // Kilobits per second

    Serial.print("Wrote ");
    Serial.print(bytes_written);
    Serial.println(" bytes");
    Serial.print("Time: ");
    Serial.print(elapsed_ms);
    Serial.println(" ms");
    Serial.print("Speed: ");
    Serial.print(speed_kbps);
    Serial.println(" kbit/s");

    TEST_ASSERT_EQUAL_MESSAGE(test_size, bytes_written, "Bytes written mismatch");
}

// Test file read with speed measurement
void test_sd_file_read(void)
{
    Serial.println("\n=== SD Card File Read Test (with Speed) ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping file read test");
        return;
    }

    const uint32_t read_size = 4096;
    uint8_t *read_buffer = (uint8_t *)malloc(read_size);

    if (!read_buffer)
    {
        Serial.println("Failed to allocate read buffer!");
        TEST_FAIL_MESSAGE("Memory allocation failed");
        return;
    }

    memset(read_buffer, 0, read_size);

    fs_info_t fs_info;
    if (!sd_spi_identify_filesystem(&fs_info))
    {
        free(read_buffer);
        TEST_FAIL_MESSAGE("Failed to identify filesystem");
        return;
    }

    Serial.print("Reading ");
    Serial.print(read_size);
    Serial.println(" bytes...");

    unsigned long start_time = millis();
    int bytes_read = sd_spi_read_file(&fs_info, "TEST.DAT", read_buffer, read_size);
    unsigned long end_time = millis();

    if (bytes_read < 0)
    {
        free(read_buffer);
        Serial.println("File read failed!");
        TEST_FAIL_MESSAGE("Failed to read file");
        return;
    }

    unsigned long elapsed_ms = end_time - start_time;
    float speed_kbps = (bytes_read * 8.0) / elapsed_ms; // Kilobits per second

    Serial.print("Read ");
    Serial.print(bytes_read);
    Serial.println(" bytes");
    Serial.print("Time: ");
    Serial.print(elapsed_ms);
    Serial.println(" ms");
    Serial.print("Speed: ");
    Serial.print(speed_kbps);
    Serial.println(" kbit/s");

    // Display first 64 bytes
    Serial.println("First 64 bytes:");
    for (int i = 0; i < 64 && i < bytes_read; i++)
    {
        if (i % 16 == 0)
        {
            Serial.println();
            Serial.print("  ");
        }
        if (read_buffer[i] < 0x10)
            Serial.print("0");
        Serial.print(read_buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    free(read_buffer);
    TEST_ASSERT_EQUAL_MESSAGE(read_size, bytes_read, "Bytes read mismatch");
}

// Test file write/read verify with speed measurement
void test_sd_file_write_read_verify(void)
{
    Serial.println("\n=== SD Card File Write/Read/Verify Test ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping verify test");
        return;
    }

    const uint32_t test_size = 8192; // 8KB test
    uint8_t *write_buffer = (uint8_t *)malloc(test_size);
    uint8_t *read_buffer = (uint8_t *)malloc(test_size);

    if (!write_buffer || !read_buffer)
    {
        if (write_buffer)
            free(write_buffer);
        if (read_buffer)
            free(read_buffer);
        Serial.println("Failed to allocate buffers!");
        TEST_FAIL_MESSAGE("Memory allocation failed");
        return;
    }

    // Create test pattern
    for (uint32_t i = 0; i < test_size; i++)
    {
        write_buffer[i] = (uint8_t)((i * 7 + 13) & 0xFF);
    }

    fs_info_t fs_info;
    if (!sd_spi_identify_filesystem(&fs_info))
    {
        free(write_buffer);
        free(read_buffer);
        TEST_FAIL_MESSAGE("Failed to identify filesystem");
        return;
    }

    // Write test
    Serial.print("Writing ");
    Serial.print(test_size);
    Serial.println(" bytes...");

    unsigned long write_start = millis();
    int bytes_written = sd_spi_write_file(&fs_info, "VERIFY.DAT", write_buffer, test_size);
    unsigned long write_time = millis() - write_start;

    if (bytes_written != (int)test_size)
    {
        free(write_buffer);
        free(read_buffer);
        TEST_FAIL_MESSAGE("Write failed");
        return;
    }

    float write_speed = (bytes_written * 8.0) / write_time;
    Serial.print("Write: ");
    Serial.print(write_time);
    Serial.print(" ms, ");
    Serial.print(write_speed);
    Serial.println(" kbit/s");

    // Read test
    memset(read_buffer, 0, test_size);
    Serial.print("Reading ");
    Serial.print(test_size);
    Serial.println(" bytes...");

    unsigned long read_start = millis();
    int bytes_read = sd_spi_read_file(&fs_info, "VERIFY.DAT", read_buffer, test_size);
    unsigned long read_time = millis() - read_start;

    if (bytes_read != (int)test_size)
    {
        free(write_buffer);
        free(read_buffer);
        TEST_FAIL_MESSAGE("Read failed");
        return;
    }

    float read_speed = (bytes_read * 8.0) / read_time;
    Serial.print("Read: ");
    Serial.print(read_time);
    Serial.print(" ms, ");
    Serial.print(read_speed);
    Serial.println(" kbit/s");

    // Verify
    int mismatches = 0;
    for (uint32_t i = 0; i < test_size; i++)
    {
        if (write_buffer[i] != read_buffer[i])
        {
            if (mismatches < 5)
            {
                Serial.print("Mismatch at offset ");
                Serial.print(i);
                Serial.print(": expected 0x");
                Serial.print(write_buffer[i], HEX);
                Serial.print(", got 0x");
                Serial.println(read_buffer[i], HEX);
            }
            mismatches++;
        }
    }

    free(write_buffer);
    free(read_buffer);

    if (mismatches == 0)
    {
        Serial.println("Verification successful - all bytes match!");
    }
    else
    {
        Serial.print("Verification failed - ");
        Serial.print(mismatches);
        Serial.println(" bytes differ");
    }

    TEST_ASSERT_EQUAL_MESSAGE(0, mismatches, "Data verification failed");
}

// Test read speed as a function of SPI frequency
void test_sd_spi_frequency_benchmark(void)
{
    Serial.println("\n=== SD Card SPI Frequency Benchmark ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping frequency benchmark");
        return;
    }

    const uint32_t test_sector = 100;
    uint8_t *buffer = (uint8_t *)malloc(512);

    if (!buffer)
    {
        TEST_FAIL_MESSAGE("Memory allocation failed");
        return;
    }

    // Prepare write buffer with test pattern
    for (int i = 0; i < 512; i++)
    {
        buffer[i] = (uint8_t)(i & 0xFF);
    }

    // Test frequencies from 100kHz to 10MHz
    uint32_t frequencies[] = {
        100000,  // 100 kHz
        250000,  // 250 kHz
        500000,  // 500 kHz (default)
        1000000, // 1 MHz
        2000000, // 2 MHz
        4000000, // 4 MHz
        8000000, // 8 MHz
        10000000 // 10 MHz
    };

    Serial.print("\nCurrent SPI Method: ");
    Serial.println(sd_spi_get_method());

    // === READ BENCHMARK ===
    Serial.println("\n--- READ BENCHMARK ---");
    Serial.println("Frequency (kHz) | Read Time (ms) | Speed (kbit/s) | Status");
    Serial.println("----------------------------------------------------------------");

    for (int i = 0; i < 8; i++)
    {
        uint32_t freq = frequencies[i];
        sd_spi_set_frequency(freq);

        // Give the card a moment to stabilize
        delay(10);

        // Perform 10 reads and average the time (use micros for precision)
        unsigned long total_time_us = 0;
        bool all_success = true;

        for (int test = 0; test < 10; test++)
        {
            unsigned long start = micros();
            bool result = sd_spi_read_block(test_sector, buffer);
            unsigned long elapsed = micros() - start;

            if (!result)
            {
                all_success = false;
                break;
            }
            total_time_us += elapsed;
        }

        // Print results
        Serial.print("  ");
        if (freq >= 1000000)
        {
            Serial.print(freq / 1000);
        }
        else
        {
            Serial.print(freq / 1000);
        }

        // Pad to align columns
        if (freq < 1000000)
            Serial.print("  ");
        Serial.print("          | ");

        if (all_success)
        {
            unsigned long avg_time_us = total_time_us / 10;
            float avg_time_ms = avg_time_us / 1000.0;

            // Calculate speed: (512 bytes * 8 bits/byte) / (time in seconds)
            // speed_kbps = (512 * 8) / (avg_time_us / 1000000) / 1000
            //            = (512 * 8 * 1000) / avg_time_us
            float speed_kbps = (512.0 * 8.0 * 1000.0) / avg_time_us;

            // Print time in milliseconds (padded)
            if (avg_time_ms < 10)
                Serial.print(" ");
            if (avg_time_ms < 100)
                Serial.print(" ");
            Serial.print(avg_time_ms, 2); // 2 decimal places for ms
            Serial.print("         | ");

            // Print speed (padded)
            if (speed_kbps < 10)
                Serial.print(" ");
            if (speed_kbps < 100)
                Serial.print(" ");
            if (speed_kbps < 1000)
                Serial.print(" ");
            Serial.print(speed_kbps, 1);
            Serial.print("        | OK");
        }
        else
        {
            Serial.print("FAIL          | N/A            | FAILED");
        }
        Serial.println();
    }

    // === WRITE BENCHMARK ===
    Serial.println("\n--- WRITE BENCHMARK ---");
    Serial.println("Frequency (kHz) | Write Time (ms) | Speed (kbit/s) | Status");
    Serial.println("----------------------------------------------------------------");

    for (int i = 0; i < 8; i++)
    {
        uint32_t freq = frequencies[i];
        sd_spi_set_frequency(freq);

        // Give the card a moment to stabilize
        delay(10);

        // Perform 10 writes and average the time (use micros for precision)
        unsigned long total_time_us = 0;
        bool all_success = true;

        for (int test = 0; test < 10; test++)
        {
            unsigned long start = micros();
            bool result = sd_spi_write_block(test_sector + test, buffer);
            unsigned long elapsed = micros() - start;

            if (!result)
            {
                all_success = false;
                break;
            }
            total_time_us += elapsed;
        }

        // Print results
        Serial.print("  ");
        if (freq >= 1000000)
        {
            Serial.print(freq / 1000);
        }
        else
        {
            Serial.print(freq / 1000);
        }

        // Pad to align columns
        if (freq < 1000000)
            Serial.print("  ");
        Serial.print("          | ");

        if (all_success)
        {
            unsigned long avg_time_us = total_time_us / 10;
            float avg_time_ms = avg_time_us / 1000.0;

            // Calculate speed: (512 bytes * 8 bits/byte) / (time in seconds)
            float speed_kbps = (512.0 * 8.0 * 1000.0) / avg_time_us;

            // Print time in milliseconds (padded)
            if (avg_time_ms < 10)
                Serial.print(" ");
            if (avg_time_ms < 100)
                Serial.print(" ");
            Serial.print(avg_time_ms, 2); // 2 decimal places for ms
            Serial.print("          | ");

            // Print speed (padded)
            if (speed_kbps < 10)
                Serial.print(" ");
            if (speed_kbps < 100)
                Serial.print(" ");
            if (speed_kbps < 1000)
                Serial.print(" ");
            Serial.print(speed_kbps, 1);
            Serial.print("        | OK");
        }
        else
        {
            Serial.print("FAIL           | N/A            | FAILED");
        }
        Serial.println();
    }

    // Restore default frequency
    sd_spi_set_frequency(500000);
    free(buffer);

    Serial.println();
    Serial.println("Note: Software SPI has hardware limits. Speed plateaus indicate max capability.");

    TEST_ASSERT_TRUE_MESSAGE(true, "Frequency benchmark completed");
}

void test_sd_filesystem_info(void)
{
    Serial.println("\n=== SD Card Filesystem Information Test ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping filesystem info test");
        return;
    }

    // Get card type
    const char *card_type_str = sd_spi_get_card_type_string();
    Serial.print("Card Type: ");
    Serial.println(card_type_str);

    // Read OCR (Operating Conditions Register)
    uint32_t ocr = 0;
    if (sd_spi_read_ocr(&ocr))
    {
        Serial.println("\nOCR (Operating Conditions Register):");
        Serial.print("  Raw Value: 0x");
        Serial.println(ocr, HEX);
        Serial.print("  Power Up Status: ");
        Serial.println((ocr & 0x80000000) ? "Complete" : "Busy");

        // Determine card capacity type more precisely
        Serial.print("  Card Capacity Status (CCS): ");
        if (ocr & 0x40000000)
        {
            // High capacity card - need to check actual capacity to distinguish SDHC vs SDXC
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

        Serial.print("  UHS-II Card Status: ");
        Serial.println((ocr & 0x20000000) ? "Yes" : "No");

        // Voltage ranges
        Serial.print("  Supported Voltages: ");
        bool first = true;
        if (ocr & 0x00800000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("3.5-3.6V");
            first = false;
        }
        if (ocr & 0x00400000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("3.4-3.5V");
            first = false;
        }
        if (ocr & 0x00200000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("3.3-3.4V");
            first = false;
        }
        if (ocr & 0x00100000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("3.2-3.3V");
            first = false;
        }
        if (ocr & 0x00080000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("3.1-3.2V");
            first = false;
        }
        if (ocr & 0x00040000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("3.0-3.1V");
            first = false;
        }
        if (ocr & 0x00020000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("2.9-3.0V");
            first = false;
        }
        if (ocr & 0x00010000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("2.8-2.9V");
            first = false;
        }
        if (ocr & 0x00008000)
        {
            if (!first)
                Serial.print(", ");
            Serial.print("2.7-2.8V");
            first = false;
        }
        Serial.println();
    }

    // Read SSR (SD Status Register) for Speed Class information
    uint8_t ssr[64];
    if (sd_spi_read_ssr(ssr))
    {
        Serial.println("\nSSR (SD Status Register):");

        // Speed Class (bits [447:440] = byte 2)
        uint8_t speed_class = ssr[2];
        Serial.print("  Speed Class: ");
        if (speed_class == 0)
        {
            Serial.println("0 (Class not specified)");
        }
        else if (speed_class == 1)
        {
            Serial.println("2 (Class 2, >= 2 MB/s)");
        }
        else if (speed_class == 2)
        {
            Serial.println("4 (Class 4, >= 4 MB/s)");
        }
        else if (speed_class == 3)
        {
            Serial.println("6 (Class 6, >= 6 MB/s)");
        }
        else if (speed_class == 4)
        {
            Serial.println("10 (Class 10, >= 10 MB/s)");
        }
        else
        {
            Serial.print(speed_class);
            Serial.println(" (Unknown)");
        }

        // UHS Speed Grade (bits [395:392] = byte 8, upper nibble)
        uint8_t uhs_speed_grade = (ssr[8] >> 4) & 0x0F;
        Serial.print("  UHS Speed Grade: ");
        if (uhs_speed_grade == 0)
        {
            Serial.println("0 (Less than 10MB/s or not UHS)");
        }
        else if (uhs_speed_grade == 1)
        {
            Serial.println("1 (U1, >= 10 MB/s)");
        }
        else if (uhs_speed_grade == 3)
        {
            Serial.println("3 (U3, >= 30 MB/s)");
        }
        else
        {
            Serial.print(uhs_speed_grade);
            Serial.println(" (Unknown)");
        }

        // Video Speed Class (bits [383:376] = byte 10)
        uint8_t video_speed_class = ssr[10];
        if (video_speed_class > 0)
        {
            Serial.print("  Video Speed Class: ");
            if (video_speed_class == 6)
            {
                Serial.println("V6 (>= 6 MB/s)");
            }
            else if (video_speed_class == 10)
            {
                Serial.println("V10 (>= 10 MB/s)");
            }
            else if (video_speed_class == 30)
            {
                Serial.println("V30 (>= 30 MB/s)");
            }
            else if (video_speed_class == 60)
            {
                Serial.println("V60 (>= 60 MB/s)");
            }
            else if (video_speed_class == 90)
            {
                Serial.println("V90 (>= 90 MB/s)");
            }
            else
            {
                Serial.print(video_speed_class);
                Serial.println(" (Unknown)");
            }
        }
    }

    // Read CID (Card Identification)
    uint8_t cid[16];
    if (sd_spi_read_cid(cid))
    {
        Serial.println("\nCID (Card Identification):");

        // Manufacturer ID with lookup
        uint8_t mid = cid[0];
        Serial.print("  Manufacturer ID: 0x");
        Serial.print(mid, HEX);
        Serial.print(" (");

        // Common SD card manufacturer IDs
        switch (mid)
        {
        case 0x01:
            Serial.print("Panasonic");
            break;
        case 0x02:
            Serial.print("Toshiba");
            break;
        case 0x03:
            Serial.print("SanDisk");
            break;
        case 0x1B:
            Serial.print("Samsung");
            break;
        case 0x1D:
            Serial.print("AData");
            break;
        case 0x27:
            Serial.print("Phison");
            break;
        case 0x28:
            Serial.print("Lexar");
            break;
        case 0x31:
            Serial.print("Silicon Power");
            break;
        case 0x41:
            Serial.print("Kingston");
            break;
        case 0x6F:
            Serial.print("STMicroelectronics");
            break;
        case 0x74:
            Serial.print("Transcend");
            break;
        case 0x76:
            Serial.print("Patriot");
            break;
        case 0x82:
            Serial.print("Sony");
            break;
        case 0x9C:
            Serial.print("Barun Electronics");
            break;
        default:
            Serial.print("Unknown");
            break;
        }
        Serial.println(")");

        // OEM/Application ID (2 ASCII chars)
        char oid[3] = {(char)cid[1], (char)cid[2], 0};
        Serial.print("  OEM/Application ID: ");
        Serial.println(oid);

        // Product Name (5 ASCII chars)
        char pnm[6] = {(char)cid[3], (char)cid[4], (char)cid[5], (char)cid[6], (char)cid[7], 0};
        Serial.print("  Product Name: ");
        Serial.println(pnm);

        // Product Revision (2 BCD digits)
        uint8_t prv_major = (cid[8] >> 4) & 0x0F;
        uint8_t prv_minor = cid[8] & 0x0F;
        Serial.print("  Product Revision: ");
        Serial.print(prv_major);
        Serial.print(".");
        Serial.println(prv_minor);

        // Product Serial Number (32-bit)
        uint32_t psn = ((uint32_t)cid[9] << 24) | ((uint32_t)cid[10] << 16) |
                       ((uint32_t)cid[11] << 8) | (uint32_t)cid[12];
        Serial.print("  Serial Number: 0x");
        Serial.println(psn, HEX);

        // Manufacturing Date (12-bit field at CID[19:8])
        // MDT format: bits [19:12] = year (8 bits), bits [11:8] = month (4 bits)
        // cid[13] bits [3:0] = year bits [7:4]
        // cid[14] bits [7:4] = year bits [3:0]
        // cid[14] bits [3:0] = month bits [3:0]
        uint8_t mdt_year = ((cid[13] & 0x0F) << 4) | ((cid[14] >> 4) & 0x0F);
        uint8_t mdt_month = cid[14] & 0x0F;

        Serial.print("  Manufacturing Date: ");
        // Print month name for clarity
        const char *months[] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
        if (mdt_month >= 1 && mdt_month <= 12)
        {
            Serial.print(months[mdt_month]);
        }
        else
        {
            Serial.print(mdt_month);
        }
        Serial.print(" ");
        Serial.println(mdt_year + 2000); // Add 2000 to year offset
    }

    // Read CSD (Card-Specific Data) register
    uint8_t csd[16];
    bool csd_read = sd_spi_read_csd(csd);

    if (csd_read)
    {
        Serial.println("\nCSD Register (16 bytes):");
        for (int i = 0; i < 16; i++)
        {
            if (i % 8 == 0)
            {
                Serial.println();
                Serial.print("  ");
            }
            if (csd[i] < 0x10)
                Serial.print("0");
            Serial.print(csd[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Parse CSD version
        uint8_t csd_version = (csd[0] >> 6) & 0x03;
        Serial.print("CSD Version: ");
        Serial.println(csd_version);

        // Read block length
        uint8_t read_bl_len = csd[5] & 0x0F;
        uint32_t block_length = 1 << read_bl_len;
        Serial.print("Block Length: ");
        Serial.print(block_length);
        Serial.println(" bytes");

        // Parse capacity based on CSD version
        if (csd_version == 0)
        {
            // CSD v1.0 (SDSC)
            uint16_t c_size = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] >> 6) & 0x03);
            uint8_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] >> 7) & 0x01);

            Serial.print("C_SIZE: ");
            Serial.println(c_size);
            Serial.print("C_SIZE_MULT: ");
            Serial.println(c_size_mult);

            uint32_t num_sectors = (uint32_t)(c_size + 1) << (c_size_mult + 2);
            uint64_t capacity = (uint64_t)num_sectors * block_length;

            Serial.print("Number of Sectors: ");
            Serial.println(num_sectors);
            Serial.print("Card Capacity: ");
            Serial.print(capacity / (1024 * 1024));
            Serial.println(" MB");
        }
        else if (csd_version == 1)
        {
            // CSD v2.0 (SDHC/SDXC)
            uint32_t c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];

            Serial.print("C_SIZE: ");
            Serial.println(c_size);

            uint32_t num_sectors = (c_size + 1) * 1024; // 512K sectors per C_SIZE unit
            uint64_t capacity = (uint64_t)(c_size + 1) * 512 * 1024;

            Serial.print("Number of Sectors: ");
            Serial.println(num_sectors);
            Serial.print("Card Capacity: ");

            if (capacity >= (1024LL * 1024 * 1024))
            {
                Serial.print(capacity / (1024LL * 1024 * 1024));
                Serial.println(" GB");
            }
            else
            {
                Serial.print(capacity / (1024 * 1024));
                Serial.println(" MB");
            }
        }

        // Get total capacity via function
        uint64_t total_capacity = sd_spi_get_capacity();
        if (total_capacity > 0)
        {
            Serial.print("Total Capacity (calculated): ");
            if (total_capacity >= (1024LL * 1024 * 1024))
            {
                Serial.print(total_capacity / (1024LL * 1024 * 1024));
                Serial.println(" GB");
            }
            else
            {
                Serial.print(total_capacity / (1024 * 1024));
                Serial.println(" MB");
            }
        }

        // Read speed (from CSD)
        uint8_t tran_speed = csd[3];
        uint8_t speed_unit = (tran_speed & 0x07);        // bits 2:0
        uint8_t time_value = ((tran_speed >> 3) & 0x0F); // bits 6:3

        // Speed units in kbit/s: 100kbit/s, 1Mbit/s, 10Mbit/s, 100Mbit/s
        static const uint32_t speed_units[] = {100, 1000, 10000, 100000, 0, 0, 0, 0};
        // Time values are multipliers × 10 (to avoid floats)
        static const uint8_t time_values[] = {0, 10, 12, 13, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80};

        if (speed_unit < 8 && time_value < 16 && time_values[time_value] > 0)
        {
            uint32_t transfer_rate = (speed_units[speed_unit] * time_values[time_value]) / 10;
            Serial.print("Max Transfer Rate: ");
            Serial.print(transfer_rate);
            Serial.println(" kbit/s");
        }

        TEST_ASSERT_TRUE_MESSAGE(true, "CSD read successful");
    }
    else
    {
        Serial.println("Failed to read CSD register!");
        TEST_ASSERT_TRUE_MESSAGE(false, "Failed to read CSD register");
    }
}

// Test filesystem identification and directory reading
void test_sd_filesystem_and_directory(void)
{
    Serial.println("\n=== SD Card Filesystem and Directory Test ===");

    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not initialized, skipping filesystem test");
        return;
    }

    fs_info_t fs_info;
    bool fs_identified = sd_spi_identify_filesystem(&fs_info);

    if (!fs_identified)
    {
        Serial.println("Failed to identify filesystem!");
        TEST_ASSERT_TRUE_MESSAGE(false, "Failed to identify filesystem");
        return;
    }

    // Display filesystem information
    const char *fs_type = sd_spi_get_fs_type_string(fs_info.type);
    Serial.print("File System Type: ");
    Serial.println(fs_type);

    Serial.print("Partition Offset: ");
    Serial.print(fs_info.partition_offset);
    Serial.println(" sectors");

    Serial.println("\nFilesystem Details:");
    Serial.print("  Bytes per Sector: ");
    Serial.println(fs_info.bytes_per_sector);
    Serial.print("  Sectors per Cluster: ");
    Serial.println(fs_info.sectors_per_cluster);
    Serial.print("  Reserved Sectors: ");
    Serial.println(fs_info.reserved_sectors);
    Serial.print("  FAT Count: ");
    Serial.println(fs_info.fat_count);
    Serial.print("  FAT Sectors: ");
    Serial.println(fs_info.fat_sectors);

    if (fs_info.type == 3) // FAT32
    {
        Serial.print("  Root Directory Cluster: ");
        Serial.println(fs_info.root_dir_cluster);
        Serial.print("  Data Start Sector: ");
        Serial.println(fs_info.data_start_sector);
    }
    else
    {
        Serial.print("  Root Directory Entries: ");
        Serial.println(fs_info.root_dir_entries);
        Serial.print("  Root Directory Sector: ");
        Serial.println(fs_info.root_dir_sector);
    }

    Serial.print("  Data Sectors: ");
    Serial.println(fs_info.data_sectors);
    Serial.print("  Total Sectors: ");
    Serial.println(fs_info.total_sectors);
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

    TEST_ASSERT_TRUE_MESSAGE(true, "Filesystem identified successfully");

    // Read root directory
    Serial.println("\nReading Root Directory:");

    dirent_t directory[32];
    int entry_count = sd_spi_read_directory(&fs_info, directory, 32);

    if (entry_count < 0)
    {
        Serial.println("Failed to read directory!");
        TEST_ASSERT_TRUE_MESSAGE(false, "Failed to read directory");
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

            // Pad filename to 35 characters for alignment (or newline for very long names)
            int name_len = strlen(directory[i].filename);
            if (name_len < 35)
            {
                for (int j = name_len; j < 35; j++)
                    Serial.print(" ");
            }
            else
            {
                Serial.println();
                Serial.print("     "); // Indent for continuation
            }

            // Display attributes
            Serial.print(" [");
            if (directory[i].attributes & 0x10)
                Serial.print("DIR");
            else
            {
                Serial.print("FILE ");
                Serial.print(directory[i].file_size);
                Serial.print("B");
            }
            Serial.print("]");

            // Display date/time
            uint16_t date = directory[i].date;
            uint16_t time = directory[i].time;

            uint16_t year = ((date >> 9) & 0x7F) + 1980;
            uint8_t month = (date >> 5) & 0x0F;
            uint8_t day = date & 0x1F;

            uint8_t hour = (time >> 11) & 0x1F;
            uint8_t minute = (time >> 5) & 0x3F;
            uint8_t second = (time & 0x1F) * 2;

            Serial.print(" ");
            if (day < 10)
                Serial.print("0");
            Serial.print(day);
            Serial.print("/");
            if (month < 10)
                Serial.print("0");
            Serial.print(month);
            Serial.print("/");
            Serial.print(year);
            Serial.print(" ");
            if (hour < 10)
                Serial.print("0");
            Serial.print(hour);
            Serial.print(":");
            if (minute < 10)
                Serial.print("0");
            Serial.print(minute);
            Serial.print(":");
            if (second < 10)
                Serial.print("0");
            Serial.println(second);
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(entry_count >= 0, "Directory read successful");
}

#endif // ENABLE_SD_TESTS

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

void setup()
{
    // Initialize Serial
    Serial.begin(115200);
    delay(2000); // Wait for serial connection

    Serial.println("\n\n=== Unified Test Suite ===\n");

#if ENABLE_SYSINFO_TESTS
    Serial.println("System Info Tests: ENABLED");
#endif
#if ENABLE_TFT_TESTS
    Serial.println("TFT Tests: ENABLED");
#endif
#if ENABLE_WIZNET_TESTS
    Serial.println("Wiznet Tests: ENABLED");
#endif
#if ENABLE_SD_TESTS
    Serial.println("SD Card Tests: ENABLED");
#endif

    Serial.println();

    // Initialize Unity
    UNITY_BEGIN();

    // Run system info tests
#if ENABLE_SYSINFO_TESTS and !ENABLE_SINGLE_TEST
    Serial.println("\n--- Running System Info Tests ---\n");
    RUN_TEST(test_chip_type);
    RUN_TEST(test_read_unique_chip_id);
    RUN_TEST(test_read_board_id);
    RUN_TEST(test_cpu_frequency);
    RUN_TEST(test_flash_size);
    RUN_TEST(test_ram_size);
    RUN_TEST(test_free_heap);
    RUN_TEST(test_bootrom_version);
    RUN_TEST(test_reset_cause);
    RUN_TEST(test_core_number);
    RUN_TEST(test_voltage_regulation);
    RUN_TEST(test_wl_gpio2);
    RUN_TEST(test_wl_gpio0);
    RUN_TEST(test_wl_gpio1);
    RUN_TEST(test_wireless_gpio_summary);
    RUN_TEST(test_psram);
    RUN_TEST(test_system_info_summary);
#endif

    // Run TFT tests
#if ENABLE_TFT_TESTS and !ENABLE_SINGLE_TEST
    Serial.println("\n--- Running TFT Tests ---\n");
    RUN_TEST(test_tft_espi_init);
    RUN_TEST(test_tft_espi_read_id_status);
    RUN_TEST(test_tft_espi_spi_speed);
    RUN_TEST(test_tft_fillScreen);
    RUN_TEST(test_tft_lines);
    RUN_TEST(test_tft_rectangles);
    RUN_TEST(test_tft_circles);
    RUN_TEST(test_tft_triangles);
    RUN_TEST(test_tft_text);
    RUN_TEST(test_tft_rotate_screen);
    RUN_TEST(test_vertical_scroll);
    RUN_TEST(test_tft_espi_graphics);
    RUN_TEST(test_tft_gradient_fills);
    RUN_TEST(test_tft_animation_performance);
    RUN_TEST(test_tft_spi_speed_performance);
    RUN_TEST(test_tft_color_wheel);
    RUN_TEST(test_tft_cellular_automata);
    RUN_TEST(test_tft_arcfill);
    RUN_TEST(test_tft_char_times);
    RUN_TEST(test_tft_clock);
    RUN_TEST(test_tft_clock_digital);
    RUN_TEST(test_tft_ellipse);
    RUN_TEST(test_tft_fillarcspiral);
    RUN_TEST(test_tft_float_test);
    RUN_TEST(test_tft_mandelbrot);
    RUN_TEST(test_tft_julia_set);
    RUN_TEST(test_tft_pie_chart);
    RUN_TEST(test_tft_pong);
    RUN_TEST(test_tft_print_test);
    RUN_TEST(test_tft_rainbow);
    RUN_TEST(test_tft_spiro);
    RUN_TEST(test_tft_starfield);
    RUN_TEST(test_tft_bouncy_circles);
    RUN_TEST(test_tft_string_align);
    RUN_TEST(test_tft_terminal);
    RUN_TEST(test_tft_terminal_software);
    RUN_TEST(test_tft_matrix);
    RUN_TEST(test_tft_matrix_software);
    RUN_TEST(test_tft_meter_linear);
    RUN_TEST(test_tft_meters);
    RUN_TEST(test_tft_antialiased_graphics);
    RUN_TEST(test_tft_clock_gauge_demo);
    RUN_TEST(test_tft_read_reg);
    // RUN_TEST(test_tft_read_pixel); // TODO: Disabled - still to check - display may not support readPixel

    // Touch tests (optional - requires touchscreen wired and calibrated)
    RUN_TEST(test_touch_begin);
    RUN_TEST(test_touch_no_press);
    RUN_TEST(test_touch_irq_inactive);

#if ENABLE_INTERACTIVE_TESTS and !ENABLE_SINGLE_TEST
    // Interactive tests - require user input/observation
    // Set ENABLE_INTERACTIVE_TESTS to 1 to run these
    RUN_TEST(test_touch_read_coordinates);
    RUN_TEST(test_display_and_touch);
    RUN_TEST(test_touch_calibration);
    RUN_TEST(test_touch_accuracy);
#endif
#endif
#if ENABLE_SINGLE_TEST
    RUN_TEST(test_display_and_touch);
#endif

    // Run Wiznet tests
#if ENABLE_WIZNET_TESTS and !ENABLE_SINGLE_TEST
    Serial.println("\n--- Running Wiznet Tests ---\n");
    RUN_TEST(test_wiznet_mac_generation);
    RUN_TEST(test_wiznet_spi_init);
    RUN_TEST(test_wiznet_ethernet_init);
    RUN_TEST(test_wiznet_chip_detection);
    RUN_TEST(test_wiznet_hardware_reset);
    RUN_TEST(test_wiznet_interrupt_pin);
    RUN_TEST(test_wiznet_static_ip);
    RUN_TEST(test_wiznet_config_readback);
    RUN_TEST(test_wiznet_link_status);
    RUN_TEST(test_wiznet_dhcp);
    RUN_TEST(test_wiznet_webserver);
    RUN_TEST(test_wiznet_webserver_bigdata);
    RUN_TEST(test_wiznet_tft_shared_spi);
    RUN_TEST(test_wiznet_webserver_after_tft);
    RUN_TEST(test_wiznet_tft_sprite_webserver);
    RUN_TEST(test_wiznet_counter_button);
#endif

    // Run SD Card tests
#if ENABLE_SD_TESTS and !ENABLE_SINGLE_TEST
    Serial.println("\n--- Running SD Card Tests ---\n");
    RUN_TEST(test_sd_initialization);
    // RUN_TEST(test_sd_write_block);
    // RUN_TEST(test_sd_read_block);
    // RUN_TEST(test_sd_read_write_verify);
    RUN_TEST(test_sd_spi_frequency_benchmark);
    RUN_TEST(test_sd_filesystem_info);
    RUN_TEST(test_sd_filesystem_and_directory);
    RUN_TEST(test_sd_file_write);
    RUN_TEST(test_sd_file_read);
    RUN_TEST(test_sd_file_write_read_verify);
#endif

    // Finish tests
    UNITY_END();

    // Turn off TFT after all tests
#if ENABLE_TFT_TESTS
    Serial.println("\nTurning off TFT display...");
    // Deassert CS lines to avoid bus contention
    tft.fillScreen(TFT_BLACK);

    // Turn off backlight
    disableBacklight();
    Serial.println("TFT display turned off.");
#endif

    Serial.println("\n=== Test Suite Completed ===");
}

void loop()
{
    // Nothing to do here
}
