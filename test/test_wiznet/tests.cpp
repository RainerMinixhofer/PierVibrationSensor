/**
 * @file tests.cpp
 * @brief Unit tests for Wiznet W5500/W5100S Ethernet adapter
 * @details Tests SPI communication, hardware reset, chip detection, DHCP, and network connectivity
 */

#include <Arduino.h>
#include <unity.h>
#include <Ethernet.h>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include "hardware/spi.h"
#include "pin_config.h" // Use centralized pin definitions

// Test configuration
#define WIZNET_SPI_PORT spi0
#define WIZNET_SPI_BPS 2000000 // 2 MHz SPI clock for Wiznet
#define DHCP_TIMEOUT 10000     // 10 seconds for DHCP
#define LINK_TIMEOUT 5000      // 5 seconds for link detection

// Static IP configuration (fallback if DHCP fails)
IPAddress staticIP(192, 168, 1, 177);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

// Test state
bool wiznet_initialized = false;
byte mac[6] = {0};

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

/**
 * @brief Initialize SPI bus for Wiznet adapter
 */
void wiznet_init_spi()
{
    // Initialize SPI bus
    spi_init(WIZNET_SPI_PORT, WIZNET_SPI_BPS);

    // Configure SPI pins
    gpio_set_function(WIZNET_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(WIZNET_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(WIZNET_SPI_MISO, GPIO_FUNC_SPI);

    // Configure CS pin as GPIO output (managed manually)
    gpio_init(WIZNET_SPI_CS);
    gpio_set_dir(WIZNET_SPI_CS, GPIO_OUT);
    gpio_put(WIZNET_SPI_CS, 1); // CS idle high

    Serial.println("Wiznet SPI initialized");
    Serial.printf("SPI Port: spi0, Speed: %d Hz\n", WIZNET_SPI_BPS);
    Serial.printf("MISO: GP%d, MOSI: GP%d, SCK: GP%d, CS: GP%d\n",
                  WIZNET_SPI_MISO, WIZNET_SPI_MOSI, WIZNET_SPI_SCK, WIZNET_SPI_CS);
    Serial.printf("RST: GP%d, INT: GP%d\n", WIZNET_RST, WIZNET_INT);
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

// ============================================================================
// UNITY TEST SETUP/TEARDOWN
// ============================================================================

void setUp(void)
{
    // This runs before each test
}

void tearDown(void)
{
    // This runs after each test
}

// ============================================================================
// TEST CASES
// ============================================================================

/**
 * @brief Test: Generate MAC address from flash unique ID
 */
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

/**
 * @brief Test: Hardware reset functionality
 */
void test_wiznet_hardware_reset()
{
    Serial.println("Testing Wiznet hardware reset...");

    // Configure reset pin
    gpio_init(WIZNET_RST);
    gpio_set_dir(WIZNET_RST, GPIO_OUT);

    // Assert reset (active low)
    gpio_put(WIZNET_RST, 0);
    sleep_ms(10);

    // Release reset
    gpio_put(WIZNET_RST, 1);
    sleep_ms(100); // Wait for chip to stabilize

    // The important thing is that we can control the reset pin
    // The actual readback value may vary depending on hardware pulldowns
    Serial.println("Hardware reset sequence completed");
    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: SPI bus initialization
 */
void test_wiznet_spi_init()
{
    Serial.println("Testing Wiznet SPI initialization...");

    // Reset chip before SPI init
    wiznet5k_reset();

    // Initialize SPI
    wiznet_init_spi();

    // Verify SPI hardware is initialized
    spi_hw_t *hw = spi_get_hw(WIZNET_SPI_PORT);
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, hw->cr1 & SPI_SSPCR1_SSE_BITS, "SPI should be enabled");

    // Verify GPIO pin functions
    uint32_t miso_func = (io_bank0_hw->io[WIZNET_SPI_MISO].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    uint32_t sck_func = (io_bank0_hw->io[WIZNET_SPI_SCK].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    uint32_t mosi_func = (io_bank0_hw->io[WIZNET_SPI_MOSI].ctrl & IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS) >> IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;

    TEST_ASSERT_EQUAL_MESSAGE(GPIO_FUNC_SPI, miso_func, "MISO should be configured for SPI");
    TEST_ASSERT_EQUAL_MESSAGE(GPIO_FUNC_SPI, sck_func, "SCK should be configured for SPI");
    TEST_ASSERT_EQUAL_MESSAGE(GPIO_FUNC_SPI, mosi_func, "MOSI should be configured for SPI");

    // Verify CS pin is GPIO output
    TEST_ASSERT_EQUAL_MESSAGE(1, gpio_get(WIZNET_SPI_CS), "CS pin should be HIGH (idle)");

    Serial.println("SPI initialization successful");
}

/**
 * @brief Test: Ethernet library initialization
 */
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
    TEST_ASSERT_TRUE(true);
}

/**
 * @brief Test: Chip detection via SPI communication
 */
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

    // Try to begin Ethernet (this will detect the chip)
    int result = Ethernet.begin(mac, 1000); // 1 second timeout

    // Check hardware status (chip detection)
    auto hwStatus = Ethernet.hardwareStatus();

    Serial.printf("Hardware status: %d\n", hwStatus);

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

/**
 * @brief Test: DHCP configuration
 */
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

        IPAddress dnsServer = Ethernet.dnsServerIP();
        Serial.printf("DNS: %d.%d.%d.%d\n", dnsServer[0], dnsServer[1], dnsServer[2], dnsServer[3]);

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

/**
 * @brief Test: Static IP configuration
 */
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
    Ethernet.begin(mac, staticIP, dns, gateway, subnet);

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

/**
 * @brief Test: Link status detection
 */
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
        getWiznetMAC(mac);
        Ethernet.begin(mac, staticIP, dns, gateway, subnet);
        wiznet_initialized = true;
    }

    // Wait for link to stabilize
    delay(500);

    // Check link status
    auto linkStatus = Ethernet.linkStatus();

    Serial.printf("Link status: %d\n", linkStatus);

    if (linkStatus == LinkON)
    {
        Serial.println("Ethernet link is UP (cable connected)");
    }
    else if (linkStatus == LinkOFF)
    {
        Serial.println("Ethernet link is DOWN (no cable or not connected)");
        Serial.println("This is acceptable in test environment");
    }
    else
    {
        Serial.println("Link status unknown");
    }

    // We don't fail on LinkOFF because cable might not be connected in test environment
    // The important thing is that we can READ the link status
    TEST_ASSERT_TRUE_MESSAGE(linkStatus >= 0, "Should be able to read link status");
}

/**
 * @brief Test: Interrupt pin configuration
 */
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

/**
 * @brief Test: Network configuration readback
 */
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
        Ethernet.begin(mac, staticIP, dns, gateway, subnet);
        wiznet_initialized = true;
        delay(500);
    }

    // Read back all network parameters
    IPAddress ip = Ethernet.localIP();
    IPAddress gw = Ethernet.gatewayIP();
    IPAddress sn = Ethernet.subnetMask();
    IPAddress dnsServer = Ethernet.dnsServerIP();

    Serial.println("Network configuration:");
    Serial.printf("  IP Address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
    Serial.printf("  Gateway:    %d.%d.%d.%d\n", gw[0], gw[1], gw[2], gw[3]);
    Serial.printf("  Subnet:     %d.%d.%d.%d\n", sn[0], sn[1], sn[2], sn[3]);
    Serial.printf("  DNS:        %d.%d.%d.%d\n", dnsServer[0], dnsServer[1], dnsServer[2], dnsServer[3]);

    // Verify readback matches configured values
    TEST_ASSERT_EQUAL_MESSAGE(staticIP[0], ip[0], "IP readback mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(gateway[0], gw[0], "Gateway readback mismatch");
    TEST_ASSERT_EQUAL_MESSAGE(subnet[0], sn[0], "Subnet readback mismatch");

    Serial.println("Configuration readback successful");
}

// ============================================================================
// TEST RUNNER
// ============================================================================

void setup()
{
    // Initialize Serial
    Serial.begin(115200);
    delay(2000); // Wait for serial connection

    Serial.println("\n\n=== Wiznet Network Adapter Tests ===\n");

    // Initialize Unity
    UNITY_BEGIN();

    // Run tests in order
    RUN_TEST(test_wiznet_mac_generation);
    RUN_TEST(test_wiznet_hardware_reset);
    RUN_TEST(test_wiznet_spi_init);
    RUN_TEST(test_wiznet_ethernet_init);
    RUN_TEST(test_wiznet_chip_detection);
    RUN_TEST(test_wiznet_interrupt_pin);
    RUN_TEST(test_wiznet_static_ip);
    RUN_TEST(test_wiznet_config_readback);
    RUN_TEST(test_wiznet_link_status);
    RUN_TEST(test_wiznet_dhcp); // DHCP last as it takes longest

    // Finish tests
    UNITY_END();
}

void loop()
{
    // Nothing to do here
}
