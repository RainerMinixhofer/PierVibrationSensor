#include <unity.h>
#include <Arduino.h>
#include "sd_pio.h"
#include "../../include/pin_config.h"

// PIO instance and state machine
PIO pio = pio1;
uint sm = 0;

// SD card instance
SDCardPIO sd(pio, sm, SD_CS, SD_CLK, SD_MOSI, SD_MISO);

// Test buffer
uint8_t testBuffer[512];
uint8_t readBuffer[512];

void setUp(void)
{
    // This runs before each test
}

void tearDown(void)
{
    // This runs after each test
}

// Test 1: Basic initialization
void test_sd_pio_init(void)
{
    TEST_MESSAGE("Initializing SD card with PIO SPI...");

    bool success = sd.begin(10000000.0f); // 10 MHz
    TEST_ASSERT_TRUE_MESSAGE(success, "SD card initialization failed");

    uint8_t cardType = sd.getCardType();
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, cardType, "Card type is unknown");

    char msg[100];
    snprintf(msg, sizeof(msg), "Card type: 0x%02X", cardType);
    TEST_MESSAGE(msg);
}

// Test 2: Card information
void test_sd_card_info(void)
{
    TEST_MESSAGE("Reading card information...");

    // Read OCR
    uint8_t ocr[4];
    bool success = sd.readOCR(ocr);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read OCR");

    char msg[200];
    snprintf(msg, sizeof(msg), "OCR: %02X %02X %02X %02X", ocr[0], ocr[1], ocr[2], ocr[3]);
    TEST_MESSAGE(msg);

    // Read CSD
    uint8_t csd[16];
    success = sd.readCSD(csd);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read CSD");

    uint64_t size = sd.getCardSize();
    snprintf(msg, sizeof(msg), "Card size: %llu bytes (%.2f GB)",
             size, size / (1024.0 * 1024.0 * 1024.0));
    TEST_MESSAGE(msg);

    // Read CID
    uint8_t cid[16];
    success = sd.readCID(cid);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read CID");

    snprintf(msg, sizeof(msg), "CID: Manufacturer ID=0x%02X, OEM=%c%c",
             cid[0], cid[1], cid[2]);
    TEST_MESSAGE(msg);
}

// Test 3: Single block read/write
void test_sd_single_block(void)
{
    TEST_MESSAGE("Testing single block read/write...");

    // Prepare test data
    for (int i = 0; i < 512; i++)
    {
        testBuffer[i] = (uint8_t)(i & 0xFF);
    }

    // Write block
    bool success = sd.writeBlock(1000, testBuffer);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to write block");

    // Read block
    memset(readBuffer, 0, 512);
    success = sd.readBlock(1000, readBuffer);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read block");

    // Verify
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(testBuffer, readBuffer, 512, "Data mismatch");
    TEST_MESSAGE("Single block test passed");
}

// Test 4: Multiple block read/write
void test_sd_multiple_blocks(void)
{
    TEST_MESSAGE("Testing multiple block read/write...");

    const uint32_t NUM_BLOCKS = 10;
    uint8_t *writeData = new uint8_t[512 * NUM_BLOCKS];
    uint8_t *readData = new uint8_t[512 * NUM_BLOCKS];

    // Prepare test data
    for (uint32_t i = 0; i < 512 * NUM_BLOCKS; i++)
    {
        writeData[i] = (uint8_t)((i * 7) & 0xFF);
    }

    // Write blocks
    bool success = sd.writeBlocks(2000, writeData, NUM_BLOCKS);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to write multiple blocks");

    // Read blocks
    memset(readData, 0, 512 * NUM_BLOCKS);
    success = sd.readBlocks(2000, readData, NUM_BLOCKS);
    TEST_ASSERT_TRUE_MESSAGE(success, "Failed to read multiple blocks");

    // Verify
    TEST_ASSERT_EQUAL_UINT8_ARRAY_MESSAGE(writeData, readData, 512 * NUM_BLOCKS,
                                          "Multi-block data mismatch");

    delete[] writeData;
    delete[] readData;

    TEST_MESSAGE("Multiple block test passed");
}

// Test 5: Write speed benchmark
void test_sd_write_speed(void)
{
    TEST_MESSAGE("Running write speed benchmark...");

    const uint32_t NUM_BLOCKS = 1024; // 512 KB
    uint8_t *data = new uint8_t[512];

    // Prepare test pattern
    for (int i = 0; i < 512; i++)
    {
        data[i] = (uint8_t)(i & 0xFF);
    }

    uint32_t startTime = millis();

    for (uint32_t block = 0; block < NUM_BLOCKS; block++)
    {
        bool success = sd.writeBlock(10000 + block, data);
        if (!success)
        {
            char msg[100];
            snprintf(msg, sizeof(msg), "Write failed at block %lu", block);
            TEST_FAIL_MESSAGE(msg);
        }
    }

    uint32_t endTime = millis();
    uint32_t duration = endTime - startTime;

    delete[] data;

    float seconds = duration / 1000.0f;
    float kb_written = (NUM_BLOCKS * 512) / 1024.0f;
    float write_speed_kbps = kb_written / seconds;
    float write_speed_mbps = (kb_written * 8) / (seconds * 1024.0f);

    char msg[200];
    snprintf(msg, sizeof(msg), "Wrote %lu blocks (%.1f KB) in %lu ms",
             NUM_BLOCKS, kb_written, duration);
    TEST_MESSAGE(msg);
    snprintf(msg, sizeof(msg), "Write speed: %.2f KB/s (%.2f Mbit/s)",
             write_speed_kbps, write_speed_mbps);
    TEST_MESSAGE(msg);

    TEST_ASSERT_GREATER_THAN_MESSAGE(0, duration, "Duration should be positive");
}

// Test 6: Read speed benchmark
void test_sd_read_speed(void)
{
    TEST_MESSAGE("Running read speed benchmark...");

    const uint32_t NUM_BLOCKS = 1024; // 512 KB
    uint8_t *data = new uint8_t[512];

    uint32_t startTime = millis();

    for (uint32_t block = 0; block < NUM_BLOCKS; block++)
    {
        bool success = sd.readBlock(10000 + block, data);
        if (!success)
        {
            char msg[100];
            snprintf(msg, sizeof(msg), "Read failed at block %lu", block);
            TEST_FAIL_MESSAGE(msg);
        }
    }

    uint32_t endTime = millis();
    uint32_t duration = endTime - startTime;

    delete[] data;

    float seconds = duration / 1000.0f;
    float kb_read = (NUM_BLOCKS * 512) / 1024.0f;
    float read_speed_kbps = kb_read / seconds;
    float read_speed_mbps = (kb_read * 8) / (seconds * 1024.0f);

    char msg[200];
    snprintf(msg, sizeof(msg), "Read %lu blocks (%.1f KB) in %lu ms",
             NUM_BLOCKS, kb_read, duration);
    TEST_MESSAGE(msg);
    snprintf(msg, sizeof(msg), "Read speed: %.2f KB/s (%.2f Mbit/s)",
             read_speed_kbps, read_speed_mbps);
    TEST_MESSAGE(msg);

    TEST_ASSERT_GREATER_THAN_MESSAGE(0, duration, "Duration should be positive");
}

// Test 7: Multi-block write/read speed benchmark
void test_sd_multi_block_speed(void)
{
    TEST_MESSAGE("Running multi-block read/write benchmark...");

    const uint32_t NUM_TEST_BLOCKS = 100;
    const uint32_t MULTI_BLOCK_SIZE = 10;
    uint8_t *data = new uint8_t[512 * MULTI_BLOCK_SIZE];

    // Prepare test data
    for (uint32_t i = 0; i < 512 * MULTI_BLOCK_SIZE; i++)
    {
        data[i] = (uint8_t)(i & 0xFF);
    }

    // Write benchmark
    uint32_t startTime = millis();
    for (uint32_t i = 0; i < NUM_TEST_BLOCKS; i++)
    {
        sd.writeBlocks(20000 + i * MULTI_BLOCK_SIZE, data, MULTI_BLOCK_SIZE);
    }
    uint32_t writeTime = millis() - startTime;

    // Read benchmark
    startTime = millis();
    for (uint32_t i = 0; i < NUM_TEST_BLOCKS; i++)
    {
        sd.readBlocks(20000 + i * MULTI_BLOCK_SIZE, data, MULTI_BLOCK_SIZE);
    }
    uint32_t readTime = millis() - startTime;

    delete[] data;

    float total_kb = (NUM_TEST_BLOCKS * MULTI_BLOCK_SIZE * 512) / 1024.0f;

    char msg[200];
    snprintf(msg, sizeof(msg), "Multi-block write: %.2f KB/s (%.2f Mbit/s)",
             total_kb / (writeTime / 1000.0f),
             (total_kb * 8) / (writeTime / 1000.0f * 1024.0f));
    TEST_MESSAGE(msg);

    snprintf(msg, sizeof(msg), "Multi-block read: %.2f KB/s (%.2f Mbit/s)",
             total_kb / (readTime / 1000.0f),
             (total_kb * 8) / (readTime / 1000.0f * 1024.0f));
    TEST_MESSAGE(msg);
}

// Test 8: Frequency sweep benchmark
void test_sd_frequency_sweep(void)
{
    TEST_MESSAGE("Running frequency sweep benchmark...");

    const float frequencies[] = {
        1000000.0f,  // 1 MHz
        2000000.0f,  // 2 MHz
        4000000.0f,  // 4 MHz
        5000000.0f,  // 5 MHz
        8000000.0f,  // 8 MHz
        10000000.0f, // 10 MHz
        12500000.0f, // 12.5 MHz
        15000000.0f, // 15 MHz
        20000000.0f  // 20 MHz
    };

    const uint32_t NUM_BLOCKS = 100;
    uint8_t *data = new uint8_t[512];

    for (int i = 0; i < 512; i++)
    {
        data[i] = (uint8_t)(i & 0xFF);
    }

    char msg[200];
    TEST_MESSAGE("\nFrequency | Read Speed | Write Speed");
    TEST_MESSAGE("----------|------------|------------");

    for (size_t f = 0; f < sizeof(frequencies) / sizeof(frequencies[0]); f++)
    {
        // Reinitialize at new frequency
        sd.end();
        delay(100);

        if (!sd.begin(frequencies[f]))
        {
            snprintf(msg, sizeof(msg), "%.1f MHz: Initialization failed",
                     frequencies[f] / 1000000.0f);
            TEST_MESSAGE(msg);
            continue;
        }

        delay(50);

        // Write test
        uint32_t startTime = micros();
        for (uint32_t block = 0; block < NUM_BLOCKS; block++)
        {
            sd.writeBlock(30000 + block, data);
        }
        uint32_t writeTime = micros() - startTime;

        // Read test
        startTime = micros();
        for (uint32_t block = 0; block < NUM_BLOCKS; block++)
        {
            sd.readBlock(30000 + block, data);
        }
        uint32_t readTime = micros() - startTime;

        float kb = (NUM_BLOCKS * 512) / 1024.0f;
        float write_mbps = (kb * 8) / (writeTime / 1000000.0f * 1024.0f);
        float read_mbps = (kb * 8) / (readTime / 1000000.0f * 1024.0f);

        snprintf(msg, sizeof(msg), "%5.1f MHz | %7.2f Mbit/s | %7.2f Mbit/s",
                 frequencies[f] / 1000000.0f, read_mbps, write_mbps);
        TEST_MESSAGE(msg);
    }

    delete[] data;

    // Reinitialize at default speed
    sd.end();
    delay(100);
    sd.begin(10000000.0f);
}

void setup()
{
    Serial.begin(115200);
    delay(2000); // Wait for serial connection

    Serial.println("\n========================================");
    Serial.println("SD Card PIO SPI Test Suite");
    Serial.println("========================================\n");

    // Pre-initialize CS pin to ensure it's high before any PIO operations
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    delay(100);

    UNITY_BEGIN();

    RUN_TEST(test_sd_pio_init);
    RUN_TEST(test_sd_card_info);
    RUN_TEST(test_sd_single_block);
    RUN_TEST(test_sd_multiple_blocks);
    RUN_TEST(test_sd_write_speed);
    RUN_TEST(test_sd_read_speed);
    RUN_TEST(test_sd_multi_block_speed);
    RUN_TEST(test_sd_frequency_sweep);

    sd.end();

    UNITY_END();
}

void loop()
{
    // Nothing here
}
