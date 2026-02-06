#include <Arduino.h>
#include <unity.h>
#include <SPI.h>
#include "pin_config.h"
#include "sd_spi.h"

// Test buffer for block operations
static uint8_t test_buffer[512];
static uint8_t read_buffer[512];

// Global flag to track SD card initialization status
bool sd_card_available = false;

// Test setup (called before each test)
void setUp(void)
{
    // Ensure CS is high before each test
    digitalWrite(SD_CS, HIGH);
}

// Test teardown
void tearDown(void)
{
    // Nothing to clean up
}

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
        test_buffer[i] = (uint8_t)(i & 0xFF);
    }

    // Add signature to beginning of block
    const char *signature = "PicoSD Test Block";
    memcpy(test_buffer, signature, strlen(signature));

    Serial.println("Writing test block to sector 100...");
    bool result = sd_spi_write_block(100, test_buffer);

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
    memset(read_buffer, 0, sizeof(read_buffer));

    Serial.println("Reading test block from sector 100...");
    bool result = sd_spi_read_block(100, read_buffer);

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
            if (read_buffer[i] < 0x10)
                Serial.print("0");
            Serial.print(read_buffer[i], HEX);
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
    bool signature_match = (memcmp(read_buffer, signature, strlen(signature)) == 0);

    if (signature_match)
    {
        Serial.print("Signature match: \"");
        for (size_t i = 0; i < strlen(signature); i++)
        {
            Serial.print((char)read_buffer[i]);
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
        if (test_buffer[i] != read_buffer[i])
        {
            if (mismatches < 10) // Report first 10 mismatches
            {
                Serial.print("Mismatch at offset ");
                Serial.print(i);
                Serial.print(": expected 0x");
                Serial.print(test_buffer[i], HEX);
                Serial.print(", got 0x");
                Serial.println(read_buffer[i], HEX);
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
    Serial.println("\nFrequency (kHz) | Read Time (ms) | Speed (kbit/s) | Status");
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

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    delay(2000); // Wait for serial monitor

    Serial.println("\n=================================================");
    Serial.println("SD Card Test Suite - Custom SPI Implementation");
    Serial.println("Using pin definitions from pin_config.h");
    Serial.println("=================================================");

    UNITY_BEGIN();

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

    UNITY_END();

    Serial.println("\n=== Test Suite Completed ===");
}

void loop()
{
    // Tests run once in setup()
    delay(1000);
}
