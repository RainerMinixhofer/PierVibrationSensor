#include <Arduino.h>
#include <unity.h>
#include <SPI.h>
#include "pin_config.h"

// pin_config.h defines SD_SPI_PORT = -1, so set SPI_DRIVER_SELECT before SdFat headers
#if SD_SPI_PORT == -1
#define SPI_DRIVER_SELECT 2
#endif

#include "SdFatConfig.h" // Must come BEFORE SdFat.h to use custom config
#include <SdFat.h>

// SD configuration (matching main.cpp)
#define SD_FAT_TYPE 3

// Try even slower speeds for software SPI reliability
#ifndef SD_SPI_BPS_INIT_TEST
#define SD_SPI_BPS_INIT_TEST 250000 // 250 kHz for initialization (slower than normal)
#endif
#ifndef SD_SPI_BPS_TEST
#define SD_SPI_BPS_TEST 400000 // 400 kHz for normal operation
#endif

// Software SPI driver using SoftSpiDriver template (same as main.cpp)
#if SD_SPI_PORT == -1
SoftSpiDriver<SD_MISO, SD_MOSI, SD_CLK> sdSoftSpi;
#define SD_CONFIG_INIT SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SPI_BPS_INIT_TEST, &sdSoftSpi)
#define SD_CONFIG_FAST SdSpiConfig(SD_CS, DEDICATED_SPI, SD_SPI_BPS_TEST, &sdSoftSpi)
#elif SD_SPI_PORT == 1
#define SD_CONFIG_INIT SdSpiConfig(SD_CS, SHARED_SPI, SD_SPI_BPS_INIT, &SPI1)
#define SD_CONFIG_FAST SdSpiConfig(SD_CS, SHARED_SPI, SD_SPI_BPS, &SPI1)
#else
#define SD_CONFIG_INIT SdSpiConfig(SD_CS, SHARED_SPI, SD_SPI_BPS_INIT)
#define SD_CONFIG_FAST SdSpiConfig(SD_CS, SHARED_SPI, SD_SPI_BPS)
#endif

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
#else
#error Invalid SD_FAT_TYPE
#endif

// Global flag to track SD card initialization status
bool sd_card_available = false;

// Test setup (called before each test)
void setUp(void)
{
    // Pin initialization is done once in setup(), not before each test
    // Just ensure CS is high before each test
    digitalWrite(SD_CS, HIGH);
}

// Test teardown
void tearDown(void)
{
    if (file.isOpen())
    {
        file.close();
    }
}

// Test SD card initialization
void test_sd_initialization(void)
{
    Serial.println("=== SD Card Initialization Test ===");
    Serial.print("SD_SPI_PORT: ");
    Serial.println(SD_SPI_PORT);
    Serial.print("SD_CLK: ");
    Serial.print(SD_CLK);
    Serial.print(", SD_MOSI: ");
    Serial.print(SD_MOSI);
    Serial.print(", SD_MISO: ");
    Serial.print(SD_MISO);
    Serial.print(", SD_CS: ");
    Serial.println(SD_CS);
    Serial.println("Calling sd.begin()...");
    Serial.flush();

    bool result = sd.begin(SD_CONFIG_INIT);
    Serial.print("sd.begin() returned: ");
    Serial.println(result ? "true" : "false");

    if (!result)
    {
        // Try to get error details
        Serial.print("Error code: ");
        Serial.println(sd.sdErrorCode(), HEX);
        Serial.print("Error data: ");
        Serial.println(sd.sdErrorData(), HEX);
    }

    if (result)
    {
        Serial.println("Switching to fast speed...");
        Serial.flush();
        result = sd.cardBegin(SD_CONFIG_FAST) && sd.volumeBegin();
        Serial.print("Fast speed switch returned: ");
        Serial.println(result ? "true" : "false");
    }
    else
    {
        Serial.println("SD card initialization failed - no card inserted or connection error");
        Serial.println("Subsequent tests will be skipped.");
    }
    Serial.flush();

    sd_card_available = result;

    if (!result)
    {
        TEST_IGNORE_MESSAGE("SD card not available - insert card and retry");
    }
    TEST_PASS();
}

// Test file system type detection
void test_filesystem_type(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    uint8_t fsType = sd.fatType();
    TEST_ASSERT_TRUE_MESSAGE(fsType == 16 || fsType == 32 || fsType == 64,
                             "File system should be FAT16, FAT32, or exFAT");
}

// Test root directory existence
void test_root_directory(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    bool exists = sd.exists("/");
    TEST_ASSERT_TRUE_MESSAGE(exists, "Root directory should exist");
}

// Test file creation and writing
void test_file_create_write(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    file = sd.open("/test.txt", FILE_WRITE);
    TEST_ASSERT_TRUE_MESSAGE(file.isOpen(), "File should open for writing");

    const char *testData = "Hello, SD Card!";
    size_t bytesWritten = file.print(testData);
    TEST_ASSERT_EQUAL_MESSAGE(strlen(testData), bytesWritten, "All bytes should be written");

    file.close();
    TEST_ASSERT_FALSE_MESSAGE(file.isOpen(), "File should be closed");
}

// Test file reading
void test_file_read(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    file = sd.open("/test.txt", FILE_READ);
    TEST_ASSERT_TRUE_MESSAGE(file.isOpen(), "File should open for reading");

    String readData = file.readString();
    TEST_ASSERT_EQUAL_STRING_MESSAGE("Hello, SD Card!", readData.c_str(), "Read data should match written data");

    file.close();
}

// Test file deletion
void test_file_delete(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    bool result = sd.remove("/test.txt");
    TEST_ASSERT_TRUE_MESSAGE(result, "File should be deleted");

    bool exists = sd.exists("/test.txt");
    TEST_ASSERT_FALSE_MESSAGE(exists, "File should not exist after deletion");
}

// Test directory creation
void test_directory_create(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    bool result = sd.mkdir("/testdir");
    TEST_ASSERT_TRUE_MESSAGE(result, "Directory should be created");

    bool exists = sd.exists("/testdir");
    TEST_ASSERT_TRUE_MESSAGE(exists, "Directory should exist");
}

// Test directory removal
void test_directory_remove(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    bool result = sd.rmdir("/testdir");
    TEST_ASSERT_TRUE_MESSAGE(result, "Directory should be removed");

    bool exists = sd.exists("/testdir");
    TEST_ASSERT_FALSE_MESSAGE(exists, "Directory should not exist after removal");
}

// Test file system info
void test_filesystem_info(void)
{
    if (!sd_card_available)
    {
        TEST_IGNORE_MESSAGE("SD card not available");
    }
    // Note: SdFs may not have totalBytes/usedBytes for all FS types
    // This test checks if the volume is accessible
    FsVolume *vol = sd.vol();
    TEST_ASSERT_NOT_NULL_MESSAGE(vol, "Volume should be accessible");

    uint8_t fatType = vol->fatType();
    TEST_ASSERT_TRUE_MESSAGE(fatType == FAT_TYPE_FAT16 || fatType == FAT_TYPE_FAT32 || fatType == FAT_TYPE_EXFAT,
                             "FAT type should be valid");
}

// Main test runner
void setup()
{
    Serial.begin(115200);
    delay(2000); // Wait for serial

    Serial.println("\n=== SD Card Test Suite ===");
    Serial.println("Initializing SD card with Software SPI...");

    // SoftSpiDriver will handle pin initialization
    // Just configure CS pin manually
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH); // CS inactive

    delay(100); // Short delay for pin stabilization

    Serial.println("Pin configuration:");
    Serial.print("  SD_CLK (GP");
    Serial.print(SD_CLK);
    Serial.println(") - Software SPI");
    Serial.print("  SD_MOSI (GP");
    Serial.print(SD_MOSI);
    Serial.println(") - Software SPI");
    Serial.print("  SD_MISO (GP");
    Serial.print(SD_MISO);
    Serial.println(") - Software SPI");
    Serial.print("  SD_CS (GP");
    Serial.print(SD_CS);
    Serial.println(") = OUTPUT, HIGH");

    Serial.println("\nSPI Configuration:");
    Serial.print("  SPI Mode: ");
#if SD_SPI_PORT == -1
    Serial.println("Software SPI (SoftSpiDriver)");
    Serial.print("  Init Speed: ");
    Serial.print(SD_SPI_BPS_INIT_TEST / 1000);
    Serial.println(" kHz");
    Serial.print("  Fast Speed: ");
    Serial.print(SD_SPI_BPS_TEST / 1000);
    Serial.println(" kHz");
#else
    Serial.println("Hardware SPI");
#endif

    Serial.println("\nNOTE: sd.begin() may take 10-30 seconds if no SD card is inserted.");
    Serial.println("Please wait - tests will continue automatically after timeout.\n");
    Serial.flush();

    UNITY_BEGIN();

    RUN_TEST(test_sd_initialization);
    RUN_TEST(test_filesystem_type);
    RUN_TEST(test_root_directory);
    RUN_TEST(test_file_create_write);
    RUN_TEST(test_file_read);
    RUN_TEST(test_file_delete);
    RUN_TEST(test_directory_create);
    RUN_TEST(test_directory_remove);
    RUN_TEST(test_filesystem_info);

    UNITY_END();
}

void loop()
{
    // Nothing to do here
}