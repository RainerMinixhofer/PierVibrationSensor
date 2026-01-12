#include <Arduino.h>
#include <unity.h>
#include <SdFat.h>
#include <SdFatConfig.h>

// SD configuration (matching main.cpp)
#define SD_FAT_TYPE 3
#define SD_CS 26
#define SD_SCK 2  // GP2
#define SD_MOSI 7 // GP7
#define SD_MISO 0 // GP0
#define SD_SPI_PORT spi0
#define SD_SPI_BPS 10000000
#define SD_CONFIG SdSpiConfig(SD_CS, SHARED_SPI, SD_SCK_MHZ(10))

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

// Test setup
void setUp(void)
{
    // Initialize SPI for SD card
    spi_init(SD_SPI_PORT, SD_SPI_BPS);
    gpio_set_function(SD_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SD_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SD_MISO, GPIO_FUNC_SPI);

    // Configure CS pin
    gpio_init(SD_CS);
    gpio_set_dir(SD_CS, GPIO_OUT);
    gpio_put(SD_CS, 1);
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
    bool result = sd.begin(SD_CONFIG);
    TEST_ASSERT_TRUE_MESSAGE(result, "SD card initialization should succeed");
}

// Test file system type detection
void test_filesystem_type(void)
{
    uint8_t fsType = sd.fatType();
    TEST_ASSERT_TRUE_MESSAGE(fsType == 16 || fsType == 32 || fsType == 64,
                             "File system should be FAT16, FAT32, or exFAT");
}

// Test root directory existence
void test_root_directory(void)
{
    bool exists = sd.exists("/");
    TEST_ASSERT_TRUE_MESSAGE(exists, "Root directory should exist");
}

// Test file creation and writing
void test_file_create_write(void)
{
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
    file = sd.open("/test.txt", FILE_READ);
    TEST_ASSERT_TRUE_MESSAGE(file.isOpen(), "File should open for reading");

    String readData = file.readString();
    TEST_ASSERT_EQUAL_STRING_MESSAGE("Hello, SD Card!", readData.c_str(), "Read data should match written data");

    file.close();
}

// Test file deletion
void test_file_delete(void)
{
    bool result = sd.remove("/test.txt");
    TEST_ASSERT_TRUE_MESSAGE(result, "File should be deleted");

    bool exists = sd.exists("/test.txt");
    TEST_ASSERT_FALSE_MESSAGE(exists, "File should not exist after deletion");
}

// Test directory creation
void test_directory_create(void)
{
    bool result = sd.mkdir("/testdir");
    TEST_ASSERT_TRUE_MESSAGE(result, "Directory should be created");

    bool exists = sd.exists("/testdir");
    TEST_ASSERT_TRUE_MESSAGE(exists, "Directory should exist");
}

// Test directory removal
void test_directory_remove(void)
{
    bool result = sd.rmdir("/testdir");
    TEST_ASSERT_TRUE_MESSAGE(result, "Directory should be removed");

    bool exists = sd.exists("/testdir");
    TEST_ASSERT_FALSE_MESSAGE(exists, "Directory should not exist after removal");
}

// Test file system info
void test_filesystem_info(void)
{
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
    delay(2000); // Wait for serial
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