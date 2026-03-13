/**
 * @file sd_spi.h
 * @brief SD Card SPI interface for FatFs
 */

#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// SD Card commands
#define CMD0 0    // GO_IDLE_STATE
#define CMD1 1    // SEND_OP_COND (MMC)
#define CMD8 8    // SEND_IF_COND
#define CMD9 9    // SEND_CSD
#define CMD10 10  // SEND_CID
#define CMD12 12  // STOP_TRANSMISSION
#define CMD16 16  // SET_BLOCKLEN
#define CMD17 17  // READ_SINGLE_BLOCK
#define CMD18 18  // READ_MULTIPLE_BLOCK
#define CMD23 23  // SET_BLOCK_COUNT (MMC)
#define CMD24 24  // WRITE_BLOCK
#define CMD25 25  // WRITE_MULTIPLE_BLOCK
#define CMD55 55  // APP_CMD
#define CMD58 58  // READ_OCR
#define ACMD41 41 // SEND_OP_COND (SDC)

// SD Card responses
#define R1_READY_STATE 0x00
#define R1_IDLE_STATE 0x01
#define R1_ILLEGAL_COMMAND 0x04

// Data tokens
#define TOKEN_START_BLOCK 0xFE
#define TOKEN_START_MULT_BLOCK 0xFC
#define TOKEN_STOP_TRAN 0xFD

    /**
     * @brief Initialize SD card via SPI
     * @return true if successful, false otherwise
     */
    bool sd_spi_init(void);

    /**
     * @brief Read a single block from SD card
     * @param block Block number to read
     * @param buffer Buffer to store data (must be at least 512 bytes)
     * @return true if successful, false otherwise
     */
    bool sd_spi_read_block(uint32_t block, uint8_t *buffer);

    /**
     * @brief Write a single block to SD card
     * @param block Block number to write
     * @param buffer Data to write (must be 512 bytes)
     * @return true if successful, false otherwise
     */
    bool sd_spi_write_block(uint32_t block, const uint8_t *buffer);

    /**
     * @brief Get SD card status
     * @return Disk status flags
     */
    uint8_t sd_spi_get_status(void);

    /**
     * @brief Get SD card sector count
     * @return Number of sectors, or 0 on error
     */
    uint32_t sd_spi_get_sector_count(void);

    /**
     * @brief Read Card-Specific Data (CSD) register
     * @param csd Buffer to store CSD data (16 bytes)
     * @return true if successful, false otherwise
     */
    bool sd_spi_read_csd(uint8_t *csd);

    /**
     * @brief Read Card Identification (CID) register
     * @param cid Buffer to store CID data (16 bytes)
     * @return true if successful, false otherwise
     */
    bool sd_spi_read_cid(uint8_t *cid);

    /**
     * @brief Read Operating Conditions Register (OCR)
     * @param ocr Pointer to store OCR value (32-bit)
     * @return true if successful, false otherwise
     */
    bool sd_spi_read_ocr(uint32_t *ocr);

    /**
     * @brief Read SD Status Register (SSR/SD_STATUS)
     * @param ssr Buffer to store SSR data (64 bytes)
     * @return true if successful, false otherwise
     */
    bool sd_spi_read_ssr(uint8_t *ssr);

    /**
     * @brief Set software SPI frequency
     * @param freq_hz Desired frequency in Hz (max 10MHz for software SPI)
     */
    void sd_spi_set_frequency(uint32_t freq_hz);

    /**
     * @brief Get current software SPI frequency
     * @return Current frequency in Hz
     */
    uint32_t sd_spi_get_frequency(void);

    /**
     * @brief Get current SPI implementation method
     * @return String describing the active SPI method (PIO/Fast GPIO/Standard)
     */
    const char *sd_spi_get_method(void);

    /**
     * @brief Calculate card capacity in bytes
     * @return Card capacity in bytes, or 0 on error
     */
    uint64_t sd_spi_get_capacity(void);

    /**
     * @brief Get card type string
     * @return String describing card type
     */
    const char *sd_spi_get_card_type_string(void);

    /**
     * @brief Filesystem information structure
     */
    typedef struct
    {
        uint8_t type;              // 0=Unknown, 1=FAT12, 2=FAT16, 3=FAT32, 4=exFAT
        uint32_t partition_offset; // Partition start sector (0 if no partition table)
        uint32_t bytes_per_sector;
        uint32_t sectors_per_cluster;
        uint32_t reserved_sectors;
        uint32_t fat_count;
        uint32_t fat_sectors;
        uint32_t root_dir_entries;
        uint32_t root_dir_sector;   // For FAT12/16: sector number; ignored for FAT32
        uint32_t root_dir_cluster;  // For FAT32: root directory cluster number
        uint32_t data_start_sector; // First data region sector
        uint32_t data_sectors;
        uint32_t total_sectors;
        uint32_t cluster_count;
    } fs_info_t;

    /**
     * @brief Identify the filesystem type from boot sector
     * @param fs_info Pointer to filesystem info structure to fill
     * @return true if successful, false otherwise
     */
    bool sd_spi_identify_filesystem(fs_info_t *fs_info);

    /**
     * @brief Get filesystem type string
     * @param type Filesystem type (from fs_info_t)
     * @return String describing filesystem type
     */
    const char *sd_spi_get_fs_type_string(uint8_t type);

    /**
     * @brief Directory entry structure
     */
    typedef struct
    {
        char filename[256]; // Long filename support (max 255 chars + null)
        uint8_t attributes; // File attributes
        uint32_t file_size; // File size in bytes
        uint16_t date;      // Creation date
        uint16_t time;      // Creation time
    } dirent_t;

    /**
     * @brief Read root directory entries
     * @param fs_info Filesystem info from sd_spi_identify_filesystem()
     * @param entries Array to store directory entries
     * @param max_entries Maximum number of entries to read
     * @return Number of entries read, or -1 on error
     */
    int sd_spi_read_directory(const fs_info_t *fs_info, dirent_t *entries, int max_entries);

    /**
     * @brief Read directory entries for a specific path
     * @param fs_info Filesystem info from sd_spi_identify_filesystem()
     * @param path Directory path (e.g., "/data/miniseed")
     * @param entries Array to store directory entries
     * @param max_entries Maximum number of entries to read
     * @return Number of entries read, or -1 on error
     */
    int sd_spi_read_directory_path(const fs_info_t *fs_info, const char *path, dirent_t *entries, int max_entries);

    /**
     * @brief Create a directory in the filesystem
     * @param fs_info Filesystem info
     * @param path Directory path (e.g., "DIRNAME" or "DIR1/DIR2")
     * @return true if successful, false otherwise
     */
    bool sd_spi_create_directory(const fs_info_t *fs_info, const char *path);

    /**
     * @brief File write with full FAT filesystem support (overwrite/create)
     * @param fs_info Filesystem info
     * @param filepath Full file path (e.g., "DIR1/DIR2/FILE.TXT")
     * @param data Data buffer to write
     * @param size Size of data to write
     * @return Number of bytes written, or -1 on error
     */
    int sd_spi_write_file(const fs_info_t *fs_info, const char *filepath, const uint8_t *data, uint32_t size);

    /**
     * @brief Append data to an existing file (or create file if it does not exist)
     * @param fs_info Filesystem info
     * @param filepath Full file path (e.g., "DIR1/DIR2/FILE.TXT")
     * @param data Data buffer to append
     * @param size Size of data to append
     * @return Number of appended bytes, or -1 on error
     * @note WARNING: This function reads entire file into RAM then writes it back. Use sd_spi_append_file_fast() for large files.
     */
    int sd_spi_append_file(const fs_info_t *fs_info, const char *filepath, const uint8_t *data, uint32_t size);

    /**
     * @brief Efficiently append data to an existing file without reading it
     * @param fs_info Filesystem info
     * @param filepath Full file path (e.g., "DIR1/DIR2/FILE.TXT")
     * @param data Data buffer to append
     * @param size Size of data to append
     * @return Number of appended bytes, or -1 on error
     * @note This function performs true incremental append without loading the entire file into RAM.
     */
    int sd_spi_append_file_fast(const fs_info_t *fs_info, const char *filepath, const uint8_t *data, uint32_t size);

    /**
     * @brief File read with full FAT filesystem support
     * @param fs_info Filesystem info
     * @param filepath Full file path (e.g., "DIR1/DIR2/FILE.TXT")
     * @param buffer Buffer to store read data
     * @param max_size Maximum bytes to read
     * @return Number of bytes read, or -1 on error
     */
    int sd_spi_read_file(const fs_info_t *fs_info, const char *filepath, uint8_t *buffer, uint32_t max_size);

    /**
     * @brief Read a chunk of a file from a specific offset
     * @param fs_info Filesystem info
     * @param filepath Full file path (e.g., "DIR1/DIR2/FILE.TXT")
     * @param offset Byte offset into file
     * @param buffer Buffer to store read data
     * @param max_size Maximum bytes to read
     * @return Number of bytes read, 0 if offset past EOF, or -1 on error
     */
    int sd_spi_read_file_chunk(const fs_info_t *fs_info, const char *filepath, uint32_t offset,
                               uint8_t *buffer, uint32_t max_size);

    /**
     * @brief Get file size for a path
     * @param fs_info Filesystem info
     * @param filepath Full file path
     * @param size_out Output file size in bytes
     * @return true if file exists and size was obtained, false otherwise
     */
    bool sd_spi_get_file_size(const fs_info_t *fs_info, const char *filepath, uint32_t *size_out);

    /**
     * @brief Delete a file from the filesystem (with path support)
     * @param fs_info Filesystem info
     * @param filepath File path to delete (e.g., "DIR1/DIR2/FILE.TXT")
     * @return true if successful, false otherwise
     */
    bool sd_spi_delete_file(const fs_info_t *fs_info, const char *filepath);

    /**
     * @brief File handle for efficient sequential reading
     */
    typedef struct
    {
        const fs_info_t *fs_info;
        uint32_t file_size;
        uint32_t current_cluster;
        uint32_t bytes_per_cluster;
        uint32_t position; // Current read position in file
        bool is_open;
    } file_handle_t;

    /**
     * @brief Open a file for efficient streaming reads
     * @param fs_info Filesystem info
     * @param filepath Full file path
     * @param handle Output file handle
     * @return true if successful, false otherwise
     */
    bool sd_spi_open_file(const fs_info_t *fs_info, const char *filepath, file_handle_t *handle);

    /**
     * @brief Read data from an opened file (sequential, efficient)
     * @param handle File handle from sd_spi_open_file
     * @param buffer Buffer to store read data
     * @param size Number of bytes to read
     * @return Number of bytes actually read, 0 at EOF, -1 on error
     */
    int sd_spi_read_from_handle(file_handle_t *handle, uint8_t *buffer, uint32_t size);

    /**
     * @brief Close a file handle
     * @param handle File handle to close
     */
    void sd_spi_close_file(file_handle_t *handle);

    /**
     * @brief Read multiple blocks efficiently (for large file transfers)
     * @param start_block Starting block number
     * @param block_count Number of blocks to read
     * @param buffer Buffer to store data (must be block_count * 512 bytes)
     * @return true if successful, false otherwise
     */
    bool sd_spi_read_multiple_blocks(uint32_t start_block, uint32_t block_count, uint8_t *buffer);

    /**
     * @brief Check if a directory exists
     * @param fs_info Filesystem info
     * @param path Directory path to check
     * @return true if directory exists, false otherwise
     */
    bool sd_spi_directory_exists(const fs_info_t *fs_info, const char *path);

    /**
     * @brief Check if a file exists
     * @param fs_info Filesystem info
     * @param filepath File path to check
     * @return true if file exists, false otherwise
     */
    bool sd_spi_file_exists(const fs_info_t *fs_info, const char *filepath);

#ifdef __cplusplus
}
#endif

#endif // SD_SPI_H