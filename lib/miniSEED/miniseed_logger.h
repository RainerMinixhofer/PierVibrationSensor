#ifndef MINISEED_LOGGER_H
#define MINISEED_LOGGER_H

#include <stdint.h>
#include <time.h>
#include "sd_spi.h"

/**
 * @brief miniSEED Logger - Records sensor data in SEED format
 * Uses low-level SD SPI functions for reliable file I/O
 * Compatible with Raspberry Shake and seismic analysis tools (ObsPy, etc.)
 */
class MinISeedLogger
{
public:
    enum Format
    {
        FORMAT_V2 = 2,
        FORMAT_V3 = 3
    };

    /**
     * @brief Initialize miniSEED logger
     * @param fs_info Pointer to fs_info_t structure (can be nullptr, filled at first write)
     * @param station_code Station code (e.g., "PIER1")
     * @param network_code Network code (e.g., "XX")
     * @param data_dir Root directory for data storage (e.g., "/data/miniseed")
     * @return true if initialization successful
     */
    bool init(fs_info_t *fs_info_ptr, const char *station_code = "PIER1",
              const char *network_code = "XX", const char *data_dir = "/data/miniseed");

    /**
     * @brief Enable/disable recording
     * @param enable true to enable recording, false to disable
     */
    void setRecordingEnabled(bool enable);

    /**
     * @brief Set miniSEED file format.
     * @param new_format FORMAT_V2 or FORMAT_V3
     */
    void setFormat(Format new_format);

    /**
     * @brief Get configured miniSEED file format.
     */
    Format getFormat() const { return format; }

    /**
     * @brief Get configured format as string.
     */
    const char *getFormatString() const { return (format == FORMAT_V3) ? "v3" : "v2"; }

    /**
     * @brief Check if recording is enabled
     * @return true if recording is active
     */
    bool isRecordingEnabled() const { return recording_enabled; }

    /**
     * @brief Record velocity data (ADS1256 channels)
     * @param channel Channel code (e.g., "VEL_X", "VEL_Y")
     * @param sample Raw 32-bit integer sample
     * @param sample_rate Sampling rate in Hz
     * @param timestamp Unix timestamp of the sample
     * @return true if recording successful
     */
    bool recordSample(const char *channel, int32_t sample, uint16_t sample_rate, time_t timestamp);

    /**
     * @brief Record a sample with sub-second epoch precision.
     * @param channel Channel code (e.g., "VEL_X", "VEL_Y")
     * @param sample Raw 32-bit integer sample
     * @param sample_rate Sampling rate in Hz
     * @param timestamp_epoch Unix epoch seconds including fractional part
     * @return true if recording successful
     */
    bool recordSample(const char *channel, int32_t sample, uint16_t sample_rate, double timestamp_epoch);

    /**
     * @brief Batch record multiple samples (more efficient)
     * @param channel Channel code
     * @param samples Array of samples
     * @param count Number of samples
     * @param sample_rate Sampling rate in Hz
     * @param start_timestamp Unix timestamp of first sample
     * @return true if recording successful
     */
    bool recordBatch(const char *channel, const int32_t *samples, uint32_t count,
                     uint16_t sample_rate, time_t start_timestamp);

    /**
     * @brief Flush current record to disk
     * @return true if flush successful
     */
    bool flush();

    /**
     * @brief Close recording file
     * @return true if close successful
     */
    bool closeFile();

    /**
     * @brief Get current recording statistics
     * @param out_count Number of samples recorded
     * @param out_label Channel being recorded
     * @return true if valid data available
     */
    bool getStats(uint32_t &out_count, const char **out_label) const;

private:
    bool recording_enabled = false;
    Format format = FORMAT_V2;
    fs_info_t *fs_info = nullptr;
    char station_code[6];
    char network_code[3];
    char data_dir[64];
    uint32_t record_sequence = 1;

    // Per-channel buffering for efficient multi-channel recording
    static const int MAX_CHANNELS = 8;
    static const int MAX_SAMPLES_PER_CHANNEL = 2500;    // Buffer ~5 seconds at 500Hz to reduce write frequency
    static const int SAMPLES_PER_MINISEED_RECORD = 114; // Max samples per miniSEED v2 record with Blockette 1000

    struct ChannelBuffer
    {
        char channel_name[16];
        int32_t samples[MAX_SAMPLES_PER_CHANNEL];
        uint32_t sample_count;
        uint16_t sample_rate;
        double start_time_epoch;
        uint64_t samples_emitted;
        int64_t next_record_start_0001;
        uint32_t record_sequence;
        char filename[128];
        int current_day; // Track day changes for daily file rollover
        bool active;
        double next_expected_time; // Expected next sample time in epoch seconds

        ChannelBuffer() : sample_count(0), sample_rate(0), start_time_epoch(0.0), samples_emitted(0), next_record_start_0001(0), record_sequence(1), current_day(0), active(false), next_expected_time(0.0)
        {
            channel_name[0] = '\0';
            filename[0] = '\0';
        }
    };

    ChannelBuffer channel_buffers[MAX_CHANNELS];

    // Utility functions
    const char *mapChannelCode(const char *channel);
    int findChannelBuffer(const char *channel);
    int allocateChannelBuffer(const char *channel);
    bool flushChannel(int channel_idx, uint32_t max_records_to_write = 0xFFFFFFFFu);
    bool writeRecordV2(const char *filename, const int32_t *samples, uint32_t sample_count,
                       const char *channel, uint16_t sample_rate, double timestamp_epoch);
    bool writeRecordV3(const char *filename, const int32_t *samples, uint32_t sample_count,
                       const char *channel, uint16_t sample_rate, double timestamp_epoch);
    bool writeRecordToFile(const char *filename, const int32_t *samples, uint32_t count,
                           const char *channel, uint16_t sample_rate, double timestamp_epoch);
};

#endif // MINISEED_LOGGER_H
