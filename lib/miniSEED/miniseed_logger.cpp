#include "miniseed_logger.h"
#include <string.h>
#include <time.h>
#include <cstdio>
#include <cstdlib>
#include <cmath>

// Byte order conversion - use unique names to avoid system conflicts
static inline uint16_t to_big_endian_u16(uint16_t host_val)
{
    return ((host_val >> 8) & 0xFF) | ((host_val << 8) & 0xFF00);
}

static inline uint32_t to_big_endian_u32(uint32_t host_val)
{
    return ((host_val >> 24) & 0xFF) | ((host_val >> 8) & 0xFF00) | ((host_val << 8) & 0xFF0000) | ((host_val << 24) & 0xFF000000);
}

static inline int16_t to_big_endian_s16(int16_t host_val)
{
    return (int16_t)to_big_endian_u16((uint16_t)host_val);
}

static inline int32_t to_big_endian_s32(int32_t host_val)
{
    return (int32_t)to_big_endian_u32((uint32_t)host_val);
}

// Map internal channel names to SEED channel codes
const char *MinISeedLogger::mapChannelCode(const char *channel)
{
    // VEL channels: HP = High broadband Geophone (velocity sensor)
    if (strstr(channel, "VEL_X"))
        return "HPN";
    if (strstr(channel, "VEL_Y"))
        return "HPE";
    // ACC channels: HN = High broadband Accelerometer
    if (strstr(channel, "ACC_X"))
        return "HNN";
    if (strstr(channel, "ACC_Y"))
        return "HNE";
    if (strstr(channel, "ACC_Z"))
        return "HNZ";
    // GYRO channels: HJ = High broadband Rotation rate sensor (gyroscope)
    if (strstr(channel, "GYRO_X"))
        return "HJN";
    if (strstr(channel, "GYRO_Y"))
        return "HJE";
    if (strstr(channel, "GYRO_Z"))
        return "HJZ";
    return "HNZ"; // Default to Z component
}

// Initialize the logger
bool MinISeedLogger::init(fs_info_t *fs_info_ptr, const char *station_code_in, const char *network_code_in,
                          const char *data_dir_in)
{
    if (!station_code_in || !network_code_in || !data_dir_in)
        return false;

    fs_info = fs_info_ptr;

    strncpy(station_code, station_code_in, sizeof(station_code) - 1);
    station_code[sizeof(station_code) - 1] = '\0';

    strncpy(network_code, network_code_in, sizeof(network_code) - 1);
    network_code[sizeof(network_code) - 1] = '\0';

    strncpy(data_dir, data_dir_in, sizeof(data_dir) - 1);
    data_dir[sizeof(data_dir) - 1] = '\0';

    return true;
}

void MinISeedLogger::setRecordingEnabled(bool enable)
{
    if (recording_enabled && !enable)
    {
        // Ensure pending buffered data is written and logically closed immediately.
        closeFile();
    }
    recording_enabled = enable;
}

void MinISeedLogger::setFormat(Format new_format)
{
    if (new_format != FORMAT_V2 && new_format != FORMAT_V3)
        return;

    if (format != new_format)
    {
        // Finish all channel buffers with previous format before switching.
        flush();
    }

    format = new_format;
}

// Find existing channel buffer index by name
int MinISeedLogger::findChannelBuffer(const char *channel)
{
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (channel_buffers[i].active && strcmp(channel_buffers[i].channel_name, channel) == 0)
        {
            return i;
        }
    }
    return -1; // Not found
}

// Allocate a new channel buffer slot
int MinISeedLogger::allocateChannelBuffer(const char *channel)
{
    // First try to find an inactive slot
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (!channel_buffers[i].active)
        {
            channel_buffers[i].active = true;
            channel_buffers[i].sample_count = 0;
            channel_buffers[i].start_time_epoch = 0.0;
            channel_buffers[i].next_expected_time = 0.0;
            channel_buffers[i].filename[0] = '\0'; // Clear filename - will be set on first sample
            channel_buffers[i].current_day = 0;    // Clear day tracking
            strncpy(channel_buffers[i].channel_name, channel, sizeof(channel_buffers[i].channel_name) - 1);
            channel_buffers[i].channel_name[sizeof(channel_buffers[i].channel_name) - 1] = '\0';
            return i;
        }
    }

    // No free slots - flush oldest channel and reuse
    int oldest_idx = 0;
    double oldest_time = channel_buffers[0].start_time_epoch;
    for (int i = 1; i < MAX_CHANNELS; i++)
    {
        if (channel_buffers[i].start_time_epoch < oldest_time)
        {
            oldest_time = channel_buffers[i].start_time_epoch;
            oldest_idx = i;
        }
    }

    // Flush and reuse
    flushChannel(oldest_idx);
    channel_buffers[oldest_idx].active = true;
    channel_buffers[oldest_idx].sample_count = 0;
    channel_buffers[oldest_idx].start_time_epoch = 0.0;
    channel_buffers[oldest_idx].next_expected_time = 0.0;
    channel_buffers[oldest_idx].filename[0] = '\0';
    channel_buffers[oldest_idx].current_day = 0;
    strncpy(channel_buffers[oldest_idx].channel_name, channel, sizeof(channel_buffers[oldest_idx].channel_name) - 1);
    channel_buffers[oldest_idx].channel_name[sizeof(channel_buffers[oldest_idx].channel_name) - 1] = '\0';
    return oldest_idx;
}

// Flush a specific channel buffer
bool MinISeedLogger::flushChannel(int channel_idx)
{
    if (channel_idx < 0 || channel_idx >= MAX_CHANNELS)
        return false;

    ChannelBuffer &cb = channel_buffers[channel_idx];

    if (!cb.active || cb.sample_count == 0)
        return true; // Nothing to flush

    // Write multiple miniSEED records if buffer contains more than one record's worth
    // Each miniSEED v2 record can hold max 114 samples (512 bytes - 56 byte overhead)
    bool all_success = true;
    uint32_t samples_written = 0;

    // Use double precision for accurate timestamp calculation
    double timestamp_offset_seconds = 0.0;

    while (samples_written < cb.sample_count)
    {
        uint32_t samples_this_record = cb.sample_count - samples_written;
        if (samples_this_record > SAMPLES_PER_MINISEED_RECORD)
            samples_this_record = SAMPLES_PER_MINISEED_RECORD;

        // Calculate current timestamp with sub-second precision
        double current_timestamp = cb.start_time_epoch + timestamp_offset_seconds;

        bool result = writeRecordToFile(cb.filename,
                                        cb.samples + samples_written,
                                        samples_this_record,
                                        cb.channel_name,
                                        cb.sample_rate,
                                        current_timestamp);

        if (!result)
        {
            all_success = false;
            break; // Stop on first error
        }

        samples_written += samples_this_record;

        // Update timestamp offset for next record
        // Time increment = samples_this_record / sample_rate (in seconds)
        if (cb.sample_rate > 0)
            timestamp_offset_seconds += (double)samples_this_record / (double)cb.sample_rate;
    }

    // Reset buffer for next batch of samples
    // Keep filename and channel info, just reset sample count
    cb.sample_count = 0;
    // NOTE: cb.filename remains the same for continued appending to daily file

    return all_success;
}

// Write miniSEED v2 record (SEED 2.4 compatible 512-byte record)
bool MinISeedLogger::writeRecordV2(const char *filename, const int32_t *samples,
                                   uint32_t sample_count, const char *channel,
                                   uint16_t sample_rate, double start_timestamp_epoch)
{
    if (!samples || sample_count == 0 || !channel || !filename)
        return false;

    // Create miniSEED record buffer (512 bytes)
    uint8_t record[512];
    memset(record, 0, sizeof(record));

    time_t start_seconds = (time_t)floor(start_timestamp_epoch);
    double frac_seconds = start_timestamp_epoch - (double)start_seconds;
    if (frac_seconds < 0.0)
        frac_seconds = 0.0;

    uint16_t frac_0001 = (uint16_t)lround(frac_seconds * 10000.0);
    if (frac_0001 >= 10000)
    {
        start_seconds += 1;
        frac_0001 = 0;
    }

    struct tm *tm_info = gmtime(&start_seconds);
    if (!tm_info)
        return false;

    // Fill miniSEED header (48 bytes)
    // Sequence number (positions 0-5): "000001"
    snprintf((char *)record, 7, "000001");

    // Data quality (position 6): 'D' for good data
    record[6] = 'D';

    // Reserved (position 7): space
    record[7] = ' ';

    // Station code (positions 8-12): 5 chars
    memset(record + 8, ' ', 5);
    strncpy((char *)(record + 8), station_code, 5);

    // Location code (positions 13-14): 2 spaces
    record[13] = ' ';
    record[14] = ' ';

    // Channel code (positions 15-17): 3 chars, NO null terminator
    const char *ch_code = mapChannelCode(channel);
    memcpy(record + 15, ch_code, 3);

    // Network code (positions 18-19): 2 chars, NO null terminator
    memcpy(record + 18, network_code, 2);

    // Record time fields
    // Year (20-21): big-endian uint16
    uint16_t year = tm_info->tm_year + 1900;
    *(uint16_t *)(record + 20) = to_big_endian_u16(year);

    // Day of year (22-23): big-endian uint16 (1-366)
    uint16_t day_of_year = tm_info->tm_yday + 1;
    *(uint16_t *)(record + 22) = to_big_endian_u16(day_of_year);

    // Hour, minute, second (24-26)
    record[24] = tm_info->tm_hour;
    record[25] = tm_info->tm_min;
    record[26] = tm_info->tm_sec;
    record[27] = 0; // Unused

    // Fractional seconds (28-29): big-endian uint16 (in units of 0.0001 seconds)
    *(uint16_t *)(record + 28) = to_big_endian_u16(frac_0001);

    // Sample count (30-31): big-endian uint16
    // 512-byte record: 48-byte header + 8-byte blockette 1000 = 56 bytes overhead
    // Leaves 456 bytes for data = max 114 int32 samples
    uint16_t nsamp = (sample_count > 114) ? 114 : (uint16_t)sample_count;
    *(uint16_t *)(record + 30) = to_big_endian_u16(nsamp);

    // Sample rate (32-35): big-endian int16 and multiplier int16
    int16_t samprate = sample_rate;
    if (samprate > 32767)
        samprate = 32767;
    *(int16_t *)(record + 32) = to_big_endian_s16(samprate);
    *(int16_t *)(record + 34) = to_big_endian_s16(1); // Multiplier

    // Flags (36-39): all zeros for normal operation
    record[36] = 0; // Activity flags
    record[37] = 0; // I/O flags
    record[38] = 0; // Data quality flags
    record[39] = 1; // Number of blockettes (1 = Blockette 1000 present)

    // Time correction (40-43): big-endian int32, 0
    *(int32_t *)(record + 40) = 0;

    // Data offset (44-45): big-endian uint16
    // Data starts at offset 56 (after header + blockette 1000)
    *(uint16_t *)(record + 44) = to_big_endian_u16(56);

    // Blockette offset (46-47): big-endian uint16
    // Blockette 1000 starts immediately after 48-byte fixed header
    *(uint16_t *)(record + 46) = to_big_endian_u16(48);

    // Blockette 1000 (Data Only SEED Blockette) - 8 bytes at offset 48
    // This blockette specifies encoding format and byte order
    *(uint16_t *)(record + 48) = to_big_endian_u16(1000); // Blockette type
    *(uint16_t *)(record + 50) = to_big_endian_u16(0);    // Next blockette offset (0 = none)
    record[52] = 3;                                       // Encoding format: 3 = 32-bit integers
    record[53] = 1;                                       // Byte order: 1 = big-endian
    record[54] = 9;                                       // Record length: 2^9 = 512 bytes
    record[55] = 0;                                       // Reserved

    // Write sample data in big-endian format starting at offset 56 (after blockette)
    // Use nsamp (limited to 114) rather than sample_count to match header
    uint32_t data_pos = 56;
    for (uint32_t i = 0; i < nsamp; i++)
    {
        *(int32_t *)(record + data_pos) = to_big_endian_s32(samples[i]);
        data_pos += 4;
    }

    // Append the complete record so daily miniSEED files accumulate records.
    if (!fs_info)
    {
        // fs_info not yet available - cannot write
        return false;
    }

    int bytes_written = sd_spi_append_file_fast(fs_info, filename, record, sizeof(record));

    // Return false if write was incomplete - signals error to caller
    if (bytes_written != (int)sizeof(record))
    {
        return false;
    }

    return true;
}

// Write miniSEED v3-like record (simplified format used by project tests)
bool MinISeedLogger::writeRecordV3(const char *filename, const int32_t *samples,
                                   uint32_t sample_count, const char *channel,
                                   uint16_t sample_rate, double start_timestamp_epoch)
{
    if (!samples || sample_count == 0 || !channel || !filename)
        return false;

    if (!fs_info)
        return false;

    time_t start_seconds = (time_t)floor(start_timestamp_epoch);
    double frac_seconds = start_timestamp_epoch - (double)start_seconds;
    if (frac_seconds < 0.0)
        frac_seconds = 0.0;

    const uint32_t header_size = 256;
    uint32_t data_size = sample_count * 4;
    uint32_t total_size = header_size + data_size;

    uint8_t *record = (uint8_t *)malloc(total_size);
    if (!record)
        return false;

    memset(record, 0, total_size);

    // Fixed header
    record[0] = 'M';
    record[1] = 'S';
    record[2] = 3;    // Version
    record[3] = 0x01; // Flags

    uint32_t nanos = (uint32_t)lround(frac_seconds * 1000000000.0);
    if (nanos >= 1000000000u)
    {
        start_seconds += 1;
        nanos = 0;
    }

    struct tm *tm_info = gmtime(&start_seconds);
    if (!tm_info)
        return false;

    *(uint32_t *)(record + 4) = to_big_endian_u32(nanos);

    uint16_t year = tm_info->tm_year + 1900;
    *(uint16_t *)(record + 8) = to_big_endian_u16(year);

    uint16_t day_of_year = tm_info->tm_yday + 1;
    *(uint16_t *)(record + 10) = to_big_endian_u16(day_of_year);

    record[12] = tm_info->tm_hour;
    record[13] = tm_info->tm_min;
    record[14] = tm_info->tm_sec;
    record[15] = 4; // int32 samples

    float rate_f = (float)sample_rate;
    uint32_t rate_bits;
    memcpy(&rate_bits, &rate_f, sizeof(rate_bits));
    *(uint32_t *)(record + 16) = to_big_endian_u32(rate_bits);

    *(uint32_t *)(record + 20) = to_big_endian_u32(sample_count);
    *(uint32_t *)(record + 24) = 0;                            // CRC
    *(uint32_t *)(record + 28) = to_big_endian_u32(1);         // Publication version
    *(uint32_t *)(record + 32) = to_big_endian_u32(64);        // Extra header length
    *(uint32_t *)(record + 36) = to_big_endian_u32(data_size); // Data length

    const char *ch_code = mapChannelCode(channel);
    char json_header[128];
    snprintf(json_header, sizeof(json_header),
             "{\"SID\":\"%s.%s..%s\",\"FormatVersion\":3}",
             network_code, station_code, ch_code);
    memcpy(record + 40, json_header, strlen(json_header));

    uint32_t data_offset = header_size;
    for (uint32_t i = 0; i < sample_count; i++)
    {
        // v3 test implementation uses little-endian payload samples.
        *(int32_t *)(record + data_offset) = samples[i];
        data_offset += 4;
    }

    int bytes_written = sd_spi_append_file_fast(fs_info, filename, record, total_size);
    free(record);

    return bytes_written == (int)total_size;
}

bool MinISeedLogger::writeRecordToFile(const char *filename, const int32_t *samples,
                                       uint32_t sample_count, const char *channel,
                                       uint16_t sample_rate, double start_timestamp_epoch)
{
    if (format == FORMAT_V3)
    {
        return writeRecordV3(filename, samples, sample_count, channel, sample_rate, start_timestamp_epoch);
    }
    return writeRecordV2(filename, samples, sample_count, channel, sample_rate, start_timestamp_epoch);
}

// Record a single sample
bool MinISeedLogger::recordSample(const char *channel, int32_t sample,
                                  uint16_t sample_rate, time_t timestamp)
{
    if (!channel || !recording_enabled)
        return false;

    // Find or allocate channel buffer
    int idx = findChannelBuffer(channel);
    if (idx < 0)
    {
        idx = allocateChannelBuffer(channel);
    }

    ChannelBuffer &cb = channel_buffers[idx];

    struct tm *tm_info = gmtime(&timestamp);
    if (!tm_info)
        return false;

    // Calculate current day number (days since epoch)
    int current_day = tm_info->tm_year * 1000 + tm_info->tm_yday;

    // Check if we need to create a new filename (first sample OR day changed)
    if (cb.filename[0] == '\0' || cb.current_day != current_day)
    {
        // Flush any pending data from previous day
        if (cb.sample_count > 0)
        {
            flushChannel(idx);
        }

        // Create daily filename: YYYYMMDD.MSE
        // NOTE: All channels for a given day write to the same file.
        // This is standard miniSEED format - one daily file contains multiple traces (channels).
        // Each 512-byte record includes channel identification in the SEED header.
        snprintf(cb.filename, sizeof(cb.filename), "%s/%04d%02d%02d.MSE",
                 data_dir, tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

        cb.current_day = current_day;
        cb.start_time_epoch = 0.0;
        cb.next_expected_time = 0.0;
    }

    // Handle timing and continuity
    if (cb.sample_count == 0)
    {
        const double sample_period = 1.0 / (double)sample_rate;
        double incoming_time = (double)timestamp;
        const uint16_t previous_rate = cb.sample_rate;

        cb.sample_rate = sample_rate;

        // Keep a continuous timeline across flushes even when input timestamps are 1-second resolution.
        if (cb.next_expected_time > 0.0 && previous_rate == sample_rate)
        {
            cb.start_time_epoch = cb.next_expected_time;
        }
        else
        {
            cb.start_time_epoch = incoming_time;
        }

        cb.next_expected_time = cb.start_time_epoch + sample_period;
    }
    else
    {
        const double sample_period = 1.0 / (double)sample_rate;

        // If sampling rate changes mid-stream, flush and restart timing for this channel.
        if (cb.sample_rate != sample_rate)
        {
            if (!flushChannel(idx))
                return false;
            cb.sample_rate = sample_rate;
            cb.start_time_epoch = (double)timestamp;
            cb.next_expected_time = cb.start_time_epoch + sample_period;
        }
        else
        {
            cb.next_expected_time += sample_period;
        }
    }

    // Add sample to buffer
    cb.samples[cb.sample_count++] = sample;

    // Flush in smaller chunks to avoid long blocking SD write bursts.
    // High-rate channels (500 Hz) flush after ~0.9-1.1 s and are staggered by channel name
    // so multiple channels do not flush in the same loop iteration.
    uint32_t target_records = (sample_rate >= 400) ? 4u : 8u;
    if (sample_rate >= 400)
    {
        const uint32_t channel_stagger = ((uint8_t)cb.channel_name[0] + (uint8_t)cb.channel_name[1]) & 0x1u;
        target_records += channel_stagger; // 4 or 5 records for desynchronization
    }

    uint32_t auto_flush_threshold = target_records * (uint32_t)SAMPLES_PER_MINISEED_RECORD;
    if (auto_flush_threshold > (uint32_t)MAX_SAMPLES_PER_CHANNEL)
        auto_flush_threshold = (uint32_t)MAX_SAMPLES_PER_CHANNEL;

    if (cb.sample_count >= auto_flush_threshold)
    {
        if (!flushChannel(idx))
            return false;
    }

    return true;
}

// Record a batch of samples
bool MinISeedLogger::recordBatch(const char *channel, const int32_t *samples,
                                 uint32_t count, uint16_t sample_rate, time_t start_timestamp)
{
    if (!channel || count == 0 || !samples || !recording_enabled)
        return false;

    struct tm *tm_info = gmtime(&start_timestamp);
    if (!tm_info)
        return false;

    // Use daily filename format consistent with recordSample
    char filename[256];
    snprintf(filename, sizeof(filename), "%s/%04d%02d%02d.MSE",
             data_dir, tm_info->tm_year + 1900, tm_info->tm_mon + 1, tm_info->tm_mday);

    return writeRecordToFile(filename, samples, count, channel, sample_rate, start_timestamp);
}

// Flush buffered data for all channels
bool MinISeedLogger::flush()
{
    bool all_success = true;

    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (channel_buffers[i].active && channel_buffers[i].sample_count > 0)
        {
            if (!flushChannel(i))
            {
                all_success = false;
            }
        }
    }

    return all_success;
}

// Close file (flush all channels and deactivate buffers)
bool MinISeedLogger::closeFile()
{
    bool result = flush();

    // Deactivate all channel buffers
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        channel_buffers[i].active = false;
        channel_buffers[i].sample_count = 0;
        channel_buffers[i].start_time_epoch = 0.0;
        channel_buffers[i].next_expected_time = 0.0;
    }

    return result;
}

// Get recording statistics
bool MinISeedLogger::getStats(uint32_t &out_count, const char **out_label) const
{
    // Return stats for first active channel with data
    for (int i = 0; i < MAX_CHANNELS; i++)
    {
        if (channel_buffers[i].active && channel_buffers[i].sample_count > 0)
        {
            out_count = channel_buffers[i].sample_count;
            *out_label = channel_buffers[i].channel_name;
            return true;
        }
    }

    return false;
}
