/**
 * @file sd_spi.cpp
 * @brief SD Card SPI interface implementation
 *
 * Performance optimizations enabled:
 * - USE_PIO_SPI: PIO-based SPI for RP2040 (2-5x faster than bit-banging) ~10-15 Mbit/s
 * - USE_FAST_GPIO: Direct port manipulation for RP2040 (10-20x faster than digitalWrite) ~4 Mbit/s
 * - Manual loop unrolling: Eliminates loop overhead
 * - Inline bit operations: Reduces function call overhead
 *
 * Priority: PIO > Fast GPIO > Standard digitalWrite
 */

#include "sd_spi.h"
#include <Arduino.h>
#include <SPI.h>
#include "../../include/pin_config.h"

// Enable PIO-based SPI for RP2040 (highest performance)
#ifdef ARDUINO_ARCH_RP2040
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#define USE_PIO_SPI 1
#define USE_FAST_GPIO 1
#endif

// SD card type
static uint8_t card_type = 0;
static bool card_initialized = false;
static uint64_t card_capacity = 0;
static uint8_t card_csd[16] = {0};

// FAT cache and allocation hint to reduce repeated metadata I/O.
static bool fat_cache_valid = false;
static bool fat_cache_dirty = false;
static uint32_t fat_cache_sector = 0;
static uint8_t fat_cache_data[512] = {0};
static uint32_t fat_next_free_cluster_hint = 2;

// SPI timing control (delay in microseconds for half clock period)
// Default: 1us = 500kHz SPI clock
static uint32_t spi_half_period_us = 1;

// Forward declarations for FAT metadata cache helpers.
static void fat_cache_invalidate(void);
static bool fat_cache_flush(void);
static bool fat_cache_load_sector(uint32_t sector);

// PIO state machine tracking
#ifdef USE_PIO_SPI
static PIO pio_instance = nullptr;
static uint sm = 0;
static bool pio_initialized = false;
#endif

// Helper macros
#define SD_CS_LOW() digitalWrite(SD_CS, LOW)
#define SD_CS_HIGH() digitalWrite(SD_CS, HIGH)

// PIO SPI program for RP2040
#ifdef USE_PIO_SPI
// PIO assembly program for SPI transfer (MSB first, mode 0)
// Transfers 8 bits via PIO state machine
const uint16_t spi_pio_program_instructions[] = {
    //     .wrap_target
    0x6001, // 0: out    pins, 1         ; Output MOSI bit
    0x2001, // 1: wait   0 gpio, 1       ; Delay (optional, for clock shaping)
    0xe101, // 2: set    pins, 1         ; Set CLK high
    0x4001, // 3: in     pins, 1         ; Read MISO bit
    0xe100, // 4: set    pins, 0         ; Set CLK low
    //     .wrap
};

// Use SDK's pio_program_t structure
const pio_program_t spi_pio_program = {
    .instructions = spi_pio_program_instructions,
    .length = 5,
    .origin = -1};

// Initialize PIO for SPI
static bool init_pio_spi(void)
{
    if (pio_initialized)
        return true;

    // Try to claim PIO0 first, then PIO1
    pio_instance = pio0;
    if (!pio_can_add_program(pio_instance, &spi_pio_program))
    {
        pio_instance = pio1;
        if (!pio_can_add_program(pio_instance, &spi_pio_program))
        {
            Serial.println("PIO: No space available");
            return false;
        }
    }

    // Claim a state machine
    sm = pio_claim_unused_sm(pio_instance, false);
    if (sm < 0)
    {
        Serial.println("PIO: No free state machine");
        return false;
    }

    // Add program to PIO
    uint offset = pio_add_program(pio_instance, &spi_pio_program);

    // Configure state machine
    pio_sm_config c = pio_get_default_sm_config();

    // Configure OUT pins (MOSI)
    sm_config_set_out_pins(&c, SD_MOSI, 1);
    // Configure SET pins (CLK)
    sm_config_set_set_pins(&c, SD_CLK, 1);
    // Configure IN pins (MISO)
    sm_config_set_in_pins(&c, SD_MISO);

    // Shift out MSB first
    sm_config_set_out_shift(&c, false, true, 8); // shift right, autopull, 8 bits
    sm_config_set_in_shift(&c, false, true, 8);  // shift right, autopush, 8 bits

    // Set clock divider (lower = faster)
    // For max speed, use 1.0. For 10 MHz on 133 MHz clock: 133/10 = 13.3
    sm_config_set_clkdiv(&c, 1.0f); // Maximum speed

    // Set wrap
    sm_config_set_wrap(&c, offset, offset + spi_pio_program.length - 1);

    // Initialize pins
    pio_gpio_init(pio_instance, SD_MOSI);
    pio_gpio_init(pio_instance, SD_CLK);
    pio_gpio_init(pio_instance, SD_MISO);

    // Set pin directions
    pio_sm_set_consecutive_pindirs(pio_instance, sm, SD_MOSI, 1, true);  // MOSI output
    pio_sm_set_consecutive_pindirs(pio_instance, sm, SD_CLK, 1, true);   // CLK output
    pio_sm_set_consecutive_pindirs(pio_instance, sm, SD_MISO, 1, false); // MISO input

    // Initialize and enable state machine
    pio_sm_init(pio_instance, sm, offset, &c);
    pio_sm_set_enabled(pio_instance, sm, true);

    pio_initialized = true;
    Serial.println("PIO SPI initialized successfully");
    return true;
}

// PIO-based SPI transfer
static uint8_t spi_transfer_pio(uint8_t data)
{
    // Wait for TX FIFO to have space
    while (pio_sm_is_tx_fifo_full(pio_instance, sm))
        ;

    // Send byte
    pio_sm_put(pio_instance, sm, data);

    // Wait for RX FIFO to have data
    while (pio_sm_is_rx_fifo_empty(pio_instance, sm))
        ;

    // Read received byte
    return (uint8_t)pio_sm_get(pio_instance, sm);
}
#endif

// Fast GPIO macros for RP2040
#ifdef USE_FAST_GPIO
#define FAST_GPIO_SET(pin) sio_hw->gpio_set = (1u << (pin))
#define FAST_GPIO_CLR(pin) sio_hw->gpio_clr = (1u << (pin))
#define FAST_GPIO_READ(pin) ((sio_hw->gpio_in >> (pin)) & 1u)

// Maximum speed SPI transfer (no delays)
#define SPI_BIT_TRANSFER_FAST(bit_pos)                    \
    do                                                    \
    {                                                     \
        if (data & (0x80 >> bit_pos))                     \
            FAST_GPIO_SET(SD_MOSI);                       \
        else                                              \
            FAST_GPIO_CLR(SD_MOSI);                       \
        FAST_GPIO_SET(SD_CLK);                            \
        result = (result << 1) | FAST_GPIO_READ(SD_MISO); \
        FAST_GPIO_CLR(SD_CLK);                            \
    } while (0)

// Controlled speed SPI transfer (with delays)
#define SPI_BIT_TRANSFER_DELAYED(bit_pos)                 \
    do                                                    \
    {                                                     \
        if (data & (0x80 >> bit_pos))                     \
            FAST_GPIO_SET(SD_MOSI);                       \
        else                                              \
            FAST_GPIO_CLR(SD_MOSI);                       \
        delayMicroseconds(spi_half_period_us);            \
        FAST_GPIO_SET(SD_CLK);                            \
        delayMicroseconds(spi_half_period_us);            \
        result = (result << 1) | FAST_GPIO_READ(SD_MISO); \
        FAST_GPIO_CLR(SD_CLK);                            \
    } while (0)
#endif

// SPI transfer helper - optimized for RP2040
static uint8_t spi_transfer(uint8_t data)
{
#ifdef USE_PIO_SPI
    // Use PIO only for maximum speed transfers (when delay is 0)
    // PIO should already be initialized after successful SD init
    if (pio_initialized && spi_half_period_us == 0)
    {
        return spi_transfer_pio(data);
    }
#endif

#ifdef USE_FAST_GPIO
    // Choose optimized path based on delay setting
    uint8_t result = 0;

    if (spi_half_period_us == 0)
    {
        // Maximum speed - no delay overhead
        SPI_BIT_TRANSFER_FAST(0);
        SPI_BIT_TRANSFER_FAST(1);
        SPI_BIT_TRANSFER_FAST(2);
        SPI_BIT_TRANSFER_FAST(3);
        SPI_BIT_TRANSFER_FAST(4);
        SPI_BIT_TRANSFER_FAST(5);
        SPI_BIT_TRANSFER_FAST(6);
        SPI_BIT_TRANSFER_FAST(7);
    }
    else
    {
        // Controlled speed with delays
        SPI_BIT_TRANSFER_DELAYED(0);
        SPI_BIT_TRANSFER_DELAYED(1);
        SPI_BIT_TRANSFER_DELAYED(2);
        SPI_BIT_TRANSFER_DELAYED(3);
        SPI_BIT_TRANSFER_DELAYED(4);
        SPI_BIT_TRANSFER_DELAYED(5);
        SPI_BIT_TRANSFER_DELAYED(6);
        SPI_BIT_TRANSFER_DELAYED(7);
    }

    return result;
#else
    // Standard digitalWrite version (slower but portable)
    uint8_t result = 0;

    for (int i = 0; i < 8; i++)
    {
        // Set MOSI
        digitalWrite(SD_MOSI, (data & 0x80) ? HIGH : LOW);
        data <<= 1;

        // Clock high
        digitalWrite(SD_CLK, HIGH);
        if (spi_half_period_us > 0)
            delayMicroseconds(spi_half_period_us);

        // Read MISO
        result <<= 1;
        if (digitalRead(SD_MISO))
            result |= 1;

        // Clock low
        digitalWrite(SD_CLK, LOW);
        if (spi_half_period_us > 0)
            delayMicroseconds(spi_half_period_us);
    }

    return result;
#endif
}

// Send command to SD card
static uint8_t sd_send_command(uint8_t cmd, uint32_t arg)
{
    uint8_t response;
    uint16_t retry = 0;

    // Send command packet
    spi_transfer(0x40 | cmd);           // Command with start bits
    spi_transfer((uint8_t)(arg >> 24)); // Argument[31..24]
    spi_transfer((uint8_t)(arg >> 16)); // Argument[23..16]
    spi_transfer((uint8_t)(arg >> 8));  // Argument[15..8]
    spi_transfer((uint8_t)arg);         // Argument[7..0]

    // CRC (only needed for CMD0 and CMD8)
    if (cmd == CMD0)
        spi_transfer(0x95); // CRC for CMD0
    else if (cmd == CMD8)
        spi_transfer(0x87); // CRC for CMD8
    else
        spi_transfer(0xFF); // Dummy CRC

    // Wait for response (R1)
    do
    {
        response = spi_transfer(0xFF);
        retry++;
    } while ((response & 0x80) && retry < 0xFF);

    return response;
}

bool sd_spi_init(void)
{
    uint8_t response;
    uint16_t retry;

    fat_cache_invalidate();
    fat_next_free_cluster_hint = 2;

    // Initialize pins - use Arduino functions for consistency with rest of codebase
    // (Don't mix Arduino pinMode with Pico SDK gpio_* functions - they conflict)
    pinMode(SD_CS, OUTPUT);
    pinMode(SD_CLK, OUTPUT);
    pinMode(SD_MOSI, OUTPUT);
    pinMode(SD_MISO, INPUT);

    // Set initial state
    digitalWrite(SD_CS, HIGH);
    digitalWrite(SD_CLK, LOW);
    digitalWrite(SD_MOSI, HIGH);

    // SD Specification: Power-on delay
    // Cards need 1ms minimum (typ. 74 clocks) after power reaches operating voltage
    // Add 250ms for robust initialization across different card manufacturers
    delay(250);

    // Note: PIO is NOT initialized here - SD card init requires slow clock.
    // PIO will be initialized automatically when high-speed transfers are needed.

    // Send 80+ clock pulses with CS high to initialize the card
    // SD spec requires minimum 74 clocks, send extra for reliability
    for (int i = 0; i < 20; i++)
        spi_transfer(0xFF);

    // Allow card to process initialization clocks
    delay(50);

    // Enter SPI mode (CMD0) with retry
    // Some cards need multiple attempts, especially on cold boot
    retry = 0;
    do
    {
        SD_CS_LOW();
        delay(1);

        response = sd_send_command(CMD0, 0);

        SD_CS_HIGH();

        if (response == R1_IDLE_STATE)
        {
            break; // Success
        }

        // Failed - wait and retry
        delay(100);
        retry++;

        // Send more idle clocks between retries
        for (int i = 0; i < 10; i++)
            spi_transfer(0xFF);

    } while (retry < 10);

    if (response != R1_IDLE_STATE)
    {
        Serial.print("CMD0 failed after ");
        Serial.print(retry);
        Serial.print(" retries: 0x");
        Serial.println(response, HEX);

        // Diagnostic: Test if pins are responding
        Serial.println("\n--- SD Card Diagnostic ---");
        Serial.print("Testing GPIO pin states...");
        Serial.print(" CLK=");
        Serial.print(digitalRead(SD_CLK));
        Serial.print(" MOSI=");
        Serial.print(digitalRead(SD_MOSI));
        Serial.print(" MISO=");
        Serial.print(digitalRead(SD_MISO));
        Serial.print(" CS=");
        Serial.println(digitalRead(SD_CS));

        // Test clock toggle
        Serial.print("Testing clock toggle... ");
        digitalWrite(SD_CLK, HIGH);
        delay(1);
        uint8_t clk_hi = digitalRead(SD_CLK);
        digitalWrite(SD_CLK, LOW);
        delay(1);
        uint8_t clk_lo = digitalRead(SD_CLK);
        Serial.print("CLK_HI=");
        Serial.print(clk_hi);
        Serial.print(" CLK_LO=");
        Serial.println(clk_lo);

        // Test MOSI
        Serial.print("Testing MOSI toggle... ");
        digitalWrite(SD_MOSI, LOW);
        delay(1);
        uint8_t mosi_lo = digitalRead(SD_MOSI);
        digitalWrite(SD_MOSI, HIGH);
        delay(1);
        uint8_t mosi_hi = digitalRead(SD_MOSI);
        Serial.print("MOSI_LO=");
        Serial.print(mosi_lo);
        Serial.print(" MOSI_HI=");
        Serial.println(mosi_hi);

        // Test CS toggle
        Serial.print("Testing CS toggle... ");
        digitalWrite(SD_CS, LOW);
        delay(1);
        uint8_t cs_lo = digitalRead(SD_CS);
        digitalWrite(SD_CS, HIGH);
        delay(1);
        uint8_t cs_hi = digitalRead(SD_CS);
        Serial.print("CS_LO=");
        Serial.print(cs_lo);
        Serial.print(" CS_HI=");
        Serial.println(cs_hi);

        Serial.print("MISO reads: ");
        for (int i = 0; i < 10; i++)
        {
            Serial.print(digitalRead(SD_MISO));
            delay(100);
        }
        Serial.println();
        Serial.println("--- Check SD card connections and power ---\n");

        return false;
    }

    // CMD0 succeeded, continue with initialization
    SD_CS_LOW();
    delay(1);

    // Check voltage range (CMD8)
    response = sd_send_command(CMD8, 0x1AA);
    if (response == R1_IDLE_STATE)
    {
        // SD Ver 2.x
        uint32_t ocr = 0;
        for (int i = 0; i < 4; i++)
        {
            ocr = (ocr << 8) | spi_transfer(0xFF);
        }

        if ((ocr & 0x1FF) != 0x1AA)
        {
            SD_CS_HIGH();
            Serial.println("Voltage range not supported");
            return false;
        }

        // Initialize card (ACMD41)
        retry = 0;
        do
        {
            sd_send_command(CMD55, 0);
            response = sd_send_command(ACMD41, 0x40000000);
            retry++;
            delay(10);
        } while (response != R1_READY_STATE && retry < 100);

        if (response != R1_READY_STATE)
        {
            SD_CS_HIGH();
            Serial.println("ACMD41 timeout");
            return false;
        }

        // Read OCR (CMD58)
        response = sd_send_command(CMD58, 0);
        if (response == R1_READY_STATE)
        {
            ocr = 0;
            for (int i = 0; i < 4; i++)
            {
                ocr = (ocr << 8) | spi_transfer(0xFF);
            }
            card_type = (ocr & 0x40000000) ? 2 : 1; // SDHC/SDXC : SDSC
        }
    }
    else
    {
        // SD Ver 1.x or MMC
        retry = 0;
        do
        {
            sd_send_command(CMD55, 0);
            response = sd_send_command(ACMD41, 0);
            retry++;
            delay(10);
        } while (response != R1_READY_STATE && retry < 100);

        if (response == R1_READY_STATE)
        {
            card_type = 1; // SD Ver 1.x
        }
        else
        {
            // Try MMC
            retry = 0;
            do
            {
                response = sd_send_command(CMD1, 0);
                retry++;
                delay(10);
            } while (response != R1_READY_STATE && retry < 100);

            if (response != R1_READY_STATE)
            {
                SD_CS_HIGH();
                Serial.println("Card initialization failed");
                return false;
            }
            card_type = 3; // MMC
        }
    }

    // Set block length to 512 bytes (for SDSC only)
    if (card_type == 1)
    {
        response = sd_send_command(CMD16, 512);
        if (response != R1_READY_STATE)
        {
            SD_CS_HIGH();
            Serial.println("CMD16 failed");
            return false;
        }
    }

    SD_CS_HIGH();
    spi_transfer(0xFF);

    card_initialized = true;
    Serial.print("SD card initialized, type: ");
    Serial.println(card_type);

    // DEBUG: PIO initialization disabled temporarily - causing SPI failures
    // PIO will be re-enabled after debugging
    // #ifdef USE_PIO_SPI
    // if (init_pio_spi())
    // {
    //     Serial.println("PIO SPI enabled for high-speed transfers");
    // }
    // else
    // {
    //     Serial.println("PIO SPI initialization failed, using Fast GPIO fallback");
    // }
    // #endif

    return true;
}

bool sd_spi_read_block(uint32_t block, uint8_t *buffer)
{
    if (!card_initialized)
        return false;

    // Convert block address for SDSC
    if (card_type == 1)
        block *= 512;

    SD_CS_LOW();

    uint8_t response = sd_send_command(CMD17, block);
    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Wait for data token
    uint16_t retry = 0;
    do
    {
        response = spi_transfer(0xFF);
        retry++;
    } while (response != TOKEN_START_BLOCK && retry < 0xFFFF);

    if (response != TOKEN_START_BLOCK)
    {
        SD_CS_HIGH();
        return false;
    }

    // Read data
    for (int i = 0; i < 512; i++)
    {
        buffer[i] = spi_transfer(0xFF);
    }

    // Read CRC (ignore)
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);

    return true;
}

bool sd_spi_write_block(uint32_t block, const uint8_t *buffer)
{
    if (!card_initialized)
        return false;

    // Convert block address for SDSC
    if (card_type == 1)
        block *= 512;

    SD_CS_LOW();

    uint8_t response = sd_send_command(CMD24, block);
    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Send data token
    spi_transfer(TOKEN_START_BLOCK);

    // Write data
    for (int i = 0; i < 512; i++)
    {
        spi_transfer(buffer[i]);
    }

    // Write dummy CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    // Check data response
    response = spi_transfer(0xFF);
    if ((response & 0x1F) != 0x05)
    {
        SD_CS_HIGH();
        return false;
    }

    // Wait for write completion
    uint16_t retry = 0;
    do
    {
        response = spi_transfer(0xFF);
        retry++;
    } while (response == 0x00 && retry < 0xFFFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);

    return true;
}

uint8_t sd_spi_get_status(void)
{
    return card_initialized ? 0x00 : 0x01; // 0 = OK, 1 = Not initialized
}

uint32_t sd_spi_get_sector_count(void)
{
    // This would require reading the CSD register (CMD9)
    // For simplicity, return a default value or implement CMD9
    return 0; // Not implemented in this basic version
}

bool sd_spi_read_csd(uint8_t *csd)
{
    if (!card_initialized || !csd)
        return false;

    SD_CS_LOW();

    uint8_t response = sd_send_command(CMD9, 0);
    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Wait for data token
    uint16_t retry = 0;
    do
    {
        response = spi_transfer(0xFF);
        retry++;
    } while (response != TOKEN_START_BLOCK && retry < 0xFFFF);

    if (response != TOKEN_START_BLOCK)
    {
        SD_CS_HIGH();
        return false;
    }

    // Read CSD (16 bytes)
    for (int i = 0; i < 16; i++)
    {
        csd[i] = spi_transfer(0xFF);
    }

    // Read CRC (ignore)
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);

    // Store CSD locally
    memcpy(card_csd, csd, 16);

    return true;
}

uint64_t sd_spi_get_capacity(void)
{
    if (!card_initialized)
        return 0;

    // If we haven't read CSD yet, try to read it
    if (card_csd[0] == 0)
    {
        if (!sd_spi_read_csd(card_csd))
            return 0;
    }

    uint64_t capacity = 0;

    // Parse CSD based on version
    uint8_t csd_version = (card_csd[0] >> 6) & 0x03;

    if (csd_version == 0)
    {
        // CSD v1.0 (SDSC)
        // Capacity = (C_SIZE + 1) * 2^(C_SIZE_MULT + 2) * 2^READ_BL_LEN
        uint16_t c_size = ((card_csd[6] & 0x03) << 10) | (card_csd[7] << 2) | ((card_csd[8] >> 6) & 0x03);
        uint8_t c_size_mult = ((card_csd[9] & 0x03) << 1) | ((card_csd[10] >> 7) & 0x01);
        uint8_t read_bl_len = card_csd[5] & 0x0F;

        capacity = (uint32_t)(c_size + 1) << (c_size_mult + 2 + read_bl_len);
    }
    else if (csd_version == 1)
    {
        // CSD v2.0 (SDHC/SDXC)
        // Capacity = (C_SIZE + 1) * 512K
        uint32_t c_size = ((card_csd[7] & 0x3F) << 16) | (card_csd[8] << 8) | card_csd[9];
        capacity = (uint64_t)(c_size + 1) * 512 * 1024;
    }

    return capacity;
}

const char *sd_spi_get_card_type_string(void)
{
    switch (card_type)
    {
    case 0:
        return "Unknown/Not Initialized";
    case 1:
        return "SD Ver 1.x (SDSC)";
    case 2:
        return "SD Ver 2.x (SDHC/SDXC)";
    case 3:
        return "MMC";
    default:
        return "Invalid Type";
    }
}

bool sd_spi_read_cid(uint8_t *cid)
{
    if (!card_initialized)
        return false;

    SD_CS_LOW();
    uint8_t response = sd_send_command(CMD10, 0);

    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Wait for data token
    int timeout = 1000;
    while (spi_transfer(0xFF) != TOKEN_START_BLOCK)
    {
        if (--timeout == 0)
        {
            SD_CS_HIGH();
            return false;
        }
    }

    // Read 16 bytes of CID
    for (int i = 0; i < 16; i++)
    {
        cid[i] = spi_transfer(0xFF);
    }

    // Read CRC (2 bytes, ignored)
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);

    return true;
}

bool sd_spi_read_ocr(uint32_t *ocr)
{
    if (!card_initialized)
        return false;

    SD_CS_LOW();
    uint8_t response = sd_send_command(CMD58, 0);

    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Read 4 bytes of OCR
    *ocr = 0;
    for (int i = 0; i < 4; i++)
    {
        *ocr = (*ocr << 8) | spi_transfer(0xFF);
    }

    SD_CS_HIGH();
    spi_transfer(0xFF);

    return true;
}

bool sd_spi_read_ssr(uint8_t *ssr)
{
    if (!card_initialized)
        return false;

    // Send ACMD13 (SD_STATUS) - need CMD55 first
    SD_CS_LOW();
    uint8_t response = sd_send_command(CMD55, 0);

    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Send ACMD13 (get SD status)
    response = sd_send_command(13, 0); // ACMD13

    if (response != R1_READY_STATE)
    {
        SD_CS_HIGH();
        return false;
    }

    // Wait for data token
    int timeout = 1000;
    while (spi_transfer(0xFF) != TOKEN_START_BLOCK)
    {
        if (--timeout == 0)
        {
            SD_CS_HIGH();
            return false;
        }
    }

    // Read 64 bytes of SSR
    for (int i = 0; i < 64; i++)
    {
        ssr[i] = spi_transfer(0xFF);
    }

    // Read CRC (2 bytes, ignored)
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    SD_CS_HIGH();
    spi_transfer(0xFF);

    return true;
}

void sd_spi_set_frequency(uint32_t freq_hz)
{
    // Calculate half-period delay in microseconds
    // freq = 1 / (2 * delay_us * 1e-6)
    // delay_us = 1000000 / (2 * freq)
    if (freq_hz == 0)
    {
        spi_half_period_us = 1; // Default to 500kHz
    }
    else if (freq_hz > 10000000)
    {
        spi_half_period_us = 0; // No delay for >10MHz (max speed)
    }
    else
    {
        spi_half_period_us = 500000 / freq_hz;
        if (spi_half_period_us == 0)
            spi_half_period_us = 0; // No delay for very high frequencies
    }
}

uint32_t sd_spi_get_frequency(void)
{
    if (spi_half_period_us == 0)
    {
        return 10000000; // Return max theoretical frequency
    }
    return 500000 / spi_half_period_us;
}

const char *sd_spi_get_method(void)
{
#ifdef USE_PIO_SPI
    if (pio_initialized && spi_half_period_us == 0)
    {
        return "PIO";
    }
#endif
#ifdef USE_FAST_GPIO
    return "Fast GPIO";
#else
    return "Standard digitalWrite";
#endif
}

// Helper function to read a 16-bit value in little-endian
static uint16_t read_u16_le(const uint8_t *buffer, uint16_t offset)
{
    return buffer[offset] | (buffer[offset + 1] << 8);
}

// Helper function to read a 32-bit value in little-endian
static uint32_t read_u32_le(const uint8_t *buffer, uint32_t offset)
{
    return buffer[offset] | (buffer[offset + 1] << 8) | (buffer[offset + 2] << 16) | (buffer[offset + 3] << 24);
}

bool sd_spi_identify_filesystem(fs_info_t *fs_info)
{
    if (!card_initialized || !fs_info)
        return false;

    uint8_t sector_buffer[512];
    uint32_t boot_sector_offset = 0;

    // Read sector 0 (MBR or boot sector)
    if (!sd_spi_read_block(0, sector_buffer))
        return false;

    // Check for valid sector signature
    if (sector_buffer[510] != 0x55 || sector_buffer[511] != 0xAA)
        return false;

    // Check if this is an MBR (has partition table) or direct boot sector
    // MBR check: byte at offset 0x1BE should be 0x00 or 0x80 (bootable flag)
    // and filesystem type bytes should indicate FAT partition
    bool is_mbr = false;

    // Simple heuristic: if bytes 11-12 (bytes per sector) are NOT 0x0200 (512),
    // it's likely an MBR, not a boot sector
    uint16_t possible_bps = read_u16_le(sector_buffer, 11);
    if (possible_bps != 512 && possible_bps != 1024 && possible_bps != 2048 && possible_bps != 4096)
    {
        is_mbr = true;
    }

    // Also check for partition table marker
    if (sector_buffer[0x1BE] == 0x00 || sector_buffer[0x1BE] == 0x80)
    {
        // Check if partition 1 has a valid filesystem type
        uint8_t partition_type = sector_buffer[0x1BE + 4]; // Partition type
        if (partition_type == 0x01 || partition_type == 0x04 || partition_type == 0x06 ||
            partition_type == 0x0B || partition_type == 0x0C || partition_type == 0x0E ||
            partition_type == 0x07) // FAT12/16/32 or exFAT
        {
            is_mbr = true;
        }
    }

    if (is_mbr)
    {
        // Read partition 1 offset from MBR (offset 0x1C6)
        boot_sector_offset = read_u32_le(sector_buffer, 0x1C6);

        Serial.print("MBR detected, partition 1 starts at sector: ");
        Serial.println(boot_sector_offset);

        // Read actual boot sector from partition
        if (!sd_spi_read_block(boot_sector_offset, sector_buffer))
            return false;

        // Verify boot sector signature again
        if (sector_buffer[510] != 0x55 || sector_buffer[511] != 0xAA)
            return false;
    }

    fs_info->partition_offset = boot_sector_offset;

    // Check for exFAT signature
    if (sector_buffer[3] == 'E' && sector_buffer[4] == 'X' && sector_buffer[5] == 'F' &&
        sector_buffer[6] == 'A' && sector_buffer[7] == 'T')
    {
        fs_info->type = 4;                                      // exFAT
        fs_info->bytes_per_sector = 1 << sector_buffer[108];    // 2^N bytes
        fs_info->sectors_per_cluster = 1 << sector_buffer[109]; // 2^N sectors
        fs_info->reserved_sectors = read_u32_le(sector_buffer, 72);
        fs_info->fat_sectors = read_u32_le(sector_buffer, 80);
        fs_info->data_sectors = read_u32_le(sector_buffer, 88);
        fs_info->root_dir_sector = read_u32_le(sector_buffer, 96);
        return true;
    }

    // Parse standard FAT boot sector
    fs_info->bytes_per_sector = read_u16_le(sector_buffer, 11);
    fs_info->sectors_per_cluster = sector_buffer[13];
    fs_info->reserved_sectors = read_u16_le(sector_buffer, 14);
    fs_info->fat_count = sector_buffer[16];

    uint16_t root_entries = read_u16_le(sector_buffer, 17);
    uint16_t total_sectors_16 = read_u16_le(sector_buffer, 19);
    uint16_t fat_size_16 = read_u16_le(sector_buffer, 22);

    fs_info->root_dir_entries = root_entries;

    // For FAT32, FAT size is at a different location
    if (fat_size_16 == 0)
    {
        // FAT32
        fs_info->fat_sectors = read_u32_le(sector_buffer, 36);
    }
    else
    {
        fs_info->fat_sectors = fat_size_16;
    }

    fs_info->total_sectors = total_sectors_16 ? total_sectors_16 : read_u32_le(sector_buffer, 32);

    // Calculate root directory sector (absolute sector on card)
    fs_info->root_dir_sector = boot_sector_offset + fs_info->reserved_sectors + (fs_info->fat_count * fs_info->fat_sectors);

    // Calculate data sectors
    uint32_t root_dir_sectors = (root_entries * 32 + fs_info->bytes_per_sector - 1) / fs_info->bytes_per_sector;
    fs_info->data_sectors = fs_info->total_sectors - fs_info->reserved_sectors - (fs_info->fat_count * fs_info->fat_sectors) - root_dir_sectors;

    // Calculate cluster count
    fs_info->cluster_count = fs_info->data_sectors / fs_info->sectors_per_cluster;

    // Determine filesystem type based on cluster count
    if (fs_info->cluster_count < 4085)
    {
        fs_info->type = 1; // FAT12
    }
    else if (fs_info->cluster_count < 65525)
    {
        fs_info->type = 2; // FAT16
    }
    else
    {
        fs_info->type = 3; // FAT32
        // For FAT32, read root directory cluster from offset 44
        fs_info->root_dir_cluster = read_u32_le(sector_buffer, 44);
    }

    // Calculate data start sector (first sector of cluster 2)
    fs_info->data_start_sector = boot_sector_offset + fs_info->reserved_sectors +
                                 (fs_info->fat_count * fs_info->fat_sectors) + root_dir_sectors;

    return true;
}

const char *sd_spi_get_fs_type_string(uint8_t type)
{
    switch (type)
    {
    case 0:
        return "Unknown";
    case 1:
        return "FAT12";
    case 2:
        return "FAT16";
    case 3:
        return "FAT32";
    case 4:
        return "exFAT";
    default:
        return "Invalid Filesystem";
    }
}

// Helper to convert FAT filename to null-terminated string
static void parse_fat_filename(const uint8_t *entry, char *filename)
{
    int i = 0;

    // Read 8-character filename
    for (int j = 0; j < 8 && entry[j] != ' '; j++, i++)
    {
        filename[i] = entry[j];
    }

    // Add dot if there's an extension
    if (entry[8] != ' ')
    {
        filename[i++] = '.';

        // Read 3-character extension
        for (int j = 0; j < 3 && entry[8 + j] != ' '; j++, i++)
        {
            filename[i] = entry[8 + j];
        }
    }

    filename[i] = '\0';
}

// Helper to convert FAT date/time
static uint16_t parse_fat_date(const uint8_t *entry, uint16_t offset)
{
    return read_u16_le(entry, offset);
}

static uint16_t parse_fat_time(const uint8_t *entry, uint16_t offset)
{
    return read_u16_le(entry, offset);
}

// Helper to extract Unicode characters from LFN entry
static void extract_lfn_chars(const uint8_t *lfn_entry, char *buffer, int *pos)
{
    // LFN entry structure:
    // Offset 1-10: 5 Unicode chars (10 bytes)
    // Offset 14-25: 6 Unicode chars (12 bytes)
    // Offset 28-31: 2 Unicode chars (4 bytes)

    // Extract first 5 characters (offset 1-10)
    for (int i = 0; i < 5; i++)
    {
        uint16_t unicode_char = lfn_entry[1 + i * 2] | (lfn_entry[1 + i * 2 + 1] << 8);
        if (unicode_char == 0x0000 || unicode_char == 0xFFFF)
            return;                                     // End of name or padding
        buffer[(*pos)++] = (char)(unicode_char & 0xFF); // Simple Unicode to ASCII
    }

    // Extract next 6 characters (offset 14-25)
    for (int i = 0; i < 6; i++)
    {
        uint16_t unicode_char = lfn_entry[14 + i * 2] | (lfn_entry[14 + i * 2 + 1] << 8);
        if (unicode_char == 0x0000 || unicode_char == 0xFFFF)
            return;
        buffer[(*pos)++] = (char)(unicode_char & 0xFF);
    }

    // Extract last 2 characters (offset 28-31)
    for (int i = 0; i < 2; i++)
    {
        uint16_t unicode_char = lfn_entry[28 + i * 2] | (lfn_entry[28 + i * 2 + 1] << 8);
        if (unicode_char == 0x0000 || unicode_char == 0xFFFF)
            return;
        buffer[(*pos)++] = (char)(unicode_char & 0xFF);
    }
}

// ============================================================================
// FULL FAT FILESYSTEM IMPLEMENTATION
// ============================================================================

/**
 * FAT Filesystem Helper Structures and Constants
 */

// FAT Directory Entry Structure (32 bytes)
typedef struct
{
    uint8_t filename[11];        // Filename with extension
    uint8_t attributes;          // File attributes
    uint8_t reserved;            // Reserved for NT
    uint8_t creation_time_tenth; // Creation time finer resolution
    uint16_t creation_time;      // Creation time
    uint16_t creation_date;      // Creation date
    uint16_t access_date;        // Last access date
    uint16_t cluster_high;       // High word of first cluster (FAT32)
    uint16_t write_time;         // Write time
    uint16_t write_date;         // Write date
    uint16_t cluster_low;        // Low word of first cluster
    uint32_t file_size;          // File size in bytes
} fat_dir_entry_t;

// FAT Directory Entry Attributes
#define FAT_ATTR_READ_ONLY 0x01
#define FAT_ATTR_HIDDEN 0x02
#define FAT_ATTR_SYSTEM 0x04
#define FAT_ATTR_VOLUME_ID 0x08
#define FAT_ATTR_DIRECTORY 0x10
#define FAT_ATTR_ARCHIVE 0x20
#define FAT_ATTR_LONG_NAME 0x0F

// Helper: Get first cluster from FAT entry
static uint32_t fat_get_cluster(const fat_dir_entry_t *entry, uint8_t fs_type)
{
    uint32_t cluster = entry->cluster_low;
    if (fs_type == 3) // FAT32
    {
        cluster |= ((uint32_t)entry->cluster_high << 16);
    }
    return cluster;
}

// Helper: Convert DOS time/date to text
static void fat_format_datetime(uint16_t date, uint16_t time, char *buffer, size_t buf_len)
{
    int year = ((date >> 9) & 0x7F) + 1980;
    int month = (date >> 5) & 0x0F;
    int day = date & 0x1F;
    int hour = (time >> 11) & 0x1F;
    int min = (time >> 5) & 0x3F;
    int sec = (time & 0x1F) * 2;

    snprintf(buffer, buf_len, "%04d-%02d-%02d %02d:%02d:%02d", year, month, day, hour, min, sec);
}

static void fat_cache_invalidate(void)
{
    fat_cache_valid = false;
    fat_cache_dirty = false;
    fat_cache_sector = 0;
}

static bool fat_cache_flush(void)
{
    if (!fat_cache_valid || !fat_cache_dirty)
        return true;

    if (!sd_spi_write_block(fat_cache_sector, fat_cache_data))
        return false;

    fat_cache_dirty = false;
    return true;
}

static bool fat_cache_load_sector(uint32_t sector)
{
    if (fat_cache_valid && fat_cache_sector == sector)
        return true;

    if (!fat_cache_flush())
        return false;

    if (!sd_spi_read_block(sector, fat_cache_data))
        return false;

    fat_cache_valid = true;
    fat_cache_dirty = false;
    fat_cache_sector = sector;
    return true;
}

// Helper: Write FAT table entry
static bool fat_write_fat_entry(const fs_info_t *fs_info, uint32_t cluster, uint32_t next_cluster)
{
    if (fs_info->type == 3) // FAT32
    {
        // Each FAT32 entry is 4 bytes. Update all FAT copies to keep host readers in sync.
        uint32_t fat_offset = cluster * 4;
        uint32_t fat_sector_offset = fat_offset / 512;
        uint16_t fat_offset_in_sector = fat_offset % 512;

        for (uint32_t fat_index = 0; fat_index < fs_info->fat_count; fat_index++)
        {
            uint32_t fat_sector = fs_info->partition_offset + fs_info->reserved_sectors +
                                  (fat_index * fs_info->fat_sectors) + fat_sector_offset;

            if (!fat_cache_load_sector(fat_sector))
                return false;

            uint8_t preserved_status = fat_cache_data[fat_offset_in_sector + 3] & 0xF0;
            fat_cache_data[fat_offset_in_sector] = (uint8_t)(next_cluster & 0xFF);
            fat_cache_data[fat_offset_in_sector + 1] = (uint8_t)((next_cluster >> 8) & 0xFF);
            fat_cache_data[fat_offset_in_sector + 2] = (uint8_t)((next_cluster >> 16) & 0xFF);
            fat_cache_data[fat_offset_in_sector + 3] = preserved_status | (uint8_t)((next_cluster >> 24) & 0x0F);
            fat_cache_dirty = true;
        }
    }
    else if (fs_info->type == 2) // FAT16
    {
        uint32_t fat_offset = cluster * 2;
        uint32_t fat_sector_offset = fat_offset / 512;
        uint16_t fat_offset_in_sector = fat_offset % 512;

        for (uint32_t fat_index = 0; fat_index < fs_info->fat_count; fat_index++)
        {
            uint32_t fat_sector = fs_info->partition_offset + fs_info->reserved_sectors +
                                  (fat_index * fs_info->fat_sectors) + fat_sector_offset;

            if (!fat_cache_load_sector(fat_sector))
                return false;

            fat_cache_data[fat_offset_in_sector] = (uint8_t)(next_cluster & 0xFF);
            fat_cache_data[fat_offset_in_sector + 1] = (uint8_t)((next_cluster >> 8) & 0xFF);
            fat_cache_dirty = true;
        }
    }
    else if (fs_info->type == 1) // FAT12
    {
        uint32_t fat_offset = (cluster * 12) / 8;
        uint32_t fat_sector_offset = fat_offset / 512;
        uint16_t fat_offset_in_sector = fat_offset % 512;

        for (uint32_t fat_index = 0; fat_index < fs_info->fat_count; fat_index++)
        {
            uint32_t fat_sector = fs_info->partition_offset + fs_info->reserved_sectors +
                                  (fat_index * fs_info->fat_sectors) + fat_sector_offset;

            if (!fat_cache_load_sector(fat_sector))
                return false;

            if ((cluster & 1) == 0)
            {
                fat_cache_data[fat_offset_in_sector] = (uint8_t)(next_cluster & 0xFF);
                fat_cache_data[fat_offset_in_sector + 1] = (uint8_t)((next_cluster >> 8) & 0x0F) |
                                                           (fat_cache_data[fat_offset_in_sector + 1] & 0xF0);
            }
            else
            {
                fat_cache_data[fat_offset_in_sector] = (uint8_t)((next_cluster << 4) & 0xF0) |
                                                       (fat_cache_data[fat_offset_in_sector] & 0x0F);
                fat_cache_data[fat_offset_in_sector + 1] = (uint8_t)((next_cluster >> 4) & 0xFF);
            }

            fat_cache_dirty = true;
        }
    }

    return true;
}

// Helper: Read FAT table entry
static uint32_t fat_read_fat_entry(const fs_info_t *fs_info, uint32_t cluster)
{
    uint32_t next_cluster = 0;

    if (fs_info->type == 3) // FAT32
    {
        uint32_t fat_offset = cluster * 4;
        uint32_t fat_sector = fs_info->partition_offset + fs_info->reserved_sectors + (fat_offset / 512);
        uint16_t fat_offset_in_sector = fat_offset % 512;

        if (!fat_cache_load_sector(fat_sector))
            return 0;

        next_cluster = fat_cache_data[fat_offset_in_sector] |
                       (fat_cache_data[fat_offset_in_sector + 1] << 8) |
                       (fat_cache_data[fat_offset_in_sector + 2] << 16) |
                       ((fat_cache_data[fat_offset_in_sector + 3] & 0x0F) << 24);
    }
    else if (fs_info->type == 2) // FAT16
    {
        uint32_t fat_offset = cluster * 2;
        uint32_t fat_sector = fs_info->partition_offset + fs_info->reserved_sectors + (fat_offset / 512);
        uint16_t fat_offset_in_sector = fat_offset % 512;

        if (!fat_cache_load_sector(fat_sector))
            return 0;

        next_cluster = fat_cache_data[fat_offset_in_sector] |
                       (fat_cache_data[fat_offset_in_sector + 1] << 8);
    }
    else if (fs_info->type == 1) // FAT12
    {
        uint32_t fat_offset = (cluster * 12) / 8;
        uint32_t fat_sector = fs_info->partition_offset + fs_info->reserved_sectors + (fat_offset / 512);
        uint16_t fat_offset_in_sector = fat_offset % 512;

        if (!fat_cache_load_sector(fat_sector))
            return 0;

        if ((cluster & 1) == 0)
        {
            next_cluster = fat_cache_data[fat_offset_in_sector] |
                           ((fat_cache_data[fat_offset_in_sector + 1] & 0x0F) << 8);
        }
        else
        {
            next_cluster = ((fat_cache_data[fat_offset_in_sector] & 0xF0) >> 4) |
                           (fat_cache_data[fat_offset_in_sector + 1] << 4);
        }
    }

    return next_cluster;
}

// Helper: Find free cluster
static uint32_t fat_find_free_cluster(const fs_info_t *fs_info)
{
    uint32_t max_cluster_number = fs_info->cluster_count + 2;
    uint32_t start_cluster = fat_next_free_cluster_hint;
    if (start_cluster < 2 || start_cluster >= max_cluster_number)
        start_cluster = 2;

    // Scan from hint to end.
    for (uint32_t cluster = start_cluster; cluster < max_cluster_number; cluster++)
    {
        uint32_t fat_entry = fat_read_fat_entry(fs_info, cluster);
        if (fat_entry == 0) // Free cluster
        {
            fat_next_free_cluster_hint = cluster + 1;
            return cluster;
        }
    }

    // Wrap around once.
    for (uint32_t cluster = 2; cluster < start_cluster; cluster++)
    {
        uint32_t fat_entry = fat_read_fat_entry(fs_info, cluster);
        if (fat_entry == 0) // Free cluster
        {
            fat_next_free_cluster_hint = cluster + 1;
            return cluster;
        }
    }

    return 0; // No free cluster found
}

// Helper: Get cluster sector address
static uint32_t fat_cluster_to_sector(const fs_info_t *fs_info, uint32_t cluster)
{
    if (cluster < 2)
        return 0;

    return fs_info->data_start_sector + ((cluster - 2) * fs_info->sectors_per_cluster);
}

// Helper: Create FAT directory entry
static void fat_create_dir_entry(fat_dir_entry_t *entry, const char *name, uint8_t attr, uint32_t cluster, uint32_t size)
{
    memset(entry, 0, sizeof(fat_dir_entry_t));

    // Set filename (8 chars, padded with spaces)
    memset(entry->filename, ' ', 11);

    // Special case for "." and ".." directory entries (FAT spec requirement)
    if (strcmp(name, ".") == 0)
    {
        entry->filename[0] = '.';
        // Rest is spaces (already set by memset above)
    }
    else if (strcmp(name, "..") == 0)
    {
        entry->filename[0] = '.';
        entry->filename[1] = '.';
        // Rest is spaces (already set by memset above)
    }
    else
    {
        // Parse filename and extension for normal files
        const char *dot = strrchr(name, '.');
        int name_len = dot ? (dot - name) : strlen(name);
        int ext_len = dot ? strlen(dot + 1) : 0;

        // Limit to 8.3 format
        if (name_len > 8)
            name_len = 8;
        if (ext_len > 3)
            ext_len = 3;

        // Copy filename and extension
        memcpy(entry->filename, name, name_len);
        if (ext_len > 0)
        {
            memcpy(entry->filename + 8, dot + 1, ext_len);
        }
    }

    // Convert to uppercase
    for (int i = 0; i < 11; i++)
    {
        if (entry->filename[i] >= 'a' && entry->filename[i] <= 'z')
            entry->filename[i] -= 32;
    }

    entry->attributes = attr;
    entry->cluster_low = (uint16_t)(cluster & 0xFFFF);
    entry->cluster_high = (uint16_t)((cluster >> 16) & 0xFFFF);
    entry->file_size = size;

    // Set current date/time
    uint16_t dos_date = ((2026 - 1980) << 9) | (3 << 5) | 2; // 2026-03-02
    uint16_t dos_time = (12 << 11) | (0 << 5) | (0 / 2);     // 12:00:00
    entry->creation_date = dos_date;
    entry->creation_time = dos_time;
    entry->write_date = dos_date;
    entry->write_time = dos_time;
    entry->access_date = dos_date;
}

// Find directory entry in a directory
typedef struct
{
    uint32_t sector;      // Sector containing the entry
    uint16_t offset;      // Offset within the sector
    uint32_t dir_cluster; // Cluster number of the directory
} dir_entry_location_t;

static bool fat_find_entry_in_dir(const fs_info_t *fs_info, uint32_t dir_cluster, const char *name,
                                  fat_dir_entry_t *entry, dir_entry_location_t *location)
{
    uint8_t sector_buffer[512];
    uint32_t current_cluster = dir_cluster;

    // For root directory in FAT12/16
    if (fs_info->type != 3 && dir_cluster == 0)
    {
        uint32_t sector = fs_info->root_dir_sector;
        uint32_t root_dir_size = fs_info->root_dir_entries * 32;
        uint32_t sectors_to_read = (root_dir_size + 511) / 512;

        for (uint32_t s = 0; s < sectors_to_read; s++)
        {
            if (!sd_spi_read_block(sector + s, sector_buffer))
                return false;

            for (int offset = 0; offset < 512; offset += 32)
            {
                fat_dir_entry_t *dir_entry = (fat_dir_entry_t *)(sector_buffer + offset);

                // Check for end of directory
                if (dir_entry->filename[0] == 0x00)
                    return false;

                // Skip deleted or long name entries
                if (dir_entry->filename[0] == 0xE5 || dir_entry->attributes == FAT_ATTR_LONG_NAME)
                    continue;

                // Compare filename
                char test_name[13];
                memset(test_name, 0, sizeof(test_name));

                // Copy filename part
                for (int i = 0; i < 8 && dir_entry->filename[i] != ' '; i++)
                    test_name[i] = dir_entry->filename[i];

                // Add extension if present
                int base_len = strlen(test_name);
                if (dir_entry->filename[8] != ' ')
                {
                    test_name[base_len] = '.';
                    for (int i = 0; i < 3 && dir_entry->filename[8 + i] != ' '; i++)
                        test_name[base_len + 1 + i] = dir_entry->filename[8 + i];
                    test_name[base_len + 1 + 3] = '\0'; // Ensure null termination
                }

                // Convert both names to lowercase for comparison
                // Also apply 8.3 formatting to search name to match storage format
                char compare_name[13];
                memset(compare_name, 0, sizeof(compare_name));

                // Parse filename and extension from search name (same as fat_create_dir_entry)
                const char *dot = strrchr(name, '.');
                int name_len = dot ? (dot - name) : strlen(name);
                int ext_len = dot ? strlen(dot + 1) : 0;

                // Limit to 8.3 format
                if (name_len > 8)
                    name_len = 8;
                if (ext_len > 3)
                    ext_len = 3;

                // Copy name part (first 8 chars only)
                memcpy(compare_name, name, name_len);
                if (ext_len > 0)
                {
                    compare_name[name_len] = '.';
                    memcpy(compare_name + name_len + 1, dot + 1, ext_len);
                    compare_name[name_len + 1 + ext_len] = '\0'; // Ensure null termination

                    // Convert both to lowercase for comparison
                    for (char *p = test_name; *p; ++p)
                        *p = tolower(*p);
                    for (char *p = compare_name; *p; ++p)
                        *p = tolower(*p);

                    if (strcmp(test_name, compare_name) == 0)
                    {
                        *entry = *dir_entry;
                        if (location)
                        {
                            location->sector = sector + s;
                            location->offset = offset;
                            location->dir_cluster = dir_cluster;
                        }
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // For subdirectories or FAT32 root
    while (current_cluster < 0x0FFFFFF0)
    {
        uint32_t sector = fat_cluster_to_sector(fs_info, current_cluster);

        for (int s = 0; s < fs_info->sectors_per_cluster; s++)
        {
            if (!sd_spi_read_block(sector + s, sector_buffer))
                return false;

            for (int offset = 0; offset < 512; offset += 32)
            {
                fat_dir_entry_t *dir_entry = (fat_dir_entry_t *)(sector_buffer + offset);

                // Check for end of directory
                if (dir_entry->filename[0] == 0x00)
                    return false;

                // Skip deleted or long name entries
                if (dir_entry->filename[0] == 0xE5 || dir_entry->attributes == FAT_ATTR_LONG_NAME)
                    continue;

                // Compare filename
                char test_name[13];
                memset(test_name, 0, sizeof(test_name));

                // Copy filename part
                for (int i = 0; i < 8 && dir_entry->filename[i] != ' '; i++)
                    test_name[i] = dir_entry->filename[i];

                // Add extension if present
                int base_len = strlen(test_name);
                if (dir_entry->filename[8] != ' ')
                {
                    test_name[base_len] = '.';
                    for (int i = 0; i < 3 && dir_entry->filename[8 + i] != ' '; i++)
                        test_name[base_len + 1 + i] = dir_entry->filename[8 + i];
                    test_name[base_len + 1 + 3] = '\0'; // Ensure null termination
                }

                // Convert both names to lowercase for comparison
                // Also apply 8.3 formatting to search name to match storage format
                char compare_name[13];
                memset(compare_name, 0, sizeof(compare_name));

                // Parse filename and extension from search name (same as fat_create_dir_entry)
                const char *dot = strrchr(name, '.');
                int name_len = dot ? (dot - name) : strlen(name);
                int ext_len = dot ? strlen(dot + 1) : 0;

                // Limit to 8.3 format
                if (name_len > 8)
                    name_len = 8;
                if (ext_len > 3)
                    ext_len = 3;

                // Copy name part (first 8 chars only)
                memcpy(compare_name, name, name_len);
                if (ext_len > 0)
                {
                    compare_name[name_len] = '.';
                    memcpy(compare_name + name_len + 1, dot + 1, ext_len);
                    compare_name[name_len + 1 + ext_len] = '\0'; // Ensure null termination
                }

                // Convert both to lowercase for comparison
                for (char *p = test_name; *p; ++p)
                    *p = tolower(*p);
                for (char *p = compare_name; *p; ++p)
                    *p = tolower(*p);

                if (strcmp(test_name, compare_name) == 0)
                {
                    *entry = *dir_entry;
                    if (location)
                    {
                        location->sector = sector + s;
                        location->offset = offset;
                        location->dir_cluster = dir_cluster;
                    }
                    return true;
                }
            }
        }

        // Read next cluster
        current_cluster = fat_read_fat_entry(fs_info, current_cluster);
    }

    return false;
}

// Add entry to directory
static bool fat_add_entry_to_dir(const fs_info_t *fs_info, uint32_t dir_cluster, fat_dir_entry_t *new_entry)
{
    uint8_t sector_buffer[512];
    uint32_t current_cluster = dir_cluster;
    bool entry_added = false;

    // For root directory in FAT12/16
    if (fs_info->type != 3 && dir_cluster == 0)
    {
        uint32_t sector = fs_info->root_dir_sector;
        uint32_t root_dir_size = fs_info->root_dir_entries * 32;
        uint32_t sectors_to_read = (root_dir_size + 511) / 512;

        for (uint32_t s = 0; s < sectors_to_read && !entry_added; s++)
        {
            if (!sd_spi_read_block(sector + s, sector_buffer))
                return false;

            for (int offset = 0; offset < 512; offset += 32)
            {
                if (sector_buffer[offset] == 0x00 || sector_buffer[offset] == 0xE5)
                {
                    // Found free entry
                    memcpy(sector_buffer + offset, (uint8_t *)new_entry, 32);
                    if (!sd_spi_write_block(sector + s, sector_buffer))
                        return false;
                    entry_added = true;
                    break;
                }
            }
        }
        return entry_added;
    }

    // For subdirectories or FAT32 root
    while (current_cluster < 0x0FFFFFF0)
    {
        uint32_t sector = fat_cluster_to_sector(fs_info, current_cluster);

        for (int s = 0; s < fs_info->sectors_per_cluster && !entry_added; s++)
        {
            if (!sd_spi_read_block(sector + s, sector_buffer))
                return false;

            for (int offset = 0; offset < 512; offset += 32)
            {
                if (sector_buffer[offset] == 0x00 || sector_buffer[offset] == 0xE5)
                {
                    // Found free entry
                    memcpy(sector_buffer + offset, (uint8_t *)new_entry, 32);
                    if (!sd_spi_write_block(sector + s, sector_buffer))
                        return false;
                    entry_added = true;
                    break;
                }
            }
        }

        if (entry_added)
            break;

        // Move to next cluster
        current_cluster = fat_read_fat_entry(fs_info, current_cluster);
    }

    return entry_added;
}

// Navigate to a directory using path
static bool fat_navigate_to_dir(const fs_info_t *fs_info, const char *path, uint32_t *out_cluster)
{
    *out_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;

    if (!path || path[0] == 0 || (path[0] == '/' && path[1] == 0))
        return true; // Root directory

    // Make a copy to tokenize
    char path_copy[256];
    strncpy(path_copy, path, sizeof(path_copy) - 1);
    path_copy[sizeof(path_copy) - 1] = 0;

    // Remove leading slash
    char *p = path_copy;
    if (*p == '/')
        p++;

    // Tokenize path
    char *saveptr = NULL;
    char *token = strtok_r(p, "/", &saveptr);

    while (token && *token)
    {
        fat_dir_entry_t entry;
        if (!fat_find_entry_in_dir(fs_info, *out_cluster, token, &entry, NULL))
            return false; // Directory not found

        if (!(entry.attributes & FAT_ATTR_DIRECTORY))
            return false; // Not a directory

        *out_cluster = fat_get_cluster(&entry, fs_info->type);
        token = strtok_r(NULL, "/", &saveptr);
    }

    return true;
}

bool sd_spi_create_directory(const fs_info_t *fs_info, const char *path)
{
    if (!fs_info || !path)
        return false;

    // Find parent directory and directory name
    const char *last_slash = strrchr(path, '/');
    char parent_path[512];
    const char *dir_name;

    if (last_slash)
    {
        int parent_len = last_slash - path;
        strncpy(parent_path, path, parent_len);
        parent_path[parent_len] = 0;
        dir_name = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        dir_name = path;
    }

    // Navigate to parent directory
    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0])
    {
        if (!fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        {
            // Parent directory doesn't exist, create it recursively
            if (!sd_spi_create_directory(fs_info, parent_path))
                return false;

            // Try navigating again after creating parent
            if (!fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
                return false;
        }
    }

    // Check if directory already exists
    fat_dir_entry_t existing_entry;
    if (fat_find_entry_in_dir(fs_info, parent_cluster, dir_name, &existing_entry, NULL))
        return (existing_entry.attributes & FAT_ATTR_DIRECTORY) != 0; // Already exists

    // Allocate free cluster for new directory
    uint32_t new_cluster = fat_find_free_cluster(fs_info);
    if (new_cluster == 0)
        return false;

    // Create directory entry
    fat_dir_entry_t dir_entry;
    fat_create_dir_entry(&dir_entry, dir_name, FAT_ATTR_DIRECTORY, new_cluster, 0);

    // Add entry to parent directory
    if (!fat_add_entry_to_dir(fs_info, parent_cluster, &dir_entry))
        return false;

    // Mark cluster as end of chain
    if (!fat_write_fat_entry(fs_info, new_cluster, 0x0FFFFFF8))
        return false;

    // Initialize new directory with "." and ".." entries
    uint8_t dir_sector_buffer[512];
    memset(dir_sector_buffer, 0, 512);

    // Create "." entry (current directory)
    fat_dir_entry_t dot_entry;
    fat_create_dir_entry(&dot_entry, ".", FAT_ATTR_DIRECTORY, new_cluster, 0);
    memcpy(dir_sector_buffer, (uint8_t *)&dot_entry, 32);

    // Create ".." entry (parent directory)
    fat_dir_entry_t dotdot_entry;
    fat_create_dir_entry(&dotdot_entry, "..", FAT_ATTR_DIRECTORY, parent_cluster, 0);
    memcpy(dir_sector_buffer + 32, (uint8_t *)&dotdot_entry, 32);

    // Write all sectors in the cluster (first sector has "." and "..", rest are zeros)
    uint32_t dir_sector = fat_cluster_to_sector(fs_info, new_cluster);

    // Write first sector with "." and ".." entries
    if (!sd_spi_write_block(dir_sector, dir_sector_buffer))
        return false;

    // Write remaining sectors in the cluster as zeros
    memset(dir_sector_buffer, 0, 512);
    for (uint32_t i = 1; i < fs_info->sectors_per_cluster; i++)
    {
        if (!sd_spi_write_block(dir_sector + i, dir_sector_buffer))
            return false;
    }

    return true;
}

bool sd_spi_directory_exists(const fs_info_t *fs_info, const char *path)
{
    uint32_t cluster;
    return fat_navigate_to_dir(fs_info, path, &cluster);
}

bool sd_spi_file_exists(const fs_info_t *fs_info, const char *filepath)
{
    if (!fs_info || !filepath)
        return false;

    // Find parent directory and filename
    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    // Navigate to parent directory
    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        return false;

    // Find file entry
    fat_dir_entry_t entry;
    return fat_find_entry_in_dir(fs_info, parent_cluster, filename, &entry, NULL) &&
           !(entry.attributes & FAT_ATTR_DIRECTORY);
}

int sd_spi_write_file(const fs_info_t *fs_info, const char *filepath, const uint8_t *data, uint32_t size)
{
    if (!fs_info || !filepath || !data || size == 0)
        return -1;

    // Find parent directory and filename
    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    // Navigate to parent directory, creating if necessary
    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0])
    {
        if (!fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        {
            // Try to create the directory path
            if (!sd_spi_create_directory(fs_info, parent_path))
                return -1;

            if (!fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
                return -1;
        }
    }

    // Check if file already exists and remove it
    fat_dir_entry_t existing_entry;
    dir_entry_location_t existing_loc;
    if (fat_find_entry_in_dir(fs_info, parent_cluster, filename, &existing_entry, &existing_loc))
    {
        if (existing_entry.attributes & FAT_ATTR_DIRECTORY)
            return -1; // Can't overwrite a directory

        // Mark entry as deleted
        uint8_t dir_sector_buffer[512];
        if (!sd_spi_read_block(existing_loc.sector, dir_sector_buffer))
            return -1;

        dir_sector_buffer[existing_loc.offset] = 0xE5; // Deleted marker
        if (!sd_spi_write_block(existing_loc.sector, dir_sector_buffer))
            return -1;
    }

    // Allocate clusters for file data
    uint32_t clusters_needed = (size + fs_info->bytes_per_sector * fs_info->sectors_per_cluster - 1) /
                               (fs_info->bytes_per_sector * fs_info->sectors_per_cluster);

    uint32_t *cluster_chain = (uint32_t *)malloc(clusters_needed * sizeof(uint32_t));
    if (!cluster_chain)
        return -1;

    // Find free clusters
    uint32_t first_cluster = fat_find_free_cluster(fs_info);
    if (first_cluster == 0)
    {
        free(cluster_chain);
        return -1;
    }

    cluster_chain[0] = first_cluster;
    if (!fat_write_fat_entry(fs_info, first_cluster, 0x0FFFFFF8))
    {
        free(cluster_chain);
        return -1;
    }

    uint32_t current_cluster = first_cluster;
    for (uint32_t i = 1; i < clusters_needed; i++)
    {
        uint32_t next_cluster = fat_find_free_cluster(fs_info);
        if (next_cluster == 0 || next_cluster == current_cluster)
        {
            free(cluster_chain);
            return -1;
        }

        cluster_chain[i] = next_cluster;

        // Mark the new cluster allocated before the next free-cluster search.
        if (!fat_write_fat_entry(fs_info, next_cluster, 0x0FFFFFF8))
        {
            free(cluster_chain);
            return -1;
        }

        if (!fat_write_fat_entry(fs_info, current_cluster, next_cluster))
        {
            free(cluster_chain);
            return -1;
        }

        current_cluster = next_cluster;
    }
    if (!fat_cache_flush())
    {
        free(cluster_chain);
        return -1;
    }

    // Write file data
    uint32_t bytes_written = 0;
    uint8_t sector_buffer[512];
    uint32_t chain_index = 0;

    while (bytes_written < size && chain_index < clusters_needed)
    {
        current_cluster = cluster_chain[chain_index++];
        uint32_t cluster_sector = fat_cluster_to_sector(fs_info, current_cluster);
        uint32_t bytes_in_cluster = (size - bytes_written > fs_info->bytes_per_sector * fs_info->sectors_per_cluster) ? (fs_info->bytes_per_sector * fs_info->sectors_per_cluster) : (size - bytes_written);

        for (int s = 0; s < fs_info->sectors_per_cluster && bytes_written < size; s++)
        {
            uint32_t bytes_in_sector = (bytes_in_cluster > 512) ? 512 : bytes_in_cluster;

            memset(sector_buffer, 0, 512);
            memcpy(sector_buffer, data + bytes_written, bytes_in_sector);

            if (!sd_spi_write_block(cluster_sector + s, sector_buffer))
                return -1;

            bytes_written += bytes_in_sector;
            bytes_in_cluster -= bytes_in_sector;
        }
    }

    free(cluster_chain);

    // Create file entry
    fat_dir_entry_t file_entry;
    fat_create_dir_entry(&file_entry, filename, FAT_ATTR_ARCHIVE, first_cluster, size);

    // Add entry to parent directory
    if (!fat_add_entry_to_dir(fs_info, parent_cluster, &file_entry))
        return -1;

    return size;
}

int sd_spi_append_file(const fs_info_t *fs_info, const char *filepath, const uint8_t *data, uint32_t size)
{
    if (!fs_info || !filepath || !data || size == 0)
        return -1;

    // Find parent directory and filename
    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
    {
        // Directory does not exist yet, so append degenerates to a normal create/write.
        int bytes_written = sd_spi_write_file(fs_info, filepath, data, size);
        return (bytes_written == (int)size) ? (int)size : -1;
    }

    fat_dir_entry_t existing_entry;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &existing_entry, NULL))
    {
        // File does not exist yet.
        int bytes_written = sd_spi_write_file(fs_info, filepath, data, size);
        return (bytes_written == (int)size) ? (int)size : -1;
    }

    if (existing_entry.attributes & FAT_ATTR_DIRECTORY)
        return -1;

    uint32_t existing_size = existing_entry.file_size;
    if (existing_size > 0xFFFFFFFFu - size)
        return -1;

    uint32_t combined_size = existing_size + size;
    uint8_t *combined = (uint8_t *)malloc(combined_size);
    if (!combined)
        return -1;

    if (existing_size > 0)
    {
        int bytes_read = sd_spi_read_file(fs_info, filepath, combined, existing_size);
        if (bytes_read != (int)existing_size)
        {
            free(combined);
            return -1;
        }
    }

    memcpy(combined + existing_size, data, size);

    int bytes_written = sd_spi_write_file(fs_info, filepath, combined, combined_size);
    free(combined);

    if (bytes_written != (int)combined_size)
        return -1;

    return (int)size;
}

/**
 * @brief Efficiently append data to an existing file without reading the entire file
 *
 * This function implements true incremental append by:
 * 1. Finding the file's last cluster and position
 * 2. Writing new data starting from the end position
 * 3. Allocating new clusters only as needed
 * 4. Updating the directory entry with the new file size
 *
 * This avoids the expensive read-modify-write cycle of sd_spi_append_file().
 */
int sd_spi_append_file_fast(const fs_info_t *fs_info, const char *filepath, const uint8_t *data, uint32_t size)
{
    if (!fs_info || !filepath || !data || size == 0)
        return -1;

    // Parse filepath into parent directory and filename
    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    // Navigate to parent directory
    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
    {
        // Directory does not exist - fall back to write (which creates directories)
        return sd_spi_write_file(fs_info, filepath, data, size);
    }

    // Find file entry
    fat_dir_entry_t file_entry;
    dir_entry_location_t entry_location;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &file_entry, &entry_location))
    {
        // File does not exist - create it
        return sd_spi_write_file(fs_info, filepath, data, size);
    }

    if (file_entry.attributes & FAT_ATTR_DIRECTORY)
        return -1; // Cannot append to a directory

    // Get current file info
    uint32_t old_size = file_entry.file_size;
    uint32_t new_size = old_size + size;

    if (new_size < old_size) // Overflow check
        return -1;

    uint32_t first_cluster = fat_get_cluster(&file_entry, fs_info->type);

    // Special case: empty file (first_cluster == 0)
    if (first_cluster == 0 || first_cluster >= 0x0FFFFFF0)
    {
        // File exists but has no allocated clusters - treat as new file
        return sd_spi_write_file(fs_info, filepath, data, size);
    }

    // Calculate cluster and sector positions
    uint32_t bytes_per_cluster = fs_info->bytes_per_sector * fs_info->sectors_per_cluster;
    uint32_t old_clusters = (old_size + bytes_per_cluster - 1) / bytes_per_cluster;
    uint32_t new_clusters = (new_size + bytes_per_cluster - 1) / bytes_per_cluster;
    uint32_t additional_clusters = new_clusters - old_clusters;

    // Traverse to last cluster
    uint32_t last_cluster = first_cluster;
    uint32_t prev_cluster = last_cluster;

    for (uint32_t i = 1; i < old_clusters; i++)
    {
        prev_cluster = last_cluster;
        last_cluster = fat_read_fat_entry(fs_info, last_cluster);
        if (last_cluster >= 0x0FFFFFF0)
        {
            // Chain broken or end marker - something is wrong
            return -1;
        }
    }

    // Calculate position within last cluster
    uint32_t bytes_in_last_cluster = old_size % bytes_per_cluster;
    if (bytes_in_last_cluster == 0 && old_size > 0)
        bytes_in_last_cluster = bytes_per_cluster; // Last cluster is full

    uint32_t sector_in_cluster = bytes_in_last_cluster / fs_info->bytes_per_sector;
    uint32_t byte_in_sector = bytes_in_last_cluster % fs_info->bytes_per_sector;

    // Write data
    uint32_t bytes_written = 0;
    uint32_t current_cluster = last_cluster;
    uint8_t sector_buffer[512];

    // Handle partial sector at the end of existing file
    if (byte_in_sector > 0)
    {
        uint32_t sector_addr = fat_cluster_to_sector(fs_info, current_cluster) + sector_in_cluster;

        // Read existing sector content
        if (!sd_spi_read_block(sector_addr, sector_buffer))
            return -1;

        // Append new data to partially filled sector
        uint32_t bytes_to_write = 512 - byte_in_sector;
        if (bytes_to_write > size)
            bytes_to_write = size;

        memcpy(sector_buffer + byte_in_sector, data, bytes_to_write);

        // Write back sector
        if (!sd_spi_write_block(sector_addr, sector_buffer))
            return -1;

        bytes_written += bytes_to_write;
        sector_in_cluster++;
    }

    // Write remaining full sectors in the last cluster
    while (bytes_written < size && sector_in_cluster < fs_info->sectors_per_cluster)
    {
        uint32_t sector_addr = fat_cluster_to_sector(fs_info, current_cluster) + sector_in_cluster;
        uint32_t bytes_to_write = size - bytes_written;
        if (bytes_to_write > 512)
            bytes_to_write = 512;

        memset(sector_buffer, 0, 512);
        memcpy(sector_buffer, data + bytes_written, bytes_to_write);

        if (!sd_spi_write_block(sector_addr, sector_buffer))
            return -1;

        bytes_written += bytes_to_write;
        sector_in_cluster++;
    }

    // Allocate and write additional clusters if needed
    for (uint32_t i = 0; i < additional_clusters && bytes_written < size; i++)
    {
        uint32_t new_cluster = fat_find_free_cluster(fs_info);
        if (new_cluster == 0 || new_cluster == current_cluster)
            return -1; // No free space

        // Mark the cluster allocated first so the allocator cannot hand it out again.
        if (!fat_write_fat_entry(fs_info, new_cluster, 0x0FFFFFF8))
            return -1;

        // Link previous cluster to new cluster
        if (!fat_write_fat_entry(fs_info, current_cluster, new_cluster))
            return -1;

        current_cluster = new_cluster;

        // Write data to new cluster
        for (uint32_t s = 0; s < fs_info->sectors_per_cluster && bytes_written < size; s++)
        {
            uint32_t sector_addr = fat_cluster_to_sector(fs_info, current_cluster) + s;
            uint32_t bytes_to_write = size - bytes_written;
            if (bytes_to_write > 512)
                bytes_to_write = 512;

            memset(sector_buffer, 0, 512);
            memcpy(sector_buffer, data + bytes_written, bytes_to_write);

            if (!sd_spi_write_block(sector_addr, sector_buffer))
                return -1;

            bytes_written += bytes_to_write;
        }
    }

    // Flush FAT cache
    if (!fat_cache_flush())
        return -1;

    // Update directory entry with new file size
    uint8_t dir_sector_buffer[512];
    if (!sd_spi_read_block(entry_location.sector, dir_sector_buffer))
        return -1;

    fat_dir_entry_t *dir_entry = (fat_dir_entry_t *)(dir_sector_buffer + entry_location.offset);
    dir_entry->file_size = new_size;

    if (!sd_spi_write_block(entry_location.sector, dir_sector_buffer))
        return -1;

    return (int)size;
}

int sd_spi_read_file(const fs_info_t *fs_info, const char *filepath, uint8_t *buffer, uint32_t max_size)
{
    if (!fs_info || !filepath || !buffer || max_size == 0)
        return -1;

    // Find parent directory and filename
    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    // Navigate to parent directory
    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        return -1;

    // Find file entry
    fat_dir_entry_t file_entry;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &file_entry, NULL))
        return -1;

    if (file_entry.attributes & FAT_ATTR_DIRECTORY)
        return -1; // It's a directory, not a file

    // Read file data
    uint32_t bytes_to_read = (file_entry.file_size < max_size) ? file_entry.file_size : max_size;
    uint32_t bytes_read = 0;
    uint32_t current_cluster = fat_get_cluster(&file_entry, fs_info->type);
    uint8_t sector_buffer[512];

    while (bytes_read < bytes_to_read && current_cluster < 0x0FFFFFF0)
    {
        uint32_t cluster_sector = fat_cluster_to_sector(fs_info, current_cluster);

        for (int s = 0; s < fs_info->sectors_per_cluster && bytes_read < bytes_to_read; s++)
        {
            if (!sd_spi_read_block(cluster_sector + s, sector_buffer))
                return -1;

            uint32_t bytes_in_sector = (bytes_to_read - bytes_read > 512) ? 512 : (bytes_to_read - bytes_read);
            memcpy(buffer + bytes_read, sector_buffer, bytes_in_sector);

            bytes_read += bytes_in_sector;
        }

        // Move to next cluster
        current_cluster = fat_read_fat_entry(fs_info, current_cluster);
    }

    return bytes_read;
}

bool sd_spi_delete_file(const fs_info_t *fs_info, const char *filepath)
{
    if (!fs_info || !filepath)
        return false;

    // Find parent directory and filename
    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    // Navigate to parent directory
    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        return false;

    // Find file entry
    fat_dir_entry_t file_entry;
    dir_entry_location_t entry_loc;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &file_entry, &entry_loc))
        return false;

    if (file_entry.attributes & FAT_ATTR_DIRECTORY)
        return false; // Can't delete directories this way

    // Mark entry as deleted
    uint8_t dir_sector_buffer[512];
    if (!sd_spi_read_block(entry_loc.sector, dir_sector_buffer))
        return false;

    dir_sector_buffer[entry_loc.offset] = 0xE5; // Deleted marker
    if (!sd_spi_write_block(entry_loc.sector, dir_sector_buffer))
        return false;

    // Mark clusters as free (optional - they're just marked as unused when overwritten)
    return true;
}

bool sd_spi_get_file_size(const fs_info_t *fs_info, const char *filepath, uint32_t *size_out)
{
    if (!fs_info || !filepath || !size_out)
        return false;

    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        return false;

    fat_dir_entry_t file_entry;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &file_entry, NULL))
        return false;

    if (file_entry.attributes & FAT_ATTR_DIRECTORY)
        return false;

    *size_out = file_entry.file_size;
    return true;
}

int sd_spi_read_file_chunk(const fs_info_t *fs_info, const char *filepath, uint32_t offset,
                           uint8_t *buffer, uint32_t max_size)
{
    if (!fs_info || !filepath || !buffer || max_size == 0)
        return -1;

    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        return -1;

    fat_dir_entry_t file_entry;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &file_entry, NULL))
        return -1;

    if (file_entry.attributes & FAT_ATTR_DIRECTORY)
        return -1;

    if (offset >= file_entry.file_size)
        return 0;

    uint32_t bytes_to_read = file_entry.file_size - offset;
    if (bytes_to_read > max_size)
        bytes_to_read = max_size;

    uint32_t bytes_per_cluster = fs_info->bytes_per_sector * fs_info->sectors_per_cluster;
    uint32_t cluster_skip = offset / bytes_per_cluster;
    uint32_t cluster_offset = offset % bytes_per_cluster;

    uint32_t current_cluster = fat_get_cluster(&file_entry, fs_info->type);
    if (current_cluster == 0 || current_cluster >= 0x0FFFFFF0)
        return 0;

    for (uint32_t i = 0; i < cluster_skip; i++)
    {
        current_cluster = fat_read_fat_entry(fs_info, current_cluster);
        if (current_cluster >= 0x0FFFFFF0)
            return -1;
    }

    uint8_t sector_buffer[512];
    uint32_t bytes_read = 0;

    while (bytes_read < bytes_to_read && current_cluster < 0x0FFFFFF0)
    {
        uint32_t cluster_sector = fat_cluster_to_sector(fs_info, current_cluster);
        uint32_t start_sector = cluster_offset / fs_info->bytes_per_sector;
        uint32_t start_byte = cluster_offset % fs_info->bytes_per_sector;

        for (uint32_t s = start_sector; s < fs_info->sectors_per_cluster && bytes_read < bytes_to_read; s++)
        {
            if (!sd_spi_read_block(cluster_sector + s, sector_buffer))
                return -1;

            uint32_t available = fs_info->bytes_per_sector - start_byte;
            uint32_t copy_n = bytes_to_read - bytes_read;
            if (copy_n > available)
                copy_n = available;

            memcpy(buffer + bytes_read, sector_buffer + start_byte, copy_n);
            bytes_read += copy_n;
            start_byte = 0;
        }

        cluster_offset = 0;
        current_cluster = fat_read_fat_entry(fs_info, current_cluster);
    }

    return (int)bytes_read;
}

// ============================================================================
// OPTIMIZED FILE STREAMING API (for fast downloads)
// ============================================================================

bool sd_spi_read_multiple_blocks(uint32_t start_block, uint32_t block_count, uint8_t *buffer)
{
    if (!card_initialized || !buffer || block_count == 0)
        return false;

    // For efficiency, read blocks one at a time but could be optimized with CMD18
    for (uint32_t i = 0; i < block_count; i++)
    {
        if (!sd_spi_read_block(start_block + i, buffer + (i * 512)))
            return false;
    }
    return true;
}

bool sd_spi_open_file(const fs_info_t *fs_info, const char *filepath, file_handle_t *handle)
{
    if (!fs_info || !filepath || !handle)
        return false;

    const char *last_slash = strrchr(filepath, '/');
    char parent_path[512];
    const char *filename;

    if (last_slash)
    {
        int parent_len = last_slash - filepath;
        strncpy(parent_path, filepath, parent_len);
        parent_path[parent_len] = 0;
        filename = last_slash + 1;
    }
    else
    {
        parent_path[0] = 0;
        filename = filepath;
    }

    uint32_t parent_cluster = (fs_info->type == 3) ? fs_info->root_dir_cluster : 0;
    if (parent_path[0] && !fat_navigate_to_dir(fs_info, parent_path, &parent_cluster))
        return false;

    fat_dir_entry_t file_entry;
    if (!fat_find_entry_in_dir(fs_info, parent_cluster, filename, &file_entry, NULL))
        return false;

    if (file_entry.attributes & FAT_ATTR_DIRECTORY)
        return false;

    // Initialize handle
    handle->fs_info = fs_info;
    handle->file_size = file_entry.file_size;
    handle->current_cluster = fat_get_cluster(&file_entry, fs_info->type);
    handle->bytes_per_cluster = fs_info->bytes_per_sector * fs_info->sectors_per_cluster;
    handle->position = 0;
    handle->is_open = true;

    return true;
}

int sd_spi_read_from_handle(file_handle_t *handle, uint8_t *buffer, uint32_t size)
{
    if (!handle || !handle->is_open || !buffer || size == 0)
        return -1;

    if (handle->position >= handle->file_size)
        return 0; // EOF

    uint32_t bytes_remaining = handle->file_size - handle->position;
    uint32_t bytes_to_read = (size < bytes_remaining) ? size : bytes_remaining;
    uint32_t bytes_read = 0;

    while (bytes_read < bytes_to_read && handle->current_cluster < 0x0FFFFFF0)
    {
        // Calculate position within current cluster
        uint32_t cluster_position = handle->position % handle->bytes_per_cluster;
        uint32_t bytes_left_in_cluster = handle->bytes_per_cluster - cluster_position;
        uint32_t bytes_to_read_cluster = (bytes_to_read - bytes_read < bytes_left_in_cluster)
                                             ? (bytes_to_read - bytes_read)
                                             : bytes_left_in_cluster;

        // Calculate sector within cluster
        uint32_t sector_in_cluster = cluster_position / handle->fs_info->bytes_per_sector;
        uint32_t byte_in_sector = cluster_position % handle->fs_info->bytes_per_sector;
        uint32_t cluster_sector = fat_cluster_to_sector(handle->fs_info, handle->current_cluster);

        // Read data sector by sector
        while (bytes_to_read_cluster > 0 && sector_in_cluster < handle->fs_info->sectors_per_cluster)
        {
            uint8_t sector_buffer[512];
            if (!sd_spi_read_block(cluster_sector + sector_in_cluster, sector_buffer))
                return -1;

            uint32_t bytes_available = handle->fs_info->bytes_per_sector - byte_in_sector;
            uint32_t bytes_to_copy = (bytes_to_read_cluster < bytes_available)
                                         ? bytes_to_read_cluster
                                         : bytes_available;

            memcpy(buffer + bytes_read, sector_buffer + byte_in_sector, bytes_to_copy);
            bytes_read += bytes_to_copy;
            bytes_to_read_cluster -= bytes_to_copy;
            handle->position += bytes_to_copy;
            byte_in_sector = 0; // Next reads start at beginning of sector
            sector_in_cluster++;
        }

        // Advance to the next cluster whenever the current one has been fully consumed.
        // This must also happen when the caller's requested read size ends exactly on a
        // cluster boundary; otherwise the next read call starts over at the same cluster.
        if (handle->position < handle->file_size &&
            handle->position % handle->bytes_per_cluster == 0)
        {
            handle->current_cluster = fat_read_fat_entry(handle->fs_info, handle->current_cluster);
        }
    }

    return (int)bytes_read;
}

void sd_spi_close_file(file_handle_t *handle)
{
    if (handle)
    {
        handle->is_open = false;
    }
}

int sd_spi_read_directory(const fs_info_t *fs_info, dirent_t *entries, int max_entries)
{
    if (!card_initialized || !fs_info || !entries || max_entries <= 0)
        return -1;

    // Handle different filesystem types
    if (fs_info->type == 4) // exFAT
    {
        // exFAT directory reading not implemented for this basic version
        return 0;
    }

    // Read FAT directory from root directory sector
    uint8_t sector_buffer[512];
    int entry_count = 0;
    uint32_t root_sectors;
    uint32_t first_sector;

    // Buffer for accumulating long filename entries
    char lfn_buffer[256];
    int lfn_pos = 0;
    bool has_lfn = false;

    if (fs_info->type == 3) // FAT32
    {
        // For FAT32, root directory is a cluster chain
        // Calculate sector from cluster number: sector = data_start + (cluster - 2) * sectors_per_cluster
        first_sector = fs_info->data_start_sector + ((fs_info->root_dir_cluster - 2) * fs_info->sectors_per_cluster);
        // For simplicity, just read one cluster worth of sectors
        root_sectors = fs_info->sectors_per_cluster;
    }
    else // FAT12/FAT16
    {
        first_sector = fs_info->root_dir_sector;
        root_sectors = (fs_info->root_dir_entries * 32 + fs_info->bytes_per_sector - 1) / fs_info->bytes_per_sector;
    }

    for (uint32_t sector_idx = 0; sector_idx < root_sectors && entry_count < max_entries; sector_idx++)
    {
        if (!sd_spi_read_block(first_sector + sector_idx, sector_buffer))
            continue;

        // Parse directory entries (32 bytes each)
        for (int entry_idx = 0; entry_idx < 16 && entry_count < max_entries; entry_idx++)
        {
            const uint8_t *entry = &sector_buffer[entry_idx * 32];

            // Check for end of directory
            if (entry[0] == 0x00)
            {
                return entry_count;
            }

            // Skip deleted entries
            if (entry[0] == 0xE5)
                continue;

            // Check for long filename entry (attribute 0x0F)
            if (entry[11] == 0x0F)
            {
                // This is an LFN entry
                uint8_t sequence = entry[0] & 0x1F;    // Sequence number (1-20)
                bool is_last = (entry[0] & 0x40) != 0; // Last LFN entry

                if (is_last)
                {
                    // Start of a new LFN sequence (entries are in reverse order)
                    lfn_pos = 0;
                    has_lfn = true;
                    memset(lfn_buffer, 0, sizeof(lfn_buffer));
                }

                if (has_lfn)
                {
                    // Calculate position in the final string
                    // LFN entries are stored in reverse order, each holds 13 chars
                    int base_pos = (sequence - 1) * 13;
                    int temp_pos = base_pos;
                    extract_lfn_chars(entry, lfn_buffer, &temp_pos);
                    if (temp_pos > lfn_pos)
                        lfn_pos = temp_pos;
                }
                continue;
            }

            // Skip volume labels
            if (entry[11] == 0x08)
                continue;

            // Skip dot and dotdot entries if desired, or include them
            if (entry[0] == '.')
                continue;

            // Parse filename - use LFN if available, otherwise use 8.3 name
            if (has_lfn && lfn_pos > 0)
            {
                // Use the long filename
                lfn_buffer[lfn_pos] = '\0';
                strncpy(entries[entry_count].filename, lfn_buffer, 255);
                entries[entry_count].filename[255] = '\0';
                has_lfn = false;
                lfn_pos = 0;
            }
            else
            {
                // Use short 8.3 filename
                parse_fat_filename(entry, entries[entry_count].filename);
            }

            // Parse attributes
            entries[entry_count].attributes = entry[11];

            // Parse file size
            entries[entry_count].file_size = read_u32_le(entry, 28);

            // Parse date and time
            entries[entry_count].date = parse_fat_date(entry, 16);
            entries[entry_count].time = parse_fat_time(entry, 14);

            entry_count++;
        }
    }

    return entry_count;
}

int sd_spi_read_directory_path(const fs_info_t *fs_info, const char *path, dirent_t *entries, int max_entries)
{
    if (!card_initialized || !fs_info || !entries || max_entries <= 0)
        return -1;

    if (!path || path[0] == 0 || (path[0] == '/' && path[1] == 0))
        return sd_spi_read_directory(fs_info, entries, max_entries);

    if (fs_info->type == 4)
        return -1;

    uint32_t dir_cluster = 0;
    if (!fat_navigate_to_dir(fs_info, path, &dir_cluster))
        return -1;

    if (fs_info->type != 3 && dir_cluster == 0)
        return sd_spi_read_directory(fs_info, entries, max_entries);

    int entry_count = 0;
    uint8_t sector_buffer[512];
    uint32_t current_cluster = dir_cluster;

    while (current_cluster < 0x0FFFFFF0 && entry_count < max_entries)
    {
        uint32_t sector = fat_cluster_to_sector(fs_info, current_cluster);

        for (uint32_t s = 0; s < fs_info->sectors_per_cluster && entry_count < max_entries; s++)
        {
            if (!sd_spi_read_block(sector + s, sector_buffer))
                continue;

            for (int offset = 0; offset < 512 && entry_count < max_entries; offset += 32)
            {
                fat_dir_entry_t *dir_entry = (fat_dir_entry_t *)(sector_buffer + offset);

                if (dir_entry->filename[0] == 0x00)
                    return entry_count;

                if (dir_entry->filename[0] == 0xE5 || dir_entry->attributes == FAT_ATTR_LONG_NAME ||
                    (dir_entry->attributes & FAT_ATTR_VOLUME_ID))
                    continue;

                dirent_t *out = &entries[entry_count++];
                memset(out, 0, sizeof(dirent_t));

                int pos = 0;
                for (int i = 0; i < 8 && dir_entry->filename[i] != ' ' && pos < (int)sizeof(out->filename) - 1; i++)
                    out->filename[pos++] = (char)dir_entry->filename[i];

                if (dir_entry->filename[8] != ' ' && pos < (int)sizeof(out->filename) - 1)
                {
                    out->filename[pos++] = '.';
                    for (int i = 8; i < 11 && dir_entry->filename[i] != ' ' && pos < (int)sizeof(out->filename) - 1; i++)
                        out->filename[pos++] = (char)dir_entry->filename[i];
                }

                out->filename[pos] = 0;
                out->attributes = dir_entry->attributes;
                out->file_size = dir_entry->file_size;
                out->date = dir_entry->write_date;
                out->time = dir_entry->write_time;
            }
        }

        current_cluster = fat_read_fat_entry(fs_info, current_cluster);
    }

    return entry_count;
}
// Simplified file operations for testing
// NOTE: These are basic implementations for testing purposes only
// They do not handle FAT table updates or complex error cases

int sd_spi_write_file_old_test(const fs_info_t *fs_info, const char *filename, const uint8_t *data, uint32_t size)
{
    // DEPRECATED: Old test implementation - kept for reference
    // For simplicity, this test implementation writes to a temporary test area
    // NOT a full FAT filesystem implementation - just for speed testing
    if (!card_initialized || !fs_info || !filename || !data || size == 0)
        return -1;

    // Use a test sector range (sectors 100-200 as temp storage)
    // This avoids corrupting the filesystem
    uint32_t test_sector_start = 100;
    uint32_t sectors_needed = (size + 511) / 512;

    if (sectors_needed > 100)
        return -1; // Too large for test area

    uint32_t bytes_written = 0;
    uint8_t sector_buffer[512];

    for (uint32_t i = 0; i < sectors_needed; i++)
    {
        uint32_t bytes_to_write = (size - bytes_written > 512) ? 512 : (size - bytes_written);

        memset(sector_buffer, 0, 512);
        memcpy(sector_buffer, data + bytes_written, bytes_to_write);

        if (!sd_spi_write_block(test_sector_start + i, sector_buffer))
            return -1;

        bytes_written += bytes_to_write;
    }

    return bytes_written;
}

int sd_spi_read_file_old_test(const fs_info_t *fs_info, const char *filename, uint8_t *buffer, uint32_t max_size)
{
    // DEPRECATED: Old test implementation - kept for reference
    // Simplified test implementation - reads from test sector area
    if (!card_initialized || !fs_info || !filename || !buffer || max_size == 0)
        return -1;

    uint32_t test_sector_start = 100;
    uint32_t sectors_to_read = (max_size + 511) / 512;

    if (sectors_to_read > 100)
        sectors_to_read = 100;

    uint32_t bytes_read = 0;
    uint8_t sector_buffer[512];

    for (uint32_t i = 0; i < sectors_to_read && bytes_read < max_size; i++)
    {
        if (!sd_spi_read_block(test_sector_start + i, sector_buffer))
            return -1;

        uint32_t bytes_to_copy = (max_size - bytes_read > 512) ? 512 : (max_size - bytes_read);
        memcpy(buffer + bytes_read, sector_buffer, bytes_to_copy);
        bytes_read += bytes_to_copy;
    }

    return bytes_read;
}

bool sd_spi_delete_file_old_test(const fs_info_t *fs_info, const char *filename)
{
    // DEPRECATED: Old test implementation - kept for reference
    // Simplified test implementation - just clears test sectors
    if (!card_initialized || !fs_info || !filename)
        return false;

    uint8_t zero_buffer[512];
    memset(zero_buffer, 0, 512);

    // Clear first 10 test sectors
    for (int i = 0; i < 10; i++)
    {
        sd_spi_write_block(100 + i, zero_buffer);
    }

    return true;
}