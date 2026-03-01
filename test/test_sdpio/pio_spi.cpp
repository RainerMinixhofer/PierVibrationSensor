#include "pio_spi.h"
#include "spi.pio.h"
#include "hardware/clocks.h"
#include <Arduino.h>

static uint pio_program_offset = 0;
static bool pio_program_loaded = false;

void pio_spi_init_instance(PIO pio, uint sm, uint pin_sck, uint pin_mosi, uint pin_miso, float freq_hz)
{
    // Restart state machine and clear FIFOs if already loaded
    if (pio_program_loaded)
    {
        pio_sm_set_enabled(pio, sm, false);
        pio_sm_clear_fifos(pio, sm);
        pio_sm_restart(pio, sm);
    }

    // Add PIO program (only once per PIO instance)
    if (!pio_program_loaded)
    {
        pio_program_offset = pio_add_program(pio, &spi_cpha0_program);
        pio_program_loaded = true;
        Serial.print("[PIO] Program loaded at offset ");
        Serial.println(pio_program_offset);
    }

    // Calculate clock divider for desired frequency
    // PIO runs 4 cycles per SPI bit (out + in with side-set delays)
    float clk_div = (float)clock_get_hz(clk_sys) / (freq_hz * 4.0f);

    Serial.print("[PIO] Initializing SPI - SCK:");
    Serial.print(pin_sck);
    Serial.print(" MOSI:");
    Serial.print(pin_mosi);
    Serial.print(" MISO:");
    Serial.print(pin_miso);
    Serial.print(" Freq:");
    Serial.print(freq_hz);
    Serial.print("Hz ClkDiv:");
    Serial.println(clk_div);

    // Initialize PIO SPI
    pio_spi_init(pio, sm, pio_program_offset, 8, clk_div, false, false,
                 pin_sck, pin_mosi, pin_miso);

    // Clear FIFOs after initialization
    pio_sm_clear_fifos(pio, sm);

    Serial.println("[PIO] State machine initialized and FIFOs cleared");

    // Enable the state machine
    pio_sm_set_enabled(pio, sm, true);

    Serial.println("[PIO] State machine enabled");

    if (pio_sm_is_enabled(pio, sm))
    {
        Serial.println("[PIO] SM is enabled");
    }
    else
    {
        Serial.println("[PIO] SM is not enabled");
    }
}

void pio_spi_write8_blocking(PIO pio, uint sm, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        while (pio_sm_is_tx_fifo_full(pio, sm))
            ;
        pio_sm_put(pio, sm, (uint32_t)src[i]);
    }

    // Drain RX FIFO
    for (size_t i = 0; i < len; ++i)
    {
        while (pio_sm_is_rx_fifo_empty(pio, sm))
            ;
        (void)pio_sm_get(pio, sm);
    }
}

void pio_spi_read8_blocking(PIO pio, uint sm, uint8_t *dst, size_t len)
{
    const uint8_t dummy = 0xFF;
    for (size_t i = 0; i < len; ++i)
    {
        while (pio_sm_is_tx_fifo_full(pio, sm))
            ;
        pio_sm_put(pio, sm, (uint32_t)dummy);
    }

    for (size_t i = 0; i < len; ++i)
    {
        while (pio_sm_is_rx_fifo_empty(pio, sm))
            ;
        dst[i] = (uint8_t)pio_sm_get(pio, sm);
    }
}

void pio_spi_write8_read8_blocking(PIO pio, uint sm, const uint8_t *src, uint8_t *dst, size_t len)
{
    for (size_t i = 0; i < len; ++i)
    {
        while (pio_sm_is_tx_fifo_full(pio, sm))
            ;
        pio_sm_put(pio, sm, (uint32_t)src[i]);
    }

    for (size_t i = 0; i < len; ++i)
    {
        while (pio_sm_is_rx_fifo_empty(pio, sm))
            ;
        dst[i] = (uint8_t)pio_sm_get(pio, sm);
    }
}
