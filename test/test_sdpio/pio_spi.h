#ifndef PIO_SPI_H
#define PIO_SPI_H

#include <stdint.h>
#include "hardware/pio.h"

// Initialize PIO SPI
void pio_spi_init_instance(PIO pio, uint sm, uint pin_sck, uint pin_mosi, uint pin_miso, float freq_hz);

// Transfer functions
void pio_spi_write8_blocking(PIO pio, uint sm, const uint8_t *src, size_t len);
void pio_spi_read8_blocking(PIO pio, uint sm, uint8_t *dst, size_t len);
void pio_spi_write8_read8_blocking(PIO pio, uint sm, const uint8_t *src, uint8_t *dst, size_t len);

#endif // PIO_SPI_H
