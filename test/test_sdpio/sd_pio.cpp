#include "sd_pio.h"
#include "pio_spi.h"
#include <Arduino.h>
#include <string.h>
#include "hardware/gpio.h"
#include "hardware/structs/sio.h"

SDCardPIO::SDCardPIO(PIO pio, uint sm, uint pin_cs, uint pin_sck, uint pin_mosi, uint pin_miso)
    : pio(pio), sm(sm), pinCS(pin_cs), pinSCK(pin_sck), pinMOSI(pin_mosi), pinMISO(pin_miso), cardType(0)
{
}

void SDCardPIO::csLow()
{
    delayMicroseconds(2);
    digitalWrite(pinCS, LOW);
    delayMicroseconds(2);
}

void SDCardPIO::csHigh()
{
    // Wait for any pending PIO transfers to complete
    while (!pio_sm_is_tx_fifo_empty(pio, sm))
        ;
    delayMicroseconds(2);
    digitalWrite(pinCS, HIGH);
    delayMicroseconds(20);
}

uint8_t SDCardPIO::xchgByte(uint8_t data)
{
    uint8_t result;
    pio_spi_write8_read8_blocking(pio, sm, &data, &result, 1);
    return result;
}

void SDCardPIO::sendBytes(const uint8_t *data, uint32_t len)
{
    pio_spi_write8_blocking(pio, sm, data, len);
}

void SDCardPIO::recvBytes(uint8_t *data, uint32_t len)
{
    pio_spi_read8_blocking(pio, sm, data, len);
}

bool SDCardPIO::waitReady(uint32_t timeout_ms)
{
    uint32_t start = millis();
    uint8_t d;
    do
    {
        d = xchgByte(0xFF);
        if (d == 0xFF)
            return true;
    } while (millis() - start < timeout_ms);
    return false;
}

uint8_t SDCardPIO::sendCommand(uint8_t cmd, uint32_t arg)
{
    if (cmd != CMD0 && cmd != CMD58 && cmd != CMD9 && cmd != CMD10 && cmd != CMD17 && cmd != CMD24 && cmd != CMD18 && cmd != CMD25 && cmd != CMD12)
    {
        if (!waitReady())
            return 0xFF;
    }

    // Send ACMD<n> as CMD55 + CMD<n>
    if (cmd & 0x80)
    {
        cmd &= 0x7F;
        uint8_t res = sendCommand(CMD55, 0);
        if (res > 1)
            return res;
    }

    // Deselect then select card
    csHigh();
    delayMicroseconds(10);
    xchgByte(0xFF);
    csLow();
    delayMicroseconds(10);
    xchgByte(0xFF);

    // Send command packet
    uint8_t buf[6];
    buf[0] = 0x40 | cmd;
    buf[1] = (uint8_t)(arg >> 24);
    buf[2] = (uint8_t)(arg >> 16);
    buf[3] = (uint8_t)(arg >> 8);
    buf[4] = (uint8_t)arg;

    // CRC
    uint8_t crc = 0xFF;
    if (cmd == CMD0)
        crc = 0x95; // Valid CRC for CMD0(0)
    if (cmd == CMD8)
        crc = 0x87; // Valid CRC for CMD8(0x1AA)
    if (cmd == CMD58)
        crc = 0xFD; // Valid CRC for CMD58(0)
    if (cmd == CMD9)
        crc = 0xAF; // Valid CRC for CMD9(0)
    if (cmd == CMD10)
        crc = 0x1B; // Valid CRC for CMD10(0)
    buf[5] = crc;

    sendBytes(buf, 6);

    // Debug: print command bytes
    if (cmd == CMD8 || cmd == CMD58 || cmd == CMD9 || cmd == CMD10)
    {
        Serial.print("[CMD] Sent: ");
        for (int i = 0; i < 6; i++)
        {
            if (buf[i] < 0x10)
                Serial.print("0");
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Wait for response
    uint8_t res;
    uint8_t n = 10;
    do
    {
        res = xchgByte(0xFF);
    } while ((res & 0x80) && --n);

    return res;
}

bool SDCardPIO::readData(uint8_t *buffer, uint32_t len)
{
    // Wait for start token with timeout
    uint32_t timeout = 1000; // Increased timeout
    uint8_t token;
    do
    {
        token = xchgByte(0xFF);
        if (token == START_BLOCK_TOKEN)
            break;
        delayMicroseconds(100);
    } while (--timeout);

    if (token != START_BLOCK_TOKEN)
    {
        Serial.print("[DATA] Token timeout, last token: 0x");
        Serial.println(token, HEX);
        return false;
    }

    // Read data
    recvBytes(buffer, len);

    // Discard CRC
    xchgByte(0xFF);
    xchgByte(0xFF);

    return true;
}

bool SDCardPIO::writeData(uint8_t token, const uint8_t *buffer)
{
    if (!waitReady())
        return false;

    // Send token
    uint8_t tk = token;
    sendBytes(&tk, 1);

    if (token != STOP_TRAN_TOKEN)
    {
        // Send data block
        sendBytes(buffer, 512);

        // Dummy CRC
        uint8_t dummy[2] = {0xFF, 0xFF};
        sendBytes(dummy, 2);

        // Check data response
        uint8_t resp = xchgByte(0xFF);
        if ((resp & 0x1F) != 0x05)
        {
            return false;
        }

        // Wait for write completion
        if (!waitReady(500))
            return false;
    }

    return true;
}

bool SDCardPIO::begin(float freq_hz)
{
    cardType = 0;

    Serial.println("[SD] Initializing pins...");

    // Initialize CS pin BEFORE PIO init
    pinMode(pinCS, OUTPUT);
    digitalWrite(pinCS, HIGH);
    delay(10); // Let CS stabilize

    Serial.println("[SD] Using PIO SPI mode");
    // Initialize PIO SPI at low speed first (400 kHz for initialization)
    pio_spi_init_instance(pio, sm, pinSCK, pinMOSI, pinMISO, 400000.0f);
    delay(50); // Let PIO stabilize and pins settle

    // 80 dummy clocks with CS high
    Serial.println("[SD] Sending dummy clocks...");
    for (int i = 0; i < 10; i++)
    {
        xchgByte(0xFF);
    }
    Serial.println("[SD] Dummy clocks complete - ready for CMD0");

    // Reset card
    Serial.println("[SD] Sending CMD0 (reset)...");
    uint8_t res;
    uint16_t timer = 1000;
    while ((res = sendCommand(CMD0, 0)) != R1_IDLE_STATE && --timer)
    {
        delay(1);
    }
    Serial.print("[SD] CMD0 response: 0x");
    Serial.println(res, HEX);
    if (res != R1_IDLE_STATE)
    {
        Serial.println("[SD] CMD0 failed - card not responding");
        csHigh();
        return false;
    }

    // Check voltage range
    Serial.println("[SD] Sending CMD8 (voltage check)...");
    res = sendCommand(CMD8, 0x1AA);
    Serial.print("[SD] CMD8 response: 0x");
    Serial.println(res, HEX);
    if (res == R1_IDLE_STATE)
    {
        // SDC V2
        uint8_t ocr[4];
        recvBytes(ocr, 4);

        if (ocr[2] == 0x01 && ocr[3] == 0xAA)
        {
            // Wait for leaving idle state
            timer = 1000;
            while (sendCommand(ACMD41 | 0x80, 0x40000000) && --timer)
            {
                delay(1);
            }

            if (timer && sendCommand(CMD58, 0) == 0)
            {
                recvBytes(ocr, 4);
                cardType = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
            }
        }
    }
    else
    {
        // SDC V1 or MMC
        uint8_t cmd;
        if (sendCommand(ACMD41 | 0x80, 0) <= 1)
        {
            cardType = CT_SD1;
            cmd = ACMD41 | 0x80;
        }
        else
        {
            cardType = CT_MMC;
            cmd = CMD1;
        }

        timer = 1000;
        while (sendCommand(cmd, 0) && --timer)
        {
            delay(1);
        }

        if (!timer || sendCommand(CMD16, 512) != 0)
        {
            cardType = 0;
        }
    }

    csHigh();

    if (cardType == 0)
    {
        return false;
    }

    // Reinitialize at higher speed
    pio_sm_set_enabled(pio, sm, false);
    pio_spi_init_instance(pio, sm, pinSCK, pinMOSI, pinMISO, freq_hz);

    return true;
}

void SDCardPIO::end()
{
    pio_sm_set_enabled(pio, sm, false);
}

bool SDCardPIO::readBlock(uint32_t block, uint8_t *buffer)
{
    if (!(cardType & CT_BLOCK))
        block *= 512;

    bool success = false;
    if (sendCommand(CMD17, block) == 0)
    {
        success = readData(buffer, 512);
    }

    csHigh();
    return success;
}

bool SDCardPIO::writeBlock(uint32_t block, const uint8_t *buffer)
{
    if (!(cardType & CT_BLOCK))
        block *= 512;

    bool success = false;
    if (sendCommand(CMD24, block) == 0)
    {
        success = writeData(START_BLOCK_TOKEN, buffer);
    }

    csHigh();
    return success;
}

bool SDCardPIO::readBlocks(uint32_t block, uint8_t *buffer, uint32_t count)
{
    if (!(cardType & CT_BLOCK))
        block *= 512;

    if (sendCommand(CMD18, block) != 0)
    {
        csHigh();
        return false;
    }

    bool success = true;
    for (uint32_t i = 0; i < count; i++)
    {
        if (!readData(buffer + i * 512, 512))
        {
            success = false;
            break;
        }
    }

    sendCommand(CMD12, 0); // STOP_TRANSMISSION
    csHigh();
    return success;
}

bool SDCardPIO::writeBlocks(uint32_t block, const uint8_t *buffer, uint32_t count)
{
    if (!(cardType & CT_BLOCK))
        block *= 512;

    if (cardType & CT_SD1)
    {
        sendCommand(ACMD41 | 0x80, 0);
    }

    if (sendCommand(CMD25, block) != 0)
    {
        csHigh();
        return false;
    }

    bool success = true;
    for (uint32_t i = 0; i < count; i++)
    {
        if (!writeData(0xFC, buffer + i * 512))
        { // Multiple block write token
            success = false;
            break;
        }
    }

    writeData(STOP_TRAN_TOKEN, nullptr);
    csHigh();
    return success;
}

uint64_t SDCardPIO::getCardSize() const
{
    uint8_t csd[16];
    if (!const_cast<SDCardPIO *>(this)->readCSD(csd))
    {
        return 0;
    }

    uint64_t capacity = 0;
    uint8_t csd_structure = csd[0] >> 6;

    if (csd_structure == 0)
    {
        // CSD Version 1.0 (SD V1, MMC)
        uint32_t c_size = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] & 0xC0) >> 6);
        uint8_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] & 0x80) >> 7);
        uint8_t read_bl_len = csd[5] & 0x0F;
        capacity = ((uint64_t)(c_size + 1)) << (c_size_mult + read_bl_len + 2);
    }
    else if (csd_structure == 1)
    {
        // CSD Version 2.0 (SD V2)
        uint32_t c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];
        capacity = ((uint64_t)(c_size + 1)) * 512 * 1024;
    }

    return capacity;
}

bool SDCardPIO::readOCR(uint8_t *ocr)
{
    bool success = false;
    uint8_t res = sendCommand(CMD58, 0);
    Serial.print("[SD] CMD58 response: 0x");
    Serial.println(res, HEX);
    if (res == 0)
    {
        recvBytes(ocr, 4);
        Serial.print("[SD] OCR read: ");
        for (int i = 0; i < 4; i++)
        {
            if (ocr[i] < 0x10)
                Serial.print("0");
            Serial.print(ocr[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        success = true;
    }
    csHigh();
    return success;
}

bool SDCardPIO::readCID(uint8_t *cid)
{
    bool success = false;
    if (sendCommand(CMD10, 0) == 0)
    {
        success = readData(cid, 16);
    }
    csHigh();
    return success;
}

bool SDCardPIO::readCSD(uint8_t *csd)
{
    bool success = false;
    if (sendCommand(CMD9, 0) == 0)
    {
        success = readData(csd, 16);
    }
    csHigh();
    return success;
}

bool SDCardPIO::readSSR(uint8_t *ssr)
{
    bool success = false;
    if (sendCommand(ACMD41 | 0x80, 0) == 0)
    {
        // Only for SD cards
        if (sendCommand(13 | 0x80, 0) == 0)
        { // ACMD13
            success = readData(ssr, 64);
        }
    }
    csHigh();
    return success;
}
