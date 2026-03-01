#ifndef SD_PIO_H
#define SD_PIO_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

// SD Card Commands
#define CMD0 0    // GO_IDLE_STATE
#define CMD1 1    // SEND_OP_COND (MMC)
#define CMD8 8    // SEND_IF_COND
#define CMD9 9    // SEND_CSD
#define CMD10 10  // SEND_CID
#define CMD12 12  // STOP_TRANSMISSION
#define CMD13 13  // SEND_STATUS
#define CMD16 16  // SET_BLOCKLEN
#define CMD17 17  // READ_SINGLE_BLOCK
#define CMD18 18  // READ_MULTIPLE_BLOCK
#define CMD24 24  // WRITE_BLOCK
#define CMD25 25  // WRITE_MULTIPLE_BLOCK
#define CMD55 55  // APP_CMD
#define CMD58 58  // READ_OCR
#define ACMD41 41 // SD_SEND_OP_COND

// Response tokens
#define R1_READY_STATE 0x00
#define R1_IDLE_STATE 0x01
#define START_BLOCK_TOKEN 0xFE
#define STOP_TRAN_TOKEN 0xFD

// Card types
#define CT_MMC 0x01
#define CT_SD1 0x02
#define CT_SD2 0x04
#define CT_BLOCK 0x08

// SD Card class
class SDCardPIO
{
public:
    SDCardPIO(PIO pio, uint sm, uint pin_cs, uint pin_sck, uint pin_mosi, uint pin_miso);

    bool begin(float freq_hz = 10000000.0f); // Default 10 MHz
    void end();

    bool readBlock(uint32_t block, uint8_t *buffer);
    bool writeBlock(uint32_t block, const uint8_t *buffer);
    bool readBlocks(uint32_t block, uint8_t *buffer, uint32_t count);
    bool writeBlocks(uint32_t block, const uint8_t *buffer, uint32_t count);

    uint8_t getCardType() const { return cardType; }
    uint64_t getCardSize() const;

    bool readOCR(uint8_t *ocr);
    bool readCID(uint8_t *cid);
    bool readCSD(uint8_t *csd);
    bool readSSR(uint8_t *ssr);

private:
    PIO pio;
    uint sm;
    uint pinCS, pinSCK, pinMOSI, pinMISO;
    uint8_t cardType;

    void csLow();
    void csHigh();
    uint8_t xchgByte(uint8_t data);
    uint8_t xchgByteBitBang(uint8_t data);
    void sendBytes(const uint8_t *data, uint32_t len);
    void recvBytes(uint8_t *data, uint32_t len);
    uint8_t sendCommand(uint8_t cmd, uint32_t arg);
    bool waitReady(uint32_t timeout_ms = 500);
    bool readData(uint8_t *buffer, uint32_t len);
    bool writeData(uint8_t token, const uint8_t *buffer);
};

#endif // SD_PIO_H
