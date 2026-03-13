#ifndef DIGPOT_H
#define DIGPOT_H

#include <Arduino.h>
#include <Wire.h>

namespace digpot
{

    constexpr int8_t kBothChannels = -1;

    bool probeAddress(TwoWire &wire, uint8_t address);
    size_t discoverAddresses(TwoWire &wire, const uint8_t *candidates, size_t candidateCount, uint8_t *matches, size_t maxMatches);
    bool discoverSingleAddress(TwoWire &wire, const uint8_t *candidates, size_t candidateCount, uint8_t &address);
    uint8_t resolveChannels(int8_t channel, uint8_t channels[2]);

    class TPL0102
    {
    public:
        static constexpr uint8_t kCandidateAddresses[] = {0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57};
        static constexpr size_t kCandidateAddressCount = 8;

        static constexpr uint8_t kRegWra = 0x00;
        static constexpr uint8_t kRegWrb = 0x01;
        static constexpr uint8_t kRegAcr = 0x10;
        static constexpr uint8_t kAcrVolatileOnlyBit = 0x80;
        static constexpr uint8_t kAcrShutdownBit = 0x40;
        static constexpr uint8_t kAcrWipBit = 0x02;

        struct AcrState
        {
            uint8_t raw;
            bool volatileOnly;
            bool shutdownDisabled;
            bool writeInProgress;
        };

        TPL0102(TwoWire &wire, uint8_t address);

        uint8_t address() const;

        bool readRegister(uint8_t reg, uint8_t &value) const;
        bool writeRegister(uint8_t reg, uint8_t value);

        bool setVolatileAccess(bool enabled, uint8_t *previousAcr = nullptr);

        bool readWiper(uint8_t channel, uint8_t &value) const;
        bool writeWiper(uint8_t channel, uint8_t value);

        bool readNvram(uint8_t channel, uint8_t &value);
        bool writeNvram(uint8_t channel, uint8_t value);

        bool readAcr(AcrState &acr) const;
        bool writeAcr(uint8_t value);
        bool setShutdown(bool enableShutdown);
        bool pollWip(uint32_t timeoutMs, uint16_t intervalMs = 10) const;

    private:
        bool channelToRegister(uint8_t channel, uint8_t &reg) const;

        TwoWire &_wire;
        uint8_t _address;
    };

    class AD5142A
    {
    public:
        static constexpr uint8_t kCandidateAddresses[] = {0x20, 0x22, 0x23, 0x28, 0x2A, 0x2B, 0x2C, 0x2E, 0x2F};
        static constexpr size_t kCandidateAddressCount = 9;

        enum class ReadbackSource : uint8_t
        {
            Input = 0x0,
            Eeprom = 0x1,
            Control = 0x2,
            Rdac = 0x3,
        };

        struct VoltageDividerConfig
        {
            float vhVoltage = 5.0f;
            float vlVoltage = 0.0f;
            float rhResistance = 1000.0f;
            float rlResistance = 0.0f;
            float rpotOhms = 100000.0f;
        };

        struct VoltageDividerResult
        {
            uint8_t position;
            float expectedVoltage;
            float errorVoltage;
        };

        AD5142A(TwoWire &wire, uint8_t address);

        uint8_t address() const;

        static uint16_t buildWord(uint8_t command, uint8_t commandAddress, uint8_t data);

        bool writeWord(uint16_t word);
        bool writeCommand(uint8_t command, uint8_t commandAddress, uint8_t data);
        bool readResponseByte(uint8_t &value) const;

        bool nop();

        bool writeRdac(uint8_t channel, uint8_t value);
        bool readRdac(uint8_t channel, uint8_t &value);

        bool writeInput(uint8_t channel, uint8_t value);
        bool inputToRdac(uint8_t channel);

        bool readback(uint8_t channel, ReadbackSource source, uint8_t &value);

        bool linearStep(uint8_t channel, bool increment, uint16_t steps = 1);
        bool step6dB(uint8_t channel, bool increment, uint16_t steps = 1);

        bool rdacToEeprom(uint8_t channel);
        bool eepromToRdac(uint8_t channel);
        bool eepromWrite(uint8_t channel, uint8_t value);

        bool topScale(uint8_t channel, bool shutdown);
        bool bottomScale(uint8_t channel, bool enterBottomScale);

        bool reset();
        bool shutdown(uint8_t channel, bool enable);

        bool controlWrite(uint8_t value);

        static bool validateVoltageDividerConfig(const VoltageDividerConfig &cfg);
        static float expectedVoltageForPosition(uint8_t position, const VoltageDividerConfig &cfg);
        static uint8_t positionForVoltage(float voltage, const VoltageDividerConfig &cfg);

        bool voltdivWrite(uint8_t channel, float targetVoltage, const VoltageDividerConfig &cfg, VoltageDividerResult &result);
        bool voltdivRead(uint8_t channel, const VoltageDividerConfig &cfg, uint8_t &position, float &expectedVoltage);

    private:
        bool channelToAddress(uint8_t channel, uint8_t &commandAddress) const;

        TwoWire &_wire;
        uint8_t _address;
    };

} // namespace digpot

#endif // DIGPOT_H
