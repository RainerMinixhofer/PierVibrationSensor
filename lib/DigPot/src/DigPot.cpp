#include "DigPot.h"

namespace digpot
{

    bool probeAddress(TwoWire &wire, uint8_t address)
    {
        wire.beginTransmission(address);
        return wire.endTransmission() == 0;
    }

    size_t discoverAddresses(TwoWire &wire, const uint8_t *candidates, size_t candidateCount, uint8_t *matches, size_t maxMatches)
    {
        size_t matchCount = 0;
        for (size_t i = 0; i < candidateCount; ++i)
        {
            if (probeAddress(wire, candidates[i]))
            {
                if (matchCount < maxMatches)
                {
                    matches[matchCount] = candidates[i];
                }
                ++matchCount;
            }
        }
        return matchCount;
    }

    bool discoverSingleAddress(TwoWire &wire, const uint8_t *candidates, size_t candidateCount, uint8_t &address)
    {
        uint8_t matches[4] = {0};
        size_t count = discoverAddresses(wire, candidates, candidateCount, matches, 4);
        if (count != 1)
        {
            return false;
        }
        address = matches[0];
        return true;
    }

    uint8_t resolveChannels(int8_t channel, uint8_t channels[2])
    {
        if (channel == kBothChannels)
        {
            channels[0] = 0;
            channels[1] = 1;
            return 2;
        }
        if (channel == 0 || channel == 1)
        {
            channels[0] = static_cast<uint8_t>(channel);
            return 1;
        }
        return 0;
    }

    TPL0102::TPL0102(TwoWire &wire, uint8_t address) : _wire(wire), _address(address) {}

    uint8_t TPL0102::address() const
    {
        return _address;
    }

    bool TPL0102::channelToRegister(uint8_t channel, uint8_t &reg) const
    {
        if (channel == 0)
        {
            reg = kRegWra;
            return true;
        }
        if (channel == 1)
        {
            reg = kRegWrb;
            return true;
        }
        return false;
    }

    bool TPL0102::readRegister(uint8_t reg, uint8_t &value) const
    {
        _wire.beginTransmission(_address);
        _wire.write(reg);
        if (_wire.endTransmission(false) != 0)
        {
            return false;
        }

        uint8_t read = _wire.requestFrom(static_cast<int>(_address), 1);
        if (read != 1 || _wire.available() < 1)
        {
            return false;
        }

        value = _wire.read();
        return true;
    }

    bool TPL0102::writeRegister(uint8_t reg, uint8_t value)
    {
        _wire.beginTransmission(_address);
        _wire.write(reg);
        _wire.write(value);
        return _wire.endTransmission() == 0;
    }

    bool TPL0102::setVolatileAccess(bool enabled, uint8_t *previousAcr)
    {
        uint8_t acr = 0;
        if (!readRegister(kRegAcr, acr))
        {
            return false;
        }

        if (previousAcr != nullptr)
        {
            *previousAcr = acr;
        }

        uint8_t updated = enabled ? static_cast<uint8_t>(acr | kAcrVolatileOnlyBit)
                                  : static_cast<uint8_t>(acr & ~kAcrVolatileOnlyBit);
        if (updated == acr)
        {
            return true;
        }
        return writeRegister(kRegAcr, updated);
    }

    bool TPL0102::readWiper(uint8_t channel, uint8_t &value) const
    {
        uint8_t reg = 0;
        if (!channelToRegister(channel, reg))
        {
            return false;
        }
        return readRegister(reg, value);
    }

    bool TPL0102::writeWiper(uint8_t channel, uint8_t value)
    {
        uint8_t reg = 0;
        if (!channelToRegister(channel, reg))
        {
            return false;
        }
        return writeRegister(reg, value);
    }

    bool TPL0102::readNvram(uint8_t channel, uint8_t &value)
    {
        if (!setVolatileAccess(false))
        {
            return false;
        }

        uint8_t reg = 0;
        bool ok = channelToRegister(channel, reg) && readRegister(reg, value);
        bool restoreOk = setVolatileAccess(true);
        return ok && restoreOk;
    }

    bool TPL0102::writeNvram(uint8_t channel, uint8_t value)
    {
        if (!setVolatileAccess(false))
        {
            return false;
        }

        uint8_t reg = 0;
        bool ok = channelToRegister(channel, reg) && writeRegister(reg, value);
        bool restoreOk = setVolatileAccess(true);
        return ok && restoreOk;
    }

    bool TPL0102::readAcr(AcrState &acr) const
    {
        uint8_t raw = 0;
        if (!readRegister(kRegAcr, raw))
        {
            return false;
        }

        acr.raw = raw;
        acr.volatileOnly = (raw & kAcrVolatileOnlyBit) != 0;
        acr.shutdownDisabled = (raw & kAcrShutdownBit) != 0;
        acr.writeInProgress = (raw & kAcrWipBit) != 0;
        return true;
    }

    bool TPL0102::writeAcr(uint8_t value)
    {
        return writeRegister(kRegAcr, value);
    }

    bool TPL0102::setShutdown(bool enableShutdown)
    {
        uint8_t acr = 0;
        if (!readRegister(kRegAcr, acr))
        {
            return false;
        }

        uint8_t updated = enableShutdown ? static_cast<uint8_t>(acr & ~kAcrShutdownBit)
                                         : static_cast<uint8_t>(acr | kAcrShutdownBit);
        return writeRegister(kRegAcr, updated);
    }

    bool TPL0102::pollWip(uint32_t timeoutMs, uint16_t intervalMs) const
    {
        uint32_t start = millis();
        while (true)
        {
            uint8_t acr = 0;
            if (!readRegister(kRegAcr, acr))
            {
                return false;
            }

            if ((acr & kAcrWipBit) == 0)
            {
                return true;
            }

            if (millis() - start >= timeoutMs)
            {
                return false;
            }

            delay(intervalMs);
        }
    }

    AD5142A::AD5142A(TwoWire &wire, uint8_t address) : _wire(wire), _address(address) {}

    uint8_t AD5142A::address() const
    {
        return _address;
    }

    uint16_t AD5142A::buildWord(uint8_t command, uint8_t commandAddress, uint8_t data)
    {
        return static_cast<uint16_t>(((command & 0x0F) << 12) | ((commandAddress & 0x0F) << 8) | data);
    }

    bool AD5142A::channelToAddress(uint8_t channel, uint8_t &commandAddress) const
    {
        if (channel == 0 || channel == 1)
        {
            commandAddress = channel;
            return true;
        }
        return false;
    }

    bool AD5142A::writeWord(uint16_t word)
    {
        _wire.beginTransmission(_address);
        _wire.write(static_cast<uint8_t>((word >> 8) & 0xFF));
        _wire.write(static_cast<uint8_t>(word & 0xFF));
        return _wire.endTransmission() == 0;
    }

    bool AD5142A::writeCommand(uint8_t command, uint8_t commandAddress, uint8_t data)
    {
        return writeWord(buildWord(command, commandAddress, data));
    }

    bool AD5142A::readResponseByte(uint8_t &value) const
    {
        uint8_t read = _wire.requestFrom(static_cast<int>(_address), 2);
        if (read < 1 || _wire.available() < 1)
        {
            return false;
        }

        value = _wire.read();
        if (_wire.available())
        {
            _wire.read();
        }
        return true;
    }

    bool AD5142A::nop()
    {
        return writeCommand(0x0, 0x0, 0x00);
    }

    bool AD5142A::writeRdac(uint8_t channel, uint8_t value)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }
        return writeCommand(0x1, address, value);
    }

    bool AD5142A::readRdac(uint8_t channel, uint8_t &value)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        if (!writeCommand(0x3, address, static_cast<uint8_t>(ReadbackSource::Rdac)))
        {
            return false;
        }
        return readResponseByte(value);
    }

    bool AD5142A::writeInput(uint8_t channel, uint8_t value)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }
        return writeCommand(0x2, address, value);
    }

    bool AD5142A::inputToRdac(uint8_t channel)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }
        return writeCommand(0x8, address, 0x00);
    }

    bool AD5142A::readback(uint8_t channel, ReadbackSource source, uint8_t &value)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        if (!writeCommand(0x3, address, static_cast<uint8_t>(source)))
        {
            return false;
        }
        return readResponseByte(value);
    }

    bool AD5142A::linearStep(uint8_t channel, bool increment, uint16_t steps)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        uint8_t data = increment ? 1 : 0;
        for (uint16_t i = 0; i < steps; ++i)
        {
            if (!writeCommand(0x4, address, data))
            {
                return false;
            }
        }
        return true;
    }

    bool AD5142A::step6dB(uint8_t channel, bool increment, uint16_t steps)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        uint8_t data = increment ? 1 : 0;
        for (uint16_t i = 0; i < steps; ++i)
        {
            if (!writeCommand(0x5, address, data))
            {
                return false;
            }
        }
        return true;
    }

    bool AD5142A::rdacToEeprom(uint8_t channel)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }
        return writeCommand(0x9, address, 0x01);
    }

    bool AD5142A::eepromToRdac(uint8_t channel)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }
        return writeCommand(0xA, address, 0x00);
    }

    bool AD5142A::eepromWrite(uint8_t channel, uint8_t value)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }
        return writeCommand(0xB, static_cast<uint8_t>(address & 0x3), value);
    }

    bool AD5142A::topScale(uint8_t channel, bool shutdown)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        uint8_t data = shutdown ? 0x81 : 0x80;
        return writeCommand(0xC, address, data);
    }

    bool AD5142A::bottomScale(uint8_t channel, bool enterBottomScale)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        uint8_t data = enterBottomScale ? 0x01 : 0x00;
        return writeCommand(0xD, address, data);
    }

    bool AD5142A::reset()
    {
        return writeCommand(0xE, 0x0, 0x00);
    }

    bool AD5142A::shutdown(uint8_t channel, bool enable)
    {
        uint8_t address = 0;
        if (!channelToAddress(channel, address))
        {
            return false;
        }

        uint8_t data = enable ? 0x01 : 0x00;
        return writeCommand(0xF, address, data);
    }

    bool AD5142A::controlWrite(uint8_t value)
    {
        return writeCommand(0x10, 0x0, static_cast<uint8_t>(value & 0x0F));
    }

    bool AD5142A::validateVoltageDividerConfig(const VoltageDividerConfig &cfg)
    {
        return cfg.vhVoltage != cfg.vlVoltage && cfg.rpotOhms > 0.0f;
    }

    float AD5142A::expectedVoltageForPosition(uint8_t position, const VoltageDividerConfig &cfg)
    {
        float ratio = static_cast<float>(position) / 255.0f;
        return cfg.vlVoltage + (cfg.vhVoltage - cfg.vlVoltage) *
                                   (cfg.rlResistance + cfg.rpotOhms * ratio) /
                                   (cfg.rhResistance + cfg.rlResistance + cfg.rpotOhms);
    }

    uint8_t AD5142A::positionForVoltage(float voltage, const VoltageDividerConfig &cfg)
    {
        float numerator =
            ((voltage - cfg.vlVoltage) *
             (cfg.rhResistance + cfg.rlResistance + cfg.rpotOhms) /
             (cfg.vhVoltage - cfg.vlVoltage)) -
            cfg.rlResistance;

        float position = 255.0f * numerator / cfg.rpotOhms;

        if (position < 0.0f)
        {
            return 0;
        }
        if (position > 255.0f)
        {
            return 255;
        }
        return static_cast<uint8_t>(lroundf(position));
    }

    bool AD5142A::voltdivWrite(uint8_t channel, float targetVoltage, const VoltageDividerConfig &cfg, VoltageDividerResult &result)
    {
        if (!validateVoltageDividerConfig(cfg))
        {
            return false;
        }

        result.position = positionForVoltage(targetVoltage, cfg);
        result.expectedVoltage = expectedVoltageForPosition(result.position, cfg);
        result.errorVoltage = fabsf(result.expectedVoltage - targetVoltage);

        return writeRdac(channel, result.position);
    }

    bool AD5142A::voltdivRead(uint8_t channel, const VoltageDividerConfig &cfg, uint8_t &position, float &expectedVoltage)
    {
        if (!validateVoltageDividerConfig(cfg))
        {
            return false;
        }

        if (!readRdac(channel, position))
        {
            return false;
        }

        expectedVoltage = expectedVoltageForPosition(position, cfg);
        return true;
    }

} // namespace digpot
