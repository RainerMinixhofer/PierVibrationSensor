"""
Test program for reading and writing to the ADC ADS1256
"""
import time
# pylint: disable=import-error
from machine import SoftSPI, Pin
# pylint: enable=import-error
from ads1256lib import ADS1256

PDWN_PIN = 14
CS_PIN = 13
DRDY_PIN = 15
DOUT_PIN = 12
DIN_PIN = 11
SCLK_PIN = 10

# SoftSPI used since it seems to be more stable than HW SPI
spi = SoftSPI(baudrate=115200, polarity=0, phase=1, bits=8, firstbit=SoftSPI.MSB,
              sck=Pin(SCLK_PIN, Pin.OUT), mosi=Pin(DIN_PIN, Pin.OUT), miso=Pin(DOUT_PIN, Pin.IN, pull=None))
# Pin SYNC/PDWN as OUTPUT, active low.
# Is used as SYNC pin when low signal is only 0.6 us long, and used as power down when low for 20 DRDY cycles = 20/DRATE
# which is 667 us for fastest and 8s for slowest data rate
# Attention! since the SYNC Pin is active low it needs to be assigned 1 at initialization otherwise the ads1256 will not startup!
ads1256 = ADS1256(spi, ref_voltage=2.5, pga=ADS1256.PGA1,
                  cs=Pin(CS_PIN, Pin.OUT, value=1), drdy=Pin(DRDY_PIN, Pin.IN, pull=None),
                  sync_pwdn=Pin(PDWN_PIN, Pin.OUT, value=1))

# Trigger Reset
ads1256.reset()
ads1256.wakeup()

reg = ads1256.read_reg(ADS1256.STATUS)[0]
assert (reg in (0x30, 0x31)), f"STATUS register should read either 0x30 or 0x31 after reset, got {reg}"
reg = ads1256.read_reg(ADS1256.MUX)[0]
assert (reg == 0x01), f"MUX register should read 0x01 after reset, got {reg}"
reg = ads1256.read_reg(ADS1256.ADCON)[0]
assert (reg == 0x20), f"ADCON register should read 0x20 after reset, got {reg}"
reg = ads1256.read_reg(ADS1256.DRATE)[0]
assert (reg == 0xF0), f"DRATE register should read 0xF0 after reset, got {reg}"
#LSB of IO register DIO0 is configured as CLKOUT after reset so it can read 1 or 0 thus we have default 0xE0 or 0xE1
reg = ads1256.read_reg(ADS1256.IO)[0]
assert (reg in (0xE0, 0xE1)), f"IO register should read 0xE0 or 0xE1 after reset, got {reg}"
# For demo we select the lowest datarate of 2.5 SPS
ads1256.write_reg(ADS1256.DRATE,ADS1256.DR100)
reg = ads1256.read_reg(ADS1256.DRATE)[0]
assert (reg == ADS1256.DR100), f"DRATE register should read DR100, got {reg}"
# Select Single ended channel 0 for read
ads1256.write_reg(ADS1256.MUX,ADS1256.CH0)
reg = ads1256.read_reg(ADS1256.MUX)[0]
assert (reg == ADS1256.CH0), f"MUX register should read CH0, got {reg}"

while True:
    data = ads1256.cycle_channel(ADS1256.CH0)
    print("ADS1256 CH0 reading: ",int(data.hex(),16))
    data = ads1256.cycle_channel(ADS1256.CH7)
    print("ADS1256 CH7 reading: ",int(data.hex(),16))
    time.sleep(1)
