"""
Test program for reading and writing to the ADC ADS1256
"""
import time
import sys
# pylint: disable=import-error
from machine import SPI, Pin
# pylint: enable=import-error
from ads1256lib import ADS1256

PDWN_PIN = 14
CS_PIN = 13
DRDY_PIN = 15
DOUT_PIN = 12
DIN_PIN = 11
SCLK_PIN = 10

def cycle_channel_timing(adc, ch, ignore_drdy=False):
    """
    Cycle to channel ch and returns the conversion of the previous channel
    Cycling through channels has 3 steps (see page 21 of ADS1256 datasheet)
    """
    ts = time.ticks_us() # pylint: disable=no-member
    adc.select()
    while adc.drdy.value() and not ignore_drdy:
        pass
    t0 = time.ticks_us() # pylint: disable=no-member
    dt0 = time.ticks_diff(t0,ts) # pylint: disable=no-member
    #Step1
    channel = bytearray(1)
    channel[0] = ch
    adc.spi.write(adc.cmd_write_mux) #Write into MUX register
    adc.spi.write(channel)
    time.sleep_us(1) #Delay t11=0.52us # pylint: disable=no-member
    t1 = time.ticks_us() # pylint: disable=no-member
    dt1 = time.ticks_diff(t1,t0) # pylint: disable=no-member
    #Step2
    adc.spi.write(adc.cmd_sync) #SYNC
    time.sleep_us(4) #Delay t11=3.12us # pylint: disable=no-member
    adc.spi.write(adc.cmd_wakeup) #WAKEUP
    t2 = time.ticks_us() # pylint: disable=no-member
    dt2 = time.ticks_diff(t2,t1) # pylint: disable=no-member
    #Step3
    adc.spi.write(adc.cmd_rdata) #RDATA
    time.sleep_us(7) #Delay t6=6.5us # pylint: disable=no-member
    adc.spi.readinto(adc.conversion,0xFF) #Read 3bytes
    t3 = time.ticks_us() # pylint: disable=no-member
    dt3 = time.ticks_diff(t3,t2) # pylint: disable=no-member
    adc.deselect()
    return dt0, dt1,dt2,dt3, t3-ts

def cycle_channel(adc, ch, ignore_drdy=False):
    """
    Cycle to channel ch and returns the conversion of the previous channel
    Cycling through channels has 3 steps (see page 21 of ADS1256 datasheet)
    """
    adc.select()
    while adc.drdy.value() and not ignore_drdy:
        pass
    #Step1
    channel = bytearray(1)
    channel[0] = ch
    adc.spi.write(adc.cmd_write_mux) #Write into MUX register
    adc.spi.write(channel)
    time.sleep_us(1) #Delay t11=0.52us # pylint: disable=no-member
    #Step2
    adc.spi.write(adc.cmd_sync) #SYNC
    time.sleep_us(4) #Delay t11=3.12us # pylint: disable=no-member
    adc.spi.write(adc.cmd_wakeup) #WAKEUP
    #Step3
    adc.spi.write(adc.cmd_rdata) #RDATA
    time.sleep_us(7) #Delay t6=6.5us # pylint: disable=no-member
    adc.spi.readinto(adc.conversion,0xFF) #Read 3bytes
    adc.deselect()
    return adc.conversion


spi = SPI(1,baudrate=1000000, polarity=0, phase=1, bits=8, firstbit=SPI.MSB,
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
ads1256.write_reg(ADS1256.DRATE,ADS1256.DR500)
reg = ads1256.read_reg(ADS1256.DRATE)[0]
assert (reg == ADS1256.DR500), f"DRATE register should read DR500, got {reg}"
# Select Single ended channel 0 for read
ads1256.write_reg(ADS1256.MUX,ADS1256.CH0)
reg = ads1256.read_reg(ADS1256.MUX)[0]
assert (reg == ADS1256.CH0), f"MUX register should read CH0, got {reg}"

print("\nMeasuring time intervals of substeps of cycle_channel")
dt0,dt1,dt2,dt3,ttot = cycle_channel_timing(ads1256, ADS1256.CH0, ignore_drdy=False)
print("Pre-Step1: ",dt0,"us", "Step1: ",dt1,"us", "Step2: ",dt2,"us", "Step3: ",dt3,"us", "Total: ",ttot,"us")

print("\nMeasuring time for 100 reads using cycle_channel in this routine. Should be limited by sampling rate")
cur_ch = ADS1256.CH0
N = 100
start = time.ticks_ms() # pylint: disable=no-member
for _ in range(N):
    nxt_ch = ADS1256.CH7 if cur_ch == ADS1256.CH0 else ADS1256.CH0
    cycle_channel(ads1256, nxt_ch, ignore_drdy=True)
    cur_ch = nxt_ch
end = time.ticks_ms() # pylint: disable=no-member
print(f"cycle_channel took {time.ticks_diff(end,start)/N} ms averaged from {N} reads") # pylint: disable=no-member

print("\nMeasuring time for 100 reads using cycle_channel in ads1256 class. Should be limited by sampling rate")
cur_ch = ADS1256.CH0
N = 100
start = time.ticks_ms() # pylint: disable=no-member
for _ in range(N):
    nxt_ch = ADS1256.CH7 if cur_ch == ADS1256.CH0 else ADS1256.CH0
    ads1256.cycle_channel(nxt_ch)
    cur_ch = nxt_ch
end = time.ticks_ms() # pylint: disable=no-member
print(f"cycle_channel took {time.ticks_diff(end,start)/N} ms averaged from {N} reads") # pylint: disable=no-member

print("\nLooping through channels 0 and 7 indefinitely")
while True:
    data = ads1256.raw_to_signed(ads1256.cycle_channel(ADS1256.CH7))
    print("ADS1256 CH0 reading: ",data)
    data = ads1256.raw_to_signed(ads1256.cycle_channel(ADS1256.CH0))
    print("ADS1256 CH7 reading: ",data)
    time.sleep(1)
