"""
Program for writing out data for Raspberry Shake Data Producer over UDP Port
"""
import time
import socket
import struct
from secrets import WiFi
# pylint: disable=import-error
from machine import Pin,SPI,SoftSPI,RTC
import rp2
import network
# pylint: enable=import-error
from ads1256lib import ADS1256

led = Pin("LED", Pin.OUT, value=0)

NTP_DELTA = 2208988800
NTPHOST = "pool.ntp.org"

def set_time():
    """
    get time for pico from NTP server and update RTC
    """
    ntp_query = bytearray(48)
    ntp_query[0] = 0x1B
    addr = socket.getaddrinfo(NTPHOST, 123)[0][-1]
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.settimeout(1)
        s.sendto(ntp_query, addr)
        msg = s.recv(48)
    finally:
        s.close()
    val = struct.unpack("!I", msg[40:44])[0]
    t = val - NTP_DELTA
    tm = time.gmtime(t)
    RTC().datetime((tm[0], tm[1], tm[2], tm[6] + 1, tm[3], tm[4], tm[5], 0))

def connect_to_network(use_wlan, ssid=None, password=None, connection_timeout = 10):
    """
    Enable Network interface and get IP address
    If wlan = True then wlan interface is activated
    If wlan = False then the wired ethernet W5500 interface is activated
    """
    # Use WLAN
    if use_wlan:
        wlan = network.WLAN(network.STA_IF)
        rp2.country('AT')
        wlan.active(True)
        wlan.config(pm = 0xa11140) # Disable power-save mode to get faster responsivity
        mac = network.WLAN().config('mac').hex()
        print("MAC address of Raspberry Pico 2W: "+':'.join([mac[i:i+2] for i in range(0,11,2)]))
        print("WLAN Channel: ",wlan.config('channel'))
        print("WLAN ESSID: ",wlan.config('essid'))
        print("WLAN TXPOWER: ",wlan.config('txpower'))
        if (ssid is None or password is None):
            raise RuntimeError('no ssid or password provided')
        wlan.connect(ssid, password)

        while connection_timeout > 0:
            if wlan.status() < 0 or wlan.status() >= 3:
                break
            connection_timeout -= 1
            print('waiting for connection...')
            time.sleep(1)

        if wlan.status() != 3:
            raise RuntimeError('network connection failed')
        print('connected')
        status = wlan.ifconfig()
        print('ip = ' + status[0])
        return True
    # Use W5500 Ethernet HAT
    nicspi=SPI(0,2_000_000, mosi=Pin(19),miso=Pin(16),sck=Pin(18))
    nic = network.WIZNET5K(nicspi, Pin(17), Pin(20)) #spi,cs and reset pins # type: ignore
    mac = nic.config("mac").hex()
    print("MAC address of W5500 Ethernet HAT: "+':'.join([mac[i:i+2] for i in range(0,11,2)]))
    nic.active(True)
    nic.ipconfig(dhcp4=True)
    while connection_timeout > 0:
        if nic.isconnected():
            break
        connection_timeout -= 1
        print('waiting for connection...')
        time.sleep(1)
    if not nic.isconnected():
        raise RuntimeError('network connection failed')
    print('connected')
    print("IP-Address:",nic.ifconfig())
    return True

def main():
    """ Main Loop"""
    connect_to_network(False, WiFi.ssid, WiFi.password)
    set_time()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    #Initialize ADC
    adcspi = SoftSPI(baudrate=115200, polarity=0, phase=1, bits=8, firstbit=SoftSPI.MSB,
                sck=Pin(10, Pin.OUT), mosi=Pin(11, Pin.OUT), miso=Pin(12, Pin.IN, pull=None))
    ads1256 = ADS1256(adcspi, cs=Pin(13, Pin.OUT, value=1), drdy=Pin(15, Pin.IN, pull=None),
                    sync_pwdn=Pin(14, Pin.OUT, value=1), ref_voltage=2.5, pga = ADS1256.PGA1)
    ads1256.reset()
    ads1256.wakeup()
    print("ADS1256 id: " + hex(ads1256.whoami()))
    ads1256.write_reg(ADS1256.DRATE,ADS1256.DR100)
    reg = ads1256.read_reg(ADS1256.DRATE)[0]
    assert (reg == ADS1256.DR100), f"ADS1256 DRATE register should read DR100, got {reg}"
    ads1256.write_reg(ADS1256.MUX,ADS1256.CH0)
    reg = ads1256.read_reg(ADS1256.MUX)[0]
    assert (reg == ADS1256.CH0), f"ADS1256 MUX register should read CH0, got {reg}"

    packet_count = 500
    while packet_count>0:
        channel = 'EHN' # 100 sample per second (E), high gain (H), N-component seismograph
        timestamp = int(round(time.time(), 3)) # timestamp, in epoch seconds, down to milliseconds
        datapipe = []

        ads1256.select_channel(ADS1256.CH0)

        for i in range(25):
            data = ads1256.read_continuous()
            data = int(data.hex(),16)
            if data & 0x800000:
                data += -0x1000000
            datapipe.append(data)
        datapipestr = ','.join(map(str, datapipe))
        packet = f"{{'{channel:s}', {timestamp:.3f} {datapipestr}}}"

        sock.sendto(packet.encode(), ("10.0.0.141", 9999))
        packet_count -= 1

    sock.close()

if __name__ == "__main__":
    main()
