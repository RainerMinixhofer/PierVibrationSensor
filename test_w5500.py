"""
Test program for reading and writing to the W5500 ethernet hat
"""
import time
# pylint: disable=import-error
from machine import Pin,SPI
import network
# pylint: enable=import-error

led = Pin("LED", Pin.OUT, value=0)

def w5x00_init():
    """ Initialization of the W5500 ethernet hat and getting the IP address """
    spi=SPI(0,2_000_000, mosi=Pin(19),miso=Pin(16),sck=Pin(18))
    nic = network.WIZNET5K(spi, Pin(17), Pin(20)) #spi,cs,reset pin # type: ignore
    nic.active(True)
    nic.ipconfig(dhcp4=True)
    while not nic.isconnected():
        time.sleep(1)
        print("Waiting for connection...")
    mac = nic.config("mac").hex()
    print("MAC address:"+':'.join([mac[i:i+2] for i in range(0,11,2)]))
    print("IP-Address:",nic.ifconfig())

def main():
    """ Main Loop"""
    w5x00_init()

    while True:
        led.value(1)
        time.sleep(1)
        led.value(0)
        time.sleep(1)

if __name__ == "__main__":
    main()
