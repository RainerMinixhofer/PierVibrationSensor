"""
Test I2C Bus 0 for devices
"""
# pylint: disable=import-error
import machine
# pylint: disable=import-error

sdaPIN=machine.Pin("GPIO8")
sclPIN=machine.Pin("GPIO9")
i2c=machine.I2C(0,sda=sdaPIN, scl=sclPIN, freq=400000)
devices = i2c.scan()
if len(devices) != 0:
    print('Number of I2C devices found=',len(devices))
    for device in devices:
        print("Device Hexadecimal Address= ",hex(device))
else:
    print("No device found")
