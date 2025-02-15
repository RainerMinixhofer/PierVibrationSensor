'''
Description: Onboard LED Blink Program.
Author     : M.Pugazhendi
Date       : 06thMar2021

A. Intialize timer_one, trigger LED blink period to 500mSec.

'''

# pylint: disable=import-error
from machine import Pin, Timer
# pylint: enable=import-error

led = Pin("LED", Pin.OUT, value=0)

def blink(t): # pylint: disable=W0613
    """ Toggle LED """
    led.toggle()

Timer().init(period=500, mode=Timer.PERIODIC, callback=blink)

while True:
    pass
