from machine import Pin
from machine import I2C
from dummy.tcs34725 import TCS34725
from time import sleep


i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))
tcs = TCS34725(i2c_bus)

while True:
        
    print('raw: {}'.format(tcs.read(raw = True)))

    print('rgb: {}'.format(tcs.read(raw = False)))

    sleep(0.8)