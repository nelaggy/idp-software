from machine import Pin
from machine import I2C
from dummy.tcs34725 import TCS34725
import time


i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))
tcs = TCS34725(i2c_bus)

start_time = time.time()

while time.time()-start_time < 3:
        
    print('cct: {}'.format(tcs.read(raw = False)))

    print('rgb: {}'.format(tcs.read(raw = True)))

    time.sleep(0.5)