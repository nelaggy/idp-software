from machine import Pin
from machine import I2C
from dummy.tcs34725 import TCS34725
import time


class ColourSensor:

    def __init__(self, sda_pin=16, scl_pin=17):
        self.i2c_bus = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.tcs = TCS34725(self.i2c_bus)
    
    def detect(self): # True if green or blue, false if red or yellow
        cct_sum = 0
        B_sum = 0
        n = 0
        start_time = time.time()

        while time.time() - start_time < 0.5: # Run for 3 seconds
            cct_sum += self.tcs.read(raw=False)[0] # Get cct value
            B_sum += self.tcs.read(raw=True)[2] # Get blue value
            n += 1
            time.sleep(0.05)
        
        cct_average = cct_sum/n
        B_average = B_sum/n
        return cct_average > 20000 or B_average > 20