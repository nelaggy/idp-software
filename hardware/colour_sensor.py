from machine import Pin
from machine import I2C
from dummy.tcs34725 import TCS34725
import time


class ColourSensor:

    def __init__(self, sda_pin=16, scl_pin=17):
        self.i2c_bus = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.tcs = TCS34725(self.i2c_bus)
    
    def detect(self):
        """
        Detects blue/green red/yellow colours using the TCS34725 sensor.
        
        Arguments:
            self

        Returns:
            bool: True if blue/green, False if red/yellow
        """
        cct_sum = 0
        B_sum = 0
        n = 0

        while n < 10:
            cct_sum += self.tcs.read(raw=False)[0]
            B_sum += self.tcs.read(raw=True)[2]
            n += 1
            time.sleep_ms(40)
        
        cct_average = cct_sum/n
        B_average = B_sum/n
        return cct_average > 18000 or B_average > 18