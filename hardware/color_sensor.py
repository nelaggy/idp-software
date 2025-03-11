from machine import Pin
from machine import I2C
from dummy.tcs34725 import TCS34725
import time


class colour_sensor:

    def __init__(self, sda_pin=16, scl_pin=17):
        self.i2c_bus = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.tcs = TCS34725(self.i2c_bus)

        # 0 - blue/green, 1 - yellow/red
        self.station_colour = [None,None,None,None]
        '''
        self.station_colour = {
            'A' : None,
            'B' : None,
            'C' : None,
            'D' : None
        }
        '''
    
    def detect(self,station):
        '''
        Detect colour and stores classification for given station
        Station 0-3 = A-D
        '''
        cct_sum = 0
        n = 0
        start_time = time.time()

        while time.time() - start_time < 3: # Run for 3 seconds
            cct_sum += self.tcs.read(raw=False)[0] # Get cct value
            n+=1
            time.sleep(0.1)
        
        cct_average = cct_sum/n
        self.station_colour[station] = 0 if cct_average>8000 else 1
        return self.station_colour

    def reset(self):
        self.station_colour = [None,None,None,None]
        '''
        self.station_colour = {
            'A' : None,
            'B' : None,
            'C' : None,
            'D' : None
        }
        '''