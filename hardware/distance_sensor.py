from machine import Pin, I2C, Timer
from vl53l0x import VL53L0X
from micropython import schedule

class DistanceSensor:
    def __init__(self, threshold, cb, sda_pin = 16, scl_pin = 17, id = 0, budget = 40000):
        self.i2c = I2C(id=id, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.sensor = VL53L0X(self.i2c)
        self.threshold = threshold
        self.cb = cb
        self.sensor.set_measurement_timing_budget(budget)  # Set the measurement timing budget to 40ms
        self.sensor.set_Vcsel_pulse_period(self.sensor.vcsel_period_type[0], 12)
        self.sensor.set_Vcsel_pulse_period(self.sensor.vcsel_period_type[1], 8)
    
    def start_ranging(self):
        self.timer = Timer(mode=Timer.PERIODIC, period=500, callback=self.measure)

    def measure(self):
        schedule(self.check_distance, None)

    def check_distance(self):
        distance = self.sensor.ping() - 50
        if distance < self.threshold:
            self.cb(distance)

    def stop_ranging(self):
        self.timer.deinit()
