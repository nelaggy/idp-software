from machine import Pin, PWM
from time import sleep

# Connect to Motor 2

l_actuator_dir = Pin(3, Pin.OUT)
l_actuator_spd = PWM(Pin(2, Pin.OUT), freq=1000, duty_u16=0)

def linear_actuator_test():
    while True:

        l_actuator_dir.value(0)
        l_actuator_spd.duty_u16(65535)
        sleep(10)
        l_actuator_dir.value(1)
        l_actuator_spd.duty_u16(32768)
        sleep(10)