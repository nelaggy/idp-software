from machine import Pin, PWM
from time import sleep

l_motor_dir = Pin(7, Pin.OUT)
l_motor_spd = PWM(Pin(6, Pin.OUT), freq=1000, duty_u16=0)
r_motor_spd = PWM(Pin(5, Pin.OUT), freq=1000, duty_u16=0)
r_motor_dir = Pin(4, Pin.OUT)

def motor_test():
    l_motor_dir.value(0)
    r_motor_dir.value(0)
    l_motor_spd.duty_u16(65535)
    r_motor_spd.duty_u16(65535)
    # sleep(1)
    # l_motor_dir.value(1)
    # r_motor_dir.value(1)
    # l_motor_spd.duty_u16(32768)
    # r_motor_spd.duty_u16(32768)
    # sleep(1)
    # l_motor_spd.duty_u16(0)
    # r_motor_spd.duty_u16(0)
    # sleep(1)
    # l_motor_dir.value(1)
    # r_motor_dir.value(0)
    # l_motor_spd.duty_u16(32768)
    # r_motor_spd.duty_u16(32768)
    # sleep(1)
    # l_motor_dir.value(0)
    # r_motor_dir.value(1)
    # l_motor_spd.duty_u16(65535)
    # r_motor_spd.duty_u16(65535)
    # sleep(1)
    # l_motor_spd.duty_u16(0)
    # r_motor_spd.duty_u16(0)
    # sleep(1)
