from machine import Pin, PWM
from time import sleep

l_motor_dir = Pin(7, Pin.OUT)
l_motor_spd = PWM(Pin(6, Pin.OUT), freq=1000, duty_u16=0)
r_motor_spd = PWM(Pin(5, Pin.OUT), freq=1000, duty_u16=0)
r_motor_dir = Pin(4, Pin.OUT)

# Connect to 

line_sensor_1 = Pin(14, Pin.IN)  # Set GPIO2 as an input pin

def line_sensor_test():
    while True:
        if line_sensor_1.value() == 1:  # HIGH output (White detected)
            print("White detected")
        else:  # LOW output (Black detected)
            print("Black detected")

def line_sensor_with_motor_test():
    while True:
        if line_sensor_1.value() == 1:  # HIGH output (White detected)
            l_motor_dir.value(0)
            r_motor_dir.value(0)
            l_motor_spd.duty_u16(65535)
            r_motor_spd.duty_u16(65535)
            sleep(10)
        else:  # LOW output (Black detected)
            l_motor_dir.value(1)
            r_motor_dir.value(1)
            l_motor_spd.duty_u16(32768)
            r_motor_spd.duty_u16(32768)
            