from machine import Pin, PWM
from time import sleep
from hardware.servo import Servo

# Connect servo to J35

# Set up PWM Pin for servo control
servo_pin = Pin(15)
servo = Servo()

# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

#Set PWM frequency
# frequency = 50
# servo.freq (frequency)

def servo_test():
    try:
        while True:
            pass
            # #Servo at 0 degrees
            # servo.duty_u16(min_duty)
            # sleep(2)

            # #Servo at 90 degrees
            # servo.duty_u16(half_duty)
            # sleep(2)

            # #Servo at 180 degrees
            # servo.duty_u16(max_duty)
            # sleep(2)

    except KeyboardInterrupt:
        print("Keyboard interrupt")
        # Turn off PWM
        servo.deinit()

servo.drop()