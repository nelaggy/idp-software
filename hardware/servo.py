from machine import Pin, PWM
from time import sleep

# Connect servo to J35
# Servo rotates to an angle determined by width of duty cycle

class servo:

    def __init__(self,pin = 15, freq = 50):
        self.servo = PWM(Pin(pin))
        self.servo.freq(freq)

        self.min_duty = 1802 # 0°
        self.max_duty = 7864 # 180°
        self.angle = 0 #in degrees, tracks current angle of servo
    
    def set_angle(self, angle):
        if 0 <= angle <= 180:
            duty = self.min_duty + (angle / 180) * (self.max_duty - self.min_duty)
            self.servo.duty_u16(int(duty))
            sleep(0.5)  # Simulate movement time
            self.angle = angle

    def lift(self):
        '''Move to 45°'''
        self.set_angle(45)

    def drop(self):
        '''Move to 0°'''
        self.set_angle(0)
    
    def deinit(self):
        '''Turn off PWM and reset angle tracking'''
        self.servo.deinit() #Turn off PWM
        self.angle = None
