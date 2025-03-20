from machine import Pin, PWM
class Servo:

    def __init__(self,pin = 15, freq = 50):
        self.servo = PWM(Pin(pin))
        self.servo.freq(freq)

        self.min_duty = 1802
        self.max_duty = 7864 
        self.angle = 0
    
    def set_angle(self, angle):
        if 0 <= angle <= 180:
            duty = self.min_duty + (angle / 180) * (self.max_duty - self.min_duty)
            self.servo.duty_u16(int(duty))
            self.angle = angle

    def lift(self):
        self.set_angle(45)
    
    def lift90(self):
        self.set_angle(90)

    def drop(self):
        self.set_angle(25)
    
    def deinit(self):
        self.servo.deinit()
        self.angle = None
