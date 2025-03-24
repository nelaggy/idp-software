from machine import Pin, PWM
class Servo:
    """
    Class to control the servo motor.
    """
    def __init__(self,pin = 15, freq = 50):
        self.servo = PWM(Pin(pin))
        self.servo.freq(freq)

        self.min_duty = 1802
        self.max_duty = 7864 
        self.angle = 0
    
    def set_angle(self, angle):
        """
        Sets the angle of the servo motor.
        """
        if 0 <= angle <= 180:
            duty = self.min_duty + (angle / 180) * (self.max_duty - self.min_duty)
            self.servo.duty_u16(int(duty))
            self.angle = angle

    def lift(self):
        """
        Lifts the arm of the servo motor by 45 degrees.
        """
        self.set_angle(45)
    
    def lift90(self):
        """
        Lifts the arm of the servo motor by 90 degrees.
        """
        self.set_angle(90)

    def drop(self):
        """
        drops servo arm
        """
        self.set_angle(25)
    
    def deinit(self):
        self.servo.deinit()
        self.angle = None
