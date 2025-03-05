
# left speed, right speed 0-100
# left direction, right direction True- forwards

# check if changing frequency changes max speed
# check colour sensor
# check if pico still works even if disconnected with computer

from machine import Pin, PWM

class Motors:

    def __init__(self, freq=1000):
        self.freq = freq
        self.l_motor_dir = Pin(7, Pin.OUT)
        self.l_motor_spd = PWM(Pin(6))
        self.l_motor_spd.freq(self.freq)
        self.l_motor_spd.duty_u16(0)

        self.r_motor_dir = Pin(4, Pin.OUT)
        self.r_motor_spd = PWM(Pin(5))
        self.r_motor_spd.freq(self.freq)
        self.r_motor_spd.duty_u16(0)

        self.l_speed = 0
        self.r_speed = 0
        self.l_dir = False
        self.r_dir = False

    def wheel_speed(self,l_speed=None, r_speed=None):
        '''
        Set speed of both motors (0,100)
        Directions are handled by +- sign of speed
        '''
        if l_speed is not None:
            self.l_speed = max(-100, min(100,l_speed)) # clamp speed between -100, 100
            self.l_dir = 1 if l_speed >= 0 else 0
            self.l_motor_dir.value(self.l_dir)
            self.l_motor_spd.duty_u16(int(65535*self.l_speed/100)) # speed
        
        if r_speed is not None:
            self.r_speed = max(-100, min(100,r_speed)) # clamp speed between -100, 100
            self.r_dir = 1 if r_speed >= 0 else 0
            self.r_motor_dir.value(self.r_dir)
            self.r_motor_spd.duty_u16(int(65535*self.r_speed/100)) # speed
        
 
    def wheel_direction(self,l_wheel_dir=None, r_wheel_dir=None):
        if l_wheel_dir is not None:
            self.l_dir = 1 if l_wheel_dir else 0
            self.l_motor_dir.value(self.l_dir)
        
        if r_wheel_dir is not None:
            self.r_dir = 1 if r_wheel_dir else 0
            self.r_motor_dir.value(self.r_dir)
    
    def stop(self, left=True, right=True):
        if left:
            self.l_motor_spd.duty_u16(0)
            self.l_speed = 0
        
        if right:
            self.r_motor_spd.duty_u16(0)
            self.r_speed = 0

        