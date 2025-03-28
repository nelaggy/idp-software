from machine import Pin, PWM

class Motors:
    """
    Class to control the motors of the robot.
    """

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
        """
        Sets wheel speed by setting the duty cycle of the PWM signal.
        """
        if l_speed is not None:
            self.l_speed = min(100,abs(l_speed))
            self.l_dir = 1 if l_speed >= 0 else 0
            self.l_motor_dir.value(self.l_dir)
            self.l_motor_spd.duty_u16(int(65535*self.l_speed/100)) 
        
        if r_speed is not None:
            self.r_speed = min(100,abs(r_speed))
            self.r_dir = 1 if r_speed >= 0 else 0
            self.r_motor_dir.value(self.r_dir)
            self.r_motor_spd.duty_u16(int(65535*self.r_speed/100)) 
        
 
    def wheel_direction(self,l_wheel_dir=None, r_wheel_dir=None):
        """
        Sets wheel direction by setting the direction pin.
        """
        if l_wheel_dir is not None:
            self.l_dir = 1 if l_wheel_dir else 0
            self.l_motor_dir.value(self.l_dir)
        
        if r_wheel_dir is not None:
            self.r_dir = 1 if r_wheel_dir else 0
            self.r_motor_dir.value(self.r_dir)
    
    def stop(self, left=True, right=True):
        """
        Stops wheel movement by setting the duty cycle to 0.
        """
        if left:
            self.l_motor_spd.duty_u16(0)
            self.l_speed = 0
        
        if right:
            self.r_motor_spd.duty_u16(0)
            self.r_speed = 0

        