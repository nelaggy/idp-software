from machine import Pin, I2C
from dummy.vl53l0x import VL53L0X
from hardware.servo import Servo
from micropython import schedule
import time
'''
go in straight line
check distance
pick up box
check colour
move backwards
turn around
'''

class OffRoadController:
    def __init__(self, line_sensors, wheels, on_complete, sda_pin = 16, scl_pin = 17) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.on_complete = on_complete

        # tof distance sensor
        self.i2c_vl53l0x = I2C(0, sda=Pin(sda_pin), scl=Pin(scl_pin))
        self.tof = VL53L0X(self.i2c_vl53l0x)
        self.tof.set_measurement_timing_budget(40000)
        # tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)
        self.tof.set_Vcsel_pulse_period(self.tof.vcsel_period_type[0], 12)
        # tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 14)
        self.tof.set_Vcsel_pulse_period(self.tof.vcsel_period_type[1], 8)

        self.distance_list_max_length = 20
        self.distance_list = [10000]*self.distance_list_max_length
        self.distance_cnt = 0
        self.threshold_distance = 50 # distance between the box and robot to stop

        # Servo
        self.servo = Servo()

        # PID constants (Tuning required)
        self.Kp = 5  # Proportional gain
        self.Ki = 0  # Integral gain (usually small or 0)
        self.Kd = 0  # Derivative gain

        # PID variables
        self.D = 0
        self.I = 0
        self.last_error = 0
        self.pickup = False
        self.integral = 0

        self.lspeed = 0
        self.rspeed = 0
        self.target_lspeed = 50
        self.target_rspeed = 50

        
    def on_change(self,values):
        '''Handles sensor input updates and applies PID control'''
        # get distance between box, activate self.pickup if distance smaller than threshold distance
        self.distance_list[self.distance_cnt] = self.tof.ping()-50
        distance = sum(self.distance_list)/self.distance_list_max_length
        if distance <= self.threshold_distance:
            self.pickup = True
        self.distance_cnt = (self.distance_cnt+1)%self.distance_list_max_length

        if not self.pickup: # follow line if not self.pickup activated
            error = 3*values[0] + values[1] - values[2] - 3*values[3]
            if error is None: # detects all black
                self.lost()
                return
            self.I += error
            self.D = error - self.last_error
            self.last_error = error
            pid = self.Kp * error + self.Ki * self.I + self.Kd * self.D

            # Motor speed adjustments
            self.lspeed = max(0, min(55, self.target_lspeed - pid))
            self.rspeed = max(0, min(55, self.target_rspeed + pid))

            self.wheels.wheel_speed(int(self.lspeed), int(self.rspeed))
        
        else: # pickup box if self.pickup activated
            self.wheels.stop()
            schedule(self.pickup_box, None)

    def lost(self) -> None:
        print('lost')
        # self.wheels.wheel_speed(0,50) # slow spin to search, change
        self.wheels.stop()
    
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)
    
    def pickup_box(self) -> None:
        # lift box
        self.servo.lift()
        self.move_backwards()
        self.turn_180()
    
    def move_backwards(self) -> None:
        # Move backwards
        while True:
            values = self.line_sensors.read()  # Read sensor values
            if values == b'\1\1\1\1':  # Stop condition when detecting 1111
                self.wheels.wheel_speed(-30,-30)
                time.sleep(2) # move backwards for 2 second after the junction
                self.wheels.stop()
                break
            
            error = 3*values[0] + values[1] - values[2] - 3*values[3]
            if error is None:  # detects all black
                self.lost()
                return
            self.I += error
            self.D = error - self.last_error
            self.last_error = error
            pid = self.Kp * error + self.Ki * self.I + self.Kd * self.D

            # Motor speed adjustments for backward movement
            self.lspeed = max(-55, min(0, -self.target_lspeed*3/5 - pid))
            self.rspeed = max(-55, min(0, -self.target_rspeed*3/5 + pid))

            self.wheels.wheel_speed(int(self.lspeed), int(self.rspeed))

    def turn_180(self) -> None:
        left_sensor_detected = False
        middle_l_sensor_detected = False
        middle_r_sensor_detected = False

        prev_values = list(self.line_sensors.read()) # initial sensor readings

        while not (left_sensor_detected and middle_l_sensor_detected and middle_r_sensor_detected):
            values = list(self.line_sensors.read()) # read current sensor values

            if not left_sensor_detected and prev_values[0] == 0 and values[0] == 1:
                left_sensor_detected = True
            if not middle_l_sensor_detected and prev_values[1] == 0 and values[1] == 1:
                middle_l_sensor_detected = True
            if not middle_r_sensor_detected and prev_values[2] == 0 and values[2] == 1:
                middle_r_sensor_detected = True

            self.wheels.wheel_speed(-15,15) # keep turning left
            prev_values = values # update previous sensor values
        
        self.on_complete()

        
        
class OffRoadControllerTest:
    def __init__(self, line_sensors, wheels) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels

        # PID constants (Tuning required)
        self.Kp = 5  # Proportional gain
        self.Ki = 0  # Integral gain (usually small or 0)
        self.Kd = 0  # Derivative gain

        # PID variables
        self.D = 0
        self.I = 0
        self.last_error = 0
        self.pickup = False
        self.integral = 0

        self.lspeed = 0
        self.rspeed = 0
        self.target_lspeed = 50
        self.target_rspeed = 50

    
    def on_change(self, values: bytearray) -> None:
        '''Handles sensor input updates and applies PID control'''

        if not self.pickup: # follow line if not self.pickup activated
            error = 3*values[0] + values[1] - values[2] - 3*values[3]
            if error is None: # detects all black
                self.lost()
                return
            self.I += error
            self.D = error - self.last_error
            self.last_error = error
            pid = self.Kp * error + self.Ki * self.I + self.Kd * self.D

            # Motor speed adjustments
            self.lspeed = max(0, min(55, self.target_lspeed - pid))
            self.rspeed = max(0, min(55, self.target_rspeed + pid))

            self.wheels.wheel_speed(int(self.lspeed), int(self.rspeed))
        
        else: # pickup box if self.pickup activated
            self.wheels.stop()
            schedule(self.pickup_box, None)
    

    def lost(self) -> None:
        print('lost')
        # self.wheels.wheel_speed(0,50) # slow spin to search, change
        self.wheels.stop()
    
    def pickup_box(self) -> None:
        self.move_backwards()
        self.turn_180()
        self.wheels.stop()
        print('turn complete!')
    
    def move_backwards(self) -> None:
        # Move backwards
        while True:
            values = self.line_sensors.read()  # Read sensor values
            if values == b'\1\1\1\1':  # Stop condition when detecting 1111
                self.wheels.wheel_speed(-30,-30)
                time.sleep(2) # move backwards for 2 second after the junction
                self.wheels.stop()
                break
            
            error = 3*values[0] + values[1] - values[2] - 3*values[3]
            if error is None:  # detects all black
                self.lost()
                return
            self.I += error
            self.D = error - self.last_error
            self.last_error = error
            pid = self.Kp * error + self.Ki * self.I + self.Kd * self.D

            # Motor speed adjustments for backward movement
            self.lspeed = max(-55, min(0, -self.target_lspeed*3/5 - pid))
            self.rspeed = max(-55, min(0, -self.target_rspeed*3/5 + pid))

            self.wheels.wheel_speed(int(self.lspeed), int(self.rspeed))
            

    def turn_180(self) -> None:
        left_sensor_detected = False
        middle_l_sensor_detected = False
        middle_r_sensor_detected = False

        prev_values = list(self.line_sensors.read()) # initial sensor readings

        while not (left_sensor_detected and middle_l_sensor_detected and middle_r_sensor_detected):
            values = list(self.line_sensors.read()) # read current sensor values

            if not left_sensor_detected and prev_values[0] == 0 and values[0] == 1:
                left_sensor_detected = True
            if not middle_l_sensor_detected and prev_values[1] == 0 and values[1] == 1:
                middle_l_sensor_detected = True
            if not middle_r_sensor_detected and prev_values[2] == 0 and values[2] == 1:
                middle_r_sensor_detected = True

            self.wheels.wheel_speed(-15,15) # keep turning left
            prev_values = values # update previous sensor values
            
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)