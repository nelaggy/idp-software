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

#not using this anymore
class OffRoadControllerPrev:
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
    
    def on_block(self):
        pass

        
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
            if values == b'\x00\x00\x00\x00': # detects all black
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
            if values == b'\x00\x00\x00\x00': # detects all black
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

        
        
class OffRoadController:
    def __init__(self, line_sensors, wheels, on_complete) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.on_complete = on_complete

        # PID constants (Tuning required)
        self.Kp = 5  # Proportional gain
        self.Ki = 0  # Integral gain (usually small or 0)
        self.Kd = 0  # Derivative gain

        # PID variables
        self.D = 0
        self.I = 0
        self.last_error = 0
        self.integral = 0

        self.lspeed = 0
        self.rspeed = 0
        self.target_lspeed = 50
        self.target_rspeed = 50

        # Servo
        self.servo = Servo()

        # flags
        self.turn_180_flag = False
        self.reverse_flag = False
        self.pickup_flag = False

        # for turn 180
        self.prev_values = b'\1\1\1\1'
        self.left_sensor_detected = False
        self.middle_l_sensor_detected = False
        self.middle_r_sensor_detected = False
    
    def on_change(self, values: bytearray) -> None:
        '''Handles sensor input updates and applies PID control'''

        if not self.pickup_flag: # follow line if not self.pickup_flag activated
            if not self.reverse_flag: # moving forward
                error = 3*values[0] + values[1] - values[2] - 3*values[3]
                if values == b'\x00\x00\x00\x00': # detects all black
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
            
            else: # reverse
                error = 3*values[0] + values[1] - values[2] - 3*values[3]
                if values == b'\x00\x00\x00\x00': # detects all black
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

                if values == b'\1\1\1\1' and not self.turn_180_flag:
                    self.turn_180_flag = True
                    schedule(time.sleep,2) # move backwards for further two seconds
                    schedule(self.line_sensors.set_callback,self.turn_180)
        
        
        else: # pickup box if self.pickup activated
            self.wheels.stop()
            schedule(self.pickup_box, None)
    
    def on_block(self):
        self.wheels.stop()
        self.pickup_flag = True
        schedule(self.pickup_box,None)
    

    def lost(self) -> None:
        print('lost')
        self.wheels.stop()
    
    def pickup_box(self) -> None:
        #self.servo.lift()
        self.pickup_flag = False
        self.reverse_flag = True

    def turn_180(self, values: bytearray) -> None:

        if not (self.left_sensor_detected and self.middle_l_sensor_detected and self.middle_r_sensor_detected):

            if not self.left_sensor_detected and self.prev_values[0] == 0 and values[0] == 1:
                self.left_sensor_detected = True
            if not self.middle_l_sensor_detected and self.prev_values[1] == 0 and values[1] == 1:
                self.middle_l_sensor_detected = True
            if not self.middle_r_sensor_detected and self.prev_values[2] == 0 and values[2] == 1:
                self.middle_r_sensor_detected = True

            self.wheels.wheel_speed(-15,15) # keep turning left
            self.prev_values = values # update previous sensor values

        else:
            self.on_complete
            
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)