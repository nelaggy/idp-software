from machine import Pin, I2C
from dummy.vl53l0x import VL53L0X
from hardware.servo import Servo
from micropython import schedule
import time


class OffRoadController2:
    #def __init__(self, line_sensors, wheels, on_complete) -> None:
    def __init__(self, line_sensors, wheels) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        #self.on_complete = on_complete

        # PID constants (Tuning required)
        self.Kp = 15  # Proportional gain
        self.Ki = 0.0015  # Integral gain (usually small or 0)
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
        self.drop_box_flag = False

        # for turn 180
        self.prev_values = b'\1\1\1\1'
        self.left_sensor_detected = False
        self.middle_l_sensor_detected = False
        self.middle_r_sensor_detected = False

    def calc_pid(self, values):
        error = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.I += error
        self.D = error - self.last_error
        self.last_error = error
        pid = self.Kp * error + self.Ki * self.I + self.Kd * self.D
        return pid
    
    def on_change(self, values: bytearray) -> None:
        '''Handles sensor input updates and applies PID control'''
        # if values == b'\x00\x00\x00\x00': # detects all black
        #     self.lost()
        #     return
        
        #if not self.drop_box: # at pickup station
        if not self.pickup_flag: # follow line if not self.pickup_flag activated
            if not self.reverse_flag: # moving forward
                
                pid = self.calc_pid(values)

                # Motor speed adjustments
                self.lspeed = self.target_lspeed - pid
                self.rspeed = self.target_rspeed + pid

                self.wheels.wheel_speed(self.lspeed, self.rspeed)

                if values == b'\1\1\1\1':
                    print('detected all white, pickup activated')
                    self.pickup_flag = True

            
            else: # reverse
                pid = self.calc_pid(values)

                # Motor speed adjustments for backward movement
                self.lspeed = -self.target_lspeed - pid
                self.rspeed = -self.target_rspeed + pid

                self.wheels.wheel_speed(self.lspeed, self.rspeed)

                if values == b'\1\1\1\1' and not self.turn_180_flag:
                    self.turn_180_flag = True
                    print("Turn 180 starting")
                    schedule(time.sleep,2) # move backwards for further two seconds
                    schedule(self.line_sensors.set_callback,self.turn_180)
        
        
        else: # pickup box if self.pickup activated
            self.wheels.stop()
            schedule(self.pickup_box, None)

        '''
        else: # drop box
            if not self.drop_box_flag: # follow line if not self.drop_box_flag activated
                if not self.reverse_flag: # moving forward
                    if values == b'\1\1\1\1':
                        self.drop_box_flag = True

                    pid = self.calc_pid(values)

                    # Motor speed adjustments
                    self.lspeed = max(0, min(self.target_lspeed+5, self.target_lspeed - pid))
                    self.rspeed = max(0, min(self.target_rspeed+5, self.target_rspeed + pid))

                    self.wheels.wheel_speed(int(self.lspeed), int(self.rspeed))
                
                else: # reverse
                    pid = self.calc_pid(values)

                    # Motor speed adjustments for backward movement
                    self.lspeed = max(-self.target_lspeed-5, min(0, -self.target_lspeed - pid))
                    self.rspeed = max(-self.target_rspeed-5, min(0, -self.target_rspeed + pid))

                    self.wheels.wheel_speed(int(self.lspeed), int(self.rspeed))

                    if values == b'\1\1\1\1' and not self.turn_180_flag:
                        self.turn_180_flag = True
                        schedule(time.sleep,2) # move backwards for further two seconds
                        schedule(self.servo.lift90,None)
                        schedule(self.line_sensors.set_callback,self.turn_180)
            
            
            else: # drop box if self.drop_box_flag activated
                self.wheels.stop()
                schedule(self.drop_box, None)'
        '''
    
    # def on_block(self):
    #     self.wheels.stop()
    #     self.pickup_flag = True
    #     schedule(self.pickup_box,None)
    
    def lost(self) -> None:
        self.wheels.stop()
    
    def pickup_box(self, _) -> None:
        self.servo.lift()
        self.pickup_flag = False
        self.reverse_flag = True
    '''
    def drop_box(self, _) -> None:
        self.servo.drop()
        self.drop_box_flag = False
        self.reverse_flag = True
    '''
    def turn_180(self, values: bytearray) -> None:

        if not (self.left_sensor_detected and self.middle_l_sensor_detected and self.middle_r_sensor_detected):

            if not self.left_sensor_detected and self.prev_values[0] == 0 and values[0] == 1:
                self.left_sensor_detected = True
                print('self.left_sensor_detected is True')
            if not self.middle_l_sensor_detected and self.prev_values[1] == 0 and values[1] == 1:
                self.middle_l_sensor_detected = True
                print('self.middle_l_sensor_detected is True')
            if not self.middle_r_sensor_detected and self.prev_values[2] == 0 and values[2] == 1:
                self.middle_r_sensor_detected = True
                print('self.middle_r_sensor_detected is True')

            self.wheels.wheel_speed(-15,15) # keep turning left
            self.prev_values = values # update previous sensor values

        if self.left_sensor_detected and self.middle_l_sensor_detected and self.middle_r_sensor_detected:
            self.wheels.stop()
            # not sure if we need this
            self.left_sensor_detected = False
            self.middle_l_sensor_detected = False
            self.middle_r_sensor_detected = False
            self.turn_180_flag = False
            self.reverse_flag = False

            #self.on_complete()
            print("Turn 180 complete")
            
    def activate(self) -> None:
        #self.drop_box = drop_box
        #self.servo.drop() # get ready to lift box # need to fix this
        self.line_sensors.set_callback(self.on_change)


class OffRoadController:
    def __init__(self, line_sensors, wheels, on_complete) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.on_complete = on_complete

        self.kp = 5
        self.ki = 0.0015
        self.kd = 0

        self.i = 0
        self.d = 0
        self.err = 0

        self.target_lspeed = 50
        self.target_rspeed = 50

        # Servo
        self.servo = Servo()

        # flags
        self.turn_180_flag = False
        self.reverse_flag = False
        self.pickup_flag = False
        self.drop_box_flag = False
        self.activate_flag = True

        # for turn 180
        self.prev_values = b'\1\1\1\1'
        self.left_sensor_detected = False
        self.middle_l_sensor_detected = False
        self.middle_r_sensor_detected = False
        

    
    def calc_pid(self, values):
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        pid = self.kp * err + self.ki * self.i + self.kd * self.d
        return pid
    

    def on_change(self, values: bytearray) -> None:
        if not self.reverse_flag:

            if self.activate_flag:
                self.activate_flag = False
                schedule(self.servo.drop,None)
            
            pid = self.calc_pid(values)

            lspeed = self.target_lspeed - pid
            rspeed = self.target_rspeed + pid

            self.wheels.wheel_speed(lspeed, rspeed)

            if values == b'\x00\x00\x00\x00':
                self.lost()
                return
            
            if values == b'\1\1\1\1':
                print('detected all white, pickup activated')
                schedule(self.pickup_box, None)
        
        else: # reverse
            pid = self.calc_pid(values)

            # Motor speed adjustments for backward movement
            self.lspeed = -self.target_lspeed - pid
            self.rspeed = -self.target_rspeed + pid

            self.wheels.wheel_speed(self.lspeed, self.rspeed)

            if values == b'\1\1\1\1' and not self.turn_180_flag:
                self.turn_180_flag = True
                schedule(self.turn_180,1000)
    
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)

    def lost(self) -> None:
        self.wheels.stop()
    
    def turn_180(self, delay):
        time.sleep_ms(delay)
        self.turn_stage = 0
        self.wheels.wheel_speed(-50, 50)
        # if self.turn_dir == 1:
        #     print('on left')
        #     self.wheels.wheel_speed(-50, 50)
        # elif self.turn_dir == 3:
        #     print('on right')
        #     self.wheels.wheel_speed(50, -50)
        # else:
        #     print('straight ahead')
        #     self.on_complete()
        #     return
        self.line_sensors.set_callback(self.uturn_handler)
        return
    
    def uturn_handler(self, values: bytearray) -> None:
        sign = 1
        idx = 0
        if self.turn_stage == 0 and values[idx] == 1:
            print('check 1')
            self.turn_stage += 1
            return
        if self.turn_stage == 1 and values[idx + sign] == 1:
            print('check 2')
            self.turn_stage += 1
            return
        if self.turn_stage == 2 and values[idx + 2*sign] == 1:
            print('turn complete')
            self.wheels.stop()
            self.on_complete()
            return
        
    
    def pickup_box(self, _) -> None:
        self.wheels.stop()
        self.servo.lift()
        self.reverse_flag = True
    