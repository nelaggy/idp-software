from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from micropython import schedule
from time import sleep_ms

class OffRoadController:
    def __init__(self, line_sensors: LineSensors, wheels: Motors, servo: Servo, on_complete, pickup_cb) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.on_complete = on_complete
        self.pickup_cb = pickup_cb

        self.kp = 15
        self.ki = 0.0015
        self.kd = 0

        self.i = 0
        self.d = 0
        self.err = 0

        self.target_lspeed = 50
        self.target_rspeed = 50

        # Servo
        self.servo = servo

        self.stage = 0 # 0 approaching, 1 picking up, 2 reversing, 3 exit turn
        self.turn_stage = 0
        self.turn_stop = 0
        self.turn_dir = 0

        self.turn_180_flag = False
        self.reverse_flag = False
        self.pickup_flag = False
        self.drop_flag = False
        
    def calc_pid(self, values):
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        pid = self.kp * err + self.ki * self.i + self.kd * self.d
        return pid
    
    def on_change(self, values: bytearray) -> None:
        if not self.reverse_flag:
            
            pid = self.calc_pid(values)

            lspeed = self.target_lspeed - pid
            rspeed = self.target_rspeed + pid

            self.wheels.wheel_speed(lspeed, rspeed)

            if values == b'\x00\x00\x00\x00':
                self.lost()
                return
            
            if values == b'\1\1\1\1':
                print('detected all white, pickup activated')
                self.pickup_box()
        
        else: # reverse
            pid = self.calc_pid(values)

            # Motor speed adjustments for backward movement
            lspeed = self.target_lspeed - pid
            rspeed = self.target_rspeed + pid

            self.wheels.wheel_speed(lspeed, rspeed)

    # def on_change(self, values: bytearray) -> None:
    #     print(self.stage)
    #     if self.stage == 1:
    #         print('picking up')
    #         self.wheels.stop()
    #         return
        
    #     err = 3*values[0] + values[1] - values[2] - 3*values[3]
    #     self.i += err
    #     self.d = err - self.err
    #     self.err = err
    #     pid = self.kp * err + self.ki * self.i + self.kd * self.d
    #     lspeed = self.target_lspeed - pid
    #     rspeed = self.target_rspeed + pid
    #     self.wheels.wheel_speed(lspeed, rspeed)

    #     if self.stage == 0 and values == b'\x00\x00\x00\x00':
    #         print('lost')
    #         self.lost()
    #         return
        
    #     if self.stage == 2:
    #         print('reversing')
    #         self.stage = 3
    #         schedule(self.stop_reversing, 1000)
    #         return
        
    #     if self.stage == 0 and values == b'\1\1\1\1':
    #         self.stage = 1
    #         print('detected all white, pickup activated')
    #         self.wheels.stop()
    #         schedule(self.pickup_box, None)
    #         return
            
    
    def activate(self, drop_flag) -> None:
        self.line_sensors.set_callback(self.on_change)
        self.target_lspeed = 50
        self.target_rspeed = 50
        self.turn_180_flag = False
        self.reverse_flag = False
        self.pickup_flag = False
        self.drop_flag = drop_flag
        self.servo.drop()

    def lost(self) -> None:
        self.wheels.stop()

    def stop_reversing(self, delay):
        sleep_ms(delay)
        self.wheels.stop()
        self.turn_180_flag = True
        self.turn_stage = 0
        self.turn_dir = 0
        if not self.drop_flag:
            self.pickup_cb()
        return
    
    def exit_turn(self, turn_dir: int) -> None:
        print('exit turn')
        self.turn_stage = 0
        self.turn_dir = turn_dir
        self.turn_stop = abs(turn_dir)
        print('turns', self.turn_stop)
        if self.turn_dir < 0:
            self.wheels.wheel_speed(-50, 50)
        elif self.turn_dir > 0:
            self.wheels.wheel_speed(50, -50)
        else:
            self.on_complete()
            return
        self.line_sensors.set_callback(self.exit_turn_handler)
        return
    
    def exit_turn_handler(self, values: bytearray) -> None:
        sign = 1 if self.turn_dir < 0 else -1
        idx = 0 if self.turn_dir < 0 else 3
        print(self.turn_stage)
        if self.turn_stage % 3 == 0 and values[idx] == 1:
            self.turn_stage += 1
            return
        if self.turn_stage % 3 == 1 and values[idx + sign] == 1:
            self.turn_stage += 1
            return
        if self.turn_stage % 3 == 2 and values[idx + 2*sign] == 1:
            self.turn_stage += 1
            if self.turn_stage == 3*self.turn_stop:
                self.wheels.stop()
                self.on_complete()
                return
        
    
    def pickup_box(self) -> None:
        self.wheels.stop()
        if not self.drop_flag:
            self.servo.lift()
        self.target_lspeed = -90
        self.target_rspeed = -90
        self.wheels.wheel_speed(self.target_lspeed, self.target_rspeed)
        self.stage = 2
        self.reverse_flag = True
        schedule(self.stop_reversing, 1000)
    