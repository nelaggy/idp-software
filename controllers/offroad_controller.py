from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from micropython import schedule
from machine import Timer

class OffRoadController:
    def __init__(self, line_sensors: LineSensors, wheels: Motors, servo: Servo, on_complete, pickup_cb) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.on_complete = on_complete
        self.pickup_cb = pickup_cb

        self.timer = Timer(-1)

        self.kp = 25
        self.kp_reverse = 25
        self.ki = 1
        self.ki_reverse = 1
        self.ki_mag = 10
        self.kd = 2
        self.kd_reverse = 2
        self.kd_mag = 100

        self.i = 0
        self.d = 0
        self.err = 0

        self.target_lspeed = -50
        self.target_rspeed = -50

        # Servo
        self.servo = servo

        self.drop_flag = False

        self.stage = 0 # 0 approaching, 1 picking up, 2 reversing, 3 exit turn
        self.turn_stage = 0
        self.turn_stop = 0
        self.turn_dir = 0

    def on_change(self, values: bytearray) -> None:
        if self.stage == 2: # picking up
            return
        
        if self.stage == 1 and values == b'\x00\x00\x00\x00':
            self.lost()
            return
        
        if self.stage == 1 and values == b'\1\1\1\1':
            self.wheels.stop()
            self.pickup_box()
            return
        
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        if self.stage == 1:
            pid = self.kp * err + (self.ki * self.i)//self.ki_mag + (self.kd * self.d) // self.kd_mag
        else:
            pid = self.kp_reverse * err + (self.ki_reverse * self.i)//self.ki_mag + (self.kd_reverse * self.d) // self.kd_mag
        lspeed = self.target_lspeed - pid
        rspeed = self.target_rspeed + pid
        self.wheels.wheel_speed(lspeed, rspeed)
            
    
    def activate(self, drop_flag, reverse_flag) -> None:
        self.line_sensors.set_callback(self.on_change)
        self.drop_flag = drop_flag
        if reverse_flag:
            self.target_lspeed = -70
            self.target_rspeed = -70
            
            self.timer.init(mode=Timer.ONE_SHOT, period=500, callback=lambda _: self.move_forward())
            return
        self.target_lspeed = 50
        self.target_rspeed = 50
        self.stage = 1
        self.servo.drop()

    def lost(self) -> None:
        self.wheels.stop()

    def move_forward(self):
        self.servo.drop()
        self.stage = 1
        self.target_lspeed = 50
        self.target_rspeed = 50

    def stop_reversing(self):
        # schedule(print,'stop reversing, exit '+ str(self.turn_dir))
        if self.drop_flag:
            self.servo.set_angle(90)
        self.timer.deinit()
        self.wheels.stop()
        self.turn_stage = 0
        self.stage = 4
        if self.turn_dir < 0:
            self.wheels.wheel_speed(-90, 90)
        elif self.turn_dir > 0:
            self.wheels.wheel_speed(90, -90)
        else:
            self.on_complete()
            return
        self.line_sensors.set_callback(self.exit_turn_handler)
        return
    
    
    def exit_turn(self, turn_dir: int, reverse_delay: int) -> None:
        # print('reversing')
        self.turn_stage = 0
        self.turn_dir = turn_dir
        self.turn_stop = abs(self.turn_dir)
        self.target_lspeed = -70
        self.target_rspeed = -70
        # self.wheels.wheel_speed(self.target_lspeed, self.target_rspeed)
        self.stage = 3
        self.timer.init(mode=Timer.ONE_SHOT, period=reverse_delay, callback=lambda _: self.stop_reversing())
    
    def exit_turn_handler(self, values: bytearray) -> None:
        sign = 1 if self.turn_dir < 0 else -1
        idx = 0 if self.turn_dir < 0 else 3
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
        self.stage = 2
        self.wheels.stop()
        if not self.drop_flag:
            self.servo.lift()
        schedule(lambda _: self.pickup_cb(), None)
    