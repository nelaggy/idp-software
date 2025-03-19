from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from navigation.navigator import Navigator
from machine import Timer
from micropython import schedule

class OnRoadController:
    def __init__(self, line_sensors: LineSensors, wheels: Motors, servo: Servo, navigator: Navigator, on_complete) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.servo = servo
        self.navigator = navigator
        self.on_complete = on_complete

        self.timer = Timer(-1)

        self.kp = 20
        self.ki = 1
        self.ki_mag = 10
        self.kd = 2
        self.kd_mag = 100

        self.i = 0
        self.d = 0
        self.err = 0
        self.turning = False
        self.turn_dir = 0
        self.turn_stage = 0

        self.target_lspeed = 90
        self.target_rspeed = 90

    def on_change(self, values: bytearray) -> None:
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        pid = self.kp * err + (self.ki * self.i)//self.ki_mag + (self.kd * self.d) // self.kd_mag
        lspeed = self.target_lspeed - pid
        rspeed = self.target_rspeed + pid
        self.wheels.wheel_speed(lspeed, rspeed)


        if values == b'\x00\x00\x00\x00':
            self.lost()
            return

        if (values[0] == 1 or values[3] == 1) and self.turn_dir == 0 and (values[1] == 1 and values[2] == 1) and not self.turning:
            self.junction()
            return
        
        if self.turning and self.turn_dir == 1:
            if self.turn_stage == 0 and values[3] == 1:
                # schedule(print, 'left stage 1')
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[3] == 0:
                # schedule(print, 'left stage 2')
                self.turn_stage += 1
                return
            if self.turn_stage == 2 and values[3] == 1:
                # schedule(print, 'left stage 3')
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                return
        
        if self.turning and self.turn_dir == 3:
            if self.turn_stage == 0 and values[0] == 1:
                # schedule(print, 'right stage 1')
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[0] == 0:
                # schedule(print, 'right stage 2')
                self.turn_stage += 1
                return
            if self.turn_stage == 2 and values[0] == 1:
                # schedule(print, 'right stage 3')
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                return

        if self.turn_dir <= 0 and self.turning and (values[0] == 0 and values[3] == 0) and (values[1] == 1 and values[2] == 1):
            self.turning = False
            self.target_lspeed = 90
            self.target_rspeed = 90
            self.turn_dir = 0
            return
        
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)
        self.turning = False
        self.turn_dir = 0
        self.turn_stage = 0

        self.target_lspeed = 90
        self.target_rspeed = 90

    def lost(self) -> None:
        return

    def junction(self) -> None:
        turn = self.navigator.get_turn()
        self.turn_dir = turn
        # schedule(print,'turn: '+ str(turn) + ' node: '+ str(self.navigator.node))
        if self.navigator.next_node == self.navigator.destination and self.navigator.destination > 2:
            self.timer.init(mode=Timer.ONE_SHOT, period=770, callback=lambda _: self.entrance_turn())
            return
        self.turning = True
        # if self.navigator.node == 12 or self.navigator.node == 15:
        #     self.turn_stage = 1
        if turn == 0:
            self.target_lspeed = 90
            self.target_rspeed = 90
        elif turn == 1:
            self.target_lspeed = 0
            self.target_rspeed = 90
        elif turn == 2:
            self.wheels.stop()
        elif turn == 3:
            self.target_lspeed = 90
            self.target_rspeed = 0
        else:
            self.on_complete()

    def entrance_turn_handler(self, values: bytearray) -> None:
        sign = 1 if self.turn_dir == 1 else -1
        idx = 0 if self.turn_dir == 1 else 3
        if self.turn_stage == 0 and values[idx] == 1:
            self.turn_stage += 1
            return
        if self.turn_stage == 1 and values[idx + sign] == 1:
            self.turn_stage += 1
            return
        if self.turn_stage == 2 and values[idx + 2*sign] == 1:
            self.wheels.stop()
            self.on_complete()
            return
        

    def entrance_turn(self):
        self.timer.deinit()
        # self.servo.drop()
        self.turn_stage = 0
        if self.turn_dir == 1:
            self.wheels.wheel_speed(-100, 100)
        elif self.turn_dir == 3:
            self.wheels.wheel_speed(100, -100)
        else:
            self.on_complete()
            return
        self.turn_stage = 0
        self.line_sensors.set_callback(self.entrance_turn_handler)
        return  

