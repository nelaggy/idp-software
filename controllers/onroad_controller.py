from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from navigation.navigator import Navigator
from micropython import schedule

# cases
# 0000 - lost: move in random directions
# 0001 - way too far left
# 0010 - too far left: speed up left/slow down right
# 0011 - right turn, too far left
# 0100 - too far right: speed up right/slow down left
# 0101 - right turn, too far right
# 0110 - on the line: move forward
# 0111 - right turn
# 1000 - way too far right
# 1001 - ????
# 1010 - left turn, too far left
# 1011 - T-junction or crossroad, too far left
# 1100 - left turn, too far right
# 1101 - T-junction or crossroad, too far right
# 1110 - left turn
# 1111 - T-junction or crossroad


class OnRoadController:
    def __init__(self, line_sensors: LineSensors, wheels: Motors, servo: Servo, navigator: Navigator, on_complete) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.servo = servo
        self.navigator = navigator
        self.on_complete = on_complete

        self.kp = 10
        self.ki = 0.1
        self.kd = 2

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
        pid = self.kp * err + self.ki * self.i + self.kd * self.d
        lspeed = self.target_lspeed - pid
        rspeed = self.target_rspeed + pid

        # limit between 0 and 100 (preserves l_speed/r_speed ratio)
        # TODO: consider preserving difference instead
        # if abs(self.lspeed) > 100:
        #     self.rspeed = self.rspeed / abs(self.lspeed) * 100
        #     self.lspeed = self.lspeed / abs(self.lspeed) * 100
        # if abs(self.rspeed) > 100:s
        #     self.lspeed = self.lspeed / abs(self.rspeed) * 100
        #     self.rspeed = self.rspeed / abs(self.rspeed) * 100
        self.wheels.wheel_speed(lspeed, rspeed)


        if values == b'\x00\x00\x00\x00':
            self.lost()
            return

        if (values[0] == 1 or values[3] == 1) and self.turn_dir == 0 and (values[1] == 1 and values[2] == 1):
            self.junction()
            return
        
        # if self.turn_dir != 0:
        #     print(values)
        
        if self.turn_dir == 1:
            if self.turn_stage == 0 and values[3] == 0:
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[3] == 1:
                print('end turn')
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                return
        
        if self.turn_dir == 3:
            if self.turn_stage == 0 and values[0] == 0:
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[0] == 1:
                print('end turn')
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                return

        if self.turn_dir <= 0 and self.turning and (values[0] == 0 and values[3] == 0) and (values[1] == 1 and values[2] == 1):
            print('end turn')
            self.turning = False
            self.target_lspeed = 90
            self.target_rspeed = 90
            self.turn_dir = 0
            return
        
        if self.turning:
            print(lspeed, rspeed)
        
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)

    def lost(self) -> None:
        # self.wheels.stop()
        self.target_lspeed = 90
        self.target_rspeed = 90

    def junction(self) -> None:
        turn = self.navigator.get_turn()
        self.turning = True
        self.turn_dir = turn
        if self.navigator.next_node == self.navigator.destination and self.navigator.destination != 1:
            schedule(self.servo.drop, None)
            pass
        print('turn: ', turn)
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