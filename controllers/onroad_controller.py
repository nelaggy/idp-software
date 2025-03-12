from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from navigation.navigator import Navigator

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
    def __init__(self, line_sensors, wheels, navigator) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.navigator = navigator

        self.kp = 5
        self.ki = 0
        self.kd = 0

        self.i = 0
        self.d = 0
        self.err = 0
        self.turning = False

        self.lspeed = 0
        self.rspeed = 0
        self.target_lspeed = 100
        self.target_rspeed = 100

    def on_change(self, values: bytearray) -> None:
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        pid = self.kp * err + self.ki * self.i + self.kd * self.d
        self.lspeed = self.target_lspeed - pid
        self.rspeed = self.target_rspeed + pid

        # limit between 0 and 100 (preserves l_speed/r_speed ratio)
        # TODO: consider preserving difference instead
        # if abs(self.lspeed) > 100:
        #     self.rspeed = self.rspeed / abs(self.lspeed) * 100
        #     self.lspeed = self.lspeed / abs(self.lspeed) * 100
        # if abs(self.rspeed) > 100:s
        #     self.lspeed = self.lspeed / abs(self.rspeed) * 100
        #     self.rspeed = self.rspeed / abs(self.rspeed) * 100
        self.wheels.wheel_speed(self.lspeed, self.rspeed)

        if values == b'\x00\x00\x00\x00':
            self.lost()
            return

        # if (values[0] == 1 or values[3] == 1) and not self.turning and (values[1] == 1 and values[2] == 1):
        #     # self.junction()
        #     return

        # if self.turning and (values[0] == 0 and values[3] == 0):
        #     self.lspeed = 100
        #     self.rspeed = 100
        #     self.wheels.wheel_speed(self.lspeed, self.rspeed)
        #     self.turning = False
        #     return
        
    def activate(self) -> None:
        self.line_sensors.set_callback(self.on_change)

    def lost(self) -> None:
        self.wheels.stop()
        # self.lspeed = 100
        # self.rspeed = 100
        # self.wheels.wheel_speed(self.lspeed, self.rspeed)

    def junction(self) -> None:
        self.turning = True
        turn = self.navigator.get_turn()
        if turn == 0:
            self.lspeed = 100
            self.rspeed = 100
            self.wheels.wheel_speed(self.lspeed, self.rspeed)
        elif turn == 1:
            self.lspeed = 100
            self.rspeed = -40
            self.wheels.wheel_speed(self.lspeed, self.rspeed)
        elif turn == 2:
            self.wheels.stop()
        elif turn == 3:
            self.lspeed = -40
            self.rspeed = 100
            self.wheels.wheel_speed(self.lspeed, self.rspeed)
        else:
            # switch to off road control
            return