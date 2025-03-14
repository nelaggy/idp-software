from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from navigation.navigator import Navigator
from micropython import schedule
from time import sleep_ms

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

        self.kp = 11
        self.ki = 0.01
        self.kd = 0

        self.i = 0
        self.d = 0
        self.err = 0
        self.turning = False
        self.turn_dir = 0
        self.turn_stage = 0

        self.target_lspeed = 90
        self.target_rspeed = 90

    def on_change(self, values: bytearray) -> None:
        
        # if self.turning and self.turn_dir == 1:
        #     err = 3*values[0] + values[1] - values[2] - 2*values[3]
        # elif self.turning and self.turn_dir == 3:
        #     err = 2*values[0] + values[1] - values[2] - 3*values[3]
        # else:
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        pid = self.kp * err + self.ki * self.i + self.kd * self.d
        lspeed = self.target_lspeed - pid
        rspeed = self.target_rspeed + pid
        self.wheels.wheel_speed(lspeed, rspeed)


        if values == b'\x00\x00\x00\x00':
            self.lost()
            return

        if (values[0] == 1 or values[3] == 1) and self.turn_dir == 0 and (values[1] == 1 and values[2] == 1):
            self.junction()
            return
        
        if self.turning and self.turn_dir == 1:
            if self.turn_stage == 0 and values[3] == 1:
                print('check 1')
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[3] == 0:
                print('check 2')
                self.turn_stage += 1
                return
            if self.turn_stage == 2 and values[3] == 1:
                print('end turn')
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                # if self.navigator.next_node == self.navigator.destination and self.navigator.node != 1:
                #     self.on_complete()
                return
        
        if self.turning and self.turn_dir == 3:
            if self.turn_stage == 0 and values[0] == 1:
                print('check 1')
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[0] == 0:
                print('check 2')
                self.turn_stage += 1
                return
            if self.turn_stage == 2 and values[0] == 1:
                print('end turn')
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                # if self.navigator.next_node == self.navigator.destination and self.navigator.node != 1:
                #     self.on_complete()
                return

        if self.turn_dir <= 0 and self.turning and (values[0] == 0 and values[3] == 0) and (values[1] == 1 and values[2] == 1):
            print('end turn')
            self.turning = False
            self.target_lspeed = 90
            self.target_rspeed = 90
            self.turn_dir = 0
            # if self.navigator.next_node == self.navigator.destination and self.navigator.node != 1:
            #     self.on_complete()
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
        print('turn: ', turn)
        if self.navigator.next_node == self.navigator.destination and self.navigator.destination > 2:
            print('arriving')
            schedule(self.entrance_turn, 900)
            return
        self.turning = True
        print('not arriving')
        if self.navigator.node == 12 or self.navigator.node == 15:
            self.turn_stage = 1
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
            print('off')
            self.on_complete()

    def entrance_turn_handler(self, values: bytearray) -> None:
        sign = 1 if self.turn_dir == 1 else -1
        idx = 0 if self.turn_dir == 1 else 3
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
        

    def entrance_turn(self, delay):
        self.servo.drop()
        sleep_ms(delay)
        self.turn_stage = 0
        if self.turn_dir == 1:
            print('on left')
            self.wheels.wheel_speed(-90, 90)
        elif self.turn_dir == 3:
            print('on right')
            self.wheels.wheel_speed(90, -90)
        else:
            print('straight ahead')
            self.on_complete()
            return
        self.turn_stage = 0
        self.line_sensors.set_callback(self.entrance_turn_handler)
        return  

