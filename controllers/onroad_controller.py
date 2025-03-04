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
    def __init__(self) -> None:
        self.sensors = LineSensors(self.on_change)
        self.wheels = Motors()
        self.navigator = Navigator()

    def on_change(self, values: bytearray) -> None:
        if values == b'\0\0\0\0':
            self.lost()
            return
        if values == b'\0\0\0\1':
            self.off_left()
            return
        if values == b'\0\0\1\0':
            self.off_left()
            return
        if values == b'\0\0\1\1':
            self.junction()
            return
        if values == b'\0\1\0\0':
            self.off_right()
            return
        if values == b'\0\1\0\1':
            self.junction()
            return
        if values == b'\0\1\1\0':
            self.on_line()
            return
        if values == b'\0\1\1\1':
            self.junction()
            return
        if values == b'\1\0\0\0':
            self.off_right()
            return
        if values == b'\1\0\0\1':
            return
        if values == b'\1\0\1\0':
            self.junction()
            return
        if values == b'\1\0\1\1':
            self.junction()
            return
        if values == b'\1\1\0\0':
            self.junction()
            return
        if values == b'\1\1\0\1':
            self.junction()
            return
        if values == b'\1\1\1\0':
            self.junction()
            return
        if values == b'\1\1\1\1':
            self.junction()
            return
        

    def lost(self) -> None:
        print('lost')
        self.wheels.stop()

    def on_line(self) -> None:
        print('on line')
        self.wheels.wheel_speed(100, 100)

    def off_right(self) -> None:
        print('off right')
        self.wheels.wheel_speed(100, 90)

    def off_left(self) -> None:
        print('off left')
        self.wheels.wheel_speed(90, 100)

    def junction(self) -> None:
        print('junction')
        turn = self.navigator.get_turn()
        if turn == 0:
            self.wheels.wheel_speed(100, 100)
        elif turn == 1:
            self.wheels.wheel_speed(100, 20)
        elif turn == 2:
            self.wheels.stop()
        elif turn == 3:
            self.wheels.wheel_speed(20, 100)