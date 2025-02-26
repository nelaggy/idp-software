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


class LineTracker:
    def __init__(self) -> None:
        pass

    def lost(self) -> None:
        pass

    def on_line(self) -> None:
        pass

    def off_right(self) -> None:
        pass

    def off_left(self) -> None:
        pass

    def junction(self) -> None:
        pass