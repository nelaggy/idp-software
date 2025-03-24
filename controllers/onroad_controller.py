from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from navigation.navigator import Navigator
from machine import Timer

class OnRoadController:
    """
    Class for controller used at junctions apart from when picking up and dropping off boxes
    """
    def __init__(self, line_sensors: LineSensors, wheels: Motors, servo: Servo, navigator: Navigator, on_complete) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.servo = servo
        self.navigator = navigator
        self.on_complete = on_complete

        self.timer = Timer(-1)

        # Initialise PID constants
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

        # Initialise target speeds for left and right wheels
        self.target_lspeed = 90
        self.target_rspeed = 90

    def on_change(self, values: bytearray) -> None:
        """
        Handles sensor value updates and adjusts wheel speed accordingly using a PID controller. Handles turns.

        Arguments:
            values (bytearray): A 4-byte array representing sensor readings.

        Returns:
            None
        """

        # Compute PID correction
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        pid = self.kp * err + (self.ki * self.i)//self.ki_mag + (self.kd * self.d) // self.kd_mag
        lspeed = self.target_lspeed - pid
        rspeed = self.target_rspeed + pid
        self.wheels.wheel_speed(lspeed, rspeed)

        
        # lost if all sensors read black
        if values == b'\x00\x00\x00\x00':
            self.lost()
            return

        # if three sensors read white and not self.turning, go to self.junction
        if (values[0] == 1 or values[3] == 1) and self.turn_dir == 0 and (values[1] == 1 and values[2] == 1) and not self.turning:
            self.junction()
            return
        
        # Update stages during turning
        # turning right
        if self.turning and self.turn_dir == 1:
            if self.turn_stage == 0 and values[3] == 1:
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[3] == 0:
                self.turn_stage += 1
                return
            if self.turn_stage == 2 and values[3] == 1:
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                return
        
        # turning left
        if self.turning and self.turn_dir == 3:
            if self.turn_stage == 0 and values[0] == 1:
                self.turn_stage += 1
                return
            if self.turn_stage == 1 and values[0] == 0:
                self.turn_stage += 1
                return
            if self.turn_stage == 2 and values[0] == 1:
                self.turning = False
                self.turn_stage = 0
                self.turn_dir = 0
                self.target_lspeed = 90
                self.target_rspeed = 90
                return

        # stop turning if outside sensors read black and inside sensors read white for upper right and left junction
        if self.turn_dir <= 0 and self.turning and (values[0] == 0 and values[3] == 0) and (values[1] == 1 and values[2] == 1):
            self.turning = False
            self.target_lspeed = 90
            self.target_rspeed = 90
            self.turn_dir = 0
            return
        
    def activate(self) -> None:
        """
        Activates the OnRoadController.

        Arguments:
            self

        Returns:
            None
        """
        self.line_sensors.set_callback(self.on_change)
        self.turning = False
        self.turn_dir = 0
        self.turn_stage = 0

        self.target_lspeed = 90
        self.target_rspeed = 90

    def lost(self) -> None:
        return

    def junction(self) -> None:
        """
        Adjusts target speed at junction, and if junction is the picking up box station schedules after certain period of time self.entrance_turn()
        """
        turn = self.navigator.get_turn()
        self.turn_dir = turn

        # moves forward for a short period of time before turning into the junction for pikcing up box stations ensuring accurate turn
        if self.navigator.next_node == self.navigator.destination and self.navigator.destination > 2:
            self.timer.init(mode=Timer.ONE_SHOT, period=770, callback=lambda _: self.entrance_turn())
            return
        self.turning = True

        # sets wheel speeds according to turn direction
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
        """
        Handles the entrance turn by monitoring sensor values and advancing the turn stage.

        Arguments:
        - values (bytearray): line sensor readings.
        """
        sign = 1 if self.turn_dir == 1 else -1 # sign = 1 for left turn, sign = -1 for right turn
        idx = 0 if self.turn_dir == 1 else 3 # idx = 0 for left turn, idx = 3 for right turn

        # update stages for turning
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
        """
        Initiates the entrance turn by setting appropriate wheel speeds and registering the sensor callback.
        """
        self.timer.deinit()
        self.turn_stage = 0

        # set wheel speed according to turn direction
        if self.turn_dir == 1: # turn left
            self.wheels.wheel_speed(-100, 100)
        elif self.turn_dir == 3: # turn right
            self.wheels.wheel_speed(100, -100)
        else:
            self.on_complete()
            return
        self.turn_stage = 0
        self.line_sensors.set_callback(self.entrance_turn_handler)
        return  

