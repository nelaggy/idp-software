from hardware.line_sensor import LineSensors
from hardware.motor import Motors
from hardware.servo import Servo
from micropython import schedule
from machine import Timer

# OffRoadController used at junctions when picking up and dropping off boxes
class OffRoadController:
    def __init__(self, line_sensors: LineSensors, wheels: Motors, servo: Servo, on_complete, pickup_cb) -> None:
        self.line_sensors = line_sensors
        self.wheels = wheels
        self.on_complete = on_complete
        self.pickup_cb = pickup_cb

        self.timer = Timer(-1) # constructs virtual timer

        # Initialise PID constants for forward and reverse
        self.kp = 30
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

        # Initialise target speeds for left and right wheels
        self.target_lspeed = -90
        self.target_rspeed = -90

        self.servo = servo

        self.drop_flag = False

        # Initialise stage of the controller
        self.stage = 0 # 0: not active, 1: moving forward, 2: picking up box, 3: reversing, 4: turning 
        self.turn_stage = 0 # 0: not turning, 1: turning
        self.turn_stop = 0
        self.turn_dir = 0

    def on_change(self, values: bytearray) -> None:
        """
        Handles sensor value updates and adjusts wheel speed accordingly using a PID controller.

        Arguments:
            values (bytearray): A 4-byte array representing sensor readings.

        Returns:
            None
        """
        if self.stage == 2:
            return
        
        if self.stage == 1 and values == b'\x00\x00\x00\x00':
            self.lost()
            return
        
        if self.stage == 1 and values == b'\1\1\1\1':
            self.wheels.stop()
            self.pickup_box()
            return
        
        # Compute PID correction using different parameters for forward and reverse movement
        err = 3*values[0] + values[1] - values[2] - 3*values[3]
        self.i += err
        self.d = err - self.err
        self.err = err
        if self.stage == 1:
            pid = self.kp * err + (self.ki * self.i)//self.ki_mag + (self.kd * self.d) // self.kd_mag
        else:
            pid = self.kp_reverse * err + (self.ki_reverse * self.i)//self.ki_mag + (self.kd_reverse * self.d) // self.kd_mag
        
        # Adjust wheel speeds based on PID correction
        lspeed = self.target_lspeed - pid
        rspeed = self.target_rspeed + pid
        self.wheels.wheel_speed(lspeed, rspeed)
            
    
    def activate(self, drop_flag, reverse_flag, delay=500) -> None:
        """
        Activates the OffRoadController, either reversing first or moving forward directly.

        Arguments:
            drop_flag (bool): Determines whether the robot should drop an item.
            reverse_flag (bool): Indicates if the robot should reverse before moving forward.
            delay (int, optional): Delay in milliseconds before moving forward after reversing. Default is 500ms.

        Returns:
            None
        """
        self.line_sensors.set_callback(self.on_change) # set line sensor callback to on_change
        self.drop_flag = drop_flag
        # moves backwards before picking up box in tight junctions
        if reverse_flag:
            self.target_lspeed = -90
            self.target_rspeed = -90
            
            self.timer.init(mode=Timer.ONE_SHOT, period=delay, callback=lambda _: self.move_forward())
            return
        self.target_lspeed = 70
        self.target_rspeed = 70
        self.stage = 1 # forward movement
        self.servo.drop() # drops servo

    def lost(self) -> None:
        self.wheels.stop()

    def move_forward(self):
        """
        Initiates forward movement after changing from OnRoadController to OffRoadController.

        Arguments:
            self

        Returns:
            None

        Functionality:
        - Drops servo.
        - sets self.stage to 1 (forward movement).
        """
        self.servo.drop()
        self.stage = 1
        self.target_lspeed = 70
        self.target_rspeed = 70

    def stop_reversing(self):
        """
        Stops the reversing motion and initiates a turn if required.

        Arguments:
            None

        Returns:
            None
        """
        # set servo to 90 degrees if after dropping box
        if self.drop_flag:
            self.servo.set_angle(90)
        self.timer.deinit()
        self.wheels.stop()
        self.turn_stage = 0 # reset turn stage
        self.stage = 4 # turning

        # set wheel speed for turning depending on turn direction
        if self.turn_dir < 0:
            self.wheels.wheel_speed(-100, 100)
        elif self.turn_dir > 0:
            self.wheels.wheel_speed(100, -100)
        else:
            self.on_complete()
            return
        self.line_sensors.set_callback(self.exit_turn_handler)
        return
    
    
    def exit_turn(self, turn_dir: int, reverse_delay: int) -> None:
        """
        Initiates the exit turn process by setting up turn parameters and a reverse timer.

        Arguments:
            turn_dir (int): The direction of the turn.
            reverse_delay (int): The delay (in milliseconds) before stopping the reversing motion.

        Returns:
            None
        """
        self.turn_stage = 0 # intialise turn stage
        self.turn_dir = turn_dir
        self.turn_stop = abs(self.turn_dir)
        self.target_lspeed = -90
        self.target_rspeed = -90
        self.stage = 3 # reversing
        self.timer.init(mode=Timer.ONE_SHOT, period=reverse_delay, callback=lambda _: self.stop_reversing()) # set timer to stop reversing after reverse_delay milliseconds
    
    def exit_turn_handler(self, values: bytearray) -> None:
        """
        Handles the turn exit process by updating the turn stage based on sensor values.

        Arguments:
            self
            values (bytearray): line sensor readings

        Returns:
            None

        Functionality:
        - Determines the direction sign based on self.turn_dir (negative for left, positive for right).
        - Initialise index (idx) to read sensor values depending on turn direction.
        - Progresses self.turn_stage if specific sensor conditions are met.
        - Stops the wheels and triggers self.on_complete (switches to OnRoadController) when the turn is fully executed.
        """
        sign = 1 if self.turn_dir < 0 else -1 # left turn: 1, right turn: -1
        idx = 0 if self.turn_dir < 0 else 3 # left turn: 0, right turn: 3
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
        """
        Handles picking up the box.

        Arguments:
            self

        Returns:
            None
        """
        self.stage = 2 # picking up box
        self.wheels.stop()
        if not self.drop_flag: # lift servo if not sself.drop_flag
            self.servo.lift()
        schedule(lambda _: self.pickup_cb(), None) # schedule pickup_cb callback (read colour and activate exit_turn)
    