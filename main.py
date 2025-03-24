from hardware.colour_sensor import ColourSensor
from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from hardware.servo import Servo
from navigation.navigator import Navigator
from controllers.onroad_controller import OnRoadController
from machine import Pin, Timer
from micropython import schedule
from controllers.offroad_controller import OffRoadController



class MainController:
    def __init__(self):
        self.line_sensors = LineSensors(None)
        self.wheels = Motors()
        self.navigator = Navigator()
        self.servo = Servo()
        self.led = Pin(26, Pin.OUT, value=0)
        self.button = Pin(14, Pin.IN)
        self.colour_sensor = ColourSensor()
        self.start_flag = True
        self.end_flag = False
        self.running = False
        self.cnt = 0

        self.timer = Timer(-1)
        
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())
        self.servo.set_angle(90)
        self.destinations = [17, 18, 19, 16, 17, 18, 19, 1]
        self.onroad_controller = OnRoadController(self.line_sensors, self.wheels, self.servo, self.navigator, self.go_offroad)
        self.offroad_controller = OffRoadController(self.line_sensors, self.wheels, self.servo, self.go_onroad, self.get_colour)

    def toggle(self):
        """
        Toggles the running state of the robot
        - If running, stops robot and resets components
        - If not running, starts navigation process
        """
        self.button.irq(trigger=Pin.IRQ_RISING, handler=None)

        # Debounce button press to prevent false triggers
        self.timer.init(mode=Timer.ONE_SHOT, period=200, callback=lambda _: self.debounce())
        
        if self.running: # stops sensors, wheels, and turn of LED if already running
            self.line_sensors.set_callback(None)
            self.wheels.stop()
            self.led.off()
            self.servo.set_angle(90)
            self.running = False
            return

        # resets navigator and sets destination to 1
        self.navigator.reset()
        self.navigator.set_destination(1)

        # resets flags and starts navigation process
        self.start_flag = True
        self.end_flag = False
        self.carrying_block = False
        self.running = True
        
        self.cnt = 0 # reset destination counter
        self.onroad_controller.activate() # start on-road navigation

    def debounce(self):
        """
        Re-enables button interrupts after debounce delay
        """
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())

    def go_onroad(self):
        """
        Transitions from OffRoadController to OnRoadController
        """
        if not self.carrying_block: # if not carrying block, set servo position to 90 degrees
            self.servo.set_angle(90)
        self.onroad_controller.activate()
        
    def go_offroad(self):
        """
        Transitions from OnRoadController to OffRoadController
        """
        if self.start_flag:
            self.start_flag = False
            self.navigator.set_destination(16)
            self.timer.init(mode=Timer.ONE_SHOT, period=1000, callback=lambda _: self.led.on())
            return
        
        # if at final destination, stop robot
        if self.end_flag:
            self.end_flag = False
            self.led.off()
            self.servo.set_angle(90)
            self.running = False
            self.line_sensors.set_callback(None)
            self.wheels.wheel_speed(70, 70)
            self.timer.init(mode=Timer.ONE_SHOT, period=1500, callback=lambda _: self.wheels.stop()) # move forward for 1.5 seconds before stopping
            return
        
        self.navigator.get_turn()
        reverse_flag = False
        delay = 500

        # reverse before picking up box if node 17, 18, or 19
        if self.navigator.node == 17 or self.navigator.node == 18 or self.navigator.node == 19:
            reverse_flag = True
        
        # reverse longer before picking up box if node 17 or 19
        if self.navigator.node == 17 or self.navigator.node == 19:
            delay = 600

        # Activate OffRoadController
        self.offroad_controller.activate(self.carrying_block, reverse_flag, delay=delay)

    def get_colour(self):
        """
        Handles colour-based decision making:
        - Changes direction to drop or pick up blocks.
        - Determines next destination based on detected colour.
        """
        self.navigator.change_direction(2) # reverse direction after handling block
        self.carrying_block = not self.carrying_block # update carrying state
        
        if not self.carrying_block:
            # set next destination in cycle
            self.navigator.set_destination(self.destinations[self.cnt])
            self.cnt += 1

            # if all destinations are visited, mark as end
            if self.cnt == len(self.destinations):
                self.end_flag = True
            
        else:
            # determine drop-off destination based on colour
            is_bg = self.colour_sensor.detect()
            if is_bg:
                self.navigator.set_destination(2)
            else:
                self.navigator.set_destination(0)

        reverse_delay = 1200 # set reverse after picking up/placing box for 1.2 seconds
        
        # Adjust delay based on node
        if self.navigator.node == 9 or self.navigator.node == 17 or self.navigator.node == 19 or self.navigator.node == 14:
            reverse_delay = 700
        elif self.navigator.node == 3 or self.navigator.node == 6:
            reverse_delay = 1400
        
        # Determine turn direction for exit
        turn = self.navigator.get_turn()
        if turn == 1:
            self.offroad_controller.exit_turn(1, reverse_delay)
        elif turn == 3:
            self.offroad_controller.exit_turn(-1, reverse_delay)
        
        # Set direction of turn when turning 180 degrees after dropping box.
        elif turn == 0:
            if self.navigator.node == 6:
                self.offroad_controller.exit_turn(2, reverse_delay)
            else:
                self.offroad_controller.exit_turn(-2, reverse_delay)
        return
        
        
if __name__ == "__main__":
    controller = MainController()

    while True:
        pass
