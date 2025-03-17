## Main loop
from hardware.colour_sensor import ColourSensor
from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from hardware.servo import Servo
from navigation.navigator import Navigator
from controllers.onroad_controller import OnRoadController
from machine import Pin, Timer
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
        self.running = False
        
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())
        self.servo.set_angle(90)
        self.destinations = [17, 18, 19]
        self.onroad_controller = OnRoadController(self.line_sensors, self.wheels, self.servo, self.navigator, self.go_offroad)
        self.offroad_controller = OffRoadController(self.line_sensors, self.wheels, self.servo, self.go_onroad, self.get_colour)

    def toggle(self):
        print('toggle')
        self.button.irq(trigger=Pin.IRQ_RISING, handler=None)
        self.timer = Timer(mode=Timer.ONE_SHOT, period=200, callback=self.debounce)
        if self.running:
            self.line_sensors.set_callback(None)
            self.wheels.stop()
            self.led.off()
            self.servo.set_angle(90)
            self.running = False
            return
        self.navigator.reset()
        self.navigator.set_destination(1)

        self.start_flag = True
        self.carrying_block = False
        self.running = True
        
        self.cnt = 0
        self.onroad_controller.activate()

    def debounce(self, _):
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())

    def go_onroad(self):
        print('go onroad')
        self.onroad_controller.activate()
        
    def go_offroad(self):
        if self.start_flag:
            self.start_flag = False
            self.navigator.set_destination(16)
            self.led.on()
            return
        print('go offroad')
        self.navigator.get_turn()
        self.offroad_controller.activate(self.carrying_block)

    def get_colour(self, _):
        self.navigator.change_direction(2)
        self.carrying_block = not self.carrying_block
        if not self.carrying_block:
            self.navigator.set_destination(self.destinations[self.cnt])
            self.cnt += 1
        else:
            is_bg = self.colour_sensor.detect()
            if is_bg:
                print('green or blue')
                self.navigator.set_destination(2)
            else:
                print('red or yellow')
                self.navigator.set_destination(0)
        print(self.navigator.node, self.navigator.destination, self.navigator.path, self.navigator.direction)
        turn = self.navigator.get_turn()
        if turn == 1:
            self.offroad_controller.exit_turn(1)
        elif turn == 3:
            self.offroad_controller.exit_turn(-1)
        elif turn == 0:
            if self.navigator.node == 2:
                self.offroad_controller.exit_turn(2)
            else:
                self.offroad_controller.exit_turn(-2)
        
        

controller = MainController()

while True:
    pass
