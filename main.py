## Main loop
from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from hardware.servo import Servo
from navigation.navigator import Navigator
from controllers.onroad_controller import OnRoadController
from machine import Pin
from time import sleep_ms
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
        self.running = False
        
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())
        self.servo.set_angle(90)
        self.destinations = [0, 17, 2, 19, 0, 18, 2]
        self.onroad_controller = OnRoadController(self.line_sensors, self.wheels, self.servo, self.navigator, self.go_offroad)
        self.offroad_controller = OffRoadController(self.line_sensors, self.wheels, self.servo, self.go_onroad, self.get_colour)

    def toggle(self):
        print('toggle')
        self.button.irq(trigger=Pin.IRQ_RISING, handler=None)
        schedule(self.debounce, 500)
        if self.running:
            self.line_sensors.set_callback(None)
            self.wheels.stop()
            self.led.off()
            self.servo.set_angle(90)
            self.running = False
            return
        self.navigator.reset()
        self.navigator.set_destination(1)

        self.start_box_flag = True
        self.carrying_block = False
        self.running = True
        
        self.cnt = 0
        self.onroad_controller.activate()

    def debounce(self, delay):
        sleep_ms(delay)
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())

    def go_onroad(self):
        # get colour and hence next destination
        self.carrying_block = not self.carrying_block
        self.cnt += 1
        self.onroad_controller.activate()
        
    def go_offroad(self):
        if self.start_box_flag:
            self.start_box_flag = False
            self.navigator.set_destination(16)
            self.led.on()
            return
        # pass dropoff=self.carrying_block
        self.navigator.get_turn()
        self.offroad_controller.activate()

    def get_colour(self, _):
        # get colour and hence next destination
        self.navigator.change_direction(2)
        self.navigator.set_destination(self.destinations[self.cnt])
        turn = self.navigator.get_turn()
        if turn == 1:
            self.offroad_controller.exit_turn(1)
        elif turn == 3:
            self.offroad_controller.exit_turn(-1)
        elif turn == 0:
            self.offroad_controller.exit_turn(2)
        
        

controller = MainController()

while True:
    pass
