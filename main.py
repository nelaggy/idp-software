## Main loop
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
        self.destinations = [17, 18, 19, 1]
        self.onroad_controller = OnRoadController(self.line_sensors, self.wheels, self.servo, self.navigator, self.go_offroad)
        self.offroad_controller = OffRoadController(self.line_sensors, self.wheels, self.servo, self.go_onroad, self.get_colour)

    def toggle(self):
        self.button.irq(trigger=Pin.IRQ_RISING, handler=None)
        self.timer.init(mode=Timer.ONE_SHOT, period=200, callback=lambda _: self.debounce())
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
        self.end_flag = False
        self.carrying_block = False
        self.running = True
        
        self.cnt = 0
        self.onroad_controller.activate()

    def debounce(self):
        self.button.irq(trigger=Pin.IRQ_RISING, handler=lambda _: self.toggle())

    def go_onroad(self):
        if not self.carrying_block:
            self.servo.set_angle(90)
        self.onroad_controller.activate()
        
    def go_offroad(self):
        if self.start_flag:
            self.start_flag = False
            self.navigator.set_destination(16)
            self.timer.init(mode=Timer.ONE_SHOT, period=1000, callback=lambda _: self.led.on())
            return
        if self.end_flag:
            self.end_flag = False
            self.led.off()
            self.servo.set_angle(90)
            self.running = False
            self.line_sensors.set_callback(None)
            self.wheels.wheel_speed(70, 70)
            self.timer.init(mode=Timer.ONE_SHOT, period=1500, callback=lambda _: self.wheels.stop())
            return
        self.navigator.get_turn()
        reverse_flag = False
        if self.navigator.node == 17 or self.navigator.node == 18 or self.navigator.node == 19:
            reverse_flag = True
        self.offroad_controller.activate(self.carrying_block, reverse_flag)

    def get_colour(self):
        self.navigator.change_direction(2)
        self.carrying_block = not self.carrying_block
        if not self.carrying_block:
            self.navigator.set_destination(self.destinations[self.cnt])
            # print('carrying block: ', self.carrying_block, 'destination: ', self.destinations[self.cnt])
            self.cnt += 1
            if self.cnt == len(self.destinations):
                self.end_flag = True
            
        else:
            is_bg = self.colour_sensor.detect()
            if is_bg:
                self.navigator.set_destination(2)
            else:
                self.navigator.set_destination(0)
        # print(self.navigator.node, self.navigator.destination, self.navigator.path, self.navigator.direction)
        reverse_delay = 1200
        if self.navigator.node == 9:
            reverse_delay = 600
        elif self.navigator.node == 3 or self.navigator.node == 6:
            reverse_delay = 1400
        turn = self.navigator.get_turn()
        if turn == 1:
            self.offroad_controller.exit_turn(1, reverse_delay)
        elif turn == 3:
            self.offroad_controller.exit_turn(-1, reverse_delay)
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
