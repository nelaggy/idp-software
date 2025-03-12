## Main loop
from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from hardware.servo import Servo
from navigation.navigator import Navigator
from controllers.onroad_controller import OnRoadController
from controllers.offroad_controller import OffRoadController



class MainController:
    def __init__(self):
        self.line_sensors = LineSensors(None)
        self.wheels = Motors()
        self.navigator = Navigator()
        self.servo = Servo()
        self.onroad_controller = OnRoadController(self.line_sensors, self.wheels, self.servo, self.navigator, self.go_offroad)
        self.offroad_controller = OffRoadController(self.line_sensors, self.wheels, self.go_onroad)
        self.navigator.set_destination(5)

        self.start_box_flag = True
        self.carrying_block = False
        self.destinations = [0, 17, 2, 19, 0, 18, 2]
        self.cnt = 0

        self.onroad_controller.activate()

    def go_onroad(self):
        # get colour and hence next destination
        self.navigator.set_destination(self.destinations[self.cnt])
        self.cnt += 1
        self.onroad_controller.activate()
        
    def go_offroad(self):
        if self.start_box_flag:
            self.start_box_flag = False
            self.navigator.set_destination(16)
            self.offroad_controller.activate()
            return
        
        # pass dropoff=self.carrying_block
        self.offroad_controller.activate()
        self.carrying_block = not self.carrying_block

controller = MainController()

while True:
    pass
