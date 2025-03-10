from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from navigation.navigator import Navigator
from controllers.offroad_controller import OffRoadControllerTest

line_sensors = LineSensors(None)
wheels = Motors()
controller = OffRoadControllerTest(line_sensors, wheels)

# go to A
controller.activate()
while True:
    pass
