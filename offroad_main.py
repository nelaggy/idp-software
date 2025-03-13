from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from navigation.navigator import Navigator
from controllers.offroad_controller import OffRoadControllerTest
from dummy.servo import servo_test

line_sensors = LineSensors(None)
wheels = Motors()
controller = OffRoadControllerTest(line_sensors, wheels)

controller.activate()
while True:
    pass
