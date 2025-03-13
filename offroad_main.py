from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from controllers.offroad_controller import OffRoadController, OffRoadController2

line_sensors = LineSensors(None)
wheels = Motors()
controller = OffRoadController(line_sensors, wheels, wheels.stop())

controller.activate()
while True:
    pass