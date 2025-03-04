## Main loop
from time import sleep
from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from dummy.motors import motor_test
from controllers.onroad_controller import OnRoadController
from dummy.line_sensor import line_sensor_with_motor_test, line_sensor_test
from dummy.servo import servo_test

line_sensors = LineSensors(None)
wheels = Motors()
controller = OnRoadController(line_sensors, wheels)
controller.activate()
while True:
    pass
