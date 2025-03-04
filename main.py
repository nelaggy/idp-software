## Main loop
from time import sleep
from dummy.motors import motor_test
from controllers.onroad_controller import OnRoadController
from dummy.line_sensor import line_sensor_with_motor_test, line_sensor_test
from dummy.servo import servo_test


controller = OnRoadController()
while True:
    pass
