## Main loop
import os
print(os.listdir('dummy'))

from dummy.motors import motor_test
from dummy.line_sensor import line_sensor_with_motor_test, line_sensor_test
from dummy.servo import servo_test


while True:
    motor_test()
