## Main loop
from hardware.motor import Motors
from hardware.line_sensor import LineSensors
from navigation.navigator import Navigator
from controllers.onroad_controller import OnRoadController
from machine import Pin

line_sensors = LineSensors(None)
wheels = Motors()
navigator = Navigator()
controller = OnRoadController(line_sensors, wheels, navigator)
led = Pin(26)
led.off()

# go to A
navigator.set_destination(16)
controller.activate()
while True:
    pass
