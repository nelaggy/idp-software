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

# go to A
navigator.set_destination(16)
controller.activate()
pin = Pin(26, Pin.OUT)
pin.on()
while True:
    pass
