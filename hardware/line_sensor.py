from machine import Pin
from micropython import schedule
LL_PIN = const(11)
L_PIN = const(10)
R_PIN = const(9)
RR_PIN = const(8)

class LineSensors:
    def __init__(self, cb):
        self.buf = bytearray(4)
        self.on_line = False
        self.turning_flag = False

        self.cb = cb

        self.ll_sensor = Pin(LL_PIN, Pin.IN, Pin.PULL_DOWN)
        self.l_sensor = Pin(L_PIN, Pin.IN, Pin.PULL_UP)
        self.r_sensor = Pin(R_PIN, Pin.IN, Pin.PULL_UP)
        self.rr_sensor = Pin(RR_PIN, Pin.IN, Pin.PULL_DOWN)

        self.ll_sensor.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.on_change)
        self.l_sensor.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.on_change)
        self.r_sensor.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.on_change)
        self.rr_sensor.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.on_change)
    
    def read(self):
        self.buf[0] = self.ll_sensor.value()
        self.buf[1] = self.l_sensor.value()
        self.buf[2] = self.r_sensor.value()
        self.buf[3] = self.rr_sensor.value()
        return self.buf
    
    def on_change(self, _):
        # self.cb(self.read()) # if schedule is still too slow
        schedule(self.cb, self.read())
        pass

    def deactivate_central_trackers(self):
        self.l_sensor.irq(handler=None)
        self.r_sensor.irq(handler=None)

    def activate_central_trackers(self):
        self.l_sensor.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.on_change)
        self.r_sensor.irq(trigger=Pin.IRQ_FALLING|Pin.IRQ_RISING, handler=self.on_change)
        