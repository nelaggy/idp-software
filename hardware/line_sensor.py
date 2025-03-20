from machine import Pin, Timer
from micropython import schedule
LL_PIN = const(11)
L_PIN = const(10)
R_PIN = const(9)
RR_PIN = const(8)

class LineSensors:
    def __init__(self, cb, dt=10):
        self.buf = bytearray(4)
        self.on_line = False
        self.turning_flag = False
        self.timer = Timer(mode=Timer.PERIODIC, period=dt, callback=self.on_change)

        self.read = self.read_fn

        self.cb = cb

        self.ll_sensor = Pin(LL_PIN, Pin.IN, Pin.PULL_DOWN)
        self.l_sensor = Pin(L_PIN, Pin.IN, Pin.PULL_UP)
        self.r_sensor = Pin(R_PIN, Pin.IN, Pin.PULL_UP)
        self.rr_sensor = Pin(RR_PIN, Pin.IN, Pin.PULL_DOWN)
    
    def read_fn(self):
        self.buf[0] = self.ll_sensor.value()
        self.buf[1] = self.l_sensor.value()
        self.buf[2] = self.r_sensor.value()
        self.buf[3] = self.rr_sensor.value()
        return self.buf
    
    def on_change(self, _):
        if not self.cb:
            return
        self.cb(self.read())
        return

    def set_callback(self, cb, dt=10):
        self.cb = cb
        self.timer.init(mode=Timer.PERIODIC, period=dt, callback=self.on_change)