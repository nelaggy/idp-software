# from machine import Pin, I2C
# import time

# TCS34725_I2C_ADDR = 0x29  # Default I2C address

# # TCS34725 Registers
# ENABLE_REGISTER = 0x80
# ATIME_REGISTER = 0x81
# CONTROL_REGISTER = 0x8F
# CDATAL = 0x94  # Clear channel data low byte

# # Commands
# ENABLE_AIEN = 0x10  # RGBC Interrupt Enable
# ENABLE_WEN = 0x08  # Wait Enable
# ENABLE_AEN = 0x02  # RGBC Enable
# ENABLE_PON = 0x01  # Power ON

# # Initialize I2C
# i2c = I2C(1, scl=Pin(11), sda=Pin(10), freq=400000)

# def write_register(register, value):
#     """Write a byte to the TCS34725 register"""
#     i2c.writeto_mem(TCS34725_I2C_ADDR, register, bytes([value]))

# def read_register(register, length=1):
#     """Read bytes from the TCS34725 register"""
#     return i2c.readfrom_mem(TCS34725_I2C_ADDR, register, length)

# def enable_sensor():
#     """Enable the TCS34725 sensor"""
#     write_register(ENABLE_REGISTER, ENABLE_PON)  # Power ON
#     time.sleep(0.003)  # Wait for power up
#     write_register(ENABLE_REGISTER, ENABLE_PON | ENABLE_AEN)  # Enable RGBC

# def read_rgbc():
#     """Read Red, Green, Blue, and Clear channel data"""
#     data = read_register(CDATAL, 8)  # Read 8 bytes (RGBC values)
#     clear = int.from_bytes(data[0:2], "little")
#     red = int.from_bytes(data[2:4], "little")
#     green = int.from_bytes(data[4:6], "little")
#     blue = int.from_bytes(data[6:8], "little")
#     return clear, red, green, blue

# # Main Program
# enable_sensor()
# time.sleep(0.1)  # Allow sensor to initialize

# while True:
#     clear, red, green, blue = read_rgbc()
    
#     if clear > 0:  # Normalize values to 0-255 range
#         r = (red / clear) * 255
#         g = (green / clear) * 255
#         b = (blue / clear) * 255
#     else:
#         r, g, b = 0, 0, 0
    
#     print(f"RGB: ({int(r)}, {int(g)}, {int(b)})")
#     time.sleep(0.5)  # Adjust sampling rate


from machine import Pin
from machine import I2C
from dummy.tcs34725 import TCS34725
from time import sleep

# i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)

# print("Scanning I2C bus...")
# devices = i2c.scan()
# for device in devices:
#     try:
#         sensor_id = i2c.readfrom_mem(device, 0x12, 1)  # Read sensor ID register
#         print(f"Sensor found at address {hex(device)} with ID: {sensor_id}")
#     except OSError as e:
#         print(f"Failed to communicate with sensor at address {hex(device)}: {e}")

# if devices:
#     print("I2C devices found:", [hex(d) for d in devices])
# else:
#     print("No I2C devices found. Check wiring!")

# sensor_address = 0x29  # TCS34725 default address
# if 0x29 in devices:  # Check if the sensor is present in the detected addresses
#     try:
#         sensor_id = i2c.readfrom_mem(sensor_address, 0x12, 1)  # Read sensor ID register
#         print(f"Sensor ID: {sensor_id}")
#     except OSError as e:
#         print(f"Failed to communicate with sensor: {e}")
# else:
#     print("Sensor not found at 0x29 address.")

i2c_bus = I2C(0, sda=Pin(16), scl=Pin(17))
tcs = TCS34725(i2c_bus)

while True:
        
    print('raw: {}'.format(tcs.read(raw = True)))

    print('rgb: {}'.format(tcs.read(raw = False)))

    sleep(0.8)