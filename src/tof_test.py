# Simple demo of the VL53L0X distance sensor.
# Will print the sensed range/distance every second.
import time
import machine
from tof_sensor import vl53l0x

# initialize i2c bus
sda = machine.Pin(20) # SDA pin
scl = machine.Pin(21) # SCL pin
i2c = machine.I2C(0, sda=sda, scl=scl, freq=400000)

# scan for devices
print("Scanning I2C bus...")
devices = i2c.scan()

if len(devices) == 0:
    print("No I2C devices found!")
else:
    print(f"I2C devices found: {len(devices)}")
    
for device in devices:
    print(f"Decimal address: {device} | Hex address: {hex(device)}")

# initialize sensor
vl53 = vl53l0x(i2c)

# Optionally adjust the measurement timing budget to change speed and accuracy.
# See the example here for more details:
#   https://github.com/pololu/vl53l0x-arduino/blob/master/examples/Single/Single.ino
# For example a higher speed but less accurate timing budget of 20ms:
# vl53.measurement_timing_budget = 20000
# Or a slower but more accurate timing budget of 200ms:
# vl53.measurement_timing_budget = 200000
# The default timing budget is 33ms, a good compromise of speed and accuracy.

# Main loop will read the range and print it every second.
while True:
    print("Range: {0}mm".format(vl53.range))
    time.sleep(1.0)
