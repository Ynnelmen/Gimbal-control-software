import math
import os
import time

x = 0
y = 0
while True:
    x = (y/360)*2*math.pi
    y += 1
    os.system("i2cset -y 1 0x47 0x00" + hex(math.sin(x)) + " w")
    time.sleep(.05)
