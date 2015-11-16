import math
import os
import time

x = 0
y = 0
while True:
    y += (2*math.pi)/360
    os.system("i2cset -y 1 0x47 0x00" + hex(int((255*math.sin(y)+255)/2)) + " w")
    time.sleep(.001)
