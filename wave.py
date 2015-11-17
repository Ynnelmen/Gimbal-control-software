import math
import os
import time

motorposition1 = 0
motorposition2 = 0
motorposition3 = 0

def motorX(xpos):
    os.system("i2cset -y 1 0x47 0x00 " + hex(motorposition1[xpos]) + " w")
    os.system("i2cset -y 1 0x47 0x01 " + hex(motorposition2[xpos]) + " w")
    os.system("i2cset -y 1 0x47 0x02 " + hex(motorposition3[xpos]) + " w")


def turnX():
    pass
    x = 0
    while True:
        motorX(x)
        x += 1
        if x > 4:
            x = 0
        #time.sleep(0.00001)

def generatesteps(resolution, offset):
    deltastep = offset
    motorstep = 0
    motormap = []
    motormap.extend(range(1,(360/resolution)+1))
    for item in motormap:
        motormap[motorstep] = int((255*math.sin(deltastep)+255)/2)
        deltastep += (2*resolution*math.pi)/360
        motorstep += 1
    return motormap

motorposition1 = generatesteps(120,0)
motorposition2 = generatesteps(120,2.0943933333)
motorposition3 = generatesteps(120,4.1887866666)
print motorposition1
print motorposition2
print motorposition3
turnX()
