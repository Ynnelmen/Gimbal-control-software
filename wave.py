import math
import os
import time

motorposition = 0

def turnX():
    while True:
        x = 0
        motorX(x)
        x += 1
        if x > 359:
            x = 0
        time.sleep(0.001)


def motorX(xpos):
    print motorposition[xpos],motorposition[xpos+120],motorposition[xpos+240]
    os.system("i2cset -y 1 0x47 0x00 " + hex(motorposition[xpos]) + " w")
	os.system("i2cset -y 1 0x47 0x01 " + hex(motorposition[xpos + 120]) + " w")
	os.system("i2cset -y 1 0x47 0x02 " + hex(motorposition[xpos + 240]) + " w")

def generatesteps(resolution):
    deltastep = 0
    motorstep = 0
    motormap = []
    motormap.extend(range(1,(360/resolution)+1))
    for item in motormap:
        motormap[motorstep] = int((255*math.sin(deltastep)+255)/2)
        deltastep += (2*resolution*math.pi)/360
        motorstep += 1
    return motormap

motorposition = generatesteps(1)
print motorposition
turnX()
