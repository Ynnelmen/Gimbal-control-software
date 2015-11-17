import math
import os
import time

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

motorposition1 = generatesteps(1,0)
motorposition2 = generatesteps(1,2.0943933333)
motorposition3 = generatesteps(1,4.1887866666)
print motorposition1
print motorposition2
print motorposition3
