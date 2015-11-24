import math
import os
import time
import RPi.GPIO as GPIO

motorposition1 = 0
motorposition2 = 0
motorposition3 = 0

def motorX(xpos):
    p.ChangeDutyCycle(motorposition1[xpos])
    q.ChangeDutyCycle(motorposition1[xpos])
    r.ChangeDutyCycle(motorposition1[xpos])


def turnX():
    pass
    x = 0
    while True:
        motorX(x)
        x += 1
        if x > 11:
            x = 0
        time.sleep(0.001)
    #except KeyboardInterrupt:
        #pass

def generatesteps(resolution, offset):
    deltastep = offset
    motorstep = 0
    motormap = []
    motormap.extend(range(1,(360/resolution)+1))
    for item in motormap:
        motormap[motorstep] = int((100*math.sin(deltastep)+100)/2)
        deltastep += (2*resolution*math.pi)/360
        motorstep += 1
    return motormap

motorposition1 = generatesteps(2,0)
motorposition2 = generatesteps(2,2.0943933333)
motorposition3 = generatesteps(2,4.1887866666)
print motorposition1
print motorposition2
print motorposition3
GPIO.setmode(GPIO.BOARD)
GPIO.setup(10, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
p = GPIO.PWM(10, 250)
q = GPIO.PWM(11, 250)
r = GPIO.PWM(12, 250)
p.start(0)
q.start(0)
r.start(0)
turnX()
GPIO.clenup()
