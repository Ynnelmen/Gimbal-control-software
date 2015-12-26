import math
import time
import RPi.GPIO as GPIO

motorposition1 = 0
motorposition2 = 0
motorposition3 = 0

def motorX(xpos):
    p.ChangeDutyCycle(motorposition1[xpos])
    q.ChangeDutyCycle(motorposition2[xpos])
    r.ChangeDutyCycle(motorposition3[xpos])
    s.ChangeDutyCycle(motorposition1[xpos])
    t.ChangeDutyCycle(motorposition2[xpos])
    u.ChangeDutyCycle(motorposition3[xpos])

def turnX():
    x = 0
    while True:
        while x < 91:
            motorX(x)
            x += 1
            time.sleep(0.08)
        while x != 0:
            motorX(x)
            x -= 1
            time.sleep(0.08)
        time.sleep(1)
        while x != 0:
            motorX(x)
            x -= 1
            time.sleep(0.08)
        while x < 91:
            motorX(x)
            x += 1
            time.sleep(0.08)
        time.sleep(1)

def generatesteps(resolution, offset): # generate array values for every possible motor microstep
    deltastep = offset
    motorstep = 0
    motormap = []
    motormap.extend(range(1,((12*360)/resolution)+1)) # set range as a function of requested angular resolution
    for item in motormap: # fill list with calculated output duty cycles
        motormap[motorstep] = int((100*math.sin(deltastep)+100)/2)
        deltastep += (2*resolution*math.pi)/360
        motorstep += 1
    return motormap

motorposition1 = generatesteps(4,0)
motorposition2 = generatesteps(4,2.0943933333)
motorposition3 = generatesteps(4,4.1887866666)
print motorposition1
print motorposition2
print motorposition3
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
p = GPIO.PWM(11, 200)
q = GPIO.PWM(13, 200)
r = GPIO.PWM(15, 200)
p.start(0)
q.start(0)
r.start(0)
GPIO.setup(36, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)
s = GPIO.PWM(36, 200)
t = GPIO.PWM(38, 200)
u = GPIO.PWM(40, 200)
s.start(0)
t.start(0)
u.start(0)
turnX()
GPIO.cleanup()
