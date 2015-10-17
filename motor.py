import RPi.GPIO as GPIO
import time

COILA = 21
COILB = 22
COILC = 23

speed = 100
pwmSignal = [0,0,0,1,1,1]
phaseA = 0
phaseB = 0
phaseC = 0

GPIO.setmode(GPIO.BCM)
GPIO.setup(COILA,GPIO.OUT)
GPIO.setup(COILB,GPIO.OUT)
GPIO.setup(COILC,GPIO.OUT)

while True:
    if speed < 0:
        step = -1

    else:
        step = 1

    phaseA = phaseA + step
    if phaseA > 5:
        phaseA = 0
    if phaseA < 0:
        phaseA = 5

    phaseB = phaseB + step
    if phaseB > 5:
        phaseB = 0
    if phaseB < 0:
        phaseB = 5

    phaseC = phaseC + step
    if phaseC > 5:
        phaseC = 0
    if phaseC < 0:
        phaseC = 5

    GPIO.output(COILA,pwmSignal[phaseA])
    GPIO.output(COILb,pwmSignal[phaseB])
    GPIO.output(COILC,pwmSignal[phaseC])
    time.sleep(1/speed)