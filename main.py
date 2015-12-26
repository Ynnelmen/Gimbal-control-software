# PyRoscope - IMU software for gimbal-control, written by Jeremie Reusser.
# Version 1.0a
# This software is meant to be run on Raspberry Pi B+ or Raspberry Pi 2 systems.
# Designed for 2x ADXL345 accelerometers and 2x L3GD20/L3GD20H gyros (connected via I2C - you need to activate alternate addresses for one pair of modules).
# Motor control through PWM through L298N h-bridge. Any BLDC should do after a bit of tweaking.
# This is experimental technical demonstration software. For realtime performance, use embedded systems with low level implementations or dedicated controller boards.

from ADXL345.adxl345 import ADXL345
from L3GD20.L3GD20 import L3GD20
from threading import Thread
import RPi.GPIO as GPIO
import time
import math
import os
import PID

P = 0.33
I = 2.2
D = 0.013
prevx = [0]*26
prevy = [0]*26
factorIN = 0.4  # bestimmt Gewichtung des neuen input-Wertes
factor0 = 0.1   # "         "         der alten Werte
factor1 = 0.2
factor2 = 0.3
DT = 0.02 # master program duty cycle (regulator refresh rate) - set to 0.04 for low frequency
PWM_FREQ = 400 # set master PWM refresh rate (motors only)

motorposition1 = 0
motorposition2 = 0
motorposition3 = 0

def motorX(): # controls x-axis
    pidX = PID.PID(P, I, D)
    pidX.setSampleTime(0.02)
    #pidX.setKp(1)
    #pidX.setKi(1)
    #pidX.setKd(0.01)
    windupfactor = 10
    time.sleep(30)
    pidX.SetPoint = 10
    while True:
        target = gentarget(output1[1],1) # calculates requested output using reference sensor
        pidX.update(output2[1])
        #if target < (max(prevx) - windupfactor):
        #    pidX.setWindup(target)
        #elif target > (min(prevx) + windupfactor):
        #    pidX.setWindup(target)
        #else:
        #    pidX.SetPoint = target
        output = int(pidX.output)
        if output > 359: # compensate for full revolution
            output -= 360
        if output < 0: # compensate for full revolution
            output += 360
        # set phase values
        x1.ChangeDutyCycle(xmotorposition1[output])
        x2.ChangeDutyCycle(xmotorposition2[output])
        x3.ChangeDutyCycle(xmotorposition3[output])
        time.sleep(DT/2)

def motorY(): # controls y-axis
    y = 0
    offset = 0
    time.sleep(30)
    while True:
        offset = int(output1[0]) - int(gentarget(output1[0],0)) # filters reference values
        y = 2*(-int(output1[0]) + offset) # calculates requested output
        if y > 359: # compensate for full revolution
            y = y - 360
        if y < 0: # compensate for full revolution
            y = 360 + y
        time.sleep(DT/2)
        # set phase values
        y1.ChangeDutyCycle(ymotorposition1[y])
        y2.ChangeDutyCycle(ymotorposition2[y])
        y3.ChangeDutyCycle(ymotorposition3[y])

def gentarget(newinput, axis):
    if axis == 0:
        output = newinput*factorIN + prevx[0]*factor0 + prevx[12]*factor1 + prevx[25]*factor2
        del prevx[0]
        prevx.append(output) #speichert angepassten output im array
    elif axis == 1:
        output = newinput*factorIN + prevy[0]*factor0 + prevy[12]*factor1 + prevy[25]*factor2
        del prevy[0]
        prevy.append(output) #speichert angepassten output im array
    return (output)

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

def filter_reference(): # combines raw sensor data from reference pair
    global x1, y1, z1, output1
    offset = [0.0,0.0]
    output1 = [0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        output1[0] = offset[0] + .99*(output1[0]+reference_gyro_omega[0]*DT-offset[0]) + .01*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['y'], reference_accelerometer_acc['z'])+math.pi) # calculate x-rotation
        output1[1] = offset[1] + .99*(output1[1]+reference_gyro_omega[1]*DT-offset[1]) + .01*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['z'], reference_accelerometer_acc['x'])+math.pi) # calculate y-rotation
        iterations += 1 # iteration counter
        time.sleep(DT)
        if iterations == 20*(1/DT): # set values to zero once values are stabilized based on fixed iteration number (as a function of the master clock)
            offset[0] = 0.0 - output1[0] # zero x
            offset[1] = 0.0 - output1[1] # zero y
            output1[0] = 0 # output to 0 for instant response (avoid values in- or decrementing)
            output1[1] = 0 # output to 0 for instant response (avoid values in- or decrementing)
            print "Sensor 1 calibrated"

def filter_stabilized(): # combines raw sensor data from stabilized pair
    global output2
    offset = [0.0,0.0]
    output2 = [0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        output2[0] = offset[0] + .99*(output2[0]+stabilized_gyro_omega[0]*DT-offset[0]) + .01*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['y'], stabilized_accelerometer_acc['z'])+math.pi) # calculate x-rotation
        output2[1] = offset[1] + .99*(output2[1]+stabilized_gyro_omega[1]*DT-offset[1]) + .01*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['z'], stabilized_accelerometer_acc['x'])+math.pi) # calculate y-rotation
        iterations += 1 # iteration counter
        time.sleep(DT)
        if iterations == 20*(1/DT): # set values to zero once values are stabilized based on fixed iteration number (as a function of the master clock)
            offset[0] = 0.0 - output2[0] # zero x
            offset[1] = 0.0 - output2[1] # zero y
            output2[0] = 0 # output to 0 for instant response (avoid values in- or decrementing)
            output2[1] = 0 # output to 0 for instant response (avoid values in- or decrementing)
            print "Sensor 2 calibrated"

def accelerometer_reference(): # read reference accelerometer values via ADXL345 library
    global reference_accelerometer_acc
    reference_accelerometer_acc = 0
    reference_accelerometer = ADXL345(0x53) # read correct I2C device address
    while True:
        time.sleep(DT) # match program clock
        reference_accelerometer_acc = reference_accelerometer.getAxes(True) # read value

def accelerometer_stabilized(): # read stabilized accelerometer values via ADXL345 library
    global stabilized_accelerometer_acc
    stabilized_accelerometer_acc = 0
    stabilized_accelerometer = ADXL345(0x1d) # read correct I2C device address
    while True:
        time.sleep(DT) # match program clock
        stabilized_accelerometer_acc = stabilized_accelerometer.getAxes(True) # read value

def gyro_reference(): # read reference gyroscope values
    global reference_gyro_omega
    reference_gyro_omega = 0
    reference_gyro = L3GD20(busId = 1, slaveAddr = 0x6b, ifLog = False, ifWriteBlock=False)
    reference_gyro.Set_PowerMode("Normal")
    reference_gyro.Set_FullScale_Value("250dps")
    reference_gyro.Set_AxisX_Enabled(True)
    reference_gyro.Set_AxisY_Enabled(True)
    reference_gyro.Set_AxisZ_Enabled(True)
    reference_gyro.Init()
    reference_gyro.Calibrate()
    while True:
        time.sleep(DT)
        reference_gyro_omega = reference_gyro.Get_CalOut_Value()

def gyro_stabilized(): # read stabilized gyroscope values
    global stabilized_gyro_omega
    stabilized_gyro_omega = 0
    stabilized_gyro = L3GD20(busId = 1, slaveAddr = 0x6a, ifLog = False, ifWriteBlock=False)
    stabilized_gyro.Set_PowerMode("Normal")
    stabilized_gyro.Set_FullScale_Value("250dps")
    stabilized_gyro.Set_AxisX_Enabled(True)
    stabilized_gyro.Set_AxisY_Enabled(True)
    stabilized_gyro.Set_AxisZ_Enabled(True)
    stabilized_gyro.Init()
    stabilized_gyro.Calibrate()
    while True:
        time.sleep(DT)
        stabilized_gyro_omega = stabilized_gyro.Get_CalOut_Value()

# generate array values for every possible motor step
xmotorposition1 = generatesteps(4,0)
xmotorposition2 = generatesteps(4,2.0943933333)
xmotorposition3 = generatesteps(4,4.1887866666)
ymotorposition1 = generatesteps(4,0)
ymotorposition2 = generatesteps(4,2.0943933333)
ymotorposition3 = generatesteps(4,4.1887866666)
# set up I/O for motor control (set PWM outputs)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(36, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)
GPIO.setup(40, GPIO.OUT)
# set PWM frequency
x1 = GPIO.PWM(11, PWM_FREQ)
x2 = GPIO.PWM(13, PWM_FREQ)
x3 = GPIO.PWM(15, PWM_FREQ)
y1 = GPIO.PWM(36, PWM_FREQ)
y2 = GPIO.PWM(38, PWM_FREQ)
y3 = GPIO.PWM(40, PWM_FREQ)
# enable PWM output
x1.start(0)
x2.start(0)
x3.start(0)
y1.start(0)
y2.start(0)
y3.start(0)
# initialize parallel tasks for reading, processing and output
readGyroRef = Thread(target=gyro_reference, args=())
readAccRef = Thread(target=accelerometer_reference, args=())
readGyroStab = Thread(target=gyro_stabilized, args=())
readAccStab = Thread(target=accelerometer_stabilized, args=())
filterRef = Thread(target=filter_reference, args=())
filterStab = Thread(target=filter_stabilized, args=())
motorXThread = Thread(target=motorX, args=())
#motorYThread = Thread(target=motorY, args=())
# start tasks
readGyroRef.start()
readAccRef.start()
readGyroStab.start()
readAccStab.start()
filterRef.start()
filterStab.start()
motorXThread.start()
#motorYThread.start()
# wait for calibration to complete - sensors should be held still
print "Catching up..."
time.sleep(2)
print "Calibrating..."
time.sleep(25)
print "Get ready!"
time.sleep(5)
# work in progess: display info
print "Running... (^C to stop)"
while True:
    try:
        #print output1
        print output2[1]
        time.sleep(DT)
    except KeyboardInterrupt: # wait for exit
        GPIO.cleanup()
        os.system("sudo killall python")
