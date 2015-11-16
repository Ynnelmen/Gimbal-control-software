# PyRoscope - IMU software for gimbal-control, written by Jeremie Reusser.
# Beta 0.3

from ADXL345.adxl345 import ADXL345
from L3GD20.L3GD20 import L3GD20
from threading import Thread
import RPi.GPIO as GPIO
import time
import math
import os

COILAX = 21
COILBX = 22
COILCX = 23
COILAY = None
COILBY = None
COILCY = None
DT = 0.02 # set to 0.04 for low frequency
pwmSignal = [0,0,0,1,1,1]
speedX = 5
speedY = 10

def motorX(xpos):
	os.system("i2cset -y 1 0x47 0x00 " + motorposition[xpos] + " w")
	os.system("i2cset -y 1 0x47 0x01 " + motorposition[xpos + 120] + " w")
	os.system("i2cset -y 1 0x47 0x02 " + motorposition[xpos + 240] + " w")

def motorY(ypos):
	os.system("i2cset -y 1 0x47 0x03 " + motorposition[ypos] + " w")
	os.system("i2cset -y 1 0x47 0x04 " + motorposition[ypos + 120] + " w")
	os.system("i2cset -y 1 0x47 0x05 " + motorposition[ypos + 240] + " w")

def filter_reference():
    global x1, y1, z1, output1
    offset = [0.0,0.0,0.0]
    output1 = [0.0,0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        output1[0] = offset[0] + .99*(output1[0]+reference_gyro_omega[0]*DT-offset[0]) + .01*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['y'], reference_accelerometer_acc['z'])+math.pi)
        output1[1] = offset[1] + .99*(output1[1]+reference_gyro_omega[1]*DT-offset[1])+.01*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['z'], reference_accelerometer_acc['x'])+math.pi)
        #output1[2] = offset[2] + .99999*(output1[2]+reference_gyro_omega[2]*DT-offset[2]) + .00001*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['x'], reference_accelerometer_acc['y'])+math.pi)
        iterations += 1
        time.sleep(DT)
        if iterations == 40*(1/DT):
            offset[0] = 0.0 - output1[0]
            offset[1] = 0.0 - output1[1]
            #offset[2] = 0.0 - output1[2]
            print "Sensor 1 calibrated"

def filter_stabilized():
    global output2
    offset = [0.0,0.0,0.0]
    output2 = [0.0,0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        output2[0] = offset[0] + .99*(output2[0]+stabilized_gyro_omega[0]*DT-offset[0]) + .01*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['y'], stabilized_accelerometer_acc['z'])+math.pi)
        output2[1] = offset[1] + .99*(output2[1]+stabilized_gyro_omega[1]*DT-offset[1]) + .01*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['z'], stabilized_accelerometer_acc['x'])+math.pi)
        #output2[2] = offset[2] + .99999*(output2[2]+stabilized_gyro_omega[2]*DT-offset[2]) + .00001*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['x'], stabilized_accelerometer_acc['y'])+math.pi)
        iterations += 1
        time.sleep(DT)
        if iterations == 40*(1/DT):
            offset[0] = 0.0 - output2[0]
            offset[1] = 0.0 - output2[1]
            #offset[2] = 0.0 - output2[2]
            print "Sensor 2 calibrated"

def accelerometer_reference():
    global reference_accelerometer_acc
    reference_accelerometer_acc = 0
    reference_accelerometer = ADXL345(0x1d)
    while True:
        time.sleep(DT)
        reference_accelerometer_acc = reference_accelerometer.getAxes(True)

def accelerometer_stabilized():
    global stabilized_accelerometer_acc
    stabilized_accelerometer_acc = 0
    stabilized_accelerometer = ADXL345(0x53)
    while True:
        time.sleep(DT)
        stabilized_accelerometer_acc = stabilized_accelerometer.getAxes(True)

def gyro_reference():
    global reference_gyro_omega
    reference_gyro_omega = 0
    reference_gyro = L3GD20(busId = 1, slaveAddr = 0x6a, ifLog = False, ifWriteBlock=False)
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
def gyro_stabilized():
    global stabilized_gyro_omega
    stabilized_gyro_omega = 0
    stabilized_gyro = L3GD20(busId = 1, slaveAddr = 0x6b, ifLog = False, ifWriteBlock=False)
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
readGyroRef = Thread(target=gyro_reference, args=())
readAccRef = Thread(target=accelerometer_reference, args=())
readGyroStab = Thread(target=gyro_stabilized, args=())
readAccStab = Thread(target=accelerometer_stabilized, args=())
readGyroRef.start()
readAccRef.start()
readGyroStab.start()
readAccStab.start()
filterRef = Thread(target=filter_reference, args=())
filterStab = Thread(target=filter_stabilized, args=())
filterRef.start()
filterStab.start()
controlMotorX = Thread(target=motorX, args=())
controlMotorX.start()
#controlMotorY = Thread(target=motorY, args=())
#controlMotorY.start()


print "Catching up..."
time.sleep(2)
print "Calibrating..."
time.sleep(45)
while True:
    print output1[0:2]
    print output2[0:2]
    #print("x1:{: 7.0f} y1:{:7.0f} z1:{:7.0f}".format(output1[0], output1[1], output1[2]));
    #print("x2:{:7.0f} y2:{:7.0f} z2:{:7.0f}".format(output2[0], output2[1], output2[3]));
    time.sleep(DT)
