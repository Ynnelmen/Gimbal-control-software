# PyRoscope - IMU software for gimbal-control, written by Jeremie Reusser.
# Beta 0.2

from ADXL345.adxl345 import ADXL345
from L3GD20.L3GD20 import L3GD20
from threading import Thread
import RPi.GPIO as GPIO
import time
import math

DT = 0.02

def filter_reference():
    global x1, y1, z1, output1
    x1 = y1 = z1 = 0.0
    offset = [0.0,0.0,0.0]
    output1 = [0.0,0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        output1[0] = offset[0] + .98*(output1[0]+reference_gyro_omega[0]*DT-offset[0])+.02*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['y'], reference_accelerometer_acc['z'])+math.pi)
        output1[1] = .98*(output1[1]+reference_gyro_omega[1]*DT)+.02*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['z'], reference_accelerometer_acc['x'])+math.pi)
        output1[2] = .99999*(output1[2]+reference_gyro_omega[2]*DT)+.00001*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['x'], reference_accelerometer_acc['y'])+math.pi)
        iterations += 1    
        time.sleep(DT)
        if iterations == 20*(1/DT):
            offset[0] = 0.0 - output1[0]
            offset[1] = 0.0 - output1[1]
            offset[2] = 0.0 - output1[2]
            print "Sensor 1 calibrated"
    
def filter_stabilized():
    global x2, y2, z2
    x2 = y2 = z2 = 0.0
    offset = output = [0.0,0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        output[0] = .98*(output[0]+stabilized_gyro_omega[0]*DT)+.02*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['y'], stabilized_accelerometer_acc['z'])+math.pi)
        output[1] = .98*(output[1]+stabilized_gyro_omega[1]*DT)+.02*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['z'], stabilized_accelerometer_acc['x'])+math.pi)
        output[2] = .99999*(output[2]+stabilized_gyro_omega[2]*DT)+.00001*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['x'], stabilized_accelerometer_acc['y'])+math.pi)
        x2 = output[0] + offset[0] 
        z2 = output[1] + offset[1]
        y2 = output[2] + offset[2]
        iterations += 1    
        time.sleep(DT)
        if iterations == 1000:
            offset[0] = 0.0 - output[0]
            offset[1] = 0.0 - output[1]
            offset[2] = 0.0 - output[2]
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
print "Catching up..."
time.sleep(2)
print "Calibrating..."
#time.sleep(20)
while True:
    print output1
    #print("x1:{: 7.0f} y1:{:7.0f} z1:{:7.0f}".format(x1, y1, z1));
    #print("x2:{:7.0f} y2:{:7.0f} z2:{:7.0f}".format(x2, y2, z2));
    time.sleep(DT)
	
