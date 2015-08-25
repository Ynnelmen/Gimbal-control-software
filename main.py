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
    global x1, y1, z1, offset1
    x1 = y1 = z1 = 0
    offset1 = [0.0,0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        x1 = .98*(x1+reference_gyro_omega[0]*DT)+.02*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['y'], reference_accelerometer_acc['z'])+math.pi)
        y1 = .98*(y1+reference_gyro_omega[1]*DT)+.02*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['z'], reference_accelerometer_acc['x'])+math.pi)
        z1 = .99999*(z1+reference_gyro_omega[2]*DT)+.00001*(360/(2*math.pi))*(math.atan2(reference_accelerometer_acc['x'], reference_accelerometer_acc['y'])+math.pi)
        iterations += 1    
        time.sleep(DT)
        if iterations == 1000:
            offset1[0] = 0.0 - x1
            offset1[1] = 0.0 - y1
            offset1[2] = 0.0 - z1
    
def filter_stabilized():
    global x2, y2, z2, offset2
    x2 = y2 = z2 = 0
    offset2 = [0.0,0.0,0.0]
    iterations = 0
    time.sleep(2)
    while True:
        x2 = .98*(x2+stabilized_gyro_omega[0]*DT)+.02*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['y'], stabilized_accelerometer_acc['z'])+math.pi)
        y2 = .98*(y2+stabilized_gyro_omega[1]*DT)+.02*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['z'], stabilized_accelerometer_acc['x'])+math.pi)
        z2 = .99999*(z2+stabilized_gyro_omega[2]*DT)+.00001*(360/(2*math.pi))*(math.atan2(stabilized_accelerometer_acc['x'], stabilized_accelerometer_acc['y'])+math.pi)
        iterations += 1    
        time.sleep(DT)
        if iterations == 1000:
            offset2[0] = 0.0 - x2
            offset2[1] = 0.0 - y2
            offset2[2] = 0.0 - z2

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
while True:
    print("x1:{: 7.0f} y1:{:7.0f} z1:{:7.0f}".format(x1+offset1[0], y1+offset1[1], z1+offset1[2]));
    print("x2:{:7.0f} y2:{:7.0f} z2:{:7.0f}".format(x2+offset2[0], y2+offset2[1], z2+offset2[2]));
    time.sleep(DT)
	
