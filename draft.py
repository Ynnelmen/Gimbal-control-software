from ADXL345.adxl345 import ADXL345
from L3GD20.L3GD20 import L3GD20
from threading import Thread
import RPi.GPIO as GPIO
import time

def filter():
    CFangleX=.98*(CFangleX+rate_gyr_x*DT) +(1 - .02) * AccXangle;

def accref():
    while True:
        axesref = adxl345.getAxes(True)

def accstab():
    while True:
        axesstab = adxl345.getAxes(True)

def gyroref():
    x1 = 0
    y1 = 0
    z1 = 0
    while True:
        time.sleep(dt)
        dxyz1 = s.Get_CalOut_Value()
        x1 += dxyz1[0]*dt;
        y1 += dxyz1[1]*dt;
        z1 += dxyz1[2]*dt;

def gyrostab():
    x2 = 0
    y2 = 0
    z2 = 0
    while True:
        time.sleep(dt)
        dxyz2 = t.Get_CalOut_Value()
        x2 += dxyz2[0]*dt;
        y2 += dxyz2[1]*dt;
        z2 += dxyz2[2]*dt;

def config():
    global dt
    dt = 0.02
    s = L3GD20(busId = 1, slaveAddr = 0x6a, ifLog = False, ifWriteBlock=False)
    t = L3GD20(busId = 1, slaveAddr = 0x6b, ifLog = False, ifWriteBlock=False)
    s.Set_PowerMode("Normal")
    s.Set_FullScale_Value("250dps")
    s.Set_AxisX_Enabled(True)
    s.Set_AxisY_Enabled(True)
    s.Set_AxisZ_Enabled(True)
    t.Set_PowerMode("Normal")
    t.Set_FullScale_Value("250dps")
    t.Set_AxisX_Enabled(True)
    t.Set_AxisY_Enabled(True)
    t.Set_AxisZ_Enabled(True)
    s.Init()
    s.Calibrate()
    t.Init()
    t.Calibrate()
    adxl345 = ADXL345()

config()
readGyroRef = Thread(target=gyroref, args=())
readGyroStab = Thread(target=gyrostab, args=())
readAccRef = Thread(target=accref, args=())
readAccStab = Thread(target=accstab, args=())
readGyroRef.start()
readGyroStab.start()
readAccRef.start()
readAccStab.start()
filterData = Thread(target=filter, args=())
filterData.start()

#print("{:7.2f} {:7.2f} {:7.2f}".format(x1, y1, z1))
