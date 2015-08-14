from ADXL345.adxl345 import ADXL345
from L3GD20.L3GD20 import L3GD20
from threading import Thread
import RPi.GPIO as GPIO
import time

def accelerometer_reference():
    global ax1, ay1, az1
    ax1 = 0
    ay1 = 0
    az1 = 0
    axyz1 = 0
    adxl345 = ADXL345(0x1d)
    axyz1 = adxl345.getAxes(True)
    ax1 = axyz1['x']
    ay1 = axyz1['y']
    az1 = axyz1['z']
    print "ADXL345 on address 0x%x:" % (adxl345.address)
    
accelerometer_reference()
print ax1, ay1, az1