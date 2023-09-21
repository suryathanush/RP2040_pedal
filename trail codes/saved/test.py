from mpu6050 import MPU6050
from time import sleep
import utime
from machine import Pin, I2C
import math

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)

while True:
    print(imu.ypr)
    #sleep(0.2)
    