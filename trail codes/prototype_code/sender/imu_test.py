from imu import MPU6050
from time import sleep
from machine import Pin, I2C
import math
import utime as time

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)

accCoef = 0.02
gyroCoef = 0.98
angleGyroX = 0
angleGyroY = 0
angleGyroZ = 0
angleX = 0
angleZ = 0
angleY = 0
gyroXoffset = 0
gyroYoffset = 0
gyroZoffset = 0

SF_G = 1
SF_M_S2 = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 57.295779513082 # 1 rad/s is 57.295779578552 deg/s

def dist(a,b):
        return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

while True:
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    tem=round(imu.temperature,2)
    #print(get_x_rotation(ax,ay,az), "\t", get_y_rotation(ax,ay,az), end="\r")
    #x= round(math.degrees(math.atan2(-ay, -az)+ math.pi),2)
    y= round(math.degrees(math.atan2(-ax, -az)+math.pi),2)
    #print(y, end="\r")
    print("	RPM / Degree/sec : ",gz-1," Temperature",tem, " inclination : ",y, "        ",end="\r")
    sleep(0.2)