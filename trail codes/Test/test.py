from imu import MPU6050
import time
from machine import Pin, I2C

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
MPU6050._chip_id = 0x75
imu = MPU6050(i2c)

while True:
    #print(imu.accel.xyz,imu.gyro.xyz,imu.temperature,end='\r')
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x)
    gy=round(imu.gyro.y)
    gz=round(imu.gyro.z)
    tem=round(imu.temperature,2)
    print(ax,"\t",ay,"\t",az,"\t",gx,"\t",gy,"\t",gz,"\t",tem,"        ",end="\r")
    time.sleep(0.5)


