from imu import MPU6050
from time import sleep
from machine import Pin, I2C
import math
import utime as time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
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

total_angle_y = 0
elapsedtime =0
time_prev = 0

SF_G = 1
SF_M_S2 = 9.80665 # 1 g = 9.80665 m/s2 ie. standard gravity
SF_DEG_S = 1
SF_RAD_S = 57.295779513082 # 1 rad/s is 57.295779578552 deg/s
rad_to_deg = 180/3.141592654; 

'''
    Set accelerometer range
    Pass:               0   1   2   3
    for range +/-:      2   4   8   16  g
        '''
imu.accel_range = 2 

'''
    Set gyroscope range
    Pass:               0   1   2    3
    for range +/-:      250 500 1000 2000  degrees/second
        '''
imu.gyro_range = 2


# def dist(a,b):
#         return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def y_angle():
    #print("y+angle")
    global elapsedtime, total_angle_y, time_prev
    elapsedtime = (time.ticks_us() - time_prev)/1000000.0
    time_prev = time.ticks_us()
    #Now we integrate the raw value in degrees per seconds in order to obtain the angle
    #If you multiply degrees/seconds by seconds you obtain degrees
    gyro_angle_x = imu.gyro.x * elapsedtime
    gyro_angle_y = imu.gyro.y * elapsedtime
    
    acc_raw_x = imu.accel.x
    acc_raw_y = imu.accel.y
    acc_raw_z = imu.accel.z
    
    acc_angle_x = (math.atan((acc_raw_y)/math.sqrt(pow((acc_raw_x),2) + pow((acc_raw_z),2)))*rad_to_deg)
    acc_angle_y = (math.atan(-1*(acc_raw_x)/math.sqrt(pow((acc_raw_y),2) + pow((acc_raw_z),2)))*rad_to_deg)
    
    total_angle_y = 0.98*(total_angle_y + gyro_angle_y) + 0.02*acc_angle_y;
    print(total_angle_y)
    #return total_angle_y

while True:
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x, 4)
    gy=round(imu.gyro.y, 4)
    gz=round(imu.gyro.z, 4)
    tem=round(imu.temperature,2)
    #print(get_x_rotation(ax,ay,az), "\t", get_y_rotation(ax,ay,az), end="\r")
    #x= round(math.degrees(math.atan2(-ay, -az)+ math.pi),2)
    y= round(math.degrees(math.atan2(-ax, -az)+math.pi),2)
    #print(y, end="\r")
    #print("	RPM / Degree/sec : ",gz-1," Temperature",tem, " inclination : ",y, "        ",end="\r")
    #print("	AX : ",gx," AY : ",gy, " AZ : ",gz, "        ",end="\r")
    #y_angl = y_angle()
    #print(y_angle(), end="\r")
    y_angle()
    sleep(0.2)