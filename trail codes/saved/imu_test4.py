from imu import MPU6050
from time import sleep
from machine import Pin, I2C
import math
import utime as time

i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)

imu.accel_range = 2
imu.gyro_range = 2
imu.sample_rate = 255

settling_time = 4
print('Settling MPU for %d seconds' % settling_time)
time.sleep(4)
print('MPU is Done Settling')


def get_gyro():
    gx=imu.gyro.x
    gy=imu.gyro.y
    gz=imu.gyro.z
    print(gx, gy, gz)
    return gx, gy, gz
    

def gyro_calibration(calibration_time=10):
    """
        Description: This is a function to get the offset values
            for gyro calibration for mpu6050.
        
        Parameters:
        
        calibration_time[int]: Time in seconds you want to calibrate
            mpu6050. The longer the time the more accurate the
            calibration
    
        Outputs: Array with offsets pertaining to three axes of
            rotation [offset_gx, offset_gy, offset_gz]. Add these
            offsets to your sensor readins later on for more
            accurate readings!
    """
    print('--' * 25)
    print('Beginning Gyro Calibration - Do not move the MPU6050')
    
    # placeholder for the average of tuples in mpu_gyro_array
    offsets = [0, 0, 0]
    # placeholder for number of calculations we get from the mpu
    num_of_points = 0
    
    # We get the current time and add the calibration time
    end_loop_time = time.time() + calibration_time
    # We end the loop once the calibration time has passed
    while end_loop_time > time.time():
        num_of_points += 1
        (gx, gy, gz) = get_gyro()
        offsets[0] += gx
        offsets[1] += gy
        offsets[2] += gz
        
        # This is just to show you its still calibrating :)
        if num_of_points % 100 == 0:
            print('Still Calibrating Gyro... %d points so far' % num_of_points)
        
    print('Calibration for Gyro is Complete! %d points total' % num_of_points)
    offsets = [i/num_of_points for i in offsets] # we divide by the length to get the mean
    print("cabliration done.....")
    return offsets
  
offsets = gyro_calibration(5)
print(offsets)
time.sleep(5)

aa_x, aa_y, aa_z, ga_x, ga_y, ga_z = 0,0,0,0,0,0
prevtime = time.ticks_us()

while True:
    ax=round(imu.accel.x,2)
    ay=round(imu.accel.y,2)
    az=round(imu.accel.z,2)
    gx=round(imu.gyro.x, 4)
    gy=round(imu.gyro.y, 4)
    gz=round(imu.gyro.z, 4)
    tem=round(imu.temperature,2)
    
    #print("gx:",gx," gy:",gy," gz:",gz)
    times = time.ticks_us()
    ga_x = ga_x + gx*(times-prevtime)/1000000
    prevtime = time.ticks_us()
    print(round(ga_x, 2))