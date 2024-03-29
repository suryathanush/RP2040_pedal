	
from machine import Pin, I2C
import utime
import math
from imu import MPU6050

def get_mpu6050_data():
    temp = imu.temperature
    accel_x = imu.accel.x
    accel_y = imu.accel.y
    accel_z = imu.accel.z
    gyro_x = imu.gyro.x
    gyro_y = imu.gyro.y
    gyro_z = imu.gyro.z
 
    return {
        'temp': temp,
        'accel': {
            'x': accel_x,
            'y': accel_y,
            'z': accel_z,
        },
        'gyro': {
            'x': gyro_x,
            'y': gyro_y,
            'z': gyro_z,
        }
    }

def calculate_tilt_angles(accel_data):
    x, y, z = accel_data['x'], accel_data['y'], accel_data['z']
 
    tilt_x = math.atan2(y, math.sqrt(x * x + z * z)) * 180 / math.pi
    tilt_y = math.atan2(-x, math.sqrt(y * y + z * z)) * 180 / math.pi
    tilt_z = math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi
 
    return tilt_x, tilt_y, tilt_z
 
def complementary_filter(pitch, roll, gyro_data, dt, alpha=0.98):
    pitch += gyro_data['x'] * dt
    roll -= gyro_data['y'] * dt
 
    pitch = alpha * pitch + (1 - alpha) * math.atan2(gyro_data['y'], math.sqrt(gyro_data['x'] * gyro_data['x'] + gyro_data['z'] * gyro_data['z'])) * 180 / math.pi
    roll = alpha * roll + (1 - alpha) * math.atan2(-gyro_data['x'], math.sqrt(gyro_data['y'] * gyro_data['y'] + gyro_data['z'] * gyro_data['z'])) * 180 / math.pi
 
    return pitch, roll
 
i2c = I2C(0, sda=Pin(12), scl=Pin(13), freq=400000)
imu = MPU6050(i2c)
 
imu.accel_range = 2
imu.gyro_range = 2

pitch = 0
roll = 0
prev_time = utime.ticks_ms()
 
while True:
    data = get_mpu6050_data()
    curr_time = utime.ticks_ms()
    dt = (curr_time - prev_time) / 1000
 
    tilt_x, tilt_y, tilt_z = calculate_tilt_angles(data['accel'])
    pitch, roll = complementary_filter(pitch, roll, data['gyro'], dt)
 
    prev_time = curr_time
 
    print("Temperature: {:.2f} °C".format(data['temp']))
    print("Tilt angles: X: {:.2f}, Y: {:.2f}, Z: {:.2f} degrees".format(tilt_x, tilt_y, tilt_z))
    print("Pitch: {:.2f}, Roll: {:.2f} degrees".format(pitch, roll))
    print("Acceleration: X: {:.2f}, Y: {:.2f}, Z: {:.2f} g".format(data['accel']['x'], data['accel']['y'], data['accel']['z']))
    print("Gyroscope: X: {:.2f}, Y: {:.2f}, Z: {:.2f} °/s".format(data['gyro']['x'], data['gyro']['y'], data['gyro']['z']))
    utime.sleep(1)