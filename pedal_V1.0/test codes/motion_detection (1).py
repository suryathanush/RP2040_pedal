from machine import Pin, I2C,RTC
import utime
import math
import lowpower

from neopixel import Neopixel
import time

numpix = 1
pixels = Neopixel(numpix, 0, 16, "GRB")
 
yellow = (255, 100, 0)
orange = (255, 50, 0)
green = (0, 255, 0)
blue = (0, 0, 255)
red = (255, 0, 0)
color0 = red
 
pixels.brightness(50)
pixels.set_pixel(0, color0)
pixels.show()

int_pin = machine.Pin(3, machine.Pin.IN, machine.Pin.PULL_UP)
i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400000)

DORMANT_PIN = 3

#configuration registers used
PWR_MGMT_1 = 0x6B
SIGNAL_PATH_RESET = 0x68
I2C_SLV0_ADDR = 0x37
ACCEL_CONFIG = 0x1C
MOT_THR = 0x1F # Motion detection threshold bits [7:0]
MOT_DUR = 0x20 # Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
MOT_DETECT_CTRL = 0x69
INT_ENABLE = 0x38
WHO_AM_I_MPU6050 = 0x75 # Should return 0x68
INT_STATUS = 0x3A

#motion detection registers used
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

#device adress
MPU6050_ADDRESS = 0x69

#value to initilize variable for storing time 
prev_time = 0

# Function to read a 16-bit signed value from two registers
def read_word_2c(register):
    high_byte = i2c.readfrom_mem(MPU6050_ADDRESS, register, 1)[0]
    low_byte = i2c.readfrom_mem(MPU6050_ADDRESS, register + 1, 1)[0]
    value = (high_byte << 8) + low_byte
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

#function to calibrate the gyroscope and get offset values
def calibrate_gyroscope():
    print("Keep the MPU6050 stationary during calibration...")
    
    sum_gyro_x = 0
    sum_gyro_y = 0
    sum_gyro_z = 0
    num_samples = 1000  # Adjust this value based on your calibration preference

    # Read gyroscope data and calculate the average
    for _ in range(num_samples):
        gyro_x = read_word_2c(GYRO_XOUT_H)
        gyro_y = read_word_2c(GYRO_YOUT_H)
        gyro_z = read_word_2c(GYRO_ZOUT_H)
        sum_gyro_x += gyro_x
        sum_gyro_y += gyro_y
        sum_gyro_z += gyro_z
        utime.sleep(0.01)  # Add a small delay between samples

    # Calculate the average gyroscope readings
    avg_gyro_x = sum_gyro_x / num_samples
    avg_gyro_y = sum_gyro_y / num_samples
    avg_gyro_z = sum_gyro_z / num_samples

    print("Gyroscope Calibration Complete.")
    print("Average Gyroscope Readings (x, y, z):", avg_gyro_x, avg_gyro_y, avg_gyro_z)
    
    return avg_gyro_x, avg_gyro_y, avg_gyro_z
# Read accelerometer data (helper function)
def read_accel_data():
    accel_x = read_word_2c(ACCEL_XOUT_H)
    accel_y = read_word_2c(ACCEL_YOUT_H)
    accel_z = read_word_2c(ACCEL_ZOUT_H)
    return accel_x, accel_y, accel_z

# Read gyroscope data (helper function)
def read_gyro_data():
    gyro_x = read_word_2c(GYRO_XOUT_H) + 488.2
    gyro_y = read_word_2c(GYRO_YOUT_H) - 14.07
    gyro_z = read_word_2c(GYRO_ZOUT_H) + 36.833
    return gyro_x, gyro_y, gyro_z

#function to write to device subAdresses
def writeByte(address, subAddress, data):
    #i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400000)  # Initialize I2C on Pico (GPIO21 as SCL and GPIO20 as SDA)
    i2c.writeto_mem(address, subAddress, data)
    
# Function to calculate roll, pitch, and yaw angles using the sensor data and complementary filter
def get_roll_pitch_yaw(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, alpha):    # Calculate roll and pitch angles using accelerometer data
    roll = 0.0  # Initialize roll to zero or any other suitable initial value
    pitch = 0.0
    yaw = 0.0
    roll_acc = math.atan2(accel_y, accel_z) * 180.0 / math.pi
    pitch_acc = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / math.pi

    # Calculate the change in angles using the gyroscope data
    roll_gyro = gyro_x * dt
    pitch_gyro = gyro_y * dt
    yaw_gyro = gyro_z * dt *250
    # Fuse the accelerometer and gyroscope angles using the complementary filter
    roll = alpha * (roll + roll_gyro) + (1 - alpha) * roll_acc
    pitch = alpha * (pitch + pitch_gyro) + (1 - alpha) * pitch_acc

    roll_rad = roll * math.pi / 180.0
    pitch_rad = pitch * math.pi / 180.0
    
    # Estimate gyro bias (simple average over time)
    bias_adjust = 0.1 #adjust the values to get best results
    
    gyro_bias_z = 0.0
    gyro_bias_z = (1 - 0.1) * gyro_bias_z + 0.01 * gyro_z

    # Apply gyro bias compensation
    yaw_gyro -= gyro_bias_z * dt
    
    acc_total_vector = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
    pitch_acc_adjust = pitch_acc * math.cos(roll_rad) - roll_acc * math.sin(roll_rad)
    yaw_acc_adjust = math.atan2(accel_y, accel_z)
    yaw = alpha * (yaw + yaw_gyro) + (1 - alpha) * yaw_acc_adjust
    
    return roll, pitch, yaw

# Initialize accelerometer and gyroscope readings to calculate initial roll and pitch angles
accel_x, accel_y, accel_z = read_accel_data()
gyro_x, gyro_y, gyro_z = read_gyro_data()

# Calculate initial roll and pitch angles using accelerometer data
roll = math.atan2(accel_y, accel_z) * 180.0 / math.pi
pitch = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / math.pi
yaw = 0.0  # Initialize yaw to zero or any other suitable initial value

writeByte(MPU6050_ADDRESS, PWR_MGMT_1, bytes([0x00]))
writeByte(MPU6050_ADDRESS, SIGNAL_PATH_RESET, bytes([0x07])) #Reset all internal signal paths in the MPU-6050 by writing 0x07 to register 0x68;
writeByte(MPU6050_ADDRESS, I2C_SLV0_ADDR, bytes([0x20])) #write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, bytes([0x01])) #Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
writeByte(MPU6050_ADDRESS, MOT_THR, bytes([0x01])) #Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
writeByte(MPU6050_ADDRESS, MOT_DUR, bytes([0x5A])) #Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
writeByte(MPU6050_ADDRESS, MOT_DETECT_CTRL, bytes([0x15])) #to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
writeByte(MPU6050_ADDRESS, INT_ENABLE, bytes([0x40])) #write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
#writeByte(MPU6050_ADDRESS, 0x37, bytes([0xA0])) #now INT pin is active low
writeByte(MPU6050_ADDRESS, 0x37, bytes([0x20])) #now INT pin is active high
data = i2c.readfrom_mem(MPU6050_ADDRESS,0x3A ,  1)


print(i2c.readfrom_mem(MPU6050_ADDRESS, PWR_MGMT_1, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, I2C_SLV0_ADDR, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, ACCEL_CONFIG, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, MOT_THR, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, MOT_DUR, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, MOT_DETECT_CTRL, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, INT_ENABLE, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, 0x37, 1))
print(i2c.readfrom_mem(MPU6050_ADDRESS, 0x75, 1))

#writeByte(MPU6050_ADDRESS, 0x6B, bytes([0x28]))
writeByte(MPU6050_ADDRESS, 0x6C, bytes([0x07])) #

def wakeUpNow(pin): #// THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP    (i.e. after getting interrupt)
    dont_stop = 0
    prev_time = 0
    prev_roll = 0
    prev_pitch = 0
    prev_yaw = 0
    max_movement = 3
    print("interrupted")
    pixels.set_pixel(0, color0)
    pixels.show()
    while max_movement > 2:
        int_pin.irq(handler=None)  # Remove the existing interrupt handler
        
        
        current_time = utime.ticks_ms()
        # Read accelerometer data
        accel_x, accel_y, accel_z = read_accel_data()

        # Read gyroscope data
        gyro_x, gyro_y, gyro_z = read_gyro_data()
        
       
        # Convert accelerometer and gyroscope data to physical units (e.g., m/s^2 and deg/s)
        accel_x /= 16384.0  # Sensitivity scale factor for +/- 2g range
        accel_y /= 16384.0
        accel_z /= 16384.0

        gyro_x /= 131.0  # Sensitivity scale factor for +/- 250 deg/s range
        gyro_y /= 131.0
        gyro_z /= 131.0

        # Calculate the time interval (dt) between sensor readings
        dt = (current_time - prev_time) / 1000.0
        
        # You can choose a suitable value for alpha based on your application requirements
        alpha = 0

        # Calculate roll, pitch, and yaw angles using the sensor data
        roll, pitch, yaw = get_roll_pitch_yaw(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt, alpha)
        
        print(-180,roll,180)

        # Update the previous time for the next iteration
        
        if current_time - prev_time > 1:
            print("if ran")
            d_roll = abs(roll) - abs(prev_roll) 
            d_pitch = abs(pitch) - abs(prev_pitch) 
            d_yaw = abs(yaw) - abs(prev_yaw)
            d_roll, d_pitch, d_yaw = abs(d_roll), abs(d_pitch), abs(d_yaw)
            max_movement = max(d_roll, d_pitch, d_yaw)
            prev_roll = roll
            prev_pitch = pitch
            prev_yaw = yaw
        prev_time = current_time
        utime.sleep(0.1)
    pixels.set_pixel(0, (0,0,0))
    pixels.show()
    #writeByte(MPU6050_ADDRESS, 0x6B, bytes([0x28]))
    writeByte(MPU6050_ADDRESS, 0x6C, bytes([0x07])) 
    int_pin.irq(handler=wakeUpNow)
    data = i2c.readfrom_mem(MPU6050_ADDRESS, 0x3A, 1)
    lowpower.dormant_with_modes({
        DORMANT_PIN: (lowpower.EDGE_LOW | lowpower.EDGE_HIGH),
    })
    
    print("woke up")
    print("woke up2")
    print("woke up3")
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, bytes([0x00]))
    writeByte(MPU6050_ADDRESS, 0x6C, bytes([0x00]))
    
int_pin.irq(trigger = machine.Pin.IRQ_FALLING, handler = wakeUpNow)


#read register to initilize the interrupt function
while True:
    data = i2c.readfrom_mem(MPU6050_ADDRESS, 0x3A, 1)
    #print(data);
    #print(utime.ticks_ms(), data)
    if data == b'\x0a':
        print(utime.ticks_ms(), data)
        break
#main while loop
# while True:
#     print(".................................................................")
#     machine.deepsleep()
#     pass

#
