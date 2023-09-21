#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <math.h>
#include <stdint.h>
#include "pico/time.h"
//#include "pico/sleep.h"

#define WAKE_PIN 10
#define LED_PIN 25


//global variables

const float alpha = 0.3;
const float dt = 0.1; // Sample time in seconds
float yaw_gyro = 0.0;
float gyro_bias_z = 0.0;
int16_t gyro_offset[3] = {21, -2, 32};
const uint i2c_speed = 400000; // Replace 100000 with your desired clock speed (in Hz)

//Registers of MPU6050
uint8_t device_address = 0x69;

const uint8_t PWR_MGMT_1 = 0x6B;
const uint8_t SIGNAL_PATH_RESET = 0x68;
const uint8_t I2C_SLV0_ADDR = 0x37;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t MOT_THR = 0x1F; // Motion detection threshold bits [7:0]
const uint8_t MOT_DUR = 0x20;// Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t INT_ENABLE = 0x38;
const uint8_t WHO_AM_I_MPU6050 = 0x75;// # Should return 0x68
const uint8_t INT_STATUS = 0x3A;

const uint8_t GYRO_CNF = 0x1B;
const uint8_t ACC_CNF = 0x1C;

const uint8_t ACCEL_XOUT_H = 0x3B;
const uint8_t ACCEL_XOUT_L = 0x3C;
const uint8_t ACCEL_YOUT_H = 0x3D;
const uint8_t ACCEL_YOUT_L = 0x3E;
const uint8_t ACCEL_ZOUT_H = 0x3F;
const uint8_t ACCEL_ZOUT_L = 0x40;

const uint8_t GYRO_XOUT_H = 0x43;
const uint8_t GYRO_XOUT_L = 0x44;
const uint8_t GYRO_YOUT_H = 0x45;
const uint8_t GYRO_YOUT_L = 0x46;
const uint8_t GYRO_ZOUT_H = 0x47;
const uint8_t GYRO_ZOUT_L = 0x48;

//_ACCEL_FS_MASK = const(0b00011000)
uint8_t ACCEL_FS_SEL_2G = 0b00000000;
uint8_t ACCEL_FS_SEL_4G = 0b00001000;
uint8_t ACCEL_FS_SEL_8G = 0b00010000;
uint8_t ACCEL_FS_SEL_16G = 0b00011000;

int ACCEL_SO_2G = 16384; // 1 / 16384 ie. 0.061 mg / digit
int ACCEL_SO_4G = 8192; // 1 / 8192 ie. 0.122 mg / digit
int ACCEL_SO_8G = 4096; // 1 / 4096 ie. 0.244 mg / digit
int ACCEL_SO_16G = 2048; // 1 / 2048 ie. 0.488 mg / digit

//_GYRO_FS_MASK = const(0b00011000)
uint8_t GYRO_FS_SEL_250DPS = 0b00000000;
uint8_t GYRO_FS_SEL_500DPS = 0b00001000;
uint8_t GYRO_FS_SEL_1000DPS = 0b00010000;
uint8_t GYRO_FS_SEL_2000DPS = 0b00011000;

int GYRO_SO_250DPS = 131;
int GYRO_SO_500DPS = 65.5;
int GYRO_SO_1000DPS = 32.8;
int GYRO_SO_2000DPS = 16.4;

//function to write to register
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    const uint i2c_speed = 400000; // Set the I2C speed to 400 kHz
    uint8_t buffer[2] = {subAddress, data};

    // Initialize I2C interface
    i2c_init(i2c_default, i2c_speed);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);

    // Write the data to the I2C device
    i2c_write_blocking(i2c_default, address, buffer, 2, false);
}

//function to read from register
int16_t read_word_2c(uint8_t register_address) {
    const uint i2c_speed = 400000; // Set the I2C speed to 400 kHz

    // Initialize I2C interface
    i2c_init(i2c_default, i2c_speed);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);

    uint8_t high_byte, low_byte;
    int16_t value = 0;

    // Read the high byte
    if (i2c_read_blocking(i2c_default, 0x68, &register_address, 1, true) == PICO_ERROR_GENERIC) {
        printf("Error reading high byte.\n");
        return value;
    }
    high_byte = register_address; // Assign the read value to high_byte

    // Read the low byte
    if (i2c_read_blocking(i2c_default, 0x68, &register_address, 1, false) == PICO_ERROR_GENERIC) {
        printf("Error reading low byte.\n");
        return value;
    }
    low_byte = register_address; // Assign the read value to low_byte

    value = (high_byte << 8) + low_byte;
    if (value >= 0x8000) {
        value = -((65535 - value) + 1);
    }
    
    return value;
}
//Reading gyroscope and accelrometer  values

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = ACCEL_XOUT_H;
    i2c_write_blocking(i2c_default, device_address, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, device_address, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = GYRO_XOUT_H;
    i2c_write_blocking(i2c_default, device_address, &val, 1, true);
    i2c_read_blocking(i2c_default, device_address, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

   
}

//calebration is done once, no need to calebrate on every boot
//when running for first time on new hardware
//uncomment the print command, and see the values
//then update the offset values accordingly
void calibrate_gyroscope(int16_t gyro[3], int num_samples, int16_t* gyro_offset) {
    int16_t sum[3] = {0, 0, 0};
  
    for (int i = 0; i < num_samples; i++) {
        int16_t gyro_data[3];
        mpu6050_read_raw(NULL, gyro_data); // Read only gyroscope data
        sum[0] += gyro_data[0];
        sum[1] += gyro_data[1];
        sum[2] += gyro_data[2];
        sleep_ms(10); // Adjust the sampling interval as needed
    }
  
    // Calculate the average
    gyro_offset[0] = sum[0] / num_samples;
    gyro_offset[1] = sum[1] / num_samples;
    gyro_offset[2] = sum[2] / num_samples;
    printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro_offset[0], gyro_offset[1], gyro_offset[2]);

}

//getting the roll, pitch, and yaw values
void calculate_angles(int16_t accel[3], int16_t gyro[3], float* roll, float* pitch, float* yaw) {
    // Convert raw accelerometer data to G values
    float accel_x = accel[0] / 16384.0; // Assuming sensitivity scale factor of 16384 LSB/G
    float accel_y = accel[1] / 16384.0;
    float accel_z = accel[2] / 16384.0;

    // Convert raw gyroscope data to degrees per second
    float gyro_x = (gyro[0] - gyro_offset[0]) / 131.0; // Assuming sensitivity scale factor of 131 LSB/°/s
    float gyro_y = (gyro[1] - gyro_offset[1]) / 131.0;
    float gyro_z = (gyro[2] - gyro_offset[2]) / 131.0;

    // printf("accelX: %.2f G, accelY: %.2f G, accelZ: %.2f G\n", accel_x, accel_y, accel_z);
    // printf("gyroX: %.2f G, gyroY: %.2f G, gyroZ: %.2f G\n", gyro_x, gyro_y, gyro_z);
    // Complementary Filter: Update roll and pitch angles using accelerometer data
    float accel_roll = atan2(accel_y, sqrt(accel_x * accel_x + accel_z * accel_z)) * 180.0 / M_PI;
    float accel_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;

    // Calculate roll and pitch angles from gyroscope data
    *roll += gyro_x * dt;
    *pitch += gyro_y * dt;

    // Complementary Filter: Combine accelerometer and gyroscope data
    *roll = alpha * *roll + (1 - alpha) * accel_roll;
    *pitch = alpha * *pitch + (1 - alpha) * accel_pitch;

    // Calculate yaw angle from gyroscope data (optional, assuming yaw = 0 at the start)
    //*yaw += gyro_z * dt;
    float yaw_acc_adjust = atan2(accel_y, accel_z);
    gyro_bias_z = (1 - 0.1) * gyro_bias_z + 0.01 * gyro_z;
    yaw_gyro -= gyro_bias_z * dt;
    *yaw = alpha * (*yaw + yaw_gyro) + (1 - alpha) * yaw_acc_adjust;
}

//this function is the main function which will
//run when pico is awaken
void wakeUp(){
    int16_t acceleration[3], gyro[3];
    // Initialize I2C interface
    i2c_init(i2c_default, i2c_speed);
    gpio_set_function(12, GPIO_FUNC_I2C);
    gpio_set_function(13, GPIO_FUNC_I2C);
    gpio_pull_up(12);
    gpio_pull_up(13);
    
    // gpio_put(LED_PIN, 1);
    // sleep_ms(250);
    // gpio_put(LED_PIN, 0);
    // sleep_ms(250);
    // gpio_put(LED_PIN, 1);
    // sleep_ms(250);
    // gpio_put(LED_PIN, 0);
    // sleep_ms(250);
    
    // Initialize MPU6050
    
    int16_t gyro_offset[3];
    //calibrate_gyroscope(gyro, 1000, gyro_offset);
    float roll = 0.0, pitch = 0.0, yaw = 0.0;

    mpu6050_read_raw(acceleration, gyro );
    calculate_angles(acceleration, gyro, &roll, &pitch, &yaw);
    printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
    printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
    printf("Roll: %.2f degrees, Pitch: %.2f degrees, Yaw: %.2f degrees\n", roll, pitch, yaw);
    sleep_ms(200);
}


int main() {
    setup_default_uart();
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(WAKE_PIN, GPIO_IN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    //since the pico code runs fast to look at all the print statements 
    //starting from main function 
    //uncomment this sleep_ms() command
    //pico will wait so you have time to open putty or other program
    //sleep_ms(5000);

    // ... pico main configurations
    writeByte(device_address, PWR_MGMT_1, 0x00);
    writeByte(device_address, SIGNAL_PATH_RESET, 0x07);
    // ... other MPU6050 configurations
    //writeByte(device_address,GYRO_CNF, GYRO_FS_SEL_250DPS);
    //writeByte(device_address,ACC_CNF, ACCEL_FS_SEL_2G);
    writeByte(device_address, I2C_SLV0_ADDR, 0x20); //write register 0x37 to select how to use the interrupt pin. For an active high, push-pull signal that stays until register (decimal) 58 is read, write 0x20.
    writeByte(device_address, ACCEL_CONFIG, 0x01); //Write register 28 (==0x1C) to set the Digital High Pass Filter, bits 3:0. For example set it to 0x01 for 5Hz. (These 3 bits are grey in the data sheet, but they are used! Leaving them 0 means the filter always outputs 0.)
    writeByte(device_address, MOT_THR, 1); //Write the desired Motion threshold to register 0x1F (For example, write decimal 20).  
    writeByte(device_address, MOT_DUR, 90); //Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate  
    writeByte(device_address, MOT_DETECT_CTRL, 0x15); //to register 0x69, write the motion detection decrement and a few other settings (for example write 0x15 to set both free-fall and motion decrements to 1 and accelerometer start-up delay to 5ms total by adding 1ms. )   
    writeByte(device_address, INT_ENABLE, 0x40); //write register 0x38, bit 6 (0x40), to enable motion detection interrupt.     
    writeByte(device_address, 0x37, 0x20); // now INT pin is active low
    
    //put pico in low power mode
    //sleep_run_from_xosc();
   
   
    while (1) {
        uint8_t data = read_word_2c(0x3A);
        while (data != 0x0A){
            data = read_word_2c(0x3A);
            //printf("Received data: %02X\n", data);
         }
        //dorment sleep mode command
        //this is the lowest power consuming mode of pico 
        sleep_goto_dormant_until_edge_high(WAKE_PIN);
        //when pico wakes up the below function will run once
        //if (gpio_get(WAKE_PIN)){
        wakeUp();
        //}
    }
}