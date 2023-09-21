#include "hardware/i2c.h"
#include "mpu6050.h"

//Registers of MPU6050
uint8_t device_address = 0x68;

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

uint8_t mpu_int_status(){
    uint8_t data = read_word_2c(0x3A);
    if (data == 0x0A){
        return true;
    }
    else{
        return false;
    }
}