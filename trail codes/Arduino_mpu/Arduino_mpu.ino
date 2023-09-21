//#include <Wire.h>
//
//uint8_t _address = 0x69;
//
//uint8_t SMPLRT_DIV = 0x19;
//uint8_t GYRO_CONFIG = 0x1B;
//uint8_t ACCEL_CONFIG = 0x1C;
//uint8_t MOT_THR = 0x1F;
//uint8_t INT_PIN_CFG = 0x37;
//uint8_t INT_ENABLE = 0x38;
//uint8_t INT_STATUS = 0x3A;
//uint8_t WHO_AM_I = 0x75;
//uint8_t PWR_MGMT_1 = 0x6B;
//uint8_t SIGNAL_PATH_RESET = 0x68;
//uint8_t MOT_DUR = 0x20;
//uint8_t MOT_DETECT_CTRL = 0x69;
//
//void setup() {
//  Serial.begin(115200);
//  pinMode(3, INPUT);
//  while(!Serial){}
//  delay(2000);
//  Wire.setSDA(12);
//  Wire.setSCL(13);
//  Wire.begin();
//
//  Serial.print("read : MOT_THR - ");
//  Serial.println(readRegister(MOT_THR), BIN);
//  delay(100);
//
//  writeRegister(PWR_MGMT_1, 0x00);
//  writeRegister(INT_PIN_CFG, 0xE0);
//  writeRegister(ACCEL_CONFIG, 0x01);
//  writeRegister(MOT_THR, 0x14);
//  writeRegister(MOT_DUR, 0x28);
//  writeRegister(MOT_DETECT_CTRL, 0x15);
//  writeRegister(INT_ENABLE, 0x40);
//  delay(1000);
//  Serial.println("----------------------------");
//  Serial.print("read : PWR_MGMT_1 - ");
//  Serial.println(readRegister(PWR_MGMT_1), HEX);
//  delay(100);
//  Serial.print("read : INT_PIN_CFG - ");
//  Serial.println(readRegister(INT_PIN_CFG), HEX);
//  delay(100);
//  Serial.print("read : ACCEL_CONFIG - ");
//  Serial.println(readRegister(ACCEL_CONFIG), HEX);
//  delay(100);
//  Serial.print("read : MOT_THR - ");
//  Serial.println(readRegister(MOT_THR), HEX);
//  delay(100);
//  Serial.print("read : MOT_DUR - ");
//  Serial.println(readRegister(MOT_DUR), HEX);
//  delay(100);
//  Serial.print("read : MOT_DETECT_CTRL - ");
//  Serial.println(readRegister(MOT_DETECT_CTRL), HEX);
//  delay(100);
//  Serial.print("read : INT_ENABLE - ");
//  Serial.println(readRegister(INT_ENABLE), HEX);
//  delay(100);
// 
//}
//
//void writeRegister(uint8_t myRegister, uint8_t myValue) {
//  Wire.beginTransmission(_address);
//  Wire.write(myRegister);
//  Wire.write(myValue);
//  Wire.endTransmission();
//}
//
//byte readRegister(uint8_t myRegister) {
//  Wire.beginTransmission(_address);
//  Wire.write(myRegister);
//  Wire.endTransmission();
//
//  Wire.requestFrom(_address, 1, true);
//  byte returnValue;
//  while (Wire.available()) {
//    returnValue = Wire.read();
//  }
//  return returnValue;
//}
//
//int prev_time = millis();
//void loop() {
//  if(!digitalRead(3)){
//    Serial.println("interrupt triggered");
//  }
//
//  if(millis()-prev_time > 1000){ 
//    uint8_t Status = readRegister(INT_STATUS);
//    Serial.println(Status);
//    prev_time = millis();
//  }
//}

#include "Wire.h"
#include "MPU6050.h"
//#include "avr/sleep.h"

#define LED_PIN 14
#define INTERRUPT_PIN 2

#define MOTION_THRESHOLD 1
#define MOTION_EVENT_DURATION 40

MPU6050 accelgyro;
volatile bool ledState = true;

void setup() {
  pinMode(3, INPUT_PULLUP);
  //Wire.setSDA(12);
  //Wire.setSCL(13);
  Wire.begin();
  Serial.begin(115200);
  while(!Serial){}
  accelgyro.initialize();

  // verify connection to MPU6050
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setAccelerometerPowerOnDelay(3);
  accelgyro.setInterruptMode(true); // Interrupts enabled
  accelgyro.setInterruptDrive(true);
  accelgyro.setInterruptLatch(0); // Interrupt pulses when triggered instead of remaining on until cleared
  accelgyro.setIntMotionEnabled(true); // Interrupts sent when motion detected

  // Set sensor filter mode.
  // 0 -> Reset (disable high pass filter)
  // 1 -> On (5Hz)
  // 2 -> On (2.5Hz)
  // 3 -> On (1.25Hz)
  // 4 -> On (0.63Hz)
  // 5 -> Hold (Future outputs are relative to last output when this mode was set)
  accelgyro.setDHPFMode(1);


  // Motion detection acceleration threshold. 1LSB = 2mg.
  accelgyro.setMotionDetectionThreshold(MOTION_THRESHOLD);

  // Motion detection event duration in ms
  accelgyro.setMotionDetectionDuration(MOTION_EVENT_DURATION);
}

void sleepNow() {
  //sleep_enable();
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), wakeUp, RISING);
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //sleep_cpu();
}

void wakeUp() {
  //sleep_disable();
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN));
  Serial.println("interrput triggered");
  //ledState = !ledState;
}

void loop() {
  digitalWrite(LED_PIN, ledState);
  //sleepNow();
  if(!digitalRead(3)){
    Serial.println("interrupt...");
  }
}
