/***************************************************************************
 * 
 * This is an example for the sensor Grove MPU6050. MPU6050 is a six axis
 * Gyroscope and Accelerometer sensor will fall and motion detection algorimthms
 * integrated with it.
 * https://gitlab.com/UJUR007/mpu6050_ind
 * 
 * This is an example code to trigger a hardware interuppt if the fall is detected
 * on the MPU6050 sensoe beyond given thresold.
 * 
 * The example is specifically designed to work on Arduino Uno development board.
 * For other development boards or microcontroller architecture one may need to
 * modify the following code.
 * 
 *************************************************************************/

#include <MPU6050reg.h>
#include <Wire.h>
#include "MPU6050IMU.h"

#define INTRUPPT_PIN 3
#define LED_PIN LED_BUILTIN


MPU6050IMU imu;
volatile bool falldetection = false;

int count = 0;

void interrupt_service_routine()
{
  Serial.println("Fall..");
  falldetection = true;  
  digitalWrite(LED_PIN, HIGH);
  detachInterrupt(digitalPinToInterrupt(3));
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  //attachInterrupt(digitalPinToInterrupt(3), interrupt_service_routine, CHANGE);  
  Serial.begin(9600);
  imu.begin();
  imu.enablefreefall(0xF0, 0xFF);  
  imu.disablemotiondetection();
  imu.enablemotiondetection(10, 40);
  pinMode(INTRUPPT_PIN, INPUT);
  digitalWrite(INTRUPPT_PIN, LOW);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(500);
  
  if(falldetection == true)
  {
    count += 1;
    if(count==10)
    {
      digitalWrite(LED_PIN, LOW);
      falldetection = false;  
      //attachInterrupt(digitalPinToInterrupt(3), interrupt_service_routine, CHANGE);   
      imu.readbyte(INT_STATUS);
      count=0;
    }
    //Serial.println("Fall detected....LED is Turned ON");   
    delay(100); 
  }
  //Serial.print("Count ");
  Serial.println(digitalRead(3));
  if(digitalRead(3)){
    imu.readbyte(INT_STATUS);
  }
    
}
