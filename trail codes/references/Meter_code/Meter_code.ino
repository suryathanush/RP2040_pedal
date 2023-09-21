////////////////////////Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>         //download here: https://www.electronoobs.com/eng_arduino_Adafruit_GFX.php
#include <Adafruit_SSD1306.h>     //downlaod here: https://www.electronoobs.com/eng_arduino_Adafruit_SSD1306.php
#include <VL53L0X.h>              //downlaod here: https://www.electronoobs.com/eng_arduino_VL53L0X.php
////////////////////////

////////////////////////OLED 64x124 display with i2c
#define OLED_RESET 11
Adafruit_SSD1306 display(OLED_RESET);
////////////////////////


static const unsigned char PROGMEM  LOGO[] ={ 
/* 0X00,0X01,0X80,0X00,0X1F,0X00, */
0X00,0X38,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0XFE,0X00,0X00,0X00,0X70,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X01,0XFF,0X0F,0XBF,0XF7,0X7F,0X80,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X01,0XFF,0X8F,0X1F,0XFF,0XFF,0XC0,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X03,0XFF,0X8F,0XDF,0XFF,0XFF,0XC0,0X30,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X07,0XFF,0XCF,0XDF,0XFF,0XFF,0XE0,0X3F,0X07,0XE6,0X0F,0XC3,0X9F,0XEF,0X81,0XE0,
0X07,0XFF,0XCF,0X9F,0XFF,0XFF,0XE0,0X3F,0X8F,0XEE,0X0F,0XCF,0XDF,0XEF,0XE3,0XF0,
0X07,0XFF,0XCF,0X7F,0XFF,0XFF,0XE0,0X3F,0X0F,0X0E,0X0E,0X0E,0X67,0X8E,0XE7,0X38,
0X0F,0XFE,0X6F,0X70,0X00,0XFF,0XF0,0X7F,0X0F,0X0E,0X0E,0X1C,0X67,0X8E,0XE7,0X38,
0X08,0X30,0X6F,0XE0,0X00,0XFF,0XF0,0X7F,0X0F,0XCE,0X0F,0X9C,0X07,0X8E,0XE7,0X3C,
0X08,0X00,0X6F,0XE0,0X00,0XFF,0XF8,0X7F,0X0F,0XCE,0X0F,0X9C,0X07,0X8F,0XCF,0X3C,
0X08,0X00,0X2F,0XFF,0XFC,0XFF,0XF8,0X7F,0X0F,0X0E,0X0E,0X1C,0X27,0X8E,0XE7,0X3C,
0X08,0X00,0X2F,0XFF,0XFC,0XFF,0XF8,0X7F,0X0F,0X0E,0X0E,0X1C,0X27,0X8E,0XE7,0X38,
0X00,0X00,0X1F,0XFF,0XFC,0XFD,0XFC,0X7F,0X0F,0X0E,0X0E,0X1E,0X67,0X8E,0XE7,0X38,
0X03,0X8E,0X1F,0XFF,0XFC,0XFC,0XFC,0X7F,0X0F,0XEF,0XEF,0XCF,0XC7,0X8E,0XE3,0XF0,
0X00,0X00,0X1F,0XFF,0XFC,0XFC,0XFE,0XFF,0X00,0X18,0X43,0XC1,0XE1,0XF0,0X78,0X00,
0X00,0X00,0X2C,0XFF,0XFC,0XF8,0X72,0XFE,0X00,0X1C,0X4E,0XE3,0X31,0XDC,0XCC,0X00,
0X00,0X00,0X6D,0XE0,0X01,0XF4,0X32,0XFE,0X00,0X1E,0X4E,0X67,0X39,0XDD,0X8C,0X00,
0X00,0X00,0X0B,0XE0,0X01,0XEC,0X03,0XFE,0X00,0X1E,0X4E,0X77,0X39,0XDD,0XE0,0X00,
0X00,0X00,0X03,0XE0,0X01,0XFC,0X21,0XFE,0X00,0X1F,0X5E,0X77,0X39,0XF8,0XF8,0X00,
0X00,0X00,0X0B,0XF0,0X03,0XDC,0X3F,0XFE,0X00,0X1F,0XDE,0X77,0X39,0XFC,0XFC,0X00,
0X00,0X70,0X89,0XFF,0XFF,0XFC,0X1F,0XFE,0X00,0X13,0XCE,0X77,0X39,0XDC,0X3C,0X00,
0X02,0X00,0X08,0XFF,0XFF,0XF8,0X1F,0XFE,0X00,0X13,0XCE,0X77,0X39,0XDD,0X8C,0X00,
0X01,0X00,0X0C,0XFF,0XFF,0XF8,0X1F,0XFE,0X00,0X11,0XCE,0X67,0X39,0XDD,0X8C,0X00,
0X01,0X06,0X8D,0XFF,0XFF,0XF8,0X1F,0XFE,0X00,0X11,0XC7,0XE3,0XF1,0XFC,0XFC,0X00,
0X00,0X1B,0X89,0XFF,0XFF,0XF8,0X0F,0XFC,0X00,0X00,0X01,0X80,0XC0,0X00,0X70,0X00,
0X00,0X09,0X8B,0XFF,0XFC,0XF8,0X0F,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X0D,0XC0,0X00,0X00,0X00,0X0F,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X07,0XC0,0X00,0X00,0X00,0X07,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X07,0X80,0X00,0X00,0X00,0X00,0XFC,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X01,0X00,0X00,0X00,0X00,0X00,0X08,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
};


//INPUTS/OUTPUTS
#define zero_button 4
#define mode_button 5
#define plus_button 6
#define sound_button 7
#define rot_data 8
//#define rot_clock 9
#define battery_in A0
#define sharp_in A1
#define IR_rpm_in 10
#define laser_out 2
#define buzzer 3


//Variables
bool mode_button_state = false;
bool zero_button_state = false;
bool plus_button_state = false;
bool sound_button_state = false;
int mode = 0;
float battery_level = 3.7;

//For Sharp sensor
int Sharp_cm_previous;

//For VL53L0X sensor
VL53L0X sensor;
int VL53L0X_mm_previous;

//Rotary encoder
int counter = 0; 
bool data_state;
bool sound = false;
bool laser = false;

//MPU6050 gyro/acc
//Gyro Variables
float elapsedTime, time, timePrev;        //Variables for time control
int gyro_error=0;                         //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read 
float Gyro_angle_x, Gyro_angle_y, Gyro_angle_z;         //Here we store the angle value obtained with Gyro data
//Acc Variables
int acc_error=0;                         //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;      //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read 
float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
float Total_angle_x, Total_angle_y, Total_angle_z;
int Z_loop = 0;
bool Z_error = false;
float Gyr_rawZ_error = 0;
float angle_x_error = -6.6;
float angle_y_error = 3.6;
int print_time = 0;

//IR sensor
//We create variables for the time width values of each PWM input signal
unsigned long counter_1, current_count;
//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_IR_state;
//To store the 1000us to 2000us value we create variables and store each channel
int width;      //In my case channel 4 of the receiver and pin D12 of arduino
bool IR_enable = false;




void setup() {
  //Serial.begin (9600);
  analogReference(INTERNAL);
  
  //Define the pins as INPUT or OUTPUT and set them to LOW or HIGH
  pinMode(zero_button, INPUT);
  pinMode(mode_button, INPUT);
  pinMode(plus_button, INPUT);
  pinMode(sound_button, INPUT);
  pinMode(rot_data, INPUT_PULLUP);
  //pinMode(rot_clock, INPUT_PULLUP);
  pinMode(battery_in, INPUT);
  pinMode(sharp_in, INPUT);
  pinMode(IR_rpm_in, INPUT);
  pinMode(laser_out, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(laser_out,LOW);
  digitalWrite(buzzer,LOW);

  PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  //PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.

  //OLED display settings...
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32 or 64 from eBay)
  delay(100);
  display.clearDisplay();
  display.drawBitmap(0, 8,  LOGO, 128, 31, 1); 
  display.display();
  delay(1000);
  display.clearDisplay();
  display.display();


  //VL53L0X sensor
  sensor.init();
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(200000);

  //MPU6050 gyro/acc
  Wire.begin();                           //begin the wire comunication  
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);   
  time = millis();
      

}

void loop() {
  analogReference(INTERNAL);
  battery_level = map(analogRead(battery_in),0,1024, 0, 4730);
  battery_level = battery_level / 1000;


  /////////////////////////////////CHANGE THE MODE/////////////////////////////////  
  if(digitalRead(mode_button) && !mode_button_state)
  {   
    data_state = digitalRead(rot_data);
    mode_button_state = true;
    Z_error = false;
    Z_loop = 0;    
    IR_enable = false;
    Gyr_rawZ_error = 0;
    print_time = 0; //Reset print time for angle mode (for MODE 3 and MODE 4)
    mode = mode + 1;
    if(sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(15);
      digitalWrite(buzzer, LOW);
    }
    if(mode > 5)
    { 
      mode = 0; 
    }
  }
  if(!digitalRead(mode_button) && mode_button_state)
  {
    mode_button_state = false;    
  }
  /////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////ZERO the values/////////////////////////////////  
  if(digitalRead(zero_button) && !zero_button_state)
  {
    counter = 0;
    Z_error = false;
    Total_angle_z = 0;
    zero_button_state = true;
    if(sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(15);
      digitalWrite(buzzer, LOW);
    }
  }
  if(!digitalRead(zero_button) && zero_button_state)
  {
    zero_button_state = false;    
  }
  /////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////SOUND ON OFF////////////////////////////////////  
  if(digitalRead(sound_button) && !sound_button_state)
  {
    sound = !sound;
    sound_button_state = true;
    if(sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(15);
      digitalWrite(buzzer, LOW);
    }
  }
  if(!digitalRead(sound_button) && sound_button_state)
  {
    sound_button_state = false;    
  }
  /////////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////PLUS BUTTON////////////////////////////////////  
  if(digitalRead(plus_button) && !plus_button_state)
  {
    laser = !laser;
    digitalWrite(laser_out, laser);
    plus_button_state = true;
    if(sound)
    {
      digitalWrite(buzzer, HIGH);
      delay(15);
      digitalWrite(buzzer, LOW);
    }
  }
  if(!digitalRead(plus_button) && plus_button_state)
  {
    plus_button_state = false;    
  }
  /////////////////////////////////////////////////////////////////////////////////




















  



  ///////////////////////////////////////MODE 0////////////////////////////////////  
  if(mode == 0)
  {
    //analogReference(DEFAULT);

    float Sharp_read = analogRead(sharp_in); // lectura del sensor 0
    float Sharp_cm = pow(4727.4 / Sharp_read, 1.2134); // conversion to cm
    int Sharp_cm_int = Sharp_cm;
    float Sharp_inch = Sharp_cm * 0.393701;
    if (Sharp_cm > 10 && Sharp_cm < 80) 
    { 
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE); 
      display.setCursor(0,0); 
      display.print("1-Distance"); 
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V"); 
      display.setTextSize(2); 
      display.setCursor(0,30);    
      display.print("cm: "); 
      display.print(Sharp_cm,0); 
      display.setCursor(0,50);    
      display.print("in: "); 
      display.print(Sharp_inch,0);
      display.display();
      if (Sharp_cm_int != Sharp_cm_previous)
      {
        if(sound)
        {
          digitalWrite(buzzer, HIGH);
          delay(15);
          digitalWrite(buzzer, LOW);
          delay(30);
        }
      }
      else
      {
        delay(45);
      }
      Sharp_cm_previous = Sharp_cm_int;
    }
    if (Sharp_cm < 10)
    {
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE); 
      display.setCursor(0,0); 
      display.print("1-Distance"); 
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V"); 
      display.setTextSize(2);       
      display.setCursor(0,10);    
      display.print("Too");      
      display.setCursor(0,30);    
      display.print("CLOSE");      
      display.display();
    }    
    if (Sharp_cm > 80)
    {
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE); 
      display.setCursor(0,0); 
      display.print("1-Distance"); 
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V"); 
      display.setTextSize(2);       
      display.setCursor(0,10);    
      display.print("OUT");      
      display.setCursor(0,30);    
      display.print("OF");
      display.setCursor(0,50);    
      display.print("RANGE");
      display.display();
    }    
  }
 
  /////////////////////////////////////////////////////////////////////////////////

  
  
  ///////////////////////////////////////MODE 1////////////////////////////////////  
  if(mode == 1)
  {        
    float VL53L0X_mm = sensor.readRangeSingleMillimeters() - 40; //40mm is the error
    float VL53L0X_cm = VL53L0X_mm / 10;
    float VL53L0X_inch = VL53L0X_cm * 0.393701;
    int VL53L0X_mm_int = VL53L0X_mm;

    if (VL53L0X_mm < 300) 
    {        
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE);       
      display.setCursor(0,0); 
      display.print("2-Distance");
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V"); 
      display.setTextSize(2);       
      display.setCursor(0,10);    
      display.print("mm: "); 
      display.print(VL53L0X_mm,0);
      display.setCursor(0,30);    
      display.print("cm: "); 
      display.print(VL53L0X_cm,1); 
      display.setCursor(0,50);    
      display.print("in: "); 
      display.print(VL53L0X_inch,1);
      display.display();
      if (VL53L0X_mm_int != VL53L0X_mm_previous)
      {
        if(sound)
        {
          digitalWrite(buzzer, HIGH);
          delay(15);
          digitalWrite(buzzer, LOW);
          delay(30);
        }
      }
      else
      {
        delay(45);
      }
      VL53L0X_mm_previous = VL53L0X_mm_int;
    } 
    else 
    {
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE); 
      display.setCursor(0,0); 
      display.print("2-Distance"); 
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V");
      display.setTextSize(2);     
      display.setCursor(0,10);    
      display.print("OUT");      
      display.setCursor(0,30);    
      display.print("OF");
      display.setCursor(0,50);    
      display.print("RANGE");
      display.display();
    }    
  }
  /////////////////////////////////////////////////////////////////////////////////





  ///////////////////////////////////////MODE 2////////////////////////////////////  
  if(mode == 2)
  {   
    float rotary_mm = (counter * (2.5* 3.1415 / 30)) * 10; //2.5 is the diameter of the wheel in cm, 30 is the steps / rotation of the encoder
    float rotary_cm = rotary_mm / 10;
    float rotary_inch = rotary_cm * 0.393701;
    int   rotary_mm_int = rotary_mm;

    display.clearDisplay();    
    display.setTextSize(1);   
    display.setTextColor(BLACK,WHITE);       
    display.setCursor(0,0); 
    display.print("3-Length");
    display.setTextColor(WHITE); 
    display.setCursor(95,0); 
    display.print(battery_level); 
    display.print("V"); 
    display.setTextSize(2);    
    display.setCursor(0,10);    
    display.print("mm: "); 
    display.print(rotary_mm,0);
    display.setCursor(0,30);    
    display.print("cm: "); 
    display.print(rotary_cm,1); 
    display.setCursor(0,50);    
    display.print("in: "); 
    display.print(rotary_inch,1);
    display.display();    
  } 
  /////////////////////////////////////////////////////////////////////////////////










  ///////////////////////////////////////MODE 3////////////////////////////////////  
  if(mode == 3)
  {  
    print_time = print_time + 1;
    timePrev = time;                        // the previous time is stored before the actual time read
    time = millis();                        // actual time read
    elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds
  
    //////////////////////////////////////Gyro read/////////////////////////////////////  
    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers
        
    Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
    Gyr_rawY=Wire.read()<<8|Wire.read();
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
    /*---X---*/
    Gyr_rawX = (Gyr_rawX/32.8); 
    /*---Y---*/
    Gyr_rawY = (Gyr_rawY/32.8);
    
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
    /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
    /*---X---*/
    Gyro_angle_y = Gyr_rawY*elapsedTime;  
      
    //////////////////////////////////////Acc read/////////////////////////////////////
    Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68) 
    Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);      //keep the transmission and next
    Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B  
    /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.
    If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/    
    Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
    Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
    Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ; 
    /*Now in order to obtain the Acc angles we use euler formula with acceleration values
    after that we substract the error value found before*/  
    /*---X---*/
    Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg);
    /*---Y---*/
    Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg);    
    //////////////////////////////////////Total angle and filter/////////////////////////////////////
    /*---X axis angle---*/
    Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
    /*---Y axis angle---*/
    Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;

    float x_without_error = (Total_angle_x - angle_x_error)*-1;
    float y_without_error = (Total_angle_y - angle_y_error);

    if(print_time > 150)
    {
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE);       
      display.setCursor(0,0); 
      display.print("4-Level"); 
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V");
      display.setTextSize(2);       
      display.setCursor(0,25);    
      display.print("X: "); 
      display.print(y_without_error,1); //I print Y values for X because of how the IMU is palced inside the case
      display.setCursor(0,45);    
      display.print("Y: "); 
      display.print(x_without_error,1);    //I print X values for Y because of how the IMU is palced inside the case
      display.display();   
  
      if(  (-1 < x_without_error &&  x_without_error < 1) && (-1 < y_without_error && y_without_error < 1) && sound)
      {
        digitalWrite(buzzer,HIGH);      
      }
      else
      {
        digitalWrite(buzzer,LOW); 
      } 
    }
  } 
  /////////////////////////////////////////////////////////////////////////////////





  ///////////////////////////////////////MODE 4////////////////////////////////////  
  if(mode == 4)
  {
    print_time = print_time + 1;
    timePrev = time;                        // the previous time is stored before the actual time read
    time = millis();                        // actual time read
    elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds
    
    //////////////////////////////////////Gyro read///////////////////////////////////// 
    if(!Z_error)
    { 
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE);       
      display.setCursor(0,0); 
      display.print("5-Angle");       
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V");   
      display.setTextSize(2);     
      display.setCursor(20,30); 
      display.print("WAIT..."); 
      display.display();
      while(Z_loop < 20)
      {        
        timePrev = time;                        // the previous time is stored before the actual time read
        time = millis();                        // actual time read
        elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds
        Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
        Wire.write(0x47);                        //First adress of the Z Gyro data
        Wire.endTransmission(false);    
        Wire.requestFrom(0x68,2,true);           //We ask for just 2 registers  
        Gyr_rawZ=Wire.read()<<8|Wire.read();
        Gyr_rawZ = (Gyr_rawZ/32.8);
        Gyro_angle_z = Gyr_rawZ*elapsedTime;          
        Gyr_rawZ_error = Gyr_rawZ_error + Gyro_angle_z;
        Z_loop = Z_loop + 1;
        delay(50);
      }
      Gyr_rawZ_error = Gyr_rawZ_error / 20;
      
      Z_error = true;
    }
    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
    Wire.write(0x47);                        //First adress of the Z Gyro data
    Wire.endTransmission(false);    
    Wire.requestFrom(0x68,2,true);           //We ask for just 2 registers  
    Gyr_rawZ=Wire.read()<<8|Wire.read();

    
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
    /*---Z---*/
    Gyr_rawZ = (Gyr_rawZ/32.8);
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
    Gyro_angle_z = Gyr_rawZ*elapsedTime;
    Gyro_angle_z = Gyro_angle_z - Gyr_rawZ_error;   
    Total_angle_z = Total_angle_z + Gyro_angle_z;
   

    if(print_time > 150)
    {
      display.clearDisplay();    
      display.setTextSize(1);   
      display.setTextColor(BLACK,WHITE);       
      display.setCursor(0,0); 
      display.print("5-Angle"); 
      display.setTextColor(WHITE); 
      display.setCursor(95,0); 
      display.print(battery_level); 
      display.print("V");
      display.setTextSize(2);        
      display.setCursor(0,30);    
      display.print("X: "); 
      display.print(Total_angle_z,2); //I print Y values for X because of how the IMU is palced inside the case
      display.display();       
    }    
  } 
  /////////////////////////////////////////////////////////////////////////////////





  ///////////////////////////////////////MODE 5////////////////////////////////////  
  if(mode == 5)
  {
    IR_enable = true;
    int rpm = 60000000 / width;   //values are in in us
    display.clearDisplay();    
    display.setTextSize(1);   
    display.setTextColor(BLACK,WHITE);       
    display.setCursor(0,0); 
    display.print("6-RPM"); 
    display.setTextColor(WHITE); 
    display.setCursor(95,0); 
    display.print(battery_level); 
    display.print("V");
    display.setTextSize(2);     
    display.setCursor(0,20);    
    display.print("   -RPM-"); 
    display.setCursor(0,37); 
    display.print(rpm); //I print Y values for X because of how the IMU is palced inside the case
    display.display();      
  } 
  /////////////////////////////////////////////////////////////////////////////////












  
}//end of void loop











ISR(PCINT0_vect){

  if( (PINB & B00000001) && !data_state )
  {
    counter ++;
    data_state = true; 
    delay(2);
  }
  if( !(PINB & B00000001) && data_state )
  {
    counter ++;
    data_state = false; 
    delay(2);
  }

  if(IR_enable)
  {
    current_count = micros();
    ///////////////////////////////////////Channel 1
    if(PINB & B00000100)
    {                              
      if(last_IR_state == 0){                         
        last_IR_state = 1; 
        width = current_count - counter_1;                            
        counter_1 = current_count;                    
      }
    }
    else if(last_IR_state == 1)
    {                           
      last_IR_state = 0;      
    }
  }

  


  
}//end if ISR
