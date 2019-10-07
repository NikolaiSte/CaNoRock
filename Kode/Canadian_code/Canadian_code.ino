//This code will function to read out data from the Invensense, LSM9DS1, & BMI088

#include <Arduino.h>
#include <ICM20649.h>
#include "BMI088.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
//INVENSENSE*************
ICM20649 imu2;
int ledPin = 13;
//**********************

//BMI088****************
Bmi088Accel accel(Wire,0x18);
Bmi088Gyro gyro(Wire,0x68);
//***********************

//LSM9DS1***********************
LSM9DS1 imu;
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW#define PRINT_CALCULATED, the 0x prefix implies that the hex value is constant
#define PRINT_CALCULATED
//*********************************
void setupPorts(){
    pinMode(ledPin,OUTPUT); // triggers a blinking LED light on the TEENCY 
}

void setup() {
  // set the baud rate to 9600:
  Serial.begin(9600);

  //INVENSENSE intialization ***************************************
    setupPorts();
    if(!imu2.initialize(ACCEL_RANGE_30G, GYRO_RANGE_4000DPS)) //set range to 30 g and 4000dps
    {
        Serial.println("Connection to INVENSENSE IMU can not be established. Check wiring."); //error message
    }; 
   //INVENSENSE****************************************


  //BMI088**************************
  int status;
  float elapsed_time=0;
  

  
  while(!Serial) {}
  // turn on the sensors
  status = accel.begin();
  if (status < 0) {
    Serial.println("ERROR WITH BMI088 ACCELEROMETER");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.begin();
  if (status < 0) {
    Serial.println("ERROR WITH GYRO");
    Serial.println(status);
    while (1) {}
  }


status = accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ); //the odr and bandwidth can be changed to other values
status = gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ); // the settings for the gyro odr and BW
status = accel.setRange(Bmi088Accel::RANGE_24G); // set BMI to 24 g
status = gyro.setRange(Bmi088Gyro::RANGE_2000DPS); //set BMI to 2000 dps

  //********************************

  //LSM9DS1**************************
  
  //Now set the IMU's communication mode to I2C
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.settings.accel.scale = 16; // Set LSM accel range to +/-16g
  imu.settings.gyro.scale = 2000; // Set LSM gyro range to +/-2000dps
  imu.settings.mag.scale = 16; // Set LSM mag range to +/-16Gs
  imu.begin(); // Call begin to update the sensor's new settings

  //set the sample rates to 50 Hz
  imu.settings.accel.sampleRate = 2;//accel sample rate of 50 Hz
  imu.settings.gyro.sampleRate = 2; // 59.5Hz ODR for gyro
  imu.settings.mag.sampleRate = 6; //40 Hz ODR for magnetometer

  //Now check if the LSM IMU is powered up and working
  //output a single error message if something is wrong
    if (!imu.begin()) //checks if the begin command is not functioning
    {
    Serial.println("ERROR WITH LSM9DS1");
    while (1) //this prevents an infinite number of error messages
   ; 
    
    }
  //***********************************
  //state the order of data print out
  //these are the column headings in the CSV file
Serial.print("Time");
Serial.print(","); //everything is separated by a comma for the CSV format
Serial.print("BMIax");
Serial.print(",");
Serial.print("BMIay");
Serial.print(",");
Serial.print("BMIaz");
Serial.print(",");
Serial.print("BMIgx");
Serial.print(",");
Serial.print("BMIgy");
Serial.print(",");
Serial.print("BMIgz");
Serial.print(",");
Serial.println("BMItemp");
}

void loop() {
  //now read out the IMU data

  //LSM9DS1****************************
  if (imu.accelAvailable())
    {
    imu.readAccel();
    }

  if (imu.magAvailable())
    {
      imu.readMag();
    }
  if (imu.gyroAvailable())
    {
    imu.readGyro();
    }

    //********************************

  //BMI088****************************
   /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  //**********************************


  //INVENSENSE*************************
  digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);

    imu2.readAcceleration();
    imu2.readGyro();
    imu2.readTemperature();
    //*********************************

    
    //Print THE DATA TO SERIAL
    double time_ran = millis();   //this is the elapsed time in milliseconds
    Serial.print(time_ran);  //this is the elapsed time in milliseconds
    Serial.print(", ");
    //INVENSENSE SERIAL DATA*******************
    //***************************************

    //BMI088 Values*************************
    uint8_t imu2_buffer[18];
    const float G = 9.807f;
    float accel_range_mss = 24.0f*G;
    const float D2R = M_PI / 180.0f;
    float gyro_range_rads = 2000.0f*D2R;
        float accelX = ((accel.getAccelX_mss()*32768.0f)/accel_range_mss);
        float accelY = ((accel.getAccelY_mss()*32768.0f)/accel_range_mss);
        float accelZ = ((accel.getAccelZ_mss()*32768.0f)/accel_range_mss);

        float gyroX = ((gyro.getGyroX_rads()*32768.0f)/gyro_range_rads);
        float gyroY = ((gyro.getGyroY_rads()*32768.0f)/gyro_range_rads);
        float gyroZ = ((gyro.getGyroZ_rads()*32768.0f)/gyro_range_rads);

        //Accelerometer
        imu2_buffer[0] = ((uint8_t)accelX&0xFF00)>>8;
        imu2_buffer[1] = ((uint8_t)accelX)&0xFF;
        imu2_buffer[2] = ((uint8_t)accelY&0xFF00)>>8;
        imu2_buffer[3] = ((uint8_t)accelY)&0xFF;
        imu2_buffer[4] = ((uint8_t)accelZ&0xFF00)>>8;
        imu2_buffer[5] = ((uint8_t)accelZ)&0xFF;

        //Gyro
        imu2_buffer[6] = ((uint8_t)gyroX&0xFF00)>>8;
        imu2_buffer[7] = ((uint8_t)gyroX)&0xFF;
        imu2_buffer[8] = ((uint8_t)gyroY&0xFF00)>>8;
        imu2_buffer[9] = ((uint8_t)gyroY)&0xFF;
        imu2_buffer[10] = ((uint8_t)gyroZ&0xFF00)>>8;
        imu2_buffer[11] = ((uint8_t)gyroZ)&0xFF;
    
  Serial.print(imu2_buffer[0]);
  Serial.print(", ");
  Serial.print(imu2_buffer[1]);
  Serial.print(", ");
  Serial.print(imu2_buffer[2]);
  Serial.print(", ");
  Serial.print(imu2_buffer[3]);
  Serial.print(", ");
  Serial.print(imu2_buffer[4]);
  Serial.print(", ");
  Serial.print(imu2_buffer[5]);
  Serial.print(", ");
  Serial.print(imu2_buffer[6]);
  Serial.print(", ");
  Serial.print(imu2_buffer[7]);
  Serial.print(", ");
  Serial.print(imu2_buffer[8]);
  Serial.print(", ");
  Serial.print(imu2_buffer[9]);
  Serial.print(", ");
  Serial.print(imu2_buffer[10]);
  Serial.print(", ");
  Serial.print(imu2_buffer[11]);
  Serial.print(", ");
  Serial.println(accel.getTemperature_C());
  //***********************************************


  //LSM9DS1 Values*********************************
//************************************

}
