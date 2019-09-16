//All packages was installed with the Arduino package installer
#include <Arduino.h>
#include <ICM20649.h>
#include "BMI088.h"
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define LENGTH_OF_BUFFER  (5*(2+18))
#define SYNC_WORD_0     0xA0
#define SYNC_WORD_1     0xA1
#define DEBUG 1

unsigned long counter_2 = 0;
uint8_t to_encoder[LENGTH_OF_BUFFER+100];
uint8_t imu_buffer[18];
uint8_t imu2_buffer[18];
uint8_t imu3_buffer[18];
unsigned int counter_frame=0;
bool update_imu=true;
const float G = 9.807f;
float accel_range_mss = 24.0f*G;
const float D2R = M_PI / 180.0f;
float gyro_range_rads = 2000.0f*D2R;


//LSM9DS1
LSM9DS1 imu;

//BMI088
Bmi088Accel accel(Wire, 0x18);
Bmi088Gyro gyro(Wire, 0x68);

//ICM20649
ICM20649 imu3;

void setup(){
#if DEBUG
    Serial.begin(9600);
    //while(!Serial);
#endif

    //LSM9DS1
    imu.settings.device.commInterface = IMU_MODE_I2C;
    imu.settings.device.mAddress = LSM9DS1_M;
    imu.settings.device.agAddress = LSM9DS1_AG;
    imu.setAccelScale(16); // acc scale: 0.000732
    imu.setGyroScale(2000); // gyro scale: 0.07
    // mag is 4 gauss by default. mag scale: 0.00014 (gauss)

    //BMI088
    accel.setOdr(Bmi088Accel::ODR_100HZ_BW_19HZ); //the odr and bandwidth can be changed to other values
    gyro.setOdr(Bmi088Gyro::ODR_100HZ_BW_12HZ); // the settings for the gyro odr and BW
    accel.setRange(Bmi088Accel::RANGE_24G); // set BMI to 24 g
    gyro.setRange(Bmi088Gyro::RANGE_2000DPS); //set BMI to 2000 dps

    //ICM20649


#if DEBUG
  Serial.println("Starting...");
#endif

#if DEBUG
    if(!imu.begin()){
        Serial.println("Failed to communicate with LSM9DS1. Double-check wiring.");
        while(1);
    }
    int status;
    while(!Serial){
        status = accel.begin();
        if(status < 0){
            Serial.println("Failed to communicate with BMI088 accelerometer.");
            Serial.println(status);
            while(1){}
        }
        status = gyro.begin();
        if(status < 0){
            Serial.println("Failed to communicate with BMI088 gyroscope.");
            Serial.println(status);
            while(1){}
        }
    }

    if(!imu3.initialize(ACCEL_RANGE_30G, GYRO_RANGE_4000DPS)){
        Serial.println("Connection to ICM20649 could not be established. Check wiring.");
    }
#endif

    // Set sync words. Must use auto-corr to find proper ones, not 0xEB90
    to_encoder[0*(2+18)] = SYNC_WORD_0;
    to_encoder[0*(2+18)+1] = SYNC_WORD_1;
    to_encoder[1*(2+18)] = SYNC_WORD_0;
    to_encoder[1*(2+18)+1] = 0x01;
    to_encoder[2*(2+18)] = SYNC_WORD_0;
    to_encoder[2*(2+18)+1] = 0x02;
    to_encoder[3*(2+18)] = SYNC_WORD_0;
    to_encoder[3*(2+18)+1] = 0x03;
    to_encoder[4*(2+18)] = SYNC_WORD_0;
    to_encoder[4*(2+18)+1] = 0x04;

    //Serial.begin(115200);

    //GPIOC_PDDR |= (uint32_t)0xFF; // Set all lower 8 pins to output (to encoder)
    pinMode(15, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);

    pinMode(2, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(2), irq_encoder, RISING);

#if DEBUG
    Serial.println("Done with startup");
#endif
}

void loop(){
    if(update_imu){
        update_imu = false;
        //LSM9DS1
        if (imu.accelAvailable()){
            imu.readAccel();
        }
        if(imu.magAvailable()){
            imu.readMag();
        }
        if(imu.gyroAvailable()){
            imu.readGyro();
        }

        //Accelerometer
        imu_buffer[0] = (imu.ax&0xFF00)>>8;
        imu_buffer[1] = imu.ax&0xFF;
        imu_buffer[2] = (imu.ay&0xFF00)>>8;
        imu_buffer[3] = imu.ay&0xFF;
        imu_buffer[4] = (imu.az&0xFF00)>>8;
        imu_buffer[5] = imu.az&0xFF;

        //Gyro
        imu_buffer[6] = (imu.gx&0xFF00)>>8;
        imu_buffer[7] = imu.gx&0xFF;
        imu_buffer[8] = (imu.gy&0xFF00)>>8;
        imu_buffer[9] = imu.gy&0xFF;
        imu_buffer[10] = (imu.gz&0xFF00)>>8;
        imu_buffer[11] = imu.gz&0xFF;

        //Magnetometer
        imu_buffer[12] = (imu.mx&0xFF00)>>8;
        imu_buffer[13] = imu.mx&0xFF;
        imu_buffer[14] = (imu.my&0xFF00)>>8;
        imu_buffer[15] = imu.my&0xFF;
        imu_buffer[16] = (imu.mz&0xFF00)>>8;
        imu_buffer[17] = imu.mz&0xFF;

        //BMI088
        accel.readSensor();
        gyro.readSensor();
        float accelX = ((accel.getAccelX_mss()*32768.0f)/accel_range_mss);
        float accelY = ((accel.getAccelY_mss()*32768.0f)/accel_range_mss);
        float accelZ = ((accel.getAccelZ_mss()*32768.0f)/accel_range_mss);

        float gyroX = ((gyro.getGyroX_rads()*32768.0f)/gyro_range_rads);
        float gyroY = ((gyro.getGyroY_rads()*32768.0f)/gyro_range_rads);
        float gyroZ = ((gyro.getGyroZ_rads()*32768.0f)/gyro_range_rads);
        
        //Accelerometer
        imu2_buffer[0] = ((int)accelX&0xFF00)>>8;
        imu2_buffer[1] = ((int)accelX)&0xFF;
        imu2_buffer[2] = ((int)accelY&0xFF00)>>8;
        imu2_buffer[3] = ((int)accelY)&0xFF;
        imu2_buffer[4] = ((int)accelZ&0xFF00)>>8;
        imu2_buffer[5] = ((int)accelZ)&0xFF;

        //Gyro
        imu2_buffer[6] = ((int)gyroX&0xFF00)>>8;
        imu2_buffer[7] = ((int)gyroX)&0xFF;
        imu2_buffer[8] = ((int)gyroY&0xFF00)>>8;
        imu2_buffer[9] = ((int)gyroY)&0xFF;
        imu2_buffer[10] = ((int)gyroZ&0xFF00)>>8;
        imu2_buffer[11] = ((int)gyroZ)&0xFF;

        //non-exsistent magnetometer
        imu2_buffer[12] = 0;
        imu2_buffer[13] = 0;
        imu2_buffer[14] = 0;
        imu2_buffer[15] = 0;
        imu2_buffer[16] = 0;
        imu3_buffer[17] = 0;

        //ICM20649
        imu3.readAcceleration();
        imu3.readGyro();
        imu3.readTemperature(); 

        //Accelerometer
        imu3_buffer[0] = (imu3.accelRaw.x&0xFF00)>>8;
        imu3_buffer[1] = (imu3.accelRaw.x)&0xFF;
        imu3_buffer[2] = (imu3.accelRaw.y&0xFF00)>>8;
        imu3_buffer[3] = (imu3.accelRaw.y)&0xFF;
        imu3_buffer[4] = (imu3.accelRaw.z&0xFF00)>>8;
        imu3_buffer[5] = (imu3.accelRaw.z)&0xFF;

        //Gyro
        imu3_buffer[6] = (imu3.gyroRaw.x&0xFF00)>>8;
        imu3_buffer[7] = (imu3.gyroRaw.x)&0xFF;
        imu3_buffer[8] = (imu3.gyroRaw.y&0xFF00)>>8;
        imu3_buffer[9] = (imu3.gyroRaw.y)&0xFF;
        imu3_buffer[10] = (imu3.gyroRaw.z&0xFF00)>>8;
        imu3_buffer[11] = (imu3.gyroRaw.z)&0xFF;

        //non-exsistent magnetometer
        imu3_buffer[12] = 0;
        imu3_buffer[13] = 0;
        imu3_buffer[14] = 0;
        imu3_buffer[15] = 0;
        imu3_buffer[16] = 0;
        imu3_buffer[17] = 0;
    
    }
}

void irq_encoder(void){
    int i;
    counter_frame++;
    if(counter_frame >= LENGTH_OF_BUFFER){
        counter_frame = 0;
        update_imu = true;
    }

    GPIOC_PDOR = (uint32_t)((GPIOC_PDOR&0xFFFFFF00) | to_encoder[counter_frame]);
    
    if(counter_frame%20 == 0){
        for(i = 1; i < 18; i++){
            to_encoder[2+counter_frame+i] = imu_buffer[i];
        }
    }

    if(counter_frame%40 == 0){
        for(i = 1; i < 18; i++){
            to_encoder[2+counter_frame+i] = imu2_buffer[i];
        }
    }

    if(counter_frame%60 == 0){
        for(i = 1; i < 18; i++){
            to_encoder[2+counter_frame+i] = imu3_buffer[i];
        }
        //update_imu = true;
    }
    if(counter_frame%80 == 0){
      for(i = 1; i <18; i++){
        to_encoder[2+counter_frame+i] = 0;
      }
    }
    
    Serial.print("counter_frame: ");
    Serial.println(counter_frame);
    Serial.print("to_encoder[counter_frame]: ");
    Serial.println(to_encoder[counter_frame]);
    
}
