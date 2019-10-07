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

uint8_t to_encoder[LENGTH_OF_BUFFER+100];
uint8_t imu_buffer[18];
uint8_t imu2_buffer[18];
uint8_t imu3_buffer[18];
uint8_t GPS_buffer[66];
uint8_t tmp, prev_tmp, GPS_data[66];
unsigned long counter=0, counter_imugps=0, counter_2=0, gps_time=0, show_time;
unsigned int counter_frame=0;
bool lock=false, update_imu=true;
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
    imu.begin();
    accel.begin();
    gyro.begin();
    imu3.initialize(ACCEL_RANGE_30G, GYRO_RANGE_4000DPS);
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

    Serial1.begin(115200);

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
    
    while(Serial1.available()){
        if(counter_imugps){
#if DEBUG
            Serial.print("counter-thingy: ");
            Serial.println(counter_imugps);
#endif
            counter_imugps = 0;
        }

        prev_tmp = tmp;
        tmp = Serial1.read();
        if(lock){
            counter++;

            if(counter == 66){
                counter = 0;
                update_GPS_data();
            }

            GPS_data[counter] = tmp;

            if((counter == 0) && (tmp != 0xA0)){
                lock = false;
#if DEBUG
                Serial.print("lost lock 0: 0x");
                Serial.println(tmp, HEX);
#endif
            }

            if((counter == 1) && (tmp != 0xA1)){
                lock = false;
#if DEBUG
                Serial.print("Lost lock 1: 0x");
                Serial.println(tmp, HEX);
#endif
            }
        }
        else{
            if((prev_tmp == 0xA0)&&(tmp == 0xA1)){
                lock = true;
                counter = 1;
                Serial.println("Got lock!");
            }
        }
    }
    if(millis()-show_time > 100){
        show_time = millis();
      // this condition is satisfied when i tested with serial print here
      
#if DEBUG
//        update_imu=true;
//        Serial.print(show_time);
//        Serial.print("\t");
//        Serial.print(counter_2);
//        Serial.print("\t");
//        Serial.print(Serial1.available());
//        Serial.println();
#endif
    }
}

void irq_encoder(void){
    Serial.print("HER");
    int i;
    counter_frame++;
    if(counter_frame >= LENGTH_OF_BUFFER){
        counter_frame = 0;
        update_imu = true;
    }

    GPIOC_PDOR = (uint32_t)((GPIOC_PDOR&0xFFFFFF00) | to_encoder[counter_frame]);// Output next byte to encoder
    
    if(counter_frame == 0){
        for(i = 0; i < 18; i++){
            to_encoder[2+i] = GPS_buffer[i];
        }
    }

    if(counter_frame == 19){
        for(i = 0; i < 18; i++){
            to_encoder[3+counter_frame+i] = imu_buffer[i];
        }
    }

    if(counter_frame == 39){
        for(i = 0; i < 18; i++){
            to_encoder[3+counter_frame+i] = imu2_buffer[i];
        }
        Serial.println("BMI");
        Serial.print(to_encoder[41]);
        Serial.print(",");
        Serial.print(to_encoder[42]);
        Serial.print(",");        
        Serial.print(to_encoder[43]);
        Serial.print(",");        
        Serial.print(to_encoder[44]);
        Serial.print(",");        
        Serial.print(to_encoder[45]);
        Serial.print(",");        
        Serial.print(to_encoder[46]);
        Serial.print(",");        
        Serial.print(to_encoder[47]);
        Serial.print(",");        
        Serial.print(to_encoder[48]);
        Serial.print(",");        
        Serial.print(to_encoder[49]);
        Serial.print(",");        
        Serial.print(to_encoder[50]);
        Serial.print(",");        
        Serial.print(to_encoder[51]);
        Serial.print(",");        
        Serial.print(to_encoder[52]);
        Serial.println(",");        

    }

    if(counter_frame == 59){
        for(i = 0; i < 18; i++){
            to_encoder[3+counter_frame+i] = imu3_buffer[i];
        }
    }
    update_imu = true;
}

void update_GPS_data(){
    GPS_buffer[0] = ((GPS_data[3+2]&0x01 << 7) | (GPS_data[3+3]&0x7f));
    if(millis()-gps_time > 200){
        gps_time = millis();
        GPS_buffer[0] |= 64;
    }
    // Latitude
    GPS_buffer[1] = GPS_data[3+10];
    GPS_buffer[2] = GPS_data[3+11];
    GPS_buffer[3] = GPS_data[3+12];
    GPS_buffer[4] = GPS_data[3+13];

    // Longitude
    GPS_buffer[5] = GPS_data[3+14];
    GPS_buffer[6] = GPS_data[3+15];
    GPS_buffer[7] = GPS_data[3+16];
    GPS_buffer[8] = GPS_data[3+17];

    // Altitude (if GPS_data[3+22]&0x80, then the unsigned integer is indeed negative
    GPS_buffer[9] = ((GPS_data[3+22]&0x80) | (GPS_data[3+23]&0x7F)); 
    GPS_buffer[10] = GPS_data[3+24];
    GPS_buffer[11] = GPS_data[3+25];

    int32_t EXEF_X_Vel_SINT = (GPS_data[3+48] << 24) + (GPS_data[3+49] << 16) + (GPS_data[3+50] << 8) + (GPS_data[3+51] << 0);

    int32_t EXEF_Y_Vel_SINT = (GPS_data[3+52] << 24) + (GPS_data[3+53] << 16) + (GPS_data[3+54] << 8) + (GPS_data[3+55] << 0);

    int32_t EXEF_Z_Vel_SINT = (GPS_data[3+56] << 24) + (GPS_data[3+57] << 16) + (GPS_data[3+58] << 8) + (GPS_data[3+59] << 0);

    uint32_t Total_Vel = sqrt(EXEF_X_Vel_SINT*EXEF_X_Vel_SINT + EXEF_Y_Vel_SINT*EXEF_Y_Vel_SINT + EXEF_Z_Vel_SINT*EXEF_Z_Vel_SINT);

    GPS_buffer[12] = (Total_Vel&0xFF0000) >> 16; // discard higher values, as they shouldn't exist
    GPS_buffer[13] = (Total_Vel&0xFF00) >> 8;
    GPS_buffer[14] = Total_Vel&0xFF; // LSB
    
    
}
