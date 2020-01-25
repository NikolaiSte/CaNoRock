/**
 * @Author: Ketil Røed <ketilroe>
 * @Date:   2019-04-19T15:02:44+02:00
 * @Filename: ICM20649.cpp
 * @Last modified by:   ketilroe
 * @Last modified time: 2019-04-19T16:15:47+02:00
 * @Copyright: Ketil Røed
 */



/*
  ICM20649.cpp - Library InvenSense 6-axis gyro and accelerometer (ICM20649)
  Created by Ketil Røed, April, 2019.
  Released into the public domain.

  Inspired by https://github.com/drcpattison/DPEng_ICM20948_AK09916
*/

#include "Arduino.h"
#include <Wire.h>
#include "ICM20649.h"

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/



ICM20649::ICM20649(void)
{
  Wire.begin();
}





bool ICM20649::initialize(icm20649AccelRange_t rangeAccel, icm20649GyroRange_t rangeGyro){
    //switch to user bank 0

    deviceID = readByte(ICM20649_ADDR,WHO_AM_I_ICM20649);

    if (deviceID != ICM20649_ID)
    {
      return false;
    }

    writeByte(ICM20649_ADDR, REG_BANK_SEL, 0x00);
    writeByte(ICM20649_ADDR,PWR_MGMT_1, 0x01);

    //switch to user bank 2
    writeByte(ICM20649_ADDR,REG_BANK_SEL, 0x20);
    accelConfig = readByte(ICM20649_ADDR,ACCEL_CONFIG_1);

    _rangeAccel = rangeAccel;
    _rangeGyro = rangeGyro;

    setAccelConfig = accelConfig;
    // reset range values
    setAccelConfig &= ~0x6;


    switch(rangeAccel) {
      case (ACCEL_RANGE_4G):
        setAccelConfig = setAccelConfig | (0b00 <<1);
      break;
      case (ACCEL_RANGE_8G):
        setAccelConfig = setAccelConfig | (0b01 <<1);
      break;
      case (ACCEL_RANGE_16G):
        setAccelConfig = setAccelConfig | (0b10 <<1);
      break;
      case (ACCEL_RANGE_30G):
        setAccelConfig = setAccelConfig | (0b11 <<1);
      break;
    }

    /* Configure the accelerometer */
    /* Set ACCEL_CONFIG (0x14)
     ====================================================================
     BIT  Symbol    		Description                                   Default
     ---  ------    		--------------------------------------------- -------
     7:6  -	 		Reserved                                   		  -
     5:3  ACCEL_DLFPFCFG[2:0] 	Accelerometer low pass filter                     000
  		  		000 = 246 Hz / 1209 Hz if FCHOICE is 0
                    		001 = 246 Hz
                    		010 = 111.4 Hz
                    		011 = 50.4 Hz
                    		100 = 23.9 Hz
                    		101 = 11.5 Hz
                    		110 = 5.7 Hz
                    		111 = 473 Hz
     2:1  ACCEL_FS_SEL[1:0]   	Accelerometer Full Scale Select                    00
       0  ACCEL_FCHOICE           0 :Bypass accel DLPF, 1 :Enable accel DLPF          1
    */
    setAccelConfig = setAccelConfig | 0x01; // Set enable accel DLPF for the accelerometer


    setAccelConfig = setAccelConfig | 0x18; // and set DLFPFCFG to 50.4 hz

    //update configuration register for accelerometer
    writeByte(ICM20649_ADDR,ACCEL_CONFIG_1, setAccelConfig);


    //LSB for accel sample Rate
    // ODR =  1.125kHz/(1+ACCEL_SMPLRT_DIV[11:0]) => 53.57 ?
    writeByte(ICM20649_ADDR,ACCEL_SMPLRT_DIV_1, 0x00); //[11:8]
    writeByte(ICM20649_ADDR,ACCEL_SMPLRT_DIV_2, 0x14); //[7:0]

    accelConfig = readByte(ICM20649_ADDR,ACCEL_CONFIG_1);


    /* Set GYRO_CONFIG_1 to selected DPS Range - Default value 0x01 (250 dps)
    =====================================================================
    BIT  Symbol     		Description                                   Default
    7:6  -          		Reserved				           00
    5:3  GYRO_DLPFCFG[2:0]	Gyro low pass filter configuration                000
    2:1  GYRO_FS_SEL[1:0]        	Gyro Full Scale Select:				   00
          00 = +-500 dps
                        01 = +-1000 dps
                        10 = +-2000 dps
                        11 = +-4000 dps
      0  GYRO_FCHOICE    		0 :Bypass gyro DLPF, 1 :Enable gyro DLPF            1

    */


    switch(rangeGyro)
    {
      case GYRO_RANGE_500DPS:
        gyroConfig = 0x01;
        break;
      case GYRO_RANGE_1000DPS:
        gyroConfig = 0x03;
        break;
      case GYRO_RANGE_2000DPS:
        gyroConfig = 0x05;
        break;
      case GYRO_RANGE_4000DPS:
        gyroConfig = 0x07;
        break;
    }

    writeByte(ICM20649_ADDR,GYRO_CONFIG_1, gyroConfig);
    delay(100); // 60 ms + 1/ODR
    // ODR =  1.1kHz/(1+GYRO_SMPLRT_DIV[7:0]) => 100 ?
    writeByte(ICM20649_ADDR,GYRO_SMPLRT_DIV, 0x0A); // Set gyro sample rate divider

    //reset to register bank 0
    writeByte(ICM20649_ADDR,REG_BANK_SEL, 0x00);
    return true;
}


void ICM20649::readAcceleration(void)
{
    uint8_t buf[6] = {0};
    uint16_t ax = 0, ay = 0, az = 0;
    float value = 0;

    readByte(ICM20649_ADDR, ACCEL_XOUT_H , buf, 6);

    ax = (buf[0] << 8) | buf[1];
    ay = (buf[2] << 8) | buf[3];
    az = (buf[4] << 8) | buf[5];


    value = (int16_t)ax;
    accelRaw.x = value;
    accelInG.x = _rangeAccel * value / 32768;

    value = (int16_t)ay;
    accelRaw.y = value;
    accelInG.y = _rangeAccel * value / 32768;

    value = (int16_t)az;
    accelRaw.z = value; //accRange * value / 32768;
    accelInG.z = _rangeAccel * value / 32768;

}

void ICM20649::readGyro(void)
{
    uint8_t buf[6] = {0};
    uint16_t gx = 0, gy = 0, gz = 0;
    float value = 0;

    readByte(ICM20649_ADDR, GYRO_XOUT_H , buf, 6);

    gx = (buf[0] << 8) | buf[1];
    gy = (buf[2] << 8) | buf[3];
    gz = (buf[4] << 8) | buf[5];


    value = (int16_t)gx;
    gyroRaw.x = value;
    gyroDPS.x = _rangeGyro * value / 32768;

    value = (int16_t)gy;
    gyroRaw.y = value;
    gyroDPS.y = _rangeGyro * value / 32768;

    value = (int16_t)gz;
    gyroRaw.z = value; //accRange * value / 32768;
    gyroDPS.z = _rangeGyro * value / 32768;

}



void ICM20649::readTemperature(void)
{
  uint8_t buf[2] = {0};
  //float value;

  readByte(ICM20649_ADDR,TEMP_OUT_H,buf,2);


  tempRaw = (buf[0] << 8) | buf[1];



  //To convert the output of the temperature sensor to degrees C use the following formula:
  //TEMP_degC = ((TEMP_OUT – RoomTemp_Offset)/Temp_Sensitivity) + 21degC

}



/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

 void ICM20649::writeByte(int dev_addr, uint8_t reg, uint8_t value)
 {
  Wire.beginTransmission(dev_addr);
  Wire.write((uint8_t)reg);
  Wire.write((uint8_t)value);
  Wire.endTransmission();
 }


 uint8_t ICM20649::readByte(int dev_addr,uint8_t reg)
 {
  uint8_t value=0;
  Wire.beginTransmission(dev_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(dev_addr, 1);
  if (Wire.available() >= 1){
    value = Wire.read();
  }
  return value;
 }


 void ICM20649::readByte(int dev_addr, uint8_t reg, uint8_t *buf, uint16_t len)
 {

     Wire.beginTransmission(dev_addr);
     Wire.write(reg);
     Wire.endTransmission();

     Wire.requestFrom(dev_addr, len);
     while(Wire.available())
     {
         for(uint16_t i = 0; i < len; i ++) buf[i] = Wire.read();
     }
 }
