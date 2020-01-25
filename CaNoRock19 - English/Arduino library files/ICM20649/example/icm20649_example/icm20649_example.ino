/**
 * @Author: Ketil Røed
 * @Date:   2019-04-19T14:11:21+02:00
 * @Filename: icm20649_example.ino
 * @Last modified by:   ketilroe
 * @Last modified time: 2019-04-19T15:01:08+02:00
 * @Copyright: Ketil Røed
 */


#include <Arduino.h>
#include <ICM20649.h>



ICM20649 imu;
int ledPin = 13;

void setupPorts(){
    pinMode(ledPin,OUTPUT);
}

void setup() {
    Serial.begin(115200);
    setupPorts();
    if(!imu.initialize(ACCEL_RANGE_8G, GYRO_RANGE_2000DPS))
    {
        Serial.println("Connection to IMU can not be established. Check wiring.");
    };  
}


void loop() {
    digitalWrite(ledPin,HIGH);
    delay(200);
    digitalWrite(ledPin,LOW);
    delay(200);

    imu.readAcceleration();
    imu.readGyro();
    imu.readTemperature();

    Serial.print(imu.accelInG.x,2);
    Serial.print(",");
    Serial.print(imu.accelInG.y,2);
    Serial.print(",");
    Serial.print(imu.accelInG.z,2);
    Serial.print(",  ");
    Serial.print(imu.gyroDPS.x,2);
    Serial.print(",");
    Serial.print(imu.gyroDPS.y,2);
    Serial.print(",");
    Serial.print(imu.gyroDPS.z,2);
    Serial.print(",  ");
    Serial.println(imu.tempRaw,HEX);
  

}
