#include <SparkFunLSM9DS1.h>

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW
#define LENGTH_OF_BUFFER  (5*(2+18))
#define SYNC_WORD_0     0xA0
#define SYNC_WORD_1     0xA1
#define DEBUG 1

uint8_t to_encoder[LENGTH_OF_BUFFER+100], GPS_buffer[66];
uint8_t tmp, prev_tmp, GPS_data[66], imu_buffer[18];
unsigned long counter=0, counter_imugps=0, counter_2=0, gps_time=0, show_time;
unsigned int counter_frame=0;
bool lock=false, update_imu=true;
LSM9DS1 imu;

void setup() {
#if DEBUG
  Serial.begin(9600);
  //while(!Serial);
#endif

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

#if DEBUG
  Serial.println("Starting...");
#endif

#if DEBUG
  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1. Double-check wiring.");
    while(1);
  }
#endif

  imu.setAccelScale(16); // acc scale: 0.000732
  imu.setGyroScale(2000); // gyro scale: 0.07
  // mag is 4 gauss by default. mag scale: 0.00014 (gauss)

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
  //GPIOC_PDOR = (uint32_t)0;

  pinMode(2, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(2), irq_encoder, RISING);

#if DEBUG
  Serial.println("Done with startup");
#endif
}

void loop() {
  if (update_imu) {
    //Serial.println("HEEEI!");
    update_imu = false;
    
    if (imu.gyroAvailable())
      imu.readGyro();
    if (imu.accelAvailable())
      imu.readAccel();
    if (imu.magAvailable())
      imu.readMag();

    // Acc
    imu_buffer[0] = (imu.ax&0xFF00)>>8;
    imu_buffer[1] = imu.ax&0xFF;
    imu_buffer[2] = (imu.ay&0xFF00)>>8;
    imu_buffer[3] = imu.ay&0xFF;
    imu_buffer[4] = (imu.az&0xFF00)>>8;
    imu_buffer[5] = imu.az&0xFF;

    // Gyro
    imu_buffer[6] = (imu.gx&0xFF00)>>8;
    imu_buffer[7] = imu.gx&0xFF;
    imu_buffer[8] = (imu.gy&0xFF00)>>8;
    imu_buffer[9] = imu.gy&0xFF;
    imu_buffer[10] = (imu.gz&0xFF00)>>8;
    imu_buffer[11] = imu.gz&0xFF;

    // Gyro
    imu_buffer[12] = (imu.mx&0xFF00)>>8;
    imu_buffer[13] = imu.mx&0xFF;
    imu_buffer[14] = (imu.my&0xFF00)>>8;
    imu_buffer[15] = imu.my&0xFF;
    imu_buffer[16] = (imu.mz&0xFF00)>>8;
    imu_buffer[17] = imu.mz&0xFF;
  }  

  while (Serial1.available()) {
    if (counter_imugps) {
#if DEBUG
      Serial.print("Counter-thingy: ");
      Serial.println(counter_imugps);
#endif
      counter_imugps=0;
    }

    prev_tmp = tmp;
    //Serial.println("Iz hear");
    tmp = Serial1.read();
    if (lock) {
      counter++;

      //Serial.print("Med lock. Counter:  ");
      //Serial.print(counter);
      //Serial.print("    tmp: 0x");
      //Serial.println(tmp, HEX);
      
      if (counter == 66) {
        counter = 0;
        //Serial.println("Updating GPS");
        update_GPS_data();
      }

      GPS_data[counter] = tmp;
      
      if ((counter == 0) && (tmp != 0xA0)) { // If true, then lost lock
        lock = false;
#if DEBUG
        Serial.print("Lost lock 0: 0x");
        Serial.println(tmp, HEX);
#endif
      }
      if ((counter == 1) && (tmp != 0xA1)) { // If true, then lost lock
        lock = false;
#if DEBUG
        Serial.print("Lost lock 1: 0x");
        Serial.println(tmp, HEX);
#endif
      }
    }
    else {
      if ((prev_tmp == 0xA0) && (tmp == 0xA1)) {
        lock = true;
        counter = 1;

        Serial.println("Got lock!"); 
      }

      //Serial.print("Uten lock, tmp= 0x");
      //Serial.println(tmp, HEX);
      //Serial.println("No lock"); 
    }
  }

  //Serial.println(counter_frame);
  
  /*
  if (Serial1.available()) {
    Serial.println(Serial1.read(),HEX);
    //Serial.write(Serial1.read());
  }
  */

  if (millis()-show_time > 100) {
    show_time=millis();
#if DEBUG
    //update_imu=true;
    //Serial.print(show_time);
    //Serial.print("\t");
    //Serial.print(counter_2);
    //Serial.println();
#endif
  }
}

void irq_encoder(void) {
  int i;
  
  counter_frame++;
  if (counter_frame >= LENGTH_OF_BUFFER) {
    counter_frame=0;
    update_imu=true;
  }

  GPIOC_PDOR = (uint32_t)((GPIOC_PDOR&0xFFFFFF00) | to_encoder[counter_frame]); // Output next byte to encoder
  //digitalWriteFast(13, (to_encoder[counter_frame]&0b0010000)>>5);
  //digitalWriteFast(12, (to_encoder[counter_frame]&0x80)>>7);
  //Serial.println(to_encoder[counter_frame]);

  if (counter_frame == 0) { // update GPS
    for (i=0; i<18; i++)
      to_encoder[2+i] = GPS_buffer[i];
  }
  else if (counter_frame%20 == 0) {
    for (i=0; i<18; i++)
      to_encoder[2+counter_frame+i] = imu_buffer[i];
    update_imu=true;
  }
  
  Serial.print("counter_frame: ");
  Serial.println(counter_frame);
  Serial.print("to_encoder[counter_frame]: ");
  Serial.println(to_encoder[counter_frame]);

  counter_2++;
  //Serial.println(counter_2++);
}

/*
 * byte 0:  fix and number of satellites (others?) and accuracy?
 * byte 1-3: latitude (discard msb?)
 * byte 4-6: longitude (discard msb)
 * byte 7-8: altitude (discard someting, and what about negative?)
 * byte 9-11: velocity x (discard something?)
 * byte 12-14: velocity y (discard something?)
 * byte 15-17: velocity z (discard something?)
 */
void update_GPS_data() {
  
  // Fix and number of satellites
  GPS_buffer[0] = ((GPS_data[3+2]&0x01 << 7) | (GPS_data[3+3]&0x7F));
  if (millis()-gps_time > 200) { // Then something is wrong, and we flag it
    gps_time = millis();
    GPS_buffer[0] |= 64;
  }

  /*
  // Latitude
  GPS_buffer[1] = ((GPS_data[3+10]&0x80) | (GPS_data[3+11]&0x7F));
  GPS_buffer[2] = GPS_data[3+12];
  GPS_buffer[3] = GPS_data[3+13];

  // Longitude
  GPS_buffer[4] = ((GPS_data[3+14]&0x80) | (GPS_data[3+15]&0x7F));
  GPS_buffer[5] = GPS_data[3+16];
  GPS_buffer[6] = GPS_data[3+17];

  // Altitude (if GPS_data[3+22]&0x80, then the unsigned integer is indeed negative
  GPS_buffer[7] = ((GPS_data[3+22]&0x80) | (GPS_data[3+23]&0x7F)); 
  GPS_buffer[8] = GPS_data[3+24];
  GPS_buffer[9] = GPS_data[3+25];
  */
  
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
  
  // Caclulate total celcity (get the direction from the change in position, which should be fine enough
  // The largest total velocity is 165 772 m/s, which should be high enough :)
  int32_t EXEF_X_Vel_SINT = (GPS_data[3+48] << 24) + (GPS_data[3+49] << 16) + (GPS_data[3+50] << 8) + (GPS_data[3+51] << 0);
  int32_t EXEF_Y_Vel_SINT = (GPS_data[3+52] << 24) + (GPS_data[3+53] << 16) + (GPS_data[3+54] << 8) + (GPS_data[3+55] << 0);
  int32_t EXEF_Z_Vel_SINT = (GPS_data[3+56] << 24) + (GPS_data[3+57] << 16) + (GPS_data[3+58] << 8) + (GPS_data[3+59] << 0);
  uint32_t Total_Vel = sqrt(EXEF_X_Vel_SINT*EXEF_X_Vel_SINT + EXEF_Y_Vel_SINT*EXEF_Y_Vel_SINT +
    EXEF_Z_Vel_SINT*EXEF_Z_Vel_SINT);
  GPS_buffer[12] = (Total_Vel&0xFF0000) >> 16; // discard higher values, as they shouldn't exist
  GPS_buffer[13] = (Total_Vel&0xFF00) >> 8;
  GPS_buffer[14] = Total_Vel&0xFF; // LSB
}
