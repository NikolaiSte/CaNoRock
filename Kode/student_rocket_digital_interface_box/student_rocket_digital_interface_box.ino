#define CLOCK_PIN 2
#define DATA_DELAY 20

void setup() {
  Serial.begin(9600);
  while(!Serial);

  digitalWriteFast(CLOCK_PIN, HIGH);
  pinMode(CLOCK_PIN, OUTPUT);
  for (int i=0; i<8; i++)
    pinMode((3+i), INPUT_PULLUP);
  
  pinMode(13, OUTPUT);
  digitalWriteFast(13, HIGH);
}

void loop() {
  uint8_t tmp_data;
    
  tmp_data = interfaceReader();
  //Serial.print(micros());
  Serial.print(millis());
  Serial.print(",");
  Serial.print(tmp_data);
  Serial.println();

  delay(DATA_DELAY);
}

uint8_t interfaceReader() {
  uint8_t tmp, i;
  
  digitalWriteFast(CLOCK_PIN, HIGH);
  delayMicroseconds(64);
  digitalWriteFast(CLOCK_PIN, LOW);
  delayMicroseconds(64);

  // Using digitalReadFast, the input cannot be a variable
  // The order was more random than it should have been
  tmp = digitalReadFast(5);
  tmp |= (digitalReadFast(3) << 1);
  tmp |= (digitalReadFast(6) << 2);
  tmp |= (digitalReadFast(4) << 3);
  tmp |= (digitalReadFast(8) << 4);
  tmp |= (digitalReadFast(7) << 5);
  tmp |= (digitalReadFast(10) << 6);
  tmp |= (digitalReadFast(9) << 7);

  return(tmp);
}
