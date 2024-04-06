#include <Arduino.h>

#define RX 1
#define TX 0
#define led 25

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  Serial1.setRX(RX);
  Serial1.setTX(TX);
  // Serial1.setTimeout(500);
  Serial1.begin(115200);
  
}

void loop() {

  while(!Serial){}
  delay(10);


  digitalWrite(led, HIGH);
  String yaw = Serial1.readStringUntil('\n');
  Serial.println(yaw);
  delay(50);
  digitalWrite(led, LOW);
  delay(50);
}


