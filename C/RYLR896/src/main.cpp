#include <Arduino.h>


const int led = 25;
const int TX = 8;
const int RX = 9;

void radioSetup(){
  Serial2.println("AT+RESET");
  delay(1000);
  Serial2.println("AT+ADDRESS=18");
  delay(1000);
  Serial2.println("AT+NETWORKID=4");
  delay(1000);
  Serial2.println("AT+BAND=86850000");
  delay(1000);
  Serial2.println("AT+PARAMETER=10,7,1,7");
  delay(1000);
  Serial2.println("AT+VER?");
  delay(1000);
  // Serial2.println("AT+FACTORY");
  // Serial.print("Setting to factory settings...");
  // Serial.println(Serial2.readString());
}

void setup() {
  Serial2.setTX(TX);
  Serial2.setRX(RX);
  Serial2.setTimeout(1000);
  Serial2.begin(115200);
  Serial.begin(115200);

  pinMode(led, OUTPUT);

  // receiver only
  while(!Serial){}
  delay(10);

  radioSetup();
}

String message;




//TX-------------------------------------------

// void loop() {

  
//   Serial2.println("AT+SEND=50,5,HELLO");
//   digitalWrite(led, HIGH);
//   delay(100);
  
//   if(Serial2.available()){
//     message = Serial2.readString();
//     Serial.println(message);
    
//   }
//   digitalWrite(led, LOW);
//   delay(100);
// }


//RX---------------------------------------------------
void loop(){
  if(Serial2.available() > 0){
    Serial.println(Serial2.readString());
  }

  delay(1000);
  
}

