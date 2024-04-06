#include <Arduino.h>


const int led = 4;
const int TX = 8;
const int RX = 9;

const int Button = 2;

const int pin = 5;



void radioSetup(){
  // Serial2.println("AT+RESET");
  // Serial.print("Response: ");
  // Serial.println(Serial2.readString());
  // Serial.println(Serial2.readString());
  // Serial2.println("AT+BAND=915000000");
  // Serial.print("Response: ");
  // Serial.println(Serial2.readString());
  // Serial2.println("AT+NETWORKID=5");
  // Serial.print("Response: ");
  // Serial.println(Serial2.readString());
  // Serial2.println("AT+ADDRESS=123");
  // Serial.print("Response: ");
  // Serial.println(Serial2.readString());

  Serial2.println("AT+RESET");
  // Serial.println(Serial2.readString());
  // Serial.println(Serial2.readString());
  Serial2.readString();
  Serial2.readString();
  Serial2.println("AT+BAND=915000000");
  Serial2.readString();
  // Serial.println(Serial2.readString());

  Serial2.println("AT+NETWORKID=5");
  Serial2.readString();
  Serial2.println("AT+ADDRESS=123");
  Serial2.readString();




}


void setup() {

  Serial2.setTX(TX);
  Serial2.setRX(RX);
  Serial2.setTimeout(1000);
  Serial2.begin(115200);
  Serial.begin(115200);

  pinMode(led, OUTPUT);
  pinMode(Button, INPUT);
  pinMode(pin, INPUT);

  // receiver only
  // while(!Serial){}
  // delay(10);

  radioSetup();

}

String message;
int buttonCount = 0;
//TX-------------------------------------------

void loop() {
  if(digitalRead(Button) == HIGH){
    delay(200);
    if(buttonCount % 2 == 0){
      Serial2.println("AT+SEND=123,5,START");
      // Serial.println(Serial2.readString());
      buttonCount += 1;
      digitalWrite(led, HIGH);
    }
    else{
      Serial2.println("AT+SEND=123,4,STOP");
      // Serial.println(Serial2.readString());
      buttonCount += 1;
      digitalWrite(led, LOW);
    }

  }


}


//RX---------------------------------------------------
// void loop(){
//   if(Serial2.available() > 0){
//     Serial.println(Serial2.readString());
//   }

//   delay(100);
  
// }


