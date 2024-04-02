#include <Arduino.h>


const int led = 25;
const int TX = 8;
const int RX = 9;

const int Button = 2;



void radioSetup(){
  Serial2.println("AT+RESET");
  Serial.print("Response: ");
  Serial.println(Serial2.readString());
  Serial.println(Serial2.readString());
  Serial2.println("AT+BAND=915000000");
  Serial.print("Response: ");
  Serial.println(Serial2.readString());
  Serial2.println("AT+NETWORKID=5");
  Serial.print("Response: ");
  Serial.println(Serial2.readString());
  Serial2.println("AT+ADDRESS=50");
  Serial.print("Response: ");
  Serial.println(Serial2.readString());

  // Serial2.println("AT+RESET");
  // Serial2.readString();
  // Serial2.readString();
  // Serial2.println("AT+BAND=915000000");
  // Serial2.readString();
  // Serial2.println("AT+NETWORKID=5");
  // Serial2.readString();
  // Serial2.println("AT+ADDRESS=50");
  // Serial2.readString();




}


void setup() {
  Serial2.setTX(TX);
  Serial2.setRX(RX);
  Serial2.setTimeout(1000);
  Serial2.begin(115200);
  Serial.begin(115200);

  pinMode(led, OUTPUT);
  pinMode(Button, INPUT);

  // receiver only
  while(!Serial){}
  delay(10);

  radioSetup();

}

String message;
int buttonCount = 0;
//TX-------------------------------------------

// void loop() {
//   if(digitalRead(Button) == HIGH){
//     delay(300);
//     if(buttonCount % 2 == 0){
//       Serial2.println("AT+SEND=50,5,START");
//       buttonCount += 1;
//       digitalWrite(led, HIGH);
//     }
//     else{
//       Serial2.println("AT+SEND=50,4,STOP");
//       buttonCount += 1;
//       digitalWrite(led, LOW);
//     }

//   }


// }


//RX---------------------------------------------------
void loop(){
  if(Serial2.available() > 0){
    Serial.println(Serial2.readString());
  }

  delay(100);
  
}


