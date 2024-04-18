#include <VL53L1X.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Arduino.h>
#include <motordriver.h>
#include "Wire.h"

// initialize LED
const int led = 25;

//  I2C1 Pins
#define sda 6
#define scl 7

const int TX2 = 8;
const int RX2 = 9;

const int TX1 = 0;
const int RX1 = 1;

// arduino::UART Serial2(TX1, RX1);
// arduino::MbedI2C Wire1(sda, scl);

// ICM20948 - pico IMU

// IMU restart
#define imuRestart 28

//VL53L1X Sensors
const int sensorCount = 4;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {11,22,17,16};

// Create sensors object
VL53L1X sensors[sensorCount];
int sensorAdresses[] = {0x2A, 0x2B, 0x2C, 0x2D};


// Create motor driver object 
motordriver robot;

// Radio --------------------------

// Sets radio settings
void radioSetup(){
  Serial2.println("AT+RESET");
  Serial2.readString();
  Serial2.readString();
  Serial2.println("AT+BAND=915000000");
  Serial2.readString();
  Serial2.println("AT+NETWORKID=5");
  Serial2.readString();
  Serial2.println("AT+ADDRESS=123");
  Serial2.readString();
}

// Struct for coordinates
struct Coordinates{
  double x;
  double y;
};


void setup1(){
   Serial1.setRX(RX1);
  Serial1.setTX(TX1);
  Serial1.begin(115200);

  Serial1.println("RESTART");


}


void setup(){

  // pinMode(imuRestart, OUTPUT);
  pinMode(led, OUTPUT);
  
    Serial.begin(115200);
    // while (!Serial) 
    // {
    // digitalWrite(led, HIGH);
    // delay(100);
    // digitalWrite(led, LOW);
    // delay(100);
    // digitalWrite(led, HIGH);
    // delay(100);
    // digitalWrite(led, LOW);
    // delay(500);
    // }
    // delay(10);
  
  // Initialize I2C 
  Wire.begin();
  Wire.setClock(30000); //30000
  // Wire1.setSCL(scl);
  // Wire1.setSDA(sda);
  // Wire1.begin();
  // Wire1.setClock(30000);


  // VL53L1X--------------------------------------------------------

   // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCount; i++){
      pinMode(xshutPins[i], OUTPUT);
      digitalWrite(xshutPins[i], LOW);
    }


  // Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    pinMode(xshutPins[i], INPUT);
    delay(10);

    sensors[i].setTimeout(250);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i+1);
      while (1){
        delay(500);
        Serial.println("Still searching for sensors");
        if(sensors[i].init()){
          Serial.print("Sensor ");
          Serial.print(i+1);
          Serial.print(" found");
          break;
        }
      };
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.

    sensors[i].setAddress(sensorAdresses[i]);
    sensors[i].startContinuous(50);
  }

  // Radio--------------------------------------------
  Serial2.begin(115200);
  Serial2.setTX(TX2);
  Serial2.setRX(RX2);
  Serial2.setTimeout(1000);
  Serial2.begin(115200);

  radioSetup();

  // UART IMU
  // Serial1.setRX(RX1);
  // Serial1.setTX(TX1);
  // Serial1.begin(115200);

}

Coordinates currLocation;
Coordinates prevLocation = {0,0};
double x; 
double y;
double halfRobotWidthx = 3.425;
double halfRobotWidthy = 4.76;

double sensor1;
double sensor2;
double sensor3;
double sensor4;

// Function to retrieve the current location of the robot
struct Coordinates getCoordinates(){
  // Set the half-width and half-length of the robot. 
  // Read the sensors
  sensor1 = static_cast<double>(sensors[0].read());
  // delay(5);
  sensor1 = sensor1/25.4;
  sensor2 = static_cast<double>(sensors[1].read());
  // delay(5);
  sensor2 = sensor2/25.4;
  sensor3 = static_cast<double>(sensors[2].read());
  // delay(5);
  sensor3 = sensor3/25.4;
  sensor4 = static_cast<double>(sensors[3].read());
  // delay(5);
  sensor4 = sensor4/25.4;


// Sets up sensor again on failure AKA: sensor == 0
//  if(sensor1 == 0.0){
//     sensors[0].setAddress(sensorAdresses[0]);
//     sensor1 = static_cast<double>(sensors[0].read(false));
//     sensor1 = sensor1/25.4;
//     Serial.println("Sensor1 reset");
//  }

//   if(sensor2 == 0.0){
//     sensors[1].setAddress(sensorAdresses[1]);
//     sensor2 = static_cast<double>(sensors[1].read(false));
//     sensor2 = sensor2/25.4;
//     Serial.println("Sensor2 reset");
//  }

//   if(sensor3 == 0.0){
//     sensors[2].setAddress(sensorAdresses[2]);
//     sensor3 = static_cast<double>(sensors[2].read(false));
//     sensor3 = sensor3/25.4;
//     Serial.println("Sensor3 reset");
    
//  }

//   if(sensor4 == 0.0){
//     sensors[3].setAddress(sensorAdresses[3]);
//     sensor4 = static_cast<double>(sensors[3].read(false));
//     sensor4 = sensor4/25.4;
//     Serial.println("Sensor4 reset");
//  }

  // Calculate coordinates from sensor data
   x = (0.5)*(sensor1 + halfRobotWidthx) + (0.5)*(96 - halfRobotWidthx - sensor3);
   y = (0.5)*(sensor4 + halfRobotWidthy) + (0.5)*(96 - halfRobotWidthy - sensor2);




  Serial.println("running");
  Serial.print("sensor1 ");
  Serial.print( sensor1 );
    Serial.print(" sensor3 ");
  Serial.println(sensor3);
  Serial.print("sensor2 ");
    Serial.print(sensor2);
      Serial.print(" sensor4 ");
  Serial.println(sensor4);


  return {x,y};
}



String yawString; 
volatile double yaw = 0;

// Get Yaw from UART connection
double getYaw(){
 
  yawString = Serial1.readStringUntil('\n');
  yaw = yawString.toFloat();
  
  return yaw;
}


// Coil locations in inches
// additions/subtractions are for the offset of the robot
// x-coil distance to center: 4.53
// y-coil distance to center: 5.9
const Coordinates coilA = {24.0, 0.0 + 6.0};
const Coordinates coilB = {72.0, 0.0 + 6.0};
const Coordinates coilC = {96.0 - 4.5, 24.0};
const Coordinates coilD = {96.0 - 4.5, 72.0};
const Coordinates coilE = {72.0, 96.0 - 6.0};
const Coordinates coilF = {32.0, 96.0 - 6.0};
const Coordinates coilG = {0.0 + 4.5, 72.0};
const Coordinates coilH = {0.0 + 4.5, 24.0};

// Create states (last known coil)
enum State { stateA, stateD, stateH, stateF, stateB, stateG, stateE, stateC }; 


State currentState = stateB; // initialize the starting location of the robot 
Coordinates previousCoordinates = coilB; //innitalize with starting location of coil
int loopCount = 0;
float initialYaw = 0.0; // inital yaw on startup

Coordinates destination;
int ledStatus = 1;

int pwm1;
int pwm2;
int pwm3;
int pwm4;

int remoteCount = 0;



void loop1(){
  yaw = getYaw();
}

void loop(){
  

// OPTION 1 -----------------------------------------------------------------------------------------------

switch (currentState){
  case stateA: 
    destination = coilD;
    currentState = stateD;
    Serial.println("Reached coil A");
    break;
  case stateB:
    destination = coilG;
    currentState = stateG;
    Serial.println("Reached coil B");
    break;
  case stateC:
    destination = coilA;
    currentState = stateA;
    Serial.println("Reached coil C");
    break;
  case stateD:
    destination = coilH;
    currentState = stateH;
    Serial.println("Reached coil D");
    break;
  case stateE:
    destination = coilC;
    currentState = stateC;
    Serial.println("Reached coil E");
    break;
  case stateF:
    destination = coilB;
    currentState = stateB;
    Serial.println("Reached coil F");
    break;
  case stateG:
    destination = coilE;
    currentState = stateE;
    Serial.println("Reached coil G");
    break;
  case stateH:
    destination = coilF;
    currentState = stateF;
    Serial.println("Reached coil H");
    break;
}

  currLocation = getCoordinates();
  double errorx = destination.x - currLocation.x;
  double errory = destination.y - currLocation.y;
  double erroryaw = initialYaw - yaw;



// // "P" loop for motor control
// // TODO: Add tolerance (error will never actually be 0)


while(abs(errorx) > 4.0 || abs(errory) > 4.0){

  while(remoteCount % 2 == 0){
    robot.stopMotors();
    getCoordinates();
    Serial.print("Yaw: ");
    Serial.println(yaw);
    Serial.println("Waiting for start.");
    digitalWrite(led, HIGH);
  
    
    if(Serial2.available() > 0){
      remoteCount += 1;
      Serial.println(Serial2.readString());
      Serial.println("Starting movement");
      
    }
 }


  
  if(ledStatus == 1){
    digitalWrite(led,LOW);
    ledStatus = 0;
  }
  else{
    digitalWrite(led, HIGH);
    ledStatus = 1;
  }

float kx = 9.25;
float ky = 8.75;
float kyaw = 5.7;


  pwm1 = int((kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
  pwm2 = int((-kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 
  pwm3 = int((-kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
  pwm4 = int((kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 



  // Serial.print("PWM1: ");
  // Serial.println(pwm1);
  // Serial.print("PWM2: ");
  // Serial.println(pwm2);
  // Serial.print("PWM3: ");
  // Serial.println(pwm3);
  // Serial.print("PWM4: ");
  // Serial.println(pwm4);

  
  robot.M1(pwm1);
  robot.M2(pwm2);
  robot.M3(pwm3);
  robot.M4(pwm4);




  currLocation = getCoordinates();
  errorx = destination.x - currLocation.x;
  errory = destination.y - currLocation.y;
  // PAUSE CORE?
  erroryaw = initialYaw - yaw;


  Serial.print("X-Coordinate: ");
  Serial.println(currLocation.x);
  Serial.print("Y-Coordinate: ");
  Serial.println(currLocation.y);
  Serial.print("Destination X: ");
  Serial.println(destination.x);
  Serial.print("Destination Y: ");
  Serial.println(destination.y);
  Serial.print("ErrorX: ");
  Serial.println(errorx);
  Serial.print("ErrorY: ");
  Serial.println(errory);
  Serial.print("ErrorYaw: ");
  Serial.println(erroryaw);
  Serial.print("Yaw: ");
  Serial.println(yaw);
  

    if(Serial2.available() > 0){
    robot.stopMotors();
    remoteCount += 1;
    Serial.println(Serial2.readString());
    
  }


}









loopCount += 1;



}

