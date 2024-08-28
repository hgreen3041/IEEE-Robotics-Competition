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




  // Calculate coordinates from sensor data
   x = (0.5)*(sensor1 + halfRobotWidthx) + (0.5)*(24 - halfRobotWidthx - sensor3);
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


float kx = 38;
float ky = 7.3;
float kyaw = 5.7;

volatile double yaw = 0;


int pwm1;
int pwm2;
int pwm3;
int pwm4;

float errorx;
float errory;
float erroryaw; 

void navigateToButton1(){
  currLocation = getCoordinates();
  errorx = 19.25 - currLocation.x;
  errory = 78.0 - currLocation.y;
  erroryaw = 0.0 - yaw;

  while(abs(errory) >= 3.0){
  currLocation = getCoordinates();
  errorx = 19.25 - currLocation.x;
  errory = 78.0 - currLocation.y;
  erroryaw = 0.0 - yaw;
  Serial.print("X: ");
  Serial.println(currLocation.x);
  Serial.print("Y: ");
  Serial.println(currLocation.y);
  Serial.print("Errory: ");
  Serial.println(errory);
  Serial.print("Errorx: ");
  Serial.println(errorx);
  Serial.println("Navigating to button 1.");

  pwm1 = int((kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
  pwm2 = int((-kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 
  pwm3 = int((-kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
  pwm4 = int((kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 
  
  robot.M1(pwm1);
  robot.M2(pwm2);
  robot.M3(pwm3);
  robot.M4(pwm4);

  }
}
float xTakeoff = 12;

void pushButton(int button){
  currLocation = getCoordinates();
  if(button == 1){
    xTakeoff = 13.0;
  }

  if(button == 2){
    xTakeoff = 11.0;
  }

  errorx = xTakeoff - currLocation.x;
  errory = 0.0;
  erroryaw = 0.0 - yaw;

  while(abs(errorx) >= 1.0 ){
    currLocation = getCoordinates();
    errorx = xTakeoff - currLocation.x;
    erroryaw = 0.0 - yaw;
    pwm1 = int((kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
    pwm2 = int((-kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 
    pwm3 = int((-kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
    pwm4 = int((kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 

    Serial.print("Errory: ");
    Serial.println(errory);
    Serial.print("Errorx: ");
    Serial.println(errorx);

    Serial.print("Pushing button ");
    Serial.println(button);
    
    robot.M1(pwm1);
    robot.M2(pwm2);
    robot.M3(pwm3);
    robot.M4(pwm4);
  }

  if(button == 1){
    robot.stopMotors();
    robot.moveForward(212);
    delay(450);
    robot.stopMotors();
  }

  else if(button == 2){
    robot.stopMotors();
    robot.moveBackward(212);
    delay(450);
    robot.stopMotors();
  }

  else{
    robot.stopMotors();
  }



}

void navigateToButton2(){
   currLocation = getCoordinates();
  errorx = 4.75 - currLocation.x;
  errory = 18.0 - currLocation.y;
  erroryaw = 0.0 - yaw;

  while(abs(errory) >= 3.0){
  currLocation = getCoordinates();
  errorx = 4.75 - currLocation.x;
  errory = 18.0 - currLocation.y;
  erroryaw = 0.0 - yaw;

  pwm1 = int((kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
  pwm2 = int((-kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 
  pwm3 = int((-kx*errorx + ky*errory - kyaw*erroryaw)/(500)*255); 
  pwm4 = int((kx*errorx + ky*errory + kyaw*erroryaw)/(500)*255); 

  Serial.print("Errory: ");
  Serial.println(errory);
  Serial.print("Errorx: ");
  Serial.println(errorx);

  Serial.println("Navigating to button2");
  
  robot.M1(pwm1);
  robot.M2(pwm2);
  robot.M3(pwm3);
  robot.M4(pwm4);

  }
}


String yawString; 

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
const Coordinates button1 = {12.0, 94};
const Coordinates button2 = {12.0, 1};


// Create states (last known coil)
enum State { start, atButton1, atButton2, finished, button1Pressed }; 


State currentState = start; // initialize the starting location of the robot 
// Coordinates previousCoordinates = coilB; //innitalize with starting location of coil
int loopCount = 0;
float initialYaw = 0.0; // inital yaw on startup

Coordinates destination;
int ledStatus = 1;



int remoteCount = 0;



void loop1(){
  yaw = getYaw();
}

void loop(){
  
  switch (currentState){
  case start: 
    robot.moveRight(200);
    delay(500);
    robot.stopMotors();
    navigateToButton1();
    currentState = atButton1;
    break;
  case atButton1:
   pushButton(1);
   robot.moveLeft(200);
   delay(500);
   robot.stopMotors();
   currentState = button1Pressed;
    break;
  
  case button1Pressed: 
    navigateToButton2();
    currentState = atButton2;
    break;

  case atButton2:
  pushButton(2);
  currentState = finished;
  break;
  
  case finished:
    while(1){}
    break;
  }


  



}

