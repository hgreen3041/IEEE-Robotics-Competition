#include <VL53L1X.h>
#include <Adafruit_BNO08x.h>
// #include <iostream>
#include <Arduino.h>
// #include <stdio.h>
#include <motordriver.h>

// initialize LED
const int led = 25;

// BNO08X IMU
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


//VL53L1X Sensors
const int sensorCount = 4;


// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {6,22,17,16};

// Create sensors object
VL53L1X sensors[sensorCount];

// Create motor driver object 
motordriver robot;


// Course specific stuff
struct Coordinates{
  double x;
  double y;
};



void setup(){

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

    Serial.begin(115200);
    while (!Serial) delay(10);

  // BNO08X IMU------------------------------------------------------
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }


  setReports(reportType, reportIntervalUs);

  delay(100);

  
  // VL53L1X---------------------------------------------------------
  Wire.begin();
  Wire.setClock(400000);

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

    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }

    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensors[i].setAddress(0x2A + i);

    sensors[i].startContinuous(50);
  }
}


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}



// Function to retrieve the current location of the robot
struct Coordinates getCoordinates(){

  Coordinates location;
  // Set the half-width and half-length of the robot. 
  double halfRobotWidthx = 3.75;
  double halfRobotWidthy = 5.25;

  // Read the sensors
  double sensor1 = static_cast<double>(sensors[0].read());
  sensor1 = sensor1/25.4;
  double sensor2 = static_cast<double>(sensors[1].read());
  sensor2 = sensor2/25.4;
  double sensor3 = static_cast<double>(sensors[2].read());
  sensor3 = sensor3/25.4;
  double sensor4 = static_cast<double>(sensors[3].read());
  sensor4 = sensor4/25.4;

  // Calculate coordinates from sensor data
  double x = (0.5)*(sensor1 + halfRobotWidthx) + (0.5)*(96 - halfRobotWidthx - sensor3);
  double y = (0.5)*(sensor4 + halfRobotWidthy) + (0.5)*(96 - halfRobotWidthy - sensor2);


  location = {x, y};

  return location; 
}


// BREAKS THE CODE FOR SOME UNKNOWN REASON
// float getYaw(){
//     if (bno08x.wasReset()) {
//     Serial.print("sensor was reset ");
//     setReports(reportType, reportIntervalUs);
//   }
  
//   if (bno08x.getSensorEvent(&sensorValue)) {
//     // in this demo only one report type will be received depending on FAST_MODE define (above)
//     switch (sensorValue.sensorId) {
//       case SH2_ARVR_STABILIZED_RV:
//         quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
//       case SH2_GYRO_INTEGRATED_RV:
//         // faster (more noise?)
//         quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
//         break;
//     }
    
//     // Serial.print(ypr.yaw); 
//     // Serial.print("\t");              
//     // Serial.print(ypr.pitch);              
//     // Serial.print("\t");
//     // Serial.println(ypr.roll);
//   }

//   return ypr.yaw; 

// }



// Coil locations in inches
// additions/subtractions are for the offset of the robot
const Coordinates coilA = {32.0, 0.0 + 6.25};
const Coordinates coilB = {64.0, 0.0 + 6.25};
const Coordinates coilC = {96.0 - 4.75, 32.0};
const Coordinates coilD = {96.0 - 4.75, 64.0};
const Coordinates coilE = {64.0, 96.0 - 6.25};
const Coordinates coilF = {32.0, 96.0 - 6.25};
const Coordinates coilG = {0.0 + 4.75, 64.0};
const Coordinates coilH = {0.0 + 4.75, 32.0};

// Create states (last known coil)
enum State { stateA, stateD, stateH, stateF, stateB, stateG, stateE, stateC }; 

// Returns the next state given the current state of the robot.
// State getNextState(State currentState){
//   switch(currentState){
//     case stateA:
//       return stateD;
//     case stateD: 
//       return stateH;
//     case stateH:
//       return stateF;
//     case stateF: 
//       return stateB;
//     case stateB: 
//       return stateG;
//     case stateG:
//       return stateE;
//     case stateE:
//       return stateC;
//     case stateC:
//       return stateA;
//   }
//   return stateA;
// }

    if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    
    // Serial.print(ypr.yaw); 
    // Serial.print("\t");              
    // Serial.print(ypr.pitch);              
    // Serial.print("\t");
    // Serial.println(ypr.roll);
  }

  const float initialYaw = ypr.yaw;
  float currentYaw;




State currentState = stateH; // initialize the starting location of the robot


void loop(){


    if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)
    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV:
        quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
      case SH2_GYRO_INTEGRATED_RV:
        // faster (more noise?)
        quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
        break;
    }
    
    // Serial.print(ypr.yaw); 
    // Serial.print("\t");              
    // Serial.print(ypr.pitch);              
    // Serial.print("\t");
    // Serial.println(ypr.roll);
  }

   currentYaw = ypr.yaw;

}


Coordinates currLocation = getCoordinates();



// Serial.print("\t");
// Serial.print("X-coordinate: ");
// Serial.print(getCoordinates().x);
// Serial.print("\t");
// Serial.print("Y-coordinate: ");
// Serial.print(getCoordinates().y);
// Serial.println("\t");



// TODO: Need to implement functions for getting new sensor data if going to use switch statement

// OPTION 1 -----------------------------------------------------------------------------------------------

switch (currentState){
  case stateA: 
    destination = coilD;
    currentState = stateD;
    break;
  case stateB:
    destination = coilG;
    currentState = stateG;
    break;
  case stateC:
    destination = coilA;
    currentState = stateA;
    break;
  case stateD:
    destination = coilH;
    currentState = stateH;
    break;
  case stateE:
    destination = coilC;
    currentState = stateC;
    break;
  case stateF:
    destination = coilB;
    currentState = stateB;
    break;
  case stateG:
    destination = coilE;
    currentState = stateE;
    break;
  case stateH:
    destination = coilF;
    currentState = stateF;
    break;
}

// double errorx = destination.x - getCoordinates().x; 
// double errory = destination.y - getCoordinates().y;
// double erroryaw = initialYaw - currentYaw;

// Serial.println(errorx);
// Serial.println(errory);
// Serial.println(erroryaw);


// "P" loop for motor control
// TODO: Add tolerance (error will never actually be 0)

while(errorx != 0 || errory != 0 || erroryaw != 0){
  int pwmy = int(errorx);
  robot.xMovement(pwmy*5);

  int pwmx = int(errory);
  robot.yMovement(pwmx*5);

  int pwmyaw = int(erroryaw);
  robot.rotationalMovement(pwmyaw*5);

  errorx = destination.x - getCoordinates().x;
  errory = destination.y - getCoordinates().y;
  erroryaw = initialYaw - getYaw();

  

}



// OPTION 2--------------------------------------------------------------------------
// dummy loop

// TODO: Change while condition to utilize xMovement and yMovement in driver file
//        - implement IMU

// switch (currentState){
//   case stateA:
//     while(coilD.y - getCoordinates().y > 0){
//       // drive forward
//       Serial.println("Driving Forward");
//     }

//     while(coilD.x - getCoordinates().x > 0){
//       // drive right
//       Serial.println("Driving Right");
//     }
//     currentState = getNextState(currentState);
//     Serial.print("Arrived at ");
//     Serial.println(currentState);

//   case stateB: 
//     while(coilD.y - getCoordinates().y > 0){
//       // drive forward
//       Serial.println("Driving forward");
//     }
//     while(getCoordinates().x - coilG.x){
//       // drive left
//       Serial.println("Driving left");
//     }
//     currentState = getNextState(currentState);
//     Serial.print("Arrived at: ");
//     Serial.println(currentState);

//   case stateC: 
//     while(getCoordinates().x - coilA.x > 0){
//       // drive left
//       Serial.println("Driving left");
//     }
//     while(getCoordinates().y - coilA.y > 0){
//       // drive backward
//       Serial.println("Driving Backward");
//     }
//     currentState = getNextState(currentState);
//   case stateD: 
//     while(getCoordinates().x - coilH.x > 0){
//       // drive left
//       Serial.println("Driving left");
//     }
//     while(getCoordinates().y - coilH.y > 0){
//       //drive backward
//       Serial.println("Driving backward");
//     }
//     currentState = getNextState(currentState);
//     Serial.print("Arrived at ");
//     Serial.println(currentState);

//   case stateE: 
//     while(getCoordinates().y - coilC.y > 0){
//       // drive backward
//       Serial.println("Driving backward");
//     }
//     while(coilA.x - getCoordinates().x > 0){
//       // drive right
//       Serial.println("Driving Right");
//     }
//     currentState = getNextState(currentState);
//     Serial.print("Arrived at ");
//     Serial.println(currentState);

//   case stateF:
//     while(getCoordinates().y - coilB.y > 0){
//       // drive backward
//       Serial.println("Driving backward");
//     } 
//     while(coilB.x - getCoordinates().x > 0){
//       // drive right
//       Serial.println("Driving Right");
//     }
//     Serial.print("Arrived at ");
//     Serial.println(currentState);
//     currentState = getNextState(currentState);

//   case stateG:
//     while(coilE.x - getCoordinates().x > 0){
//       // drive right
//       Serial.println("Driving Right");
//     }
//     while(coilE.y - getCoordinates().y > 0){
//       // drive forward
//       Serial.println("Driving forward");
//     }
//     currentState = getNextState(currentState);
//     Serial.print("Arrived at ");
//     Serial.println(currentState);

//   case stateH: 
//     while(coilF.x - getCoordinates().x > 0){
//       // drive right
//       Serial.println("Driving right");
//     }
//     while(coilF.y - getCoordinates().y > 0){
//       // drive forward
//       Serial.println("Driving forward");
//     }
//     currentState = getNextState(currentState);
//     Serial.print("Arrived at ");
//     Serial.println(currentState);
// }






