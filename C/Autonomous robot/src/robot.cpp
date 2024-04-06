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

// ICM20948 IMU

// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

//VL53L1X Sensors
const int sensorCount = 4;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = {11,22,17,16};

// Create sensors object
VL53L1X sensors[sensorCount];

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

void setup(){

  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

    Serial.begin(115200);
    while (!Serial) 
    {
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(100);
    digitalWrite(led, HIGH);
    delay(100);
    digitalWrite(led, LOW);
    delay(500);
    }
    delay(10);
  
  // Initialize I2C 
  Wire.begin();
  Wire.setClock(30000);
  // Wire1.setSCL(scl);
  // Wire1.setSDA(sda);
  // Wire1.begin();
  // Wire1.setClock(30000);


  // ICM20948------------------------------------------------------

  // myICM.begin(Wire, 0);
  // bool success = true; // Use success to show if the DMP configuration was successful

  // // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  // success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // // Enable the DMP Game Rotation Vector sensor (Quat6)
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // // Enable additional sensors / features
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);


  // // Configuring DMP to output data at multiple ODRs:
  // // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // // Setting value can be calculated as follows:
  // // Value = (DMP running rate / ODR ) - 1
  // // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) == ICM_20948_Stat_Ok);        // Set to 5Hz
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // // Enable the FIFO
  // success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // // Enable the DMP
  // success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // // Reset DMP
  // success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // // Reset FIFO
  // success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // // Check success
  // if (success)
  // {
  //   Serial.println(F("DMP enabled!"));
  // }
  // else
  // {
  //   Serial.println(F("Enable DMP failed!"));
  //   Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
  //   while (1)
  //     ; // Do nothing more
  // }


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

    sensors[i].setTimeout(500);
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
    sensors[i].setAddress(0x2A + i);
    sensors[i].startContinuous(50);
  }

  // Radio--------------------------------------------
  Serial2.begin(115200);
  Serial2.setTX(TX2);
  Serial2.setRX(DREQ_PIO0_RX2);
  Serial2.setTimeout(1000);
  Serial2.begin(115200);

  radioSetup();

  // UART IMU
  Serial1.setRX(RX1);
  Serial1.setTX(TX1);
  Serial1.begin(115200);

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
   x = (0.5)*(sensor1 + halfRobotWidthx) + (0.5)*(96 - halfRobotWidthx - sensor3);
   y = (0.5)*(sensor4 + halfRobotWidthy) + (0.5)*(96 - halfRobotWidthy - sensor2);

  if((sensor1+sensor3+2*halfRobotWidthx > 110 ) || (sensor1+sensor3+2*halfRobotWidthx < 80 ) && (sensor2+sensor4+2*halfRobotWidthy < 110)||(sensor2+sensor4+2*halfRobotWidthy < 80))
    {
      Serial.print("sensor 1 + 3 ");
      Serial.println(sensor1+sensor3+(2*halfRobotWidthx));
      Serial.print("sensor 2 + 4 ");
      Serial.println(sensor2+sensor4+(2*halfRobotWidthy));
      Serial.print("Sensor 1: ");
      Serial.println(sensor1);
      Serial.print("Sensor 2: ");
      Serial.println(sensor2);
      Serial.print("Sensor 3: ");
      Serial.println(sensor3);
      Serial.print("Sensor 4 ");
      Serial.println(sensor4);


      return {prevLocation.x, prevLocation.y};
      Serial.println("We are driving Blind!!!");
    }
    else if((sensor1+sensor3+2*halfRobotWidthx > 110 ) || (sensor1+sensor3+2*halfRobotWidthx < 80 )){
      Serial.print("sensor 1 + 3 ");
      Serial.println(sensor1+sensor3+(2*halfRobotWidthx));
      Serial.print("Sensor 1: ");
      Serial.println(sensor1);
      Serial.print("Sensor 3: ");
      Serial.println(sensor3);

      prevLocation.y=y;
      currLocation={prevLocation.x,y};
      return {prevLocation.x, y};
    }
    else if((sensor2+sensor4+2*halfRobotWidthy < 110)||(sensor2+sensor4+2*halfRobotWidthy < 80)){
      Serial.print("sensor 2 + 4 ");
      Serial.println(sensor2+sensor4+(2*halfRobotWidthy));
      Serial.print("Sensor 2: ");
      Serial.println(sensor2);
      Serial.print("Sensor 4 ");
      Serial.println(sensor4);

      prevLocation.x=x;
      currLocation={x,prevLocation.y};
      return{x,prevLocation.y};

    }

    else{
      prevLocation.x = x;
      prevLocation.y = y;
      currLocation = {x,y};
      return {x,y};

    }

  // Serial.println("running");
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

// IMPLEMENT
// get Yaw from IMU

double q1, q2, q3, q0, q2sqr, t0, t1, roll, t2, pitch, t3, t4, yaw;

double getYaw(){
    // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) Serial.print( "0" );
    //if ( data.header < 0x10) Serial.print( "0" );
    //Serial.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      /*
      Serial.print(F("Q1:"));
      Serial.print(q1, 3);
      Serial.print(F(" Q2:"));
      Serial.print(q2, 3);
      Serial.print(F(" Q3:"));
      Serial.println(q3, 3);
*/

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      q2sqr = q2 * q2;

      // roll (x-axis rotation)
      t0 = +2.0 * (q0 * q1 + q2 * q3);
      t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      t3 = +2.0 * (q0 * q3 + q1 * q2);
      t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      yaw = atan2(t3, t4) * 180.0 / PI;

      // Serial.print(F("Roll:"));
      // Serial.print(roll, 1);
      // Serial.print(F(" Pitch:"));
      // Serial.print(pitch, 1);
      // Serial.print(F(" Yaw:"));
      // Serial.println(yaw, 1);

      
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }


  // if(abs(yaw) == 90.0){
  //   return 0;
  // }
  return yaw;
}


// Coil locations in inches
// additions/subtractions are for the offset of the robot
// x-coil distance to center: 4.53
// y-coil distance to center: 5.9
const Coordinates coilA = {32.0, 0.0 + 8.0};
const Coordinates coilB = {64.0, 0.0 + 8.0};
const Coordinates coilC = {96.0 - 6.5, 32.0};
const Coordinates coilD = {96.0 - 6.5, 64.0};
const Coordinates coilE = {64.0, 96.0 - 8.0};
const Coordinates coilF = {32.0, 96.0 - 8.0};
const Coordinates coilG = {0.0 + 6.5, 64.0};
const Coordinates coilH = {0.0 + 6.5, 32.0};

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

State currentState = stateB; // initialize the starting location of the robot 
Coordinates previousCoordnates = coilB; //innitalize with starting location of coil
int loopCount = 0;
float initialYaw = 0.0; // inital yaw on startup

Coordinates destination;
int ledStatus = 1;

int pwm1;
int pwm2;
int pwm3;
int pwm4;

int remoteCount = 0;


float kx = 8;//14
float ky = 2;//3
float ktheta = 1;

void loop(){
  
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

  currLocation = getCoordinates();
  double errorx = destination.x - currLocation.x;
  double errory = destination.y - currLocation.y;
  double erroryaw = initialYaw - getYaw();

//   //  while(abs(errory) >= 3){
//   //   robot.yMovement(ky*errory);
//   //   Serial.print("Errory: ");
//   //   Serial.println(errory);
//   // }


//   // while(abs(errorx) >= 3){
//   //   robot.xMovement(kx*errorx);
//   //   Serial.print("Errorx: ");
//   //   Serial.println(errorx);
//   // }

 

// // Serial.print("errorx: ");
// // Serial.println(errorx);
// // Serial.print("errory: ");
// // Serial.println(errory);
// // Serial.print("erroryaw");
// // Serial.println(erroryaw);


// // "P" loop for motor control
// // TODO: Add tolerance (error will never actually be 0)


// // while(abs(errorx) > 1.0 || abs(errory) > 1.0 || abs(erroryaw) > 1.0){

while(true){     
  while(remoteCount % 2 == 0){
    getCoordinates();
    Serial.print("Waiting for start.\r");
    // Serial.print("X-Coordinate: ");
    // Serial.println(currLocation.x);
    // Serial.print("Y-coordinate: ");
    // Serial.println(currLocation.y);
    robot.stopMotors();
    
    if(Serial2.available() > 0){
      remoteCount += 1;
      Serial.println(Serial2.readString());
      Serial.println("Starting movement");
      
    }
 }

  if(Serial2.available() > 0){
    robot.stopMotors();
    remoteCount += 1;
    Serial.println(Serial2.readString());
    
  }
  
  if(ledStatus == 1){
    digitalWrite(led,LOW);
    ledStatus = 0;
  }
  else{
    digitalWrite(led, HIGH);
    ledStatus = 1;
  }

float kx = 0;
float ky = 0;
float kyaw = 1;

  pwm1 = int((kx*errorx - ky*errory - kyaw*erroryaw)/(96*(kx + ky) + 180*(kyaw))*255); 
  pwm2 = int((kx*errorx + ky*errory + kyaw*erroryaw)/(96*(kx + ky) + 180*(kyaw))*255); 
  pwm3 = int((kx*errorx + ky*errory - kyaw*erroryaw)/(96*(kx + ky) + 180*(kyaw))*255); 
  pwm4 = int((kx*errorx - ky*errory + kyaw*erroryaw)/(96*(kx + ky) + 180*(kyaw))*255); 


  Serial.print("PWM1: ");
  Serial.println(pwm1);
  Serial.print("PWM2: ");
  Serial.println(pwm2);
  Serial.print("PWM3: ");
  Serial.println(pwm3);
  Serial.print("PWM4: ");
  Serial.println(pwm4);

  
  robot.M1(pwm1);
  robot.M2(pwm2);
  robot.M3(pwm3);
  robot.M4(pwm4);




  currLocation = getCoordinates();
  errorx = destination.x - currLocation.x;
  errory = destination.y - currLocation.y;
  erroryaw = initialYaw - getYaw();

  Serial.print("ErrorX: ");
  Serial.println(errory);
  Serial.print("ErrorY: ");
  Serial.println(errory);
  Serial.print("ErrorYaw: ");
  Serial.println(erroryaw);
  Serial.print("X-coordinate: ");
  Serial.println(currLocation.x);
  Serial.print("Y-coordinate: ");
  Serial.println(currLocation.y);
  Serial.print("Yaw: ");
  Serial.println(Serial1.r);

  


}







// OPTION 2--------------------------------------------------------------------------
// dummy loop

// TODO: Change while condition to utilize xMovement and yMovement in driver file
//        - implement IMU


// switch (currentState){
//   case stateA:
//     while(coilD.y - getCoordinates().y > 10.0){
//       // drive forward
//       Serial.println("Driving Forward");
//       robot.yMovement(int(ky*(coilD.y - getCoordinates().y)));
//     }

//     while(coilD.x - getCoordinates().x > 10.0){
//       // drive right
//       Serial.println("Driving Right");
//       robot.xMovement(int(kx*(coilD.x - getCoordinates().x)));
//     }
//     // Serial.print("Arrived at ");
//     // Serial.println(currentState);
//     currentState = stateD;
//     break;
//   case stateB: 
//     while(coilG.y - getCoordinates().y > 10.0){
//       // drive forward
//       Serial.println("Driving forward");
//       robot.yMovement(int(ky*(coilG.y - getCoordinates().y)));
//     }

//     while(coilG.x - getCoordinates().x > 10.0){
//       // drive left
//       Serial.println("Driving left");
//       robot.xMovement(int(kx*(coilG.x - getCoordinates().x)));
//     }
//     // Serial.print("Arrived at: ");
//     // Serial.println(currentState);
//     currentState = stateG;
//     break;
//   case stateC: 
//     while(coilA.x - getCoordinates().x > 10.0){
//       // drive left
//       Serial.println("Driving left");
//       robot.xMovement(int(kx*(coilA.x - getCoordinates().x)));
//     }
//     while(coilA.y - getCoordinates().y > 10.0){
//       // drive backward
//       Serial.println("Driving Backward");
//       robot.yMovement(int(ky*(coilA.y - getCoordinates().y)));
//     }
//     currentState = stateA;
//     break;

//   case stateD: 
//     while(coilH.x - getCoordinates().x > 10.0){
//       // drive left
//       Serial.println("Driving left");
//       robot.xMovement(int(kx*(coilH.x - getCoordinates().x)));
//     }
//     while(coilH.y - getCoordinates().y > 10.0){
//       //drive backward
//       Serial.println("Driving backward");
//       robot.yMovement(int(ky*(coilH.x - getCoordinates().x)));
//     }
//     currentState = stateH;
//     break;

//   case stateE: 
//     while(coilC.x - getCoordinates().x > 10.0){
//       // drive backward
//       Serial.println("Driving backward");
//       robot.xMovement(int(kx*(coilH.x - getCoordinates().x)));
//     }
//     while(coilC.y - getCoordinates().y > 10.0){
//       // drive right
//       Serial.println("Driving Right");
//       robot.yMovement(int(ky*(coilH.x - getCoordinates().x)));
//     }
//     currentState = stateC;
//     break;

//   case stateF:
//     while(coilB.y - getCoordinates().y > 10.0){
//       // drive backward
//       Serial.println("Driving backward");
//       robot.yMovement(int(ky*(coilH.x - getCoordinates().x)));

//     } 
//     while(coilB.x - getCoordinates().x > 10.0){
//       // drive right
//       Serial.println("Driving Right");
//       robot.xMovement(int(kx*(coilH.x - getCoordinates().x)));

//     }
//     currentState = stateB;
//     break;

//   case stateG:
//     while(coilE.x - getCoordinates().x > 10.0){
//       // drive right
//       Serial.println("Driving Right");
//       robot.xMovement(int(kx*(coilH.x - getCoordinates().x)));

//     }
//     while(coilE.y - getCoordinates().y > 10.0){
//       // drive forward
//       Serial.println("Driving forward");
//       robot.yMovement(int(ky*(coilH.x - getCoordinates().x)));
//     }
//     currentState = stateE;
//     break;

//   case stateH: 
//     while(coilF.x - getCoordinates().x > 10.0){
//       // drive right
//       Serial.println("Driving right");
//       robot.xMovement(int(kx*(coilH.x - getCoordinates().x)));
//     }
//     while(coilF.y - getCoordinates().y > 10.0){
//       // drive forward
//       Serial.println("Driving forward");
//       robot.yMovement(int(ky*(coilH.x - getCoordinates().x)));
//     }
//     break;
//     currentState = stateF;


// }

loopCount += 1;



}

