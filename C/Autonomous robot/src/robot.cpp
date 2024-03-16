#include <VL53L1X.h>
#include <Adafruit_BNO08x.h>



//BNO08X
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

//VL53L1X Sensors
const int sensorCount = 4;


// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPins[sensorCount] = { 10,11,12,13};

// Create sensors object
VL53L1X sensors[sensorCount];


// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game vector");
  }
}


void setup(){
    Serial.begin(115200);
    while (!Serial) delay(10);
// bno08x----------------------------------------------------------

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();
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





void loop(){

//BNO08X----------------------------------------
if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }


 if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }


switch (sensorValue.sensorId) {

case SH2_GAME_ROTATION_VECTOR:
    Serial.print("Game Rotation Vector - r: ");
    Serial.print(sensorValue.un.gameRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.gameRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.gameRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.gameRotationVector.k);
    break;
}


Serial.print("Sensor 1: ");
Serial.print(sensors[0].read());
Serial.print("\t");
Serial.print("Sensor 2: ");
Serial.print(sensors[1].read());
Serial.print("\t");
Serial.print("Sensor 3: ");
Serial.print(sensors[2].read());
Serial.print("\t");
Serial.print("Sensor 4: ");
Serial.print(sensors[3].read());
Serial.println();




}
