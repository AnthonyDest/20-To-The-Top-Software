#include "Robot.h"
#include "Encoder.h"
#include "Rotary.h"
#include "TOFSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "Adafruit_VL53L0X.h"

//---------------------------------
#define SHT_LOX2 5
#define scanDuration 20
#define LOX1_ADDRESS 0x30
// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X(); //1 is Top
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X(); //2 is Bot

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
uint16_t botDistance = 0;
uint16_t topDistance = 0;
//---------------------------------


#define DIR1_M1 11   // LEFT
#define DIR1_M2 6  // RIGHT

#define PWM_M1 12   // LEFT
#define PWM_M2 10  // RIGHT

// Pins for encoder
#define ENCA_M1 A4
#define ENCB_M1 A5
#define ENCA_M2 A3
#define ENCB_M2 A2

//  Gyro pins
#define AD0_VAL 86

// VL53L0X_RangingMeasurementData_t measureBot;
// VL53L0X_RangingMeasurementData_t measureTop;

//TOF START---------------

// address we will assign if dual sensor is present
//Bot addr stays the same (is reset)
// #define TOP_TOF_ADDRESS 0x31 //(top addr changes)

// set the pins to shutdown
// #define SHT_LOX1 13
#define BOT_TOF_RESET_PIN 5

// objects for the vl53l0x
// Adafruit_VL53L0X botTOFLib = Adafruit_VL53L0X();
// Adafruit_VL53L0X topTOFLib = Adafruit_VL53L0X();

//TOF END---------------


ICM_20948_I2C myICM;
float averageGyroX = 0;
float angle = 0;
int tempCounter = 0;

const int DirectionInvertLeft = -1;
const int DirectionInvertRight = -1;
int state = 0; //get enumerations later
int tempSpeed = 50;
int counter = 0;
unsigned long currentMillis;
unsigned long previousMillis = 0UL;
Motor leftMotor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);    // digital pin
Encoder rightEncoder(ENCA_M2, ENCB_M2, DirectionInvertRight);  // analog pin, dont think digital read works on nanos - will need testing
                                                               // Rotary rotary = Rotary(ENCA_M1, ENCB_M1);
// TOFSensor botTOF(botTOFLib, "BOT", measureBot);
// TOFSensor topTOF(topTOFLib, "TOP", measureTop);
// TOFSensor botTOF(botTOFLib, "BOT");
// TOFSensor topTOF(topTOFLib, "TOP");
// TOFSensor botTOF(botTOFLib,"BOT", measureBot);
// TOFSensor topTOF(topTOFLib, "TOP", measureTop);

// Robot motorOnlyBot(leftMotor, rightMotor);
 Robot meBot(leftMotor, rightMotor, leftEncoder, rightEncoder);
//  Robot mesBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF);
// VL53L0X sensor;
void setup() {

  Serial.begin(57600);
  // Serial.begin(9600);
   while (! Serial) { delay(1); }
  Serial.println("\nSTART---------------------------");
  // Wire.begin();
  // Wire.setClock(400000);
  // myICM.begin(Wire, AD0_VAL);


 pinMode(SHT_LOX2, OUTPUT);
  Serial.println(F("Shutdown pins inited..."));
  digitalWrite(SHT_LOX2, LOW);

  attachInterrupt(ENCA_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCB_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCA_M2, updateRightEncoder, CHANGE);
  attachInterrupt(ENCB_M2, updateRightEncoder, CHANGE);

Serial.println("START");
setID();
  // topTOF.sensor->setTimeout(200);
  //   // topTOF.sensor->init();
  // if (!topTOF.sensor->init())
  // {
  //   Serial.println("Failed to detect and initialize sensor!");
  //   delay(2000);
  //   while (1) {}
  // } else {
  //   Serial.println("Init good");
  //   delay(2000);
  // }

  // Serial.println(topTOF.sensor->getAddress());


  // topSensor.setAddress(0b0101011);

  // botTOF.resetOn();

  // //init top with address
  // topTOF.initTOF(0x31);

  // //reset high
  // botTOF.resetOff();

  // //init bot no addr
  // botTOF.initTOF();
 

  // mesBot.allConfiguration();
  // mesBot.Setup_TOF_Address();
  // topTOF.startContinuous();
  // botTOF.startContinuous();
}

void updateLeftEncoder() {
  leftEncoder.EncScanActive();
}
void updateRightEncoder() {
  rightEncoder.EncScanActive();
}

uint16_t botScanDistance(){ // needs debouncing -ACTUALLY 
lox1.rangingTest(&measure1, false);
// Serial.println(measure1.RangeMilliMeter);
  if(measure1.RangeStatus != 4) {     // if not out of range
    return measure1.RangeMilliMeter; // Use trig here
  }
  return 0;
} 

uint16_t topScanDistance(){ // needs debouncing
lox2.rangingTest(&measure2, false);
// Serial.println(measure2.RangeMilliMeter);
  if(measure2.RangeStatus != 4) {     // if not out of range
    return measure2.RangeMilliMeter; // Use trig here
  }
  return 0;
} 

void loop() {   //state machine
 Serial.println("TIME: " + String(millis()));

// Serial.println("BLACK :" + String(botScanDistance()));
// Serial.println("RED :" + String(topScanDistance()));
// delay(200);
// Serial.println();
// delay(300);


// meBot.forwardDrive(100);
Serial.println("SCANNING FOR POLE");
searchForPole();

Serial.println("DRIVINIG TO POLE");
driveToPole();

Serial.println("DONE");
delay(500);
 delay(10000);
}


// lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
//   lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

//   // print sensor one reading
//   Serial.print(F("1: "));
//   if(measure1.RangeStatus != 4) {     // if not out of range
//     Serial.print("ONE = " + String(measure1.RangeMilliMeter));
//   } else {
//     Serial.print(F("Out of range"));
//   }
  
//   Serial.print(F(" "));

//   // print sensor two reading
//   Serial.print(F("2: "));
//   if(measure2.RangeStatus != 4) {
//     Serial.print("TWO =" + String(measure2.RangeMilliMeter));
//   } else {
//     Serial.print(F("Out of range"));
//   }
  
//   Serial.println();
// delay(1000);
// }

// delay(100);
// Serial.print("ONE: " + String(botTOF.scanDistance()));
// delay(100);
// Serial.println("      TWO " + String(topTOF.scanDistance()));

// botTOF.scanDistanceGET();
// topTOF.scanDistanceGET();
// delay(100);
// Serial.println("TOP: " + topTOF.scanDistance());
// Serial.println("BOT: " + botTOF.scanDistance());

// // delay(200);
// botTOFLib.rangingTest(&measureBot, false);
// // delay(200);
// topTOFLib.rangingTest(&measureTop, false);

// // delay(200);
// if (measureBot.RangeStatus != 4) {  // if not out of range
//     Serial.println("BOT: " + String(measureBot.RangeMilliMeter));
//   } else {
//     Serial.println(F("BOT Out of range"));
//   }
// // delay(200);
// if (measureTop.RangeStatus != 4) {  // if not out of range
//     Serial.println("TOP: " + String(measureTop.RangeMilliMeter));
//   } else {
//     Serial.println(F("TOP Out of range"));
//   }
// Serial.println();
// delay(1000);

// }

bool poleFound(){

  botDistance = botScanDistance();
  topDistance = topScanDistance();
  Serial.println("BOT: " + String(botDistance));
  delay(200);
  Serial.println("TOP: " + String(topDistance));

  if(botDistance >= topDistance*0.7 && botDistance <= topDistance*1.3){ // If both values are within tolerance of eachother, pole found
    return true;
  }

  return false;
}

void searchForPole(){

int scanCounterMax = 72; //360 rotation
int scanCounter = 0;
   while(!poleFound() and scanCounter < scanCounterMax){
     Serial.println("Turning to scan " + scanCounter);
     meBot.leftTurnStationaryUsingEncoder(300);
    scanCounter++;
    delay(200);
   }
}

void driveToPole(){

  //validate pole is infront of you, else keep scanning (redundant?)
  Serial.println("Driving to pole");
  meBot.forwardDriveDistance(100, float((botDistance+topDistance)/2));
  Serial.println("Stop Drive");
}

void setID() {
  digitalWrite(SHT_LOX2, LOW);
  delay(100);

//initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    // while(1);
  } else{
     Serial.println(F("ONE Good"));
  }

  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin()) {
    Serial.println(F("Failed to boot second VL53L0X"));
    // while(1);
  } else{
     Serial.println(F("Two Good"));
  }
    delay(2000);
}