#include "Robot.h"
#include "Encoder.h"
#include "Rotary.h"
#include "TOFSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "Adafruit_VL53L0X.h"

#define DIR1_M1 11   // LEFT
#define DIR1_M2 6  // RIGHT
#define PWM_M1 12   // LEFT
#define PWM_M2 10  // RIGHT

// Pins for encoder
#define ENCA_M1 A4
#define ENCB_M1 A5
#define ENCA_M2 A3
#define ENCB_M2 A2

#define AD0_VAL 86 // gyro
#define BOT_TOF_RESET_PIN 5 //reset

const int DirectionInvertLeft = -1;
const int DirectionInvertRight = -1;
int state = 0; //get enumerations later

Motor leftMotor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);    
Encoder rightEncoder(ENCA_M2, ENCB_M2, DirectionInvertRight);
TOFSensor botTOF;
TOFSensor topTOF;

int temp = 0; 
int dist = 0;                                     
// Robot mBot(leftMotor, rightMotor);
// Robot meBot(leftMotor, rightMotor, leftEncoder, rightEncoder);
Robot mesBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF);
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("\nSTART---------------------------");

  attachInterrupt(ENCA_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCB_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCA_M2, updateRightEncoder, CHANGE);
  attachInterrupt(ENCB_M2, updateRightEncoder, CHANGE);

  mesBot.allConfiguration();
}

void updateLeftEncoder() {
  leftEncoder.EncScanActive();
}
void updateRightEncoder() {
  rightEncoder.EncScanActive();
}

void loop() {   //state machine
//  Serial.println("TIME: " + String(millis()));

  if (state == 5){
    Serial.println("Getting ready");
    delay(500); //change delay to a "wait"
    state++;
  }

  if (state == 1){
    Serial.println("Searching for pole");
    mesBot.searchForPole();
    delay(2000);
    state++;
  }

  if (state == 2){
    Serial.println("Driving to pole, distance of [CM]: " + String(mesBot.linearDistToPole));
    delay(500);
    mesBot.forwardDriveDistance(100,mesBot.linearDistToPole);
    delay(500);
    state++;
  }

  if (state == 3){
  }

 delay(500);

//testing enc steps
// Serial.println(leftEncoder.stepCounter);
// temp = leftEncoder.stepCounter;
// dist = 3000; //600~ 1 turn, max ~8 turns = 2m = 4800, max max = 10 turns = 6000 (testing )
// mesBot.forwardDriveDistance(50,dist);
// Serial.println(leftEncoder.stepCounter - temp - dist);
// delay(5000);

}
