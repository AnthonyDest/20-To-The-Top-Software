#include "Robot.h"
#include "Encoder.h"
#include "Rotary.h"
#include "ICM_20948.h"
#include <Arduino.h>

#define DIR1_M1 6   // LEFT
#define DIR1_M2 11  // RIGHT

#define PWM_M1 5   // LEFT
#define PWM_M2 10  // RIGHT

// Pins for encoder
// Digital pins
#define ENCA_M1 A4
#define ENCB_M1 A5
#define ENCA_M2 A3
#define ENCB_M2 A2

#define WIRE_PORT Wire
#define AD0_VAL 86

ICM_20948_I2C myICM;
float averageGyroX = 0;
float angle = 0;
int tempCounter = 0;

const int DirectionInvertLeft = -1;
const int DirectionInvertRight = -1;
int state = 0;
int tempSpeed = 50;
int counter = 0;
unsigned long currentMillis;
unsigned long previousMillis = 0UL;
Motor leftMotor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);    // digital pin
Encoder rightEncoder(ENCA_M2, ENCB_M2, DirectionInvertRight);  // analog pin, dont think digital read works on nanos - will need testing
                                                               // Rotary rotary = Rotary(ENCA_M1, ENCB_M1);

Robot motorOnlyBot(leftMotor, rightMotor);
// Robot bobBot(leftMotor, rightMotor, leftEncoder, rightEncoder);

void setup() {

  Serial.begin(57600);
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  myICM.begin(WIRE_PORT, AD0_VAL);

  attachInterrupt(ENCA_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCB_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCA_M2, updateRightEncoder, CHANGE);
  attachInterrupt(ENCB_M2, updateRightEncoder, CHANGE);
}

void updateLeftEncoder() {
  leftEncoder.EncScanActive();
}
void updateRightEncoder() {
  rightEncoder.EncScanActive();
}

void loop() {

  // //Testing ICM, will incorperate into proper library once implementation finalized
  // myICM.getAGMT();
  // // Serial.println(myICM.gyrX());
  // // Serial.println(myICM.gyrY());
  // // Serial.println(myICM.gyrZ());

  // //get average to debounce
  // // Serial.println("BB");
  // currentMillis = millis();
  // if (currentMillis - previousMillis < 200UL) {
  //   tempCounter++;
  //   averageGyroX += myICM.gyrX();
  // } else {

  //   averageGyroX /= tempCounter;

  //   if (abs(averageGyroX) > 0.70) {
  //     angle += averageGyroX;
  //   }

  //   tempCounter = 0;
  //   averageGyroX = 0;
  //   previousMillis = currentMillis;
  // }
  // Serial.println(angle);
}
