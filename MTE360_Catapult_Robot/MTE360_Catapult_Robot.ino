#include "Robot.h"
#include <Arduino.h>

#define DIR1_M1 7  // LEFT
#define DIR1_M2 8  // RIGHT

#define PWM_M1 6  // LEFT
#define PWM_M2 9  // RIGHT

// Pins for encoder
// Digital pins
#define ENCA_M1 3
#define ENCB_M1 2

// Analog pins to be converted to digital (doesn't auto convert on nano)
#define ENCA_M2 A2
#define ENCB_M2 A3

const int DirectionInvertLeft = -1;
const int DirectionInvertRight = -1;
int state = 0;

Motor leftMotor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);    // digital pin
Encoder rightEncoder(ENCA_M2, ENCB_M2, DirectionInvertRight);  // analog pin, dont think digital read works on nanos - will need testing

Robot motorOnlyBot(leftMotor, rightMotor);
//  Robot bobBot(leftMotor, rightMotor, leftEncoder, rightEncoder);

void setup() {

  Serial.begin(9600);
}

void loop() {

  if (state == 0) {
    motorOnlyBot.forwardDrive(100);
    delay(5000);
    motorOnlyBot.brake();
    state++;
  }

  if (state == 1) {
    motorOnlyBot.backwardDrive(100);
    delay(5000);
    motorOnlyBot.brake();
    state++;
  }

  if (state == 2) {
    motorOnlyBot.leftTurnStationary(50);
    delay(5000);
    motorOnlyBot.brake();
    state++;
  }

  if (state == 3) {
    motorOnlyBot.rightTurnStationary(50);
    delay(5000);
    motorOnlyBot.brake();
    state++;
  }

}
