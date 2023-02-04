#include "TB6612_Motor.h"

// Pins for all inputs (Correct? but not finalized - check with Lana)
// #define AIN1 10
// #define BIN1 11
// #define AIN2 
// #define BIN2 
// #define PWMA 9
// #define PWMB 8
// #define STBY 

// Pins for all inputs (NOT CONFIGURED YET)
#define AIN1 2
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1

//used to flip direction so forward function spins each wheel the correct direction (based on wiring and orientation)
// const bool DirectionInvertLeft = TRUE;
// const bool DirectionInvertRight = TRUE;
const int DirectionInvertLeft = 1;
const int DirectionInvertRight = -1;

// Initializing motors
Motor leftMotor = Motor(AIN1, AIN2, PWMA, DirectionInvertLeft, STBY);
Motor rightMotor = Motor(BIN1, BIN2, PWMB, DirectionInvertRight, STBY);

Robot robot = Robot(leftMotor, rightMotor);

void setup() {
  //Nothing needed here for motors
}


void loop() {
  robot.forwardDrive(100);
  delay(3000);

  robot.brake(100);
  delay(3000);

  robot.backwardDrive(100)
  delay(3000);

  robot.brake(100);
  delay(3000);

  robot.rightTurn(100)
  delay(3000);

  robot.brake(100);
  delay(3000);

  robot.leftTurn(100)
  delay(3000);

  robot.brake(100);
  delay(3000);

  robot.idle();
  delay(3000);

}