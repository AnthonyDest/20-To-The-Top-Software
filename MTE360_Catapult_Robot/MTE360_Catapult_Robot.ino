#include "TB6612_Motor.h"
// Limited functionality, only 4/6 combos due to pins

// Pins for all inputs (NOT CONFIGURED YET)
#define DIR1_M1 10// LEFT
#define DIR1_M2 11 // RIGHT

#define PWM_M1 9// LEFT
#define PWM_M2 8// RIGHT

// Pins for encoder
// Digital pins
#define ENCA_M1 6
#define ENCB_M1 5
// Analog pins to be converted to digital (doesn't auto convert on nano)
#define ENCA_M2 21
#define ENCB_M2 22

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1

//used to flip direction so forward function spins each wheel the correct direction (based on wiring and orientation)
// const bool DirectionInvertLeft = TRUE;
// const bool DirectionInvertRight = TRUE;
const int DirectionInvertLeft = 1;
const int DirectionInvertRight = -1;

// Initializing
Motor leftMotor = Motor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor = Motor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder = Encoder(ENCA_M1,ENCB_M1); // digital pin
Encoder rightEncoder = Encoder(ENCA_M2,ENCB_M2); // analog pin, dont think digital read works on nanos - will need testing
//add encoder here and put in robot object

Robot robot = Robot(leftMotor, rightMotor, leftEncoder, rightEncoder);

void setup() {
  // need to clock down based on Lana's undervolting (change 9600)
  Serial.begin(9600);
}


void loop() {
  robot.forwardDrive(100);
  delay(3000);

  robot.brake();
  delay(3000);

  robot.backwardDrive(100);
  delay(3000);

  robot.brake();
  delay(3000);

  robot.rightTurnStationary(100);
  delay(3000);

  robot.brake();
  delay(3000);

  robot.leftTurnStationary(100);
  delay(3000);

  robot.brake();
  delay(3000);

  robot.forwardDriveDistance(200,60);
  delay(3000);

  robot.brake();
  delay(3000);

  robot.reverseDriveDistance(200,60);
  delay(3000);

  robot.brake();
  delay(3000);

}