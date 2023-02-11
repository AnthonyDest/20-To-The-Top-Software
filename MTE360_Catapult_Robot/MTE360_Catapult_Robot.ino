#include "TB6612_Motor.h"
#include <SPI.h>
#include <SD.h>
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

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1

//used to flip direction so forward function spins each wheel the correct direction (based on wiring and orientation)
// const bool DirectionInvertLeft = TRUE;
// const bool DirectionInvertRight = TRUE;
const int DirectionInvertLeft = 1;
const int DirectionInvertRight = -1;

// change this to match the SD shield or module;
const int chipSelect = 10;
File dataLog;

// Initializing
Motor leftMotor = Motor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor = Motor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder = Encoder(ENCA_M1,ENCB_M1); // digital pin
Encoder rightEncoder = Encoder(ENCA_M2,ENCB_M2); // analog pin, dont think digital read works on nanos - will need testing

// Will add sd card file as parameter later so we can log anytime a command was run in all other functions

Robot robot = Robot(leftMotor, rightMotor, leftEncoder, rightEncoder);

void setup() {
  // need to clock down based on Lana's undervolting (change 9600)

    if (!SD.begin()) { // can put in slave pin as parameter
    Serial.println("SD CARD FAILED, OR NOT PRESENT!");
  }
  Serial.println("SD CARD initialization sucessful");

  // have each save be unique based on current time so we can track multipule results at a time?
 dataLog = SD.open("dataLog.txt", FILE_WRITE);

// This code can be moved to loop, dont want to put now and continously write to file until we test a bit more
// if the file opened okay, write to it:
 if (dataLog) {
    dataLog.println("testing 1, 2, 3.");
    // close the file:
    dataLog.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }

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