#include "TB6612_Motor.h"
#include <SPI.h>
// #include <SD.h>

// // Limited functionality, only 4/6 combos due to pins
// // Pins for all inputs (NOT CONFIGURED YET)
#define DIR1_M1 7  // LEFT 10
#define DIR1_M2 8  // RIGHT 11
#define PWM_M1 6   // LEFT 9
#define PWM_M2 9   // RIGHT 12

// // Pins for encoder
// // Digital pins
//left currently
#define ENCA_M1 3
#define ENCB_M1 2
// // Analog pins to be converted to digital (might not auto convert on nano), try A0 to A5
//Right Currently
#define ENCA_M2 A2
#define ENCB_M2 A3

// //used to flip direction so forward function spins each wheel the correct direction (based on wiring and orientation)
const int DirectionInvertLeft = 1;
const int DirectionInvertRight = 1;

// // change this to match the SD shield or module;
// const int chipSelect = 10;
// File dataLog;

// // Initializing
Motor leftMotor = Motor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor = Motor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder = Encoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);    // digital pin
Encoder rightEncoder = Encoder(ENCA_M2, ENCB_M2, DirectionInvertRight);  // analog pin, dont think digital read works on nanos - will need testing

// Robot robot = Motor(leftMotor, rightMotor, leftEncoder, rightEncoder);

// // Will add sd card file as parameter later so we can log anytime a command was run in all other functions

void setup() {
  //     if (!SD.begin()) { // can put in slave pin as parameter
  //     Serial.println("SD CARD FAILED, OR NOT PRESENT!");
  //   }
  //   Serial.println("SD CARD initialization sucessful");

  //   // have each save be unique based on current time so we can track multipule results at a time?
  //  dataLog = SD.open("dataLog.txt", FILE_WRITE);

  // This code can be moved to loop, dont want to put now and continously write to file until we test a bit more
  // // if the file opened okay, write to it:
  //  if (dataLog) {
  //     dataLog.println("testing 1, 2, 3.");
  //     // close the file:
  //     dataLog.close();
  //   } else {
  //     // if the file didn't open, print an error:
  //     Serial.println("error opening file");
  //   }

  Serial.begin(9600);
  // leftMotor.speedUp(50, 0, 1);
  // rightMotor.fwd(100);

  // leftEncoder.resetCounter();
  // rightEncoder.resetCounter();
  // Serial.println("RESET");
  // Serial.println("aa" + (unsigned long)&leftMotor);


  // Serial.println("LEFT IN COIDE: " + (unsigned long)&leftMotor);
  //  Serial.println("LEFT IN BOT: " + (unsigned long)&robot.leftMotor);
}


void loop() {

  // leftEncoder.EncScanActive();
  // rightEncoder.EncScanActive();

  //  leftMotor.rev(200);
  // rightMotor.rev(200);

  // delay(2000);

  //  leftMotor.rev(0);
  //  rightMotor.rev(0);

  //  delay(50000);

}
