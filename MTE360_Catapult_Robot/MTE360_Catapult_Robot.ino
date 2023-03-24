#include "Robot.h"
#include "Encoder.h"
#include "TOFSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"

#define DIR1_M1 12   // LEFT
#define DIR1_M2 6  // RIGHT
#define PWM_M1 11   // LEFT
#define PWM_M2 10  // RIGHT

// Pins for encoder
#define ENCA_M1 A4
#define ENCB_M1 A5
#define ENCA_M2 A3
#define ENCB_M2 A2


#define AD0_VAL 86 // gyro
#define BOT_TOF_RESET_PIN 5 //reset

const int STATE_END = 5;

const int DirectionInvertLeft = 1;
const int DirectionInvertRight = 1;
int state = 0; //get enumerations later

Motor leftMotor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);    
Encoder rightEncoder(ENCA_M2, ENCB_M2, DirectionInvertRight);
TOFSensor botTOF;
TOFSensor topTOF;
Gyro gyro;

int temp = 0; 
int dist = 0;                                     
// Robot mBot(leftMotor, rightMotor);
//  Robot meBot(leftMotor, rightMotor, leftEncoder, rightEncoder);
// Robot mesBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF);
Robot mesgBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF, gyro);

void setup() {
  Serial.begin(115200);
  //  while (!Serial) {}
   Serial.println("\nSTART---------------------------");

  attachInterrupt(ENCA_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCB_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCA_M2, updateRightEncoder, CHANGE);
  attachInterrupt(ENCB_M2, updateRightEncoder, CHANGE);
  
  mesgBot.allConfiguration();
  // botTOF.initalizeTOF();

  state = 0;
  Serial.println("START");
  // rightMotor.drive(255, 1);

  // delay(500);
  //     leftMotor.drive(255, 1);
  // gyro.setup();
  delay(100); // replace with wait to stabalize gyro ? 
}

void updateLeftEncoder() {
  leftEncoder.EncScanActive();
}
void updateRightEncoder() {
  rightEncoder.EncScanActive();
}

void loop() {   //state machine
    //  Serial.println("TIME: " + String(millis()));
    //State 10: Drive Fwd
    //State 20: Drive fwd to enc distance
    //State 30: Scan/rotate and drive to enc distance
    //State 40: Rotate 90 deg by gyro
    //State 50: Drive straight by gyro
    //State 60: Rotate to heading (go back to 0 deg)

// Serial.println(botTOF.scanDistanceMM());
// delay(100);

if(state == 0){
  state = 55;
  blink(10, 100);
  // delay(5000);
  blink(3, 500);
}

if (state == STATE_END){
  // Serial.println("STARTING");
blink(20,100);
  delay(1000);
  state++;
}
// state = 5;
// if(state ==5){
  
//   // mesgBot.scanBothTOF();
//   Serial.println(botTOF.scanDistanceMM());
//   // Serial.println(mesgBot.scanDistanceBotAverage);
// }


if(state == 10){ //Drive Fwd
Serial.println("Drive Start");
  mesgBot.forwardDrive(100);
  

  // rightMotor.fwd(100);
  // leftMotor.fwd(100);
  delay(5000);
  Serial.println("Drive End");
  mesgBot.brake();
  delay(2000);
  state = STATE_END;
}

if(state == 20){ //drive forward PID
  mesgBot.forwardDrivePID(3000);
  state = STATE_END;
}

if(state == 30){ //State 30: Scan/rotate and drive to enc distance
  Serial.println("Start 30");
  mesgBot.searchForPole();

  // mesgBot.forwardDrivePID(mesgBot.linearDistToPole);
  mesgBot.driveForwardAtCurrentHeading(mesgBot.linearDistToPole);

  // mesgBot.rightTurnStationaryPID(200);

  // mesgBot.searchForPole();

  // mesgBot.driveForwardAtCurrentHeading(mesgBot.linearDistToPole);

  state = STATE_END;
}


if(state == 40){ //State 40: Rotate 90 deg by gyro
  
  mesgBot.turnDeltaAngleGyro(90,BACKWARD_DIR, FORWARD_DIR);
  state = STATE_END;
}

if(state == 50){ //State 50: Drive straight using gyro

  mesgBot.driveForwardAtCurrentHeading(4000);
  state = STATE_END;
}

if(state == 55){ //State 55: Drive straight using gyro AND PID

  mesgBot.driveForwardAtCurrentHeadingWithPID(4000);
  state = STATE_END;
}

if(state == 60){ //State 60: Rotate to heading (0 deg, aka back to starting condition)
  mesgBot.turnToHeading(0);

  for (int i = 0; i<5; i++) {
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(500);                     
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);     
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
}

if(state == 80){ //State 60: Rotate to heading (0 deg, aka back to starting condition)
  
blink(5, 250);
mesgBot.forwardDrive(50);
delay(3000);
mesgBot.brake();

blink(10, 250);
mesgBot.forwardDrive(100);
delay(3000);
mesgBot.brake();

blink(15, 250);
mesgBot.forwardDrive(150);
delay(3000);
mesgBot.brake();

blink(20, 250);
mesgBot.forwardDrive(200);
delay(3000);
mesgBot.brake();

blink(25, 250);
mesgBot.forwardDrive(250);
delay(3000);
mesgBot.brake();

blink(100,1000);

}

}

void blink(int numberOfBlinks, int blinkDurationMS){

  for (int i = 0; i<numberOfBlinks; i++) {
  digitalWrite(LED_BUILTIN, HIGH);  
  delay(blinkDurationMS);                     
  digitalWrite(LED_BUILTIN, LOW);
  delay(blinkDurationMS);     
  }
  digitalWrite(LED_BUILTIN, HIGH);

}
 