#include "Robot.h"
#include "Encoder.h"
#include "TOFSensor.h"
#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"

#define DIR1_M1 12  // LEFT
#define DIR1_M2 6   // RIGHT
#define PWM_M1 11   // LEFT
#define PWM_M2 10   // RIGHT

// Pins for encoder
#define ENCA_M1 A4
#define ENCB_M1 A5
#define ENCA_M2 A3
#define ENCB_M2 A2


#define AD0_VAL 86           // gyro
#define BOT_TOF_RESET_PIN 5  //reset


               //State 20: Drive fwd to enc distance with PID
               //State 30: Drive straight using gyro AND PID
               //State 40: Scan/rotate and find pole
               //State 1: Stop
#define TEST_STATE1 20
#define TEST_STATE2 30


const int DirectionInvertLeft = 1;
const int DirectionInvertRight = 1;
int state = 0;  //get enumerations later
double measuredDistance = 0;

Motor leftMotor(DIR1_M1, PWM_M1, DirectionInvertLeft);
Motor rightMotor(DIR1_M2, PWM_M2, DirectionInvertRight);
Encoder leftEncoder(ENCA_M1, ENCB_M1, DirectionInvertLeft);
Encoder rightEncoder(ENCA_M2, ENCB_M2, DirectionInvertRight);
TOFSensor botTOF;
TOFSensor topTOF;
Gyro gyro;

int temp = 0;
int dist = 0;

Robot mesgBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF, gyro);

int find_next_state(int state);

void setup() {
  Serial.begin(115200);
  //  while (!Serial) {}
  Serial.println("\nSTART---------------------------");

  attachInterrupt(ENCA_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCB_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCA_M2, updateRightEncoder, CHANGE);
  attachInterrupt(ENCB_M2, updateRightEncoder, CHANGE);

  mesgBot.allConfiguration();

  state = 0;
  Serial.println("START");

  delay(100);  // replace with wait to stabalize gyro ?
}

void updateLeftEncoder() {
  leftEncoder.EncScanActive();
}
void updateRightEncoder() {
  rightEncoder.EncScanActive();
}

void loop() {  //state machine
               // Serial.println("TIME: " + String(millis()));

  //Start Countdown
  if (state == 0) {
    state = TEST_STATE1;
    blink(10, 100);
    blink(3, 500);
  }

    //Stop shit
    if (state == 1) {
    mesgBot.brake();
    while(1){
      blink(10, 100);
    }
  }

  if (state == 20) {  //drive forward PID
    mesgBot.forwardDrivePID(3000);

    //go to next test 
    state = find_next_state(state);
  }

  if (state == 30) {  //State 55: Drive straight using gyro AND PID
    mesgBot.driveForwardAtCurrentHeadingWithPID(4000,150);
    state = find_next_state(state);
  }


  if (state == 40) {  //State 30: Scan/rotate and drive to enc distance
    Serial.println("Start 300");
    int internalState = 0;

    while (!mesgBot.poleFound()) {
      delay(1000);
      switch (internalState) {
        case 0:
          mesgBot.leftTurnStationaryPID(50);
          internalState++;
          break;
        case 1:
          mesgBot.rightTurnStationaryPID(50);
          internalState++;
          break;

        case 2:
          mesgBot.rightTurnStationaryPID(50);
          internalState++;
          break;

        case 3:
          mesgBot.leftTurnStationaryPID(50);
          internalState++;
          break;

        case 4:
          // mesgBot.driveForwardAtCurrentHeadingWithPID(100,45);
          mesgBot.forwardDrive(45);
          delay(500);
          mesgBot.brake();
          delay(1000);
          // mesgBot.forwardDrive(int speed)
          internalState = 0;
          break;
      }
    }

    mesgBot.driveForwardAtCurrentHeadingWithPID(mesgBot.linearDistToPole,150);

    state = find_next_state(state);
  }


}

void blink(int numberOfBlinks, int blinkDurationMS) {

  for (int i = 0; i < numberOfBlinks; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(blinkDurationMS);
    digitalWrite(LED_BUILTIN, LOW);
    delay(blinkDurationMS);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

int find_next_state(int state)
{
     if(state == TEST_STATE1) 
        state = TEST_STATE2;
    else
      state = 1; //stop and just blink
    return state;
  
}
