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

const int STATE_END = 550;
const int DRIVE_TO_FOUND_POLE_STATE = 500;
const float accelYFallingHigh = 700.0;
const float accelYFallingLow = 10.0;
const double timer = 3000;
double start_time = 0;

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
// Robot mBot(leftMotor, rightMotor);
//  Robot meBot(leftMotor, rightMotor, leftEncoder, rightEncoder);
// Robot mesBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF);
Robot mesgBot(leftMotor, rightMotor, leftEncoder, rightEncoder, botTOF, topTOF, gyro);

void setup() {
  Serial.begin(115200);
  //  while (!Serial) {}
  Serial.println("\nSTART---------------------------");
  blink(4, 50);
  attachInterrupt(ENCA_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCB_M1, updateLeftEncoder, CHANGE);
  attachInterrupt(ENCA_M2, updateRightEncoder, CHANGE);
  attachInterrupt(ENCB_M2, updateRightEncoder, CHANGE);

  mesgBot.allConfiguration();
  //  botTOF.initalizeTOF();

  state = 0;
  Serial.println("START");
  // rightMotor.drive(255, 1);

  // wait(500);
  //     leftMotor.drive(255, 1);
  // gyro.setup();
  wait(100);  // replace with wait to stabalize gyro ?
}

void updateLeftEncoder() {
  leftEncoder.EncScanActive();
  // Serial.println("L " + String(leftEncoder.stepCounter));
  // mesgBot.getOrientationAngle();
}
void updateRightEncoder() {
  rightEncoder.EncScanActive();
  // Serial.println("R " + String(rightEncoder.stepCounter));
  // mesgBot.getOrientationAngle();
}

void loop() {  //state machine
               // Serial.println(state);
               // if(millis()>120000 and state != STATE_END){
               //   mesgBot.brake();
               //   state = STATE_END;
               //   Serial.println("STOPPED");
               // }
               // wait(5000);

  double x_roll_start = 0;
  ;
  double y_pitch_start = 0;
  double z_yaw_start = 0;
  double x_roll = 0;
  ;
  double y_pitch = 0;
  double z_yaw = 0;
  float y_change = 0;
  switch (state) {  // LOOK LINE 414 ROBOT FOR TESTING SCAN DISABLED

    case 0:  // starting condition
      blink(10, 100);
      mesgBot.updateAllGyro();
      x_roll_start = gyro.roll;
      y_pitch_start = gyro.pitch;
      z_yaw_start = gyro.yaw;
      /* while(gyro.acc_x_change != 0 && gyro.acc_y_change != 0 && gyro.acc_z_change != 0)
      {
          mesgBot.updateAllGyro();
          // Serial.println("X: " +  String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) +  " Z: " + String(gyro.acc_z)  + "," + String(gyro.acc_z_change)+ " Roll: " +  String(gyro.roll) + " Pitch: " + String(gyro.pitch) +  " Yaw: " + String(gyro.yaw));

      }
      */
      blink(3, 500);
      start_time = millis();
      // mesgBot.setupSDCard();

      // state = 200;
      state = 3;
      break;

    case 2:
      // Serial.println(millis());
      mesgBot.scanBothTOF();
      Serial.println("TOP: " + String(mesgBot.scanDistanceTopAverage) + "  Bot " + String(mesgBot.scanDistanceBotAverage));
      delay(200);

      break;


    case 3:

      mesgBot.updateAllGyro();
      // Serial.println("X: " +  String(gyro.acc_x) + " Y: " + String(gyro.acc_y) +  " Z: " + String(gyro.acc_z));
      if (gyro.success) {
        Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
      }
      break;

    case 4:

      mesgBot.driveForwardAtCurrentHeadingWithPID(2000, 150);

      state = STATE_END;

      //  wait(20000);

      break;

    case 5:

      mesgBot.driveForwardAtCurrentHeadingWithPID(100, 255);

      wait(7500);

      mesgBot.driveForwardAtCurrentHeadingWithPID(150, 200);

      wait(7500);

      mesgBot.driveForwardAtCurrentHeadingWithPID(250, 255);
      state = STATE_END;
      break;

    case 6:

      mesgBot.driveForwardAtCurrentHeadingWithPID(30, 150);

      wait(7500);

      mesgBot.driveForwardAtCurrentHeadingWithPID(500, 150);

      wait(7500);

      mesgBot.driveForwardAtCurrentHeadingWithPID(1000, 150);

      wait(7500);

      mesgBot.driveForwardAtCurrentHeadingWithPID(1500, 150);


      for (int i = 0; i < 10; i++) {
        mesgBot.leftTurnStationaryPID(50);
        wait(50);
      }


      for (int i = 0; i < 10; i++) {
        mesgBot.leftTurnStationaryPID(50);
        // wait(100);
      }
      //  mesgBot.driveForwardAtCurrentHeadingWithPID(400, 175);
      // mesgBot.driveForwardAtCurrentHeading(350);
      // mesgBot.forwardDriveDistance(325, 151);
      // wait(3000);

      //  mesgBot.forwardDriveDistance(325, 152);
      // wait(3000);

      //  mesgBot.forwardDriveDistance(325, 153);
      // wait(3000);

      //  mesgBot.forwardDriveDistance(325, 154);
      // wait(3000);

      state = STATE_END;

      break;

    case 30:
      Serial.print("get off wall");


      /*
      x_roll_start = gyro.roll;
      y_pitch_start = gyro.pitch;
      z_yaw_start = gyro.yaw;
      */

      while (!gyro.success) {
        mesgBot.updateAllGyro();
      }
      y_change = abs(gyro.acc_y_change);
      //wait until in the air
      while (accelYFallingHigh > y_change) {

        mesgBot.updateAllGyro();
        while (!gyro.success) {
          mesgBot.updateAllGyro();
        }
        // if(gyro.success)
        //{
        Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
        y_change = abs(gyro.acc_y_change);
        Serial.println(y_change);
        // }
      }
      //wait until we slow down
      while (accelYFallingLow < y_change) {

        mesgBot.updateAllGyro();
        while (!gyro.success) {
          mesgBot.updateAllGyro();
        }
        // if(gyro.success)
        //{
        Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
        y_change = abs(gyro.acc_y_change);
        Serial.println(y_change);
        // }

      }  //&& (abs(gyro.acc_y_change) < accelYFallingUp));
      while(millis() < (timer - start_time));
      Serial.print("no longer falling");
      Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));


      x_roll = x_roll_start - gyro.roll;
      y_pitch = y_pitch_start - gyro.pitch;
      z_yaw = z_yaw_start - gyro.yaw;

      Serial.println("X: " + String(x_roll) + " Y: " + String(y_pitch) + " Z: " + String(z_yaw));

      if (abs(x_roll) > 20) {
        Serial.print("robot backwards");
        while (y_pitch > 0) {
          mesgBot.drive(100, FORWARD_DIR);
          mesgBot.updateAllGyro();
          y_pitch = y_pitch_start - gyro.pitch;
        }
        mesgBot.reverseDriveDistance(10, 200);
      } else {
        Serial.print("robot forwards");

        while (y_pitch > 0) {
          mesgBot.drive(100, FORWARD_DIR);
          mesgBot.updateAllGyro();
          y_pitch = y_pitch_start - gyro.pitch;
        }
        mesgBot.forwardDriveDistance(10, 200);
      }
      state = 50;

      Serial.print("Done falling");

      break;



      // case 5:
      //   mesgBot.driveForwardAtCurrentHeadingWithPID(500, 150);

      //   wait(3000);

      //   mesgBot.driveForwardAtCurrentHeadingWithPID(1000, 150);

      //   state = STATE_END;
      //   break;

    case 50:  // rotate to start scan 1 heading
      Serial.println("turn to 90");
      // wait(5000);
      mesgBot.turnToHeading(90);
      state = 60;
      Serial.println("EXIT  STATE 50");
      // wait(3000);
      break;

    case 60:  //Look for pole at position 1
      Serial.println("Look for pole 1");
      // wait(1000);
      if (mesgBot.searchForPole(CW_DIR, 90 + 30)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 70;
      }

      break;

    case 70:  //if pole not found at position 1, go to position 2
      Serial.println("drive to pos 2");
      // wait(1000);
      mesgBot.turnToHeading(14);
      // wait(1000);
      mesgBot.driveForwardAtCurrentHeadingWithPID(990, 150);
      // wait(5000);
      state = 80;
      break;

    case 80:  // rotate to start scan 2 heading
      Serial.println("Start 80");
      mesgBot.turnToHeading(120);
      Serial.println("END 80");
      state = 90;
      break;
      ///////////////////////////////////////
    case 90:
      Serial.println("Start 90");
      // wait(1000);
      if (mesgBot.searchForPole(CW_DIR, 30 + 90 + 30)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 30 + 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;
      }
      // Serial.println("END 90");
      break;


      // case 60:  //Look for pole at position 1
      //   Serial.println("Look for pole 1");
      //   wait(1000);
      //   if (mesgBot.searchForPole(CW_DIR, 90 + 30)) {
      //     state = DRIVE_TO_FOUND_POLE_STATE;
      //   } else {
      //     state = 70;
      //   }

      //   break;
      ///////////////////

    case 200:
      mesgBot.turnToHeading(90);
      state = 220;
      break;

    case 220:  // Scan 1 Off Wall

      if (mesgBot.searchForPole(CW_DIR, 145)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 230;
      }

      break;

    case 230:  // Move to pos 2

      mesgBot.turnToHeading(21);
      mesgBot.driveForwardAtCurrentHeadingWithPID(884, 150);
      state = 240;
      break;

    case 240:  // turn to heading 2 and scan 2
      mesgBot.turnToHeading(150);
      if (mesgBot.searchForPole(CW_DIR, 260)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 250;
      }

      break;

    case 250:  //move to position 3:
      mesgBot.turnToHeading(15);
      mesgBot.driveForwardAtCurrentHeadingWithPID(624, 150);
      state = 255;
      break;

    case 255:  //Scan 3
      mesgBot.turnToHeading(130);
      if (mesgBot.searchForPole(CW_DIR, 230)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 260;
      }
      break;


    case 260:  // Move to position 4
      mesgBot.turnToHeading(-86);
      mesgBot.driveForwardAtCurrentHeadingWithPID(450, 150);
      state = 270;

      break;

    case 270:  // scan 4
      mesgBot.turnToHeading(0);
      if (mesgBot.searchForPole(CW_DIR, 150)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 280;
      }

      break;

    case 280:  // move to 5
      mesgBot.turnToHeading(-140);
      mesgBot.driveForwardAtCurrentHeadingWithPID(675, 150);
      state = 290;

      break;


    case 290:  // Scan 5 CHECK 180 DEGREE ISSUES
      mesgBot.turnToHeading(180);
      if (mesgBot.searchForPole(CCW_DIR, 155)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }

      break;

      //turn
      // mesgBot.turnToHeading(90);

      //Scan
      //  if (mesgBot.searchForPole(CW_DIR, 90 + 30)) {
      //     // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
      //       state = DRIVE_TO_FOUND_POLE_STATE;
      //     } else {
      //       state = 70;
      //     }

      //drive
      // mesgBot.turnToHeading(14);
      // mesgBot.driveForwardAtCurrentHeadingWithPID(990, 150);

      break;


      ///////////////////////////////////////

    case 100:  // Not found state
      Serial.println("Pole not found");
      blink(10, 1000);
      break;

    case DRIVE_TO_FOUND_POLE_STATE:
      Serial.println("POLE FOUND");
      // mesgBot.driveForwardAtCurrentHeadingWithPID(250, 150);

      // mesgBot.driveForwardAtCurrentHeadingWithPID(mesgBot.linearDistToPole+100, 150);
      // mesgBot.driveForwardAtCurrentHeadingWithPIDold(mesgBot.linearDistToPole, 150);
      if (mesgBot.linearDistToPole < 300) {
        mesgBot.linearDistToPole += 100;
      }
      mesgBot.driveForwardAtCurrentHeadingWithPIDPOLE(mesgBot.linearDistToPole, 150);
      Serial.println("DONE");
      state = STATE_END;
      break;

    case STATE_END:
      blink(50, 50);
      break;
  }
}

void wait(double MS) {
  // Serial.println("WAIT");
  // delay(2000);
  double startTime = millis();
  while ((millis() - startTime) < MS) {
    // mesgBot.getOrientationAngle();
    gyro.updateAllAngles();
  };
}
void blink(int numberOfBlinks, int blinkDurationMS) {

  for (int i = 0; i < numberOfBlinks; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    wait(blinkDurationMS);
    digitalWrite(LED_BUILTIN, LOW);
    wait(blinkDurationMS);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}
