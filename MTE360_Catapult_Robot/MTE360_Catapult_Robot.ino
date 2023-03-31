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

const int PATH_TO_FIND_POLE = 600;  //Change this to select path
//1300 new full sweep

//500 is back left     |   //800 is back right 
//400 is middle left   |   //700 is middle right 
//300 is close left    |   //600 is close right 

const int STATE_END = 998;
const int DRIVE_TO_FOUND_POLE_STATE = 999;

const int minPitchOnWall = -65;

const double timer = 1000;
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
  // while (!Serial) {}
  // delay(5000);
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
  //wait(100);  // replace with wait to stabalize gyro ?
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

  switch (state) {  // LOOK LINE 414 ROBOT FOR TESTING SCAN DISABLED

    case 0:  // starting condition
      blink(10, 100);
      mesgBot.updateAllGyro();
      blink(3, 500);
      start_time = millis();
      // mesgBot.setupSDCard();


       while (gyro.pitch > -45) {
          mesgBot.updateAllGyro();
           if (abs(gyro.yaw) < 5) {  // if aligned straight, constant LED
          digitalWrite(LED_BUILTIN, HIGH);
        } else {  //IF not aligned, turn off led
          digitalWrite(LED_BUILTIN, LOW);
        }
       }
       while (gyro.pitch < 0) {
          mesgBot.updateAllGyro();
           if (abs(gyro.yaw) < 5) {  // if aligned straight, constant LED
          digitalWrite(LED_BUILTIN, HIGH);
        } else {  //IF not aligned, turn off led
          digitalWrite(LED_BUILTIN, LOW);
        }
       }

       wait(5000);
      // state = 20;
      state = PATH_TO_FIND_POLE;
      // state = 8;
      break;

    case 2:
      // Serial.println(millis());
      mesgBot.scanBothTOF();
      Serial.println("TOP: " + String(mesgBot.scanDistanceTopAverage) + "  Bot " + String(mesgBot.scanDistanceBotAverage));
      delay(50);

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

      wait(20000);

      break;

    case 5:

 mesgBot.updateAllGyro();
      Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
     
    if(gyro.pitch < 0) {
        while (gyro.pitch < 0) {
          mesgBot.updateAllGyro();
          mesgBot.forwardDrive(100);
          Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
      }

      mesgBot.brake();
      wait(1000);
            mesgBot.reverseDriveDistance(200, 40);

        mesgBot.brake();
        wait(2000);

        mesgBot.turnToHeading(0);

        // mesgBot.driveForwardAtCurrentHeadingWithPID(500, 150);
    }
      // mesgBot.updateAllGyro();
      // Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
     
      // if (gyro.pitch < 0) {
      //   mesgBot.forwardDrive(150);
      //   while (gyro.pitch < 0) {
      //     mesgBot.updateAllGyro();
      //     Serial.println("X: " + String(gyro.acc_x) + "," + String(gyro.acc_x_change) + " Y: " + String(gyro.acc_y) + "," + String(gyro.acc_y_change) + " Z: " + String(gyro.acc_z) + "," + String(gyro.acc_z_change) + " Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));
      //   }
      //   mesgBot.driveForwardAtCurrentHeadingWithPID(200, 150);
      //   wait(2000);
      // }



      //   mesgBot.driveForwardAtCurrentHeadingWithPID(100, 255);

      //  wait(7500);

      //   mesgBot.driveForwardAtCurrentHeadingWithPID(150, 200);

      //   wait(7500);

      //   mesgBot.driveForwardAtCurrentHeadingWithPID(250, 255);
      //   state = STATE_END;
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


    case 7:

      mesgBot.reverseDrivePID(50);
      wait(3000);

      mesgBot.reverseDrivePID(200);
      wait(3000);

      break;

    case 8:
      Serial.println("heading1 " + String(gyro.yaw));
      mesgBot.driveForwardAtCurrentHeadingWithPID(400, 150);
      Serial.println("heading2 " + String(gyro.yaw));
      wait(2000);
      Serial.println("heading3 " + String(gyro.yaw));
      mesgBot.turnToHeading(90);
      Serial.println("heading4 " + String(gyro.yaw));
      wait(1000);
      Serial.println("heading5 " + String(gyro.yaw));
      mesgBot.turnToHeading(-45);
      Serial.println("heading6 " + String(gyro.yaw));

      wait(1000);
      Serial.println("heading7 " + String(gyro.yaw));
      mesgBot.turnToHeading(14);
      Serial.println("heading8 " + String(gyro.yaw));

      state = STATE_END;

      break;

      /////////////////////////////////////

    case 20:  //Gyro has already been init,  //Confirm heading is good, waiting on launch - Transition state = on wall when pitch < 30deg? TEST value

      while (gyro.pitch > minPitchOnWall) {  // dont change state until robot is facing downwards (on back of wall)
        mesgBot.updateAllGyro();
        Serial.println("Roll: " + String(gyro.roll) + " Pitch: " + String(gyro.pitch) + " Yaw: " + String(gyro.yaw));

        if (abs(gyro.yaw) < 5) {  // if aligned straight, constant LED
          digitalWrite(LED_BUILTIN, HIGH);
        } else {  //IF not aligned, turn off led
          digitalWrite(LED_BUILTIN, LOW);
        }
      }

      state = 30;

      break;

    case 30:
      Serial.print("get off wall");

      wait(2000);
      //using pole found to get linear distance to ground

      // mesgBot.poleFound();
      while (mesgBot.linearDistToPole > 200 or mesgBot.linearDistToPole == 0) {
        mesgBot.updateAllGyro();
        // Serial.println(mesgBot.linearDistToPole);
        mesgBot.poleFound();
        Serial.println("X: " + String(gyro.roll) + " Y: " + String(gyro.pitch) + " Z: " + String(gyro.yaw));
      }
      Serial.println("AFTER DISTANCE: " + String(mesgBot.linearDistToPole));

      Serial.println("X: " + String(gyro.roll) + " Y: " + String(gyro.pitch) + " Z: " + String(gyro.yaw));

mesgBot.updateAllGyro();
      if (abs(gyro.roll) > 90 and gyro.pitch < 0) {  //if fully backwards

      // if(gyro.pitch < 0) {
        while (gyro.pitch < 0) {
          mesgBot.updateAllGyro();
          mesgBot.forwardDrive(100);
          Serial.println("X: " + String(gyro.roll) + " Y: " + String(gyro.pitch) + " Z: " + String(gyro.yaw));

             }

      mesgBot.brake();
      wait(1000);
            mesgBot.reverseDriveDistance(200, 40);

        mesgBot.brake();
        wait(2000);

        mesgBot.turnToHeading(0);

        // mesgBot.driveForwardAtCurrentHeadingWithPID(500, 150);
    
      }



      // if (abs(gyro.roll) > 90 and gyro.pitch < 10) { //Backwards
      //   Serial.print("robot backwards");
      //   while (gyro.pitch < 0) {
      //     mesgBot.drive(100, FORWARD_DIR);
      //     mesgBot.updateAllGyro();
      //   }
      //   mesgBot.reverseDriveDistance(10, 200);
      //   wait(1000);

      // } else if (abs(gyro.roll) < 30 and gyro.pitch < 10) { //Forwards
      //   Serial.print("robot forwards");

      //   while (gyro.pitch < 0) {
      //     mesgBot.drive(100, FORWARD_DIR);
      //     mesgBot.updateAllGyro();
      //   }
      //   mesgBot.forwardDriveDistance(10, 200);

      // } else if(abs(gyro.roll) > 30 and abs(gyro.roll) <60 and gyro.pitch > 0){ //Sideways

      //   while (gyro.pitch < 0) {
      //     mesgBot.drive(100, BACKWARD_DIR);
      //     mesgBot.updateAllGyro();
      //   }



      // } else if (abs(gyro.yaw) > 45 ){ //Too far turned right or left, face wall, reverse, then turn to right heading
      //   Serial.println("(abs(z_yaw) > 45 )   Too spun around");
      //   //mesgBot.turnToHeading(180);
      //   mesgBot.reverseDrivePID(50);

      //   //remove later
      //  // wait(2000);
      //   mesgBot.turnToHeading(90);

      // }else if (abs(gyro.yaw) < 45 ){ //Slightly turned right or left, drive off the wall a bit, orientate, drive off wall normal distance, then turn to right heading
      // Serial.println("(abs(z_yaw) < 45 ) Slightly turned");

      mesgBot.driveForwardAtCurrentHeadingWithPID(50, 50);
      mesgBot.turnToHeading(0);
      mesgBot.driveForwardAtCurrentHeadingWithPID(50, 50);
      // }

      // state = 200;
      state = PATH_TO_FIND_POLE;

      Serial.print("Done falling");

      break;



      //200+ is start of actual search algorithm

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
        // state =STATE_END; //zzRemove
      }

      break;

    case 230:  // Move to pos 2
      wait(500);
      mesgBot.turnToHeading(21);
      wait(500);
      mesgBot.driveForwardAtCurrentHeadingWithPID(884, 150);
      state = 240;
      // state =STATE_END; //zzRemove
      break;

    case 240:  // turn to heading 2 and scan 2
      mesgBot.turnToHeading(150);
      if (mesgBot.searchForPole(CW_DIR, 260)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        // state =STATE_END; //zzRemove
        state = 250;
      }

      break;

    case 250:  //move to position 3:
      mesgBot.turnToHeading(5);
      wait(500);
      mesgBot.driveForwardAtCurrentHeadingWithPID(624, 150);
      state = 255;
      break;

    case 255:  //Scan 3
      mesgBot.turnToHeading(130);
      wait(500);
      // blink(30,10);
      // wait(2000);
      if (mesgBot.searchForPole(CW_DIR, 230)) {
        // if (mesgBot.searchForPoleContiniousSweep(CW_DIR, 90 + 30)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 260;
        blink(15, 30);
      }
      break;


    case 260:  // Move to position 4
               // blink(5,250);
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


      // SPLIT COURSE INTO 2X3 GRID
    case 300:  //front left
      mesgBot.turnToHeading(90);
      state = 310;
      break;

    case 310:
      if (mesgBot.searchForPole(CW_DIR, 110)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }
      break;

    case 400:  //middle left
      mesgBot.turnToHeading(21);
      mesgBot.driveForwardAtCurrentHeadingWithPID(884, 150);
      state = 410;
      break;
    case 410:
      mesgBot.turnToHeading(150);
      if (mesgBot.searchForPole(CW_DIR, 260)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }
      break;

    case 500:                                                  //back left
      mesgBot.turnToHeading(19);                               //CHANGE HEADING
      mesgBot.driveForwardAtCurrentHeadingWithPID(1309, 150);  //CHANGE DIST
      state = 510;
      break;
    case 510:
      mesgBot.turnToHeading(130);
      if (mesgBot.searchForPole(CW_DIR, 230)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }
      break;
    case 600:  //front right
      mesgBot.turnToHeading(-90);
      state = 610;
      break;
    case 610:
      if (mesgBot.searchForPole(CCW_DIR, 110)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }
      break;

    case 700:  //middle left
      mesgBot.turnToHeading(-10);
      mesgBot.driveForwardAtCurrentHeadingWithPID(634, 150);
      state = 710;
      break;
    case 710:
      mesgBot.turnToHeading(105);
      if (mesgBot.searchForPole(CW_DIR, 225)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }
      break;

    case 800:  //back left
      mesgBot.turnToHeading(-5);
      mesgBot.driveForwardAtCurrentHeadingWithPID(1222, 150);
      state = 810;
      break;
    case 810:
      mesgBot.turnToHeading(120);
      if (mesgBot.searchForPole(CW_DIR, 230)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
        state = 100;  // else pole not found
      }
      break;



    case 1300: //front
      mesgBot.turnToHeading(90);
      state = 1310;
      break;
    case 1310: 
      if (mesgBot.searchForPole(CW_DIR, 180)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
         state = 1400;  // else pole not found
      }
      break;

    case 1400: //middle left
      mesgBot.turnToHeading(21);
      mesgBot.driveForwardAtCurrentHeadingWithPID(884, 150);
      state = 1410;
      break;
    case 1410: 
      mesgBot.turnToHeading(150); 
      if (mesgBot.searchForPole(CW_DIR, 260)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
         state = 1500;  // else pole not found
      }
      break;

     case 1500: //back left
      mesgBot.turnToHeading(14);
      mesgBot.driveForwardAtCurrentHeadingWithPID(427, 150);
      state = 1510;
      break;
    case 1510: 
      mesgBot.turnToHeading(130); 
      if (mesgBot.searchForPole(CW_DIR, 230)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
         state = 1800;  // else pole not found
      }
      break;
    
     case 1800: //back right
      mesgBot.turnToHeading(-97);
      mesgBot.driveForwardAtCurrentHeadingWithPID(591, 150);
      state = 1810;
      break;
    case 1810: 
      mesgBot.turnToHeading(10); 
      if (mesgBot.searchForPole(CW_DIR, 150)) {
        state = DRIVE_TO_FOUND_POLE_STATE;
      } else {
         state = 100;  // else pole not found
      }
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
