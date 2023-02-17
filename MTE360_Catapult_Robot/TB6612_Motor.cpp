#include "HardwareSerial.h"
#include "TB6612_Motor.h"

#include <Arduino.h>
#include "Rotary.h"

Motor::Motor() {}

Motor::Motor(int _DIR, int _PWM, int _DirectionInvert) {
  DIR = _DIR;
  PWM = _PWM;
  // Standby = STBYpin;
  DirectionInvert = _DirectionInvert;

  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);
  Serial.begin(9600);
  // pinMode(Standby, OUTPUT);
}

//may want to remove while if the time is too long
void Motor::speedUp(int desiredSpeed, int curSpeed, int direction) {

  while (curSpeed < desiredSpeed) {

    curSpeed += SPEEDINCREMENT;

    // redundant input checks
    if (curSpeed > desiredSpeed) {
      curSpeed = desiredSpeed;
    }

    if (curSpeed < MINSPEED ) {
        curSpeed = MINSPEED;
      }

    if (curSpeed > MAXSPEED) {
      curSpeed = MAXSPEED;
    }

    if (direction == FORWARD_DIR) {
      fwd(curSpeed);
    } else {
      rev(curSpeed);
    }

    delay(SPEEDDELAY);
  }
}
//may want to remove while if the time is too long
void Motor::slowDown(int desiredSpeed, int curSpeed, int direction) {

  while (curSpeed > desiredSpeed) {

    curSpeed -= SPEEDINCREMENT;

   // redundant input checks
    if (curSpeed < desiredSpeed ){
        curSpeed = desiredSpeed;
      }

    if (curSpeed < MINSPEED ) {
        curSpeed = 0;
      }

    if (direction == FORWARD_DIR) {
      fwd(curSpeed);
    } else {
      rev(curSpeed);
    }

    delay(SPEEDDELAY);

  }
}

//Determines which way to spin motor and how to scale speed 
void Motor::drive(int desiredSpeed, int direction) {

  direction = direction * DirectionInvert;

  if (abs(prevSpeed) > abs(desiredSpeed)) {  // slowing down

    slowDown(desiredSpeed, prevSpeed, direction);

  } else if (abs(prevSpeed) < abs(desiredSpeed)) {  // speeding up

    speedUp(desiredSpeed, prevSpeed, direction);
  }
}

int Motor::getMotorDir() {
  return curDirState;
}


//Go Forward at entered speed
void Motor::fwd(int speed) {
  curDirState = FORWARD_DIR;
  digitalWrite(DIR, HIGH);  // MAY BE LOW BASED ON ELEC NOT
  analogWrite(PWM, speed);
}

//Go backwards at entered speed
void Motor::rev(int speed) {
  curDirState = BACKWARD_DIR;
  digitalWrite(DIR, LOW);  //MAY BE HIGH BASED ON ELEC NOT
  analogWrite(PWM, speed);
}


//Use motor to brake robot (CAREFUL, LOOK AT BACK EMF AND SHORTING)
void Motor::brake() {
  // stopped state?
  // digitalWrite(In1, HIGH);
  // digitalWrite(In2, HIGH);
  analogWrite(PWM, 0);
}

// no longer used
// //Idle motor (no power) (CAREFUL, LOOK AT BACK EMF AND SHORTING)
// void Motor::standby() {
//   digitalWrite(Standby, LOW);
// }

/***********************************************************************/

Encoder::Encoder() {}

Encoder::Encoder(int _ENC_A, int _ENC_B, int DirectionInvertP) {
  // ENC_A = ENCA_Pin;
  // ENC_B = ENCB_Pin;
DirectionInvert = DirectionInvertP;

  rotary = Rotary(_ENC_A, _ENC_B); //else just pass by reference


  // pinMode(ENC_A, INPUT);
  // pinMode(ENC_B, INPUT);

  // aPrevState = digitalRead(ENC_A);
}

void Encoder::EncScanActive(){

 rotaryDirection = rotary.process();

 //needs direction invert
  if (rotaryDirection == DIR_CW) {
    stepCounter++;
    Serial.println(stepCounter);
  } else if (rotaryDirection == DIR_CCW) {
    stepCounter--;
    Serial.println(stepCounter);
  }

}

// void Encoder::EncScanActive() {

//   // If the previous and the current state of the outputA are different, that means a Pulse (swap direction) has occured
//   aCurrentState = digitalRead(ENC_A);  // Reads the "current" state of the outputA
//   if (aCurrentState != aPrevState) {
//     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//     if (digitalRead(ENC_B) != aCurrentState) {
//       stepCounter++;
//       // Serial.println("Forward");
//     } else {
//       stepCounter--;
//       // Serial.println("Backward");
//     }
//     //  Serial.println(stepCounter);
//   }
//   aPrevState = aCurrentState;
// }

void Encoder::resetCounter() {
  stepCounter = 0;
}

float Encoder::getDistanceCM() {

// Serial.println(float(stepCounter) / float(stepsPerWheelRotation));
  // return ((stepCounter / float(stepsPerWheelRotation) )* float(wheelDiamMM) * float(3.14159 / 10));  // replace pi with math library
  return (float(stepCounter) / float(stepsPerWheelRotation));
}
void Encoder::getMotorDirection() {}  // can use encoder to sample motor and verify rotation


/*****************************************************************************/

Robot::Robot(){};

// Robot::Robot(Motor& testMotorP){

//   // testMotor -> testMotorP;
//   // Serial.println("bb" + (unsigned long)&testMotorP);
// }


Robot::Robot(Motor &_leftMotor, Motor &_rightMotor, Encoder &_leftEncoder, Encoder &_rightEncoder) {
  leftMotor = &_leftMotor;
  rightMotor = &_rightMotor;
  leftEncoder = &_leftEncoder;
  rightEncoder = &_rightEncoder;
}

//Drive forward
void Robot::forwardDrive(int speed) {
  speed = abs(speed);
  
  // Serial.println("LEFT IN FXN: " + (unsigned long)&leftMotor);
  leftMotor->drive(speed, FORWARD_DIR);
  rightMotor->drive(speed, FORWARD_DIR);
}


//Drive backward
void Robot::backwardDrive(int speed) {
  speed = abs(speed);
  leftMotor->drive(speed, BACKWARD_DIR);
  rightMotor->drive(speed, BACKWARD_DIR);
}


//Turn left
void Robot::leftTurnStationary(int speed) {
  speed = abs(speed) / 2;
  leftMotor->drive(speed, BACKWARD_DIR);
  rightMotor->drive(speed, FORWARD_DIR);
}

//Turn right
void Robot::rightTurnStationary(int speed) {
  speed = abs(speed) / 2;
  leftMotor->drive(speed, FORWARD_DIR);
  rightMotor->drive(speed, BACKWARD_DIR);
}


//Motor braking (hard stop)
void Robot::brake() {

  // slow down (need to check current direction of motors)
  leftMotor->drive(0, leftMotor->getMotorDir());
  rightMotor->drive(0, rightMotor->getMotorDir());

  // stop
  leftMotor->brake();
  rightMotor->brake();
}

void Robot::forwardDriveDistance(int speed, float distanceCM) {
  initialDistance = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM());  // do we get average and compare end result avg, or do we just monitor each independently
  // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
  forwardDrive(speed);

  while (distanceCM >= currentDistanceTravelled) {
    currentDistanceTravelled = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM()) - initialDistance;
  }
  currentDistanceTravelled = 0;
  brake();  // will stop currently for testing, but this can be removed later to optimize
}

void Robot::reverseDriveDistance(int speed, float distanceCM) {
  initialDistance = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM());  // do we get average and compare end result avg, or do we just monitor each independently
  // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
  backwardDrive(speed);

  while (distanceCM >= currentDistanceTravelled) {
    currentDistanceTravelled = abs(average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM()) - initialDistance);
  }
  currentDistanceTravelled = 0;
  brake();  // will stop currently for testing, but this can be removed later to optimize
}


float Robot::average(float inputA, float inputB) {

  return (inputA + inputB) / 2;
}
// no longer used
// //idle
// void Robot::standby() {
//   leftMotor.standby();
//   rightMotor.standby();
// }
