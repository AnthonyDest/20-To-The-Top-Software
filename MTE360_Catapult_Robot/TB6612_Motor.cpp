#include "TB6612_Motor.h"

#include <Arduino.h>

Motor::Motor() {}

Motor::Motor(int DIRpin, int PWMpin, int DirectionInvert) {
  DIR = DIRpin;
  PWM = PWMpin;
  // Standby = STBYpin;
  this->DirectionInvert = DirectionInvert;

  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);

  // pinMode(Standby, OUTPUT);
}

void Motor::speedUp(int desiredSpeed, int prevSpeed, int direction) {
  for (curSpeed = prevSpeed; curSpeed <= desiredSpeed; curSpeed += SPEEDINCREMENT) {

    curSpeed += SPEEDINCREMENT;
    if (curSpeed > desiredSpeed) curSpeed = desiredSpeed;

    if (direction == FORWARD_DIR) fwd(desiredSpeed);
    else rev(desiredSpeed);
  }
}

void Motor::slowDown(int desiredSpeed, int prevSpeed, int direction) {
  for (curSpeed = prevSpeed; curSpeed >= desiredSpeed; curSpeed -= SPEEDINCREMENT) {

    curSpeed -= SPEEDINCREMENT;
    if (curSpeed < desiredSpeed) curSpeed = desiredSpeed;

    if (direction == FORWARD_DIR) fwd(desiredSpeed);
    else rev(desiredSpeed);
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

  // if (direction == forward) fwd(desiredSpeed);
  // else rev(-desiredSpeed);
}

// //Drive for a set duration
// void Motor::drive(int speed, int duration) {
//   drive(speed);
//   delay(duration);
// }

int Motor::getMotorDir(){
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

Encoder::Encoder(int ENCA_Pin, int ENCB_Pin) {
  ENC_A = ENCA_Pin;
  ENC_B = ENCB_Pin;

  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  aPrevState = digitalRead(ENC_A);
  
}

void Encoder::EncScanActive() {

  // If the previous and the current state of the outputA are different, that means a Pulse (swap direction) has occured
  aCurrentState = digitalRead(ENC_A);  // Reads the "current" state of the outputA
  if (aCurrentState != aPrevState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(ENC_B) != aCurrentState) {
      stepCounter++;
    } else {
      stepCounter--;
    }
  }
  aPrevState = aCurrentState;
}

void Encoder::resetCounter(){
  stepCounter = 0;
}

float Encoder::getDistanceCM(){

  return (stepCounter/stepsPerWheelRotation)*wheelDiamMM*3.14159/10; // replace pi with math library

}
void Encoder::getMotorDirection(){} // can use encoder to sample motor and verify rotation


/*****************************************************************************/

Robot::Robot(Motor leftMotorParam, Motor rightMotorParam, Encoder leftEncoderP, Encoder rightEncoderP) {
  Motor leftMotor = leftMotorParam;
  Motor rightMotor = rightMotorParam;
  Encoder leftEncoder = leftEncoderP;
  Encoder rightEncoder = rightEncoderP;

}

//Drive forward
void Robot::forwardDrive(int speed) {
  speed = abs(speed);
  leftMotor.drive(speed, FORWARD_DIR);
  rightMotor.drive(speed, FORWARD_DIR);
}


//Drive backward
void Robot::backwardDrive(int speed) {
  speed = abs(speed);
  leftMotor.drive(speed, BACKWARD_DIR);
  rightMotor.drive(speed, BACKWARD_DIR);
}


//Turn left
void Robot::leftTurnStationary(int speed) {
  speed = abs(speed) / 2;
  leftMotor.drive(speed, BACKWARD_DIR);
  rightMotor.drive(speed, FORWARD_DIR);
}

// //Turn left, default speed
// void Robot::leftTurn() {
//   speed = abs(DEFAULTSPEED) / 2;
//   leftMotor.drive(-speed);
//   rightMotor.drive(speed);
// }


//Turn right
void Robot::rightTurnStationary(int speed) {
  speed = abs(speed) / 2;
  leftMotor.drive(speed, FORWARD_DIR);
  rightMotor.drive(speed, BACKWARD_DIR);
}

// //Turn right, default speed
// void Robot::rightTurn() {
//   speed = abs(speed) / 2;
//   leftMotor.drive(speed);
//   rightMotor.drive(-speed);
// }


//Motor braking (hard stop)
void Robot::brake() {

  // slow down (need to check current direction of motors)
  leftMotor.drive(0, leftMotor.getMotorDir());
  rightMotor.drive(0, rightMotor.getMotorDir());

  // stop
  leftMotor.brake();
  rightMotor.brake();
}

void Robot::forwardDriveDistance(int speed, float distanceCM) {
initialDistance = average(leftEncoder.getDistanceCM(), rightEncoder.getDistanceCM()); // do we get average and compare end result avg, or do we just monitor each independently
// eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
forwardDrive(speed);

while(distanceCM >= currentDistanceTravelled){
currentDistanceTravelled = average(leftEncoder.getDistanceCM(), rightEncoder.getDistanceCM()) - initialDistance;
}
currentDistanceTravelled = 0;
brake(); // will stop currently for testing, but this can be removed later to optimize
}

void Robot::reverseDriveDistance(int speed, float distanceCM) {
initialDistance = average(leftEncoder.getDistanceCM(), rightEncoder.getDistanceCM()); // do we get average and compare end result avg, or do we just monitor each independently
// eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
backwardDrive(speed);

while(distanceCM >= currentDistanceTravelled){
currentDistanceTravelled = abs(average(leftEncoder.getDistanceCM(), rightEncoder.getDistanceCM()) - initialDistance);
}
currentDistanceTravelled = 0;
brake(); // will stop currently for testing, but this can be removed later to optimize
}


float Robot::average(float inputA, float inputB){

  return (inputA+inputB)/2;

}
// no longer used
// //idle
// void Robot::standby() {
//   leftMotor.standby();
//   rightMotor.standby();
// }
