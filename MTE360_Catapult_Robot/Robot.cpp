#include "Robot.h"

Robot::Robot(){};

// Robot::Robot(Motor& testMotorP){

//   // testMotor -> testMotorP;
//   // Serial.println("bb" + (unsigned long)&testMotorP);
// }

Robot::Robot(Motor &_leftMotor) {
}

Robot::Robot(Motor &_leftMotor, Motor &_rightMotor) {
  leftMotor = &_leftMotor;
  rightMotor = &_rightMotor;
}

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
