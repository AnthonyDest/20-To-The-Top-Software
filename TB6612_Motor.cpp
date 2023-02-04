#include "TB6612_Motor.h"

#include <Arduino.h>

Motor::Motor(){}

Motor::Motor(int In1pin, int In2pin, int PWMpin, int DirectionInvertParam, int STBYpin) {
  In1 = In1pin;
  In2 = In2pin;
  PWM = PWMpin;
  Standby = STBYpin;
  DirectionInvert = DirectionInvertParam;

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(Standby, OUTPUT);
}

//Determines which way to spin motor
void Motor::drive(int speed) {
  digitalWrite(Standby, HIGH);
  speed = speed * DirectionInvert;
  if (speed >= 0) fwd(speed);
  else rev(-speed);
}

//Drive for a set duration
void Motor::drive(int speed, int duration) {
  drive(speed);
  delay(duration);
}

//Go Forward at entered speed
void Motor::fwd(int speed) {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  analogWrite(PWM, speed);
}

//Go backwards at entered speed
void Motor::rev(int speed) {
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  analogWrite(PWM, speed);
}

//Use motor to brake robot (CAREFUL, LOOK AT BACK EMF AND SHORTING)
void Motor::brake() {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, HIGH);
  analogWrite(PWM, 0);
}

//Idle motor (no power) (CAREFUL, LOOK AT BACK EMF AND SHORTING)
void Motor::standby() {
  digitalWrite(Standby, LOW);
}

/*****************************************************************************/

Robot::Robot(Motor leftMotorParam, Motor rightMotorParam) {
  Motor leftMotor = leftMotorParam;
  Motor rightMotor = rightMotorParam;
}

//Drive forward
void Robot::forwardDrive(int speed) {
  speed = abs(speed);
  leftMotor.drive(speed);
  rightMotor.drive(speed);
}

//Drive forward, default speed
void Robot::forwardDrive() {
  leftMotor.drive(DEFAULTSPEED);
  rightMotor.drive(DEFAULTSPEED);
}

//Drive backward
void Robot::backwardDrive(int speed) {
  speed = -abs(speed);
  leftMotor.drive(speed);
  rightMotor.drive(speed);
}

//Drive backward, default speed
void Robot::backwardDrive() {
  leftMotor.drive(-DEFAULTSPEED);
  rightMotor.drive(-DEFAULTSPEED);
}

//Turn left
void Robot::leftTurn(int speed) {
  speed = abs(speed) / 2;
  leftMotor.drive(-speed);
  rightMotor.drive(speed);
}

// //Turn left, default speed
// void Robot::leftTurn() {
//   speed = abs(DEFAULTSPEED) / 2;
//   leftMotor.drive(-speed);
//   rightMotor.drive(speed);
// }


//Turn right
void Robot::rightTurn(int speed) {
  speed = abs(speed) / 2;
  leftMotor.drive(speed);
  rightMotor.drive(-speed);
}

// //Turn right, default speed
// void Robot::rightTurn() {
//   speed = abs(speed) / 2;
//   leftMotor.drive(speed);
//   rightMotor.drive(-speed);
// }


//Motor braking (hard stop)
void Robot::brake() {
  leftMotor.brake();
  rightMotor.brake();
}

//idle
void Robot::standby() {
  leftMotor.standby();
  rightMotor.standby();
}
