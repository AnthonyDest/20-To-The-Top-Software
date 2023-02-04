#include "TB6612_Motor.h"

#include <Arduino.h>

// Encoder::Encoder(){}

// Encoder::Encoder(int ENCA_Pin, int ENCB_Pin){

//   this->enc


//   currentState = digitalRead(outputA); // Reads the "current" state of the outputA
//     // If the previous and the current state of the outputA are different, that means a Pulse has occured
//     if (aState != aLastState){     
//       // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//       if (digitalRead(outputB) != aState) { 
//         counter ++;
//       } else {
//         counter --;
//       }


// }



Motor::Motor(){}

Motor::Motor(int DIRpin, int PWMpin, int DirectionInvert) {
  DIR = DIRpin;
  PWM = PWMpin;
  // Standby = STBYpin;
  this->DirectionInvert = DirectionInvert;

  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);

  // pinMode(Standby, OUTPUT);
}

//Determines which way to spin motor
void Motor::drive(int speed) {
  // digitalWrite(Standby, HIGH);
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
  digitalWrite(DIR, HIGH); // MAY BE LOW BASED ON ELEC NOT
  analogWrite(PWM, speed);
}

//Go backwards at entered speed
void Motor::rev(int speed) {
  digitalWrite(DIR, LOW); //MAY BE HIGH BASED ON ELEC NOT
  analogWrite(PWM, speed);
}

//Use motor to brake robot (CAREFUL, LOOK AT BACK EMF AND SHORTING)
void Motor::brake() {
  // digitalWrite(In1, HIGH);
  // digitalWrite(In2, HIGH);
  analogWrite(PWM, 0);
}

// no longer used
// //Idle motor (no power) (CAREFUL, LOOK AT BACK EMF AND SHORTING)
// void Motor::standby() {
//   digitalWrite(Standby, LOW);
// }

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

// no longer used
// //idle
// void Robot::standby() {
//   leftMotor.standby();
//   rightMotor.standby();
// }
