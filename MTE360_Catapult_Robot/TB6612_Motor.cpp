#include "HardwareSerial.h"
#include "TB6612_Motor.h"

#include <Arduino.h>
#include "Encoder.h"

Motor::Motor():ArduPID() {
}




Motor::Motor(int _DIR, int _PWM, int _DirectionInvert):ArduPID(){
  DIR = _DIR;
  PWM = _PWM;
  DirectionInvert = _DirectionInvert;
  pinMode(DIR, OUTPUT);
  pinMode(PWM, OUTPUT);

}


// void Motor::setupPID(double &input, double &output, double &setpoint, double p, double i, double d){
void Motor::setupPID(double &input, double &output, double &setpoint, double &maxSpeed){
  begin(&input, &output, &setpoint, p, i, d);
  setOutputLimits(30, maxSpeed);
  setSampleTime(1);
  setWindUpLimits(-10, 10); // Groth bounds for the integral term to prevent integral wind-up
  start();
  }




//Determines which way to spin motor and how to scale speed
void Motor::drive(int desiredSpeed, int direction) {

  direction = direction * DirectionInvert;

  if (desiredSpeed > 250){ //speedmax
  desiredSpeed = 250;
  } else if (desiredSpeed < 40){ //speedmin
  desiredSpeed = 40;
  }

if (direction == FORWARD_DIR) {
      fwd(desiredSpeed);
    } else if (direction == BACKWARD_DIR) {
      rev(desiredSpeed);
    }
}


int Motor::getMotorDir() {
  return curDirState;
}


//Go Forward at entered speed
void Motor::fwd(int speed) {
  curDirState = FORWARD_DIR;
  curSpeed = speed;
  digitalWrite(DIR, HIGH);  // MAY BE LOW BASED ON ELEC NOT
  analogWrite(PWM, speed);
}

//Go backwards at entered speed
void Motor::rev(int speed) {
  curDirState = BACKWARD_DIR;
  curSpeed = speed;
  digitalWrite(DIR, LOW);  //MAY BE HIGH BASED ON ELEC NOT
  analogWrite(PWM, speed);
}


//Use motor to brake robot (CAREFUL, LOOK AT BACK EMF AND SHORTING)
void Motor::brake() {

  analogWrite(PWM, 0);
}
