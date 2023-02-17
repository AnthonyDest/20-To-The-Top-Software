#include "HardwareSerial.h"
#include "TB6612_Motor.h"

#include <Arduino.h>
#include "Encoder.h"

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
void Motor::speedUp(int desiredSpeed, int direction) {

  while (curSpeed < desiredSpeed) {

    nextSpeed += SPEEDINCREMENT;

    // redundant input checks
    if (nextSpeed > desiredSpeed) {
      nextSpeed = desiredSpeed;
    }

    if (nextSpeed < MINSPEED) {
      nextSpeed = MINSPEED;
    }

    if (nextSpeed > MAXSPEED) {
      nextSpeed = MAXSPEED;
    }

    if (direction == FORWARD_DIR) {
      fwd(nextSpeed);
    } else {
      rev(nextSpeed);
    }

    delay(SPEEDDELAY);
  }
}
//may want to remove while if the time is too long
void Motor::slowDown(int desiredSpeed, int direction) {

  while (curSpeed > desiredSpeed) {

    nextSpeed -= SPEEDINCREMENT;

    // redundant input checks
    if (nextSpeed < desiredSpeed) {
      nextSpeed = desiredSpeed;
    }

    if (nextSpeed < MINSPEED) {
      nextSpeed = 0;
    }

    if (direction == FORWARD_DIR) {
      fwd(nextSpeed);
    } else {
      rev(nextSpeed);
    }

    delay(SPEEDDELAY);
  }
}

//Determines which way to spin motor and how to scale speed
void Motor::drive(int desiredSpeed, int direction) {

  direction = direction * DirectionInvert;

  if (abs(curSpeed) > abs(desiredSpeed)) {  // slowing down

    slowDown(desiredSpeed, direction);

  } else if (abs(curSpeed) < abs(desiredSpeed)) {  // speeding up

    speedUp(desiredSpeed, direction);
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
  // stopped state?
  // digitalWrite(In1, HIGH);
  // digitalWrite(In2, HIGH);
  slowDown(0, curDirState);
  analogWrite(PWM, 0);
}

// no longer used
// //Idle motor (no power) (CAREFUL, LOOK AT BACK EMF AND SHORTING)
// void Motor::standby() {
//   digitalWrite(Standby, LOW);
// }

/***********************************************************************/