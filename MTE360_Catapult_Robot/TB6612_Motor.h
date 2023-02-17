#ifndef TB6612_Motor_h
#define TB6612_Motor_h

#include <Arduino.h>
#include "Rotary.h"
#include "Encoder.h"

#define FORWARD_DIR 1
#define BACKWARD_DIR -1

// const int FORWARD_DIR = 1;
// const int BACKWARD_DIR = -1;

// const int DIGITAL_PIN = 0;
// const int ANALOG_PIN = 1;

#define SPEEDINCREMENT 100
#define SPEEDDELAY 50

class Motor {
public:
  // Constructor. Mainly sets up pins.
  Motor();
  Motor(int DIRpin, int PWMpin, int DirectionInvert);

  // Drive in direction given by sign, at speed given by magnitude of the
  //parameter.
  void drive(int speed, int direction);

  void speedUp(int desiredSpeed, int direction);
  void slowDown(int desiredSpeed, int direction);

  int getMotorDir();

  // drive(), but with a delay(duration)
  // void drive(int speed, int duration);

  //Stops motor by setting both input pins high
  void brake();

  //  no longer able to access
  // //set the chip to standby mode.  The drive function takes it out of standby
  // //(forward, back, left, and right all call drive)
  // void standby();

private:
  //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
  int DIR, PWM, DirectionInvert, curSpeed = 0, curDirState = 0, nextSpeed;

  // const int SPEEDINCREMENT = 25;
  // const int SPEEDDELAY = 250;

  const int MINSPEED = 50;
  const int MAXSPEED = 250;

  // enum robotSpeedDirection
  // {   forwardDir = 1,
  //     backwardDir= -1
  // };


  //private functions that spin the motor CC and CCW
  void fwd(int speed);
  void rev(int speed);
};


#endif