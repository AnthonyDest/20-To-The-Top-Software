#ifndef TB6612_Motor_h
#define TB6612_Motor_h

#include <Arduino.h>
// #include "Rotary.h"
#include "Encoder.h"
#include "ArduPID.h"

#define FORWARD_DIR 1
#define BACKWARD_DIR -1

// const int FORWARD_DIR = 1;
// const int BACKWARD_DIR = -1;

// const int DIGITAL_PIN = 0;
// const int ANALOG_PIN = 1;

#define SPEEDINCREMENT 100
#define SPEEDDELAY 50
#define TURN_SPEED 50

class Motor: public ArduPID {
public:
  // Constructor. Mainly sets up pins.
  Motor();
  // Motor(int DIRpin, int PWMpin, int DirectionInvert);

  Motor(int DIRpin, int PWMpin, int DirectionInvert);

const double p = 16.2039;
const double i = 20.2906;
const double d = 0;



  // void setupPID(double &input, double &output, double &setpoint, double p, double i, double d);
    void setupPID(double &input, double &output, double &setpoint, double &maxSpeed);


  double deltaLeft = 0;
  double deltaRight = 0;


  // Drive in direction given by sign, at speed given by magnitude of the
  //parameter.
  void drive(int speed, int direction);

  int getMotorDir();


  //Stops motor by setting both input pins high
  void brake();


// private:
  //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
  int DIR, PWM, DirectionInvert, curSpeed = 0, curDirState = 0, nextSpeed;


  const int MINSPEED = 50;
  const int MAXSPEED = 250;


  //private functions that spin the motor CC and CCW
  void fwd(int speed);
  void rev(int speed);
};


#endif
