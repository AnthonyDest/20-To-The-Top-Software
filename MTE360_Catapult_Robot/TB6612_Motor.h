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

//turning
double p_pole = 16.2039;
double i_pole =20.2906;
double d_pole = 0;


//turning
double p = 16.2039/400;
double i =20.2906/2;
double d = 0;


//driving
double pCustom = 16.2039/400;
double iCustom = 20.2906/2;
double dCustom = 0;

  // double pCustom = 4.05;
  // double iCustom = 5.07;
  // double dCustom = 0;

void setupPIDPole(double &input, double &output, double &setpoint, double &maxSpeed);
  // void setupPID(double &input, double &output, double &setpoint, double p, double i, double d);
    void setupPID(double &input, double &output, double &setpoint, double &maxSpeed);
    // void setupPID_CUSTOM(double &input, double &output, double &setpoint, double &maxSpeed, double &p, double &i, double &d);
    void setupPID_CUSTOM(double &input, double &output, double &setpoint, double &maxSpeed);

  double deltaLeft = 0;
  double deltaRight = 0;


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

// private:
  //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
  int DIR, PWM, DirectionInvert, curSpeed = 0, curDirState = 0, nextSpeed;

  // const int SPEEDINCREMENT = 25;
  // const int SPEEDDELAY = 250;

  const int MINSPEED = 0;
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