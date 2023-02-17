#ifndef TB6612_Motor_h
#define TB6612_Motor_h

#include <Arduino.h>
#include "Rotary.h"

#define FORWARD_DIR 1
#define BACKWARD_DIR -1

// const int FORWARD_DIR = 1;
// const int BACKWARD_DIR = -1;

// const int DIGITAL_PIN = 0;
// const int ANALOG_PIN = 1;

// #define SPEEDINCREMENT 50
// #define SPEEDDELAY 50

class Motor {
public:
  // Constructor. Mainly sets up pins.
  Motor();
  Motor(int DIRpin, int PWMpin, int DirectionInvert);

  // Drive in direction given by sign, at speed given by magnitude of the
  //parameter.
  void drive(int speed, int direction);

  void speedUp(int desiredSpeed, int prevSpeed, int direction);
  void slowDown(int desiredSpeed, int prevSpeed, int direction);

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
  int DIR, PWM, DirectionInvert, prevSpeed, curDirState = 0;

  const int SPEEDINCREMENT = 25;
  const int SPEEDDELAY = 250;

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


class Encoder {
public:

  Encoder();
  Encoder(int ENCA_Pin, int ENCB_Pin, int DirectionInvert);

  void EncScanActive();  // this needs to be called to use encoders

  void resetCounter(); // or a "trip 1" button
  float getDistanceCM();

  void getMotorDirection();

private:

  Rotary rotary;
  unsigned char rotaryDirection;

  int stepCounter;
  // int aCurrentState;
  // int aPrevState;
  // int ENC_A, ENC_B;
  int DirectionInvert;

  const int wheelDiamMM = 80;             // [mm]
  const int stepsPerWheelRotation = 358;  // Needs testing to calculate
};


class Robot {
public:

  Robot();
  // Robot(Motor& testMotorP);

  Robot(Motor &leftMotor, Motor &rightMotor, Encoder &leftEncoder, Encoder &rightEncoder);

  //zzDrive robot fwd, customize speed and time?) - speed only for now, all will have a 5 second oh shit timmer
  //need to determine if we want to do forward(+value) backward(+value) or just a drive and parameter symbol will determine direction (currently just doing explicity Fwd/rev)
  void forwardDrive(int speed);
  // void forwardDrive();

  void backwardDrive(int speed);
  // void backwardDrive();

  //customize more later
  void leftTurnStationary(int speed);
  //void leftTurn();

  //customize more later
  void rightTurnStationary(int speed);
  //void rightTurn();

  //Determine specifc slowing down rate (should be full stp[?])
  void brake();

  void forwardDriveDistance(int speed, float distanceCM);

  void reverseDriveDistance(int speed, float distanceCM);
  //  no longer able to access
  // //idle
  // void standby();

 private:

  float average(float inputA, float inputB);

  Motor *leftMotor;
  Motor *rightMotor;
  Encoder *leftEncoder;
  Encoder *rightEncoder;
  float initialDistance = 0, currentDistanceTravelled = 0;
};

#endif