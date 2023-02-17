#ifndef Robot_h
#define Robot_h

#include "TB6612_Motor.h"

class Robot {
public:

  Robot();
  // Robot(Motor& testMotorP);

  Robot(Motor &leftMotor);
  Robot(Motor &leftMotor, Motor &rightMotor);

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

  //  private:

  float average(float inputA, float inputB);

  Motor *leftMotor;
  Motor *rightMotor;
  Encoder *leftEncoder;
  Encoder *rightEncoder;
  float initialDistance = 0, currentDistanceTravelled = 0;
};
#endif