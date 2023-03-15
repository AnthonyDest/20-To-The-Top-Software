#ifndef Robot_h
#define Robot_h

#include "TB6612_Motor.h"
#include "Encoder.h"
#include "TOFSensor.h"

class Robot {
public:

  Robot();
  // Robot(Motor& testMotorP);

  Robot(Motor &leftMotor);
  Robot(Motor &leftMotor, Motor &rightMotor);
  Robot(Motor &leftMotor, Motor &rightMotor, Encoder &leftEncoder, Encoder &rightEncoder);
  Robot(Motor &leftMotor, Motor &rightMotor, Encoder &leftEncoder, Encoder &rightEncoder, TOFSensor &botTOF, TOFSensor &topTOF);

 


  void allConfiguration();  //Used to configure everything

  //zzDrive robot fwd, customize speed and time?) - speed only for now, all will have a 5 second oh shit timmer
  //need to determine if we want to do forward(+value) backward(+value) or just a drive and parameter symbol will determine direction (currently just doing explicity Fwd/rev)
  void forwardDrive(int speed);
  // void forwardDrive();

  void backwardDrive(int speed);
  // void backwardDrive();

  //customize more later
  void leftTurnStationary(int speed);
  //void leftTurn() steer;

  //customize more later
  //need a turn using encoder function
  void rightTurnStationary(int speed);
  //void rightTurn();

  void leftTurnStationaryUsingEncoder(float angle);
  void rightTurnStationaryUsingEncoder(float angle);
  void travelledDistanceUsingEncoder(float steps);
  
  void travelledDistanceUsingEncoder(float stepToTravel, int speed);

  void testPIDDriveEncoderStepCount(double stepToTravel);
  


  //Determine specifc slowing down rate (should be full stp[?])
  void brake();

  void forwardDriveDistance(int speed, float distanceCM);
  void reverseDriveDistance(int speed, float distanceCM);

  // void forwardDriveDistanceENCMonitor(int speed, float distanceCM);

  // TOF Sensor

  void Setup_TOF_Address();
  void scanBothTOF();
  bool poleFound(); //aka poleFound check if facing pole
  void searchForPole();

  uint16_t linearDistToPole = 0;

// private:

  float average(float inputA, float inputB);

  Motor *leftMotor;
  Motor *rightMotor;
  Encoder *leftEncoder;
  Encoder *rightEncoder;
  TOFSensor *botTOF;
  TOFSensor *topTOF;

  uint16_t scanDistanceBot = 0, scanDistanceTop = 0;
  uint16_t scanDistanceBotAverage = 0, scanDistanceTopAverage = 0;
  int scanCounter;

  float initialDistance = 0, currentDistanceTravelled = 0;
};
#endif