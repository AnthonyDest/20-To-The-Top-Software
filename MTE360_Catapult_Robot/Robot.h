#ifndef Robot_h
#define Robot_h

#include "TB6612_Motor.h"
#include "Encoder.h"
#include "TOFSensor.h"
#include "Adafruit_VL53L0X.h"
#include "Gyro.h"

// #include <SPI.h>
// #include <SD.h>

class Robot {
public:

  Robot();
  // Robot(Motor& testMotorP);

  Robot(Motor &leftMotor);
  Robot(Motor &leftMotor, Motor &rightMotor);
  Robot(Motor &leftMotor, Motor &rightMotor, Encoder &leftEncoder, Encoder &rightEncoder);
  Robot(Motor &leftMotor, Motor &rightMotor, Encoder &leftEncoder, Encoder &rightEncoder, TOFSensor &botTOF, TOFSensor &topTOF);
  Robot(Motor &leftMotor, Motor &rightMotor, Encoder &leftEncoder, Encoder &rightEncoder, TOFSensor &botTOF, TOFSensor &topTOF, Gyro &gyro);


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

  // void leftTurnStationaryUsingEncoder(float angle);
  // void rightTurnStationaryUsingEncoder(float angle);
  // void travelledDistanceUsingEncoder(float steps);
  // void travelledDistanceUsingEncoder(float stepToTravel, int speed);
  // void testPIDDriveEncoderStepCount(double stepToTravel);

  void leftTurnStationaryPID(double steps);
  void rightTurnStationaryPID(double steps);
  void forwardDrivePID(double distanceMM);
  void reverseDrivePID(double distanceMM);
  //private
  //void drivePID(double steps, int leftMotorDir, int rightMotorDir);

  //Determine specifc slowing down rate (should be full stp[?])
  void brake();

  void forwardDriveDistance(int speed, float distanceCM);
  void reverseDriveDistance(int speed, float distanceCM);

  void turnToHeading(double heading);
  void driveForwardAtCurrentHeading(double distanceMM);

  void driveForwardAtCurrentHeadingWithPID(double distanceMM, double maxSpeed);
  
  // void forwardDriveDistanceENCMonitor(int speed, float distanceCM);

  // TOF Sensor

double getOrientationAngle();

  void Setup_TOF_Address();
  void scanBothTOF();
  bool poleFound(); //aka poleFound check if facing pole
  bool searchForPole(int scanDirection, int degreesToScan);

  uint16_t linearDistToPole = 0;
bool driveDistanceTracking(double distanceCM);

void turnDeltaAngleGyro(double angle, int leftDir, int rightDir);
//  private:
void wait(double MS);
  float average(float inputA, float inputB);
  void drivePID(double steps, int leftMotorDir, int rightMotorDir, double maxSpeed);

  Motor *leftMotor;
  Motor *rightMotor;
  Encoder *leftEncoder;
  Encoder *rightEncoder;
  TOFSensor *botTOF;
  TOFSensor *topTOF;
  Gyro *gyro;

  uint16_t scanDistanceBot = 0, scanDistanceTop = 0;
  uint16_t scanDistanceBotAverage = 0, scanDistanceTopAverage = 0;
  int scanCounter;

  double deltaLeft = 0; 
  double deltaRight = 0;
  double averageSteps = 0;

  float initialDistance = 0, currentDistanceTravelled = 0;
  double deltaTurnAngle = 0;

  // File fileSD;
  // void setupSDCard();
  // void log(String dataToLog);
};
#endif