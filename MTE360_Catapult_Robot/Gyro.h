#ifndef Gyro_h
#define Gyro_h
#include "ICM_20948.h"
#include <Arduino.h>
#define CW_DIR 1
#define CCW_DIR -1

class Gyro : public ICM_20948_I2C {
public:

  Gyro();

  void setup();

  void resetAllAngles();

  double getTurnAngle();

  void updateAllAngles();

  void firstStabalizeGyroValues();

  void resetTripCounter();
  double startTurnHeading = 0;
  // double deltaTurnAngle = 0;


// private:

  icm_20948_DMP_data_t data;
  //ICM_20948_I2C myICM;
  // double lastGoodValue = 0;

  bool quaternationCalcs();

  bool success = 0;

  double q1 = 0;
  double q2 = 0;
  double q3 = 0;

  double q0 = 0;
  double q2sqr = 0;

  // roll (x-axis rotation)
  double t0 = 0;
  double t1 = 0;
  double roll = 0;
  // pitch (y-axis rotation)
  double t2 = 0;
  double pitch = 0;

  // yaw (z-axis rotation)
  double t3 = 0;
  double t4 = 0;
  double yaw = 0;
  double nextYaw = 0;

  float acc_x =0;
  float acc_y =0;
  float acc_z = 0;

  float acc_x_change =0;
  float acc_y_change =0;
  float acc_z_change = 0;


float average(float inputA, float inputB);
  double lastValidYaw = 0;
void wait(double MS);

};


#endif
