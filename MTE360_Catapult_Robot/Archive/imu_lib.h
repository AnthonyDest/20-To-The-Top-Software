#include <Wire.h>       // For I2C communication
#include "ICM_20948.h" //IMU library

#define WIRE_PORT Wire
#define AD0_VAL 1

class IMU {
public:
  IMU();
  IMU(ICM_20948_I2C IMU_in);

  int getOrientation(int num);
private:
  ICM_20948_I2C IMU_obj; 
};