#include "Gyro.h"
#define WIRE_PORT Wire
#define AD0_VAL 86
ICM_20948_I2C myICM;

Gyro::Gyro() : ICM_20948() {
}

void Gyro::setup() {
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  myICM.begin(WIRE_PORT, AD0_VAL);

  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success){
    Serial.println(F("DMP enabled!"));
  } else {
    Serial.println(F("Enable DMP failed!"));
  }
}

void Gyro::resetAllAngles(){ // aka initalize?, have own variable set to yaw and just += the yaw variable
}

bool Gyro::quaternationCalcs(){ // might need to put whole library in a polling loop, or switch to use interrupts

  myICM.readDMPdataFromFIFO(&data);

  if (((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) && ((data.header & DMP_header_bitmap_Quat6) > 0)){ 
    
    q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

    q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

    q2sqr = q2 * q2;

    return true;
  } else {
    return false;
  }

}

double Gyro::getTurnAngle(){
  
  if (quaternationCalcs()){

      t3 = +2.0 * (q0 * q3 + q1 * q2);
      t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      yaw = atan2(t3, t4) * 180.0 / PI;

      return yaw;
  } else{
    return 999; //need better error return
  }

}

