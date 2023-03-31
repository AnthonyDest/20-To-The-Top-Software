#include "wiring_constants.h"
#include <cmath>
#include "Gyro.h"
#define WIRE_PORT Wire
#define AD0_VAL 86
// ICM_20948_I2C myICM;




Gyro::Gyro()
  : ICM_20948_I2C(){};

void Gyro::setup() {
  // WIRE_PORT.begin();
  // WIRE_PORT.setClock(400000);
  begin(WIRE_PORT, AD0_VAL);

  bool success = true;
  // success &= (initializeDMP() == ICM_20948_Stat_Ok);
  // success &= (enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  
  // success &= (enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  // success &= (enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  // success &= (enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
  

  // success &= (setDMPODRrate(DMP_ODR_Reg_Quat6, 10) == ICM_20948_Stat_Ok);        // Set to 5Hz
  // success &= (setDMPODRrate(DMP_ODR_Reg_Accel, 10) == ICM_20948_Stat_Ok);        // Set to 1Hz
  // success &= (setDMPODRrate(DMP_ODR_Reg_Gyro, 54) == ICM_20948_Stat_Ok);         // Set to 1Hz
  // success &= (setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  // success &= (setDMPODRrate(DMP_ODR_Reg_Cpass, 54) == ICM_20948_Stat_Ok);        // Set to 1Hz
  // success &= (setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 54) == ICM_20948_Stat_Ok); // Set to 1Hz

  // success &= (enableFIFO() == ICM_20948_Stat_Ok);
  // success &= (enableDMP() == ICM_20948_Stat_Ok);
  // success &= (resetDMP() == ICM_20948_Stat_Ok);
  // success &= (resetFIFO() == ICM_20948_Stat_Ok);

success &= (initializeDMP() == ICM_20948_Stat_Ok);
  success &= (enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok);
  success &= (enableFIFO() == ICM_20948_Stat_Ok);
  success &= (enableDMP() == ICM_20948_Stat_Ok);
  success &= (resetDMP() == ICM_20948_Stat_Ok);
  success &= (resetFIFO() == ICM_20948_Stat_Ok);
  success &= (setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz


  success &= (enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (setDMPODRrate(DMP_ODR_Reg_Accel, 54) == ICM_20948_Stat_Ok);  // Set to 1Hz
  success &= (enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);


  if (success) {
    Serial.println(F("DMP enabled!"));
  } else {
    Serial.println(F("Enable DMP failed!"));
  }
  // wait(5000);



}

void Gyro::firstStabalizeGyroValues() {
bool ledState = true;

  while (abs(yaw) < 0.1) {
    //Serial.println(getTurnAngle());
    updateAllAngles();
    // digitalWrite(LED_BUILTIN, ledState);
    ledState = !ledState;
  if(ledState){
digitalWrite(LED_BUILTIN, HIGH);
  }else{
digitalWrite(LED_BUILTIN, LOW);    
  }
wait(50);
    Serial.println("X: " +  String(acc_x) + "," + String(acc_x_change) + " Y: " + String(acc_y) + "," + String(acc_y_change) +  " Z: " + String(acc_z)  + "," + String(acc_z_change)+ " Roll: " +  String(roll) + " Pitch: " + String(pitch) +  " Yaw: " + String(yaw));

  }

  for (int i = 0; i<100; i++) {
  updateAllAngles();
  }




}

void Gyro::resetAllAngles() {  // aka initalize?, have own variable set to yaw and just += the yaw variable
}

// The 6-Axis and 9-axis Quaternion outputs each consist of 12 bytes of data.
// These 12 bytes in turn consists of three 4-byte elements.
// 9-axis quaternion data and Geomag rv is always followed by 2-bytes of heading accuracy, hence the size of Quat9 and Geomag data size in the FIFO is 14 bytes.
// Quaternion data for both cases is cumulative/integrated values.
// For a given quaternion Q, the ordering of its elements is {Q1, Q2, Q3}.
// Each element is represented using Big Endian byte order.
// Q0 value is computed from this equation: Q20 + Q21 + Q22 + Q23 = 1.
// In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
// The quaternion data is scaled by 2^30.


bool Gyro::quaternationCalcs() {  // might need to put whole library in a polling loop, or switch to use interrupts - doesnt appear to need polling, can just call when needed

  readDMPdataFromFIFO(&data);

  if (((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) && ((data.header & DMP_header_bitmap_Quat6) > 0)) {

    q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
    q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
    q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

    q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    // if(isnan(q0)){
    //   Serial.print("Q1 " + String(q1*q1));
    //   Serial.print(" ");
    //   Serial.print("q2 " + String(q2*q2));
    //    Serial.print(" ");
    //   Serial.print("q3 " + String(q3*q3));

    //  Serial.print(" ");
    //   Serial.print("sumproduct " + String(((q1 * q1) + (q2 * q2) + (q3 * q3))));
    //    Serial.println();
    //   // }
    //   delay(2000);

    q2sqr = q2 * q2;

    return true;
  } else {
    return false;
  }
}

void Gyro::updateAllAngles() {

  readDMPdataFromFIFO(&data);
   success = 0; 
  // if (((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) && ((data.header & DMP_header_bitmap_Quat6) > 0)) {
  if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
    {
      q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      q2sqr = q2 * q2;

      // roll (x-axis rotation)
      t0 = +2.0 * (q0 * q1 + q2 * q3);
      t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      t3 = +2.0 * (q0 * q3 + q1 * q2);
      t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);

      nextYaw = atan2(t3, t4) * 180.0 / PI;
      // if(!isnan(nextYaw) and abs(yaw-nextYaw) <50){
      //   yaw = nextYaw;
      // }
      if (!isnan(nextYaw)) {
        yaw = nextYaw;
      }

      // float acc_x_new = (float)data.Raw_Accel.Data.X;  // Extract the raw accelerometer data
      // float acc_y_new  = (float)data.Raw_Accel.Data.Y;
      // float acc_z_new = (float)data.Raw_Accel.Data.Z;

      // acc_x_change = (acc_x - acc_x_new);  // record change
      // acc_y_change = (acc_y - acc_y_new);
      // acc_z_change = (acc_z - acc_z_new);

   

      // acc_x = (acc_x*0.5 + acc_x_new*0.5);  // average new data
      // acc_y = (acc_y*0.5 + acc_y_new*0.5); 
      // acc_z = (acc_z*0.5 + acc_z_new*0.5); 
      success = true;


//      acc_x = getAccMG(acc_x);
//      acc_y = getAccMG(acc_y);
//      acc_z = getAccMG(acc_z);

      // getAGMT();
      // acc_x =
      //       acc_x = (float)acc.axes.x; // Extract the raw accelerometer data
      //       acc_y = (float)acc.axes.y;
      //       acc_z = (float)acc.axes.xz;
      //       // Serial.println(acc_x);
    }
  }
}


double Gyro::getTurnAngle() {

  readDMPdataFromFIFO(&data);

  // if (((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail)) && ((data.header & DMP_header_bitmap_Quat6) > 0)) {
  if ((status == ICM_20948_Stat_Ok) || (status == ICM_20948_Stat_FIFOMoreDataAvail))  // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0)  // We have asked for GRV data so we should receive Quat6
    {
      q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0;  // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;  // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;  // Convert to double. Divide by 2^30

      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      q2sqr = q2 * q2;
      t3 = +2.0 * (q0 * q3 + q1 * q2);
      t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);

      nextYaw = atan2(t3, t4) * 180.0 / PI;
      // if(!isnan(nextYaw) and abs(yaw-nextYaw) <50){
      //   yaw = nextYaw;
      // }
      if (!isnan(nextYaw)) {
        yaw = nextYaw;
      }
    }
  }
  //  if (status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  // {
  //   wait(10);
  // }
  return yaw;
}

void Gyro::resetTripCounter() {
  startTurnHeading = getTurnAngle();
}

// double Gyro::getTurnAngle() {
//   int errorCounter = 0;
//   int avgYaw = 0;
//   bool hasNewValue = false;


//   // for (int i = 0; i < 5; i++) {

//   //   if (quaternationCalcs()) {
//   //     t3 = +2.0 * (q0 * q3 + q1 * q2);
//   //     t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
//   //     yaw = atan2(t3, t4) * 180.0 / PI;

//   //     if (!isnan(yaw)) {
//   //       if (yaw < lastValidYaw + 30 or yaw > lastValidYaw - 30) {
//   //         lastValidYaw = yaw;
//   //         if (avgYaw == 0) {
//   //           avgYaw = yaw;
//   //           hasNewValue = true;
//   //         } else {
//   //           avgYaw = average(avgYaw, yaw);
//   //         }
//   //       }
//   //     }
//   //   }
//   // }

//   // if(hasNewValue){
//   //   return avgYaw;
//   // }
// delay(10);
// if (quaternationCalcs()) {
//   delay(10);
//       t3 = +2.0 * (q0 * q3 + q1 * q2);
//       t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
//       yaw = atan2(t3, t4) * 180.0 / PI;

//       // if(isnan(yaw)){
//       //   Serial.println("NAN.  q0: " + String(q0) + "  q3: " + String(q3) + "  q1: "+ String(q1) + "  q2: " + String(q2) );
//       //   // delay(5000);
//       //   // return -900;
//       //   return lastValidYaw; // error case
//       // } else {

//       //   // return yaw;
//       // }

//       if(!isnan(yaw) and (yaw < (lastValidYaw +5)) and (yaw > (lastValidYaw-5))){ //data good, will need to make a modulo for the tolerancing
//         lastValidYaw = yaw;
//         return yaw;
//         }


// }
//   return lastValidYaw; // error case  //need better error return, return last valid heading as it should balance out
// }


float Gyro::average(float inputA, float inputB) {
  return (inputA + inputB) / 2;
}

void Gyro::wait(double MS) {
  double startTime = millis();
  while ((millis() - startTime) < MS) {};
}
