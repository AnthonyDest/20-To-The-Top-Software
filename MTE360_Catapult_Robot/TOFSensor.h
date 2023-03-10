#ifndef TOFSensor_h
#define TOFSensor_h

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"
// #include "VL53L0X.h"

//no longer needed if use cont scan
// #define SCAN_DELAY 100
// #define NUMBER_OF_SCANS 10

#define SCAN_DURATION 100
#define RESET_PIN 5
// VL53L0X_RangingMeasurementData_t measure;
// Adafruit_VL53L0X adafruitSensor = Adafruit_VL53L0X();

class TOFSensor {
public:
  String name = "DEFAULT";

  TOFSensor();
  TOFSensor(int TOF_Reset_Pin);  // Have flag to set pin output so that it doesnt edit twice?
// TOFSensor(VL53L0X &passedSensor);
TOFSensor(Adafruit_VL53L0X &passedSensor, String _name, VL53L0X_RangingMeasurementData_t &_measure);
// TOFSensor(Adafruit_VL53L0X &passedSensor, String _name);
TOFSensor(String _name);


// void testAA();
  void initTOF();  // initalizes sensor
  void initTOF(uint8_t i2c_addr);

  // void ConfigureResetPin();
  void resetOn();
  void resetOff();

  void startContinuous();
  void stopContinuous();

  // void setAddress(uint8_t new_addr);
  // uint8_t getAddress();

  uint16_t scanDistance_DebouncedForTesting();
  uint16_t scanDistance(VL53L0X_RangingMeasurementData_t &pMeasure);

void scanDistanceGET();
  uint16_t scanDistance();

   void debounceSignal(uint16_t &scanDistance, uint16_t &scanDistanceAverage);

  // bool scanDataIsGood(uint16_t distance);
//  VL53L0X testSensor;
 
 private:

  // VL53L0X *sensor;
  Adafruit_VL53L0X *sensor;
  VL53L0X_RangingMeasurementData_t *measure;

  // int const RESET_PIN = 5;
  // bool resetPinConfigured = false;
  float average(float inputA, float inputB); //make an algorithms library?
};

#endif