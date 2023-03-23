#ifndef TOFSensor_h
#define TOFSensor_h

#include <Arduino.h>
#include "Adafruit_VL53L0X.h"

#define RESET_PIN 5
#define topTOFAddress 0x30

#define SCAN_DELAY 50
#define NUMBER_OF_SCANS 5

class TOFSensor : public Adafruit_VL53L0X {
public:

  TOFSensor();
  // TOFSensor(uint8_t new_addr);
  void initalizeTOF();
  void initalizeTOF(uint8_t new_addr);
void configureResetPin();
void resetOn();
void resetOff();

uint16_t scanDistanceMM();
void debounceDistance(uint16_t &scanDistance, uint16_t &scanDistanceAverage);
bool verifyValidScanData(uint16_t distance);

private:

VL53L0X_RangingMeasurementData_t measure;
float average(float inputA, float inputB); //make an algorithms library?

};

#endif