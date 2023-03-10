#include "USB/USBAPI.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "TOFSensor.h"


  // VL53L0X_RangingMeasurementData_t _measure;
// Adafruit_VL53L0X adafruitSensor = Adafruit_VL53L0X();

TOFSensor::TOFSensor() {
  // initTOF();
}

// TOFSensor::TOFSensor(int _TOF_Reset_Pin) {
//   resetPin = _TOF_Reset_Pin;
//   pinMode(resetPin, OUTPUT);
//   initTOF();
// }
TOFSensor::TOFSensor(Adafruit_VL53L0X &passedSensor, String _name, VL53L0X_RangingMeasurementData_t &_measure){
// TOFSensor::TOFSensor(String _name){
  pinMode(RESET_PIN, OUTPUT);
  name = _name;
  measure = &_measure;
sensor = &passedSensor;


}

void TOFSensor::initTOF() {
  
  sensor->begin();
  // adafruitSensor.begin();
}

void TOFSensor::initTOF(uint8_t i2c_addr) {
  
  sensor->begin(i2c_addr);
  // adafruitSensor.begin(i2c_addr);
}
  // Serial.println("INIT START");
  // VL53L0X pSensor;
  // sensor = &pSensor;

  // sensor->setTimeout(500);
  // if (!sensor->init()) {
  //   Serial.println("Failed to detect and initialize sensor!");
  //   while (1) {}
  // }
  // Serial.println("INIT END");
// }

// void TOFSensor::testAA() {
// sensor->setTimeout(500);
//   if (!sensor->init()) {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1) {}
//   }
//   Serial.println("INIT END");
// }

void TOFSensor::resetOn() {
   digitalWrite(RESET_PIN, LOW);
   delay(10);
}

void TOFSensor::resetOff() {
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
  // initTOF();
  //initalize sensor
}

// void TOFSensor::setAddress(uint8_t new_addr) {
//   // sensor->setAddress(new_addr);
// }

// uint8_t TOFSensor::getAddress() {
//   // return sensor->getAddress();
//   return 0;
// }

void TOFSensor::startContinuous(){
  // adafruitSensor.startRangeContinuous(SCAN_DURATION);
}

// void TOFSensor::stopContinuous(){
//   sensor->stopRangeContinuous();
// }

void TOFSensor::scanDistanceGET(){
// adafruitSensor.rangingTest(&measure, false);
}

uint16_t TOFSensor::scanDistance(){

sensor->rangingTest(measure, false);

// Serial.print(F("1: "));
Serial.print("\n name " + (unsigned int)measure);
  if(measure->RangeStatus != 4) {     // if not out of range
    // Serial.print(measure1.RangeMilliMeter);
    return measure->RangeMilliMeter;
    
  } else {
    Serial.print(F("Out of range"));
  }

  return 0;

}


uint16_t TOFSensor::scanDistance(VL53L0X_RangingMeasurementData_t &pMeasure){

sensor->rangingTest(&pMeasure, false);

// Serial.print(F("1: "));
Serial.print("\n name " + (unsigned int)&pMeasure);
  if(pMeasure.RangeStatus != 4) {     // if not out of range
    // Serial.print(measure1.RangeMilliMeter);
    return pMeasure.RangeMilliMeter;
    
  } else {
    Serial.print(F("Out of range"));
  }

  return 0;

}


// uint16_t TOFSensor::scanDistanceCONTINIOUS(){

// if (sensor->isRangeComplete()) {
//     // Serial.print("Distance in mm: ");
//     // Serial.println(name + " " + sensor->readRange());
//     return sensor->readRange();
//   }
//   // will debounce in other function, and will ignore 8000-8010 range & 65535
// // return sensor->readRangeSingleMillimeters(); // For linear distance, will need to use trig based on TOF angle, will do post testing and sensors confirmed good
//   return 0;
// }

// bool TOFSensor::scanDataIsGood(uint16_t distance){

//   if(distance > 8000 and distance < 8010){
//     return false; // Scan error
//   }

//   if(distance == 65535){
//     return false; // Timeout error
//   }

//   return true;

// }

// void TOFSensor::debounceSignal(uint16_t &scanDistance, uint16_t &scanDistanceAverage){

//     //validate and average data
//     if (scanDataIsGood(scanDistance)){
//       if(scanDistanceAverage == 0){
//         scanDistanceAverage = scanDistance;        
//       } else {
//         scanDistanceAverage = average(scanDistanceAverage, scanDistance);
//       }
//   }
// }

float TOFSensor::average(float inputA, float inputB) {
  return (inputA + inputB) / 2;
}