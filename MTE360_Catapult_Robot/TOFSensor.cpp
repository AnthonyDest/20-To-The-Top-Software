#include <cmath>
#include "Arduino.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "TOFSensor.h"

TOFSensor::TOFSensor():Adafruit_VL53L0X(){};

void TOFSensor::initalizeTOF(){
if(!begin()) {
    Serial.println(F("Failed to boot Second VL53L0X"));
    while(1);
  } else{
      Serial.println(F("Two Good"));
  }
}

void TOFSensor::initalizeTOF(uint8_t new_addr){
  Serial.println("Start TOF");
if(!begin(new_addr)) {
    Serial.println(F("Failed to boot First VL53L0X"));
    while(1);
  } else{
      Serial.println(F("One Good"));
  }
  Serial.println("END TOF");
}

void TOFSensor::configureResetPin() {
  pinMode(RESET_PIN, OUTPUT);
  delay(10);
}

void TOFSensor::resetOn() {
  digitalWrite(RESET_PIN, LOW);
  delay(10);
}

void TOFSensor::resetOff() {
  digitalWrite(RESET_PIN, HIGH);
  delay(10);
}

uint16_t TOFSensor::scanDistanceMM(){
  getSingleRangingMeasurement(&measure, false);
  return measure.RangeMilliMeter;
}

void TOFSensor::debounceDistance(uint16_t &scanDistance, uint16_t &scanDistanceAverage){
    if (verifyValidScanData(scanDistance)){
      if(scanDistanceAverage == 0){
        scanDistanceAverage = scanDistance;
      } else {
        scanDistanceAverage = average(scanDistanceAverage, scanDistance) ;
      }
  }
}

bool TOFSensor::verifyValidScanData(uint16_t distance){
  if(distance > 8000 and distance < 8200){ // figure out all error codes during testing
    return false; // Scan error
  } else if(distance == 65535){
    return false; // Timeout error
  }
  return true;
}

float TOFSensor::average(float inputA, float inputB) {
  return (inputA + inputB) / 2;
}