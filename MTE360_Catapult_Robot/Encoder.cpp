#include "variant.h"
#include "Encoder.h"
#include <Arduino.h>
#include "Rotary.h"

Encoder::Encoder() {}

void Encoder::rotate(){}

Encoder::Encoder(int _ENC_A, int _ENC_B, int DirectionInvertP) {
  ENC_A = _ENC_A;
  ENC_B = _ENC_B;
  DirectionInvert = DirectionInvertP;
pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  

  // rotary = Rotary(_ENC_A, _ENC_B);  //else just pass by reference

  // if (useInterrupts) {
    //  attachInterrupt(digitalPinToInterrupt(_ENC_A), EncScanActive, CHANGE);
    //  attachInterrupt(analogInputToDigitalPin(_ENC_B), EncScanActive, CHANGE);
  // }
}

// void Encoder::EncScanActive() {

//   rotaryDirection = rotary.process();

//   //needs direction invert
//   if (rotaryDirection == DIR_CW) {
//     stepCounter++;
//     // Serial.println(stepCounter);
//   } else if (rotaryDirection == DIR_CCW) {
//     stepCounter--;
//     // Serial.println(stepCounter);
    
//   }
  
//   // Serial.println("MAIN" + String(stepCounter));
// }

void Encoder::EncScanActive() {

  // If the previous and the current state of the outputA are different, that means a Pulse (swap direction) has occured
  aCurrentState = digitalRead(ENC_A);  // Reads the "current" state of the outputA
  if (aCurrentState != aPrevState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(ENC_B) != aCurrentState) {
      stepCounter++;
      // Serial.println("Forward");
    } else {
      stepCounter--;
      // Serial.println("Backward");
    }
      //  Serial.println(stepCounter);
  }
  aPrevState = aCurrentState;
 }

void Encoder::resetCounter() {
  stepCounter = 0;
}

void Encoder::resetTurnCounter(){ // can be combined to just use trip counter
  stepCounterForTurning = stepCounter;
}

void Encoder::resetTripCounter(){
  stepTripCounterBegin = stepCounter;
}


float Encoder::getDistanceMM() { //fix
  // Serial.println(float(stepCounter) / float(stepsPerWheelRotation));
  // return ((stepCounter / float(stepsPerWheelRotation) )* float(wheelDiamMM) * float(3.14159 / 10));  // replace pi with math library
  return (float(stepCounter) * float(MM_PER_STEP));
}
void Encoder::getMotorDirection() {}  // can use encoder to sample motor and verify rotation

float Encoder::getTurnAngle(){

  if (stepCounterForTurning == 0){
    stepCounterForTurning = stepCounter;
  }

  // return (stepCounter - stepCounterForTurning)/stepsPerWheelRotation; // Gets Delta steps, multiply by [angle/steps] to get angle rotated. Might be backwards
  return 0;
}


/*****************************************************************************/