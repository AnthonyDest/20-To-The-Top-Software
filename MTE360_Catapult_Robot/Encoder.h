#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include "Rotary.h"

// #define CMperStep (float(wheelDiamMM)/10)*3.14159/float(stepsPerWheelRotation) //get actual values here once finalized
// #define CM_TO_STEPS (3.14159*80/10)/()

class Encoder {
public:

  Encoder();
  Encoder(int ENCA_Pin, int ENCB_Pin, int DirectionInvert);
  void rotate();
  void EncScanActive();  // this needs to be called to use encoders

  void resetCounter();  // or a "trip 1" button

  void resetTurnCounter(); //remove?
  void resetTripCounter();

  float getDistanceCM();

  void getMotorDirection();

  float getTurnAngle();

//  const float CMperStep = (float(wheelDiamMM)/10)*3.14159/float(stepsPerWheelRotation); 
// const float CM_TO_STEPS = 1/CMperStep;

 int stepCounter; // make a getter
  int stepCounterForTurning = 0;
  int stepTripCounterBegin = 0;

 private:

  Rotary rotary;
  unsigned char rotaryDirection;

  int aCurrentState;
  int aPrevState;
  int ENC_A, ENC_B;
  int DirectionInvert;
  const int wheelDiamMM = 80;             // [mm]
  const int stepsPerWheelRotation =  650; //358;  // Needs testing to validate [steps/360deg]
};

#endif
