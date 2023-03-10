#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
 #include "Rotary.h"

class Encoder {
public:

  Encoder();
  Encoder(int ENCA_Pin, int ENCB_Pin, int DirectionInvert);
  void rotate();
  void EncScanActive();  // this needs to be called to use encoders

  void resetCounter();  // or a "trip 1" button

  void resetTurnCounter();

  float getDistanceCM();

  void getMotorDirection();

  float getTurnAngle();

// private:

  Rotary rotary;
  unsigned char rotaryDirection;

  int stepCounter;
  int aCurrentState;
  int aPrevState;
  int ENC_A, ENC_B;
  int DirectionInvert;

  
  int stepCounterForTurning = 0;

  const int wheelDiamMM = 80;             // [mm]
  const int stepsPerWheelRotation = 358;  // Needs testing to validate [steps/360deg]
};

#endif
