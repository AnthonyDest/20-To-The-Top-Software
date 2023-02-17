#include "Encoder.h"
#include "Arduino.h"
#include "Rotary.h"

Encoder::Encoder() {}

Encoder::Encoder(int _ENC_A, int _ENC_B, int DirectionInvertP) {
  // ENC_A = ENCA_Pin;
  // ENC_B = ENCB_Pin;
  DirectionInvert = DirectionInvertP;

  rotary = Rotary(_ENC_A, _ENC_B);  //else just pass by reference

  // pinMode(ENC_A, INPUT);
  // pinMode(ENC_B, INPUT);

  // aPrevState = digitalRead(ENC_A);
}

void Encoder::EncScanActive() {

  rotaryDirection = rotary.process();

  //needs direction invert
  if (rotaryDirection == DIR_CW) {
    stepCounter++;
    Serial.println(stepCounter);
  } else if (rotaryDirection == DIR_CCW) {
    stepCounter--;
    Serial.println(stepCounter);
  }
}

// void Encoder::EncScanActive() {

//   // If the previous and the current state of the outputA are different, that means a Pulse (swap direction) has occured
//   aCurrentState = digitalRead(ENC_A);  // Reads the "current" state of the outputA
//   if (aCurrentState != aPrevState) {
//     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//     if (digitalRead(ENC_B) != aCurrentState) {
//       stepCounter++;
//       // Serial.println("Forward");
//     } else {
//       stepCounter--;
//       // Serial.println("Backward");
//     }
//     //  Serial.println(stepCounter);
//   }
//   aPrevState = aCurrentState;
// }

void Encoder::resetCounter() {
  stepCounter = 0;
}

float Encoder::getDistanceCM() {

  // Serial.println(float(stepCounter) / float(stepsPerWheelRotation));
  // return ((stepCounter / float(stepsPerWheelRotation) )* float(wheelDiamMM) * float(3.14159 / 10));  // replace pi with math library
  return (float(stepCounter) / float(stepsPerWheelRotation));
}
void Encoder::getMotorDirection() {}  // can use encoder to sample motor and verify rotation


/*****************************************************************************/