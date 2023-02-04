#ifndef TB6612_Motor_h
#define TB6612_Motor_h

#include <Arduino.h>

//used in some functions so you don't have to send a speed
#define DEFAULTSPEED 255

class Motor {
public:
  // Constructor. Mainly sets up pins.
  Motor();
  Motor(int DIRpin, int PWMpin, int DirectionInvert);

  // Drive in direction given by sign, at speed given by magnitude of the
  //parameter.
  void drive(int speed);

  // drive(), but with a delay(duration)
  void drive(int speed, int duration);

  //Stops motor by setting both input pins high
  void brake();

  //  no longer able to access
  // //set the chip to standby mode.  The drive function takes it out of standby
  // //(forward, back, left, and right all call drive)
  // void standby();

private:
  //variables for the 2 inputs, PWM input, Offset value, and the Standby pin
  int DIR, PWM, DirectionInvert;

  //private functions that spin the motor CC and CCW
  void fwd(int speed);
  void rev(int speed);
};


// class Encoder{
//   public:

//   Encoder();
//   Encoder(int ENCA_Pin, int ENCB_Pin);

//   void getDistance();

//   private:

//  int stepCounter; 
//  int currentState;
//  int prevState;
//  const int wheelRadius;
//  const int stepsPerWheelRotation;
  
// };


class Robot{
  public:
  
  Robot();
  Robot(Motor leftMotor, Motor rightMotor);

  //zzDrive robot fwd, customize speed and time?) - speed only for now, all will have a 5 second oh shit timmer
  //need to determine if we want to do forward(+value) backward(+value) or just a drive and parameter symbol will determine direction (currently just doing explicity Fwd/rev)
  void forwardDrive(int speed);
  void forwardDrive();

  void backwardDrive(int speed);
  void backwardDrive();

  //customize more later
  void leftTurn(int speed);
  //void leftTurn();

  //customize more later
  void rightTurn(int speed);
  //void rightTurn();

  //Determine specifc slowing down rate (should be full stp[?])
  void brake();

  //  no longer able to access
  // //idle
  // void standby();
  private:
    Motor leftMotor, rightMotor;
};

#endif