#include "Robot.h"

Robot::Robot(){};

Robot::Robot(Motor &_leftMotor) {
}

Robot::Robot(Motor &_leftMotor, Motor &_rightMotor) {
  leftMotor = &_leftMotor;
  rightMotor = &_rightMotor;
}

Robot::Robot(Motor &_leftMotor, Motor &_rightMotor, Encoder &_leftEncoder, Encoder &_rightEncoder) {
  leftMotor = &_leftMotor;
  rightMotor = &_rightMotor;
  leftEncoder = &_leftEncoder;
  rightEncoder = &_rightEncoder;
}

Robot::Robot(Motor &_leftMotor, Motor &_rightMotor, Encoder &_leftEncoder, Encoder &_rightEncoder, TOFSensor &_botTOF, TOFSensor &_topTOF) {
  leftMotor = &_leftMotor;
  rightMotor = &_rightMotor;
  leftEncoder = &_leftEncoder;
  rightEncoder = &_rightEncoder;
  botTOF = &_botTOF;
  topTOF = &_topTOF;
}

void Robot::allConfiguration() {
  Setup_TOF_Address();
}

//Drive forward
void Robot::forwardDrive(int speed) {
  speed = abs(speed);

  // Serial.println("LEFT IN FXN: " + (unsigned long)&leftMotor);
  leftMotor->drive(speed, FORWARD_DIR);
  rightMotor->drive(speed, FORWARD_DIR);
}

//Drive backward
void Robot::backwardDrive(int speed) {
  speed = abs(speed);
  leftMotor->drive(speed, BACKWARD_DIR);
  rightMotor->drive(speed, BACKWARD_DIR);
}

//Turn left
void Robot::leftTurnStationary(int speed) {
  speed = abs(speed) / 2;
  leftMotor->drive(speed, BACKWARD_DIR);
  rightMotor->drive(speed, FORWARD_DIR);
}

//Turn right
void Robot::rightTurnStationary(int speed) {
  speed = abs(speed) / 2;
  leftMotor->drive(speed, FORWARD_DIR);
  rightMotor->drive(speed, BACKWARD_DIR);
}

// void Robot::leftTurnStationaryUsingEncoder(float encSteps) {
//   //keep turning while angle needed < cur angle + tol
//   float deltaLeft = 0; // fix temp storage
//   float deltaRight = 0;
//   leftMotor->drive(TURN_SPEED, BACKWARD_DIR);
//   rightMotor->drive(TURN_SPEED, FORWARD_DIR);
  
//   leftEncoder->resetTurnCounter();
//   rightEncoder->resetTurnCounter();

//   while(encSteps > average(deltaLeft, deltaRight)){
//   deltaLeft = abs(leftEncoder->stepCounter - leftEncoder->stepCounterForTurning);
//   deltaRight = abs(rightEncoder->stepCounter - rightEncoder->stepCounterForTurning);
//   }
//   leftMotor->brake();
//   rightMotor->brake();
// }

void Robot::leftTurnStationaryUsingEncoder(float encSteps) { // can maybe use L/R turn stationary commands but speed should be slow enough and very custom so not significant
  leftMotor->drive(TURN_SPEED, BACKWARD_DIR);
  rightMotor->drive(TURN_SPEED, FORWARD_DIR);
  travelledDistanceUsingEncoder(encSteps);
}

void Robot::rightTurnStationaryUsingEncoder(float encSteps) {
  leftMotor->drive(TURN_SPEED, FORWARD_DIR);
  rightMotor->drive(TURN_SPEED, BACKWARD_DIR);
  travelledDistanceUsingEncoder(encSteps);
}

void Robot::travelledDistanceUsingEncoder(float encSteps) {
  //keep turning while angle needed < cur angle + tol
  float deltaLeft = 0; // fix temp storage
  float deltaRight = 0;
  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();
  while(encSteps > average(deltaLeft, deltaRight)){
  deltaLeft = abs(leftEncoder->stepCounter - leftEncoder->stepTripCounterBegin);
  deltaRight = abs(rightEncoder->stepCounter - rightEncoder->stepTripCounterBegin);
  }
  leftMotor->brake();
  rightMotor->brake();
}

//Motor braking (hard stop)
void Robot::brake() {

  // slow down (need to check current direction of motors)
  leftMotor->drive(0, leftMotor->getMotorDir());
  rightMotor->drive(0, rightMotor->getMotorDir());

  // stop
  leftMotor->brake();
  rightMotor->brake();
}

void Robot::forwardDriveDistance(int speed, float distanceCM) { //currently steps

  leftMotor->drive(speed, FORWARD_DIR);
  rightMotor->drive(speed, FORWARD_DIR);

  // Serial.println(distanceCM * rightEncoder->CM_TO_STEPS);
  // travelledDistanceUsingEncoder((distanceCM)*0.80 - 100);
  travelledDistanceUsingEncoder(distanceCM - speed); // WILL NEED CALIBRATION

  // delay(5000);
  // travelledDistanceUsingEncoder(distanceCM * rightEncoder->CM_TO_STEPS);
  


  // initialDistance = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM());  // do we get average and compare end result avg, or do we just monitor each independently
  // // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
  // forwardDrive(speed);

  // while (distanceCM >= currentDistanceTravelled) {
  //   currentDistanceTravelled = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM()) - initialDistance;
  // }
  // currentDistanceTravelled = 0;
  // brake();  // will stop currently for testing, but this can be removed later to optimize
}

void Robot::reverseDriveDistance(int speed, float distanceCM) {
  initialDistance = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM());  // do we get average and compare end result avg, or do we just monitor each independently
  // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
  backwardDrive(speed);

  while (distanceCM >= currentDistanceTravelled) {
    currentDistanceTravelled = abs(average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM()) - initialDistance);
  }
  currentDistanceTravelled = 0;
  brake();  // will stop currently for testing, but this can be removed later to optimize
}

float Robot::average(float inputA, float inputB) {
  return (inputA + inputB) / 2;
}

void Robot::Setup_TOF_Address() {
  // set pinmode
  botTOF->configureResetPin();

  // Bot has reset pin, top changes address
  // reset low
  botTOF->resetOn();

  //init top with address
  topTOF->initalizeTOF(topTOFAddress);

  //reset high
  botTOF->resetOff();

  //init bot no addr
  botTOF->initalizeTOF();
}

void Robot::scanBothTOF(){
  scanDistanceBotAverage = 0;
  scanDistanceTopAverage = 0;

  for (scanCounter = 0; scanCounter < NUMBER_OF_SCANS; scanCounter++) {
  scanDistanceBot = botTOF->scanDistanceMM();
  scanDistanceTop = topTOF->scanDistanceMM();
    botTOF->debounceDistance(scanDistanceBot, scanDistanceBotAverage);
    topTOF->debounceDistance(scanDistanceTop, scanDistanceTopAverage);
    delay(SCAN_DELAY);
  }
}

bool Robot::poleFound() {
//NEED TO TEST HOW LONG AVG FUNCTION TAKES AND IF ITS WORTH IMPROVING PERFORMANCE
//Have a better tolerance with testing
scanBothTOF();
Serial.println("BOT AVG: " + String(scanDistanceBotAverage));
Serial.println("Top AVG: " + String(scanDistanceTopAverage));

delay(200);

if (scanDistanceBotAverage != 0 && scanDistanceTopAverage != 0){
  if( (scanDistanceBotAverage >= scanDistanceTopAverage*0.9 && scanDistanceBotAverage <= scanDistanceTopAverage*1.1) || (scanDistanceBotAverage >= scanDistanceTopAverage-50 && scanDistanceBotAverage <= scanDistanceTopAverage+50)   ){ //Or to account for 10% not working well at low values
    // If both values are within 10% tolerance of eachother, pole found
    //Pole found
    linearDistToPole = average(scanDistanceBotAverage,scanDistanceTopAverage) /10; // NEEDS A SCALING FACTOR TO CM VIA TESTING, should just be MM -> CM
    return true;
  } else if (scanDistanceBotAverage == 0 && scanDistanceTopAverage > 100){ // If bot errors out, only rely on top 
    linearDistToPole = scanDistanceTopAverage /10;// NEEDS A SCALING FACTOR TO CM VIA TESTING
    return true;

  } else if (scanDistanceBotAverage > 100 && scanDistanceTopAverage == 0){  // If Top errors out, only rely on top 
    linearDistToPole = scanDistanceBotAverage /10; // NEEDS A SCALING FACTOR TO CM VIA TESTING
    return true;
  }
}
 return false;
}

void Robot::searchForPole(){  //Assumes pole will always be found during full rotation, can be later optimized to do better scanning algorithm using gyro & to ignore wall
int scanCounterMax = 72; //360 rotation // temporary full circle for testing assuming 360deg/5deg
int scanCounter = 0;
   while(!poleFound() and scanCounter < scanCounterMax){
    Serial.println("Turning to scan " + String(scanCounter));
    leftTurnStationaryUsingEncoder(10); // 650 ~ 1 rotation
    scanCounter++;
    delay(500); //shorten post testing
   }
}
