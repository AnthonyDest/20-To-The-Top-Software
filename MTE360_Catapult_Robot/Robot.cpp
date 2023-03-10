#include "Robot.h"

Robot::Robot(){};

// Robot::Robot(Motor& testMotorP){

//   // testMotor -> testMotorP;
//   // Serial.println("bb" + (unsigned long)&testMotorP);
// }

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

//Stationary turn using encoder
// void Robot::leftTurnStationaryUsingEncoder(float angle) {
// float testAvg = 0;
//   //Steps for 360 deg = 200 steps
//   //keep turning while angle needed < cur angle + tol
//   leftMotor->drive(TURN_SPEED, BACKWARD_DIR);
//   rightMotor->drive(TURN_SPEED, FORWARD_DIR);

//   leftEncoder->resetTurnCounter();
//   rightEncoder->resetTurnCounter();
//   while ((angle/5) > testAvg) {  // needs timeout
//      testAvg = average(-leftEncoder->getTurnAngle(), rightEncoder->getTurnAngle());
//     //  Serial.println("TURN" + String(testAvg));
//   }
//   leftMotor->brake();
//   rightMotor->brake();
// }

void Robot::leftTurnStationaryUsingEncoder(float encSteps) {
float testAvg = 0;
float startSteps = rightEncoder->stepCounter;
  //Steps for 360 deg = 200 steps
  //keep turning while angle needed < cur angle + tol
  leftMotor->drive(TURN_SPEED, BACKWARD_DIR);
  rightMotor->drive(TURN_SPEED, FORWARD_DIR);

  leftEncoder->resetTurnCounter();
  rightEncoder->resetTurnCounter();
  while (startSteps+encSteps > rightEncoder->stepCounter) {  // needs timeout
    //  testAvg = average(-leftEncoder->getTurnAngle(), rightEncoder->getTurnAngle());
    // testAvg = average(-leftEncoder.stegetTurnAngle(), rightEncoder->getTurnAngle());
    //  Serial.println("TURN" + String(testAvg));
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

void Robot::forwardDriveDistance(int speed, float distanceCM) {
  initialDistance = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM());  // do we get average and compare end result avg, or do we just monitor each independently
  // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
  forwardDrive(speed);

  while (distanceCM >= currentDistanceTravelled) {
    currentDistanceTravelled = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM()) - initialDistance;
  }
  currentDistanceTravelled = 0;
  brake();  // will stop currently for testing, but this can be removed later to optimize
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

// void forwardDriveDistanceENCMonitor(int speed, float distanceCM){
//    initialDistance = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM());  // do we get average and compare end result avg, or do we just monitor each independently
//   // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
//   forwardDrive(speed);
//   while (distanceCM >= currentDistanceTravelled) {
//     currentDistanceTravelled = average(leftEncoder->getDistanceCM(), rightEncoder->getDistanceCM()) - initialDistance;
//   }
//   currentDistanceTravelled = 0;
//   brake();  // will stop currently for testing, but this can be removed later to optimize
// }

float Robot::average(float inputA, float inputB) {
  return (inputA + inputB) / 2;
}

void Robot::Setup_TOF_Address() {
  // Bot has reset pin, top changes address

  //reset low
  botTOF->resetOn();

  //init top with address
  topTOF->initTOF(0x31);

  //reset high
  botTOF->resetOff();

  //init bot no addr
  botTOF->initTOF();


  // botTOF->resetOn();  // turns reset pin high
  // topTOF->setAddress(0b0101011);
  // botTOF->resetOff();  // turns reset pin low, then calls the init command
  // Serial.println("BOT ADDR: " + botTOF->getAddress());
  // Serial.println("TOP ADDR: " + topTOF->getAddress());
}

// bool Robot::scanForPole() {

//   //NEED TO TEST HOW LONG AVG FUNCTION TAKES AND IF ITS WORTH IMPROVING PERFORMANCE
//   scanDistanceBotAverage = 0;
//   scanDistanceTopAverage = 0;

//  

//   for (scanCounter = 0; scanCounter < NUMBER_OF_SCANS; scanCounter++) {
// scanDistanceBot = botTOF->scanDistance();
//   scanDistanceTop = topTOF->scanDistance();
//     botTOF->debounceSignal(scanDistanceBot, scanDistanceBotAverage);
//     topTOF->debounceSignal(scanDistanceTop, scanDistanceTopAverage);

//     delay(SCAN_DELAY);
//   }

//   //Have a better tolerance with testing
//   if (scanDistanceBotAverage >= scanDistanceTopAverage*0.9 && scanDistanceBotAverage <= scanDistanceTopAverage*1.1){ // If both values are within 10% tolerance of eachother, pole found
//     //Pole found
//     linearDistToPole = average(scanDistanceBotAverage,scanDistanceTopAverage); // NEEDS A SCALING FACTOR TO CM VIA TESTING
//     return true;
//   } else if (scanDistanceBotAverage == 0 && scanDistanceTopAverage > 100){ // If bot errors out, only rely on top 
//     linearDistToPole = scanDistanceTopAverage; // NEEDS A SCALING FACTOR TO CM VIA TESTING
//     return true;

//   } else if (scanDistanceBotAverage > 100 && scanDistanceTopAverage == 0){
//     linearDistToPole = scanDistanceBotAverage; // NEEDS A SCALING FACTOR TO CM VIA TESTING
//     return true;
//   }

//   return false;


//   //   //validate and average data
//   //   if (scanDataIsGood(scanDistanceBot)){
//   //     if(scanDistanceBotAverage == 0){
//   //       scanDistanceBotAverage = scanDistanceBot;
//   //     } else {
//   //       scanDistanceBotAverage = average(scanDistanceBotAverage, scanDistanceBot);
//   //     }
//   // }
// }
