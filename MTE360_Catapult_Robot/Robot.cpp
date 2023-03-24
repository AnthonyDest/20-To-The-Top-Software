#include "delay.h"
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

Robot::Robot(Motor &_leftMotor, Motor &_rightMotor, Encoder &_leftEncoder, Encoder &_rightEncoder, TOFSensor &_botTOF, TOFSensor &_topTOF, Gyro &_gyro) {
  leftMotor = &_leftMotor;
  rightMotor = &_rightMotor;
  leftEncoder = &_leftEncoder;
  rightEncoder = &_rightEncoder;
  botTOF = &_botTOF;
  topTOF = &_topTOF;
  gyro = &_gyro;
}

void Robot::allConfiguration() {
  Serial.println("TOF START");
  Setup_TOF_Address();
  Serial.println("TOF GOOD");
    gyro->setup();
     Serial.println("GYRO GOOD");
    gyro->firstStabalizeGyroValues();
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

//Motor braking (hard stop)
void Robot::brake() {

  // slow down (need to check current direction of motors)
  leftMotor->drive(0, leftMotor->getMotorDir());
  rightMotor->drive(0, rightMotor->getMotorDir());

  // stop
  leftMotor->brake();
  rightMotor->brake();
}

void Robot::drivePID(double stepToTravel, int leftMotorDir, int rightMotorDir, double maxSpeed) {
  //keep turning while angle needed < cur angle + tol
  deltaLeft = 0;  // fix temp storage
  deltaRight = 0;
  double leftMotorSpeed = 0;
  double rightMotorSpeed = 0;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();
  leftMotor->setupPID(deltaLeft, leftMotorSpeed, stepToTravel, maxSpeed);
  rightMotor->setupPID(deltaRight, rightMotorSpeed, stepToTravel, maxSpeed);

  while (stepToTravel > average(deltaLeft, deltaRight)) {
    deltaLeft = abs(leftEncoder->stepCounter - leftEncoder->stepTripCounterBegin);
    deltaRight = abs(rightEncoder->stepCounter - rightEncoder->stepTripCounterBegin);
    leftMotor->drive(leftMotorSpeed, leftMotorDir);
    rightMotor->drive(rightMotorSpeed, rightMotorDir);
    leftMotor->compute();
    rightMotor->compute();
  }
  leftMotor->brake();
  rightMotor->brake();
}

void Robot::leftTurnStationaryPID(double stepsToTurn) {
  drivePID(stepsToTurn, BACKWARD_DIR, FORWARD_DIR, 50);
}
void Robot::rightTurnStationaryPID(double stepsToTurn) {
  drivePID(stepsToTurn, FORWARD_DIR, BACKWARD_DIR, 50);
}
void Robot::forwardDrivePID(double distanceMM) {

  drivePID(distanceMM / MM_PER_STEP, FORWARD_DIR, FORWARD_DIR, 100);
}
void Robot::reverseDrivePID(double distanceMM) {
  drivePID(distanceMM / MM_PER_STEP, BACKWARD_DIR, BACKWARD_DIR, 100);
}

//when driving, turning left is increasing, turning right is decreasing
//so to go straight, we pick set a heading. If next angle is
//will need debouncing

void Robot::turnToHeading(double heading) {

  double currentAngle = gyro->getTurnAngle();
  double deltaAngle = 0;

  deltaAngle = heading - currentAngle;

  if (abs(deltaAngle) > 180) { // maybe remove the angle/abs(Angle), replace with just if statement
    deltaAngle = (fmod(deltaAngle, 180.0)) * deltaAngle / abs(deltaAngle);  // modulo with same sign as original
  }

  if (deltaAngle > 0) {
      turnDeltaAngleGyro(abs(deltaAngle), BACKWARD_DIR, FORWARD_DIR);

  } else if (deltaAngle < 0) {
    turnDeltaAngleGyro(abs(deltaAngle), FORWARD_DIR, BACKWARD_DIR);
  }

  //if delta =  0 (very unlikely to be accident, unless you already at heading)
}

void Robot::turnDeltaAngleGyro(double angleToTurn, int leftDir, int rightDir) { // can be simplified to just have one as opposite of the other
  int speed = 50;
  double deltaAngle = 0;
  double startAngle = gyro->getTurnAngle();
  double currentHeading = 0;
  // delay(100);
  leftMotor->drive(speed, leftDir);
  rightMotor->drive(speed, rightDir);

  while (angleToTurn > abs(deltaAngle)) {
    currentHeading = gyro->getTurnAngle();
    deltaAngle = currentHeading - startAngle;
    // deltaAngle = fmod((currentHeading - startAngle), 180);
    // Serial.println("Delta Angle: " + String(deltaAngle));
    // //log("Angle to turn: " + String(angleToTurn) + "   D" + String(deltaAngle));
    delay(10);
  }

  leftMotor->brake();
  rightMotor->brake();
}


void Robot::driveForwardAtCurrentHeading(double distanceMM) {
  double heading = gyro->getTurnAngle();
  double steerAdjustment = 0;
  
  double currentHeading = 0;
  double deltaHeading = 0;
  double maxSpeed = 150;
  
  double speed = 50;

  // leftMotor->drive(speed, FORWARD_DIR);
  // rightMotor->drive(speed, FORWARD_DIR);

  deltaLeft = 0;  // fix temp storage
  deltaRight = 0;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();

  // //log("Driving distance " + String(distanceMM/MM_PER_STEP));
  // delay(3000);
  // while (( millis() - startTime) < 30000) {  // encoder steps = distance needed
  while(driveDistanceTracking(distanceMM/MM_PER_STEP)){

  // currentHeading = gyro->getTurnAngle();
  deltaHeading = (heading-gyro->getTurnAngle());

  // //if within 10%, ramp up,
  // if((distanceMM/MM_PER_STEP)*0.10 > averageSteps){
  //   speed = speed+5;    
  // }

  //if within 90% ramp down
//   if((distanceMM/MM_PER_STEP)*0.25 < averageSteps){
//     speed = 125;
//   }

//    if((distanceMM/MM_PER_STEP)*0.50 < averageSteps){
//     speed = 80;
//   }


// if((distanceMM/MM_PER_STEP)*0.75 < averageSteps){
//     speed = 50;
//   }


  // if (speed < 100){
  //   speed = 100;
  // }

  // if (speed > maxSpeed){ // redundent? Unless you want a secondary max
  //   speed = maxSpeed;
  // } else if ( speed < 30) {
  //   speed = 30;
  // }

  if(deltaHeading < 0){
    // //log("STEER RIGHT ");
    Serial.println("STEER RIGHT ");
  }
   if(deltaHeading > 0){
    // //log("STEER LEFT ");
    Serial.println("STEER LEFT");
  }

  //log("L: " + String(speed - deltaHeading) + "R: " + String(speed + deltaHeading) + " DIST: " + String(average(deltaLeft, deltaRight) * MM_PER_STEP));
  Serial.println("L: " + String(speed - deltaHeading) + "R: " + String(speed + deltaHeading) + " DIST: " + String(average(deltaLeft, deltaRight) * MM_PER_STEP));
  // leftMotor->drive(speed - deltaHeading, FORWARD_DIR);
  rightMotor->drive(speed + deltaHeading, FORWARD_DIR);
   leftMotor->drive(speed - deltaHeading, FORWARD_DIR);
   Serial.println(speed);

  //  if(speed < maxSpeed){
  //    speed = speed+25;
  //  }

}
  leftMotor->brake();
  rightMotor->brake();

}
/////////////////////////////////////////////////////////////////////////

void Robot::driveForwardAtCurrentHeadingWithPID(double distanceMM, double maxSpeed) {
  double heading = gyro->getTurnAngle();
  double steerAdjustment = 0;
  
  double currentHeading = 0;
  double deltaHeading = 0;
  // double maxSpeed = 150;

  deltaLeft = 0;  // fix temp storage
  deltaRight = 0;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();

  double leftMotorSpeed = 0;
  double rightMotorSpeed = 0;

  double p = 16.2039;
  double i = 20.2906;
  double d = 0;


double stepToTravel  = distanceMM/MM_PER_STEP;
  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();
  leftMotor->setupPID_CUSTOM(deltaLeft, leftMotorSpeed, stepToTravel, maxSpeed);
  rightMotor->setupPID_CUSTOM(deltaRight, rightMotorSpeed, stepToTravel, maxSpeed);

  while(driveDistanceTracking(distanceMM/MM_PER_STEP)){

  deltaHeading = (heading-gyro->getTurnAngle());

    leftMotor->drive(leftMotorSpeed - deltaHeading, FORWARD_DIR);
    rightMotor->drive(rightMotorSpeed + deltaHeading, FORWARD_DIR);
    leftMotor->compute();
    rightMotor->compute();

}
  leftMotor->brake();
  rightMotor->brake();

}

bool Robot::driveDistanceTracking(double stepToTravel){

    if (stepToTravel > averageSteps) { //return true once distance travelled
  // if (stepToTravel > average(deltaLeft, deltaRight)) { //return true once distance travelled
    deltaLeft = abs(leftEncoder->stepCounter - leftEncoder->stepTripCounterBegin);
    deltaRight = abs(rightEncoder->stepCounter - rightEncoder->stepTripCounterBegin);
    averageSteps =  average(deltaLeft, deltaRight);
    return true;
  }

  return false;
  // leftMotor->brake();
  // rightMotor->brake();
}
// void Robot::forwardDriveDistance(int speed, float distanceMM) {  //currently steps

//   leftMotor->drive(speed, FORWARD_DIR);
//   rightMotor->drive(speed, FORWARD_DIR);

//   // Serial.println(distanceCM * rightEncoder->CM_TO_STEPS);
//   // travelledDistanceUsingEncoder((distanceCM)*0.80 - 100);
//   // travelledDistanceUsingEncoder(distanceCM- speed); // WILL NEED CALIBRATION
//   // travelledDistanceUsingEncoder(distanceMM / MM_PER_STEP);  // WILL NEED CALIBRATION
//     // delay(5000);
//     // travelledDistanceUsingEncoder(distanceCM * rightEncoder->CM_TO_STEPS);



//   // initialDistance = average(leftEncoder->getDistanceMM(), rightEncoder->getDistanceMM());  // do we get average and compare end result avg, or do we just monitor each independently
//   // // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
//   // forwardDrive(speed);

//   // while (distanceCM >= currentDistanceTravelled) {
//   //   currentDistanceTravelled = average(leftEncoder->getDistanceMM(), rightEncoder->getDistanceMM()) - initialDistance;
//   // }
//   // currentDistanceTravelled = 0;
//   // brake();  // will stop currently for testing, but this can be removed later to optimize
// }

// void Robot::reverseDriveDistance(int speed, float distanceCM) {
//   initialDistance = average(leftEncoder->getDistanceMM(), rightEncoder->getDistanceMM());  // do we get average and compare end result avg, or do we just monitor each independently
//   // eg: change to leftInital, rightInital. Distance travelled = average(DeltaLeft, DeltaRight) ?
//   backwardDrive(speed);

//   while (distanceCM >= currentDistanceTravelled) {
//     currentDistanceTravelled = abs(average(leftEncoder->getDistanceMM(), rightEncoder->getDistanceMM()) - initialDistance);
//   }
//   currentDistanceTravelled = 0;
//   brake();  // will stop currently for testing, but this can be removed later to optimize
// }


double Robot::getOrientationAngle() {  //will need a polling function

  double angle = gyro->getTurnAngle();
  Serial.println(String(angle));
  return angle;
}


float Robot::average(float inputA, float inputB) {
  return (inputA + inputB) / 2;
}

void Robot::Setup_TOF_Address() {
  // // set pinmode
  // botTOF->configureResetPin();

  // // Bot has reset pin, top changes address
  // // reset low
  // botTOF->resetOn();

  // //init top with address
  // topTOF->initalizeTOF(topTOFAddress);

  // //reset high
  // botTOF->resetOff();

  //init bot no addr
  botTOF->initalizeTOF();
}

void Robot::scanBothTOF() {
  scanDistanceBotAverage = 0;
  scanDistanceTopAverage = 0;

  for (scanCounter = 0; scanCounter < NUMBER_OF_SCANS; scanCounter++) {
    scanDistanceBot = botTOF->scanDistanceMM();
    // scanDistanceTop = topTOF->scanDistanceMM();
    botTOF->debounceDistance(scanDistanceBot, scanDistanceBotAverage);
    // topTOF->debounceDistance(scanDistanceTop, scanDistanceTopAverage);

    delay(SCAN_DELAY);
  }

  scanDistanceBotAverage = scanDistanceBotAverage* cos(radians(0));
  scanDistanceTopAverage = scanDistanceTopAverage* cos(radians(3));
Serial.println("Bot TOF Distance: " + String(scanDistanceBotAverage));
  //log("Bot TOF Distance: " + String(scanDistanceBotAverage));
  //log("Top TOF Distance: " + String(scanDistanceTopAverage));
}

bool Robot::poleFound() {
  //NEED TO TEST HOW LONG AVG FUNCTION TAKES AND IF ITS WORTH IMPROVING PERFORMANCE
  //Have a better tolerance with testing
  scanBothTOF();
   Serial.println("BOT AVG: " + String(scanDistanceBotAverage));
  // Serial.println("Top AVG: " + String(scanDistanceTopAverage));

  delay(200);
  if (scanDistanceBotAverage > 100 & scanDistanceTopAverage > 100) {
    linearDistToPole = average(scanDistanceBotAverage, scanDistanceTopAverage);
    //log("Pole found, both sensors >100");

Serial.println("Pole found, both sensors >100");

    return true;
  } else if (scanDistanceBotAverage > 100) {
    linearDistToPole = scanDistanceBotAverage;
    //log("Pole found, bot sensor");
    Serial.println("Pole found, bot sensor");
    return true;

  } else if (scanDistanceTopAverage > 100) {
    //log("Pole found, top sensor");
    Serial.println("Pole found, top sensor");
    linearDistToPole = scanDistanceTopAverage;
    return true;
  }
  // if (scanDistanceBotAverage != 0 && scanDistanceTopAverage != 0){
  //   if( (scanDistanceBotAverage >= scanDistanceTopAverage*0.8 && scanDistanceBotAverage <= scanDistanceTopAverage*1.2) || (scanDistanceBotAverage >= scanDistanceTopAverage-75 && scanDistanceBotAverage <= scanDistanceTopAverage+75)   ){ //Or to account for 10% not working well at low values
  //     // If both values are within 10% tolerance of eachother, pole found
  //     //Pole found
  //     linearDistToPole = average(scanDistanceBotAverage,scanDistanceTopAverage) /10; // NEEDS A SCALING FACTOR TO CM VIA TESTING, should just be MM -> CM
  //     return true;
  //   } else if (scanDistanceBotAverage == 0 && scanDistanceTopAverage > 100){ // If bot errors out, only rely on top
  //     linearDistToPole = scanDistanceTopAverage /10;// NEEDS A SCALING FACTOR TO CM VIA TESTING
  //     return true;

  //   } else if (scanDistanceBotAverage > 100 && scanDistanceTopAverage == 0){  // If Top errors out, only rely on top
  //     linearDistToPole = scanDistanceBotAverage /10; // NEEDS A SCALING FACTOR TO CM VIA TESTING
  //     return true;
  //   }
  // }
  return false;
}

void Robot::searchForPole() {  //Assumes pole will always be found during full rotation, can be later optimized to do better scanning algorithm using gyro & to ignore wall
  int scanCounterMax = 72;     //360 rotation // temporary full circle for testing assuming 360deg/5deg
  int scanCounter = 0;
  while (!poleFound() and scanCounter < scanCounterMax) {
    Serial.println("Turning to scan " + String(scanCounter));
    // leftTurnStationaryUsingEncoder(50);  // 650 ~ 1 rotation (?)
    leftTurnStationaryPID(50);
    scanCounter++;
    delay(500);  //shorten post testing
  }
}

// void Robot::setupSDCard(){
// if (!SD.begin(4)) {
//     //log("initialization failed!");
//     while (1);
//   }
//  //log("initialization done.");

// String fileNameForSD = "data1.txt";
// int runIteration = 2;

// while(!SD.exists(fileNameForSD)){
// fileNameForSD = "data"; // I hate strings and this works
// fileNameForSD += String(runIteration);
// fileNameForSD += ".txt";
// runIteration++;
// }

//  fileSD = SD.open(fileNameForSD, FILE_WRITE);

// }
// void Robot:://log(String dataTo//log){

//   Serial.println(dataTo//log);
//   if(fileSD){
//     fileSD.print(millis() + "   ");
//     fileSD.println(dataTo//log);
//   }
// }
