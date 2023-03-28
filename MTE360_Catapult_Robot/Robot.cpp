// #include "delay.h"
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
  // leftMotor->setupPID_CUSTOM(deltaLeft, leftMotorSpeed, stepToTravel, maxSpeed);
  // rightMotor->setupPID_CUSTOM(deltaRight, rightMotorSpeed, stepToTravel, maxSpeed);

  while (stepToTravel > average(deltaLeft, deltaRight)) {
    deltaLeft = abs(leftEncoder->stepCounter - leftEncoder->stepTripCounterBegin);
    deltaRight = abs(rightEncoder->stepCounter - rightEncoder->stepTripCounterBegin);
    leftMotor->drive(leftMotorSpeed, leftMotorDir);
    rightMotor->drive(rightMotorSpeed, rightMotorDir);
    leftMotor->compute();
    rightMotor->compute();
    // gyro->getTurnAngle(); //temp replace of interupts
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

  if (abs(deltaAngle) > 180) {                                              // maybe remove the angle/abs(Angle), replace with just if statement
    deltaAngle = (fmod(deltaAngle, 180.0)) * deltaAngle / abs(deltaAngle);  // modulo with same sign as original
  }

  if (deltaAngle > 0) {
    turnDeltaAngleGyro(abs(deltaAngle), BACKWARD_DIR, FORWARD_DIR);

  } else if (deltaAngle < 0) {
    turnDeltaAngleGyro(abs(deltaAngle), FORWARD_DIR, BACKWARD_DIR);
  }

  //if delta =  0 (very unlikely to be accident, unless you already at heading)
}

void Robot::turnDeltaAngleGyro(double angleToTurn, int leftDir, int rightDir) {  // can be simplified to just have one as opposite of the other
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

    // Serial.println("Current Heading: " + String(currentHeading));
    //  Serial.println("Delta Angle: " + String(deltaAngle));

    // //log("Angle to turn: " + String(angleToTurn) + "   D" + String(deltaAngle));
    // wait(10);
  }

  leftMotor->brake();
  rightMotor->brake();
}


void Robot::driveForwardAtCurrentHeading(double distanceMM) {
  double heading = gyro->getTurnAngle();
  double steerAdjustment = 0;

  double currentHeading = 0;
  double deltaHeading = 0;
  double maxSpeed = 220;

  double speed = 175;

  // leftMotor->drive(speed, FORWARD_DIR);
  // rightMotor->drive(speed, FORWARD_DIR);

  deltaLeft = 0;  // fix temp storage
  deltaRight = 0;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();

  // //log("Driving distance " + String(distanceMM/MM_PER_STEP));
  // delay(3000);
  // while (( millis() - startTime) < 30000) {  // encoder steps = distance needed
  while (driveDistanceTracking(distanceMM / MM_PER_STEP)) {

    // currentHeading = gyro->getTurnAngle();
    deltaHeading = (heading - gyro->getTurnAngle());

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

    if (deltaHeading < 0) {
      // //log("STEER RIGHT ");
      Serial.println("STEER RIGHT ");
    }
    if (deltaHeading > 0) {
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


void Robot::forwardDriveDistance(double distanceMM, double speed) {

  double stepToTravel = distanceMM / MM_PER_STEP;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();

  while (driveDistanceTracking(stepToTravel)) {

    leftMotor->drive(speed, FORWARD_DIR);
    rightMotor->drive(speed, FORWARD_DIR);
   
  }
  leftMotor->brake();
  rightMotor->brake();
}


void Robot::reverseDriveDistance(double distanceMM, double speed) {

  double stepToTravel = distanceMM / MM_PER_STEP;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();

  while (driveDistanceTracking(stepToTravel)) {

    leftMotor->drive(speed, BACKWARD_DIR);
    rightMotor->drive(speed, BACKWARD_DIR);
   
  }
  leftMotor->brake();
  rightMotor->brake();
}

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

  double stepToTravel = distanceMM / MM_PER_STEP;

  leftEncoder->resetTripCounter();
  rightEncoder->resetTripCounter();
  leftMotor->setupPID_CUSTOM(deltaLeft, leftMotorSpeed, stepToTravel, maxSpeed);
  rightMotor->setupPID_CUSTOM(deltaRight, rightMotorSpeed, stepToTravel, maxSpeed);

  while (driveDistanceTracking(stepToTravel)) {

    deltaHeading = (heading - gyro->getTurnAngle());

    leftMotor->drive(leftMotorSpeed - deltaHeading, FORWARD_DIR);
    rightMotor->drive(rightMotorSpeed + deltaHeading, FORWARD_DIR);
    if(leftMotorSpeed<maxSpeed){
      Serial.print("L: " + String(leftMotorSpeed));
    }
    if(rightMotorSpeed<maxSpeed){
      Serial.print("R: " + String(rightMotorSpeed));
    }

    if(leftMotorSpeed<maxSpeed or rightMotorSpeed<maxSpeed){
      Serial.println();
    }

    leftMotor->compute();
    rightMotor->compute();
  }
  leftMotor->brake();
  rightMotor->brake();
}

bool Robot::driveDistanceTracking(double stepToTravel) {
  // averageSteps = 0;

    deltaLeft = abs(leftEncoder->stepCounter - leftEncoder->stepTripCounterBegin);
    deltaRight = abs(rightEncoder->stepCounter - rightEncoder->stepTripCounterBegin);
    averageSteps = average(deltaLeft, deltaRight);

  if (stepToTravel > averageSteps) {  //return true once distance travelled
                                      // if (stepToTravel > average(deltaLeft, deltaRight)) { //return true once distance travelled
    return true;
  }

  return false;
}

double Robot::getOrientationAngle() {  //will need a polling function
  double angle = gyro->getTurnAngle();
  // Serial.println(String(angle));
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

    wait(SCAN_DELAY); //Wait1
  }

  scanDistanceBotAverage = scanDistanceBotAverage * cos(radians(0));
  scanDistanceTopAverage = scanDistanceTopAverage * cos(radians(3));
  Serial.println("Bot TOF Distance: " + String(scanDistanceBotAverage));
  //log("Bot TOF Distance: " + String(scanDistanceBotAverage));
  //log("Top TOF Distance: " + String(scanDistanceTopAverage));
}

// bool Robot::poleFound() { //will need to edit based on position 1 or 2
//   //NEED TO TEST HOW LONG AVG FUNCTION TAKES AND IF ITS WORTH IMPROVING PERFORMANCE
//   //Have a better tolerance with testing
//   scanBothTOF();
//    Serial.println("BOT AVG: " + String(scanDistanceBotAverage));
//   // Serial.println("Top AVG: " + String(scanDistanceTopAverage));

//   delay(200);

//   //need a better find pole algarthim, need to be careful with >100 as it is good at wall but not at point 2
//   if (scanDistanceBotAverage > 100 & scanDistanceTopAverage > 100) {
//     linearDistToPole = average(scanDistanceBotAverage, scanDistanceTopAverage);
//     //log("Pole found, both sensors >100");

// Serial.println("Pole found, both sensors >100");

//     return true;
//   } else if (scanDistanceBotAverage > 100) {
//     linearDistToPole = scanDistanceBotAverage;
//     //log("Pole found, bot sensor");
//     Serial.println("Pole found, bot sensor");
//     return true;

//   } else if (scanDistanceTopAverage > 100) {
//     //log("Pole found, top sensor");
//     Serial.println("Pole found, top sensor");
//     linearDistToPole = scanDistanceTopAverage;
//     return true;
//   }

//   return false;
// }

bool Robot::poleFound() {
  scanBothTOF();

  if (scanDistanceBotAverage > 10 && scanDistanceBotAverage < 900) {
    Serial.println("POLE FOUND");
    linearDistToPole = scanDistanceBotAverage;
    return true;
  }

  return false;
}

bool Robot::searchForPole(int scanDirection, int degreesToScan) { // Change to have pole found as an internal robot variable?
  gyro->resetTripCounter();
  deltaTurnAngle = 0;
  bool isPoleFound = false;
  double testLastGetTurnAngle = 0;

  while (!isPoleFound and deltaTurnAngle < degreesToScan) {
    isPoleFound = poleFound(); //COMMENT OUT FOR TESTING

    if(!isPoleFound){

    Serial.println("Turning to scan angle " + String(deltaTurnAngle) + "  total " +String(degreesToScan) + "   "+ isPoleFound);
    if (scanDirection == CCW_DIR) {
      leftTurnStationaryPID(50);
      // turnDeltaAngleGyro(0.5, FORWARD_DIR, BACKWARD_DIR);
    } else if (scanDirection == CW_DIR){
      rightTurnStationaryPID(50);
    }
    testLastGetTurnAngle = gyro->getTurnAngle();
    deltaTurnAngle = abs(gyro->startTurnHeading - testLastGetTurnAngle);
    // wait(100);  //shorten post testing
    }
  }
  Serial.println("Delta :" + String(deltaTurnAngle));
  Serial.println("Last :" + String(testLastGetTurnAngle)); 
  // wait(2000);
  Serial.println("Pole Found: " + String(isPoleFound));
  return isPoleFound;
  // FOR TESTING DRIVING ONLY, POLE FOUND = false;
  // return false;
}
/////////////////

bool Robot::searchForPoleContiniousSweep(int scanDirection, int degreesToScan) { // Change to have pole found as an internal robot variable?
  gyro->resetTripCounter();
  double deltaTurnAngle = 0;
  int turnSpeed  = 50;
  bool isPoleFound = false;
  double currentHeading = 0;
  double startAngle = gyro->getTurnAngle();

if (scanDirection == CCW_DIR) {
  //     leftMotor->drive(turnSpeed, for); 
  // rightMotor->drive(turnSpeed, rightDir);
  leftMotor->rev(turnSpeed);
  rightMotor->fwd(turnSpeed);

    } else if (scanDirection == CW_DIR){
  //     leftMotor->drive(turnSpeed, leftDir);
  // rightMotor->drive(turnSpeed, rightDir);
   leftMotor->fwd(turnSpeed);
  rightMotor->rev(turnSpeed);
    }
  
  while (!isPoleFound and deltaTurnAngle < degreesToScan) {
    currentHeading = gyro->getTurnAngle();
    deltaTurnAngle = currentHeading - startAngle;
    isPoleFound = poleFound();
  }

  leftMotor->brake();
  rightMotor->brake();
  Serial.println("Delta :" + String(deltaTurnAngle));
  // wait(2000);
  Serial.println("Pole Found: " + String(isPoleFound));
  return isPoleFound;
  // FOR TESTING DRIVING ONLY, POLE FOUND = false;
  // return false;
}





// bool Robot::searchForPole(int scanDirection, int degreesToScan) { //another var to be at wall?
//   //search for pole, if it thinks pole found, do 2 more scans (unless at limit?), if both scans empty, return polefound true and go to that heading
//   gyro->resetTripCounter();
//   deltaTurnAngle = 0;
//   bool possiblePoleFound = false;
//   bool poleHeadingConfirmed = false;
//   double headingOfPossiblePole = 500;

//   while (!poleHeadingConfirmed and deltaTurnAngle < degreesToScan) {
//     scanBothTOF();
//     Serial.println("Turning to scan angle " + String(deltaTurnAngle));
//     if(scanDirection == CW_DIR){
//       leftTurnStationaryPID(50);
//     } else {
//        rightTurnStationaryPID(50);
//     }
//     deltaTurnAngle = abs(gyro->startTurnHeading - gyro->getTurnAngle());
//     delay(500);  //shorten post testing

//   //might be better to just trust gyro and test furthest distance to objects?
//   //put a max distance cap at 90cm to ensure we never scan wall based on math?
//     if(possiblePoleFound and scanDistanceBotAverage > 20){ //might need a counter, as if pole is too close, one rotation may give false negatives
//       //probably wall?
//       possiblePoleFound = false;
//       headingOfPossiblePole = 500;
//     } else if(possiblePoleFound and scanDistanceBotAverage == 0){
//       poleHeadingConfirmed = true;
//     } else if(scanDistanceBotAverage > 20 and (!possiblePoleFound and !poleHeadingConfirmed)){ //minimum distance to pole
//       possiblePoleFound = true;
//       headingOfPossiblePole = getOrientationAngle();
//     }

//   }

// if(poleHeadingConfirmed){
//   return true;
// }

// return false;

// }

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

void Robot::wait(double MS){
double startTime = millis();
while ((millis() - startTime) < MS){
   gyro->getTurnAngle(); //temp replace of interupts
};

}
