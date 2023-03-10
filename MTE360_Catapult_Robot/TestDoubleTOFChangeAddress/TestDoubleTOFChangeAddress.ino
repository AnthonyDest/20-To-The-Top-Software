/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include "VL53L0X.h"

VL53L0X botSensor;

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

#define resetPin A1

void setup()
{
  Serial.begin(9600);
  Wire.begin();
//   botSensor.setTimeout(500);
//   if (!botSensor.init())
//   {
//     Serial.println("Failed to detect and initialize sensor!");
//     while (1) {}
//   }

// // Serial.println
// //Setting one as reset off to change the address of the other one

// //top connected to reset pin, turn off top and change address of bot
// pinMode(resetPin, OUTPUT);
// digitalWrite(resetPin, 255);

// //change address of bot
// botSensor.setAddress(0b0101011);

// //let go of top reset
// digitalWrite(resetPin, 0);

  botSensor.setTimeout(500);
  if (!botSensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }



#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
  Serial.println("START");
}

void loop()
{
  Serial.print("BOT: " + botSensor.readRangeSingleMillimeters());
  if (botSensor.timeoutOccurred()) { Serial.print(" TIMEOUT BOT"); }


  Serial.println();
  delay(200);
}
