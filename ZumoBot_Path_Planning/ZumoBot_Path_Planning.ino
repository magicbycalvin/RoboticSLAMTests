/**********************************************************************************
 * ZumoBot_Path_Planning
 * 
 * Author: Calvin Kielas-Jensen
 * Date: 10/29/2017
 * 
 * Description: This program is written for the Pololu Zumobot 32U4 with 50:1
 *  gearmotors. It is written as a test and proof of concept for implementing a 
 *  light weight path planning algorithm. The inital design assumes it will use 
 *  three proximity sensors (left, right, and front) as provided by the Zumobot
 *  library. It should create a map of the course and then determine the shortest
 *  path to the goal from the start. Due to the low processing and memory
 *  capabilities of the ATMEGA 32U4, the algorithms take many shortcuts and strive
 *  to avoid floating point calculations when possible.
 *  
 *  Units: Unless otherwise specified, all units are converted to mm
 *********************************************************************************/

#include <Wire.h>
#include <Zumo32U4.h>
//#include <MahonyAHRS.h>

// Length from one track to the other in mm from the Pololu website
#define WHEEL_BASE 98
// Constant to convert encoder ticks to mm traveled, the equation used was:
// TICK2MM = (D*PI) / (CPR*GearRatio)
// The values came from Pololu:
// D = 39 mm
// CPR = 12 counts per revolution
// GearRatio = 51.45
#define TICK2MM 0.198449
// Sample time in ms for all filter and control loops
#define TS 1

Zumo32U4LCD lcd;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
LSM303 compass;
L3G gyro;

//Mahony filter;

// Encoder Odometry
float encDistance, encTheta;

// IMU Odometry
//float roll, pitch, yaw;

// Timing
uint16_t lastTime = 0;
uint16_t curTime = 0;

void setup() {
  Wire.begin();
  initializeIMU( &compass, &gyro );
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
}

void loop() {

  curTime = micros();
  if ( curTime - lastTime > TS*1000 ) {
    updateEncoderOdometry( &encoders, &encDistance, &encTheta );
    //updateIMUOdometry( &compass, &gyro, &filter, &roll, &pitch, &yaw );
    updateIMUOdometry( &compass, &gyro );
//    Serial.print(roll);
//    Serial.print('\t');
//    Serial.print(pitch);
//    Serial.print('\t');
//    Serial.println(yaw);
    lastTime = curTime;
  }
  
}

/*
 * initializeIMU
 * 
 * Initializes the IMU and check for errors. Will drop into an infinite loop if either the compass
 * or gyro fail to initialize.
 */
void initializeIMU( LSM303 *compass, L3G *gyro ) {
  if (!compass->init()) {
    // Failed to detect the compass.
    ledRed(1);
    while(1);
  }
  
  compass->enableDefault();

  if (!gyro->init()) {
    // Failed to detect the gyro.
    ledRed(1);
    while(1);
  }
  
  gyro->enableDefault();
  
}

/*
 * updateEncoderOdometry
 * 
 * Reads the encoder data and produces a distance and heading in mm and radians respectively.
 */
void updateEncoderOdometry( Zumo32U4Encoders *encoders, float *encDistance, float *encTheta ) {
  int16_t leftCounts, rightCounts;

  if (encoders->checkErrorLeft() || encoders->checkErrorRight()) {
    // May add some error code here in the future, for now, do nothing
  } else {
    leftCounts = encoders->getCountsLeft();
    rightCounts = encoders->getCountsRight();
    
    *encDistance = (float) TICK2MM * (leftCounts + rightCounts) / 2.0;
    *encTheta = (float) TICK2MM * (leftCounts - rightCounts) / WHEEL_BASE;
  }
  
}

/*
 * updateIMUOdometry
 * 
 * Reads the compass and gyro data and then passes it through the Mahony
 * filter in order to produce a filtered result.
 */
//void updateIMUOdometry( LSM303 *compass, L3G *gyro, Mahony *filter,
//  float *roll, float *pitch, float *yaw ) {
void updateIMUOdometry( LSM303 *compass, L3G *gyro ) {

  float gx, gy, gz, ax, ay, az, mx, my, mz;

  compass->read();
  gyro->read();

  gx = gyro->g.x;
  gy = gyro->g.y;
  gz = gyro->g.z;
  ax = compass->a.x;
  ay = compass->a.y;
  az = compass->a.z;
  mx = compass->m.x;
  my = compass->m.y;
  mz = compass->m.z;
  
  /*
  filter->updateIMU( gx, gy, gz, ax, ay, az );

  *roll = filter->getRoll();
  *pitch = filter->getPitch();
  *yaw = filter->getYaw();
   */
  
}

