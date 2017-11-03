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

#define DEBUG
#ifdef DEBUG
  #define DBprint(x) Serial.print(x)
  #define DBprintln(x) Serial.println(x)
#else
  #define DBprint(x)
  #define DBprintln(x)
#endif

// Serial communication baud rate
#define BAUD 115200
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
// Linear acceleration sensitivity for FS = +/- 2 G 
// Comes from LSM303D datasheet, (mG/LSB)
#define LA_So 0.061
// Converts mGs to mm/s^2
#define mG2MM 9.81

Zumo32U4LCD lcd;
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
LSM303 compass;
L3G gyro;

// Encoder Odometry
float encDistance, encTheta;

// Save all IMU data into one struct
typedef struct {
  float gx;
  float gy;
  float gz;
  float ax;
  float ay;
  float az;
  float mx;
  float my;
  float mz;
} IMUStruct;
IMUStruct IMU;

// Timing
uint16_t lastTime = 0;
uint16_t curTime = 0;
uint16_t dt;

void setup() {
  #ifdef DEBUG
    Serial.begin(BAUD);
  #endif
  
  Wire.begin();
  initializeIMU( &compass, &gyro );
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
}

void loop() {

  curTime = millis();
  dt = curTime - lastTime;
  if ( dt > TS ) {
    updateEncoderOdometry( &encoders, &encDistance, &encTheta );
    updateIMUOdometry( &compass, &gyro, &IMU );
    DBprint(IMU.ax);
    DBprint('\t');
    DBprint(dt);
    DBprint('\t');
    DBprint(compass.a.x*LA_So*mG2MM*dt*dt);
    DBprint('\t');
    DBprintln(encDistance);
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
void updateIMUOdometry( LSM303 *compass, L3G *gyro, IMUStruct *IMU ) {

  compass->read();
  gyro->read();

  IMU->gx = gyro->g.x;
  IMU->gy = gyro->g.y;
  IMU->gz = gyro->g.z;
  IMU->ax = compass->a.x;
  IMU->ay = compass->a.y;
  IMU->az = compass->a.z;
  IMU->mx = compass->m.x;
  IMU->my = compass->m.y;
  IMU->mz = compass->m.z;
  
}

/*
 * compFilter
 * 
 * 
 */
float distCompFilt( float accel, float encoder, uint16_t dt ) {

  float accelDist = accel*LA_So*mG2MM/(dt*1000.0);
  return;
  
}

float rotCompFilt( float gyro, float magnometer, float encoderTheta ) {

  return;
  
}









 

