/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

 Private-use only! (you need to ask for a commercial-use)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 Private-use only! (you need to ask for a commercial-use)

 */

/* pitch/roll and heading estimation (IMU sensor fusion)
 requires: GY-80 module (L3G4200D, ADXL345B, HMC5883L)

 How to use it (example):
 1. initialize IMU:                 IMU imu;  imu.init();
 2. read IMU (yaw/pitch/roll:       Serial.println( imu.ypr.yaw );
 */

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

#include "Pid.h"

// IMU state
enum
{
  IMU_RUN,
  IMU_CAL_COM
};

typedef struct point_int_t
{
  int16_t x;
  int16_t y;
  int16_t z;
} point_int_t;

typedef struct point_long_t
{
  long x;
  long y;
  long z;
} point_long_t;

typedef struct point_float_t
{
  float x;
  float y;
  float z;
} point_float_t;

typedef struct ypr_t
{
  float yaw;
  float pitch;
  float roll;
} ypr_t;

class Imu
{
  public:
    enum imuE
    {
      DIR,
      ROLL,
      END
    };

    Imu();
    boolean init(int aPinBuzzer);
    void update(void);
    int getCallCounter(void);
    int getErrorCounter(void);
    void deleteCalib(void);

    boolean use { false };
    unsigned long nextTime {};
    unsigned long nextTimeControl {};
    bool correctDir { false };  // correct direction by compass?
    Pid pid[END];               // direction and roll PID controllers


    int callCounter;
    int errorCounter;
    boolean hardwareInitialized;
    byte state;
    unsigned long lastAHRSTime;
    unsigned long now;
    ypr_t ypr;  // gyro yaw,pitch,roll

    // --------- gyro state -----------------------------
    point_float_t gyro;   // gyro sensor data (degree)
    point_float_t gyroOfs; // gyro calibration data
    float gyroNoise;      // gyro noise
    int gyroCounter;
    boolean useGyroCalibration; // gyro calibration flag
    unsigned long lastGyroTime;

    // --------- acceleration state ---------------------
    point_float_t acc;  // acceleration sensor data
    point_float_t accGrav;  // acceleration sensor data (gravity corrected)
    point_float_t accMin;
    point_float_t accMax;
    int accelCounter;
    boolean useAccCalibration;
    float accPitch;
    float accRoll;
    point_float_t accOfs;
    point_float_t accScale;
    int calibAccAxisCounter;

    // calibrate acceleration sensor
    boolean calibAccNextAxis();
    boolean calibrationAvail;

    // --------- compass state --------------------------
    point_float_t com; // compass sensor data (raw)
    point_float_t comLast;
    point_float_t comMin; // compass sensor data (raw)
    point_float_t comMax; // compass sensor data (raw)
    point_float_t comTilt; // compass sensor data (tilt corrected)
    point_float_t comOfs;
    point_float_t comScale;
    float comYaw;         // compass heading (radiant, raw)
    boolean useComCalibration;

    // calibrate compass sensor
    void calibComStartStop(void);
    void calibComUpdate(void);
    boolean newMinMaxFound(void);
    // --------------------------------------------------

    // helpers
    float scalePI(const float v);
    float scale180(const float v);
    float distancePI(const float x, const float w);
    float distance180(const float x, const float w);
    float fusionPI(const float w, const float a, const float b);

  private:
    void read();
    void loadSaveCalib(const boolean readflag);
    void calibGyro(void);
    void loadCalib(void);

    // print IMU values
    void printPt(point_float_t p);
    void printCalib(void);
    void saveCalib(void);
    float sermin(float oldvalue, float newvalue);
    float sermax(float oldvalue, float newvalue);

    // hardware
    void initADXL345B(void);
    boolean initL3G4200D(void);
    void initHMC5883L(void);
    void readL3G4200D(boolean useTa);
    void readADXL345B(void);
    void readHMC5883L(void);
    boolean foundNewMinMax;
    int pinBuzzer;
};

#endif

