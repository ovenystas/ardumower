/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat

 Private-use only! (you need to ask for a magmercial-use)

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

 Private-use only! (you need to ask for a magmercial-use)

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
#include "L3G.h"
#include "LSM303.h"

// IMU state
enum
{
  IMU_RUN,
  IMU_CAL_COM
};

typedef struct point_int_t
{
  int x;
  int y;
  int z;
} point_int_t;

typedef struct point_longt_t
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

typedef struct calibrationData_t
{
  point_float_t accelOffset;
  point_float_t accelScale;
  point_int_t magnetometerOffset;
  point_int_t magnetometerScale;
} calibrationData_t;

class Imu
{
  public:
    enum imuPidE
    {
      DIR,
      ROLL,
      END
    };

    boolean init(int aPinBuzzer);
    void update(void);
    bool isTimeToRun(void);
    bool isTimeToControl(void);
    int getCallCounter(void);
    int getErrorCounter(void);
    void deleteCalibrationData(void);
    boolean isCalibrationAvailable(void) const
    {
      return calibrationAvailable;
    }
    void printInfo(Stream &s);

    boolean use { false };
    bool correctDir { false };  // correct direction by magnetometer?
    Pid pid[END];               // direction and roll PID controllers
    float getYaw() const
    {
      return ypr.yaw;
    }
    float getYawDeg()
    {
      return degrees(ypr.yaw);
    }

    float getPitch() const
    {
      return ypr.pitch;
    }
    float getPitchDeg()
    {
      return degrees(ypr.pitch);
    }

    float getRoll() const
    {
      return ypr.roll;
    }
    float getRollDeg()
    {
      return degrees(ypr.roll);
    }

    // --------- gyro state -----------------------------
    L3G gyro;

    // calibrate acceleration sensor
    boolean calibrateAccelerometerNextAxis();

    // --------- accelerometer/magnetometer state -------
    point_float_t accel { 0.0, 0.0, 0.0 };
    point_float_t mag { 0.0, 0.0, 0.0 };
    boolean getUseAccelCalibration(void) const
    {
      return useAccelCalibration;
    }
    void toggleUseAccelCalibration(void)
    {
      useAccelCalibration = !useAccelCalibration;
    }

    // calibrate magnetometer sensor
    void calibrateMagnetometerStartStop(void);


    // --------------------------------------------------

  private:
    void read();
    void calibrateGyro(void);
    void loadCalibrationData(void);

    // print IMU values
    void printPoint(const point_int_t point);
    void printPointln(const point_int_t point);
    void printPoint(const point_float_t point);
    void printPointln(const point_float_t point);
    void printCalibrationData(void);
    void saveCalibrationData(void);

    // hardware
    void initAccelerometer(void);
    boolean initGyroscope(void);
    void initMagnetometer(void);

    void readGyroscope(void);
    void readAccelerometer(void);
    void readMagnetometer(void);

    void calibrateMagnetometerUpdate(void);
    void playCompletedSound();

    // helpers
    float scalePI(const float v);
    float scale180(const float v);
    float scalePIangles(const float setAngle, const float currAngle);
    float distancePI(const float x, const float w);
    float distance180(const float x, const float w);
    float fusionPI(const float w, const float a, const float b);

    // Filter
    float Complementary(const float newAngle, const float newRate,
                        const int looptime, float angle);
    float Complementary2(const float newAngle, const float newRate,
                         const int looptime, float angle);
    float Kalman(const float newAngle, const float newRate,
                 const int looptime, float x_angle);

    unsigned long nextTime {};
    unsigned long nextTimeControl {};
    unsigned int timeBetweenRuns { 200 }; // 5 Hz
    unsigned int timeBetweenControl { 100 }; // 10 Hz

    boolean foundNewMinMax { false };
    int pinBuzzer {};
    int callCounter {};
    int errorCounter {};
    boolean hardwareInitialized { false };
    boolean calibrationAvailable { false };
    byte state { IMU_RUN };
    unsigned long lastAHRSTime {};
    calibrationData_t calibrationData {
      { 0, 0, 0 },
      { 1, 1, 1 },
      { 0, 0, 0 },
      { 1, 1, 1 }
    };

    // --------- acceleration state ---------------------
    LSM303 accMag;
    point_float_t accelMin { 0.0, 0.0, 0.0 };
    point_float_t accelMax { 0.0, 0.0, 0.0 };
    int calibAccelAxisCounter {};
    boolean useAccelCalibration { true };

    // --------- gyro state -----------------------------
    point_int_t gyroOffset { 0, 0, 0 }; // gyro calibration data
    int gyroNoise {};          // gyro noise
    boolean useGyroCalibration { true }; // gyro calibration flag
    ypr_t ypr { 0.0, 0.0, 0.0 };  // gyro yaw,pitch,roll

    // --------- magnetometer state --------------------------
    point_int_t magLast { 0, 0, 0 };
    point_int_t magMin { 0, 0, 0 }; // magnetometer sensor data (raw)
    point_int_t magMax { 0, 0, 0 }; // magnetometer sensor data (raw)
    boolean useMagnetometerCalibration { true };

    float accelPitch {};
    float accelRoll {};
    float scaledPitch {};
    float scaledRoll {};
    float filtPitch {};
    float filtRoll {};
    float yaw {};
    float scaledYaw {};
    float scaled2Yaw {};
    float filtYaw {};
    point_float_t magTilt { 0.0, 0.0, 0.0 };
};

#endif

