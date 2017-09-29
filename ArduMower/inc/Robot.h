/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2014 by Alexander Grau
 Copyright (c) 2013-2014 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri

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

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>

#include <EEPROM.h>

#include "AdcManager.h"
#include "Battery.h"
#include "Bumper.h"
#include "Button.h"
#include "Cutter.h"
#include "drivers.h"
#include "DropSensor.h"
#include "Gps.h"
#include "Imu.h"
#include "LawnSensor.h"
#include "Odometer.h"
#include "Perimeter.h"
#include "RainSensor.h"
#include "RemoteControl.h"
#include "Sonar.h"
#include "Wheel.h"
#include "StateMachine.h"

/*
 Generic robot class - subclass to implement concrete hardware!
 */

// code version
#define VERSION "0.1"

// error types
typedef enum errorE
{
  ERR_MOTOR_LEFT,
  ERR_MOTOR_RIGHT,
  ERR_MOTOR_CUTTER,
  ERR_CUTTER_SENSE,
  ERR_IMU_COMM,
  ERR_IMU_TILT,
  ERR_RTC_COMM,
  ERR_RTC_DATA,
  ERR_PERIMETER_TIMEOUT,
  ERR_TRACKING,
  ERR_ODOMETER_LEFT,
  ERR_ODOMETER_RIGHT,
  ERR_BATTERY,
  ERR_CHARGER,
  ERR_GPS_COMM,
  ERR_GPS_DATA,
  ERR_ADC_CALIB,
  ERR_IMU_CALIB,
  ERR_EEPROM_DATA,
  ERR_STUCK,
  // <---- add new error types here (NOTE: increase MAGIC to avoid corrupt EEPROM error data!)
  ERR_ENUM_COUNT,
} errorE;

#define MOW 2

enum
{
  LEFT,
  RIGHT,
  CENTER
};

enum
{
  FRONT,
  BACK
};

// mow patterns
enum
{
  MOW_RANDOM,
  MOW_LANES,
  MOW_BIDIR
};

// console mode
enum consoleE
{
  CONSOLE_SENSOR_COUNTERS,
  CONSOLE_SENSOR_VALUES,
  CONSOLE_PERIMETER,
  CONSOLE_IMU,
  CONSOLE_OFF,
  CONSOLE_END
};

#define MAX_TIMERS 5

#define BATTERY_SW_OFF -1

typedef struct statsT
{
  unsigned long mowTimeMinutesTotal {};
  unsigned int mowTimeMinutesTrip {};
  unsigned int batteryChargingCounterTotal {};
  float batteryChargingCapacityTrip {};
  float batteryChargingCapacityTotal {};
  float batteryChargingCapacityAverage {};
} statsT;

#define BUMPERS_NUM 2
#define DROPSENSORS_NUM 2
#define LAWNSENSORS_NUM 2
#define SONARS_NUM 3

class Robot
{
  public:
    // sensors
    typedef enum sensorE
    {
      SEN_RTC,
    } sensorE;

    // actuators
    typedef enum actuatorE
    {
      ACT_BUZZER,
      ACT_LED,
      ACT_USER_SW1,
      ACT_USER_SW2,
      ACT_USER_SW3,
      ACT_RTC,
      ACT_BATTERY_SW,
    } actuatorE;

    String name;
    bool developerActive;

    // --------- state machine --------------------------
    StateMachine stateMachine;

    // --------- timer ----------------------------------
    ttimer_t timer[MAX_TIMERS];
    datetime_t datetime;
    boolean timerUse { false }; // use timer?

    // -------- mow pattern -----------------------------
    byte mowPatternCurr { MOW_RANDOM };
    const char* mowPatternName();

    // -------- gps state -------------------------------
    Gps gps;
    boolean gpsUse;       // use GPS?
    float stuckIfGpsSpeedBelow;
    int gpsSpeedIgnoreTime; // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)

    // -------- odometer state --------------------------
    Odometer odometer;

    // -------- RC remote control state -----------------
    boolean remoteUse;       // use model remote control (R/C)?

    // --------- wheel motor state ----------------------------
    // wheel motor speed ( <0 backward, >0 forward); range -motorSpeedMaxRpm..motorSpeedMaxRpm
    Wheels wheels;

    // -------- mower motor state -----------------------
    Cutter cutter;

    // --------- bumper state ---------------------------
    // bumper state (true = pressed)
    Bumper bumperArray[BUMPERS_NUM];
    Bumpers bumpers;

    // --------- drop state ---------------------------
    DropSensor dropSensorArray[DROPSENSORS_NUM];
    DropSensors dropSensors;

    // ------- IMU state --------------------------------
    Imu imu;
    float imuDriveHeading;     // drive heading (IMU)
    float imuRollHeading;      // roll heading  (IMU)

    // ------- perimeter state --------------------------
    Perimeters perimeters;
    boolean perimeterUse { false }; // use perimeter?
    int perimeterOutRollTimeMax;
    int perimeterOutRollTimeMin;
    int perimeterOutRevTime;
    int perimeterTrackRollTime; // perimeter tracking roll time (ms)
    int perimeterTrackRevTime; // perimeter tracking reverse time (ms)
    int perimeterTriggerTimeout;   // perimeter trigger timeout (ms)
    int trackingErrorTimeOut;
    int trackingPerimeterTransitionTimeOut;
    boolean trackingBlockInnerWheelWhilePerimeterStruggling;

    //  --------- lawn state ----------------------------
    LawnSensor lawnSensorArray[LAWNSENSORS_NUM];
    LawnSensors lawnSensors;

    // --------- rain -----------------------------------
    RainSensor rainSensor;

    // --------- sonar ----------------------------------
    // ultrasonic sensor distance-to-obstacle (cm)
    Sonar sonarArray[SONARS_NUM];
    Sonars sonars;

    // --------- pfodApp ----------------------------------
    RemoteControl rc; // pfodApp

    // ----- other -----------------------------------------
    Button button;

    // ----- user-defined switch ---------------------------
    boolean userSwitch1 { false }; // user-defined switch 1 (default value)
    boolean userSwitch2 { false }; // user-defined switch 2 (default value)
    boolean userSwitch3 { false }; // user-defined switch 3 (default value)

    // --------- charging -------------------------------
    Battery battery;

    int stationRevTime {};    // charge station reverse time (ms)
    int stationRollTime {};    // charge station roll time (ms)
    int stationForwTime {};    // charge station forward time (ms)
    int stationCheckTime {};    // charge station reverse check time (ms)

    // --------- error counters --------------------------
    byte errorCounterMax[ERR_ENUM_COUNT] {};

    // ------------robot stats---------------------------
    statsT stats {};

    // --------------------------------------------------
    Robot();
    virtual ~Robot() {};

    // robot setup
    virtual void setup();

    virtual void tasks_continuous();
    virtual void tasks_50ms();
    virtual void tasks_100ms();
    virtual void tasks_200ms();
    virtual void tasks_250ms();
    virtual void tasks_300ms();
    virtual void tasks_500ms();
    virtual void tasks_1s();
    virtual void tasks_2s();
    virtual void tasks_5s();
    virtual void tasks_1m();

    virtual void resetIdleTime();

    // call this from R/C control interrupt
    virtual void setRemotePPMState(const unsigned long timeMicros,
                                   const boolean remoteSpeedState,
                                   const boolean remoteSteerState,
                                   const boolean remoteMowState,
                                   const boolean remoteSwitchState);

    // state machine
    virtual void setNextState(StateMachine::stateE stateNew, bool dir = LEFT);

    // motor
    virtual void setMotorPWMs(const int pwmLeft, const int pwmRight,
                              const boolean useAccel = false);

    // GPS
    virtual void processGPSData();

    // read hardware sensor (HAL)
    virtual int readSensor(Robot::sensorE type)
    {
      (void)type;
      return 0;
    }

    // set hardware actuator (HAL)
    virtual void setActuator(Robot::actuatorE type, int value)
    {
      (void)type;
      (void)value;
    }

    // settings
    virtual void deleteUserSettings();
    virtual void saveUserSettings();
    virtual void deleteRobotStats();

    // other
    virtual void beep(const uint8_t numberOfBeeps,
                      const bool shortbeep = false);
    virtual void printInfo(Stream &s);
    virtual void printInfo_perimeter(Stream &s);
    virtual void printInfo_odometer(Stream &s);
    virtual void printInfo_sensorValues(Stream &s);
    virtual void printInfo_sensorCounters(Stream &s);
    virtual void setUserSwitches();
    virtual void incErrorCounter(const enum errorE errType);
    virtual void resetErrorCounters();

    float getGpsX() const
    {
      return gpsX;
    }

    float getGpsY() const
    {
      return gpsY;
    }

    int getPerimeterCounter() const
    {
      return perimeterCounter;
    }

    int getPerimeterMag() const
    {
      return perimeterMag;
    }

    float getStatsMowTimeHoursTotal() const
    {
      return statsMowTimeHoursTotal;
    }

    int8_t getSpeed() const
    {
      return speed;
    }

    void setSpeed(int8_t speed)
    {
      this->speed = speed;
    }

    int8_t getSteer() const
    {
      return steer;
    }

    void setSteer(int8_t steer)
    {
      this->steer = steer;
    }

  protected:
    // convert ppm time to RC slider value
    virtual int rcValue(const int ppmTime);
    virtual void loadErrorCounters();
    virtual void saveErrorCounters();
    virtual void loadSaveUserSettings(const boolean readflag);
    virtual void loadUserSettings();
    virtual void checkErrorCounter();
    virtual void printSettingSerial();

    // read sensors
    virtual void readPerimeters();
    virtual void readRtc();
    virtual void readImu();
    virtual void readCutterMotorCurrent();
    virtual void measureCutterMotorRpm();
    // read serial
    virtual void readSerial();

    // check sensor
    virtual void checkButton();
    virtual void checkBattery();
    virtual void checkTimer();
    virtual void checkMotorPower();
    virtual void checkCutterMotorPower();
    virtual void checkWheelMotorPower(Wheel::wheelE side);
    virtual void checkBumpers();
    virtual void checkDrop();
    virtual void checkBumpersPerimeter();
    virtual void checkPerimeterBoundary();
    virtual void checkPerimeterFind();
    virtual void checkLawn();
    virtual void checkSonar();
    virtual void checkTilt();
    virtual void checkRain();
    virtual void checkTimeout();
    virtual void checkOdometerFaults();
    virtual void checkIfStuck();
    virtual void checkRobotStats_mowTime();
    virtual void checkRobotStats_battery();


    // motor controllers
    virtual void wheelControl_normal();
    virtual void wheelControl_imuRoll();
    virtual void wheelControl_perimeter();
    virtual void wheelControl_imuDir();
    virtual void cutterControl();

    // date & time
    virtual void setDefaultTime();

    // set reverse
    virtual void reverseOrChangeDirection(const byte aRollDir);

    // other
    virtual void printRemote();
    virtual void printOdometer();
    virtual void printMenu();
    virtual void delayInfo(const int ms);
    virtual void testOdometer();
    virtual void testMotors();
    virtual void setDefaults();
    virtual void receiveGPSTime();
    virtual void menu();
    virtual void configureBluetooth(boolean quick)
    {
      (void)quick;
    }


  private:
    // --------- state machine ----------------------------
    int idleTimeSec {};

    // --------- timer ------------------------------------

    // -------- gps state ---------------------------------
    float gpsLat;
    float gpsLon;
    float gpsX;   // X position (m)
    float gpsY;   // Y position (m)
    int robotIsStuckCounter;

    // -------- RC remote control state -------------------
    int remoteSteer;  // range -100..100
    int remoteSpeed;  // range -100..100
    int remoteMow;    // range 0..100
    int remoteSwitch; // range 0..100
    unsigned long remoteSteerLastTime;
    unsigned long remoteSpeedLastTime;
    unsigned long remoteMowLastTime;
    unsigned long remoteSwitchLastTime;
    boolean remoteSteerLastState;
    boolean remoteSpeedLastState;
    boolean remoteMowLastState;
    boolean remoteSwitchLastState;

    // ------- IMU state ----------------------------------
    byte imuRollDir;

    // ------- perimeter state ----------------------------
    int perimeterMag;             // perimeter magnitude
    boolean perimeterInside;      // is inside perimeter?
    unsigned long perimeterTriggerTime; // time to trigger perimeter transition (timeout)
    unsigned long perimeterLastTransitionTime;
    int perimeterCounter;         // counts perimeter transitions

    // --------- pfodApp ----------------------------------

    // --------- driving ----------------------------------
    int8_t speed {}; // Range -100..+100, - = reverse, + = forward
    int8_t steer {}; // Range -100..+100, - = left, + = right

    // --------- error counters ---------------------------
    byte errorCounter[ERR_ENUM_COUNT] {};

    // --------- other ------------------------------------
    int loopsPerSec {};  // main loops per second
    int loopsPerSecCounter {};
    byte consoleMode { CONSOLE_SENSOR_COUNTERS };
    bool rollDir;
    unsigned long nextTimeErrorCounterReset;
    unsigned long nextTimeErrorBeep;

    // ------------robot stats-----------------------------
    boolean statsMowTimeTotalStart { false };
    unsigned int statsMowTimeMinutesTripCounter {};
    float statsMowTimeHoursTotal {};
    unsigned int statsBatteryChargingCounter {};


    void setMotorPWM(int pwm, const uint8_t motor, const boolean useAccel);
    void loadRobotStats();
    void saveRobotStats();
    void runStateMachine();
};

#endif
