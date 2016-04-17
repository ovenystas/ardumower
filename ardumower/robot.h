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

//#include <Servo.h>  // for RC brushless controller
#include "drivers.h"
#include "pid.h"
#include "imu.h"
#include "adcman.h"
#include "perimeter.h"
#include "gps.h"
#include "pfod.h"
#include "button.h"
#include "bumper.h"
#include "dropSensor.h"
#include "sonar.h"
#include "lawnSensor.h"
#include "odometer.h"
#include "rainSensor.h"
#include "cutter.h"
#include "wheel.h"

//#include "QueueList.h"
//#include <limits.h>

/*
 Generic robot class - subclass to implement concrete hardware!
 */

// code version
#define VER "1.0a4-Azurit"

// sensors
enum
{
  SEN_PERIM_LEFT,        // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT,       // 0..MAX_PERIMETER
  SEN_PERIM_LEFT_EXTRA,  // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT_EXTRA, // 0..MAX_PERIMETER
  SEN_BAT_VOLTAGE,       // Volt * 100
  SEN_CHG_CURRENT,       // Ampere * 100
  SEN_CHG_VOLTAGE,       // Volt * 100
  SEN_IMU,
  SEN_MOTOR_MOW_RPM,
  SEN_RTC,
};

// actuators
enum
{
  ACT_BUZZER,
  ACT_LED,
  ACT_USER_SW1,
  ACT_USER_SW2,
  ACT_USER_SW3,
  ACT_RTC,
  ACT_CHGRELAY,
  ACT_BATTERY_SW,
};

// error types
enum errorE
{
  ERR_MOTOR_LEFT,
  ERR_MOTOR_RIGHT,
  ERR_MOTOR_MOW,
  ERR_MOW_SENSE,
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
};

// finite state machine states
enum
{
  STATE_OFF,              // off
  STATE_REMOTE,           // model remote control (R/C)
  STATE_FORWARD,          // drive forward
  STATE_ROLL,             // drive roll right/left
  STATE_REVERSE,          // drive reverse
  STATE_CIRCLE,           // drive circle
  STATE_ERROR,            // error
  STATE_PERI_FIND,        // perimeter find
  STATE_PERI_TRACK,       // perimeter track
  STATE_PERI_ROLL,        // perimeter roll
  STATE_PERI_REV,         // perimeter reverse
  STATE_STATION,          // in station
  STATE_STATION_CHARGING, // in station charging
  STATE_STATION_CHECK,    //checks if station is present
  STATE_STATION_REV,      // charge reverse
  STATE_STATION_ROLL,     // charge roll
  STATE_STATION_FORW,     // charge forward
  STATE_MANUAL,           // manual navigation
  STATE_ROLL_WAIT,        // drive roll right/left
  STATE_PERI_OUT_FORW,    // outside perimeter forward driving without checkPerimeterBoundary()
  STATE_PERI_OUT_REV,     // outside perimeter reverse driving without checkPerimeterBoundary()
  STATE_PERI_OUT_ROLL,    // outside perimeter rolling driving without checkPerimeterBoundary()
};

#define MOW 2

enum
{
  LEFT,
  RIGHT
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
  CONSOLE_OFF
};

#define MAX_TIMERS 5

#define BATTERY_SW_OFF -1

class Robot
{
  public:
    String name;
    bool developerActive;

    // --------- state machine --------------------------
    byte stateCurr { STATE_OFF };
    byte stateLast { STATE_OFF };
    byte stateNext { STATE_OFF };
    unsigned long stateTime {};
    char* stateName();
    unsigned long stateStartTime;
    unsigned long stateEndTime;
    int idleTimeSec {};

    // --------- timer ----------------------------------
    ttimer_t timer[MAX_TIMERS];
    datetime_t datetime;
    boolean timerUse;       // use timer?
    unsigned long nextTimeTimer;

    // -------- mow pattern -----------------------------
    byte mowPatternCurr { MOW_RANDOM };
    char *mowPatternName();

    // -------- gps state -------------------------------
    GPS gps;
    boolean gpsUse;       // use GPS?
    float gpsLat;
    float gpsLon;
    float gpsX;   // X position (m)
    float gpsY;   // Y position (m)
    unsigned long nextTimeGPS;
    unsigned long nextTimeCheckIfStucked;
    float stuckedIfGpsSpeedBelow;
    int gpsSpeedIgnoreTime; // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)
    int robotIsStuckedCounter;

    // -------- odometer state --------------------------
    Odometer odometer;

    // -------- RC remote control state -----------------
    boolean remoteUse;       // use model remote control (R/C)?
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
    unsigned long nextTimeRTC;

    // --------- wheel motor state ----------------------------
    // wheel motor speed ( <0 backward, >0 forward); range -motorSpeedMaxRpm..motorSpeedMaxRpm
    Wheels wheels;

    unsigned long nextTimeMotorPerimeterControl;

    // -------- mower motor state -----------------------
    Cutter cutter;

    // --------- bumper state ---------------------------
    // bumper state (true = pressed)
    Bumpers bumpers;

    // --------- drop state ---------------------------
    DropSensors dropSensors;

    // ------- IMU state --------------------------------
    IMU imu;
    float imuDriveHeading;       // drive heading (IMU)
    float imuRollHeading;      // roll heading  (IMU)
    byte imuRollDir;
    //point_float_t accMin;
    //point_float_t accMax;
    unsigned long nextTimeCheckTilt; // check if

    // ------- perimeter state --------------------------
    Perimeter perimeter;
    boolean perimeterUse;      // use perimeter?
    int perimeterOutRollTimeMax;
    int perimeterOutRollTimeMin;
    int perimeterOutRevTime;
    int perimeterTrackRollTime; // perimeter tracking roll time (ms)
    int perimeterTrackRevTime; // perimeter tracking reverse time (ms)
    int perimeterMag;             // perimeter magnitude
    boolean perimeterInside;      // is inside perimeter?
    unsigned long perimeterTriggerTime; // time to trigger perimeter transition (timeout)
    int perimeterTriggerTimeout;   // perimeter trigger timeout (ms)
    unsigned long perimeterLastTransitionTime;
    int perimeterCounter;         // counts perimeter transitions
    unsigned long nextTimePerimeter;
    int trackingPerimeterTransitionTimeOut;
    int trackingErrorTimeOut;
    boolean trackingBlockInnerWheelWhilePerimeterStruggling;

    //  --------- lawn state ----------------------------
    LawnSensor lawnSensor;

    // --------- rain -----------------------------------
    RainSensor rainSensor;

    // --------- sonar ----------------------------------
    // ultra sonic sensor distance-to-obstacle (cm)
    Sonars sonars;

    // --------- pfodApp ----------------------------------
    RemoteControl rc; // pfodApp
    unsigned long nextTimePfodLoop;

    // ----- other -----------------------------------------
    Button button;

    // ----- user-defined switch ---------------------------
    boolean userSwitch1;       // user-defined switch 1 (default value)
    boolean userSwitch2;       // user-defined switch 2 (default value)
    boolean userSwitch3;       // user-defined switch 3 (default value)

    // --------- charging -------------------------------
    boolean batMonitor;              // monitor battery and charge voltage?
    float batGoHomeIfBelow;     // drive home voltage (Volt)
    float batSwitchOffIfBelow;  // switch off if below voltage (Volt)
    int batSwitchOffIfIdle;      // switch off battery if idle for minutes
    float batFactor;     // battery conversion factor
    float batChgFactor;     // battery conversion factor
    float batFull;      // battery reference Voltage (fully charged)
    float batChargingCurrentMax; // maximum current your charger can devliver
    float batFullCurrent; // current flowing when battery is fully charged
    float startChargingIfBelow; // start charging if battery Voltage is below
    unsigned long chargingTimeout; // safety timer for charging
    int batADC;
    float chgSenseZero;       // charge current sense zero point
    float chgFactor;     // charge current conversion factor
    float chgSense; // mV/A empfindlichkeit des Ladestromsensors in mV/A (Für ACS712 5A = 185)
    byte chgChange;       // messwertumkehr von - nach +         1oder 0
    float batVoltage;  // battery voltage (Volt)
    byte chgSelection;       // Senor Auswahl
    float batRefFactor;
    float batCapacity; // battery capacity (mAh)
    float chgVoltage;  // charge voltage (Volt)
    float chgCurrent;  // charge current  (Ampere)
    int chgNull;        // Nulldurchgang Ladestromsensor
    int stationRevTime;    // charge station reverse time (ms)
    int stationRollTime;    // charge station roll time (ms)
    int stationForwTime;    // charge station forward time (ms)
    int stationCheckTime;    // charge station reverse check time (ms)
    unsigned long nextTimeBattery;
    unsigned long nextTimeCheckBattery;
    int statsBatteryChargingCounter;
    int statsBatteryChargingCounterTotal;
    float statsBatteryChargingCapacityTrip;
    float statsBatteryChargingCapacityTotal;
    float statsBatteryChargingCapacityAverage;
    float lastTimeBatCapacity;

    // --------- error counters --------------------------
    byte errorCounterMax[ERR_ENUM_COUNT] {};
    byte errorCounter[ERR_ENUM_COUNT] {};

    // --------- other ----------------------------------
    int loopsPerSec {};  // main loops per second
    int loopsPerSecCounter {};
    byte consoleMode { CONSOLE_SENSOR_COUNTERS };
    unsigned long nextTimeInfo;
    byte rollDir;
    unsigned long nextTimeErrorCounterReset;
    unsigned long nextTimeErrorBeep;

    // ------------robot stats---------------------------
    boolean statsOverride;
    boolean statsMowTimeTotalStart { false };
    unsigned int statsMowTimeMinutesTripCounter;
    unsigned long statsMowTimeMinutesTotal;
    float statsMowTimeHoursTotal;
    int statsMowTimeMinutesTrip;
    unsigned long nextTimeRobotStats;

    // --------------------------------------------------
    Robot();

    // robot setup
    virtual void setup();

    // robot main loop
    virtual void loop();

    virtual void resetIdleTime();

    // call this from R/C control interrupt
    virtual void setRemotePPMState(unsigned long timeMicros,
                                   boolean remoteSpeedState,
                                   boolean remoteSteerState,
                                   boolean remoteMowState,
                                   boolean remoteSwitchState);

    // state machine
    virtual void setNextState(byte stateNew, byte dir);

    // motor
    virtual void setMotorPWMs(int pwmLeft, int pwmRight, boolean useAccel);
    virtual void setMotorMowPWM(int pwm, boolean useAccel);

    // GPS
    virtual void processGPSData();

    // read hardware sensor (HAL)
    virtual int readSensor(char type)
    {
    }

    // set hardware actuator (HAL)
    virtual void setActuator(char type, int value)
    {
    }

    // settings
    virtual void deleteUserSettings();
    virtual void saveUserSettings();
    virtual void deleteRobotStats();

    // other
    virtual void beep(int numberOfBeeps, boolean shortbeep);
    virtual void printInfo(Stream &s);
    virtual void setUserSwitches();
    virtual void addErrorCounter(enum errorE errType);
    virtual void resetErrorCounters();

  protected:
    // convert ppm time to RC slider value
    virtual int rcValue(int ppmTime);
    virtual void loadSaveErrorCounters(boolean readflag);
    virtual void loadErrorCounters();
    virtual void saveErrorCounters();
    virtual void loadSaveUserSettings(boolean readflag);
    virtual void loadSaveRobotStats(boolean readflag);
    virtual void loadUserSettings();
    virtual void checkErrorCounter();
    virtual void printSettingSerial();

    // read sensors
    virtual void readSensors();

    // read serial
    virtual void readSerial();

    // check sensor
    virtual void checkButton();
    virtual void checkBattery();
    virtual void checkTimer();
    virtual void checkPower();
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
    virtual void checkIfStucked();
    virtual void checkRobotStats();

    // motor controllers
    virtual void motorControl();
    virtual void motorControlImuRoll();
    virtual void motorControlPerimeter();
    virtual void motorControlImuDir();
    virtual void motorMowControl();

    // date & time
    virtual void setDefaultTime();

    // set reverse
    virtual void reverseOrBidir(byte aRollDir);

    // other
    virtual void printRemote();
    virtual void printOdometer();
    virtual void printMenu();
    virtual void delayInfo(int ms);
    virtual void testOdometer();
    virtual void testMotors();
    virtual void setDefaults();
    virtual void receiveGPSTime();
    virtual void menu();
    virtual void configureBluetooth(boolean quick)
    {
    }
    ;

  private:
    void setMotorPWM(int pwm, unsigned long TaC, uint8_t motor, boolean useAccel);
    void loadRobotStats();
    void saveRobotStats();

};

#endif
