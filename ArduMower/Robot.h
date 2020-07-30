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
#pragma once

#include <Arduino.h>
#include <Wire.h>

#include <EEPROM.h>

#include "Config.h"
#include "Setting.h"

#include "AdcManager.h"
#include "Battery.h"
#include "BluetoothConfig.h"
#include "Bumper.h"
#include "Button.h"
#include "Buzzer.h"
#include "Cutter.h"
#include "DifferentialDrive.h"
#include "Drivers.h"
#include "DropSensor.h"
#include "Gps.h"
#include "Imu.h"
#include "LawnSensor.h"
#include "Led.h"
#include "Odometer.h"
#include "Perimeter.h"
#include "Pid.h"
#include "RainSensor.h"
#include "RemoteControl.h"
#include "Sonar.h"
#include "Wheel.h"
#include "StateMachine.h"

extern Robot robot;

/*
 Generic robot class - subclass to implement concrete hardware!
 */

// code version
#define VERSION F("0.1")
#define MOW 2
#define MAX_TIMERS 5
#define BATTERY_SW_OFF -1
#define BUMPERS_NUM 2
#define DROPSENSORS_NUM 2
#define LAWNSENSORS_NUM 2
#define SONARS_NUM 3
#define PERIMETERSENSORS_NUM 2

// error types
enum ErrorE
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
  ERR_STATE_MACHINE,
  // <---- add new error types here (NOTE: increase MAGIC to avoid corrupt EEPROM error data!)
  ERR_ENUM_COUNT,
};
static_assert(sizeof(ErrorE) == 1, "Enums need to be 1 byte in size in this project. Use compiler flag -fshort-enums.");

enum RollDirE
{
  LEFT,
  RIGHT
};

enum
{
  FRONT,
  BACK
};

// mow patterns
enum MowPatternE
{
  MOW_RANDOM,
  MOW_LANES,
  MOW_BIDIR
};

// console mode
enum ConsoleE
{
  CONSOLE_SENSOR_COUNTERS,
  CONSOLE_SENSOR_VALUES,
  CONSOLE_PERIMETER,
  CONSOLE_IMU,
  CONSOLE_OFF,
  CONSOLE_END
};

struct Stats
{
  uint32_t mowTimeTotal_min {};
  uint16_t mowTimeTrip_min {};
  uint16_t batteryChargingCounterTotal {};
  float batteryChargingCapacityTrip_mAh {};
  uint16_t batteryChargingCapacityTotal_mAh {};
  float batteryChargingCapacityAverage_mAh {};
};

struct SettingTimerTime
{
  Setting<uint8_t> hour;
  Setting<uint8_t> minute;
};

struct SettingTimer
{
  Setting<bool> active;
  SettingTimerTime startTime;
  SettingTimerTime stopTime;
  Setting<uint8_t> daysOfWeek;
};

struct RobotSettings
{
  Setting<bool> developer;              // Developer mode enables a few advanced settings

  Setting<int16_t> perimeterTriggerTimeout; // Perimeter trigger timeout when escaping from inside (ms)
  Setting<int16_t> perimeterOutRollTimeMax; // Roll time max after perimeter out (ms)
  Setting<int16_t> perimeterOutRollTimeMin; // Roll time min after perimeter out (ms)
  Setting<int16_t> perimeterOutRevTime;     // Reverse time after perimeter out (ms)
  Setting<int16_t> perimeterTrackRollTime;  // Perimeter tracking roll time (ms)
  Setting<int16_t> perimeterTrackRevTime;   // Perimeter tracking reverse time (ms)

  Setting<bool> trackingBlockInnerWheelWhilePerimeterStruggling;

  Setting<int16_t> stationRevTime;          // Charge station reverse time (ms)
  Setting<int16_t> stationRollTime;         // Charge station roll time (ms)
  Setting<int16_t> stationForwTime;         // Charge station forward time (ms)
  Setting<int16_t> stationCheckTime;        // Charge station reverse check time (ms)

  Setting<bool> userSwitch1;            // User-defined switch 1 (default value)
  Setting<bool> userSwitch2;            // User-defined switch 2 (default value)
  Setting<bool> userSwitch3;            // User-defined switch 3 (default value)

  Setting<bool> timerUse;               // Use RTC and timer?
  SettingTimer timer[MAX_TIMERS];

  Setting<bool> gpsUse;                 // Use GPS?
  Setting<float> stuckIfGpsSpeedBelow;  // If GPS speed is below given value the mower is considered stuck
  Setting<uint16_t> gpsSpeedIgnoreTime; // How long gpsSpeed is ignored when robot switches into a new STATE (in ms)
};

class Robot
{
public:
  Robot() {};

  // robot setup
  void setup();

  // Tasks
  void tasks_continuous();
  void tasks_50ms();
  void tasks_100ms();
  void tasks_200ms();
  void tasks_250ms();
  void tasks_300ms();
  void tasks_500ms();
  void tasks_1s();
  void tasks_2s();
  void tasks_5s();
  void tasks_1m();

  // state machine
  void setNextState(StateMachine::StateE stateNew, uint8_t rollDir = LEFT);

  // settings
  void deleteUserSettings();
  void saveUserSettings();

  // other
  void setUserSwitches();
  void resetAllErrorCounters();
  const char* mowPatternName();

  // RTC
  void setRtc();

  float getGpsX() const
  {
    return m_gpsX;
  }

  float getGpsY() const
  {
    return m_gpsY;
  }

  int16_t getPerimeterCounter() const
  {
    return m_perimeterCounter;
  }

  int16_t getPerimeterMag() const
  {
    return m_perimeterMag;
  }

  float getStatsMowTimeHoursTotal() const
  {
    return m_statsMowTimeHoursTotal;
  }

  void setSpeed(int8_t speed)
  {
    m_speed = speed;
  }

  void setSteer(int8_t steer)
  {
    m_steer = steer;
  }

  bool isDeveloper() const
  {
    return m_developer;
  }

  RobotSettings* getSettings()
  {
    return &m_settings;
  }

private:
  // Error counters
  void loadErrorCounters();
  void saveErrorCounters();
  void checkErrorCounter();
  void incErrorCounter(const ErrorE errType);

  // User settings
  void loadSaveUserSettings(bool readflag);

  void loadSaveUserSettingsPid(bool readflag, uint16_t& addr,
      Pid& pid);
  void loadSaveUserSettingsBumpers(bool readflag, uint16_t& addr,
      Bumpers& bumpers);
  void loadSaveUserSettingsImu(bool readflag, uint16_t& addr,
      Imu& imu);
  void loadSaveUserSettingsOdometer(bool readflag, uint16_t& addr,
      Odometer& odometer);
  void loadSaveUserSettingsLawnSensors(bool readflag, uint16_t& addr,
      LawnSensors& lawnSensors);
  void loadSaveUserSettingsRainSensor(bool readflag, uint16_t& addr,
      RainSensor& rainSensor);
  void loadSaveUserSettingsSonar(bool readflag, uint16_t& addr,
      Sonar& sonar);
  void loadSaveUserSettingsSonars(bool readflag, uint16_t& addr,
      Sonars& sonars);
  void loadSaveUserSettingsPerimeter(bool readflag, uint16_t& addr,
      Perimeter& perimeter);
  void loadSaveUserSettingsBattery(bool readflag, uint16_t& addr,
      Battery& battery);
  void loadSaveUserSettingsDropSensors(bool readflag, uint16_t& addr,
      DropSensors& dropSensor);
  void loadSaveUserSettingsButton(bool readflag, uint16_t& addr,
      Button& button);
  void loadSaveUserSettingsDifferentialDrive(bool readflag, uint16_t& addr,
      DifferentialDrive& wheels);
  void loadSaveUserSettingsCutter(bool readflag, uint16_t& addr,
      Cutter& cutter);
  void loadSaveUserSettingsRobot(bool readflag, uint16_t& addr);

  void loadUserSettings();

  // Print settings
  void printSettingSerial();

  void printSettingSerialWheelMotors();
  void printSettingSerialCutterMotor();
  void printSettingSerialBumper();
  void printSettingSerialDropSensors();
  void printSettingSerialRainSensor();
  void printSettingSerialSonars();
  void printSettingSerialPerimeters();
  void printSettingSerialLawnSensor();
  void printSettingSerialImu();
  void printSettingSerialBattery();
  void printSettingSerialStation();
  void printSettingSerialOdometer();
  void printSettingSerialGps();
  void printSettingSerialother();
  void printSettingSerialUserSwitches();
  void printSettingSerialTimer();
  void printSettingSerialStatus();

  template <class T>
  void printSettingNameColonValue(const Setting<T>& K);

  void printSettingSerialPidK(const __FlashStringHelper* prefixStr,
      Setting<float> K);
  void printSettingSerialPid(const __FlashStringHelper* prefixStr,
      PidSettings* pidSettings_p);

  // read serial
  void readSerial();

  // Read sensors
  void readPerimeters();
  void readImu();
  void readMotorCurrents();
  void measureCutterMotorRpm();

  // Check sensors
  void checkButton();
  void checkBattery();
  void checkTimer();
  void checkMotorPower();
  void checkCutterMotorPower();
  void checkWheelMotorPower(DifferentialDrive::SideE side);
  void checkBumpers();
  void checkDrop();
  void checkBumpersPerimeter();
  void checkPerimeterBoundary();
  void checkPerimeterFind();
  void checkLawn();
  void checkSonar();
  void checkTilt();
  void checkRain();
  void checkTimeout();
  void checkOdometerFaults();
  void checkIfStuck();
  void checkRobotStats_mowTime();
  void checkRobotStats_battery();

  // Motor
  void diffDriveControl_normal();
  void diffDriveControl_imuRoll();
  void diffDriveControl_perimeter();
  void diffDriveControl_imuDir();
  void setMotorPWMs(const int16_t pwmLeft, const int16_t pwmRight,
      const bool useAccel = false);
  void setMotorPWM(int16_t pwm, const uint8_t motor, const bool useAccel);

  // Date & time
  void setDefaultTime();
  void readRtc();

  // GPS
  void processGPSData();
  void receiveGPSTime();

  // Set reverse
  void reverseOrChangeDirection(const uint8_t rollDir);

  // Other
  void printOdometer();
  void printMenu();
  void printInfo(Stream& s);
  void printInfo_perimeter(Stream& s);
  void printInfo_odometer(Stream& s);
  void printInfo_sensorValues(Stream& s);
  void printInfo_sensorCounters(Stream& s);
  void delayInfo(const int16_t ms);
  void testOdometer();
  void testMotors();
  void setDefaults();
  void menu();
  void configureBluetooth(bool quick);
  void resetIdleTime();
  const char* consoleModeName();

  // Robot statistics
  void loadRobotStats();
  void saveRobotStats();
  void deleteRobotStats();

  // State machine
  void runStateMachine();

public:
  String m_name { "Ardumower" };

  // --------- state machine --------------------------
  StateMachine m_stateMachine;

  // --------- timer ----------------------------------
  datetime_t m_datetime;

  // -------- mow pattern -----------------------------
  uint8_t m_mowPattern { MOW_RANDOM };

  // -------- gps state -------------------------------
  Gps m_gps;

  // -------- odometer state --------------------------
  Odometer m_odometer
  {
    m_diffDrive.m_wheelLeft.m_encoder,
    m_diffDrive.m_wheelRight.m_encoder,
    m_imu
  };

  // --------- wheel motor state ----------------------------
  // wheel motor speed ( <0 backward, >0 forward); range -motorSpeedMaxRpm..motorSpeedMaxRpm
  DifferentialDrive m_diffDrive;

  // -------- mower motor state -----------------------
  Cutter m_cutter;

  // --------- bumper state ---------------------------
  // bumper state (true = pressed)
  const uint8_t bumperPins[BUMPERS_NUM] = { PIN_BUMBER_LEFT, PIN_BUMBER_RIGHT };
  Bumper m_bumperArray[BUMPERS_NUM];
  Bumpers m_bumpers { bumperPins, m_bumperArray, BUMPERS_NUM };

  // --------- drop state ---------------------------
  // drop sensor
  DropSensor m_dropSensorArray[DROPSENSORS_NUM] {
    PIN_DROP_LEFT, PIN_DROP_RIGHT
  };
  DropSensors m_dropSensors { DropSensor_Contact::NO,
    m_dropSensorArray, DROPSENSORS_NUM };

  // --------- Buzzer ---------------------------
  Buzzer m_buzzer { PIN_BUZZER , true };

  // --------- Led's ---------------------------
  Led m_ledPerimeter { PIN_LED_PERIMETER };

  // ------- IMU state --------------------------------
  Imu m_imu;
  float m_imuDriveHeading {};     // drive heading (IMU)
  float m_imuRollHeading {};      // roll heading  (IMU)

  // ------- perimeter state --------------------------
  Perimeter m_perimeter {};
  bool m_perimeterUse { false }; // use perimeter?
  uint16_t m_trackingErrorTimeOut { 10000 };
  uint16_t m_trackingPerimeterTransitionTimeOut { 2000 };

  //  --------- lawn state ----------------------------
  LawnSensor m_lawnSensorArray[LAWNSENSORS_NUM];
  LawnSensors m_lawnSensors { m_lawnSensorArray, LAWNSENSORS_NUM };

  // --------- rain -----------------------------------
  RainSensor m_rainSensor;

  // --------- sonar ----------------------------------
  // ultrasonic sensor distance-to-obstacle (cm)
  Sonar m_sonarArray[SONARS_NUM];
  Sonars m_sonars { m_sonarArray, SONARS_NUM };

  // --------- pfodApp ----------------------------------
  RemoteControl m_rc { this }; // pfodApp

  // ----- other -----------------------------------------
  Button m_button { PIN_BUTTON };

  // ----- user-defined switch ---------------------------

  // --------- charging -------------------------------
  Battery m_battery
  {
    PIN_BATTERY_VOLTAGE,
    PIN_CHARGE_VOLTAGE,
    PIN_CHARGE_CURRENT,
    PIN_CHARGE_RELAY,
    PIN_BATTERY_SWITCH
  };

  // --------- error counters --------------------------
  byte m_errorCounterMax[ERR_ENUM_COUNT] {};

  // ------------robot stats---------------------------
  Stats m_stats {};

private:
  // --------- state machine ----------------------------
  int16_t m_idleTimeSec {};

  // --------- timer ------------------------------------

  // -------- gps state ---------------------------------
  float m_gpsLat {};
  float m_gpsLon {};
  float m_gpsX {};   // X position (m)
  float m_gpsY {};   // Y position (m)
  int16_t m_robotIsStuckCounter {};

  // ------- IMU state ----------------------------------
  byte m_imuRollDir { LEFT };

  // ------- perimeter state ----------------------------
  int16_t m_perimeterMag {};             // perimeter magnitude
  bool m_perimeterInside { true };      // is inside perimeter?
  uint32_t m_perimeterTriggerTime {}; // time to trigger perimeter transition (timeout)
  uint32_t m_perimeterLastTransitionTime {};
  int16_t m_perimeterCounter {};         // counts perimeter transitions

  // --------- pfodApp ----------------------------------

  // --------- driving ----------------------------------
  int8_t m_speed {}; // Range -100..+100, - = reverse, + = forward
  int8_t m_steer {}; // Range -100..+100, - = left, + = right

  // --------- error counters ---------------------------
  byte m_errorCounter[ERR_ENUM_COUNT] {};

  // --------- other ------------------------------------
  int16_t m_loopsPerSec {};  // main loops per second
  int16_t m_loopsPerSecCounter {};
  uint8_t m_consoleMode { CONSOLE_SENSOR_COUNTERS };
  uint8_t m_rollDir { LEFT };
  uint32_t m_lastTimeErrorCounterReset {};
  uint32_t m_nextTimeErrorBeep {};
  uint32_t m_lastTimeMeasureCutterRpm {};

  // ------------robot stats-----------------------------
  bool m_statsMowTimeTotalStart { false };
  uint16_t m_statsMowTimeMinutesTripCounter {};
  float m_statsMowTimeHoursTotal {};
  uint16_t m_statsBatteryChargingCounter {};

  RobotSettings m_settings
  {
    { "Developer", false },

    { "Trigger timeout", "ms", 0, 0, 2000, 1.0f },
    { "Perimeter out roll time max", "ms", 2000, 0, 8000, 1.0f },
    { "Perimeter out roll time min", "ms", 750, 0, 8000, 1.0f },
    { "Perimeter out reverse time", "ms", 2200, 0, 8000, 1.0f },
    { "Perimeter tracking roll time", "ms", 1500, 0, 8000, 1.0f },
    { "Perimeter tracking reverse time", "ms", 2200, 0, 8000, 1.0f },

    { "Block inner wheel", true },

    { "Reverse time", "ms", 1800, 0, 8000, 1.0f },
    { "Roll time", "ms", 1000, 0, 8000, 1.0f },
    { "Forward time", "ms", 1500, 0, 8000, 1.0f },
    { "Station reverse check time", "ms", 1700, 0, 8000, 1.0f },

    { "User switch 1 is", false },
    { "User switch 2 is", false },
    { "User switch 3 is", false },

    { "Use", false }, // timerUse
    {
      {
        { "Use", false }, // timer[i].active
        {
            { "Start hour", "", 9, 0, 23, 1.0f },  // timer[i].startTime.hour
            { "Start minute", "", 0, 0, 59, 1.0f }, // timer[i].startTime.minute
        },
        {
            { "Stop hour", "", 11, 0, 23, 1.0f },  // timer[i].stopTime.hour
            { "Stop minute", "", 0, 0, 59, 1.0f }, // timer[i].stopTime.minute
        },
        { "DaysOfWeek", "", B01111110, 0, 0x7F, 1.0f }, // timer[i].daysOfWeek
      },
      {
        { "Use", false }, // timer[i].active
        {
            { "Start hour", "", 9, 0, 23, 1.0f },  // timer[i].startTime.hour
            { "Start minute", "", 0, 0, 59, 1.0f }, // timer[i].startTime.minute
        },
        {
            { "Stop hour", "", 11, 0, 23, 1.0f },  // timer[i].stopTime.hour
            { "Stop minute", "", 0, 0, 59, 1.0f }, // timer[i].stopTime.minute
        },
        { "DaysOfWeek", "", B01111110, 0, 0x7F, 1.0f }, // timer[i].daysOfWeek
      },
      {
        { "Use", false }, // timer[i].active
        {
            { "Start hour", "", 9, 0, 23, 1.0f },  // timer[i].startTime.hour
            { "Start minute", "", 0, 0, 59, 1.0f }, // timer[i].startTime.minute
        },
        {
            { "Stop hour", "", 11, 0, 23, 1.0f },  // timer[i].stopTime.hour
            { "Stop minute", "", 0, 0, 59, 1.0f }, // timer[i].stopTime.minute
        },
        { "DaysOfWeek", "", B01111110, 0, 0x7F, 1.0f }, // timer[i].daysOfWeek
      },
      {
        { "Use", false }, // timer[i].active
        {
            { "Start hour", "", 9, 0, 23, 1.0f },  // timer[i].startTime.hour
            { "Start minute", "", 0, 0, 59, 1.0f }, // timer[i].startTime.minute
        },
        {
            { "Stop hour", "", 11, 0, 23, 1.0f },  // timer[i].stopTime.hour
            { "Stop minute", "", 0, 0, 59, 1.0f }, // timer[i].stopTime.minute
        },
        { "DaysOfWeek", "", B01111110, 0, 0x7F, 1.0f }, // timer[i].daysOfWeek
      },
      {
        { "Use", false }, // timer[i].active
        {
            { "Start hour", "", 9, 0, 23, 1.0f },  // timer[i].startTime.hour
            { "Start minute", "", 0, 0, 59, 1.0f }, // timer[i].startTime.minute
        },
        {
            { "Stop hour", "", 11, 0, 23, 1.0f },  // timer[i].stopTime.hour
            { "Stop minute", "", 0, 0, 59, 1.0f }, // timer[i].stopTime.minute
        },
        { "DaysOfWeek", "", B01111110, 0, 0x7F, 1.0f }, // timer[i].daysOfWeek
      },
    },

    { "Use", false },
    { "Stuck if GPS speed is below", "", 0.2f, 0.0f, 3.0f, 0.1f },
    { "GPS speed ignore time", "ms", 5000, WHEELS_MAX_REVERSE_TIME_MS, 10000, 1.0f },
  };

  // Shorter convenient variables for settings variables
  bool& m_developer = m_settings.developer.value;

  int16_t& m_perimeterTriggerTimeout = m_settings.perimeterTriggerTimeout.value;
  int16_t& m_perimeterOutRollTimeMax = m_settings.perimeterOutRollTimeMax.value;
  int16_t& m_perimeterOutRollTimeMin = m_settings.perimeterOutRollTimeMin.value;
  int16_t& m_perimeterOutRevTime = m_settings.perimeterOutRevTime.value;
  int16_t& m_perimeterTrackRollTime = m_settings.perimeterTrackRollTime.value;
  int16_t& m_perimeterTrackRevTime = m_settings.perimeterTrackRevTime.value;

  bool& m_trackingBlockInnerWheelWhilePerimeterStruggling =
      m_settings.trackingBlockInnerWheelWhilePerimeterStruggling.value;

  int16_t& m_stationRevTime = m_settings.stationRevTime.value;
  int16_t& m_stationRollTime = m_settings.stationRollTime.value;
  int16_t& m_stationForwTime = m_settings.stationForwTime.value;
  int16_t& m_stationCheckTime = m_settings.stationCheckTime.value;

  bool& m_userSwitch1 = m_settings.userSwitch1.value;
  bool& m_userSwitch2 = m_settings.userSwitch2.value;
  bool& m_userSwitch3 = m_settings.userSwitch3.value;

  bool& m_timerUse = m_settings.timerUse.value;
  //ttimer_t& m_timer:

  bool& m_gpsUse = m_settings.gpsUse.value;
  float& m_stuckIfGpsSpeedBelow = m_settings.stuckIfGpsSpeedBelow.value;
  uint16_t& m_gpsSpeedIgnoreTime = m_settings.gpsSpeedIgnoreTime.value;
};
