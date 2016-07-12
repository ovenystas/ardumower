/*
 Ardumower (www.ardumower.de)
 Copyright (c) 2013-2015 by Alexander Grau
 Copyright (c) 2013-2015 by Sven Gennat
 Copyright (c) 2014 by Maxime Carpentieri
 Copyright (c) 2014-2015 by Stefan Manteuffel
 Copyright (c) 2015 by Uwe Zimprich
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

#include "Robot.h"

#define MAGIC 51

#define ADDR_USER_SETTINGS 0
#define ADDR_ERR_COUNTERS 400
#define ADDR_ROBOT_STATS 800

const char* stateNames[] =
{
  "OFF ", "RC  ", "FORW", "ROLL", "REV ", "CIRC", "ERR ",
  "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK",
  "STREV", "STROL", "STFOR", "MANU", "ROLW", "POUTFOR",
  "POUTREV", "POUTROLL"
};

const char *mowPatternNames[] = { "RANDOM", "LANE", "BIDIR" };

const char* consoleModeNames[] =
{
    "sensor_counters", "sensor_values", "perimeter", "imu", "off"
};

Robot::Robot()
{
  name = "Generic";
  developerActive = false;
  rc.setRobot(this);

  remoteSteer = 0;
  remoteSpeed = 0;
  remoteMow = 0;
  remoteSwitch = 0;
  remoteSteerLastTime = 0;
  remoteSpeedLastTime = 0;
  remoteMowLastTime = 0;
  remoteSwitchLastTime = 0;
  remoteSteerLastState = 0;
  remoteSpeedLastState = 0;
  remoteMowLastState = 0;
  remoteSwitchLastState = LOW;

  gpsLat = 0;
  gpsLon = 0;
  gpsX = 0;
  gpsY = 0;
  robotIsStuckCounter = 0;

  imuDriveHeading = 0;
  imuRollHeading = 0;
  imuRollDir = LEFT;

  perimeterMag = 0;
  perimeterInside = true;
  perimeterCounter = 0;
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;
  perimeterOutRevTime = 0;

  batADC = 0;
  batVoltage = 0;
  batCapacity = 0;
  batChgFactor = 0;
  lastTimeBatCapacity = 0;
  chgVoltage = 0;
  chgCurrent = 0;

  nextTimeInfo = 0;
  nextTimeCheckTilt = 0;
  nextTimeBattery = 0;
  nextTimeCheckBattery = 0;
  nextTimePerimeter = 0;
  nextTimeTimer = millis() + 60000;
  nextTimeRTC = 0;
  nextTimeGPS = 0;
  nextTimeCheckIfStuck = 0;
  nextTimePfodLoop = 0;
  nextTimeErrorCounterReset = 0;
  nextTimeErrorBeep = 0;

  nextTimeRobotStats = 0;
  statsMowTimeMinutesTripCounter = 0;
  statsBatteryChargingCounter = 0;
}

const char* Robot::stateName()
{
  return stateNames[stateCurr];
}

const char *Robot::mowPatternName()
{
  return mowPatternNames[mowPatternCurr];
}

void Robot::loadSaveRobotStats(boolean readflag)
{
  if (readflag)
  {
    Console.println(F("Loading RobotStats"));
  }
  else
  {
    Console.println(F("Saving RobotStats"));
  }
  int addr = ADDR_ROBOT_STATS;
  short magic = 0;
  if (!readflag)
  {
    magic = MAGIC;
  }
  eereadwrite(readflag, addr, magic); // magic
  if (readflag && (magic != MAGIC))
  {
    Console.println(F("PLEASE CHECK IF YOUR ROBOT STATS ARE CORRECT"));
  }
  eereadwrite(readflag, addr, statsMowTimeMinutesTrip);
  eereadwrite(readflag, addr, statsMowTimeMinutesTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCounterTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTrip);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityTotal);
  eereadwrite(readflag, addr, statsBatteryChargingCapacityAverage);
  // <----------------------------new robot stats to save goes here!----------------
  Console.print(F("loadSaveRobotStats: addrstop="));
  Console.println(addr);
}

void Robot::loadRobotStats()
{
  loadSaveRobotStats(true);
}

void Robot::saveRobotStats()
{
  loadSaveRobotStats(false);
}

void Robot::loadSaveErrorCounters(const boolean readflag)
{
  if (readflag)
  {
    Console.println(F("Loading ErrorCounters"));
  }
  else
  {
    Console.println(F("Saving ErrorCounters"));
  }

  int addr = ADDR_ERR_COUNTERS;
  short magic = 0;
  if (!readflag)
  {
    magic = MAGIC;
  }
  eereadwrite(readflag, addr, magic); // magic
  if (readflag && (magic != MAGIC))
  {
    Console.println(F("EEPROM ERR COUNTERS: NO EEPROM ERROR DATA"));
    Console.println(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    incErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR);
    return;
  }
  eereadwrite(readflag, addr, errorCounterMax);
  Console.print(F("loadSaveErrorCounters: addrstop="));
  Console.println(addr);
}

void Robot::loadErrorCounters()
{
  loadSaveErrorCounters(true);
}

void Robot::saveErrorCounters()
{
  loadSaveErrorCounters(false);
}

void Robot::loadSaveUserSettings(const boolean readflag)
{
  int addr = ADDR_USER_SETTINGS;
  short magic = 0;
  if (!readflag)
  {
    magic = MAGIC;
  }
  eereadwrite(readflag, addr, magic); // magic
  if (readflag && (magic != MAGIC))
  {
    Console.println(F("EEPROM USERDATA: NO EEPROM USER DATA"));
    Console.println(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    incErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR);
    return;
  }
  eereadwrite(readflag, addr, developerActive);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.acceleration);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.rpmMax);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.pwmMax);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.powerMax);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::RIGHT].motor.scale);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.scale);
  eereadwrite(readflag, addr, wheels.rollTimeMax);
  eereadwrite(readflag, addr, wheels.rollTimeMin);
  eereadwrite(readflag, addr, wheels.reverseTime);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime);
  eereadwrite(readflag, addr, wheels.forwardTimeMax);
  eereadwrite(readflag, addr, cutter.motor.pwmMax);
  eereadwrite(readflag, addr, cutter.motor.powerMax);
  eereadwrite(readflag, addr, cutter.motor.rpmSet);
  eereadwrite(readflag, addr, cutter.motor.scale);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.pid.Kp);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.pid.Ki);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.pid.Kd);
  eereadwrite(readflag, addr, cutter.motor.pid.Kp);
  eereadwrite(readflag, addr, cutter.motor.pid.Ki);
  eereadwrite(readflag, addr, cutter.motor.pid.Kd);
  eereadwrite(readflag, addr, wheels.biDirSpeedRatio1);
  eereadwrite(readflag, addr, wheels.biDirSpeedRatio2);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.swapDir);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::RIGHT].motor.swapDir);
  eereadwrite(readflag, addr, bumpers.used);
  eereadwrite(readflag, addr, sonars.use);
  for (uint8_t i = 0; i < Sonars::END; i++)
  {
    eereadwrite(readflag, addr, sonars.sonar[i].use);
  }
  eereadwrite(readflag, addr, sonars.triggerBelow);
  eereadwrite(readflag, addr, perimeters.use);
  eereadwrite(readflag, addr,
              perimeters.perimeter[Perimeter::LEFT].timedOutIfBelowSmag);
  eereadwrite(readflag, addr, perimeterTriggerTimeout);
  eereadwrite(readflag, addr, perimeterOutRollTimeMax);
  eereadwrite(readflag, addr, perimeterOutRollTimeMin);
  eereadwrite(readflag, addr, perimeterOutRevTime);
  eereadwrite(readflag, addr, perimeterTrackRollTime);
  eereadwrite(readflag, addr, perimeterTrackRevTime);
  eereadwrite(readflag, addr, perimeters.perimeter[Perimeter::LEFT].pid.Kp);
  eereadwrite(readflag, addr, perimeters.perimeter[Perimeter::LEFT].pid.Ki);
  eereadwrite(readflag, addr, perimeters.perimeter[Perimeter::LEFT].pid.Kd);
  eereadwrite(
      readflag, addr,
      perimeters.perimeter[Perimeter::LEFT].useDifferentialPerimeterSignal);
  eereadwrite(readflag, addr,
              perimeters.perimeter[Perimeter::LEFT].swapCoilPolarity);
  eereadwrite(readflag, addr,
              perimeters.perimeter[Perimeter::LEFT].timeOutSecIfNotInside);
  eereadwrite(readflag, addr, trackingBlockInnerWheelWhilePerimeterStruggling);
  eereadwrite(readflag, addr, lawnSensor.use);
  eereadwrite(readflag, addr, imu.use);
  eereadwrite(readflag, addr, imu.correctDir);
  eereadwrite(readflag, addr, imu.pid[Imu::DIR].Kp);
  eereadwrite(readflag, addr, imu.pid[Imu::DIR].Ki);
  eereadwrite(readflag, addr, imu.pid[Imu::DIR].Kd);
  eereadwrite(readflag, addr, imu.pid[Imu::ROLL].Kp);
  eereadwrite(readflag, addr, imu.pid[Imu::ROLL].Ki);
  eereadwrite(readflag, addr, imu.pid[Imu::ROLL].Kd);
  eereadwrite(readflag, addr, remoteUse);
  eereadwrite(readflag, addr, batMonitor);
  eereadwrite(readflag, addr, batGoHomeIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfBelow);
  eereadwrite(readflag, addr, batSwitchOffIfIdle);
  eereadwrite(readflag, addr, batFactor);
  eereadwrite(readflag, addr, batChgFactor);
  eereadwrite(readflag, addr, chgSenseZero);
  eereadwrite(readflag, addr, chgFactor);
  eereadwrite(readflag, addr, batFullCurrent);
  eereadwrite(readflag, addr, startChargingIfBelow);
  eereadwrite(readflag, addr, stationRevTime);
  eereadwrite(readflag, addr, stationRollTime);
  eereadwrite(readflag, addr, stationForwTime);
  eereadwrite(readflag, addr, stationCheckTime);
  eereadwrite(readflag, addr, odometer.use);
  eereadwrite(readflag, addr, odometer.ticksPerRevolution);
  eereadwrite(readflag, addr, odometer.ticksPerCm);
  eereadwrite(readflag, addr, odometer.wheelBaseCm);
  eereadwrite(readflag, addr, odometer.encoder.left_p->swapDir);
  eereadwrite(readflag, addr, odometer.encoder.right_p->swapDir);
  eereadwrite(readflag, addr, odometer.encoder.left_p->twoWay);
  eereadwrite(readflag, addr, button.use);
  eereadwrite(readflag, addr, userSwitch1);
  eereadwrite(readflag, addr, userSwitch2);
  eereadwrite(readflag, addr, userSwitch3);
  eereadwrite(readflag, addr, timerUse);
  eereadwrite(readflag, addr, timer);
  eereadwrite(readflag, addr, rainSensor.use);
  eereadwrite(readflag, addr, gpsUse);
  eereadwrite(readflag, addr, stuckIfGpsSpeedBelow);
  eereadwrite(readflag, addr, gpsSpeedIgnoreTime);
  eereadwrite(readflag, addr, dropSensors.used);
  eereadwrite(readflag, addr, statsOverride);
  Console.print(F("loadSaveUserSettings addrstop="));
  Console.println(addr);
}

void Robot::loadUserSettings()
{
  Console.println(F("USER SETTINGS ARE LOADED"));
  loadSaveUserSettings(true);
}

void Robot::saveUserSettings()
{
  Console.println(F("USER SETTINGS ARE SAVED"));
  loadSaveUserSettings(false);
}

void Robot::deleteUserSettings()
{
  loadRobotStats();
  int addr = 0;
  Console.println(F("ALL USER SETTINGS ARE DELETED"));
  eewrite(addr, (short)0); // magic
  saveRobotStats();
}

void Robot::printSettingSerial()
{
  // ------- wheel motors -----------------------------
  Console.println(F("== Wheels motors =="));
  Console.print(F("acceleration : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.acceleration);
  Console.print(F("rpmMax : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.rpmMax);
  Console.print(F("pwmMax : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.pwmMax);
  Console.print(F("powerMax : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.powerMax);
  Console.print(F("LEFT.scale : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.getScale());
  Console.print(F("RIGHT.scale : "));
  Console.println(wheels.wheel[Wheel::RIGHT].motor.getScale());
  Console.print(F("powerIgnoreTime : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime);
  Console.print(F("zeroSettleTime : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.zeroSettleTime);

  Console.print(F("rollTimeMax : "));
  Console.println(wheels.rollTimeMax);
  Console.print(F("rollTimeMin : "));
  Console.println(wheels.rollTimeMin);
  Console.print(F("reverseTime : "));
  Console.println(wheels.reverseTime);
  Console.print(F("forwardTimeMax : "));
  Console.println(wheels.forwardTimeMax);
  Console.print(F("biDirSpeedRatio1 : "));
  Console.println(wheels.biDirSpeedRatio1);
  Console.print(F("biDirSpeedRatio2 : "));
  Console.println(wheels.biDirSpeedRatio2);

  Console.print(F("LEFT.pid.Kp : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.pid.Kp);
  Console.print(F("LEFT.pid.Ki : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.pid.Ki);
  Console.print(F("LEFT.pid.Kd : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.pid.Kd);

  Console.print(F("RIGHT.pid.Kp : "));
  Console.println(wheels.wheel[Wheel::RIGHT].motor.pid.Kp);
  Console.print(F("RIGHT.pid.Ki : "));
  Console.println(wheels.wheel[Wheel::RIGHT].motor.pid.Ki);
  Console.print(F("RIGHT.pid.Kd : "));
  Console.println(wheels.wheel[Wheel::RIGHT].motor.pid.Kd);

  Console.print(F("LEFT.swapDir : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.swapDir);
  Console.print(F("RIGHT.swapDir : "));
  Console.println(wheels.wheel[Wheel::RIGHT].motor.swapDir);

  // ------ cutter motor -------------------------------
  Console.println(F("== Cutter motor =="));
  Console.print(F("acceleration : "));
  Console.println(cutter.motor.acceleration);
  Console.print(F("pwmMax : "));
  Console.println(cutter.motor.pwmMax);
  Console.print(F("powerMax : "));
  Console.println(cutter.motor.powerMax);
  Console.print(F("regulate : "));
  Console.println(cutter.motor.regulate);
  Console.print(F("rpmSet : "));
  Console.println(cutter.motor.rpmSet);
  Console.print(F("scale : "));
  Console.println(cutter.motor.getScale());
  Console.print(F("pid.Kp : "));
  Console.println(cutter.motor.pid.Kp);
  Console.print(F("pid.Ki : "));
  Console.println(cutter.motor.pid.Ki);
  Console.print(F("pid.Kd : "));
  Console.println(cutter.motor.pid.Kd);

  // ------ bumper ------------------------------------
  Console.println(F("== Bumpers =="));
  Console.print(F("use : "));
  Console.println(bumpers.used);

  // ------ drop ------------------------------------
  Console.println(F("== Drop sensors =="));
  Console.print(F("use : "));
  Console.println(dropSensors.used);
  Console.print(F("contactType : "));
  Console.println(dropSensors.dropSensor[DropSensors::LEFT].getContactType()); // Assume left and right has same contact type

  // ------ rain ------------------------------------
  Console.println(F("== Rain sensor =="));
  Console.print(F("use : "));
  Console.println(rainSensor.use);

  // ------ sonar ------------------------------------
  Console.println(F("== Sonars =="));
  Console.print(F("use : "));
  Console.println(sonars.use);
  Console.print(F("LEFT.use : "));
  Console.println(sonars.sonar[Sonars::LEFT].use);
  Console.print(F("CENTER.use : "));
  Console.println(sonars.sonar[Sonars::CENTER].use);
  Console.print(F("RIGHT.use : "));
  Console.println(sonars.sonar[Sonars::RIGHT].use);
  Console.print(F("triggerBelow : "));
  Console.println(sonars.triggerBelow);

  // ------ perimeter ---------------------------------
  Console.println(F("== Perimeter =="));
  Console.print(F("use : "));
  Console.println(perimeters.use);
  Console.print(F("triggerTimeout : "));
  Console.println(perimeterTriggerTimeout);
  Console.print(F("outRollTimeMax : "));
  Console.println(perimeterOutRollTimeMax);
  Console.print(F("outRollTimeMin : "));
  Console.println(perimeterOutRollTimeMin);
  Console.print(F("outRevTime : "));
  Console.println(perimeterOutRevTime);
  Console.print(F("trackRollTime : "));
  Console.println(perimeterTrackRollTime);
  Console.print(F("trackRevTime : "));
  Console.println(perimeterTrackRevTime);
  Console.print(F("pid.Kp : "));
  Console.println(perimeters.perimeter[Perimeter::LEFT].pid.Kp);
  Console.print(F("pid.Ki : "));
  Console.println(perimeters.perimeter[Perimeter::LEFT].pid.Ki);
  Console.print(F("pid.Kd : "));
  Console.println(perimeters.perimeter[Perimeter::LEFT].pid.Kd);
  Console.print(F("trackingPerimeterTransitionTimeOut : "));
  Console.println(trackingPerimeterTransitionTimeOut);
  Console.print(F("trackingErrorTimeOut : "));
  Console.println(trackingErrorTimeOut);
  Console.print(F("trackingBlockInnerWheelWhilePerimeterStruggling : "));
  Console.println(trackingBlockInnerWheelWhilePerimeterStruggling);

  // ------ lawn sensor --------------------------------
  Console.println(F("== Lawn sensor =="));
  Console.print(F("use : "));
  Console.println(lawnSensor.use);

  // ------  IMU (compass/accel/gyro) ----------------------
  Console.println(F("== IMU =="));
  Console.print(F("use : "));
  Console.println(imu.use);
  Console.print(F("correctDir : "));
  Console.println(imu.correctDir);
  Console.print(F("pid[DIR].Kp : "));
  Console.println(imu.pid[Imu::DIR].Kp);
  Console.print(F("pid[DIR].Ki : "));
  Console.println(imu.pid[Imu::DIR].Ki);
  Console.print(F("pid[DIR].Kd : "));
  Console.println(imu.pid[Imu::DIR].Kd);
  Console.print(F("pid[ROLL].Kp : "));
  Console.println(imu.pid[Imu::ROLL].Kp);
  Console.print(F("pid[ROLL].Ki : "));
  Console.println(imu.pid[Imu::ROLL].Ki);
  Console.print(F("pid[ROLL].Kd : "));
  Console.println(imu.pid[Imu::ROLL].Kd);

  // ------ model R/C ------------------------------------
  Console.println(F("== R/C =="));
  Console.print(F("use : "));
  Console.println(remoteUse);

  // ------ battery -------------------------------------
  Console.println(F("== Battery =="));
  Console.print(F("batMonitor : "));
  Console.println(batMonitor);
  Console.print(F("batGoHomeIfBelow : "));
  Console.println(batGoHomeIfBelow);
  Console.print(F("batSwitchOffIfBelow : "));
  Console.println(batSwitchOffIfBelow);
  Console.print(F("batSwitchOffIfIdle : "));
  Console.println(batSwitchOffIfIdle);
  Console.print(F("batFactor : "));
  Console.println(batFactor);
  Console.print(F("batChgFactor : "));
  Console.println(batChgFactor);
  Console.print(F("batFull : "));
  Console.println(batFull);
  Console.print(F("batChargingCurrentMax : "));
  Console.println(batChargingCurrentMax);
  Console.print(F("batFullCurrent : "));
  Console.println(batFullCurrent);
  Console.print(F("startChargingIfBelow : "));
  Console.println(startChargingIfBelow);
  Console.print(F("chargingTimeout : "));
  Console.println(chargingTimeout);
  Console.print(F("chgSelection : "));
  Console.println(chgSelection);
  Console.print(F("chgSenseZero : "));
  Console.println(chgSenseZero);
  Console.print(F("chgFactor : "));
  Console.println(chgFactor);
  Console.print(F("chgSense : "));
  Console.println(chgSense);
  Console.print(F("chgChange : "));
  Console.println(chgChange);
  Console.print(F("chgNull : "));
  Console.println(chgNull);

  // ------  charging station ---------------------------
  Console.println(F("== Station =="));
  Console.print(F("reverseTime : "));
  Console.println(stationRevTime);
  Console.print(F("rollTime : "));
  Console.println(stationRollTime);
  Console.print(F("forwardTime : "));
  Console.println(stationForwTime);
  Console.print(F("checkTime : "));
  Console.println(stationCheckTime);

  // ------ odometer ------------------------------------
  Console.println(F("== Odometer =="));
  Console.print(F("use : "));
  Console.println(odometer.use);
  Console.print(F("twoWay : "));
  Console.println(odometer.encoder.left_p->twoWay);
  Console.print(F("ticksPerRevolution : "));
  Console.println(odometer.ticksPerRevolution);
  Console.print(F("ticksPerCm : "));
  Console.println(odometer.ticksPerCm);
  Console.print(F("wheelBaseCm : "));
  Console.println(odometer.wheelBaseCm);
  Console.print(F("LEFT.swapDir : "));
  Console.println(odometer.encoder.left_p->swapDir);
  Console.print(F("RIGHT.swapDir : "));
  Console.println(odometer.encoder.right_p->swapDir);

  // ----- GPS -------------------------------------------
  Console.println(F("== GPS =="));
  Console.print(F("use : "));
  Console.println(gpsUse);
  Console.print(F("stuckIfGpsSpeedBelow : "));
  Console.println(stuckIfGpsSpeedBelow);
  Console.print(F("gpsSpeedIgnoreTime : "));
  Console.println(gpsSpeedIgnoreTime);

  // ----- other -----------------------------------------
  Console.println(F("== Button =="));
  Console.print(F("use : "));
  Console.println(button.use);

  // ----- user-defined switch ---------------------------
  Console.println(F("== User switches =="));
  Console.print(F("userSwitch1 : "));
  Console.println(userSwitch1);
  Console.print(F("userSwitch2 : "));
  Console.println(userSwitch2);
  Console.print(F("userSwitch3 : "));
  Console.println(userSwitch3);

  // ----- timer -----------------------------------------
  Console.println(F("== Timer =="));
  Console.print(F("use : "));
  Console.println(timerUse);

  // -------robot stats------------------------------------
  Console.println(F("== Robot status =="));
  Console.print(F("mowTimeMinutesTrip : "));
  Console.println(statsMowTimeMinutesTrip);
  Console.print(F("mowTimeMinutesTotal : "));
  Console.println(statsMowTimeMinutesTotal);
  Console.print(F("batteryChargingCounterTotal : "));
  Console.println(statsBatteryChargingCounterTotal);
  Console.print(F("batteryChargingCapacityTrip in mAh : "));
  Console.println(statsBatteryChargingCapacityTrip);
  Console.print(F("batteryChargingCapacityTotal in Ah : "));
  Console.println(statsBatteryChargingCapacityTotal / 1000);
  Console.print(F("batteryChargingCapacityAverage in mAh : "));
  Console.println(statsBatteryChargingCapacityAverage);
}

void Robot::deleteRobotStats()
{
  statsMowTimeMinutesTrip = 0;
  statsMowTimeMinutesTotal = 0;
  statsBatteryChargingCounterTotal = 0;
  statsBatteryChargingCapacityTotal = 0;
  statsBatteryChargingCapacityTrip = 0;
  saveRobotStats();
  Console.println(F("ALL ROBOT STATS ARE DELETED"));
}

void Robot::incErrorCounter(const enum errorE errType)
{
  // increase error counters (both temporary and maximum error counters)
  if (errorCounter[errType] < 255)
  {
    errorCounter[errType]++;
  }

  if (errorCounterMax[errType] < 255)
  {
    errorCounterMax[errType]++;
  }
}

void Robot::resetErrorCounters()
{
  Console.println(F("Reset Error Counters"));
  memset(errorCounter, 0, sizeof(errorCounter));
  memset(errorCounterMax, 0, sizeof(errorCounterMax));
  saveErrorCounters();
}

void Robot::checkErrorCounter()
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeErrorCounterReset)
  {
    // reset all temporary error counters after 30 seconds
    // (maximum error counters still continue to count)
    memset(errorCounter, 0, sizeof(errorCounter));
    nextTimeErrorCounterReset = curMillis + 30000; // 30 sec
  }

  if (stateCurr != STATE_OFF)
  {
    for (uint8_t i = 0; i < ERR_ENUM_COUNT; i++)
    {
      // set to fatal error if any temporary error counter reaches 10
      if (errorCounter[i] > 10)
      {
        setNextState(STATE_ERROR);
      }
    }
  }
}

// ---- RC (interrupt) --------------------------------------------------------------
// RC remote control helper
// convert ppm time (us) to percent (-100..+100)
// ppmtime: zero stick pos: 1500 uS
//          right stick pos: 2000 uS
//          left stick pos: 1000 uS
int Robot::rcValue(const int ppmTime)
{
  int value = (int)((double)(ppmTime - 1500) / 3.4);
  if (value > -5 && value < 5)
  {
    value = 0;  //  ensures exact zero position
  }
  return value;
}

// RC remote control driver
// 1. save time (uS) and RC channel states (HI/LO)
// 2. if new state is LO, evaluate ppm time for channel
void Robot::setRemotePPMState(const unsigned long timeMicros,
                              const boolean remoteSpeedState,
                              const boolean remoteSteerState,
                              const boolean remoteMowState,
                              const boolean remoteSwitchState)
{
  if (remoteSpeedState != remoteSpeedLastState)
  {
    remoteSpeedLastState = remoteSpeedState;
    if (remoteSpeedState)
    {
      remoteSpeedLastTime = timeMicros;
    }
    else
    {
      remoteSpeed = rcValue(timeMicros - remoteSpeedLastTime);
    }
  }

  if (remoteSteerState != remoteSteerLastState)
  {
    remoteSteerLastState = remoteSteerState;
    if (remoteSteerState)
    {
      remoteSteerLastTime = timeMicros;
    }
    else
    {
      remoteSteer = rcValue(timeMicros - remoteSteerLastTime);
    }
  }

  if (remoteMowState != remoteMowLastState)
  {
    remoteMowLastState = remoteMowState;
    if (remoteMowState)
    {
      remoteMowLastTime = timeMicros;
    }
    else
    {
      remoteMow = max(0, (rcValue(timeMicros - remoteMowLastTime) + 100) / 2);
    }
  }

  if (remoteSwitchState != remoteSwitchLastState)
  {
    remoteSwitchLastState = remoteSwitchState;
    if (remoteSwitchState)
    {
      remoteSwitchLastTime = timeMicros;
    }
    else
    {
      remoteSwitch = rcValue(timeMicros - remoteSwitchLastTime);
    }
  }
}

//motor is LEFT or RIGHT (0 or 1)
void Robot::setMotorPWM(int pwm,
                        const uint8_t motor,
                        const boolean useAccel)
{
  const unsigned long samplingTime = wheels.wheel[motor].motor.getSamplingTime();

  if (useAccel)
  {
    // http://phrogz.net/js/framerate-independent-low-pass-filter.html
    // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
    wheels.wheel[motor].motor.pwmCur +=
        samplingTime * (pwm - wheels.wheel[motor].motor.pwmCur) / wheels.wheel[motor].motor.acceleration;
  }

  // ----- driver protection (avoids driver explosion) ----------
  if ((pwm < 0 && wheels.wheel[motor].motor.pwmCur >= 0) || (pwm > 0 && wheels.wheel[motor].motor.pwmCur <= 0))
  { // changing direction should take place?
    if (wheels.wheel[motor].motor.getZeroTimeout() > 0)
    {
      pwm = wheels.wheel[motor].motor.pwmCur
          - wheels.wheel[motor].motor.pwmCur * (float)samplingTime / 200.0; // reduce speed
    }
  }

  if (odometer.use)
  {
    wheels.wheel[motor].motor.pwmCur = pwm;
    if (abs(wheels.wheel[motor].encoder.getWheelRpmCurr()) == 0)
    {
      wheels.wheel[motor].motor.setZeroTimeout(
          max(0, wheels.wheel[motor].motor.getZeroTimeout() - samplingTime));
    }
    else
    {
      wheels.wheel[motor].motor.setZeroTimeout(500);
    }
  }
  else
  {
    if (pwm == 0)
    {
      wheels.wheel[motor].motor.setZeroTimeout(
          max(0, wheels.wheel[motor].motor.getZeroTimeout() - samplingTime));
    }
    else
    {
      wheels.wheel[motor].motor.setZeroTimeout(700);
    }
  }

  wheels.wheel[motor].motor.setSpeed();
}

// sets wheel motor actuators
// - driver protection: delays polarity change until motor speed (EMV) is zero
//   http://wiki.ardumower.de/images/a/a5/Motor_polarity_switch_protection.png
// - optional: ensures that the motors (and gears) are not switched to 0%
//   (or 100%) too fast (motorAccel)
void Robot::setMotorPWMs(const int pwmLeft, int const pwmRight,
                         const boolean useAccel)
{
//  Console.print("setPwm: ");
//  Console.print(pwmLeft);
//  Console.print(" ");
//  Console.println(pwmRight);

  setMotorPWM(pwmLeft, LEFT, useAccel);
  setMotorPWM(pwmRight, RIGHT, useAccel);
}

// PID controller: roll robot to heading (requires IMU)
void Robot::wheelControl_imuRoll()
{
  if (!imu.isTimeToControl())
  {
    return;
  }

  // Control range corresponds to 80 % of maximum speed on the drive wheel
  Pid* pid_p = &imu.pid[Imu::ROLL];
  pid_p->setSetpoint(0);
  pid_p->setYMin(-80);
  pid_p->setYMax(+80);
  pid_p->setMaxOutput(80);
  float processValue = -distancePI(imu.getYaw(), imuRollHeading) / PI * 180.0;
  float y = pid_p->compute(processValue);

  if ((stateCurr == STATE_OFF ||
       stateCurr == STATE_STATION ||
       stateCurr == STATE_ERROR) &&
      (millis() - stateStartTime > 1000))
  {
    wheels.setSpeed(0);
    wheels.setSteer(0);
  }
  else
  {
    wheels.setSpeed(0);
    wheels.setSteer(round(y));
  }
}

// PID controller: track perimeter
void Robot::wheelControl_perimeter()
{
  if (!perimeters.isTimeToControl())
  {
    return;
  }

  unsigned long curMillis = millis();
  if ((curMillis > stateStartTime + 5000) &&
      (curMillis > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut))
  {
    // Robot is wheel-spinning while perimeter tracking => roll to get ground again

    if (trackingBlockInnerWheelWhilePerimeterStruggling)
    {
      // Block inner wheel and outer wheel at half speed
      wheels.setSpeed(25);
      wheels.setSteer(perimeterMag < 0 ? -25 : 25);
    }
    else
    {
      // Turn on the spot at half speed
      wheels.setSpeed(0);
      wheels.setSteer(perimeterMag < 0 ? -50 : 50);
    }

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut)
    {
      Console.println("Error: Tracking error");
      incErrorCounter(ERR_TRACKING);
      //setNextState(STATE_ERROR,0);
      setNextState(STATE_PERI_FIND);
    }
  }
  else
  {
    // Normal perimeter tracking

    Pid* pid_p = &perimeters.perimeter[Perimeter::LEFT].pid;
    pid_p->setSetpoint(0);
    pid_p->setYMin(-100);
    pid_p->setYMax(+100);
    pid_p->setMaxOutput(100);
    float y = pid_p->compute(sign(perimeterMag));

    wheels.setSpeed(speed);
    wheels.setSteer(round(y));
  }
}

// PID controller: correct direction during normal driving (requires IMU)
void Robot::wheelControl_imuDir()
{
  if (!imu.isTimeToControl())
  {
    return;
  }

  // Control range corresponds to the steer range
  Pid* pid_p = &imu.pid[Imu::DIR];
  pid_p->setSetpoint(0);
  pid_p->setYMin(-100);
  pid_p->setYMax(+100);
  pid_p->setMaxOutput(100);
  float processValue = -distancePI(imu.getYaw(), imuDriveHeading) / PI * 180.0;
  float y = pid_p->compute(processValue);

  if ((stateCurr == STATE_OFF ||
       stateCurr == STATE_STATION ||
       stateCurr == STATE_ERROR) &&
      (millis() > stateStartTime + 1000))
  {
    wheels.setSpeed(0);
    wheels.setSteer(0);
  }
  else
  {
    wheels.setSpeed(speed);
    wheels.setSteer(round(y));
  }
}

void Robot::wheelControl_normal()
{
  if (!wheels.wheel[Wheel::LEFT].motor.isTimeToControl())
  {
    return;
  }

  // Use wheel motor PID-regulator if we use odometer
  wheels.wheel[Wheel::LEFT].motor.regulate = odometer.use;

  int zeroSettleTime = wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  if (millis() < stateStartTime + zeroSettleTime)
  {
    // Start from zero speed and zero steer at state start
    wheels.setSpeed(0);
    wheels.setSteer(0);

    if (mowPatternCurr != MOW_LANES)
    {
      imuDriveHeading = imu.getYaw(); // set drive heading
    }
  }
  else
  {
    // No correction, set wheel values to same as robot values
    wheels.setSpeed(speed);
    wheels.setSteer(steer);
  }
}

// check for odometer sensor faults
void Robot::checkOdometerFaults()
{
  if (!odometer.use)
  {
    return;
  }

  bool err[2] = { false };
  unsigned long curMillis = millis();

  if (stateCurr == STATE_FORWARD && (curMillis - stateStartTime > 8000))
  {
    // just check if odometer sensors may not be working at all
    for (uint8_t i = LEFT; i <= RIGHT; i++)
    {
      if (wheels.wheel[i].motor.getPwmCur() > 100 &&
          abs(wheels.wheel[i].encoder.getWheelRpmCurr()) == 0)
      {
        err[i] = true;
      }
    }
  }

  if (stateCurr == STATE_ROLL && (curMillis - stateStartTime) > 1000)
  {
    // just check if odometer sensors may be turning in the wrong direction
    for (uint8_t i = LEFT; i <= RIGHT; i++)
    {
      int16_t pwmCur = wheels.wheel[i].motor.getPwmCur();
      int16_t wheelRpmCurr = wheels.wheel[i].encoder.getWheelRpmCurr();
      if ((pwmCur > +100 && wheelRpmCurr < -3) ||
          (pwmCur < -100 && wheelRpmCurr > +3))
      {
        err[i] = true;
      }
    }
  }

  if (err[LEFT])
  {
    Console.print("Left odometer error: PWM=");
    Console.print(wheels.wheel[Wheel::LEFT].motor.getPwmCur());
    Console.print("\tRPM=");
    Console.println(wheels.wheel[Wheel::LEFT].encoder.getWheelRpmCurr());
    incErrorCounter(ERR_ODOMETER_LEFT);
    setNextState(STATE_ERROR);
  }

  if (err[RIGHT])
  {
    Console.print("Right odometer error: PWM=");
    Console.print(wheels.wheel[Wheel::RIGHT].motor.getPwmCur());
    Console.print("\tRPM=");
    Console.println(wheels.wheel[Wheel::RIGHT].encoder.getWheelRpmCurr());
    incErrorCounter(ERR_ODOMETER_RIGHT);
    setNextState(STATE_ERROR);
  }
}

// Cutter motor speed controller (slowly adjusts output speed to set speed)
void Robot::cutterControl()
{
  if (cutter.motor.isTimeToControl())
  {
    if (cutter.isEnableOverriden())
    {
      cutter.disable();
    }
    cutter.control();
  }
}

void Robot::resetIdleTime()
{
  // battery switched off?
  if (idleTimeSec == BATTERY_SW_OFF)
  {
    Console.println(F("BATTERY switching ON again"));
    setActuator(ACT_BATTERY_SW, 1); // switch on battery again (if connected via USB)
  }
  idleTimeSec = 0;
}

void Robot::beep(const uint8_t numberOfBeeps, const bool shortbeep)
{
  uint16_t delayTime1;
  uint16_t delayTime2;

  if (shortbeep)
  {
    delayTime1 = 50;
    delayTime2 = 250;
  }
  else
  {
    delayTime1 = 500;
    delayTime2 = 500;
  }

  for (uint8_t i = 0; i < numberOfBeeps; i++)
  {
    setActuator(ACT_BUZZER, 4200);
    delay(delayTime1);
    setActuator(ACT_BUZZER, 0);
    delay(delayTime2);
  }
}

void Robot::setUserSwitches()
{
  setActuator(ACT_USER_SW1, userSwitch1);
  setActuator(ACT_USER_SW2, userSwitch2);
  setActuator(ACT_USER_SW3, userSwitch3);
}

void Robot::setDefaultTime()
{
  datetime.time.hour = 12;
  datetime.time.minute = 0;
  datetime.date.dayOfWeek = 0;
  datetime.date.day = 1;
  datetime.date.month = 1;
  datetime.date.year = 2016;

  timer[0].active = false;
  timer[0].daysOfWeek = B01111110;
  timer[0].startTime.hour = 9;
  timer[0].stopTime.hour = 11;
}

void Robot::setup()
{
  setDefaultTime();
  setMotorPWMs(0, 0);
  loadErrorCounters();
  loadUserSettings();
  if (!statsOverride)
  {
    loadRobotStats();
  }
  else
  {
    saveRobotStats();
  }
  setUserSwitches();

  if (!button.use)
  {
    // robot has no ON/OFF button => start immediately
    setNextState(STATE_FORWARD);
  }

  delay(2000);
  stateStartTime = millis();
  beep(1);

  Console.println(F("START"));
  Console.print(F("Ardumower "));
  Console.println(VERSION);
#ifdef USE_DEVELOPER_TEST
  Console.println("Warning: USE_DEVELOPER_TEST activated");
#endif
#ifdef USE_BARKER_CODE
  Console.println("Warning: USE_BARKER_CODE activated");
#endif
  Console.print(F("Config: "));
  Console.println(name);
  Console.println(F("press..."));
  Console.println(F("  d for menu"));
  Console.println(F("  v to change console output @"
                    "(sensor counters, sensor values, perimeter, off)"));
  Console.println(consoleModeNames[consoleMode]);
}

void Robot::printRemote()
{
  Console.print(F("RC "));
  Console.print(remoteSwitch);
  Console.print(", ");
  Console.print(remoteSteer);
  Console.print(", ");
  Console.print(remoteSpeed);
  Console.print(", ");
  Console.println(remoteMow);
}

void Robot::printOdometer()
{
  Console.print(F("ODO,"));
  Console.print(odometer.getX());
  Console.print(", ");
  Console.println(odometer.getY());
}

void Robot::printInfo_perimeter(Stream &s)
{
  perimeters.perimeter[Perimeter::LEFT].printInfo(s);
  Streamprint(s, "  in %-2d  cnt %-4d  on %-1d\r\n", perimeterInside,
              perimeterCounter,
              !perimeters.perimeter[Perimeter::LEFT].signalTimedOut());
}

void Robot::printInfo_odometer(Stream &s)
{
  Streamprint(s, "odo %4d %4d ",
              odometer.encoder.left_p->getCounter(),
              odometer.encoder.right_p->getCounter());
}

void Robot::printInfo_sensorValues(Stream &s)
{
  Streamprint(s, "sen %4d %4d %4d ",
              wheels.wheel[Wheel::LEFT].motor.getPowerMeas(),
              wheels.wheel[Wheel::RIGHT].motor.getPowerMeas(),
              cutter.motor.getPowerMeas());
  Streamprint(s, "bum %4d %4d ",
              bumpers.bumper[Bumpers::LEFT].isHit(),
              bumpers.bumper[Bumpers::RIGHT].isHit());
  Streamprint(s, "dro %4d %4d ",
              dropSensors.dropSensor[DropSensors::LEFT].isDetected(),
              dropSensors.dropSensor[DropSensors::RIGHT].isDetected());
  Streamprint(s, "son %4u %4u %4u ",
              sonars.sonar[Sonars::LEFT].getDistance_us(),
              sonars.sonar[Sonars::CENTER].getDistance_us(),
              sonars.sonar[Sonars::RIGHT].getDistance_us());
  Streamprint(s, "yaw %3d ", (int)(imu.getYawDeg()));
  Streamprint(s, "pit %3d ", (int)(imu.getPitchDeg()));
  Streamprint(s, "rol %3d ", (int)(imu.getRollDeg()));

  if (perimeters.use)
  {
    Streamprint(s, "per %3d ", perimeterInside);
  }

  if (lawnSensor.use)
  {
    Streamprint(s, "lawn %3d %3d ",
                (int)lawnSensor.getValue(LawnSensor::FRONT),
                (int)lawnSensor.getValue(LawnSensor::BACK));
  }
}

void Robot::printInfo_sensorCounters(Stream &s)
{
  Streamprint(s, "sen %4d %4d %4d ",
              wheels.wheel[Wheel::LEFT].motor.getOverloadCounter(),
              wheels.wheel[Wheel::RIGHT].motor.getOverloadCounter(),
              cutter.motor.getOverloadCounter());
  Streamprint(s, "bum %4d %4d ",
              bumpers.bumper[Bumpers::LEFT].getCounter(),
              bumpers.bumper[Bumpers::RIGHT].getCounter());
  Streamprint(s, "dro %4d %4d ",
              dropSensors.dropSensor[DropSensors::LEFT].getCounter(),
              dropSensors.dropSensor[DropSensors::RIGHT].getCounter());
  Streamprint(s, "son %3d ", sonars.getDistanceCounter());
  Streamprint(s, "yaw %3d ", (int)(imu.getYawDeg()));
  Streamprint(s, "pit %3d ", (int)(imu.getPitchDeg()));
  Streamprint(s, "rol %3d ", (int)(imu.getRollDeg()));
  //Streamprint(s, "per %3d ", perimeterLeft);

  if (perimeters.use)
  {
    Streamprint(s, "per %3d ", perimeterCounter);
  }

  if (lawnSensor.use)
  {
    Streamprint(s, "lawn %3d ", lawnSensor.getCounter());
  }

  if (gpsUse)
  {
    Streamprint(s, "gps %2d ", gps.satellites());
  }
}

void Robot::printInfo(Stream &s)
{
  if (consoleMode == CONSOLE_OFF)
  {
    return;
  }

  Streamprint(s, "t%4u ", (millis() - stateStartTime) / 1000);
  Streamprint(s, "l%5u ", loopsPerSec);
  //Streamprint(s, "r%4u ", freeRam());
  Streamprint(s, "v%1d ", consoleMode);
  Streamprint(s, "%8s ", stateNames[stateCurr]);

  if (consoleMode == CONSOLE_PERIMETER)
  {
    printInfo_perimeter(s);
  }
  else if (consoleMode == CONSOLE_IMU)
  {
    imu.printInfo(s);
  }
  else
  {
    if (odometer.use)
    {
      printInfo_odometer(s);
    }
    Streamprint(s, "spd %4d %4d %4d ",
                wheels.wheel[Wheel::LEFT].motor.rpmSet,
                wheels.wheel[Wheel::RIGHT].motor.rpmSet,
                cutter.motor.getRpmMeas());

    if (consoleMode == CONSOLE_SENSOR_VALUES)
    {
      printInfo_sensorValues(s);
    }
    else
    {
      printInfo_sensorCounters(s);
    }

    Streamprint(s, "bat %2d.%01d ",
                (int)batVoltage,
                (int)((batVoltage * 10) - ((int)batVoltage * 10)));
    Streamprint(s, "chg %2d.%01d %2d.%01d ",
                (int)chgVoltage,
                (int)((chgVoltage * 10) - ((int)chgVoltage * 10)),
                (int)chgCurrent,
                (int)((abs(chgCurrent) * 10) - ((int)abs(chgCurrent) * 10)));
    Streamprint(s, "imu %3d ", imu.getCallCounter());
    Streamprint(s, "adc %3d\r\n", ADCMan.getCapturedChannels());
    //Streamprint(s, "%s\r\n", name.c_str());
  }
}

void Robot::printMenu()
{
  Console.println();
  Console.println(F("1 = Test motors"));
  Console.println(F("2 = Test odometer"));
  Console.println(F("3 = Setup BT module config (quick baudscan/recommended)"));
  Console.println(F("4 = Setup BT module config (extensive baudscan)"));
  Console.println(F("5 = Calibrate IMU acc next side"));
  Console.println(F("6 = Calibrate IMU com start/stop"));
  Console.println(F("7 = Delete IMU calib"));
  Console.println(F("8 = ADC calib (perimeter sender, charger must be off)"));
  Console.println(F("9 = Save user settings"));
  Console.println(F("l = Load factory settings"));
  Console.println(F("r = Delete robot stats"));
  Console.println(F("x = Print settings"));
  Console.println(F("e = Delete all errors"));
  Console.println(F("0 = Exit"));
  Console.println();
}

void Robot::delayInfo(const int ms)
{
  unsigned long endtime = millis() + ms;
  while (millis() < endtime)
  {
    readSensors();
    printInfo(Console);
    delay(1000);
  }
}

void Robot::testOdometer()
{
  int leftPwm = wheels.wheel[Wheel::LEFT].motor.pwmMax / 2;
  int rightPwm = wheels.wheel[Wheel::RIGHT].motor.pwmMax / 2;

  wheels.wheel[Wheel::LEFT].motor.setPwmCur(leftPwm);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(rightPwm);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);

  int lastLeft = 0;
  int lastRight = 0;
  for (;;)
  {
    resetIdleTime();
    int odoCountLeft = odometer.encoder.left_p->getCounter();
    int odoCountRight = odometer.encoder.right_p->getCounter();
    if (odoCountLeft != lastLeft || odoCountRight != lastRight)
    {
      Console.print(F("Press 'f' forward, 'r' reverse, 'z' reset  "));
      Console.print(F("left="));
      Console.print(odoCountLeft);
      Console.print(F("  right="));
      Console.println(odoCountRight);
      lastLeft = odoCountLeft;
      lastRight = odoCountRight;
    }
    delay(100);
    if (Console.available() > 0)
    {
      char ch;
      ch = (char) Console.read();
      if (ch == '0')
      {
        break;
      }

      if (ch == 'f')
      {
        wheels.wheel[Wheel::LEFT].motor.setPwmCur(leftPwm);
        wheels.wheel[Wheel::RIGHT].motor.setPwmCur(rightPwm);
        setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
                     wheels.wheel[Wheel::RIGHT].motor.pwmCur);
      }

      if (ch == 'r')
      {
        wheels.wheel[Wheel::LEFT].motor.setPwmCur(-leftPwm);
        wheels.wheel[Wheel::RIGHT].motor.setPwmCur(-rightPwm);
        setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
                     wheels.wheel[Wheel::RIGHT].motor.pwmCur);
      }

      if (ch == 'z')
      {
        odometer.encoder.left_p->clearCounter();
        odometer.encoder.right_p->clearCounter();
      }
    }
  }

  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);
}

void Robot::testMotors()
{
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(0, 0);

  Console.println(F("testing left motor (forward) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(wheels.wheel[Wheel::LEFT].motor.pwmMax);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);

  Console.println(F("testing left motor (reverse) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(-wheels.wheel[Wheel::LEFT].motor.pwmMax);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);

  Console.println(F("testing right motor (forward) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = wheels.wheel[Wheel::LEFT].motor
      .pwmMax;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);

  Console.println(F("testing right motor (reverse) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(-wheels.wheel[Wheel::LEFT].motor.pwmMax);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.setPwmCur(0);
  wheels.wheel[Wheel::RIGHT].motor.setPwmCur(0);
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur);
}

void Robot::menu()
{
  printMenu();
  for (;;)
  {
    resetIdleTime();
    imu.update();
    if (Console.available() > 0)
    {
      char ch = (char)Console.read();
      switch (ch)
      {
        case '0':
          return;

        case '1':
          testMotors();
          break;

        case '2':
          testOdometer();
          break;

        case '3':
          configureBluetooth(true);
          break;

        case '4':
          configureBluetooth(false);
          break;

        case '5':
          imu.calibrateAccelerometerNextAxis();
          break;

        case '6':
          imu.calibrateMagnetometerStartStop();
          break;

        case '7':
          imu.deleteCalibrationData();
          break;

        case '8':
          ADCMan.calibrate();
          break;

        case '9':
          saveUserSettings();
          break;

        case 'l':
          printSettingSerial();
          deleteUserSettings();
          break;

        case 'r':
          printSettingSerial();
          deleteRobotStats();
          break;

        case 'x':
          printSettingSerial();
          Console.println(F("DONE"));
          break;

        case 'e':
          resetErrorCounters();
          setNextState(STATE_OFF);
          Console.println(F("ALL ERRORS ARE DELETED"));
          break;

        default:
          continue;
      }
      printMenu();
    }
    delay(10);
  }
}

void Robot::readSerial()
{
  // serial input
  if (Console.available() > 0)
  {
    char ch = (char)Console.read();
    resetIdleTime();
    switch (ch)
    {
      case 'd': // menu
        menu();
        break;

      case 'v': // change console mode
        consoleMode = (consoleMode + 1) % CONSOLE_END;
        Console.println(consoleModeNames[consoleMode]);
        break;

      case 'h': // drive home
        setNextState(STATE_PERI_FIND);
        break;

      case 't': // track perimeter
        setNextState(STATE_PERI_TRACK);
        break;

      case 'l': // simulate left bumper
        bumpers.bumper[Bumpers::LEFT].simHit();
        break;

      case 'r': // simulate right bumper
        bumpers.bumper[Bumpers::RIGHT].simHit();
        break;

      case 'j': // simulate left drop
        dropSensors.dropSensor[DropSensors::LEFT].simDetected();
        break;

      case 'k': // simulate right drop
        dropSensors.dropSensor[DropSensors::RIGHT].simDetected();
        break;

      case 's': // simulate lawn sensor
        lawnSensor.simDetected();
        break;

      case 'm': // toggle mower motor
        if (stateCurr == STATE_OFF || stateCurr == STATE_MANUAL)
        {
          cutter.setEnableOverriden(false);
        }
        else
        {
          cutter.toggleEnableOverriden();
        }
        cutter.toggleEnabled();
        break;

      case 'c': // simulate in station
        setNextState(STATE_STATION);
        break;

      case 'a': // simulate in station charging
        setNextState(STATE_STATION_CHARGING);
        break;

      case '+': // rotate 90 degrees clockwise (IMU)
        setNextState(STATE_ROLL_WAIT);
        imuRollHeading = scalePI(imuRollHeading + PI / 2);
        break;

      case '-': // rotate 90 degrees anti-clockwise (IMU)
        setNextState(STATE_ROLL_WAIT);
        imuRollHeading = scalePI(imuRollHeading - PI / 2);
        break;

      case 'i': // toggle imu.use
        imu.use = !imu.use;
        break;

      case '3': // activate model RC
        setNextState(STATE_REMOTE);
        break;

      case '0': // turn OFF
        setNextState(STATE_OFF);
        break;

      case '1': // Automode
        cutter.enable();
        //motorMowModulate = false;
        setNextState(STATE_FORWARD);
        break;
    }
  }
}

void Robot::checkButton()
{
  bool buttonPressed = false;
  if (button.isTimeToCheck())
  {
    buttonPressed = button.isPressed();
  }

  if ((!buttonPressed && button.getCounter() > 0) ||
      (buttonPressed && button.isTimeToRun()))
  {
    if (buttonPressed)
    {
      Console.println(F("Button is pressed"));
      beep(1);
      button.incCounter();
      resetIdleTime();
    }
    else
    {
      if ((stateCurr != STATE_OFF || stateCurr == STATE_ERROR) &&
          stateCurr != STATE_STATION)
      {
        setNextState(STATE_OFF);
      }
      else
      {
        switch (button.getCounter())
        {
          case 1:
            // start normal with random mowing
            cutter.enable();
            mowPatternCurr = MOW_RANDOM;
            setNextState(STATE_FORWARD);
            break;

          case 2:
            // start normal with bidir mowing
            cutter.enable();
            mowPatternCurr = MOW_BIDIR;
            setNextState(STATE_FORWARD);
            break;

          case 3:
            // start remote control mode
            setNextState(STATE_REMOTE);
            break;

          case 4:
            // start normal without perimeter
            perimeters.use = false;
            setNextState(STATE_FORWARD);
            break;

          case 5:
            // drive home
            setNextState(STATE_PERI_FIND);
            break;

          case 6:
            // track perimeter
            setNextState(STATE_PERI_TRACK);
            break;

          case 7:
            // start normal with lanes mowing
            cutter.enable();
            mowPatternCurr = MOW_LANES;
            setNextState(STATE_FORWARD);
            break;
        }
      }

      button.clearCounter();
    }
  }
}

float Robot::getBatteryVoltage()
{
  if (batVoltage > 8)
  {
    // Use measured battery voltage
    return batVoltage;
  }
  else
  {
    // Use reference voltage for fully battery in absence of battery voltage
    // measurement
    return batFull;
  }
}

void Robot::readSensors()
{
// NOTE: This function should only read in sensors into variables.
//       It should NOT change any state!

  unsigned long curMillis = millis();
  if (cutter.motor.isTimeToReadCurrent())
  {
    wheels.wheel[Wheel::LEFT].motor.readCurrent();
    wheels.wheel[Wheel::RIGHT].motor.readCurrent();
    cutter.motor.readCurrent();

    // Conversion to power in Watts
    float batV = getBatteryVoltage();
    wheels.wheel[Wheel::RIGHT].motor.calcPower(batV);
    wheels.wheel[Wheel::LEFT].motor.calcPower(batV);
    cutter.motor.calcPower(batV);

    unsigned long timeSinceLast;
    if (cutter.motor.isTimeForRpmMeas(&timeSinceLast))
    {
      if ((cutter.motor.getRpmMeas() == 0) && (cutter.motor.getRpmCounter() != 0))
      {
        // rpm may be updated via interrupt
        cutter.motor.setRpmMeas(
            (int)(((double)cutter.motor.getRpmCounter() / (double)timeSinceLast)
                * 60000.0));
        cutter.motor.clearRpmCounter();
      }
      if (!ADCMan.calibrationDataAvail())
      {
        Console.println(F("Error: Missing ADC calibration data"));
        incErrorCounter(ERR_ADC_CALIB);
        setNextState(STATE_ERROR);
      }
    }
  }

  if (perimeters.use && curMillis >= nextTimePerimeter)
  {
    nextTimePerimeter = curMillis + 50;
    perimeterMag = perimeters.perimeter[Perimeter::LEFT].calcMagnitude();
    bool inside = perimeters.perimeter[Perimeter::LEFT].isInside();
    if (inside != perimeterInside)
    {
      perimeterCounter++;
      perimeterLastTransitionTime = millis();
      perimeterInside = inside;
    }

    if (perimeterInside)
    {
      setActuator(ACT_LED, HIGH);
    }
    else
    {
      setActuator(ACT_LED, LOW);
    }

    if (!perimeterInside && perimeterTriggerTime == 0)
    {
      // set perimeter trigger time
      // far away from perimeter?
      if (curMillis > stateStartTime + 2000)
      {
        perimeterTriggerTime = curMillis + perimeterTriggerTimeout;
      }
      else
      {
        perimeterTriggerTime = curMillis;
      }
    }

    if (perimeters.perimeter[Perimeter::LEFT].signalTimedOut())
    {
      if (stateCurr != STATE_OFF &&
          stateCurr != STATE_MANUAL &&
          stateCurr != STATE_STATION &&
          stateCurr != STATE_STATION_CHARGING &&
          stateCurr != STATE_STATION_CHECK &&
          stateCurr != STATE_STATION_REV &&
          stateCurr != STATE_STATION_ROLL &&
          stateCurr != STATE_STATION_FORW &&
          stateCurr != STATE_REMOTE &&
          stateCurr != STATE_PERI_OUT_FORW &&
          stateCurr != STATE_PERI_OUT_REV &&
          stateCurr != STATE_PERI_OUT_ROLL)
      {
        Console.println("Error: Perimeter too far away");
        incErrorCounter(ERR_PERIMETER_TIMEOUT);
        setNextState(STATE_ERROR);
      }
    }
  }

  if (lawnSensor.isTimeToRead())
  {
    lawnSensor.read();
  }

  if (lawnSensor.isTimeToCheck())
  {
    lawnSensor.check();
  }

  if (sonars.isTimeToRun())
  {
    sonars.ping();
  }

  if (bumpers.isTimeToRun())
  {
    bumpers.check();
  }

  if (dropSensors.isTimeToRun())
  {
    dropSensors.check();
  }

  if (curMillis >= nextTimeRTC)
  {
    nextTimeRTC = curMillis + 60000;
    readSensor(SEN_RTC);       // read RTC
    Console.print(F("RTC date received: "));
    Console.println(date2str(datetime.date));
  }

  if (imu.isTimeToRun())
  {
    if (imu.getErrorCounter() > 0)
    {
      incErrorCounter(ERR_IMU_COMM);
      Console.println(F("IMU comm error"));
    }

    if (!imu.isCalibrationAvailable())
    {
      Console.println(F("Error: Missing IMU calibration data"));
      incErrorCounter(ERR_IMU_CALIB);
      setNextState(STATE_ERROR);
    }
  }

  if (curMillis >= nextTimeBattery)
  {
    nextTimeBattery = curMillis + 100;
    // read battery
    if (abs(chgCurrent) > 0.04 && chgVoltage > 5)
    {
      // charging
      batCapacity += (chgCurrent / 36.0); //TODO: What is 36.0?
    }

    // convert to double
    batADC = readSensor(SEN_BAT_VOLTAGE);
    double batvolt = (double)batADC * batFactor / 10; // / 10 due to arduremote bug, can be removed after fixing
    int chgADC = readSensor(SEN_CHG_VOLTAGE);
    double chgvolt = (double)chgADC * batChgFactor / 10; // / 10 due to arduremote bug, can be removed after fixing
    double current = (double)readSensor(SEN_CHG_CURRENT);

    // low-pass filter
    double accel = 0.01;

    if (abs(batVoltage - batvolt) > 5)
    {
      batVoltage = batvolt;
    }
    else
    {
      batVoltage = (1.0 - accel) * batVoltage + accel * batvolt;
    }

    if (abs(chgVoltage - chgvolt) > 5)
    {
      chgVoltage = chgvolt;
    }
    else
    {
      chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
    }

    //Deaktiviert für Ladestromsensor berechnung
//    if (abs(chgCurrent - current) > 0.4)
//    {
//      chgCurrent = current;
//    }
//    else
//    {
//      chgCurrent = (1.0 - accel) * chgCurrent + accel * current;
//    }

    // Sensor Wert Ausgabe auf Seriellen Monitor oder HandyApp
    if (chgSelection == 0)
    {
      chgCurrent = current;
    }

    // Berechnung für Ladestromsensor ACS712 5A
    if (chgSelection == 1)
    {
      float chgAMP = current;              //Sensorwert übergabe vom Ladestrompin
      float vcc = 3.30 / chgSenseZero * 1023.0; // Versorgungsspannung ermitteln!  chgSenseZero=511  ->Die Genauigkeit kann erhöt werden wenn der 3.3V Pin an ein Analogen Pin eingelesen wird. Dann ist vcc = (float) 3.30 / analogRead(X) * 1023.0;
      float asensor = chgAMP * vcc / 1023.0;              // Messwert auslesen
      asensor = asensor - (vcc / chgNull); // Nulldurchgang (vcc/2) abziehen
      chgSense = chgSense - ((5.00 - vcc) * chgFactor); // Korrekturfactor für Vcc!  chgFactor=39
      float amp = asensor / chgSense * 1000;               // Ampere berechnen
      if (chgChange == 1)
      {
        amp = -amp;                 //Lade Strom Messwertumkehr von - nach +
      }
      if (amp < 0.0)
      {
        chgCurrent = 0.0;
      }
      else
      {
        chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 anssonsten messwertau8sgabe in Ampere)
      }
    }

    // Berechnung für Ladestromsensor INA169 board
    if (chgSelection == 2)
    {
      float chgAMP = current;
      float asensor = (chgAMP * 5) / 1023; // umrechnen von messwert in Spannung (5V Reference)
      float amp = asensor / (10 * 0.1); // Ampere berechnen RL = 10k    Is = (Vout x 1k) / (RS x RL)
      if (amp < 0.0)
      {
        chgCurrent = 0.0;
      }
      else
      {
        chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 ansonsten Messwertaußsgabe in Ampere)
      }
    }
  }

  if (rainSensor.isTimeToRun())
  {
    rainSensor.check();
  }
}

void Robot::setDefaults()
{
  speed = 0;
  steer = 0;
  cutter.disable();
}

// set state machine new state
// http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
// called *ONCE* to set to a *NEW* state
void Robot::setNextState(byte stateNew, bool dir)
{
  if (stateNew == stateCurr)
  {
    return;
  }

  unsigned long curMillis = millis();

  // state correction
  if (stateCurr == STATE_PERI_FIND ||
      stateCurr == STATE_PERI_TRACK)
  {
    if (stateNew == STATE_ROLL)
    {
      stateNew = STATE_PERI_ROLL;
    }

    if (stateNew == STATE_REVERSE)
    {
      stateNew = STATE_PERI_REV;
    }
  }

  if (stateNew == STATE_FORWARD)
  {
    if (stateCurr == STATE_STATION_REV ||
        stateCurr == STATE_STATION_ROLL ||
        stateCurr == STATE_STATION_CHECK)
    {
      return;
    }

    if (stateCurr == STATE_STATION ||
        stateCurr == STATE_STATION_CHARGING)
    {
      stateNew = STATE_STATION_CHECK;
      setActuator(ACT_CHGRELAY, 0);
      cutter.disable();
    }
  }

  // evaluate new state
  stateNext = stateNew;
  rollDir = dir;
  int zeroSettleTime = wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;

  if (stateNew == STATE_STATION_REV)
  {
    speed = -100;
    steer = 0;
    stateEndTime = curMillis + stationRevTime + zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_ROLL)
  {
    speed = 0;
    steer = +100;
    stateEndTime = curMillis + stationRollTime + zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_FORW)
  {
    speed = +100;
    steer = 0;
    cutter.enable();
    stateEndTime = curMillis + stationForwTime + zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_CHECK)
  {
    speed = -25;
    steer = 0;
    stateEndTime = curMillis + stationCheckTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_ROLL)
  {
    speed = 0;
    steer = (dir == LEFT) ? -25: +25;
    stateEndTime = curMillis + perimeterTrackRollTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_REV)
  {
    speed = -25;
    steer = 0;
    stateEndTime = curMillis + perimeterTrackRevTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_OUT_FORW)
  {
    speed = +100;
    steer = 0;
    stateEndTime = curMillis + perimeterOutRevTime + zeroSettleTime + 1000;
  }
  else if (stateNew == STATE_PERI_OUT_REV)
  {
    speed = -75;
    steer = 0;
    stateEndTime = curMillis + perimeterOutRevTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_OUT_ROLL)
  {
    speed = 0;
    steer = (dir == LEFT) ? -75: +75;
    stateEndTime = curMillis + random(perimeterOutRollTimeMin, perimeterOutRollTimeMax) + zeroSettleTime;
  }
  else if (stateNew == STATE_FORWARD)
  {
    speed = 100;
    steer = 0;
    statsMowTimeTotalStart = true;
  }
  else if (stateNew == STATE_REVERSE)
  {
    speed = -75;
    steer = 0;
    stateEndTime = curMillis + wheels.reverseTime + zeroSettleTime;
  }
  else if (stateNew == STATE_ROLL)
  {
    imuDriveHeading = scalePI(imuDriveHeading + PI); // Toggle heading 180 degree (IMU)
    if (imuRollDir == LEFT)
    {
      imuRollHeading = scalePI(imuDriveHeading - PI / 20);
      imuRollDir = RIGHT;
    }
    else
    {
      imuRollHeading = scalePI(imuDriveHeading + PI / 20);
      imuRollDir = LEFT;
    }
    speed = 0;
    steer = (dir == LEFT) ? -75: +75;
    stateEndTime = curMillis + random(wheels.rollTimeMin, wheels.rollTimeMax) + zeroSettleTime;
  }
  else if (stateNew == STATE_REMOTE)
  {
    cutter.enable();
    //motorMowModulate = false;
  }
  else if (stateNew == STATE_STATION)
  {
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();
    statsMowTimeTotalStart = false;  // stop stats mowTime counter
    saveRobotStats();
  }
  else if (stateNew == STATE_STATION_CHARGING)
  {
    setActuator(ACT_CHGRELAY, 1);
    setDefaults();
  }
  else if (stateNew == STATE_OFF)
  {
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();
    statsMowTimeTotalStart = false; // stop stats mowTime counter
    saveRobotStats();
  }
  else if (stateNew == STATE_ERROR)
  {
    setDefaults();
    setActuator(ACT_CHGRELAY, 0);
    statsMowTimeTotalStart = false;
    //saveRobotStats();
  }
  else if (stateNew == STATE_PERI_FIND)
  {
    // find perimeter  => drive half speed
    speed = 50;
    steer = 0;
    //motorMowEnable = false;     // FIXME: should be an option?
  }
  else if (stateNew == STATE_PERI_TRACK)
  {
    //motorMowEnable = false;     // FIXME: should be an option?
    speed = 50;
    steer = 0;
    setActuator(ACT_CHGRELAY, 0);
    //beep(6);
  }
  else if (stateNew != STATE_REMOTE)
  {
    // TODO: This feels dangerous!!!
    cutter.motor.setPwmSet(cutter.motor.pwmMax);
  }

  sonars.obstacleTimeout = 0;
  // state has changed
  stateStartTime = curMillis;
  stateLast = stateCurr;
  stateCurr = stateNext;
  perimeterTriggerTime = 0;
  printInfo(Console);
}

void Robot::checkBattery()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeCheckBattery)
  {
    return;
  }
  nextTimeCheckBattery = curMillis + 1000;

  if (batMonitor)
  {
    if (batVoltage < batSwitchOffIfBelow &&
        stateCurr != STATE_ERROR &&
        stateCurr != STATE_OFF &&
        stateCurr != STATE_STATION &&
        stateCurr != STATE_STATION_CHARGING)
    {
      Console.println(F("Triggered batSwitchOffIfBelow"));
      incErrorCounter(ERR_BATTERY);
      beep(2, true);
      setNextState(STATE_OFF);
    }
    else if (batVoltage < batGoHomeIfBelow &&
             stateCurr != STATE_OFF &&
             stateCurr != STATE_MANUAL &&
             stateCurr != STATE_STATION &&
             stateCurr != STATE_STATION_CHARGING &&
             stateCurr != STATE_REMOTE &&
             stateCurr != STATE_ERROR &&
             stateCurr != STATE_PERI_TRACK &&
             perimeters.use)
    {
      Console.println(F("Triggered batGoHomeIfBelow"));
      beep(2, true);
      setNextState(STATE_PERI_FIND);
    }
  }

  // Check if mower is idle and battery can be switched off
  if (stateCurr == STATE_OFF ||
      stateCurr == STATE_ERROR)
  {
    if (idleTimeSec != BATTERY_SW_OFF)
    {
      // battery already switched off?
      idleTimeSec++; // add one second idle time
      if (idleTimeSec > batSwitchOffIfIdle * 60)
      {
        Console.println(F("Triggered batSwitchOffIfIdle"));
        beep(1, true);
        saveErrorCounters();
        saveRobotStats();
        idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
        Console.println(F("BATTERY switching OFF"));
        setActuator(ACT_BATTERY_SW, 0);  // switch off battery
      }
    }
  }
  else
  {
    resetIdleTime();
  }
}

void Robot::receiveGPSTime()
{
  if (!gpsUse)
  {
    return;
  }

  unsigned long chars = 0;
  unsigned short good_sentences = 0;
  unsigned short failed_cs = 0;
  gps.stats(&chars, &good_sentences, &failed_cs);

  if (good_sentences == 0)
  {
    // no GPS sentences received so far
    Console.println(F("GPS communication error!"));
    incErrorCounter(ERR_GPS_COMM);
  }
  Console.print(F("GPS sentences: "));
  Console.println(good_sentences);
  Console.print(F("GPS satellites in view: "));
  Console.println(gps.satellites());

  if (gps.satellites() == Gps::GPS_INVALID_SATELLITES)
  {
    // no GPS satellites received so far
    incErrorCounter(ERR_GPS_DATA);
  }

  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths,
                     &age);
  if (age != Gps::GPS_INVALID_AGE)
  {
    Console.print(F("GPS date received: "));
    Console.println(date2str(datetime.date));
    datetime.date.dayOfWeek = getDayOfWeek(month, day, year, 1);
    datetime.date.day = day;
    datetime.date.month = month;
    datetime.date.year = year;
    datetime.time.hour = hour;
    datetime.time.minute = minute;

    if (timerUse)
    {
      // set RTC using GPS data
      Console.print(F("RTC date set: "));
      Console.println(date2str(datetime.date));
      setActuator(ACT_RTC, 0);
    }
  }
}

void Robot::checkRobotStats_mowTime()
{
  statsMowTimeHoursTotal = float(statsMowTimeMinutesTotal) / 60;
  if (statsMowTimeTotalStart)
  {
    statsMowTimeMinutesTripCounter++;
    statsMowTimeMinutesTrip = statsMowTimeMinutesTripCounter;
    statsMowTimeMinutesTotal++;
  }
  else
  {
    statsMowTimeMinutesTripCounter = 0;
  }
}

void Robot::checkRobotStats_battery()
{
  // Cunt only if mower has charged more than 60 seconds.
  if (stateCurr == STATE_STATION_CHARGING && stateTime >= 60000)
  {
    statsBatteryChargingCounter++; // temporary counter
    if (statsBatteryChargingCounter == 1)
    {
      statsBatteryChargingCounterTotal++;
    }
    statsBatteryChargingCapacityTrip = batCapacity;
    // Sum up only the difference between actual batCapacity and last batCapacity
    statsBatteryChargingCapacityTotal += (batCapacity - lastTimeBatCapacity);
    lastTimeBatCapacity = batCapacity;
  }
  else
  {
    // Reset values to 0 when mower is not charging
    statsBatteryChargingCounter = 0;
    batCapacity = 0;
  }

  if (isnan(statsBatteryChargingCapacityTrip))
  {
    statsBatteryChargingCapacityTrip = 0;
  }

  if (isnan(statsBatteryChargingCapacityTotal))
  {
    statsBatteryChargingCapacityTotal = 0;
  }

  if (statsBatteryChargingCapacityTotal <= 0 ||
      statsBatteryChargingCounterTotal == 0)
  {
    statsBatteryChargingCapacityAverage = 0; // Avoid divide by zero
  }
  else
  {
    statsBatteryChargingCapacityAverage =
        statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;
  }
}

void Robot::checkRobotStats()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeRobotStats)
  {
    return;
  }
  nextTimeRobotStats = curMillis + 60000;

  checkRobotStats_mowTime();
  checkRobotStats_battery();
}

void Robot::checkTimer()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeTimer)
  {
    return;
  }
  nextTimeTimer = curMillis + 60000;

  // TODO: Should these inits only be done once?
  // Initialize the pseudo-random number generator for c++ rand()
  srand(time2minutes(datetime.time));

  // Initialize the pseudo-random number generator for arduino random()
  randomSeed(time2minutes(datetime.time));

  receiveGPSTime();

  if (!timerUse)
  {
    return;
  }

  boolean stopTimerTriggered = true;
  for (uint8_t i = 0; i < MAX_TIMERS; i++)
  {
    if (!timer[i].active)
    {
      continue;
    }

    if (timer[i].daysOfWeek & (1 << datetime.date.dayOfWeek))
    {
      // Matched dayOfWeek
      int startMinute = time2minutes(timer[i].startTime);
      int stopMinute = time2minutes(timer[i].stopTime);
      int currMinute = time2minutes(datetime.time);

      if (currMinute >= startMinute && currMinute < stopMinute)
      {
        // Start timer triggered
        stopTimerTriggered = false;
        if (stateCurr == STATE_STATION || stateCurr == STATE_OFF)
        {
          Console.println(F("Timer start triggered"));
          cutter.enable();
          setNextState(STATE_FORWARD);
        }
      }
    }

    if (stopTimerTriggered && stateCurr == STATE_FORWARD)
    {
      Console.println(F("Timer stop triggered"));
      if (perimeters.use)
      {
        setNextState(STATE_PERI_FIND);
      }
      else
      {
        setNextState(STATE_OFF);
      }
    }
  }
}

void Robot::reverseOrChangeDirection(const byte aRollDir)
{
  if (mowPatternCurr == MOW_BIDIR)
  {
    if (stateCurr == STATE_FORWARD)
    {
      setNextState(STATE_REVERSE, RIGHT);
    }
    else if (stateCurr == STATE_REVERSE)
    {
      setNextState(STATE_FORWARD, LEFT);
    }
  }
  else
  {
    setNextState(STATE_REVERSE, aRollDir);
  }
}

void Robot::checkCutterMotorPower()
{
  if (cutter.motor.isOverpowered())
  {
    cutter.motor.incOverloadCounter();
  }
  else
  {
    errorCounterMax[ERR_CUTTER_SENSE] = 0;
    cutter.motor.clearOverloadCounter();

    // Wait some time before switching on again
    if (cutter.motor.isWaitAfterStuckEnd())
    {
      errorCounter[ERR_CUTTER_SENSE] = 0;
      cutter.enable();
    }
  }

  // Ignore motor cutter power overload for 3 seconds
  if (cutter.motor.getOverloadCounter() >= 30)
  {
    cutter.disable();
    Console.println("Error: Motor cutter current");
    incErrorCounter(ERR_CUTTER_SENSE);
    cutter.motor.gotStuck();
  }
}

void Robot::checkWheelMotorPower(Wheel::wheelE side)
{
  bool hasPassedPowerIgnoreTime =
      millis() > (stateStartTime + wheels.wheel[side].motor.powerIgnoreTime);

  if (wheels.wheel[side].motor.isOverpowered() && hasPassedPowerIgnoreTime)
  {
    //beep(1);
    wheels.wheel[side].motor.incOverloadCounter();
    setMotorPWMs(0, 0);
    // TODO: wheels.stop();

    if (stateCurr == STATE_FORWARD ||
        stateCurr == STATE_PERI_FIND ||
        stateCurr == STATE_PERI_TRACK)
    {
      reverseOrChangeDirection(RIGHT);
    }
    else if (stateCurr == STATE_REVERSE)
    {
      setNextState(STATE_ROLL, (bool)!side);
    }
    else if (stateCurr == STATE_ROLL)
    {
      setNextState(STATE_FORWARD);
    }
  }
}

void Robot::checkMotorPower()
{
  if (cutter.motor.isTimeToCheckPower()) // Any motor could be the trigger
  {
    checkCutterMotorPower();
    checkWheelMotorPower(Wheel::LEFT);
    checkWheelMotorPower(Wheel::RIGHT);
  }
}

// check bumpers
void Robot::checkBumpers()
{
  if (mowPatternCurr == MOW_BIDIR && millis() < (stateStartTime + 4000))
  {
    return;
  }

  if (bumpers.bumper[Bumpers::LEFT].isHit())
  {
    reverseOrChangeDirection(RIGHT);
  }
  if (bumpers.bumper[Bumpers::RIGHT].isHit())
  {
    reverseOrChangeDirection(LEFT);
  }
}

// check drop
void Robot::checkDrop()
{
  unsigned long curMillis = millis();
  if (mowPatternCurr == MOW_BIDIR && curMillis < (stateStartTime + 4000))
  {
    return;
  }

  if (dropSensors.dropSensor[DropSensors::LEFT].isDetected())
  {
    reverseOrChangeDirection(RIGHT);
  }
  if (dropSensors.dropSensor[DropSensors::RIGHT].isDetected())
  {
    reverseOrChangeDirection(LEFT);
  }
}

// check bumpers while tracking perimeter
// Truth table
//   LRT|N
//   000|-
//   001|-
//   010|L
//   011|R
//   100|R
//   101|R
//   110|R
//   111|R
void Robot::checkBumpersPerimeter()
{
  if (bumpers.isAnyHit())
  {
    if (bumpers.bumper[Bumpers::LEFT].isHit() ||
        stateCurr == STATE_PERI_TRACK)
    {
      setNextState(STATE_PERI_REV, RIGHT);
    }
    else
    {
      setNextState(STATE_PERI_REV, LEFT);
    }
  }
}

// Check perimeter as a boundary
void Robot::checkPerimeterBoundary()
{
  unsigned long curMillis = millis();

  if (wheels.isTimeToRotationChange())
  {
    wheels.rotateDir = (Wheel::wheelE)!wheels.rotateDir; // Toggle rotation direction
  }

  if (mowPatternCurr == MOW_BIDIR)
  {
    if (curMillis < stateStartTime + 3000)
    {
      return;
    }
    if (!perimeterInside)
    {
      reverseOrChangeDirection(rand() % 2); // Random direction
    }
  }
  else
  {
    if (perimeterTriggerTime > 0 && curMillis >= perimeterTriggerTime)
    {
      perimeterTriggerTime = 0;
      if (stateCurr == STATE_FORWARD)
      {
        setNextState(STATE_PERI_OUT_REV, wheels.rotateDir);
      }
      else if (stateCurr == STATE_ROLL)
      {
        speed = 0;
        steer = 0;
        setNextState(STATE_PERI_OUT_FORW, wheels.rotateDir);
      }
    }
  }
}

// Check perimeter while finding it
void Robot::checkPerimeterFind()
{
  if (stateCurr == STATE_PERI_FIND)
  {
    if (perimeters.perimeter[Perimeter::LEFT].isInside())
    {
      if (wheels.wheel[Wheel::LEFT].motor.rpmSet != wheels.wheel[Wheel::RIGHT].motor.rpmSet)
      {
        // We just made an 'outside to inside' rotation, now track
        setNextState(STATE_PERI_TRACK);
      }
    }
    else
    {
      // We are outside, now roll to get inside
      speed = 0;
      steer = +50;
    }
  }
}

// check lawn
void Robot::checkLawn()
{
  if (lawnSensor.use)
  {
    if (lawnSensor.isDetected() && millis() > stateStartTime + 3000)
    {
      reverseOrChangeDirection(!rollDir); // Toggle roll direction
    }
    else
    {
      lawnSensor.clearDetected();
    }
  }
}

void Robot::checkRain()
{
  if (rainSensor.use && rainSensor.isRaining())
  {
    Console.println(F("RAIN"));
    if (perimeters.use)
    {
      setNextState(STATE_PERI_FIND);
    }
    else
    {
      setNextState(STATE_OFF);
    }
  }
}

// check sonar
void Robot::checkSonar()
{
  if (!sonars.use || !sonars.isTimeToCheck())
  {
    return;
  }

  unsigned long curMillis = millis();
  if (mowPatternCurr == MOW_BIDIR && curMillis < (stateStartTime + 4000))
  {
    return;
  }

  // slow down motor wheel speed near obstacles
  if (stateCurr == STATE_FORWARD ||
      (mowPatternCurr == MOW_BIDIR && stateCurr == STATE_REVERSE))
  {
    if (sonars.obstacleTimeout == 0)
    {
      bool isClose = false;
      for (uint8_t i = 0; i < Sonars::END; i++)
      {
        if (sonars.sonar[i].getDistance_us() < (sonars.triggerBelow * 2))
        {
          isClose = true;
        }
      }
      if (isClose)
      {
        sonars.tempDistanceCounter++;
        if (sonars.tempDistanceCounter >= 5)
        {
          // Console.println("sonar slow down");

          // Avoid slowing down to 0, stop at -1 or +1, allows speedup again
          int8_t tmpSpeed = speed / 2;
          if (tmpSpeed == 0)
          {
            speed = sign(speed);
          }
          else
          {
            speed = tmpSpeed;
          }
          sonars.obstacleTimeout = curMillis + 3000;
        }
      }
      else
      {
        sonars.tempDistanceCounter = 0;
      }
    }
    else if (sonars.obstacleTimeout != 0 && curMillis > sonars.obstacleTimeout)
    {
      //Console.println("no sonar");
      sonars.obstacleTimeout = 0;
      sonars.tempDistanceCounter = 0;
      speed = constrain(speed * 2, -100, 100);
    }
  }

  uint16_t distanceUs;

  distanceUs = sonars.sonar[Sonars::CENTER].getDistance_us();
  if (distanceUs < sonars.triggerBelow)
  {
    sonars.incDistanceCounter();
    reverseOrChangeDirection(!rollDir); // toggle roll dir
  }

  distanceUs = sonars.sonar[Sonars::RIGHT].getDistance_us();
  if (distanceUs < sonars.triggerBelow)
  {
    sonars.incDistanceCounter();
    reverseOrChangeDirection(LEFT);
  }

  distanceUs = sonars.sonar[Sonars::LEFT].getDistance_us();
  if (distanceUs < sonars.triggerBelow)
  {
    sonars.incDistanceCounter();
    reverseOrChangeDirection(RIGHT);
  }
}

// check IMU (tilt)
void Robot::checkTilt()
{
  unsigned long curMillis = millis();
  if (imu.use && curMillis >= nextTimeCheckTilt)
  {
    nextTimeCheckTilt = curMillis + 200; // 5Hz same as imu.nextTime

    if (stateCurr != STATE_OFF &&
        stateCurr != STATE_ERROR &&
        stateCurr != STATE_STATION)
    {
      int pitchAngle = (int)imu.getPitchDeg();
      int rollAngle = (int)imu.getRollDeg();
      if (abs(pitchAngle) > 40 || abs(rollAngle) > 40)
      {
        Console.println(F("Error: IMU tilt"));
        incErrorCounter(ERR_IMU_TILT);
        setNextState(STATE_ERROR);
      }
    }
  }
}

// Check if mower is stuck
// TODO: Take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStuck()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeCheckIfStuck)
  {
    return;
  }
  nextTimeCheckIfStuck = curMillis + 300;

  if (gpsUse && gps.hdop() < 500)
  {
    float gpsSpeed = gps.f_speed_kmph();
    if (gpsSpeedIgnoreTime >= wheels.reverseTime)
    {
      gpsSpeedIgnoreTime = wheels.reverseTime - 500;
    }
    // low-pass filter
    // double accel = 0.1;
    // float gpsSpeed = (1.0-accel) * gpsSpeed + accel * gpsSpeedRead;
    // Console.println(gpsSpeed);
    // Console.println(robotIsStuckedCounter);
    // Console.println(errorCounter[ERR_STUCK]);
    if (stateCurr != STATE_MANUAL &&
        stateCurr != STATE_REMOTE &&
        gpsSpeed < stuckIfGpsSpeedBelow &&
        odometer.encoder.left_p->getWheelRpmCurr() != 0 &&
        odometer.encoder.right_p->getWheelRpmCurr() != 0 &&
        curMillis > (stateStartTime + gpsSpeedIgnoreTime))
    {
      robotIsStuckCounter++;
    }
    else
    {
      // If mower gets unstuck, errorCounterMax is reset to zero and
      // cutter motor is re-enabled.
      robotIsStuckCounter = 0;    // resets temporary counter to zero
      if (errorCounter[ERR_STUCK] == 0 &&
          stateCurr != STATE_OFF &&
          stateCurr != STATE_MANUAL &&
          stateCurr != STATE_STATION &&
          stateCurr != STATE_STATION_CHARGING &&
          stateCurr != STATE_STATION_CHECK &&
          stateCurr != STATE_STATION_REV &&
          stateCurr != STATE_STATION_ROLL &&
          stateCurr != STATE_REMOTE &&
          stateCurr != STATE_ERROR)
      {
        cutter.enable();
        errorCounterMax[ERR_STUCK] = 0;
      }
    }

    if (robotIsStuckCounter >= 5)
    {
      cutter.disable();
      if (errorCounterMax[ERR_STUCK] >= 3)
      {
        // robot is definitely stuck and unable to move
        Console.println(F("Error: Mower is stuck"));
        incErrorCounter(ERR_STUCK);
        setNextState(STATE_ERROR);
        //robotIsStuckedCounter = 0;
      }
      else if (errorCounter[ERR_STUCK] < 3)
      {
        // mower tries 3 times to get unstuck
        if (stateCurr == STATE_FORWARD)
        {
          cutter.disable();
          incErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0);
          reverseOrChangeDirection(RIGHT);
        }
        else if (stateCurr == STATE_ROLL)
        {
          cutter.disable();
          incErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0);
          setNextState(STATE_FORWARD);
        }
      }
    }
  }
}

void Robot::processGPSData()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeGPS)
  {
    return;
  }
  nextTimeGPS = curMillis + 1000;

  float nlat;
  float nlon;
  unsigned long age;
  gps.f_get_position(&nlat, &nlon, &age);

  if (nlat == Gps::GPS_INVALID_F_ANGLE)
  {
    return;
  }

  if (gpsLon == 0)
  {
    gpsLon = nlon;  // this is xy (0,0)
    gpsLat = nlat;
    return;
  }

  gpsX = gps.distance_between(nlat, gpsLon, gpsLat, gpsLon);
  gpsY = gps.distance_between(gpsLat, nlon, gpsLat, gpsLon);
}

void Robot::checkTimeout()
{
  if (stateTime > wheels.forwardTimeMax)
  {
    setNextState(STATE_REVERSE, !rollDir); // Toggle roll direction
  }
}

void Robot::loop()
{
  unsigned long curMillis = millis();
  stateTime = curMillis - stateStartTime;
  ADCMan.run();
  readSerial();
  if (rc.readSerial())
  {
    resetIdleTime();
  }
  readSensors();
  checkBattery();
  checkIfStuck();
  checkRobotStats();
  odometer.loop();
  checkOdometerFaults();
  checkButton();
  checkTilt();

  wheels.control();
  cutterControl();

  if (imu.use)
  {
    imu.update();
  }

  if (gpsUse)
  {
    gps.feed();
    processGPSData();
  }

  if (curMillis >= nextTimePfodLoop)
  {
    nextTimePfodLoop = curMillis + 200;
    rc.run();
  }

  if (curMillis >= nextTimeInfo)
  {
    nextTimeInfo = curMillis + 1000;
    printInfo(Console);
    if (stateCurr == STATE_REMOTE)
    {
      printRemote();
    }
    loopsPerSec = loopsPerSecCounter;
    loopsPerSecCounter = 0;
  }

  // state machine - things to do *PERMANENTLY* for current state
  // robot state machine
  // http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
  switch (stateCurr)
  {
    case STATE_ERROR:
      // fatal-error
      if (curMillis >= nextTimeErrorBeep)
      {
        nextTimeErrorBeep = curMillis + 300;
        beep(1, true);
      }
      break;

    case STATE_OFF:
      // robot is turned off
      if (batMonitor && curMillis - stateStartTime > 2000)
      {
        if (chgVoltage > 5.0 && batVoltage > 8)
        {
          beep(2, true);
          setNextState(STATE_STATION);
        }
      }
      imuDriveHeading = imu.getYaw();
      break;

    case STATE_REMOTE:
      // remote control mode (RC)
//      if (remoteSwitch > 50)
//      {
//        setNextState(STATE_FORWARD, 0);
//      }
      wheels.setSpeed(remoteSpeed);
      wheels.setSteer(remoteSteer);
      cutter.setSpeed(remoteMow);
      break;

    case STATE_MANUAL:
      break;

    case STATE_FORWARD:
      // driving forward
      if (mowPatternCurr == MOW_BIDIR)
      {
        double ratio = wheels.biDirSpeedRatio1;
        if (stateTime > 4000)
        {
          ratio = wheels.biDirSpeedRatio2;
        }
        if (rollDir == RIGHT)
        {
          wheels.wheel[Wheel::RIGHT].motor.rpmSet =
              ((double)wheels.wheel[Wheel::LEFT].motor.rpmSet) * ratio;
        }
        else
        {
          wheels.wheel[Wheel::LEFT].motor.rpmSet =
              ((double)wheels.wheel[Wheel::RIGHT].motor.rpmSet) * ratio;
        }
      }
      checkErrorCounter();
      checkTimer();
      checkRain();
      checkMotorPower();
      checkBumpers();
      checkDrop();
      checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      checkTimeout();
      break;

    case STATE_ROLL:
      checkMotorPower();
      checkBumpers();
      checkDrop();
      //checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      // making a roll (left/right)
      if (mowPatternCurr == MOW_LANES)
      {
        if (abs(distancePI(imu.getYaw(), imuRollHeading)) < PI / 36)
        {
          setNextState(STATE_FORWARD);
        }
      }
      else
      {
        if (curMillis >= stateEndTime)
        {
          setNextState(STATE_FORWARD);
        }
      }
      break;

    case STATE_ROLL_WAIT:
      // making a roll (left/right)
      //if (abs(distancePI(imuYaw, imuRollHeading)) < PI/36) setNextState(STATE_OFF,0);
      break;

    case STATE_CIRCLE:
      // driving circles
      break;

    case STATE_REVERSE:
      // driving reverse
      checkErrorCounter();
      checkTimer();
      checkMotorPower();
      checkBumpers();
      checkDrop();
      //checkSonar();
      checkPerimeterBoundary();
      checkLawn();

      if (mowPatternCurr == MOW_BIDIR)
      {
        double ratio = wheels.biDirSpeedRatio1;
        if (stateTime > 4000)
        {
          ratio = wheels.biDirSpeedRatio2;
        }
        if (rollDir == RIGHT)
        {
          wheels.wheel[Wheel::RIGHT].motor.rpmSet =
              ((double)wheels.wheel[Wheel::LEFT].motor.rpmSet) * ratio;
        }
        else
        {
          wheels.wheel[Wheel::LEFT].motor.rpmSet =
              ((double)wheels.wheel[Wheel::RIGHT].motor.rpmSet) * ratio;
        }
        if (stateTime > wheels.forwardTimeMax)
        {
          // timeout
          if (rollDir == RIGHT)
          {
            setNextState(STATE_FORWARD, LEFT); // toggle roll dir
          }
          else
          {
            setNextState(STATE_FORWARD, RIGHT);
          }
        }
      }
      else
      {
        if (curMillis >= stateEndTime)
        {
          setNextState(STATE_ROLL, rollDir);
        }
      }
      break;

    case STATE_PERI_ROLL:
      // perimeter tracking roll
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_PERI_FIND);
      }
      break;

    case STATE_PERI_REV:
      // perimeter tracking reverse
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_PERI_ROLL, rollDir);
      }
      break;

    case STATE_PERI_FIND:
      // find perimeter
      if (wheels.wheel[Wheel::LEFT].motor.rpmSet == wheels.wheel[Wheel::RIGHT].motor.rpmSet)
      { // do not check during 'outside=>inside' rotation
        checkMotorPower();
        checkBumpersPerimeter();
        checkSonar();
      }
      checkPerimeterFind();
      checkTimeout();
      break;

    case STATE_PERI_TRACK:
      // track perimeter
      checkMotorPower();
      checkBumpersPerimeter();
      //checkSonar();
      if (batMonitor)
      {
        if (chgVoltage > 5.0)
        {
          setNextState(STATE_STATION);
        }
      }
      break;

    case STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (batMonitor)
      {
        if (chgVoltage > 5.0 && batVoltage > 8)
        {
          if (batVoltage < startChargingIfBelow &&
              curMillis - stateStartTime > 2000)
          {
            setNextState(STATE_STATION_CHARGING);
          }
          else
          {
            checkTimer();
          }
        }
        else
        {
          setNextState(STATE_OFF);
        }
      }
      else
      {
        checkTimer();
      }
      break;

    case STATE_STATION_CHARGING:
      // waiting until charging completed
      if (batMonitor)
      {
        if (chgCurrent < batFullCurrent && curMillis - stateStartTime > 2000)
        {
          setNextState(STATE_STATION);
        }
        else if (curMillis - stateStartTime > chargingTimeout)
        {
          incErrorCounter(ERR_BATTERY);
          setNextState(STATE_ERROR);
        }
      }
      break;

    case STATE_PERI_OUT_FORW:
      checkPerimeterBoundary();
      //if (millis() >= stateEndTime) setNextState(STATE_PERI_OUT_ROLL, rollDir);
      if (perimeterInside || curMillis >= stateEndTime)
      {
        setNextState(STATE_PERI_OUT_ROLL, rollDir);
      }
      break;

    case STATE_PERI_OUT_REV:
      checkPerimeterBoundary();
      // if (millis() >= stateEndTime) setNextState(STATE_PERI_OUT_ROLL, rollDir);
      if (perimeterInside || curMillis >= stateEndTime)
      {
        setNextState(STATE_PERI_OUT_ROLL, rollDir);
      }
      break;

    case STATE_PERI_OUT_ROLL:
      if (millis() >= stateEndTime)
      {
        setNextState(STATE_FORWARD);
      }
      break;

    case STATE_STATION_CHECK:
      // check for charging voltage disappearing before leaving charging station
      if (millis() >= stateEndTime)
      {
        if (chgVoltage > 5)
        {
          incErrorCounter(ERR_CHARGER);
          setNextState(STATE_ERROR);
        }
        else
        {
          setNextState(STATE_STATION_REV);
        }
      }
      break;

    case STATE_STATION_REV:
      // charging: drive reverse
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_STATION_ROLL);
      }
      break;

    case STATE_STATION_ROLL:
      // charging: roll
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_STATION_FORW);
      }
      break;

    case STATE_STATION_FORW:
      // forward (charge station)
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_FORWARD);
      }
      break;
  } // end switch

  // next line deactivated (issue with RC failsafe)
//  if (useRemoteRC && remoteSwitch < -50)
//  {
//    setNextState(STATE_REMOTE, 0);
//  }

  // decide which motor control to use
  if ((mowPatternCurr == MOW_LANES && stateCurr == STATE_ROLL) ||
      stateCurr  == STATE_ROLL_WAIT)
  {
    wheelControl_imuRoll();
  }
  else if (stateCurr == STATE_PERI_TRACK)
  {
    wheelControl_perimeter();
  }
  else if (stateCurr == STATE_FORWARD &&
           (imu.correctDir || mowPatternCurr == MOW_LANES))
  {
    wheelControl_imuDir();
  }
  else
  {
    wheelControl_normal();
  }

  if (stateCurr != STATE_REMOTE)
  {
    // TODO: This feels dangerous!!!
    cutter.motor.setPwmSet(cutter.motor.pwmMax);
  }

  bumpers.clearHit();
  dropSensors.clearDetected();

  loopsPerSecCounter++;
}
