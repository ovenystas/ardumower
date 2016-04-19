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

#include "robot.h"

#define MAGIC 51

#define ADDR_USER_SETTINGS 0
#define ADDR_ERR_COUNTERS 400
#define ADDR_ROBOT_STATS 800

char* stateNames[] = { "OFF ", "RC  ", "FORW", "ROLL", "REV ", "CIRC", "ERR ",
                       "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK",
                       "STREV", "STROL", "STFOR", "MANU", "ROLW", "POUTFOR",
                       "POUTREV", "POUTROLL" };

char *mowPatternNames[] = { "RANDOM", "LANE", "BIDIR" };

char* consoleModeNames[] = { "sen_counters", "sen_values", "perimeter" };

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
  robotIsStuckedCounter = 0;

  imuDriveHeading = 0;
  imuRollHeading = 0;
  imuRollDir = LEFT;

  perimeterMag = 0;
  perimeterInside = true;
  perimeterCounter = 0;
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;

  batADC = 0;
  batVoltage = 0;
  batRefFactor = 0;
  batCapacity = 0;
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
  nextTimeCheckIfStucked = 0;
  nextTimePfodLoop = 0;
  nextTimeErrorCounterReset = 0;
  nextTimeErrorBeep = 0;

  nextTimeRobotStats = 0;
  statsMowTimeMinutesTripCounter = 0;
  statsBatteryChargingCounter = 0;
}

char* Robot::stateName()
{
  return stateNames[stateCurr];
}

char *Robot::mowPatternName()
{
  return mowPatternNames[mowPatternCurr];
}

void Robot::loadSaveRobotStats(boolean readflag)
{
  if (readflag)
  {
    Console.println(F("loadSaveRobotStats: read"));
  }
  else
  {
    Console.println(F("loadSaveRobotStats: write"));
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
  Console.print(F("loadSaveRobotStats addrstop="));
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

void Robot::loadSaveErrorCounters(boolean readflag)
{
  if (readflag)
  {
    Console.println(F("loadSaveErrorCounters: read"));
  }
  else
  {
    Console.println(F("loadSaveErrorCounters: write"));
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
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, errorCounterMax);
  Console.print(F("loadSaveErrorCounters addrstop="));
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

void Robot::loadSaveUserSettings(boolean readflag)
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
    addErrorCounter(ERR_EEPROM_DATA);
    setNextState(STATE_ERROR, 0);
    return;
  }
  eereadwrite(readflag, addr, developerActive);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.acceleration);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.rpmMax);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.pwmMax);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.powerMax);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::RIGHT].motor.senseScale);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.senseScale);
  eereadwrite(readflag, addr, wheels.rollTimeMax);
  eereadwrite(readflag, addr, wheels.rollTimeMin);
  eereadwrite(readflag, addr, wheels.reverseTime);
  eereadwrite(readflag, addr, wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime);
  eereadwrite(readflag, addr, wheels.forwardTimeMax);
  eereadwrite(readflag, addr, cutter.motor.pwmMax);
  eereadwrite(readflag, addr, cutter.motor.powerMax);
  eereadwrite(readflag, addr, cutter.motor.rpmSet);
  eereadwrite(readflag, addr, cutter.motor.senseScale);
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
  eereadwrite(readflag, addr, bumpers.use);
  eereadwrite(readflag, addr, sonars.use);
  for (uint8_t i = 0; i < Sonars::END; i++)
  {
    eereadwrite(readflag, addr, sonars.sonar[i].use);
  }
  eereadwrite(readflag, addr, sonars.triggerBelow);
  eereadwrite(readflag, addr, perimeterUse);
  eereadwrite(readflag, addr, perimeter.timedOutIfBelowSmag);
  eereadwrite(readflag, addr, perimeterTriggerTimeout);
  eereadwrite(readflag, addr, perimeterOutRollTimeMax);
  eereadwrite(readflag, addr, perimeterOutRollTimeMin);
  eereadwrite(readflag, addr, perimeterOutRevTime);
  eereadwrite(readflag, addr, perimeterTrackRollTime);
  eereadwrite(readflag, addr, perimeterTrackRevTime);
  eereadwrite(readflag, addr, perimeter.pid.Kp);
  eereadwrite(readflag, addr, perimeter.pid.Ki);
  eereadwrite(readflag, addr, perimeter.pid.Kd);
  eereadwrite(readflag, addr, perimeter.useDifferentialPerimeterSignal);
  eereadwrite(readflag, addr, perimeter.swapCoilPolarity);
  eereadwrite(readflag, addr, perimeter.timeOutSecIfNotInside);
  eereadwrite(readflag, addr, trackingBlockInnerWheelWhilePerimeterStruggling);
  eereadwrite(readflag, addr, lawnSensor.use);
  eereadwrite(readflag, addr, imu.use);
  eereadwrite(readflag, addr, imu.correctDir);
  eereadwrite(readflag, addr, imu.pid[IMU::DIR].Kp);
  eereadwrite(readflag, addr, imu.pid[IMU::DIR].Ki);
  eereadwrite(readflag, addr, imu.pid[IMU::DIR].Kd);
  eereadwrite(readflag, addr, imu.pid[IMU::ROLL].Kp);
  eereadwrite(readflag, addr, imu.pid[IMU::ROLL].Ki);
  eereadwrite(readflag, addr, imu.pid[IMU::ROLL].Kd);
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
  eereadwrite(readflag, addr, odometer.encoder[Odometer::LEFT].swapDir);
  eereadwrite(readflag, addr, odometer.encoder[Odometer::RIGHT].swapDir);
  eereadwrite(readflag, addr, odometer.encoder[Odometer::LEFT].twoWay);
  eereadwrite(readflag, addr, button.use);
  eereadwrite(readflag, addr, userSwitch1);
  eereadwrite(readflag, addr, userSwitch2);
  eereadwrite(readflag, addr, userSwitch3);
  eereadwrite(readflag, addr, timerUse);
  eereadwrite(readflag, addr, timer);
  eereadwrite(readflag, addr, rainSensor.use);
  eereadwrite(readflag, addr, gpsUse);
  eereadwrite(readflag, addr, stuckedIfGpsSpeedBelow);
  eereadwrite(readflag, addr, gpsSpeedIgnoreTime);
  eereadwrite(readflag, addr, dropSensors.use);
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
  eewrite(addr, (short) 0); // magic
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
  Console.print(F("LEFT.senseScale : "));
  Console.println(wheels.wheel[Wheel::LEFT].motor.senseScale);
  Console.print(F("RIGHT.senseScale : "));
  Console.println(wheels.wheel[Wheel::RIGHT].motor.senseScale);
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
  Console.print(F("senseScale : "));
  Console.println(cutter.motor.senseScale);
  Console.print(F("pid.Kp : "));
  Console.println(cutter.motor.pid.Kp);
  Console.print(F("pid.Ki : "));
  Console.println(cutter.motor.pid.Ki);
  Console.print(F("pid.Kd : "));
  Console.println(cutter.motor.pid.Kd);

  // ------ bumper ------------------------------------
  Console.println(F("== Bumpers =="));
  Console.print(F("use : "));
  Console.println(bumpers.use);

  // ------ drop ------------------------------------
  Console.println(F("== Drop sensors =="));
  Console.print(F("use : "));
  Console.println(dropSensors.use);
  Console.print(F("contactType : "));
  Console.println(dropSensors.dropSensor[DropSensors::LEFT].contactType);  // Assume left and right has same contact type

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
  Console.println(perimeterUse);
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
  Console.println(perimeter.pid.Kp);
  Console.print(F("pid.Ki : "));
  Console.println(perimeter.pid.Ki);
  Console.print(F("pid.Kd : "));
  Console.println(perimeter.pid.Kd);
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
  Console.println(imu.pid[IMU::DIR].Kp);
  Console.print(F("pid[DIR].Ki : "));
  Console.println(imu.pid[IMU::DIR].Ki);
  Console.print(F("pid[DIR].Kd : "));
  Console.println(imu.pid[IMU::DIR].Kd);
  Console.print(F("pid[ROLL].Kp : "));
  Console.println(imu.pid[IMU::ROLL].Kp);
  Console.print(F("pid[ROLL].Ki : "));
  Console.println(imu.pid[IMU::ROLL].Ki);
  Console.print(F("pid[ROLL].Kd : "));
  Console.println(imu.pid[IMU::ROLL].Kd);

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
  Console.println(odometer.encoder[Odometer::LEFT].twoWay);
  Console.print(F("ticksPerRevolution : "));
  Console.println(odometer.ticksPerRevolution);
  Console.print(F("ticksPerCm : "));
  Console.println(odometer.ticksPerCm);
  Console.print(F("wheelBaseCm : "));
  Console.println(odometer.wheelBaseCm);
  Console.print(F("LEFT.swapDir : "));
  Console.println(odometer.encoder[Odometer::LEFT].swapDir);
  Console.print(F("RIGHT.swapDir : "));
  Console.println(odometer.encoder[Odometer::RIGHT].swapDir);

  // ----- GPS -------------------------------------------
  Console.println(F("== GPS =="));
  Console.print(F("use : "));
  Console.println(gpsUse);
  Console.print(F("stuckedIfGpsSpeedBelow : "));
  Console.println(stuckedIfGpsSpeedBelow);
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

void Robot::addErrorCounter(enum errorE errType)
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
  Console.println(F("resetErrorCounters"));
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
    for (int i = 0; i < ERR_ENUM_COUNT; i++)
    {
      // set to fatal error if any temporary error counter reaches 10
      if (errorCounter[i] > 10)
      {
        setNextState(STATE_ERROR, 0);
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
int Robot::rcValue(int ppmTime)
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
void Robot::setRemotePPMState(unsigned long timeMicros,
                              boolean remoteSpeedState,
                              boolean remoteSteerState,
                              boolean remoteMowState,
                              boolean remoteSwitchState)
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
void Robot::setMotorPWM(int pwm, unsigned long samplingTime,
                        uint8_t motor, boolean useAccel)
{
  if (useAccel)
  {
    // http://phrogz.net/js/framerate-independent-low-pass-filter.html
    // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
    wheels.wheel[motor].motor.pwmCur += samplingTime *
        (pwm - wheels.wheel[motor].motor.pwmCur) /
        wheels.wheel[motor].motor.acceleration;
  }

  // ----- driver protection (avoids driver explosion) ----------
  if ((pwm < 0 && wheels.wheel[motor].motor.pwmCur >= 0) ||
      (pwm > 0 && wheels.wheel[motor].motor.pwmCur <= 0))
  { // changing direction should take place?
    if (wheels.wheel[motor].motor.zeroTimeout != 0)
    {
      pwm = wheels.wheel[motor].motor.pwmCur - wheels.wheel[motor].motor.pwmCur * (float)samplingTime / 200.0; // reduce speed
    }
  }

  if (odometer.use)
  {
    wheels.wheel[motor].motor.pwmCur = pwm;
    if (abs(odometer.encoder[motor].wheelRpmCurr) < 1)
    {
      wheels.wheel[motor].motor.zeroTimeout = max(0, (wheels.wheel[motor].motor.zeroTimeout - samplingTime));
    }
    else
    {
      wheels.wheel[motor].motor.zeroTimeout = 500;
    }
  }
  else
  {
    if (pwm == 0)
    {
      wheels.wheel[motor].motor.zeroTimeout = max(0, (wheels.wheel[motor].motor.zeroTimeout - samplingTime));
    }
    else
    {
      wheels.wheel[motor].motor.zeroTimeout = 700;
    }
  }

  wheels.wheel[motor].motor.setSpeed();
}

// sets wheel motor actuators
// - driver protection: delays polarity change until motor speed (EMV) is zero
//   http://wiki.ardumower.de/images/a/a5/Motor_polarity_switch_protection.png
// - optional: ensures that the motors (and gears) are not switched to 0% (or 100%) too fast (motorAccel)
void Robot::setMotorPWMs(int pwmLeft, int pwmRight, boolean useAccel)
{
  unsigned long samplingTime;

  samplingTime = wheels.wheel[Wheel::LEFT].motor.getSamplingTime();
  setMotorPWM(pwmLeft, samplingTime, LEFT, useAccel);

  samplingTime = wheels.wheel[Wheel::LEFT].motor.getSamplingTime();
  setMotorPWM(pwmRight, samplingTime, RIGHT, useAccel);
}

// sets mower motor actuator
// - ensures that the motor is not switched to 100% too fast (cutter.motor.acceleration)
// - ensures that the motor voltage is not higher than motorMowSpeedMaxPwm
void Robot::setMotorMowPWM(int pwm, boolean useAccel)
{
  unsigned long samplingTime = cutter.motor.getSamplingTime();

  // we need to ignore acceleration for PID control, and we can ignore if speed is lowered (e.g. motor is shut down)
  if (!useAccel || pwm < cutter.motor.pwmCur)
  {
    cutter.motor.pwmCur = min(cutter.motor.pwmMax, max(0, pwm));
  }
  else
  {
    // http://phrogz.net/js/framerate-independent-low-pass-filter.html
    // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
    int addPwm = samplingTime * (pwm - cutter.motor.pwmCur) / cutter.motor.acceleration;
    cutter.motor.pwmCur = min(cutter.motor.pwmMax, max(0, cutter.motor.pwmCur + addPwm));
  }
  cutter.motor.setSpeed();
}

// PID controller: roll robot to heading (requires IMU)
void Robot::motorControlImuRoll()
{
  unsigned long curMillis = millis();
  if (curMillis < imu.nextTimeControl)
  {
    return;
  }
  imu.nextTimeControl = curMillis + 100;

  // Regelbereich entspricht 80% der maximalen Drehzahl am Antriebsrad (motorSpeedMaxRpm)
  float imuPidX = distancePI(imu.ypr.yaw, imuRollHeading) / PI * 180.0;
  imu.pid[IMU::ROLL].w = 0;
  imu.pid[IMU::ROLL].y_min = -wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25; // da der Roll generell langsamer erfolgen soll
  imu.pid[IMU::ROLL].y_max = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;   //
  imu.pid[IMU::ROLL].max_output = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;    //
  float imuPidY = imu.pid[IMU::ROLL].compute(imuPidX);

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  wheels.wheel[Wheel::LEFT].motor.pid.w = -imuPidY;                // SOLL
  wheels.wheel[Wheel::LEFT].motor.pid.y_min = -wheels.wheel[Wheel::LEFT].motor.pwmMax;        // Regel-MIN
  wheels.wheel[Wheel::LEFT].motor.pid.y_max = wheels.wheel[Wheel::LEFT].motor.pwmMax;   // Regel-MAX
  wheels.wheel[Wheel::LEFT].motor.pid.max_output = wheels.wheel[Wheel::LEFT].motor.pwmMax;    // Begrenzung
  float leftWheelPidY = wheels.wheel[Wheel::LEFT].motor.pid.compute(odometer.encoder[Odometer::LEFT].wheelRpmCurr);
  int leftSpeed = max(-wheels.wheel[Wheel::LEFT].motor.pwmMax,
                      min(wheels.wheel[Wheel::LEFT].motor.pwmMax,
                          wheels.wheel[Wheel::LEFT].motor.pwmCur + leftWheelPidY));

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  wheels.wheel[Wheel::RIGHT].motor.pid.w = imuPidY;                // SOLL
  wheels.wheel[Wheel::RIGHT].motor.pid.y_min = -wheels.wheel[Wheel::RIGHT].motor.pwmMax;       // Regel-MIN
  wheels.wheel[Wheel::RIGHT].motor.pid.y_max = wheels.wheel[Wheel::RIGHT].motor.pwmMax;  // Regel-MAX
  wheels.wheel[Wheel::RIGHT].motor.pid.max_output = wheels.wheel[Wheel::RIGHT].motor.pwmMax;   // Begrenzung
  float rightWheelPidY = wheels.wheel[Wheel::RIGHT].motor.pid.compute(odometer.encoder[Odometer::RIGHT].wheelRpmCurr);
  int rightSpeed = max(-wheels.wheel[Wheel::RIGHT].motor.pwmMax,
      min(wheels.wheel[Wheel::RIGHT].motor.pwmMax, wheels.wheel[Wheel::RIGHT].motor.pwmCur + rightWheelPidY));

  if ((stateCurr == STATE_OFF || stateCurr == STATE_STATION || stateCurr == STATE_ERROR)
      && (millis() - stateStartTime > 1000))
  {
    leftSpeed = rightSpeed = 0; // ensures PWM is zero if OFF/CHARGING
  }
  setMotorPWMs(leftSpeed, rightSpeed, false);
}

// PID controller: track perimeter
void Robot::motorControlPerimeter()
{
  unsigned long curMillis = millis();
  if (curMillis < perimeter.nextTimeControl)
  {
    return;
  }
  perimeter.nextTimeControl = curMillis + 100;

  if ((curMillis > stateStartTime + 5000) &&
      (curMillis > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut))
  {
    int leftSpeed = wheels.wheel[Wheel::LEFT].motor.pwmMax / 1.5;
    int rightSpeed = wheels.wheel[Wheel::RIGHT].motor.pwmMax / 1.5;

    // robot is wheel-spinning while tracking => roll to get ground again
    if (!trackingBlockInnerWheelWhilePerimeterStruggling)
    {
      if (perimeterMag < 0)
      {
        setMotorPWMs(-leftSpeed, rightSpeed, false);
      }
      else
      {
        setMotorPWMs(leftSpeed, -rightSpeed, false);
      }
    }

    else if (trackingBlockInnerWheelWhilePerimeterStruggling)
    {
      if (perimeterMag < 0)
      {
        setMotorPWMs(0, rightSpeed, false);
      }
      else
      {
        setMotorPWMs(leftSpeed, 0, false);
      }
    }

    if (millis() > perimeterLastTransitionTime + trackingErrorTimeOut)
    {
      Console.println("Error: tracking error");
      addErrorCounter(ERR_TRACKING);
      //setNextState(STATE_ERROR,0);
      setNextState(STATE_PERI_FIND, 0);
    }
    return;
  }

  float x;
  if (perimeterMag < 0)
  {
    x = -1;
  }
  else if (perimeterMag > 0)
  {
    x = 1;
  }
  else
  {
    x = 0;
  }
  perimeter.pid.w = 0;
  perimeter.pid.y_min = -wheels.wheel[Wheel::LEFT].motor.pwmMax;
  perimeter.pid.y_max = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  perimeter.pid.max_output = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  float y = perimeter.pid.compute(x);

  int leftSpeed = max(-wheels.wheel[Wheel::LEFT].motor.pwmMax,
                      min(wheels.wheel[Wheel::LEFT].motor.pwmMax,
                          wheels.wheel[Wheel::LEFT].motor.pwmMax / 2 - y));


  int rightSpeed = max(-wheels.wheel[Wheel::RIGHT].motor.pwmMax,
                       min(wheels.wheel[Wheel::RIGHT].motor.pwmMax,
                           wheels.wheel[Wheel::RIGHT].motor.pwmMax / 2 + y));

  setMotorPWMs(leftSpeed, rightSpeed, false);
}

// PID controller: correct direction during normal driving (requires IMU)
void Robot::motorControlImuDir()
{
  unsigned long curMillis = millis();
  if (curMillis < imu.nextTimeControl)
  {
    return;
  }
  imu.nextTimeControl = curMillis + 100;

  int correctLeft = 0;
  int correctRight = 0;

  // Regelbereich entspricht maximaler Drehzahl am Antriebsrad (motorSpeedMaxRpm)
  imu.pid[IMU::DIR].x = distancePI(imu.ypr.yaw, imuDriveHeading) / PI * 180.0;
  imu.pid[IMU::DIR].w = 0;
  imu.pid[IMU::DIR].y_min = -wheels.wheel[Wheel::LEFT].motor.rpmMax;
  imu.pid[IMU::DIR].y_max = wheels.wheel[Wheel::LEFT].motor.rpmMax;
  imu.pid[IMU::DIR].max_output = wheels.wheel[Wheel::LEFT].motor.rpmMax;
  imu.pid[IMU::DIR].compute();

  if (imu.pid[IMU::DIR].y < 0)
  {
    correctRight = abs(imu.pid[IMU::DIR].y);
  }
  if (imu.pid[IMU::DIR].y > 0)
  {
    correctLeft = abs(imu.pid[IMU::DIR].y);
  }

  // Korrektur erfolgt über Abbremsen des linken Antriebsrades, falls Kursabweichung nach rechts
  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  wheels.wheel[Wheel::LEFT].motor.pid.x = odometer.encoder[LEFT].wheelRpmCurr;                     // IST
  wheels.wheel[Wheel::LEFT].motor.pid.w = wheels.wheel[Wheel::LEFT].motor.rpmSet - correctLeft;     // SOLL
  wheels.wheel[Wheel::LEFT].motor.pid.y_min = -wheels.wheel[Wheel::LEFT].motor.pwmMax;            // Regel-MIN
  wheels.wheel[Wheel::LEFT].motor.pid.y_max = wheels.wheel[Wheel::LEFT].motor.pwmMax;       // Regel-MAX
  wheels.wheel[Wheel::LEFT].motor.pid.max_output = wheels.wheel[Wheel::LEFT].motor.pwmMax;        // Begrenzung
  wheels.wheel[Wheel::LEFT].motor.pid.compute();
  int leftSpeed = max(-wheels.wheel[Wheel::LEFT].motor.pwmMax,
                      min(wheels.wheel[Wheel::LEFT].motor.pwmMax, wheels.wheel[Wheel::LEFT].motor.pwmCur + wheels.wheel[Wheel::LEFT].motor.pid.y));
  if ((wheels.wheel[Wheel::LEFT].motor.rpmSet >= 0) && (leftSpeed < 0))
  {
    leftSpeed = 0;
  }
  if ((wheels.wheel[Wheel::LEFT].motor.rpmSet <= 0) && (leftSpeed > 0))
  {
    leftSpeed = 0;
  }

  // Korrektur erfolgt über Abbremsen des rechten Antriebsrades, falls Kursabweichung nach links
  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  wheels.wheel[Wheel::RIGHT].motor.pid.x = odometer.encoder[RIGHT].wheelRpmCurr;                   // IST
  wheels.wheel[Wheel::RIGHT].motor.pid.w = wheels.wheel[Wheel::RIGHT].motor.rpmSet - correctRight;  // SOLL
  wheels.wheel[Wheel::RIGHT].motor.pid.y_min = -wheels.wheel[Wheel::LEFT].motor.pwmMax;           // Regel-MIN
  wheels.wheel[Wheel::RIGHT].motor.pid.y_max = wheels.wheel[Wheel::LEFT].motor.pwmMax;      // Regel-MAX
  wheels.wheel[Wheel::RIGHT].motor.pid.max_output = wheels.wheel[Wheel::LEFT].motor.pwmMax;       // Begrenzung
  wheels.wheel[Wheel::RIGHT].motor.pid.compute();
  int rightSpeed = max(
      -wheels.wheel[Wheel::LEFT].motor.pwmMax,
      min(wheels.wheel[Wheel::LEFT].motor.pwmMax, wheels.wheel[Wheel::RIGHT].motor.pwmCur + wheels.wheel[Wheel::RIGHT].motor.pid.y));
  if ((wheels.wheel[Wheel::RIGHT].motor.rpmSet >= 0) && (rightSpeed < 0))
  {
    rightSpeed = 0;
  }
  if ((wheels.wheel[Wheel::RIGHT].motor.rpmSet <= 0) && (rightSpeed > 0))
  {
    rightSpeed = 0;
  }

  if (((stateCurr == STATE_OFF) || (stateCurr == STATE_STATION)
       || (stateCurr == STATE_ERROR))
      && (millis() - stateStartTime > 1000))
  {
    leftSpeed = rightSpeed = 0; // ensures PWM is zero if OFF/CHARGING
  }
  setMotorPWMs(leftSpeed, rightSpeed, false);
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

  if ((stateCurr == STATE_FORWARD) && (curMillis - stateStartTime > 8000))
  {
    // just check if odometer sensors may not be working at all
    for (uint8_t i = LEFT; i <= RIGHT; i++)
    {
      if (wheels.wheel[i].motor.pwmCur > 100 && abs(odometer.encoder[i].wheelRpmCurr) < 1)
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
      if ((wheels.wheel[i].motor.pwmCur > 100 && odometer.encoder[i].wheelRpmCurr < -3) ||
          (wheels.wheel[i].motor.pwmCur < -100 && odometer.encoder[i].wheelRpmCurr > 3))
      {
        err[i] = true;
      }
    }
  }

  if (err[LEFT])
  {
    Console.print("Left odometer error: PWM=");
    Console.print(wheels.wheel[Wheel::LEFT].motor.pwmCur);
    Console.print("\tRPM=");
    Console.println(odometer.encoder[Odometer::LEFT].wheelRpmCurr);
    addErrorCounter(ERR_ODOMETER_LEFT);
    setNextState(STATE_ERROR, 0);
  }

  if (err[RIGHT])
  {
    Console.print("Right odometer error: PWM=");
    Console.print(wheels.wheel[Wheel::RIGHT].motor.pwmCur);
    Console.print("\tRPM=");
    Console.println(odometer.encoder[Odometer::RIGHT].wheelRpmCurr);
    addErrorCounter(ERR_ODOMETER_RIGHT);
    setNextState(STATE_ERROR, 0);
  }
}

void Robot::motorControl()
{
  unsigned long curMillis = millis();
  if (curMillis < wheels.wheel[Wheel::LEFT].motor.nextTimeControl)
  {
    return;
  }

  wheels.wheel[Wheel::LEFT].motor.nextTimeControl = curMillis + 100;
  static unsigned long nextMotorControlOutputTime = 0;

  int speed[2];
  if (odometer.use)
  {
    for (uint8_t i = Wheel::LEFT; i <= Wheel::RIGHT; i++)
    {
      // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm),
      // um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
      wheels.wheel[i].motor.pid.x = odometer.encoder[i].wheelRpmCurr;               // IST
      if (curMillis < stateStartTime + wheels.wheel[i].motor.zeroSettleTime)
      {
        wheels.wheel[i].motor.pid.w = 0; // get zero speed first after state change
      }
      wheels.wheel[i].motor.pid.y_min = -wheels.wheel[i].motor.pwmMax;       // Regel-MIN
      wheels.wheel[i].motor.pid.y_max = wheels.wheel[i].motor.pwmMax;        // Regel-MAX
      wheels.wheel[i].motor.pid.max_output = wheels.wheel[i].motor.pwmMax;   // Begrenzung
      wheels.wheel[i].motor.pid.compute();
      speed[i] = max(-wheels.wheel[i].motor.pwmMax,
                     min(wheels.wheel[i].motor.pwmMax,
                         wheels.wheel[i].motor.pwmCur + wheels.wheel[i].motor.pid.y));
      // Sven reported the next two lines do make the PID controlling worse
      //if((motorSpeedRpmSet[i] >= 0 ) && (speed[i] < 0 )) speed[i] = 0;
      //if((motorSpeedRpmSet[i] <= 0 ) && (speed[i] > 0 )) speed[i] = 0;

      if (abs(wheels.wheel[i].motor.pid.x) < 2 && abs(wheels.wheel[i].motor.pid.w) < 0.1)
      {
        speed[i] = 0; // ensures PWM is really zero
      }
    }

    /*if (curMillis >= nextMotorControlOutputTime)
     {
       nextMotorControlOutputTime = curMillis + 3000;
       Console.print("motorPID[");
       if (i = LEFT)
       {
         Console.print("LEFT");
       }
       else
       {
         Console.print("RIGHT");
       }
       Console.print("]PID x=");
       Console.print(wheels.wheel[i].motor.pid.x);
       Console.print("\tPID w=");
       Console.print(wheels.wheel[i].motor.pid.w);
       Console.print("\tPID y=");
       Console.print(wheels.wheel[i].motor.pid.y);
       Console.print("\tPWM=");
       Console.println(speed[i]);
     } */

    setMotorPWMs(speed[LEFT], speed[RIGHT], false);
  }
  else
  {
    for (uint8_t i = Wheel::LEFT; i <= Wheel::RIGHT; i++)
    {
      speed[i] = min(wheels.wheel[i].motor.pwmMax,
                     max(-wheels.wheel[i].motor.pwmMax,
                         map(wheels.wheel[i].motor.rpmSet,
                             -wheels.wheel[i].motor.rpmMax,
                             wheels.wheel[i].motor.rpmMax,
                             -wheels.wheel[i].motor.pwmMax,
                             wheels.wheel[i].motor.pwmMax)));
    }
    if (curMillis < stateStartTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime)
    {
      // slow down at state start
      speed[LEFT] = 0;
      speed[RIGHT] = 0;

      if (mowPatternCurr != MOW_LANES)
      {
        imuDriveHeading = imu.ypr.yaw; // set drive heading
      }
    }
    setMotorPWMs(speed[LEFT], speed[RIGHT], true);
  }
}

// motor mow speed controller (slowly adjusts output speed to given input speed)
// input: motorMowEnable, motorMowModulate, motorMowRpmCurr
// output: motorMowPWMCurr
void Robot::motorMowControl()
{
  unsigned long curMillis = millis();
  if (curMillis < cutter.motor.nextTimeControl)
  {
    return;
  }
  cutter.motor.nextTimeControl = curMillis + 100;

  if (cutter.enableOverride)
  {
    cutter.enable = false;
  }

  if (!cutter.enable)
  {
    cutter.motor.pid.w;
    setMotorMowPWM(0, false);
  }
  else
  {
    // Need speed sensor to be able to regulate speed
    if (cutter.motor.regulate)
    {
      cutter.motor.pid.w = cutter.motor.rpmSet;
      float newPwm = cutter.motor.pid.compute(cutter.motor.rpmMeas);

      setMotorMowPWM(newPwm, false);
    }
    else
    {
      if (errorCounter[ERR_MOW_SENSE] == 0 && errorCounter[ERR_STUCK] == 0)
      {
        // no speed sensor available
        setMotorMowPWM(cutter.motor.pwmSet, true);
      }
    }
  }
}

void Robot::resetIdleTime()
{
  if (idleTimeSec == BATTERY_SW_OFF)
  { // battery switched off?
    Console.println(F("BATTERY switching ON again"));
    setActuator(ACT_BATTERY_SW, 1); // switch on battery again (if connected via USB)
  }
  idleTimeSec = 0;
}

void Robot::beep(int numberOfBeeps, boolean shortbeep = false)
{
  int delayTime1;
  int delayTime2;

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

  for (int i = 0; i < numberOfBeeps; i++)
  {
    setActuator(ACT_BUZZER, 4200);
    delay(delayTime1);
    setActuator(ACT_BUZZER, 0);
    delay(delayTime2);
  }
}

// set user-defined switches
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
  setMotorPWMs(0, 0, false);
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
    setNextState(STATE_FORWARD, 0);
  }

  stateStartTime = millis();
  beep(1);

  Console.println(F("START"));
  Console.print(F("Ardumower "));
  Console.println(VER);
#ifdef USE_DEVELOPER_TEST
  Console.println("Warning: USE_DEVELOPER_TEST activated");
#endif
  Console.print(F("Config: "));
  Console.println(name);
  Console.println(F("press..."));
  Console.println(F("  d for menu"));
  Console.println(F("  v to change console output @"
                    "(sensor counters, values, perimeter etc.)"));
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
  Console.print(odometer.x);
  Console.print(", ");
  Console.println(odometer.y);
}

void Robot::printInfo(Stream &s)
{
  if (consoleMode != CONSOLE_OFF)
  {
    Streamprint(s, "t%6u ", (millis() - stateStartTime) / 1000);
    Streamprint(s, "l%3u ", loopsPerSec);
    Streamprint(s, "r%4u ", freeRam());
    Streamprint(s, "v%1d ", consoleMode);
    Streamprint(s, "%4s ", stateNames[stateCurr]);

    if (consoleMode == CONSOLE_PERIMETER)
    {
      Streamprint(s, "sig min %4d max %4d avg %4d mag %5d qty %3d",
                  (int)perimeter.getSignalMin(0),
                  (int)perimeter.getSignalMax(0),
                  (int)perimeter.getSignalAvg(0), perimeterMag,
                  (int)(perimeter.getFilterQuality(0) * 100.0));
      Streamprint(s, "  in %2d  cnt %4d  on %1d\r\n",
                  (int)perimeterInside,
                  perimeterCounter,
                  (int)(!perimeter.signalTimedOut(0)));
    }

    else
    {
      if (odometer.use)
      {
        Streamprint(s, "odo %4d %4d ",
                    odometer.encoder[Odometer::LEFT].counter,
                    odometer.encoder[Odometer::RIGHT].counter);
      }
      Streamprint(s, "spd %4d %4d %4d ",
                  wheels.wheel[Wheel::LEFT].motor.rpmSet,
                  wheels.wheel[Wheel::RIGHT].motor.rpmSet,
                  cutter.motor.rpmMeas);

      if (consoleMode == CONSOLE_SENSOR_VALUES)
      {
        // sensor values
        Streamprint(s, "sen %4d %4d %4d ",
                    (int)wheels.wheel[Wheel::LEFT].motor.powerMeas,
                    (int)wheels.wheel[Wheel::RIGHT].motor.powerMeas,
                    (int)cutter.motor.powerMeas);
        Streamprint(s, "bum %4d %4d ",
                    bumpers.bumper[Bumpers::LEFT].isHit(),
                    bumpers.bumper[Bumpers::RIGHT].isHit());
        Streamprint(s, "dro %4d %4d ",
                    dropSensors.dropSensor[DropSensors::LEFT].isDetected(),
                    dropSensors.dropSensor[DropSensors::RIGHT].isDetected());
        Streamprint(s, "son %4u %4u %4u ",
                    sonars.sonar[Sonars::LEFT].distance,
                    sonars.sonar[Sonars::CENTER].distance,
                    sonars.sonar[Sonars::RIGHT].distance);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));

        if (perimeterUse)
        {
          Streamprint(s, "per %3d ", (int)perimeterInside);
        }

        if (lawnSensor.use)
        {
          Streamprint(s, "lawn %3d %3d ",
                      (int)lawnSensor.getValue(LawnSensor::FRONT),
                      (int)lawnSensor.getValue(LawnSensor::BACK));
        }
      }
      else
      {
        // sensor counters
        Streamprint(s, "sen %4d %4d %4d ",
                    wheels.wheel[Wheel::LEFT].motor.overloadCounter,
                    wheels.wheel[Wheel::RIGHT].motor.overloadCounter,
                    cutter.motor.overloadCounter);
        Streamprint(s, "bum %4d %4d ",
                    bumpers.bumper[Bumpers::LEFT].counter,
                    bumpers.bumper[Bumpers::RIGHT].counter);
        Streamprint(s, "dro %4d %4d ",
                    dropSensors.dropSensor[DropSensors::LEFT].counter,
                    dropSensors.dropSensor[DropSensors::RIGHT].counter);
        Streamprint(s, "son %3d ", sonars.distanceCounter);
        Streamprint(s, "yaw %3d ", (int)(imu.ypr.yaw / PI * 180.0));
        Streamprint(s, "pit %3d ", (int)(imu.ypr.pitch / PI * 180.0));
        Streamprint(s, "rol %3d ", (int)(imu.ypr.roll / PI * 180.0));
        //Streamprint(s, "per %3d ", perimeterLeft);

        if (perimeterUse)
        {
          Streamprint(s, "per %3d ", perimeterCounter);
        }

        if (lawnSensor.use)
        {
          Streamprint(s, "lawn %3d ", lawnSensor.getCounter());
        }

        if (gpsUse)
        {
          Streamprint(s, "gps %2d ", (int)gps.satellites());
        }
      }
      Streamprint(s, "bat %2d.%01d ",
                  (int)batVoltage,
                  (int)((batVoltage * 10) - ((int)batVoltage * 10)));
      Streamprint(s, "chg %2d.%01d %2d.%01d ",
                  (int)chgVoltage,
                  (int)((chgVoltage * 10) - ((int)chgVoltage * 10)),
                  (int)chgCurrent,
                  (int)((abs(chgCurrent) *10) - ((int)abs(chgCurrent)*10)));
      Streamprint(s, "imu%3d ", imu.getCallCounter());
      Streamprint(s, "adc%3d ", ADCMan.getCapturedChannels());
      Streamprint(s, "%s\r\n", name.c_str());
    }
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
  Console.println(F("x = Read settings"));
  Console.println(F("e = Delete all errors"));
  Console.println(F("0 = Exit"));
  Console.println();
}

void Robot::delayInfo(int ms)
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
  char ch;
  int lastLeft = 0;
  int lastRight = 0;
  int leftPwm = wheels.wheel[Wheel::LEFT].motor.pwmMax / 2;
  int rightPwm = wheels.wheel[Wheel::RIGHT].motor.pwmMax / 2;

  wheels.wheel[Wheel::LEFT].motor.pwmCur = leftPwm;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = rightPwm;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur, wheels.wheel[Wheel::RIGHT].motor.pwmCur, false);
  for (;;)
  {
    resetIdleTime();
    int odoCountLeft = odometer.encoder[Odometer::LEFT].counter;
    int odoCountRight = odometer.encoder[Odometer::RIGHT].counter;
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
      ch = (char) Console.read();
      if (ch == '0')
      {
        break;
      }
      if (ch == 'f')
      {
        wheels.wheel[Wheel::LEFT].motor.pwmCur = leftPwm;
        wheels.wheel[Wheel::RIGHT].motor.pwmCur = rightPwm;
        setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur, wheels.wheel[Wheel::RIGHT].motor.pwmCur, false);
      }
      if (ch == 'r')
      {
        wheels.wheel[Wheel::LEFT].motor.pwmCur = -leftPwm;
        wheels.wheel[Wheel::RIGHT].motor.pwmCur = -rightPwm;
        setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur, wheels.wheel[Wheel::RIGHT].motor.pwmCur, false);
      }
      if (ch == 'z')
      {
        odometer.encoder[Odometer::LEFT].counter = 0;
        odometer.encoder[Odometer::RIGHT].counter = 0;
      }
    }
  };
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur, wheels.wheel[Wheel::RIGHT].motor.pwmCur, false);
}

void Robot::testMotors()
{
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);

  Console.println(F("testing left motor (forward) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);

  Console.println(F("testing left motor (reverse) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = -wheels.wheel[Wheel::LEFT].motor.pwmMax;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);

  Console.println(F("testing right motor (forward) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);

  Console.println(F("testing right motor (reverse) full speed..."));
  delay(1000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = -wheels.wheel[Wheel::LEFT].motor.pwmMax;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);
  delayInfo(5000);
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);
}

void Robot::menu()
{
  char ch;
  printMenu();
  for (;;)
  {
    resetIdleTime();
    imu.update();
    if (Console.available() > 0)
    {
      ch = (char)Console.read();
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
          imu.calibAccNextAxis();
          break;
        case '6':
          imu.calibComStartStop();
          break;
        case '7':
          imu.deleteCalib();
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
          setNextState(STATE_OFF, 0);
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
      case 'd':
        menu(); // menu
        break;
      case 'v':
        consoleMode = (consoleMode + 1) % 4;
        Console.println(consoleModeNames[consoleMode]);
        break;
      case 'h':
        setNextState(STATE_PERI_FIND, 0); // press 'h' to drive home
        break;
      case 't':
        setNextState(STATE_PERI_TRACK, 0); // press 'p' to track perimeter
        break;
      case 'l':
        bumpers.bumper[Bumpers::LEFT].simHit();  // press 'l' to simulate left bumper
        break;
      case 'r':
        bumpers.bumper[Bumpers::RIGHT].simHit(); // press 'r' to simulate right bumper
        break;
      case 'j':
        dropSensors.dropSensor[DropSensors::LEFT].simDetected(); // press 'j' to simulate left drop                                                                         // Dropsensor - Absturzsensor
        break;
      case 'k':
        dropSensors.dropSensor[DropSensors::RIGHT].simDetected(); // press 'k' to simulate right drop                                                                        // Dropsensor - Absturzsensor
        break;
      case 's':
        lawnSensor.simDetected(); // press 's' to simulate lawn sensor
        break;
      case 'm':
        if (stateCurr == STATE_OFF || stateCurr == STATE_MANUAL)
        {
          cutter.enableOverride = false;
        }
        else
        {
          cutter.enableOverride = !cutter.enableOverride;
        }
        cutter.enable = !cutter.enable; // press 'm' to toggle mower motor
        break;
      case 'c':
        setNextState(STATE_STATION, 0); // press 'c' to simulate in station
        break;
      case 'a':
        setNextState(STATE_STATION_CHARGING, 0); // press 't' to simulate in station charging
        break;
      case '+':
        setNextState(STATE_ROLL_WAIT, 0); // press '+' to rotate 90 degrees clockwise (IMU)
        imuRollHeading = scalePI(imuRollHeading + PI / 2);
        break;
      case '-':
        setNextState(STATE_ROLL_WAIT, 0); // press '-' to rotate 90 degrees anti-clockwise (IMU)
        imuRollHeading = scalePI(imuRollHeading - PI / 2);
        break;
      case 'i':
        // press 'i' to toggle imu.use
        imu.use = !imu.use;
        break;
      case '3':
        setNextState(STATE_REMOTE, 0); // press '3' to activate model RC
        break;
      case '0':
        // press '0' for OFF
        setNextState(STATE_OFF, 0);
        break;
      case '1':
        // press '1' for Automode
        cutter.enable = true;
        //motorMowModulate = false;
        setNextState(STATE_FORWARD, 0);
        break;
    }
  }
}

void Robot::checkButton()
{
  unsigned long curMillis = millis();
  if (!button.use || curMillis < button.nextTimeCheck)
  {
    return;
  }

  button.nextTimeCheck = curMillis + 50;
  boolean buttonPressed = button.isPressed();
  if ((!buttonPressed && button.getCounter() > 0) ||
      (buttonPressed && curMillis >= button.nextTime))
  {
    button.nextTime = curMillis + 1000;
    if (buttonPressed)
    {
      Console.println(F("buttonPressed"));
      // ON/OFF button pressed
      beep(1);
      button.incCounter();
      resetIdleTime();
    }
    else
    {
      // ON/OFF button released
      if ((stateCurr != STATE_OFF || stateCurr == STATE_ERROR) &&
          (stateCurr != STATE_STATION))
      {
        setNextState(STATE_OFF, 0);
      }
      else
      {
        switch (button.getCounter())
        {
          case 1:
            /*if ((perimeterUse) && (!perimeter.isInside())){
             Console.println("start inside perimeter!");
             addErrorCounter(ERR_PERIMETER_TIMEOUT);
             setNextState(STATE_ERROR, 0);
             } else {*/
            // start normal with mowing
            cutter.enable = true;
            //motorMowModulate = true;
            mowPatternCurr = MOW_RANDOM;
            setNextState(STATE_FORWARD, 0);
            //}
            break;

          case 2:
            cutter.enable = true;
            mowPatternCurr = MOW_BIDIR;
            setNextState(STATE_FORWARD, 0);
            break;

          case 3:
            // start remote control mode
            setNextState(STATE_REMOTE, 0);
            break;

          case 4:
            // start normal without perimeter
            //motorMowEnable = false;
            perimeterUse = false;
            setNextState(STATE_FORWARD, 0);
            break;

          case 5:
            // drive home
            setNextState(STATE_PERI_FIND, 0);
            break;

          case 6:
            // track perimeter
            setNextState(STATE_PERI_TRACK, 0);
            break;

          case 7:
            // start normal with mowing in lanes
            cutter.enable = true;
            //motorMowModulate = true;
            mowPatternCurr = MOW_LANES;
            setNextState(STATE_FORWARD, 0);
            break;
        }
      }

      button.resetCounter();
    }
  }
}

void Robot::readSensors()
{
//NOTE: this function should only read in sensors into variables - it should NOT change any state!

  unsigned long curMillis = millis();
  if (curMillis >= cutter.motor.nextTimeReadSensor)
  {
    cutter.motor.nextTimeReadSensor = curMillis + 50;

    wheels.wheel[Wheel::LEFT].motor.readSensePin();
    wheels.wheel[Wheel::RIGHT].motor.readSensePin();
    cutter.motor.readSensePin();

    double accel = 0.05;
    wheels.wheel[Wheel::LEFT].motor.calcCurrent(accel);
    wheels.wheel[Wheel::RIGHT].motor.calcCurrent(accel);
    cutter.motor.calcCurrent(accel);

    float batV;
    if (batVoltage > 8)
    {
      // Use measured battery voltage
      batV = batVoltage;
    }
    else
    {
      // Use reference voltage for fully battery in absence of battery voltage
      // measurement
      batV = batFull;
    }

    // Conversion to power in Watts
    wheels.wheel[Wheel::RIGHT].motor.calcPower(batV);
    wheels.wheel[Wheel::LEFT].motor.calcPower(batV);
    cutter.motor.calcPower(batV);

    if ((curMillis - cutter.motor.lastRpmTime) >= 500)
    {
      cutter.motor.rpmMeas = readSensor(SEN_MOTOR_MOW_RPM);
      if ((cutter.motor.rpmMeas == 0) && (cutter.motor.rpmCounter != 0))
      {
        // rpm may be updated via interrupt
        cutter.motor.rpmMeas = (int)(((double)cutter.motor.rpmCounter
            / ((double)(curMillis - cutter.motor.lastRpmTime)))
                                 * 60000.0);
        cutter.motor.rpmCounter = 0;
      }
      cutter.motor.lastRpmTime = curMillis;
      if (!ADCMan.calibrationDataAvail())
      {
        Console.println(F("Error: missing ADC calibration data"));
        addErrorCounter(ERR_ADC_CALIB);
        setNextState(STATE_ERROR, 0);
      }
    }
  }

  if (perimeterUse && curMillis >= nextTimePerimeter)
  {
    nextTimePerimeter = curMillis + 50;
    perimeterMag = readSensor(SEN_PERIM_LEFT);
    if (perimeter.isInside(0) != perimeterInside)
    {
      perimeterCounter++;
      perimeterLastTransitionTime = millis();
      perimeterInside = perimeter.isInside(0);
    }
    if (perimeterMag < 0)
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
      if (curMillis > stateStartTime + 2000)
      { // far away from perimeter?
        perimeterTriggerTime = curMillis + perimeterTriggerTimeout;
      }
      else
      {
        perimeterTriggerTime = curMillis;
      }
    }
    if (perimeter.signalTimedOut(0))
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
        Console.println("Error: perimeter too far away");
        addErrorCounter(ERR_PERIMETER_TIMEOUT);
        setNextState(STATE_ERROR, 0);
      }
    }
  }

  if (lawnSensor.use && curMillis >= lawnSensor.nextTimeRead)
  {
    lawnSensor.nextTimeRead = curMillis + 100;
    lawnSensor.read();
  }

  if (lawnSensor.use && curMillis >= lawnSensor.nextTimeCheck)
  {
    lawnSensor.nextTimeCheck = curMillis + 2000;
    lawnSensor.check();
  }

  if (sonars.use && curMillis >= sonars.nextTime)
  {
    sonars.nextTime = curMillis + 250;

    sonars.ping();
  }

  if (bumpers.use && curMillis >= bumpers.nextTime)
  {
    bumpers.nextTime = curMillis + 100;
    bumpers.check();
  }

  if (dropSensors.use && curMillis >= dropSensors.nextTime)
  {
    dropSensors.nextTime = curMillis + 100;
    dropSensors.check();
  }

  //if ((timerUse) && (millis() >= nextTimeRTC)) {
  if (millis() >= nextTimeRTC)
  {
    nextTimeRTC = curMillis + 60000;
    readSensor(SEN_RTC);       // read RTC
    Console.print(F("RTC date received: "));
    Console.println(date2str(datetime.date));
  }

  if (imu.use && curMillis >= imu.nextTime)
  {
    // IMU
    readSensor(SEN_IMU);
    imu.nextTime = curMillis + 200;   // 5 hz
    if (imu.getErrorCounter() > 0)
    {
      addErrorCounter(ERR_IMU_COMM);
      Console.println(F("IMU comm error"));
    }
    if (!imu.calibrationAvail)
    {
      Console.println(F("Error: missing IMU calibration data"));
      addErrorCounter(ERR_IMU_CALIB);
      setNextState(STATE_ERROR, 0);
    }
  }

  if (curMillis >= nextTimeBattery)
  {
    // read battery
    nextTimeBattery = curMillis + 100;
    if (abs(chgCurrent) > 0.04 && chgVoltage > 5)
    {
      // charging
      batCapacity += (chgCurrent / 36.0);
    }
    // convert to double
    batADC = readSensor(SEN_BAT_VOLTAGE);
    double batvolt = (double) batADC * batFactor / 10; // / 10 due to arduremote bug, can be removed after fixing
    //double chgvolt = ((double)((int)(readSensor(SEN_CHG_VOLTAGE) / 10))) / 10.0;
    int chgADC = readSensor(SEN_CHG_VOLTAGE);
    //Console.println(chgADC);
    double chgvolt = (double) chgADC * batChgFactor / 10; // / 10 due to arduremote bug, can be removed after fixing
    double current = ((double) ((int) (readSensor(SEN_CHG_CURRENT))));
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
    if (abs(chgVoltage-chgvolt) > 5)
    {
      chgVoltage = chgvolt;
    }
    else
    {
      chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
    }
    // if (abs(chgCurrent-current)>0.4) chgCurrent = current; else chgCurrent = (1.0-accel) * chgCurrent + accel * current;  //Deaktiviert für Ladestromsensor berechnung

    // Anfang Ladestromsensor zur Glättung und Mittelwertbildung
    // ********************************************************************
    //  Variabeln
    double currentmitte = current;
    // ********************************************************************
    // Ende Ladestromsensor zur Glättung und Mittelwertbildung

    //  Anfang Ladestromsensor berechnen
    // ********************************************************************
    //  Variabeln
    float vcc, asensor, amp;
    float chgAMP;                                  //Sensorwert des Ladestrompin

    //Sensor Wert Ausgabe auf Seriellen Monitor oder HandyApp   wenn chgSelection =0
    if (chgSelection == 0)
    {
      chgCurrent = current;
    }

    // Berechnung für Ladestromsensor ACS712 5A                 wenn chgSelection =1
    if (chgSelection == 1)
    {
      chgAMP = currentmitte;              //Sensorwert übergabe vom Ladestrompin
      vcc = (float) 3.30 / chgSenseZero * 1023.0; // Versorgungsspannung ermitteln!  chgSenseZero=511  ->Die Genauigkeit kann erhöt werden wenn der 3.3V Pin an ein Analogen Pin eingelesen wird. Dann ist vcc = (float) 3.30 / analogRead(X) * 1023.0;
      asensor = (float) chgAMP * vcc / 1023.0;              // Messwert auslesen
      asensor = (float) asensor - (vcc / chgNull); // Nulldurchgang (vcc/2) abziehen
      chgSense = (float) chgSense - ((5.00 - vcc) * chgFactor); // Korrekturfactor für Vcc!  chgFactor=39
      amp = (float) asensor / chgSense * 1000;               // Ampere berechnen
      if (chgChange == 1)
      {
        amp = amp / -1;                 //Lade Strom Messwertumkehr von - nach +
      }
      if (amp < 0.0)
      {
        chgCurrent = 0;
      }
      else
      {
        chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 anssonsten messwertau8sgabe in Ampere)
      }
    }

    // Berechnung für Ladestromsensor INA169 board              wenn chgSelection =2
    if (chgSelection == 2)
    {
      chgAMP = currentmitte;
      asensor = (chgAMP * 5) / 1023; // umrechnen von messwert in Spannung (5V Reference)
      amp = asensor / (10 * 0.1); // Ampere berechnen RL = 10k    Is = (Vout x 1k) / (RS x RL)
      if (amp < 0.0)
      {
        chgCurrent = 0;
      }
      else
      {
        chgCurrent = amp; // Messwertrückgabe in chgCurrent   (Wenn Messwert kleiner als 0 dann Messwert =0 ansonsten Messwertaußsgabe in Ampere)
      }
    }

    //  Ladestromsensor berechnen ********** Ende
    // ********************************************************************

    //batVoltage = batVolt
    //chgVoltage = chgvolt;
    //chgCurrent = current;
  }

  if (rainSensor.use && curMillis >= rainSensor.nextTime)
  {
    // read rain sensor
    rainSensor.nextTime = curMillis + 5000;
    rainSensor.check();
  }
}

void Robot::setDefaults()
{
  wheels.wheel[Wheel::LEFT].motor.rpmSet = 0;
  wheels.wheel[Wheel::RIGHT].motor.rpmSet = 0;
  cutter.enable = false;
}

// set state machine new state
// http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
// called *ONCE* to set to a *NEW* state
void Robot::setNextState(byte stateNew, byte dir)
{
  unsigned long curMillis = millis();
  unsigned long stateTime = curMillis - stateStartTime;
  if (stateNew == stateCurr)
  {
    return;
  }

  // state correction
  if (stateCurr == STATE_PERI_FIND || stateCurr == STATE_PERI_TRACK)
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
    if (stateCurr == STATE_STATION || stateCurr == STATE_STATION_CHARGING)
    {
      stateNew = STATE_STATION_CHECK;
      setActuator(ACT_CHGRELAY, 0);
      cutter.enable = false;
    }
  }
  // evaluate new state
  stateNext = stateNew;
  rollDir = dir;
  if (stateNew == STATE_STATION_REV)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmMax;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmMax;
    stateEndTime = curMillis + stationRevTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_ROLL)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmMax;
    stateEndTime = curMillis + stationRollTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_FORW)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax;
    cutter.enable = true;
    stateEndTime = curMillis + stationForwTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_CHECK)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmMax / 2;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmMax / 2;
    stateEndTime = curMillis + stationCheckTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_ROLL)
  {
    stateEndTime = curMillis + perimeterTrackRollTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
    if (dir == RIGHT)
    {
      wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax / 2;
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmSet;
    }
    else
    {
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax / 2;
      wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmSet;
    }
  }
  if (stateNew == STATE_PERI_REV)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmMax / 2;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmMax / 2;
    stateEndTime = curMillis + perimeterTrackRevTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_OUT_FORW)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax;
    stateEndTime = curMillis + perimeterOutRevTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime + 1000;
  }
  else if (stateNew == STATE_PERI_OUT_REV)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet =  -wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmMax / 1.25;
    stateEndTime = curMillis + perimeterOutRevTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_OUT_ROLL)
  {
    stateEndTime = curMillis
        + random(perimeterOutRollTimeMin, perimeterOutRollTimeMax)
        + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
    if (dir == RIGHT)
    {
      wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmSet;
    }
    else
    {
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax / 1.25;
      wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmSet;
    }
  }
  else if (stateNew == STATE_FORWARD)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax;
    statsMowTimeTotalStart = true;
  }
  else if (stateNew == STATE_REVERSE)
  {
    wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmMax / 1.25;
    stateEndTime = curMillis + wheels.reverseTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
  }
  else if (stateNew == STATE_ROLL)
  {
    imuDriveHeading = scalePI(imuDriveHeading + PI); // toggle heading 180 degree (IMU)
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
    stateEndTime = curMillis + random(wheels.rollTimeMin, wheels.rollTimeMax)
                   + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime;
    if (dir == RIGHT)
    {
      wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmSet;
    }
    else
    {
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax / 1.25;
      wheels.wheel[Wheel::LEFT].motor.rpmSet = -wheels.wheel[Wheel::RIGHT].motor.rpmSet;
    }
  }
  if (stateNew == STATE_REMOTE)
  {
    cutter.enable = true;
    //motorMowModulate = false;
  }
  if (stateNew == STATE_STATION)
  {
    setMotorPWMs(0, 0, false);
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();
    statsMowTimeTotalStart = false;  // stop stats mowTime counter
    saveRobotStats();
  }
  if (stateNew == STATE_STATION_CHARGING)
  {
    setActuator(ACT_CHGRELAY, 1);
    setDefaults();
  }
  if (stateNew == STATE_OFF)
  {
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();
    statsMowTimeTotalStart = false; // stop stats mowTime counter
    saveRobotStats();
  }
  if (stateNew == STATE_ERROR)
  {
    cutter.enable = false;
    wheels.wheel[Wheel::LEFT].motor.rpmSet = 0;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = 0;
    setActuator(ACT_CHGRELAY, 0);
    statsMowTimeTotalStart = false;
    //saveRobotStats();
  }
  if (stateNew == STATE_PERI_FIND)
  {
    // find perimeter  => drive half speed
    wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.5;
    wheels.wheel[Wheel::RIGHT].motor.rpmSet = wheels.wheel[Wheel::RIGHT].motor.rpmMax / 1.5;
    //motorMowEnable = false;     // FIXME: should be an option?
  }
  if (stateNew == STATE_PERI_TRACK)
  {
    //motorMowEnable = false;     // FIXME: should be an option?
    setActuator(ACT_CHGRELAY, 0);
    //beep(6);
  }
  if (stateNew != STATE_REMOTE)
  {
    cutter.motor.pwmSet = cutter.motor.pwmMax;
  }

  sonars.obstacleTimeout = 0;
  // state has changed
  stateStartTime = curMillis;
  stateLast = stateCurr;
  stateCurr = stateNext;
  perimeterTriggerTime = 0;
  printInfo(Console);
}

// check battery voltage and decide what to do
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
      Console.println(F("triggered batSwitchOffIfBelow"));
      addErrorCounter(ERR_BATTERY);
      beep(2, true);
      setNextState(STATE_OFF, 0);
    }
    else if (batVoltage < batGoHomeIfBelow &&
             stateCurr != STATE_OFF &&
             stateCurr != STATE_MANUAL &&
             stateCurr != STATE_STATION &&
             stateCurr != STATE_STATION_CHARGING &&
             stateCurr != STATE_REMOTE &&
             stateCurr != STATE_ERROR &&
             stateCurr != STATE_PERI_TRACK &&
             perimeterUse)
    {    //UNTESTED please verify
      Console.println(F("triggered batGoHomeIfBelow"));
      beep(2, true);
      setNextState(STATE_PERI_FIND, 0);
    }
  }
  // check if idle and robot battery can be switched off
  if (stateCurr == STATE_OFF || stateCurr == STATE_ERROR)
  {
    if (idleTimeSec != BATTERY_SW_OFF)
    { // battery already switched off?
      idleTimeSec++; // add one second idle time
      if (idleTimeSec > batSwitchOffIfIdle * 60)
      {
        Console.println(F("triggered batSwitchOffIfIdle"));
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
  if (gpsUse)
  {
    unsigned long chars = 0;
    unsigned short good_sentences = 0;
    unsigned short failed_cs = 0;
    gps.stats(&chars, &good_sentences, &failed_cs);
    if (good_sentences == 0)
    {
      // no GPS sentences received so far
      Console.println(F("GPS communication error!"));
      addErrorCounter(ERR_GPS_COMM);
      // next line commented out as GPS communication may not be available if GPS signal is poor
      //setNextState(STATE_ERROR, 0);
    }
    Console.print(F("GPS sentences: "));
    Console.println(good_sentences);
    Console.print(F("GPS satellites in view: "));
    Console.println(gps.satellites());
    if (gps.satellites() == 255)
    {
      // no GPS satellites received so far
      addErrorCounter(ERR_GPS_DATA);
    }
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long age;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second,
                       &hundredths, &age);
    if (age != GPS::GPS_INVALID_AGE)
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
}

// check robot stats
void Robot::checkRobotStats()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeRobotStats)
  {
    return;
  }
  nextTimeRobotStats = curMillis + 60000;

  //----------------stats mow time----------------------------------------------
  statsMowTimeHoursTotal = double(statsMowTimeMinutesTotal) / 60;
  if (statsMowTimeTotalStart)
  {
    statsMowTimeMinutesTripCounter++;
    statsMowTimeMinutesTrip = statsMowTimeMinutesTripCounter;
    statsMowTimeMinutesTotal++;
  }
  else if (statsMowTimeMinutesTripCounter != 0)
  {
    statsMowTimeMinutesTripCounter = 0;
  }

  //---------------stats Battery------------------------------------------------
  if (stateCurr == STATE_STATION_CHARGING && stateTime >= 60000)
  { // count only if mower is charged longer then 60sec
    statsBatteryChargingCounter++; // temporary counter
    if (statsBatteryChargingCounter == 1)
    {
      statsBatteryChargingCounterTotal++;
    }
    statsBatteryChargingCapacityTrip = batCapacity;
    statsBatteryChargingCapacityTotal += (batCapacity - lastTimeBatCapacity); // summ up only the difference between actual batCapacity and last batCapacity
    lastTimeBatCapacity = batCapacity;
  }
  else
  {                         // resets values to 0 when mower is not charging
    statsBatteryChargingCounter = 0;
    batCapacity = 0;
  }

  if (isnan(statsBatteryChargingCapacityTrip))
  {
    statsBatteryChargingCapacityTrip = 0;
  }
  if (isnan(statsBatteryChargingCounterTotal))
  {
    statsBatteryChargingCounterTotal = 0; // for first run ensures that the counter is 0
  }
  if (isnan(statsBatteryChargingCapacityTotal))
  {
    statsBatteryChargingCapacityTotal = 0; // for first run ensures that the counter is 0
  }
  if (statsBatteryChargingCapacityTotal <= 0 ||
      statsBatteryChargingCounterTotal == 0)
  {
    statsBatteryChargingCapacityAverage = 0; // make sure that there is no dividing by zero
  }
  else
  {
    statsBatteryChargingCapacityAverage =
        statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;
  }

  //----------------new stats goes here------------------------------------------------------
}

// check timer
void Robot::checkTimer()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeTimer)
  {
    return;
  }
  nextTimeTimer = curMillis + 60000;
  srand(time2minutes(datetime.time)); // initializes the pseudo-random number generator for c++ rand()
  randomSeed(time2minutes(datetime.time)); // initializes the pseudo-random number generator for arduino random()
  receiveGPSTime();
  boolean stopTimerTriggered = true;
  if (timerUse)
  {
    for (int i = 0; i < MAX_TIMERS; i++)
    {
      if (timer[i].active)
      {
        if ((timer[i].daysOfWeek & (1 << datetime.date.dayOfWeek)) != 0)
        {
          int startmin = time2minutes(timer[i].startTime);
          int stopmin = time2minutes(timer[i].stopTime);
          int currmin = time2minutes(datetime.time);
          if (currmin >= startmin && currmin < stopmin)
          {
            // start timer triggered
            stopTimerTriggered = false;
            if (stateCurr == STATE_STATION || stateCurr == STATE_OFF)
            {
              Console.println(F("timer start triggered"));
              cutter.enable = true;
              setNextState(STATE_FORWARD, 0);
            }
          }
        }
        if (stopTimerTriggered && timer[i].active)
        {
          if (stateCurr == STATE_FORWARD)
          {
            Console.println(F("timer stop triggered"));
            if (perimeterUse)
            {
              setNextState(STATE_PERI_FIND, 0);
            }
            else
            {
              setNextState(STATE_OFF, 0);
            }
          }
        }
      }
    }
  }
}

void Robot::reverseOrBidir(byte aRollDir)
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

// check motor power
void Robot::checkPower()
{
  unsigned long curMillis = millis();
  if (curMillis < cutter.motor.nextTimeCheckPower)
  {
    return;
  }
  cutter.motor.nextTimeCheckPower = curMillis + 100;

  if (cutter.motor.powerMeas >= cutter.motor.powerMax)
  {
    cutter.motor.overloadCounter++;
  }
  else
  {
    errorCounterMax[ERR_MOW_SENSE] = 0;
    cutter.motor.overloadCounter = 0;
    if (curMillis >= cutter.motor.lastTimeStucked + 30000)
    { // wait 30 seconds before switching on again
      errorCounter[ERR_MOW_SENSE] = 0;
      cutter.enable = true;
    }
  }

  if (cutter.motor.overloadCounter >= 30)
  { //ignore motorMowPower for 3 seconds
    cutter.enable = false;
    Console.println("Error: Motor mow current");
    addErrorCounter(ERR_MOW_SENSE);
    cutter.motor.lastTimeStucked = curMillis;
    // if (rollDir == RIGHT) reverseOrBidir(LEFT); // toggle roll dir
    //else reverseOrBidir(RIGHT);
  }

  if (wheels.wheel[Wheel::LEFT].motor.powerMeas >= wheels.wheel[Wheel::LEFT].motor.powerMax)
  {
    // left wheel motor overpowered
    if ((stateCurr == STATE_FORWARD ||
        stateCurr == STATE_PERI_FIND ||
        stateCurr == STATE_PERI_TRACK) &&
        curMillis > (stateStartTime + wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime))
    {
      //beep(1);
      wheels.wheel[Wheel::LEFT].motor.overloadCounter++;
      setMotorPWMs(0, 0, false);
      reverseOrBidir(RIGHT);
    }
    else if (stateCurr == STATE_REVERSE &&
             curMillis > (stateStartTime + wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::LEFT].motor.overloadCounter++;
      setMotorPWMs(0, 0, false);
      //   reverseOrBidir(RIGHT);
      setNextState(STATE_ROLL, RIGHT);
    }
    else if (stateCurr == STATE_ROLL &&
             curMillis > (stateStartTime + wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::LEFT].motor.overloadCounter++;
      setMotorPWMs(0, 0, false);
      setNextState(STATE_FORWARD, 0);
    }
  }
  else if (wheels.wheel[Wheel::RIGHT].motor.powerMeas >= wheels.wheel[Wheel::RIGHT].motor.powerMax)
  {
    // right wheel motor overpowered
    if ((stateCurr == STATE_FORWARD || stateCurr == STATE_PERI_FIND) &&
        curMillis > (stateStartTime + wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime))
    {
      //beep(1);
      wheels.wheel[Wheel::RIGHT].motor.overloadCounter++;
      setMotorPWMs(0, 0, false);
      reverseOrBidir(RIGHT);
    }
    else if (stateCurr == STATE_REVERSE &&
             curMillis > (stateStartTime + wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::RIGHT].motor.overloadCounter++;
      setMotorPWMs(0, 0, false);
      setNextState(STATE_ROLL, LEFT);
    }
    else if (stateCurr == STATE_ROLL &&
             curMillis > (stateStartTime + wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::RIGHT].motor.overloadCounter++;
      setMotorPWMs(0, 0, false);
      setNextState(STATE_FORWARD, 0);
    }
  }
}

// check bumpers
void Robot::checkBumpers()
{
  unsigned long curMillis = millis();
  if (mowPatternCurr == MOW_BIDIR &&
      curMillis < (stateStartTime + 4000))
  {
    return;
  }

  if (bumpers.bumper[Bumpers::LEFT].isHit())
  {
    reverseOrBidir(RIGHT);
  }
  if (bumpers.bumper[Bumpers::RIGHT].isHit())
  {
    reverseOrBidir(LEFT);
  }
}

// check drop
void Robot::checkDrop()
{
  unsigned long curMillis = millis();
  if (mowPatternCurr == MOW_BIDIR &&
      curMillis < (stateStartTime + 4000))
  {
    return;
  }

  if (dropSensors.dropSensor[DropSensors::LEFT].isDetected())
  {
    reverseOrBidir(RIGHT);
  }
  if (dropSensors.dropSensor[DropSensors::RIGHT].isDetected())
  {
    reverseOrBidir(LEFT);
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
  if (bumpers.bumper[Bumpers::LEFT].isHit() ||
      bumpers.bumper[Bumpers::RIGHT].isHit())
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

// check perimeter as a boundary
void Robot::checkPerimeterBoundary()
{
  unsigned long curMillis = millis();
  if (curMillis >= wheels.nextTimeRotationChange)
  {
    wheels.nextTimeRotationChange = curMillis + 60000;
    if (wheels.rotateDir == Wheel::LEFT)
    {
      wheels.rotateDir == Wheel::RIGHT;
    }
    else
    {
      wheels.rotateDir == Wheel::LEFT;
    }
  }

  if (mowPatternCurr == MOW_BIDIR)
  {
    if (curMillis < stateStartTime + 3000)
    {
      return;
    }
    if (!perimeterInside)
    {
      if (rand() % 2 == 0)
      {
        reverseOrBidir(LEFT);
      }
      else
      {
        reverseOrBidir(RIGHT);
      }
    }
  }
  else
  {
    if (stateCurr == STATE_FORWARD)
    {
      if (perimeterTriggerTime != 0)
      {
        if (curMillis >= perimeterTriggerTime)
        {
          perimeterTriggerTime = 0;
          //if ((rand() % 2) == 0){
          if (wheels.rotateDir == Wheel::LEFT)
          {
            setNextState(STATE_PERI_OUT_REV, LEFT);
          }
          else
          {
            setNextState(STATE_PERI_OUT_REV, RIGHT);
          }
        }
      }
    }
    else if (stateCurr == STATE_ROLL)
    {
      if (perimeterTriggerTime != 0)
      {
        if (curMillis >= perimeterTriggerTime)
        {
          perimeterTriggerTime = 0;
          setMotorPWMs(0, 0, false);
          //if ((rand() % 2) == 0){
          if (wheels.rotateDir == Wheel::LEFT)
          {
            setNextState(STATE_PERI_OUT_FORW, LEFT);
          }
          else
          {
            setNextState(STATE_PERI_OUT_FORW, RIGHT);
          }
        }
      }
    }
  }
}

// check perimeter while finding it
void Robot::checkPerimeterFind()
{
  if (stateCurr == STATE_PERI_FIND)
  {
    if (perimeter.isInside(0))
    {
      // inside
      if (wheels.wheel[Wheel::LEFT].motor.rpmSet != wheels.wheel[Wheel::RIGHT].motor.rpmSet)
      {
        // we just made an 'outside=>inside' rotation, now track
        setNextState(STATE_PERI_TRACK, 0);
      }
    }
    else
    {
      // we are outside, now roll to get inside
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = -wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.5;
      wheels.wheel[Wheel::LEFT].motor.rpmSet = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.5;
    }
  }
}

// check lawn
void Robot::checkLawn()
{
  if (!lawnSensor.use)
  {
    return;
  }

  if (lawnSensor.isDetected() && millis() > stateStartTime + 3000)
  {
    if (rollDir == RIGHT)
    {
      reverseOrBidir(LEFT); // toggle roll dir
    }
    else
    {
      reverseOrBidir(RIGHT);
    }
  }
  else
  {
    lawnSensor.clearDetected();
  }
}

void Robot::checkRain()
{
  if (!rainSensor.use)
  {
    return;
  }

  if (rainSensor.raining)
  {
    Console.println(F("RAIN"));
    if (perimeterUse)
    {
      setNextState(STATE_PERI_FIND, 0);
    }
    else
    {
      setNextState(STATE_OFF, 0);
    }
  }
}

// check sonar
void Robot::checkSonar()
{
  if (!sonars.use)
  {
    return;
  }

  unsigned long curMillis = millis();
  if (curMillis < sonars.nextTimeCheck)
  {
    return;
  }
  sonars.nextTimeCheck = curMillis + 200;

  if (mowPatternCurr == MOW_BIDIR && curMillis < (stateStartTime + 4000))
  {
    return;
  }

  // slow down motor wheel speed near obstacles
  if (stateCurr == STATE_FORWARD ||
      (mowPatternCurr == MOW_BIDIR &&
       (stateCurr == STATE_FORWARD || stateCurr == STATE_REVERSE)))
  {
    if (sonars.obstacleTimeout == 0)
    {
      bool isClose = false;
      for (uint8_t i = 0; i < Sonars::END; i++)
      {
        if (sonars.sonar[i].distance > 0 &&
            sonars.sonar[i].distance < (sonars.triggerBelow << 1))
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
          wheels.wheel[Wheel::LEFT].motor.rpmSet /= 1.5;
          wheels.wheel[Wheel::RIGHT].motor.rpmSet /= 1.5;
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
      wheels.wheel[Wheel::LEFT].motor.rpmSet *= 1.5;
      wheels.wheel[Wheel::RIGHT].motor.rpmSet *= 1.5;
    }
  }

  if (sonars.sonar[Sonars::CENTER].distance > 0 &&
      sonars.sonar[Sonars::CENTER].distance < sonars.triggerBelow)
  {
    sonars.distanceCounter++;
    if (rollDir == RIGHT)
    {
      reverseOrBidir(LEFT); // toggle roll dir
    }
    else
    {
      reverseOrBidir(RIGHT);
    }
  }

  if (sonars.sonar[Sonars::RIGHT].distance > 0 &&
      sonars.sonar[Sonars::RIGHT].distance < sonars.triggerBelow)
  {
    sonars.distanceCounter++;
    reverseOrBidir(LEFT);
  }

  if (sonars.sonar[Sonars::LEFT].distance > 0 &&
      sonars.sonar[Sonars::LEFT].distance < sonars.triggerBelow)
  {
    sonars.distanceCounter++;
    reverseOrBidir(RIGHT);
  }
}

// check IMU (tilt)
void Robot::checkTilt()
{
  if (!imu.use)
  {
    return;
  }

  unsigned long curMillis = millis();
  if (curMillis < nextTimeCheckTilt)
  {
    return;
  }
  nextTimeCheckTilt = curMillis + 200; // 5Hz same as imu.nextTime

  int pitchAngle = (imu.ypr.pitch / PI * 180.0);
  int rollAngle = (imu.ypr.roll / PI * 180.0);
  if (stateCurr != STATE_OFF &&
      stateCurr != STATE_ERROR &&
      stateCurr != STATE_STATION)
  {
    if (abs(pitchAngle) > 40 || abs(rollAngle) > 40)
    {
      Console.println(F("Error: IMU tilt"));
      addErrorCounter(ERR_IMU_TILT);
      setNextState(STATE_ERROR, 0);
    }
  }
  if (stateCurr == STATE_ERROR)
  {
    //if ( (abs(pitchAngle) < 40) && (abs(rollAngle) < 40) ) setNextState(STATE_FORWARD,0);
  }
}

// check if mower is stucked
// ToDo: take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStucked()
{
  unsigned long curMillis = millis();
  if (curMillis < nextTimeCheckIfStucked)
  {
    return;
  }
  nextTimeCheckIfStucked = curMillis + 300;

  if (gpsUse && gps.hdop() < 500)
  {
    //float gpsSpeedRead = gps.f_speed_kmph();
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
        gpsSpeed <= stuckedIfGpsSpeedBelow &&
        odometer.encoder[LEFT].wheelRpmCurr != 0 &&
        odometer.encoder[RIGHT].wheelRpmCurr != 0 &&
        millis() > stateStartTime + gpsSpeedIgnoreTime)
    {
      robotIsStuckedCounter++;
    }

    else
    { // if mower gets unstuck it resets errorCounterMax to zero and re-enables motorMow
      robotIsStuckedCounter = 0;    // resets temporary counter to zero
      if (errorCounter[ERR_STUCK] == 0
          && (stateCurr != STATE_OFF)
          && (stateCurr != STATE_MANUAL)
          && (stateCurr != STATE_STATION)
          && (stateCurr != STATE_STATION_CHARGING)
          && (stateCurr != STATE_STATION_CHECK)
          && (stateCurr != STATE_STATION_REV)
          && (stateCurr != STATE_STATION_ROLL)
          && (stateCurr != STATE_REMOTE)
          && (stateCurr != STATE_ERROR))
      {
        cutter.enable = true;
        errorCounterMax[ERR_STUCK] = 0;
      }
      return;
    }

    if (robotIsStuckedCounter >= 5)
    {
      cutter.enable = false;
      if (errorCounterMax[ERR_STUCK] >= 3)
      {   // robot is definately stucked and unable to move
        Console.println(F("Error: Mower is stucked"));
        addErrorCounter(ERR_STUCK);
        setNextState(STATE_ERROR, 0);    //mower is switched into ERROR
        //robotIsStuckedCounter = 0;
      }
      else if (errorCounter[ERR_STUCK] < 3)
      {   // mower tries 3 times to get unstucked
        if (stateCurr == STATE_FORWARD)
        {
          cutter.enable = false;
          addErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0, false);
          reverseOrBidir(RIGHT);
        }
        else if (stateCurr == STATE_ROLL)
        {
          cutter.enable = false;
          addErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0, false);
          setNextState(STATE_FORWARD, 0);
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

  float nlat, nlon;
  unsigned long age;
  gps.f_get_position(&nlat, &nlon, &age);

  if (nlat == GPS::GPS_INVALID_F_ANGLE)
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
    // timeout
    if (rollDir == RIGHT)
    {
      setNextState(STATE_REVERSE, LEFT); // toggle roll dir
    }
    else
    {
      setNextState(STATE_REVERSE, RIGHT);
    }
  }
}

void Robot::loop()
{
  unsigned long curMillis = millis();
  stateTime = curMillis - stateStartTime;
  int steer;
  ADCMan.run();
  readSerial();
  if (rc.readSerial())
  {
    resetIdleTime();
  }
  readSensors();
  checkBattery();
  checkIfStucked();
  checkRobotStats();
  odometer.calc(imu);
  checkOdometerFaults();
  checkButton();
  motorMowControl();
  checkTilt();

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
          setNextState(STATE_STATION, 0);
        }
      }
      imuDriveHeading = imu.ypr.yaw;
      break;

    case STATE_REMOTE:
      // remote control mode (RC)
      //if (remoteSwitch > 50) setNextState(STATE_FORWARD, 0);
      steer = ((double) wheels.wheel[Wheel::LEFT].motor.rpmMax / 2)
                  * (((double) remoteSteer) / 100.0);
      if (remoteSpeed < 0)
      {
        steer *= -1;
      }
      wheels.wheel[Wheel::LEFT].motor.rpmSet = ((double) wheels.wheel[Wheel::LEFT].motor.rpmMax)
          * (((double) remoteSpeed) / 100.0)
                             - steer;
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = ((double) wheels.wheel[Wheel::LEFT].motor.rpmMax)
          * (((double) remoteSpeed) / 100.0)
                              + steer;
      wheels.wheel[Wheel::LEFT].motor.rpmSet = max(-wheels.wheel[Wheel::LEFT].motor.rpmMax,
                                 min(wheels.wheel[Wheel::LEFT].motor.rpmMax, wheels.wheel[Wheel::LEFT].motor.rpmSet));
      wheels.wheel[Wheel::RIGHT].motor.rpmSet = max(-wheels.wheel[Wheel::LEFT].motor.rpmMax,
                                  min(wheels.wheel[Wheel::LEFT].motor.rpmMax, wheels.wheel[Wheel::RIGHT].motor.rpmSet));
      cutter.motor.pwmSet = (double)cutter.motor.pwmMax * ((double)remoteMow / 100.0);
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
          wheels.wheel[Wheel::RIGHT].motor.rpmSet = ((double) wheels.wheel[Wheel::LEFT].motor.rpmSet) * ratio;
        }
        else
        {
          wheels.wheel[Wheel::LEFT].motor.rpmSet = ((double) wheels.wheel[Wheel::RIGHT].motor.rpmSet) * ratio;
        }
      }
      checkErrorCounter();
      checkTimer();
      checkRain();
      checkPower();
      checkBumpers();
      checkDrop();
      checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      checkTimeout();
      break;

    case STATE_ROLL:
      checkPower();
      checkBumpers();
      checkDrop();
      //checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      // making a roll (left/right)
      if (mowPatternCurr == MOW_LANES)
      {
        if (abs(distancePI(imu.ypr.yaw, imuRollHeading)) < PI / 36)
        {
          setNextState(STATE_FORWARD, 0);
        }
      }
      else
      {
        if (curMillis >= stateEndTime)
        {
          setNextState(STATE_FORWARD, 0);
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
      checkPower();
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
          wheels.wheel[Wheel::RIGHT].motor.rpmSet = ((double) wheels.wheel[Wheel::LEFT].motor.rpmSet) * ratio;
        }
        else
        {
          wheels.wheel[Wheel::LEFT].motor.rpmSet = ((double) wheels.wheel[Wheel::RIGHT].motor.rpmSet) * ratio;
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
        setNextState(STATE_PERI_FIND, 0);
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
        checkPower();
        checkBumpersPerimeter();
        checkSonar();
      }
      checkPerimeterFind();
      checkTimeout();
      break;

    case STATE_PERI_TRACK:
      // track perimeter
      checkPower();
      checkBumpersPerimeter();
      //checkSonar();
      if (batMonitor)
      {
        if (chgVoltage > 5.0)
        {
          setNextState(STATE_STATION, 0);
        }
      }
      break;

    case STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (batMonitor)
      {
        if ((chgVoltage > 5.0) && (batVoltage > 8))
        {
          if (batVoltage < startChargingIfBelow &&
              curMillis - stateStartTime > 2000)
          {
            setNextState(STATE_STATION_CHARGING, 0);
          }
          else
          {
            checkTimer();
          }
        }
        else
        {
          setNextState(STATE_OFF, 0);
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
        if ((chgCurrent < batFullCurrent) &&
            curMillis - stateStartTime > 2000)
        {
          setNextState(STATE_STATION, 0);
        }
        else if (curMillis - stateStartTime > chargingTimeout)
        {
          addErrorCounter(ERR_BATTERY);
          setNextState(STATE_ERROR, 0);
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
        setNextState(STATE_FORWARD, 0);
      }
      break;

    case STATE_STATION_CHECK:
      // check for charging voltage disappearing before leaving charging station
      if (millis() >= stateEndTime)
      {
        if (chgVoltage > 5)
        {
          addErrorCounter(ERR_CHARGER);
          setNextState(STATE_ERROR, 0);
        }
        else
        {
          setNextState(STATE_STATION_REV, 0);
        }
      }
      break;

    case STATE_STATION_REV:
      // charging: drive reverse
      if (curMillis >= stateEndTime)
        setNextState(STATE_STATION_ROLL, 0);
      break;

    case STATE_STATION_ROLL:
      // charging: roll
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_STATION_FORW, 0);
      }
      break;

    case STATE_STATION_FORW:
      // forward (charge station)
      if (curMillis >= stateEndTime)
      {
        setNextState(STATE_FORWARD, 0);
      }
      break;
  } // end switch

  // next line deactivated (issue with RC failsafe)
  //if ((useRemoteRC) && (remoteSwitch < -50)) setNextState(STATE_REMOTE, 0);

  // decide which motor control to use
  if ((mowPatternCurr == MOW_LANES && stateCurr == STATE_ROLL) ||
      stateCurr == STATE_ROLL_WAIT)
  {
    motorControlImuRoll();
  }
  else if (stateCurr == STATE_PERI_TRACK)
  {
    motorControlPerimeter();
  }
  else if (stateCurr == STATE_FORWARD &&
           imu.use &&
           (imu.correctDir || mowPatternCurr == MOW_LANES))
  {
    motorControlImuDir();
  }
  else
  {
    motorControl();
  }

  if (stateCurr != STATE_REMOTE)
  {
    cutter.motor.pwmSet = cutter.motor.pwmMax;
  }

  bumpers.clearHit();
  dropSensors.clearDetected();

  loopsPerSecCounter++;
}
