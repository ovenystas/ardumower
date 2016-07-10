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
    "sen_counters", "sen_values", "perimeter", "imu"
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
    addErrorCounter(ERR_EEPROM_DATA);
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
    addErrorCounter(ERR_EEPROM_DATA);
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
  eereadwrite(readflag, addr, perimeterUse);
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
  eereadwrite(readflag, addr, lawnSensor.used);
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
  eereadwrite(readflag, addr, odometer.encoder[Odometer::LEFT].swapDir);
  eereadwrite(readflag, addr, odometer.encoder[Odometer::RIGHT].swapDir);
  eereadwrite(readflag, addr, odometer.encoder[Odometer::LEFT].twoWay);
  eereadwrite(readflag, addr, button.used);
  eereadwrite(readflag, addr, userSwitch1);
  eereadwrite(readflag, addr, userSwitch2);
  eereadwrite(readflag, addr, userSwitch3);
  eereadwrite(readflag, addr, timerUse);
  eereadwrite(readflag, addr, timer);
  eereadwrite(readflag, addr, rainSensor.used);
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
  Console.println(rainSensor.used);

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
  Console.println(lawnSensor.used);

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
  Console.println(stuckIfGpsSpeedBelow);
  Console.print(F("gpsSpeedIgnoreTime : "));
  Console.println(gpsSpeedIgnoreTime);

  // ----- other -----------------------------------------
  Console.println(F("== Button =="));
  Console.print(F("use : "));
  Console.println(button.used);

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

void Robot::addErrorCounter(const enum errorE errType)
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
                        const unsigned long samplingTime,
                        const uint8_t motor,
                        const boolean useAccel)
{
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
    if (abs(odometer.encoder[motor].getWheelRpmCurr()) < 1)
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
  unsigned long samplingTime;
//  Console.print("setPwm: ");
//  Console.print(pwmLeft);
//  Console.print(" ");
//  Console.println(pwmRight);

  samplingTime = wheels.wheel[Wheel::LEFT].motor.getSamplingTime();
  setMotorPWM(pwmLeft, samplingTime, LEFT, useAccel);

  samplingTime = wheels.wheel[Wheel::RIGHT].motor.getSamplingTime();
  setMotorPWM(pwmRight, samplingTime, RIGHT, useAccel);
}

// sets mower motor actuator
// - ensures that the motor is not switched to 100% too fast (cutter.motor.acceleration)
// - ensures that the motor voltage is not higher than motorMowSpeedMaxPwm
void Robot::setMotorMowPWM(const int pwm, const boolean useAccel)
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
  if (!imu.isTimeToControl())
  {
    return;
  }

  Pid* pid_p;
  int pwmMax;

  // Regelbereich entspricht 80% der maximalen Drehzahl am Antriebsrad (motorSpeedMaxRpm)
  float imuPidX = distancePI(imu.getYaw(), imuRollHeading) / PI * 180.0;
  pid_p = &imu.pid[Imu::ROLL];
  pid_p->setSetpoint(0);
  int rpmMax80Percent = wheels.wheel[Wheel::LEFT].motor.rpmMax / 1.25;
  pid_p->y_min = -rpmMax80Percent ; // da der Roll generell langsamer erfolgen soll
  pid_p->y_max = rpmMax80Percent;   //
  pid_p->max_output = rpmMax80Percent; //
  float imuPidY = pid_p->compute(imuPidX);

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  pid_p = &wheels.wheel[Wheel::LEFT].motor.pid;
  pid_p->setSetpoint(-imuPidY);             // SOLL
  pwmMax = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  pid_p->y_min = -pwmMax;        // Regel-MIN
  pid_p->y_max = pwmMax;   // Regel-MAX
  pid_p->max_output = pwmMax;    // Begrenzung
  float leftWheelPidY = pid_p->compute(odometer.encoder[Odometer::LEFT].getWheelRpmCurr());
  int leftSpeed = max(-pwmMax, min(pwmMax, wheels.wheel[Wheel::LEFT].motor.pwmCur + leftWheelPidY));

  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  pid_p = &wheels.wheel[Wheel::RIGHT].motor.pid;
  pid_p->setSetpoint(imuPidY);             // SOLL
  pwmMax = wheels.wheel[Wheel::RIGHT].motor.pwmMax;
  pid_p->y_min = -pwmMax;       // Regel-MIN
  pid_p->y_max = pwmMax;  // Regel-MAX
  pid_p->max_output = pwmMax;   // Begrenzung
  float rightWheelPidY = pid_p->compute(odometer.encoder[Odometer::RIGHT].getWheelRpmCurr());
  int rightSpeed = max(-pwmMax, min(pwmMax, wheels.wheel[Wheel::RIGHT].motor.pwmCur + rightWheelPidY));

  if ((stateCurr == STATE_OFF || stateCurr == STATE_STATION || stateCurr == STATE_ERROR) && (millis() - stateStartTime > 1000))
  {
    leftSpeed = rightSpeed = 0; // ensures PWM is zero if OFF/CHARGING
  }
  setMotorPWMs(leftSpeed, rightSpeed, false);
}

// PID controller: track perimeter
void Robot::motorControlPerimeter()
{
  if (!perimeters.isTimeToControl())
  {
    return;
  }

  unsigned long curMillis = millis();
  if ((curMillis > stateStartTime + 5000) && (curMillis
      > perimeterLastTransitionTime + trackingPerimeterTransitionTimeOut))
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
      setNextState(STATE_PERI_FIND);
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
  Pid* pid_p = &perimeters.perimeter[Perimeter::LEFT].pid;
  pid_p->setSetpoint(0);
  int pwmMax;
  pwmMax = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  pid_p->y_min = -pwmMax;
  pid_p->y_max = pwmMax;
  pid_p->max_output = pwmMax;
  float y = pid_p->compute(x);

  pwmMax = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  int leftSpeed = max(-pwmMax, min(pwmMax, pwmMax / 2 - y));

  pwmMax = wheels.wheel[Wheel::RIGHT].motor.pwmMax;
  int rightSpeed = max(-pwmMax, min(pwmMax, pwmMax / 2 + y));

  setMotorPWMs(leftSpeed, rightSpeed, false);
}

// PID controller: correct direction during normal driving (requires IMU)
void Robot::motorControlImuDir()
{
  if (!imu.isTimeToControl())
  {
    return;
  }

  Pid* pid_p;

  // Regelbereich entspricht maximaler Drehzahl am Antriebsrad (motorSpeedMaxRpm)
  pid_p = &imu.pid[Imu::DIR];
  pid_p->setSetpoint(0);
  int rpmMax = wheels.wheel[Wheel::LEFT].motor.rpmMax;
  pid_p->y_min = -rpmMax;
  pid_p->y_max = rpmMax;
  pid_p->max_output = rpmMax;
  float x = distancePI(imu.getYaw(), imuDriveHeading) / PI * 180.0;
  float y = pid_p->compute(x);

  int correctLeft = 0;
  int correctRight = 0;
  if (y < 0)
  {
    correctRight = abs(y);
  }
  if (y > 0)
  {
    correctLeft = abs(y);
  }

  int pwmMax;

  // Korrektur erfolgt über Abbremsen des linken Antriebsrades, falls Kursabweichung nach rechts
  // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm), um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
  pid_p = &wheels.wheel[Wheel::LEFT].motor.pid;
  pid_p->setSetpoint(wheels.wheel[Wheel::LEFT].motor.rpmSet - correctLeft);     // SOLL
  pwmMax = wheels.wheel[Wheel::LEFT].motor.pwmMax;
  pid_p->y_min = -pwmMax;            // Regel-MIN
  pid_p->y_max = pwmMax;       // Regel-MAX
  pid_p->max_output = pwmMax;        // Begrenzung
  float leftX = odometer.encoder[Odometer::LEFT].getWheelRpmCurr();
  float leftY = pid_p->compute(leftX);
  int leftSpeed = max(-pwmMax, min(pwmMax, wheels.wheel[Wheel::LEFT].motor.pwmCur + leftY));
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
  pid_p = &wheels.wheel[Wheel::RIGHT].motor.pid;
  pid_p->setSetpoint(wheels.wheel[Wheel::RIGHT].motor.rpmSet - correctRight);  // SOLL
  pwmMax = wheels.wheel[Wheel::RIGHT].motor.pwmMax;
  pid_p->y_min = -pwmMax;           // Regel-MIN
  pid_p->y_max = pwmMax;      // Regel-MAX
  pid_p->max_output = pwmMax;       // Begrenzung
  float rightX = odometer.encoder[Odometer::RIGHT].getWheelRpmCurr();
  float rightY = wheels.wheel[Wheel::RIGHT].motor.pid.compute(rightX);
  int rightSpeed = max(-pwmMax, min(pwmMax, wheels.wheel[Wheel::RIGHT].motor.pwmCur + rightY));
  if ((wheels.wheel[Wheel::RIGHT].motor.rpmSet >= 0) && (rightSpeed < 0))
  {
    rightSpeed = 0;
  }
  if ((wheels.wheel[Wheel::RIGHT].motor.rpmSet <= 0) && (rightSpeed > 0))
  {
    rightSpeed = 0;
  }

  if (((stateCurr == STATE_OFF) ||
       (stateCurr == STATE_STATION) ||
       (stateCurr == STATE_ERROR)) &&
      (millis() - stateStartTime > 1000))
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
      if (wheels.wheel[i].motor.pwmCur > 100 &&
          abs(odometer.encoder[i].getWheelRpmCurr()) < 1)
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
      if ((wheels.wheel[i].motor.pwmCur > 100 && odometer.encoder[i].getWheelRpmCurr() < -3) ||
          (wheels.wheel[i].motor.pwmCur < -100 && odometer.encoder[i].getWheelRpmCurr() > 3))
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
    Console.println(odometer.encoder[Odometer::LEFT].getWheelRpmCurr());
    addErrorCounter(ERR_ODOMETER_LEFT);
    setNextState(STATE_ERROR);
  }

  if (err[RIGHT])
  {
    Console.print("Right odometer error: PWM=");
    Console.print(wheels.wheel[Wheel::RIGHT].motor.pwmCur);
    Console.print("\tRPM=");
    Console.println(odometer.encoder[Odometer::RIGHT].getWheelRpmCurr());
    addErrorCounter(ERR_ODOMETER_RIGHT);
    setNextState(STATE_ERROR);
  }
}

void Robot::motorControl()
{
  if (!wheels.wheel[Wheel::LEFT].motor.isTimeToControl())
  {
    return;
  }

  int speed[2];
  if (odometer.use)
  {
    for (uint8_t i = Wheel::LEFT; i <= Wheel::RIGHT; i++)
    {
      Pid* pid_p = &wheels.wheel[i].motor.pid;
      // Regelbereich entspricht maximaler PWM am Antriebsrad (motorSpeedMaxPwm),
      // um auch an Steigungen höchstes Drehmoment für die Solldrehzahl zu gewährleisten
      if (millis() < stateStartTime + wheels.wheel[i].motor.zeroSettleTime)
      {
        pid_p->setSetpoint(0); // get zero speed first after state change
      }
      int pwmMax = wheels.wheel[i].motor.pwmMax;
      pid_p->y_min = -pwmMax; // Regel-MIN
      pid_p->y_max = pwmMax; // Regel-MAX
      pid_p->max_output = pwmMax; // Begrenzung
      float x = odometer.encoder[i].getWheelRpmCurr();
      float y = pid_p->compute(x);
      speed[i] = max(-pwmMax, min(pwmMax, wheels.wheel[i].motor.pwmCur + y));

      if (abs(x) < 2 && abs(pid_p->getSetpoint()) < 0.1)
      {
        speed[i] = 0; // ensures PWM is really zero
      }
    }
    setMotorPWMs(speed[LEFT], speed[RIGHT], false);
  }
  else
  {
    for (uint8_t i = Wheel::LEFT; i <= Wheel::RIGHT; i++)
    {
      int pwmMax = wheels.wheel[i].motor.pwmMax;
      int rpmMax = wheels.wheel[i].motor.rpmMax;
      speed[i] = min(pwmMax, max(-pwmMax, map(wheels.wheel[i].motor.rpmSet,
                                              -rpmMax, rpmMax,
                                              -pwmMax, pwmMax)));
    }
    if (millis() < stateStartTime + wheels.wheel[Wheel::LEFT].motor.zeroSettleTime)
    {
      // slow down at state start
      speed[LEFT] = 0;
      speed[RIGHT] = 0;

      if (mowPatternCurr != MOW_LANES)
      {
        imuDriveHeading = imu.getYaw(); // set drive heading
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
  if (!cutter.motor.isTimeToControl())
  {
    return;
  }

  Pid* pid_p = &cutter.motor.pid;

  if (cutter.isEnableOverriden())
  {
    cutter.disable();
  }

  if (cutter.isDisabled())
  {
    pid_p->setSetpoint(0);
    setMotorMowPWM(0, false);
  }
  else
  {
    // Need speed sensor to be able to regulate speed
    if (cutter.motor.regulate)
    {
      pid_p->setSetpoint(cutter.motor.rpmSet);
      float newPwm = pid_p->compute(cutter.motor.getRpmMeas());

      setMotorMowPWM(newPwm, false);
    }
    else
    {
      if (errorCounter[ERR_MOW_SENSE] == 0 && errorCounter[ERR_STUCK] == 0)
      {
        // no speed sensor available
        setMotorMowPWM(cutter.motor.getPwmSet(), true);
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

void Robot::beep(const int numberOfBeeps, const boolean shortbeep)
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
  //loadErrorCounters();
  //loadUserSettings();
  if (!statsOverride)
  {
    //loadRobotStats();
  }
  else
  {
    saveRobotStats();
  }
  setUserSwitches();

  if (!button.used)
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
    perimeters.perimeter[Perimeter::LEFT].printInfo(s);
    Streamprint(s, "  in %-2d  cnt %-4d  on %-1d\r\n", perimeterInside,
                perimeterCounter,
                !perimeters.perimeter[Perimeter::LEFT].signalTimedOut());
  }
  else if (consoleMode == CONSOLE_IMU)
  {
    imu.printInfo(s);
  }
  else
  {
    if (odometer.use)
    {
      Streamprint(s, "odo %4d %4d ",
                  odometer.encoder[Odometer::LEFT].getCounter(),
                  odometer.encoder[Odometer::RIGHT].getCounter());
    }
    Streamprint(s, "spd %4d %4d %4d ", wheels.wheel[Wheel::LEFT].motor.rpmSet,
                wheels.wheel[Wheel::RIGHT].motor.rpmSet,
                cutter.motor.getRpmMeas());

    if (consoleMode == CONSOLE_SENSOR_VALUES)
    {
      // sensor values
      Streamprint(s, "sen %4d %4d %4d ",
                  wheels.wheel[Wheel::LEFT].motor.getPowerMeas(),
                  wheels.wheel[Wheel::RIGHT].motor.getPowerMeas(),
                  cutter.motor.getPowerMeas());
      Streamprint(s, "bum %4d %4d ", bumpers.bumper[Bumpers::LEFT].isHit(),
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

      if (perimeterUse)
      {
        Streamprint(s, "per %3d ", perimeterInside);
      }

      if (lawnSensor.used)
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
                  wheels.wheel[Wheel::LEFT].motor.getOverloadCounter(),
                  wheels.wheel[Wheel::RIGHT].motor.getOverloadCounter(),
                  cutter.motor.getOverloadCounter());
      Streamprint(s, "bum %4d %4d ", bumpers.bumper[Bumpers::LEFT].getCounter(),
                  bumpers.bumper[Bumpers::RIGHT].getCounter());
      Streamprint(s, "dro %4d %4d ",
                  dropSensors.dropSensor[DropSensors::LEFT].getCounter(),
                  dropSensors.dropSensor[DropSensors::RIGHT].getCounter());
      Streamprint(s, "son %3d ", sonars.getDistanceCounter());
      Streamprint(s, "yaw %3d ", (int)(imu.getYawDeg()));
      Streamprint(s, "pit %3d ", (int)(imu.getPitchDeg()));
      Streamprint(s, "rol %3d ", (int)(imu.getRollDeg()));
      //Streamprint(s, "per %3d ", perimeterLeft);

      if (perimeterUse)
      {
        Streamprint(s, "per %3d ", perimeterCounter);
      }

      if (lawnSensor.used)
      {
        Streamprint(s, "lawn %3d ", lawnSensor.getCounter());
      }

      if (gpsUse)
      {
        Streamprint(s, "gps %2d ", gps.satellites());
      }
    }

    Streamprint(s, "bat %2d.%01d ", (int)batVoltage,
                (int)((batVoltage * 10) - ((int)batVoltage * 10)));
    Streamprint(s, "chg %2d.%01d %2d.%01d ", (int)chgVoltage,
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

  wheels.wheel[Wheel::LEFT].motor.pwmCur = leftPwm;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = rightPwm;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);

  int lastLeft = 0;
  int lastRight = 0;
  for (;;)
  {
    resetIdleTime();
    int odoCountLeft = odometer.encoder[Odometer::LEFT].getCounter();
    int odoCountRight = odometer.encoder[Odometer::RIGHT].getCounter();
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
        wheels.wheel[Wheel::LEFT].motor.pwmCur = leftPwm;
        wheels.wheel[Wheel::RIGHT].motor.pwmCur = rightPwm;
        setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
                     wheels.wheel[Wheel::RIGHT].motor.pwmCur,
                     false);
      }
      if (ch == 'r')
      {
        wheels.wheel[Wheel::LEFT].motor.pwmCur = -leftPwm;
        wheels.wheel[Wheel::RIGHT].motor.pwmCur = -rightPwm;
        setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
                     wheels.wheel[Wheel::RIGHT].motor.pwmCur,
                     false);
      }
      if (ch == 'z')
      {
        odometer.encoder[Odometer::LEFT].clearCounter();
        odometer.encoder[Odometer::RIGHT].clearCounter();
      }
    }
  };
  wheels.wheel[Wheel::LEFT].motor.pwmCur = 0;
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = 0;
  setMotorPWMs(wheels.wheel[Wheel::LEFT].motor.pwmCur,
               wheels.wheel[Wheel::RIGHT].motor.pwmCur,
               false);
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
  wheels.wheel[Wheel::LEFT].motor.pwmCur = wheels.wheel[Wheel::LEFT].motor
      .pwmMax;
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
  wheels.wheel[Wheel::LEFT].motor.pwmCur = -wheels.wheel[Wheel::LEFT].motor
      .pwmMax;
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
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = wheels.wheel[Wheel::LEFT].motor
      .pwmMax;
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
  wheels.wheel[Wheel::RIGHT].motor.pwmCur = -wheels.wheel[Wheel::LEFT].motor
      .pwmMax;
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
      case 'd':
        menu(); // menu
        break;
      case 'v':
        consoleMode = (consoleMode + 1) % 5;
        Console.println(consoleModeNames[consoleMode]);
        break;
      case 'h':
        setNextState(STATE_PERI_FIND); // press 'h' to drive home
        break;
      case 't':
        setNextState(STATE_PERI_TRACK); // press 'p' to track perimeter
        break;
      case 'l':
        bumpers.bumper[Bumpers::LEFT].simHit(); // press 'l' to simulate left bumper
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
          cutter.setEnableOverriden(false);
        }
        else
        {
          cutter.toggleEnableOverriden();
        }
        cutter.toggleEnabled(); // press 'm' to toggle mower motor
        break;
      case 'c':
        setNextState(STATE_STATION); // press 'c' to simulate in station
        break;
      case 'a':
        setNextState(STATE_STATION_CHARGING); // press 'a' to simulate in station charging
        break;
      case '+':
        setNextState(STATE_ROLL_WAIT); // press '+' to rotate 90 degrees clockwise (IMU)
        imuRollHeading = scalePI(imuRollHeading + PI / 2);
        break;
      case '-':
        setNextState(STATE_ROLL_WAIT); // press '-' to rotate 90 degrees anti-clockwise (IMU)
        imuRollHeading = scalePI(imuRollHeading - PI / 2);
        break;
      case 'i':
        // press 'i' to toggle imu.use
        imu.use = !imu.use;
        break;
      case '3':
        setNextState(STATE_REMOTE); // press '3' to activate model RC
        break;
      case '0':
        // press '0' for OFF
        setNextState(STATE_OFF);
        break;
      case '1':
        // press '1' for Automode
        cutter.enable();
        //motorMowModulate = false;
        setNextState(STATE_FORWARD);
        break;
    }
  }
}

void Robot::checkButton()
{
  boolean buttonPressed = false;
  if (button.isTimeToCheck())
  {
    buttonPressed = button.isPressed();
  }

  if ((!buttonPressed && button.getCounter() > 0) || (buttonPressed
      && button.isTimeToRun()))
  {
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
            perimeterUse = false;
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

void Robot::readSensors()
{
// NOTE: This function should only read in sensors into variables.
//       It should NOT change any state!

  unsigned long curMillis = millis();
  if (cutter.motor.isTimeToReadSensor())
  {
    wheels.wheel[Wheel::LEFT].motor.readCurrent();
    wheels.wheel[Wheel::RIGHT].motor.readCurrent();
    cutter.motor.readCurrent();

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
        Console.println(F("Error: missing ADC calibration data"));
        addErrorCounter(ERR_ADC_CALIB);
        setNextState(STATE_ERROR);
      }
    }
  }

  if (perimeterUse && curMillis >= nextTimePerimeter)
  {
    nextTimePerimeter = curMillis + 50;
    perimeterMag = readSensor(SEN_PERIM_LEFT);
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
      if (curMillis > stateStartTime + 2000)
      { // far away from perimeter?
        perimeterTriggerTime = curMillis + perimeterTriggerTimeout;
      }
      else
      {
        perimeterTriggerTime = curMillis;
      }
    }
    if (perimeters.perimeter[Perimeter::LEFT].signalTimedOut())
    {
      if (stateCurr != STATE_OFF && stateCurr != STATE_MANUAL
          && stateCurr != STATE_STATION && stateCurr != STATE_STATION_CHARGING
          && stateCurr != STATE_STATION_CHECK && stateCurr != STATE_STATION_REV
          && stateCurr != STATE_STATION_ROLL && stateCurr != STATE_STATION_FORW
          && stateCurr != STATE_REMOTE && stateCurr != STATE_PERI_OUT_FORW
          && stateCurr != STATE_PERI_OUT_REV
          && stateCurr != STATE_PERI_OUT_ROLL)
      {
        Console.println("Error: perimeter too far away");
        addErrorCounter(ERR_PERIMETER_TIMEOUT);
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

  //if ((timerUse) && (millis() >= nextTimeRTC)) {
  if (millis() >= nextTimeRTC)
  {
    nextTimeRTC = curMillis + 60000;
    readSensor(SEN_RTC);       // read RTC
    Console.print(F("RTC date received: "));
    Console.println(date2str(datetime.date));
  }

  if (imu.isTimeToRun())
  {
    // IMU
    readSensor(SEN_IMU);
    if (imu.getErrorCounter() > 0)
    {
      addErrorCounter(ERR_IMU_COMM);
      Console.println(F("IMU comm error"));
    }
    if (!imu.isCalibrationAvailable())
    {
      Console.println(F("Error: missing IMU calibration data"));
      addErrorCounter(ERR_IMU_CALIB);
      setNextState(STATE_ERROR);
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
    double batvolt = (double)batADC * batFactor / 10; // / 10 due to arduremote bug, can be removed after fixing
    //double chgvolt = ((double)((int)(readSensor(SEN_CHG_VOLTAGE) / 10))) / 10.0;
    int chgADC = readSensor(SEN_CHG_VOLTAGE);
    //Console.println(chgADC);
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
      vcc = (float)3.30 / chgSenseZero * 1023.0; // Versorgungsspannung ermitteln!  chgSenseZero=511  ->Die Genauigkeit kann erhöt werden wenn der 3.3V Pin an ein Analogen Pin eingelesen wird. Dann ist vcc = (float) 3.30 / analogRead(X) * 1023.0;
      asensor = (float)chgAMP * vcc / 1023.0;              // Messwert auslesen
      asensor = (float)asensor - (vcc / chgNull); // Nulldurchgang (vcc/2) abziehen
      chgSense = (float)chgSense - ((5.00 - vcc) * chgFactor); // Korrekturfactor für Vcc!  chgFactor=39
      amp = (float)asensor / chgSense * 1000;               // Ampere berechnen
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

  if (rainSensor.isTimeToRun())
  {
    // read rain sensor
    rainSensor.check();
  }
}

void Robot::setDefaults()
{
  wheels.stop();
  cutter.disable();
}

// set state machine new state
// http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
// called *ONCE* to set to a *NEW* state
void Robot::setNextState(byte stateNew, bool dir)
{
  unsigned long curMillis = millis();
  if (stateNew == stateCurr)
  {
    return;
  }

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
    wheels.reverseFullSpeed();
    stateEndTime = curMillis + stationRevTime + zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_ROLL)
  {
    wheels.rollFullRight();
    stateEndTime = curMillis + stationRollTime + zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_FORW)
  {
    wheels.forwardFullSpeed();
    cutter.enable();
    stateEndTime = curMillis + stationForwTime + zeroSettleTime;
  }
  else if (stateNew == STATE_STATION_CHECK)
  {
    wheels.reverseSlowSpeed();
    stateEndTime = curMillis + stationCheckTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_ROLL)
  {
    stateEndTime = curMillis + perimeterTrackRollTime + zeroSettleTime;
    wheels.rollSlow(dir);
  }
  if (stateNew == STATE_PERI_REV)
  {
    wheels.reverseSlowSpeed();
    stateEndTime = curMillis + perimeterTrackRevTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_OUT_FORW)
  {
    wheels.forwardFullSpeed();
    stateEndTime = curMillis + perimeterOutRevTime + zeroSettleTime + 1000;
  }
  else if (stateNew == STATE_PERI_OUT_REV)
  {
    wheels.reverseFastSpeed();
    stateEndTime = curMillis + perimeterOutRevTime + zeroSettleTime;
  }
  else if (stateNew == STATE_PERI_OUT_ROLL)
  {
    stateEndTime = curMillis + random(perimeterOutRollTimeMin, perimeterOutRollTimeMax) + zeroSettleTime;
    wheels.rollFast(dir);
  }
  else if (stateNew == STATE_FORWARD)
  {
    wheels.forwardFullSpeed();
    statsMowTimeTotalStart = true;
  }
  else if (stateNew == STATE_REVERSE)
  {
    wheels.reverseFastSpeed();
    stateEndTime = curMillis + wheels.reverseTime + zeroSettleTime;
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
    stateEndTime = curMillis + random(wheels.rollTimeMin, wheels.rollTimeMax) + zeroSettleTime;
    wheels.rollFast(dir);
  }
  if (stateNew == STATE_REMOTE)
  {
    cutter.enable();
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
    cutter.disable();
    wheels.stop();
    setActuator(ACT_CHGRELAY, 0);
    statsMowTimeTotalStart = false;
    //saveRobotStats();
  }
  if (stateNew == STATE_PERI_FIND)
  {
    // find perimeter  => drive half speed
    wheels.forwardHalfSpeed();
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
    if (batVoltage < batSwitchOffIfBelow && stateCurr != STATE_ERROR
        && stateCurr != STATE_OFF && stateCurr != STATE_STATION
        && stateCurr != STATE_STATION_CHARGING)
    {
      Console.println(F("triggered batSwitchOffIfBelow"));
      addErrorCounter(ERR_BATTERY);
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
             perimeterUse)
    {    //UNTESTED please verify
      Console.println(F("triggered batGoHomeIfBelow"));
      beep(2, true);
      setNextState(STATE_PERI_FIND);
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
    statsBatteryChargingCapacityAverage = statsBatteryChargingCapacityTotal / statsBatteryChargingCounterTotal;
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
              cutter.enable();
              setNextState(STATE_FORWARD);
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
              setNextState(STATE_PERI_FIND);
            }
            else
            {
              setNextState(STATE_OFF);
            }
          }
        }
      }
    }
  }
}

void Robot::reverseOrBidir(const byte aRollDir)
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
  if (!cutter.motor.isTimeToCheckPower())
  {
    return;
  }

  unsigned long curMillis = millis();
  if (cutter.motor.getPowerMeas() >= cutter.motor.powerMax)
  {
    cutter.motor.incOverloadCounter();
  }
  else
  {
    errorCounterMax[ERR_MOW_SENSE] = 0;
    cutter.motor.clearOverloadCounter();
    if (cutter.motor.isWaitAfterStuckEnd())
    { // wait some time before switching on again
      errorCounter[ERR_MOW_SENSE] = 0;
      cutter.enable();
    }
  }

  if (cutter.motor.getOverloadCounter() >= 30)
  { //ignore motorMowPower for 3 seconds
    cutter.disable();
    Console.println("Error: Motor mow current");
    addErrorCounter(ERR_MOW_SENSE);
    cutter.motor.gotStuck();
    // if (rollDir == RIGHT) reverseOrBidir(LEFT); // toggle roll dir
    //else reverseOrBidir(RIGHT);
  }

  if (wheels.wheel[Wheel::LEFT].motor.isOverpowered())
  {
    // left wheel motor overpowered
    if ((stateCurr == STATE_FORWARD || stateCurr == STATE_PERI_FIND
         || stateCurr == STATE_PERI_TRACK)
        && curMillis > (stateStartTime + wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime))
    {
      //beep(1);
      wheels.wheel[Wheel::LEFT].motor.incOverloadCounter();
      setMotorPWMs(0, 0, false);
      reverseOrBidir(RIGHT);
    }
    else if (stateCurr == STATE_REVERSE
        && curMillis > (stateStartTime + wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::LEFT].motor.incOverloadCounter();
      setMotorPWMs(0, 0, false);
      //   reverseOrBidir(RIGHT);
      setNextState(STATE_ROLL, RIGHT);
    }
    else if (stateCurr == STATE_ROLL
        && curMillis > (stateStartTime + wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::LEFT].motor.incOverloadCounter();
      setMotorPWMs(0, 0, false);
      setNextState(STATE_FORWARD);
    }
  }
  else if (wheels.wheel[Wheel::RIGHT].motor.isOverpowered())
  {
    // right wheel motor overpowered
    if ((stateCurr == STATE_FORWARD || stateCurr == STATE_PERI_FIND) && curMillis > (stateStartTime + wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime))
    {
      //beep(1);
      wheels.wheel[Wheel::RIGHT].motor.incOverloadCounter();
      setMotorPWMs(0, 0, false);
      reverseOrBidir(RIGHT);
    }
    else if (stateCurr == STATE_REVERSE && curMillis > (stateStartTime + wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::RIGHT].motor.incOverloadCounter();
      setMotorPWMs(0, 0, false);
      setNextState(STATE_ROLL, LEFT);
    }
    else if (stateCurr == STATE_ROLL && curMillis > (stateStartTime + wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime))
    {
      wheels.wheel[Wheel::RIGHT].motor.incOverloadCounter();
      setMotorPWMs(0, 0, false);
      setNextState(STATE_FORWARD);
    }
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
  if (mowPatternCurr == MOW_BIDIR && curMillis < (stateStartTime + 4000))
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

// check perimeter as a boundary
void Robot::checkPerimeterBoundary()
{
  unsigned long curMillis = millis();

  if (wheels.isTimeToRotationChange())
  {
    if (wheels.rotateDir == Wheel::LEFT)
    {
      wheels.rotateDir = Wheel::RIGHT;
    }
    else
    {
      wheels.rotateDir = Wheel::LEFT;
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
    if (perimeters.perimeter[Perimeter::LEFT].isInside())
    {
      // inside
      if (wheels.wheel[Wheel::LEFT].motor.rpmSet != wheels.wheel[Wheel::RIGHT].motor.rpmSet)
      {
        // we just made an 'outside=>inside' rotation, now track
        setNextState(STATE_PERI_TRACK);
      }
    }
    else
    {
      // we are outside, now roll to get inside
      wheels.rollHalfRight();
    }
  }
}

// check lawn
void Robot::checkLawn()
{
  if (!lawnSensor.used)
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
  if (!rainSensor.used)
  {
    return;
  }

  if (rainSensor.isRaining())
  {
    Console.println(F("RAIN"));
    if (perimeterUse)
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
  if (!sonars.use)
  {
    return;
  }

  if (!sonars.isTimeToCheck())
  {
    return;
  }

  unsigned long curMillis = millis();
  if (mowPatternCurr == MOW_BIDIR && curMillis < (stateStartTime + 4000))
  {
    return;
  }

  // slow down motor wheel speed near obstacles
  if (stateCurr == STATE_FORWARD || (mowPatternCurr == MOW_BIDIR
      && (stateCurr == STATE_FORWARD || stateCurr == STATE_REVERSE)))
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
          wheels.slowDown();
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
      wheels.speedUp();
    }
  }

  uint16_t distanceUs;

  distanceUs = sonars.sonar[Sonars::CENTER].getDistance_us();
  if (distanceUs < sonars.triggerBelow)
  {
    sonars.incDistanceCounter();
    reverseOrBidir(!rollDir); // toggle roll dir
  }

  distanceUs = sonars.sonar[Sonars::RIGHT].getDistance_us();
  if (distanceUs < sonars.triggerBelow)
  {
    sonars.incDistanceCounter();
    reverseOrBidir(LEFT);
  }

  distanceUs = sonars.sonar[Sonars::LEFT].getDistance_us();
  if (distanceUs < sonars.triggerBelow)
  {
    sonars.incDistanceCounter();
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

  if (stateCurr != STATE_OFF &&
      stateCurr != STATE_ERROR &&
      stateCurr != STATE_STATION)
  {
    int pitchAngle = (int)imu.getPitchDeg();
    int rollAngle = (int)imu.getRollDeg();
    if (abs(pitchAngle) > 40 || abs(rollAngle) > 40)
    {
      Console.println(F("Error: IMU tilt"));
      addErrorCounter(ERR_IMU_TILT);
      setNextState(STATE_ERROR);
    }
  }
}

// check if mower is stucked
// ToDo: take HDOP into consideration if gpsSpeed is reliable
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
    if (stateCurr != STATE_MANUAL && stateCurr != STATE_REMOTE
        && gpsSpeed <= stuckIfGpsSpeedBelow
        && odometer.encoder[LEFT].getWheelRpmCurr() != 0
        && odometer.encoder[RIGHT].getWheelRpmCurr() != 0
        && millis() > stateStartTime + gpsSpeedIgnoreTime)
    {
      robotIsStuckCounter++;
    }

    else
    { // if mower gets unstuck it resets errorCounterMax to zero and re-enables motorMow
      robotIsStuckCounter = 0;    // resets temporary counter to zero
      if (errorCounter[ERR_STUCK] == 0 && (stateCurr != STATE_OFF)
          && (stateCurr != STATE_MANUAL) && (stateCurr != STATE_STATION)
          && (stateCurr != STATE_STATION_CHARGING)
          && (stateCurr != STATE_STATION_CHECK)
          && (stateCurr != STATE_STATION_REV)
          && (stateCurr != STATE_STATION_ROLL) && (stateCurr != STATE_REMOTE)
          && (stateCurr != STATE_ERROR))
      {
        cutter.enable();
        errorCounterMax[ERR_STUCK] = 0;
      }
      return;
    }

    if (robotIsStuckCounter >= 5)
    {
      cutter.disable();
      if (errorCounterMax[ERR_STUCK] >= 3)
      {   // robot is definitely stuck and unable to move
        Console.println(F("Error: Mower is stuck"));
        addErrorCounter(ERR_STUCK);
        setNextState(STATE_ERROR);    //mower is switched into ERROR
        //robotIsStuckedCounter = 0;
      }
      else if (errorCounter[ERR_STUCK] < 3)
      {   // mower tries 3 times to get unstuck
        if (stateCurr == STATE_FORWARD)
        {
          cutter.disable();
          addErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0, false);
          reverseOrBidir(RIGHT);
        }
        else if (stateCurr == STATE_ROLL)
        {
          cutter.disable();
          addErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0, false);
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
  checkIfStuck();
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
          setNextState(STATE_STATION);
        }
      }
      imuDriveHeading = imu.getYaw();
      break;

    case STATE_REMOTE:
      // remote control mode (RC)
      //if (remoteSwitch > 50) setNextState(STATE_FORWARD, 0);
      steer = (int)(((double)wheels.wheel[Wheel::LEFT].motor.rpmMax / 2) * (((double)remoteSteer) / 100.0));
      if (remoteSpeed < 0)
      {
        steer *= -1;
      }
      wheels.wheel[Wheel::LEFT].motor.rpmSet =
          ((double)wheels.wheel[Wheel::LEFT].motor.rpmMax) * (((double)remoteSpeed) / 100.0) - steer;

      wheels.wheel[Wheel::RIGHT].motor.rpmSet =
          ((double)wheels.wheel[Wheel::RIGHT].motor.rpmMax) * (((double)remoteSpeed) / 100.0) + steer;

      wheels.wheel[Wheel::LEFT].motor.rpmSet =
          max(-wheels.wheel[Wheel::LEFT].motor.rpmMax,
              min(wheels.wheel[Wheel::LEFT].motor.rpmMax, wheels.wheel[Wheel::LEFT].motor.rpmSet));

      wheels.wheel[Wheel::RIGHT].motor.rpmSet =
          max(-wheels.wheel[Wheel::RIGHT].motor.rpmMax,
              min(wheels.wheel[Wheel::RIGHT].motor.rpmMax, wheels.wheel[Wheel::RIGHT].motor.rpmSet));

      cutter.motor.setPwmSet((double)cutter.motor.pwmMax * ((double)remoteMow / 100.0));
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
          setNextState(STATE_STATION);
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
        if ((chgCurrent < batFullCurrent) && curMillis - stateStartTime > 2000)
        {
          setNextState(STATE_STATION);
        }
        else if (curMillis - stateStartTime > chargingTimeout)
        {
          addErrorCounter(ERR_BATTERY);
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
          addErrorCounter(ERR_CHARGER);
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
  //if ((useRemoteRC) && (remoteSwitch < -50)) setNextState(STATE_REMOTE, 0);

  // decide which motor control to use
  if ((mowPatternCurr == MOW_LANES && stateCurr == STATE_ROLL) ||
      stateCurr  == STATE_ROLL_WAIT)
  {
    motorControlImuRoll();
  }
  else if (stateCurr == STATE_PERI_TRACK)
  {
    motorControlPerimeter();
  }
  else if (stateCurr == STATE_FORWARD && (imu.correctDir || mowPatternCurr == MOW_LANES))
  {
    motorControlImuDir();
  }
  else
  {
    motorControl();
  }

  if (stateCurr != STATE_REMOTE)
  {
    cutter.motor.setPwmSet(cutter.motor.pwmMax);
  }

  bumpers.clearHit();
  dropSensors.clearDetected();

  loopsPerSecCounter++;
}
