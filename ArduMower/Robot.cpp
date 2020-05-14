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

#define ADDR_USER_SETTINGS  0
#define ADDR_ERROR_COUNTERS 400
#define ADDR_ROBOT_STATS    800

#define OFF 0
#define ON  1

const char *mowPatternNames[] = { "RANDOM", "LANE", "BIDIR" };

const char* consoleModeNames[] =
{
    "sensor_counters", "sensor_values", "perimeter", "imu", "off"
};

const char* Robot::mowPatternName()
{
  return mowPatternNames[m_mowPattern];
}

const char* Robot::consoleModeName()
{
  return consoleModeNames[m_consoleMode];
}

void Robot::loadRobotStats()
{
  uint16_t addr = ADDR_ROBOT_STATS;

  Console.print(F("Loading RobotStats, address="));
  Console.print(addr);

  uint8_t magic = EEPROM.read(addr);
  addr++;

  if (magic != MAGIC)
  {
    Console.println(F("PLEASE CHECK IF YOUR ROBOT STATS ARE CORRECT"));
  }

  EEPROM.get(addr, m_stats);
  addr += sizeof(m_stats);

  Console.print('-');
  Console.println(addr);
}

void Robot::saveRobotStats()
{
  uint16_t addr = ADDR_ROBOT_STATS;

  Console.println(F("Saving RobotStats, address="));
  Console.print(addr);

  EEPROM.update(addr, MAGIC);
  addr++;

  EEPROM.put(addr, m_stats);
  addr += sizeof(m_stats);

  Console.print('-');
  Console.println(addr);
}

void Robot::loadErrorCounters()
{
  uint16_t addr = ADDR_ERROR_COUNTERS;

  Console.println(F("Loading ErrorCounters, address="));
  Console.print(addr);

  uint8_t magic = EEPROM.read(addr);
  addr++;

  if (magic == MAGIC)
  {
    EEPROM.get(addr, m_errorCounterMax);
    addr += sizeof(m_errorCounterMax);
    Console.print('-');
    Console.println(addr);
  }
  else
  {
    Console.println(F("EEPROM ERR COUNTERS: NO EEPROM ERROR DATA"));
    Console.println(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    incErrorCounter(ERR_EEPROM_DATA);
    setNextState(StateMachine::STATE_ERROR);
  }
}

void Robot::saveErrorCounters()
{
  uint16_t addr = ADDR_ERROR_COUNTERS;

  Console.println(F("Saving ErrorCounters, address="));
  Console.print(addr);

  EEPROM.update(addr, MAGIC);
  addr++;

  EEPROM.put(addr, m_errorCounterMax);
  addr += sizeof(m_errorCounterMax);

  Console.print('-');
  Console.println(addr);
}

void Robot::loadSaveUserSettingsPid(bool readflag, uint16_t& addr, Pid& pid)
{
  auto settings_p = pid.getSettings();

  eereadwrite(readflag, addr, settings_p->Kp.value);
  eereadwrite(readflag, addr, settings_p->Ki.value);
  eereadwrite(readflag, addr, settings_p->Kd.value);
}

void Robot::loadSaveUserSettingsBumpers(bool readflag, uint16_t& addr,
    Bumpers& bumpers)
{
  auto settings_p = bumpers.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
}

void Robot::loadSaveUserSettingsImu(bool readflag, uint16_t& addr, Imu& imu)
{
  auto settings_p = imu.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
  eereadwrite(readflag, addr, settings_p->correctDir.value);

  loadSaveUserSettingsPid(readflag, addr, imu.m_pid[Imu::DIR]);
  loadSaveUserSettingsPid(readflag, addr, imu.m_pid[Imu::ROLL]);
}

void Robot::loadSaveUserSettingsOdometer(bool readflag, uint16_t& addr,
    Odometer& odometer)
{
  auto settings_p = odometer.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
  eereadwrite(readflag, addr, settings_p->ticksPerRevolution.value);
  eereadwrite(readflag, addr, settings_p->ticksPerCm.value);
  eereadwrite(readflag, addr, settings_p->wheelBaseCm.value);

  eereadwrite(readflag, addr, odometer.m_encoder.left.m_swapDir);
  eereadwrite(readflag, addr, odometer.m_encoder.right.m_swapDir);
}

void Robot::loadSaveUserSettingsLawnSensors(bool readflag, uint16_t& addr,
    LawnSensors& lawnSensors)
{
  auto settings_p = lawnSensors.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
}

void Robot::loadSaveUserSettingsRainSensor(bool readflag, uint16_t& addr,
    RainSensor& rainSensor)
{
  auto settings_p = rainSensor.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
}

void Robot::loadSaveUserSettingsSonar(bool readflag, uint16_t& addr,
    Sonar& sonar)
{
  auto settings_p = sonar.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
}

void Robot::loadSaveUserSettingsSonars(bool readflag, uint16_t& addr,
    Sonars& sonars)
{
  auto settings_p = sonars.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
  eereadwrite(readflag, addr, settings_p->triggerBelow.value);

  for (uint8_t i = 0; i < SONARS_NUM; i++)
  {
    loadSaveUserSettingsSonar(readflag, addr, m_sonars.m_sonarArray_p[i]);
  }
}

void Robot::loadSaveUserSettingsPerimeter(bool readflag, uint16_t& addr,
    Perimeter& perimeter)
{
  auto settings_p = perimeter.getSettings();

  eereadwrite(readflag, addr, settings_p->timedOutIfBelowSmag.value);
  eereadwrite(readflag, addr, settings_p->useDifferentialPerimeterSignal.value);
  eereadwrite(readflag, addr, settings_p->swapCoilPolarity.value);
  eereadwrite(readflag, addr, settings_p->timeOutSecIfNotInside.value);

  loadSaveUserSettingsPid(readflag, addr, perimeter.m_pid);
}

void Robot::loadSaveUserSettingsPerimeters(bool readflag, uint16_t& addr,
    Perimeters& perimeters)
{
  auto settings_p = perimeters.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);

  loadSaveUserSettingsPerimeter(readflag, addr,
      perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)]);
}

void Robot::loadSaveUserSettingsBattery(bool readflag, uint16_t& addr,
    Battery& battery)
{
  auto settings_p = battery.getSettings();

  eereadwrite(readflag, addr, settings_p->monitored.value);
  eereadwrite(readflag, addr, settings_p->batGoHomeIfBelow_mV.value);
  eereadwrite(readflag, addr, settings_p->batSwitchOffIfBelow_mV.value);
  eereadwrite(readflag, addr, settings_p->batSwitchOffIfIdle_min.value);
  eereadwrite(readflag, addr, settings_p->batFactor_mV_per_LSB.value);
  eereadwrite(readflag, addr, settings_p->batChargeFactor_mV_per_LSB.value);
  eereadwrite(readflag, addr, settings_p->chgSenseZero.value);
  eereadwrite(readflag, addr, settings_p->chgFactor.value);
  eereadwrite(readflag, addr, settings_p->batFullCurrent_mA.value);
  eereadwrite(readflag, addr, settings_p->startChargingIfBelow_mV.value);
}

void Robot::loadSaveUserSettingsDropSensors(bool readflag, uint16_t& addr,
    DropSensors& dropSensor)
{
  auto settings_p = dropSensor.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
}

void Robot::loadSaveUserSettingsButton(bool readflag, uint16_t& addr,
    Button& button)
{
  auto settings_p = button.getSettings();

  eereadwrite(readflag, addr, settings_p->use.value);
}

void Robot::loadSaveUserSettingsWheels(bool readflag, uint16_t& addr,
    Wheels& wheels)
{
  //auto settings_p = wheels.getSettings();

  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_acceleration);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_pwmMax);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_powerMax);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_scale);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_powerIgnoreTime);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_swapDir);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::RIGHT].m_motor.m_scale);
  eereadwrite(readflag, addr, wheels.m_wheel[Wheel::RIGHT].m_motor.m_swapDir);
  eereadwrite(readflag, addr, wheels.m_rollTimeMax);
  eereadwrite(readflag, addr, wheels.m_rollTimeMin);
  eereadwrite(readflag, addr, wheels.m_reverseTime);
  eereadwrite(readflag, addr, wheels.m_forwardTimeMax);
  eereadwrite(readflag, addr, wheels.m_biDirSpeedRatio1);
  eereadwrite(readflag, addr, wheels.m_biDirSpeedRatio2);

  loadSaveUserSettingsPid(readflag, addr, wheels.m_wheel[Wheel::LEFT].m_motor.m_pid);
}

void Robot::loadSaveUserSettingsCutter(bool readflag, uint16_t& addr,
    Cutter& cutter)
{
  //auto settings_p = wheels.getSettings();

  eereadwrite(readflag, addr, cutter.m_motor.m_pwmMax);
  eereadwrite(readflag, addr, cutter.m_motor.m_powerMax);
  eereadwrite(readflag, addr, cutter.m_motor.m_rpmSet);
  eereadwrite(readflag, addr, cutter.m_motor.m_scale);

  loadSaveUserSettingsPid(readflag, addr, cutter.m_motor.m_pid);
}

void Robot::loadSaveUserSettingsRobot(bool readflag, uint16_t& addr)
{
  eereadwrite(readflag, addr, m_settings.developer.value);

  eereadwrite(readflag, addr, m_settings.perimeterTriggerTimeout.value);
  eereadwrite(readflag, addr, m_settings.perimeterOutRollTimeMax.value);
  eereadwrite(readflag, addr, m_settings.perimeterOutRollTimeMin.value);
  eereadwrite(readflag, addr, m_settings.perimeterOutRevTime.value);
  eereadwrite(readflag, addr, m_settings.perimeterTrackRollTime.value);
  eereadwrite(readflag, addr, m_settings.perimeterTrackRevTime.value);

  eereadwrite(readflag, addr,
      m_settings.trackingBlockInnerWheelWhilePerimeterStruggling.value);

  eereadwrite(readflag, addr, m_settings.stationRevTime.value);
  eereadwrite(readflag, addr, m_settings.stationRollTime.value);
  eereadwrite(readflag, addr, m_settings.stationForwTime.value);
  eereadwrite(readflag, addr, m_settings.stationCheckTime.value);

  eereadwrite(readflag, addr, m_settings.userSwitch1.value);
  eereadwrite(readflag, addr, m_settings.userSwitch2.value);
  eereadwrite(readflag, addr, m_settings.userSwitch3.value);

  eereadwrite(readflag, addr, m_settings.timerUse.value);
  for (uint8_t i = 0; i < MAX_TIMERS; i++)
  {
    eereadwrite(readflag, addr, m_settings.timer[i].active.value);
    eereadwrite(readflag, addr, m_settings.timer[i].startTime.hour.value);
    eereadwrite(readflag, addr, m_settings.timer[i].startTime.minute.value);
    eereadwrite(readflag, addr, m_settings.timer[i].stopTime.hour.value);
    eereadwrite(readflag, addr, m_settings.timer[i].stopTime.minute.value);
    eereadwrite(readflag, addr, m_settings.timer[i].daysOfWeek.value);
  }

  eereadwrite(readflag, addr, m_settings.gpsUse.value);
  eereadwrite(readflag, addr, m_settings.stuckIfGpsSpeedBelow.value);
  eereadwrite(readflag, addr, m_settings.gpsSpeedIgnoreTime.value);
}

void Robot::loadSaveUserSettings(bool readflag)
{
  uint16_t addr = ADDR_USER_SETTINGS + 1;

  loadSaveUserSettingsRobot(readflag, addr);
  loadSaveUserSettingsWheels(readflag, addr, m_wheels);
  loadSaveUserSettingsCutter(readflag, addr, m_cutter);
  loadSaveUserSettingsBumpers(readflag, addr, m_bumpers);
  loadSaveUserSettingsSonars(readflag, addr, m_sonars);
  loadSaveUserSettingsPerimeters(readflag, addr, m_perimeters);
  loadSaveUserSettingsLawnSensors(readflag, addr, m_lawnSensors);
  loadSaveUserSettingsImu(readflag, addr, m_imu);
  loadSaveUserSettingsBattery(readflag, addr, m_battery);
  loadSaveUserSettingsOdometer(readflag, addr, m_odometer);
  loadSaveUserSettingsButton(readflag, addr, m_button);
  loadSaveUserSettingsRainSensor(readflag, addr, m_rainSensor);
  loadSaveUserSettingsDropSensors(readflag, addr, m_dropSensors);

  Console.print('-');
  Console.println(addr);
}

void Robot::loadUserSettings()
{
  uint16_t addr = ADDR_USER_SETTINGS;

  Console.println(F("USER SETTINGS ARE LOADED, address="));
  Console.print(addr);

  uint8_t magic = EEPROM.read(addr);
  addr++;

  if (magic == MAGIC)
  {
    loadSaveUserSettings(true);
  }
  else
  {
    Console.println(F("EEPROM USERDATA: NO EEPROM USER DATA"));
    Console.println(F("PLEASE CHECK AND SAVE YOUR SETTINGS"));
    incErrorCounter(ERR_EEPROM_DATA);
    setNextState(StateMachine::STATE_ERROR);
  }
}

void Robot::saveUserSettings()
{
  uint16_t addr = ADDR_USER_SETTINGS;

  Console.println(F("USER SETTINGS ARE SAVED, address="));
  Console.print(addr);

  EEPROM.update(addr, MAGIC);
  addr++;

  loadSaveUserSettings(false);
}

void Robot::deleteUserSettings()
{
  loadRobotStats();

  uint16_t addr = ADDR_USER_SETTINGS;

  Console.println(F("ALL USER SETTINGS ARE DELETED"));

  EEPROM.update(addr, 0); // clear magic

  saveRobotStats();
}

template <class T>
void Robot::printSettingNameColonValue(const Setting<T>& K)
{
  Console.print(K.name);
  Console.print(F(" : "));
  Console.println(K.value);
}

void Robot::printSettingSerialPidK(const __FlashStringHelper* prefixStr,
    Setting<float> K)
{
  Console.print(prefixStr);
  printSettingNameColonValue(K);
}

void Robot::printSettingSerialPid(const __FlashStringHelper* prefixStr,
    PidSettings* pidSettings_p)
{
  printSettingSerialPidK(prefixStr, pidSettings_p->Kp);
  printSettingSerialPidK(prefixStr, pidSettings_p->Ki);
  printSettingSerialPidK(prefixStr, pidSettings_p->Kd);
}

void Robot::printSettingSerialWheelMotors()
{
  auto motorLeft_p = &m_wheels.m_wheel[Wheel::LEFT].m_motor;
  auto motorRight_p = &m_wheels.m_wheel[Wheel::RIGHT].m_motor;

  Console.println(F("== Wheel motors =="));
  Console.print(F("acceleration : "));
  Console.println(motorLeft_p->m_acceleration);
  Console.print(F("rpmMax : "));
  Console.println(motorLeft_p->m_rpmMax);
  Console.print(F("pwmMax : "));
  Console.println(motorLeft_p->m_pwmMax);
  Console.print(F("powerMax : "));
  Console.println(motorLeft_p->m_powerMax);
  Console.print(F("LEFT.scale : "));
  Console.println(motorLeft_p->getScale());
  Console.print(F("RIGHT.scale : "));
  Console.println(motorRight_p->getScale());
  Console.print(F("powerIgnoreTime : "));
  Console.println(motorLeft_p->m_powerIgnoreTime);
  Console.print(F("zeroSettleTime : "));
  Console.println(motorLeft_p->m_zeroSettleTime);

  Console.print(F("rollTimeMax : "));
  Console.println(m_wheels.m_rollTimeMax);
  Console.print(F("rollTimeMin : "));
  Console.println(m_wheels.m_rollTimeMin);
  Console.print(F("reverseTime : "));
  Console.println(m_wheels.m_reverseTime);
  Console.print(F("forwardTimeMax : "));
  Console.println(m_wheels.m_forwardTimeMax);
  Console.print(F("biDirSpeedRatio1 : "));
  Console.println(m_wheels.m_biDirSpeedRatio1);
  Console.print(F("biDirSpeedRatio2 : "));
  Console.println(m_wheels.m_biDirSpeedRatio2);

  printSettingSerialPid(F("LEFT.pid."), motorLeft_p->m_pid.getSettings());
  printSettingSerialPid(F("RIGHT.pid."), motorRight_p->m_pid.getSettings());

  Console.print(F("LEFT.swapDir : "));
  Console.println(motorLeft_p->m_swapDir);
  Console.print(F("RIGHT.swapDir : "));
  Console.println(motorRight_p->m_swapDir);
}

void Robot::printSettingSerialCutterMotor()
{
  Console.println(F("== Cutter motor =="));
  Console.print(F("acceleration : "));
  Console.println(m_cutter.m_motor.m_acceleration);
  Console.print(F("pwmMax : "));
  Console.println(m_cutter.m_motor.m_pwmMax);
  Console.print(F("powerMax : "));
  Console.println(m_cutter.m_motor.m_powerMax);
  Console.print(F("regulate : "));
  Console.println(m_cutter.m_motor.m_regulate);
  Console.print(F("rpmSet : "));
  Console.println(m_cutter.m_motor.m_rpmSet);
  Console.print(F("scale : "));
  Console.println(m_cutter.m_motor.getScale());

  printSettingSerialPid(F("pid."), m_cutter.m_motor.m_pid.getSettings());
}

void Robot::printSettingSerialBumper()
{
  Console.println(F("== Bumpers =="));
  printSettingNameColonValue(m_bumpers.getSettings()->use);
}

void Robot::printSettingSerialDropSensors()
{
  Console.println(F("== Drop sensors =="));
  Console.print(F("use : "));
  Console.println(m_dropSensors.isUsed());
  Console.print(F("contactType : "));
  Console.println(m_dropSensors.getContactType() == DropSensor_Contact::NC ?
          F("NC") : F("NO"));
}

void Robot::printSettingSerialRainSensor()
{
  Console.println(F("== Rain sensor =="));
  printSettingNameColonValue(m_rainSensor.getSettings()->use);
}

void Robot::printSettingSerialSonars()
{
  Console.println(F("== Sonars =="));

  auto settings_p = m_sonars.getSettings();

  printSettingNameColonValue(settings_p->use);

  Console.print(F("LEFT."));
  printSettingNameColonValue(
      m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::LEFT)].
      getSettings()->use);

  Console.print(F("CENTER."));
  printSettingNameColonValue(
      m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::CENTER)].
      getSettings()->use);

  Console.print(F("RIGHT."));
  printSettingNameColonValue(
      m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::RIGHT)].
      getSettings()->use);

  printSettingNameColonValue(settings_p->triggerBelow);
}

void Robot::printSettingSerialPerimeters()
{
  Console.println(F("== Perimeter =="));

  printSettingNameColonValue(m_perimeters.getSettings()->use);

  printSettingNameColonValue(m_settings.perimeterTriggerTimeout);
  printSettingNameColonValue(m_settings.perimeterOutRollTimeMax);
  Console.print(F("outRollTimeMin : "));
  Console.println(m_perimeterOutRollTimeMin);
  printSettingNameColonValue(m_settings.perimeterOutRevTime);
  printSettingNameColonValue(m_settings.perimeterTrackRollTime);
  Console.print(F("trackRevTime : "));
  Console.println(m_perimeterTrackRevTime);

  printSettingSerialPid(F("pid."),
      m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)].m_pid.getSettings());

  Console.print(F("trackingPerimeterTransitionTimeOut : "));
  Console.println(m_trackingPerimeterTransitionTimeOut);
  Console.print(F("trackingErrorTimeOut : "));
  Console.println(m_trackingErrorTimeOut);
  Console.print(F("trackingBlockInnerWheelWhilePerimeterStruggling : "));
  Console.println(m_trackingBlockInnerWheelWhilePerimeterStruggling);
}

void Robot::printSettingSerialLawnSensor()
{
  Console.println(F("== Lawn sensor =="));

  auto settings_p = m_lawnSensors.getSettings();

  printSettingNameColonValue(settings_p->use);
}

void Robot::printSettingSerialImu()
{
  Console.println(F("== IMU =="));

  auto settings_p = m_imu.getSettings();

  printSettingNameColonValue(settings_p->use);
  printSettingNameColonValue(settings_p->correctDir);

  printSettingSerialPid(F("pid[DIR]."), m_imu.m_pid[Imu::DIR].getSettings());
  printSettingSerialPid(F("pid[ROLL]."), m_imu.m_pid[Imu::ROLL].getSettings());
}

void Robot::printSettingSerialBattery()
{
  Console.println(F("== Battery =="));

  auto* settings_p = m_battery.getSettings();

  printSettingNameColonValue(settings_p->monitored);
  printSettingNameColonValue(settings_p->batGoHomeIfBelow_mV);
  printSettingNameColonValue(settings_p->batSwitchOffIfBelow_mV);
  printSettingNameColonValue(settings_p->batSwitchOffIfIdle_min);
  printSettingNameColonValue(settings_p->batFactor_mV_per_LSB);
  printSettingNameColonValue(settings_p->batChargeFactor_mV_per_LSB);
  printSettingNameColonValue(settings_p->batFull_mV);
  printSettingNameColonValue(settings_p->batChargingCurrentMax_mA);
  printSettingNameColonValue(settings_p->batFullCurrent_mA);
  printSettingNameColonValue(settings_p->startChargingIfBelow_mV);

  Console.print(F("chargingTimeout : "));
  Console.println(m_battery.m_chargingTimeout_ms);

  printSettingNameColonValue(settings_p->chgSenseZero);
  printSettingNameColonValue(settings_p->chgFactor);
}

void Robot::printSettingSerialStation()
{
  Console.println(F("== Station =="));
  Console.print(F("reverseTime : "));
  Console.println(m_stationRevTime);
  Console.print(F("rollTime : "));
  Console.println(m_stationRollTime);
  Console.print(F("forwardTime : "));
  Console.println(m_stationForwTime);
  Console.print(F("checkTime : "));
  Console.println(m_stationCheckTime);
}

void Robot::printSettingSerialOdometer()
{
  Console.println(F("== Odometer =="));

  auto settings_p = m_odometer.getSettings();

  printSettingNameColonValue(settings_p->use);
  printSettingNameColonValue(settings_p->ticksPerRevolution);
  printSettingNameColonValue(settings_p->ticksPerCm);
  printSettingNameColonValue(settings_p->wheelBaseCm);

  Console.print(F("LEFT.swapDir : "));
  Console.println(m_odometer.m_encoder.left.m_swapDir);

  Console.print(F("RIGHT.swapDir : "));
  Console.println(m_odometer.m_encoder.right.m_swapDir);
}

void Robot::printSettingSerialGps()
{
  Console.println(F("== GPS =="));
  printSettingNameColonValue(m_settings.gpsUse);
  Console.print(F("stuckIfGpsSpeedBelow : "));
  Console.println(m_stuckIfGpsSpeedBelow);
  Console.print(F("gpsSpeedIgnoreTime : "));
  Console.println(m_gpsSpeedIgnoreTime);
}

void Robot::printSettingSerialother()
{
  Console.println(F("== Button =="));
  printSettingNameColonValue(m_button.getSettings()->use);
}

void Robot::printSettingSerialUserSwitches()
{
  Console.println(F("== User switches =="));
  Console.print(F("userSwitch1 : "));
  Console.println(m_userSwitch1);
  Console.print(F("userSwitch2 : "));
  Console.println(m_userSwitch2);
  Console.print(F("userSwitch3 : "));
  Console.println(m_userSwitch3);
}

void Robot::printSettingSerialTimer()
{
  Console.println(F("== Timer =="));
  printSettingNameColonValue(m_settings.timerUse);
}

void Robot::printSettingSerialStatus()
{
  Console.println(F("== Robot status =="));
  Console.print(F("Mowing time, trip [min] : "));
  Console.println(m_stats.mowTimeTrip_min);
  Console.print(F("Mowing time, total [min] : "));
  Console.println(m_stats.mowTimeTotal_min);
  Console.print(F("batteryChargingCounterTotal : "));
  Console.println(m_stats.batteryChargingCounterTotal);
  Console.print(F("batteryChargingCapacityTrip [mAh] : "));
  Console.println(m_stats.batteryChargingCapacityTrip_mAh);
  Console.print(F("batteryChargingCapacityTotal [Ah] : "));
  Console.println(m_stats.batteryChargingCapacityTotal_mAh / 1000);
  Console.print(F("batteryChargingCapacityAverage [mAh] : "));
  Console.println(m_stats.batteryChargingCapacityAverage_mAh);
}

void Robot::printSettingSerial()
{
  printSettingSerialWheelMotors();
  printSettingSerialCutterMotor();
  printSettingSerialBumper();
  printSettingSerialDropSensors();
  printSettingSerialRainSensor();
  printSettingSerialSonars();
  printSettingSerialPerimeters();
  printSettingSerialLawnSensor();
  printSettingSerialImu();
  printSettingSerialBattery();
  printSettingSerialStation();
  printSettingSerialOdometer();
  printSettingSerialGps();
  printSettingSerialother();
  printSettingSerialUserSwitches();
  printSettingSerialTimer();
  printSettingSerialStatus();
}

void Robot::deleteRobotStats()
{
  memset(&m_stats, 0, sizeof(m_stats));
  saveRobotStats();
  Console.println(F("ALL ROBOT STATS ARE DELETED"));
}

void Robot::incErrorCounter(const enum ErrorE errType)
{
  // increase error counters (both temporary and maximum error counters)
  if (m_errorCounter[errType] < 255)
  {
    ++m_errorCounter[errType];
  }

  if (m_errorCounterMax[errType] < 255)
  {
    ++m_errorCounterMax[errType];
  }
}

void Robot::resetErrorCounters()
{
  Console.println(F("Reset Error Counters"));
  memset(m_errorCounter, 0, sizeof(m_errorCounter));
  memset(m_errorCounterMax, 0, sizeof(m_errorCounterMax));
  saveErrorCounters();
}

void Robot::checkErrorCounter()
{
  unsigned long curMillis = millis();
  if (curMillis >= m_nextTimeErrorCounterReset)
  {
    // reset all temporary error counters after 30 seconds
    // (maximum error counters still continue to count)
    memset(m_errorCounter, 0, sizeof(m_errorCounter));
    m_nextTimeErrorCounterReset = curMillis + 30000; // 30 sec
  }

  if (!m_stateMachine.isCurrentState(StateMachine::STATE_OFF))
  {
    for (uint8_t i = 0; i < ERR_ENUM_COUNT; i++)
    {
      // set to fatal error if any temporary error counter reaches 10
      if (m_errorCounter[i] > 10)
      {
        setNextState(StateMachine::STATE_ERROR);
      }
    }
  }
}

//motor is LEFT or RIGHT (0 or 1)
void Robot::setMotorPWM(int pwm,
                        const uint8_t motor,
                        const bool useAccel)
{
  const uint16_t samplingTime = m_wheels.m_wheel[motor].m_motor.getSamplingTime();

  if (useAccel)
  {
    // http://phrogz.net/js/framerate-independent-low-pass-filter.html
    // smoothed += elapsedTime * ( newValue - smoothed ) / smoothing;
    m_wheels.m_wheel[motor].m_motor.m_pwmCur +=
        samplingTime * (pwm - m_wheels.m_wheel[motor].m_motor.m_pwmCur) / m_wheels.m_wheel[motor].m_motor.m_acceleration;
  }

  // ----- driver protection (avoids driver explosion) ----------
  if ((pwm < 0 && m_wheels.m_wheel[motor].m_motor.m_pwmCur >= 0) || (pwm > 0 && m_wheels.m_wheel[motor].m_motor.m_pwmCur <= 0))
  { // changing direction should take place?
    if (m_wheels.m_wheel[motor].m_motor.getZeroTimeout() > 0)
    {
      pwm = m_wheels.m_wheel[motor].m_motor.m_pwmCur
          - m_wheels.m_wheel[motor].m_motor.m_pwmCur * (float)samplingTime / 200.0; // reduce speed
    }
  }

  if (m_odometer.isUsed())
  {
    m_wheels.m_wheel[motor].m_motor.m_pwmCur = pwm;
    if (abs(m_wheels.m_wheel[motor].m_encoder.getWheelRpmCurr()) == 0)
    {
      m_wheels.m_wheel[motor].m_motor.setZeroTimeout(
          (uint16_t)max(0, (int32_t)m_wheels.m_wheel[motor].m_motor.getZeroTimeout() -
                           (int32_t)samplingTime));
    }
    else
    {
      m_wheels.m_wheel[motor].m_motor.setZeroTimeout(500);
    }
  }
  else
  {
    if (pwm == 0)
    {
      m_wheels.m_wheel[motor].m_motor.setZeroTimeout(
          (uint16_t)max(0, (int32_t)m_wheels.m_wheel[motor].m_motor.getZeroTimeout() -
                           (int32_t)samplingTime));
    }
    else
    {
      m_wheels.m_wheel[motor].m_motor.setZeroTimeout(700);
    }
  }

  m_wheels.m_wheel[motor].m_motor.setSpeed();
}

// sets wheel motor actuators
// - driver protection: delays polarity change until motor speed (EMV) is zero
//   http://wiki.ardumower.de/images/a/a5/Motor_polarity_switch_protection.png
// - optional: ensures that the motors (and gears) are not switched to 0%
//   (or 100%) too fast (motorAccel)
void Robot::setMotorPWMs(const int pwmLeft, int const pwmRight,
    const bool useAccel)
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
  // Control range corresponds to 80 % of maximum speed on the drive wheel
  Pid* pid_p = &m_imu.m_pid[Imu::ROLL];
  float processValue = -distancePI(m_imu.getYaw(), m_imuRollHeading) / PI * 180.0;
  float y = pid_p->compute(processValue);

  if ((m_stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
       m_stateMachine.isCurrentState(StateMachine::STATE_STATION) ||
       m_stateMachine.isCurrentState(StateMachine::STATE_ERROR)) &&
      (millis() - m_stateMachine.getStateStartTime() > 1000))
  {
    m_wheels.setSpeed(0);
    m_wheels.setSteer(0);
  }
  else
  {
    m_wheels.setSpeed(0);
    m_wheels.setSteer(round(y));
  }
}

// PID controller: track perimeter
void Robot::wheelControl_perimeter()
{
  unsigned long curMillis = millis();
  if ((curMillis > m_stateMachine.getStateStartTime() + 5000) &&
      (curMillis > m_perimeterLastTransitionTime + m_trackingPerimeterTransitionTimeOut))
  {
    // Robot is wheel-spinning while perimeter tracking => roll to get ground again

    if (m_trackingBlockInnerWheelWhilePerimeterStruggling)
    {
      // Block inner wheel and outer wheel at half speed
      m_wheels.setSpeed(25);
      m_wheels.setSteer(m_perimeterMag < 0 ? -25 : 25);
    }
    else
    {
      // Turn on the spot at half speed
      m_wheels.setSpeed(0);
      m_wheels.setSteer(m_perimeterMag < 0 ? -50 : 50);
    }

    if (millis() > m_perimeterLastTransitionTime + m_trackingErrorTimeOut)
    {
      Console.println("Error: Tracking error");
      incErrorCounter(ERR_TRACKING);
      //setNextState(StateMachine::STATE_ERROR,0);
      setNextState(StateMachine::STATE_PERI_FIND);
    }
  }
  else
  {
    // Normal perimeter tracking

    Pid* pid_p = &m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)].m_pid;
    float y = pid_p->compute(sign(m_perimeterMag));

    m_wheels.setSpeed(m_speed);
    m_wheels.setSteer(round(y));
  }
}

// PID controller: correct direction during normal driving (requires IMU)
void Robot::wheelControl_imuDir()
{
  // Control range corresponds to the steer range
  Pid* pid_p = &m_imu.m_pid[Imu::DIR];
  float processValue = -distancePI(m_imu.getYaw(), m_imuDriveHeading) / PI * 180.0;
  float y = pid_p->compute(processValue);

  if ((m_stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
       m_stateMachine.isCurrentState(StateMachine::STATE_STATION) ||
       m_stateMachine.isCurrentState(StateMachine::STATE_ERROR)) &&
      (millis() > m_stateMachine.getStateStartTime() + 1000))
  {
    m_wheels.setSpeed(0);
    m_wheels.setSteer(0);
  }
  else
  {
    m_wheels.setSpeed(m_speed);
    m_wheels.setSteer(round(y));
  }
}

void Robot::wheelControl_normal()
{
  // Use wheel motor PID-regulator if we use odometer
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_regulate = m_odometer.isUsed();

  int zeroSettleTime = m_wheels.m_wheel[Wheel::LEFT].m_motor.m_zeroSettleTime;
  if (millis() < m_stateMachine.getStateStartTime() + zeroSettleTime)
  {
    // Start from zero speed and zero steer at state start
    m_wheels.setSpeed(0);
    m_wheels.setSteer(0);

    if (m_mowPattern != MOW_LANES)
    {
      m_imuDriveHeading = m_imu.getYaw(); // set drive heading
    }
  }
  else
  {
    // No correction, set wheel values to same as robot values
    m_wheels.setSpeed(m_speed);
    m_wheels.setSteer(m_steer);
  }
}

// check for odometer sensor faults
void Robot::checkOdometerFaults()
{
  if (!m_odometer.isUsed())
  {
    return;
  }

  bool err[2] = { false };
  unsigned long curMillis = millis();

  if (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD) &&
      (curMillis - m_stateMachine.getStateStartTime() > 8000))
  {
    // just check if odometer sensors may not be working at all
    for (uint8_t i = LEFT; i <= RIGHT; i++)
    {
      if (m_wheels.m_wheel[i].m_motor.getPwmCur() > 100 &&
          abs(m_wheels.m_wheel[i].m_encoder.getWheelRpmCurr()) == 0)
      {
        err[i] = true;
      }
    }
  }

  if (m_stateMachine.isCurrentState(StateMachine::STATE_ROLL) &&
      (curMillis - m_stateMachine.getStateStartTime()) > 1000)
  {
    // just check if odometer sensors may be turning in the wrong direction
    for (uint8_t i = LEFT; i <= RIGHT; i++)
    {
      int16_t pwmCur = m_wheels.m_wheel[i].m_motor.getPwmCur();
      int16_t wheelRpmCurr = m_wheels.m_wheel[i].m_encoder.getWheelRpmCurr();
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
    Console.print(m_wheels.m_wheel[Wheel::LEFT].m_motor.getPwmCur());
    Console.print("\tRPM=");
    Console.println(m_wheels.m_wheel[Wheel::LEFT].m_encoder.getWheelRpmCurr());
    incErrorCounter(ERR_ODOMETER_LEFT);
    setNextState(StateMachine::STATE_ERROR);
  }

  if (err[RIGHT])
  {
    Console.print("Right odometer error: PWM=");
    Console.print(m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPwmCur());
    Console.print("\tRPM=");
    Console.println(m_wheels.m_wheel[Wheel::RIGHT].m_encoder.getWheelRpmCurr());
    incErrorCounter(ERR_ODOMETER_RIGHT);
    setNextState(StateMachine::STATE_ERROR);
  }
}

// Cutter motor speed controller (slowly adjusts output speed to set speed)
void Robot::cutterControl()
{
  if (m_cutter.isEnableOverriden())
  {
    m_cutter.disable();
  }
  m_cutter.control();
}

void Robot::resetIdleTime()
{
  // battery switched off?
  if (m_idleTimeSec == BATTERY_SW_OFF)
  {
    Console.println(F("BATTERY switching ON again"));
    m_battery.setBatterySwitch(ON); // switch on battery again (if connected via USB)
  }
  m_idleTimeSec = 0;
}

void Robot::setUserSwitches()
{
  digitalWrite(PIN_USER_SWITCH_1, m_userSwitch1);
  digitalWrite(PIN_USER_SWITCH_2, m_userSwitch1);
  digitalWrite(PIN_USER_SWITCH_3, m_userSwitch1);
}

void Robot::setDefaultTime()
{
  m_datetime.time.hour = 0;
  m_datetime.time.minute = 0;
  m_datetime.date.dayOfWeek = 3;
  m_datetime.date.day = 1;
  m_datetime.date.month = 1;
  m_datetime.date.year = 2020;
}

void Robot::setup()
{
  setDefaultTime();
  setMotorPWMs(0, 0);
  loadErrorCounters();
  loadUserSettings();
  loadRobotStats();
  setUserSwitches();

  if (!m_button.isUsed())
  {
    // robot has no ON/OFF button => start immediately
    setNextState(StateMachine::STATE_FORWARD);
  }

  delay(2000);
  m_stateMachine.init();
  m_buzzer.beepLong();

  Console.println(F("START"));
  Console.print(F("Ardumower "));
  Console.println(VERSION);
#ifdef USE_DEVELOPER_TEST
  Console.println(F("Warning: USE_DEVELOPER_TEST activated"));
#endif
#ifdef USE_BARKER_CODE
  Console.println(F("Warning: USE_BARKER_CODE activated"));
#endif
  Console.print(F("Config: "));
  Console.println(m_name);
  Console.println(F("press..."));
  Console.println(F("  d for menu"));
  Console.println(F("  v to change console output @"
                    "(sensor counters, sensor values, perimeter, off)"));
  Console.println(consoleModeName());

  Wire.begin();
  Console.begin(BAUDRATE);
  Console.println("SETUP");
  m_rc.initSerial(PFOD_BAUDRATE);

  // ------ perimeter ---------------------------------
  m_perimeters.m_perimeterArray_p[LEFT].setup(PIN_PERIMETER_LEFT);
  m_perimeters.m_perimeterArray_p[RIGHT].setup(PIN_PERIMETER_RIGHT);

  m_perimeters.m_perimeterArray_p[LEFT].m_pid.setup(
      51.0, 12.5, 0.8, -100.0, 100.0, 100.0, 0.1, 100);
  m_perimeters.m_perimeterArray_p[LEFT].m_pid.setSetPoint(0);
  //m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::RIGHT)].m_pid.setup(
  //    51.0, 12.5, 0.8, -100.0, 100.0, 100.0, 0.1, 100);


  // ------  IMU (compass/accel/gyro) ----------------------
  m_imu.m_pid[Imu::DIR].setup(5.0, 1.0, 1.0, -100.0, 100.0, 100.0, 0.1, 20);
  m_imu.m_pid[Imu::DIR].setSetPoint(0);
  m_imu.m_pid[Imu::ROLL].setup(0.8, 21.0, 0.0, -80.0, 80.0, 80.0, 0.1, 30);
  m_imu.m_pid[Imu::ROLL].setSetPoint(0);

  // ------- wheel motors -----------------------------
  m_wheels.m_rollTimeMax = 1500;      // max. roll time (ms)
  m_wheels.m_rollTimeMin = 750;       // min. roll time (ms) should be smaller than motorRollTimeMax
  m_wheels.m_reverseTime = WHEELS_MAX_REVERSE_TIME_MS;      // max. reverse time (ms)
  m_wheels.m_forwardTimeMax = 80000;  // max. forward time (ms) / timeout
  m_wheels.m_biDirSpeedRatio1 = 0.3;  // bidir mow pattern speed ratio 1
  m_wheels.m_biDirSpeedRatio2 = 0.92; // bidir mow pattern speed ratio 2

  // left wheel motor
  m_wheels.m_wheel[Wheel::LEFT].m_motor.config(1000.0,   // Acceleration
                                         255,      // Max PWM
                                         75,       // Max Power
                                         false,    // Regulate
                                         100,      // Max RPM
                                         0.0);     // Set RPM
  m_wheels.m_wheel[Wheel::LEFT].m_motor.setScale(3.25839);
  m_wheels.m_wheel[Wheel::LEFT].m_motor.setChannel(0);
  m_wheels.m_wheel[Wheel::LEFT].m_motor.setup();
  // Normal control
  int16_t pwmMax = m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pwmMax;
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid.setup(
      1.5, 0.29, 0.25, -pwmMax, pwmMax, pwmMax, 0.01, 3.0);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid.setSetPoint(
      m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet);
  // Fast control
  //wheels.wheel[Wheel::LEFT].motor.pid.setup(1.76, 0.87, 0.4);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_powerIgnoreTime = 2000;  // time to ignore motor power (ms)
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_zeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  m_wheels.m_wheel[Wheel::LEFT].m_motor.m_swapDir = 0;  // inverse left motor direction?
  m_wheels.m_wheel[Wheel::LEFT].m_encoder.setup(
      PIN_ODOMETER_LEFT, ODOMETER_SWAP_DIR_LEFT);

  // right wheel motor
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.config(1000.0,   // Acceleration
                                          255,      // Max PWM
                                          75,       // Max Power
                                          false,    // Regulate
                                          100,      // Max RPM
                                          0.0);     // Set RPM
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.setScale(3.25839);
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.setChannel(1);
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.setup();
  // Normal control
  pwmMax = m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pwmMax;
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid.setup(
      1.5, 0.29, 0.25, -pwmMax, pwmMax, pwmMax, 0.01, 3.0);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid.setSetPoint(
      m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet);
  // Fast control
  //wheels.wheel[Wheel::RIGHT].motor.pid.setup(1.76, 0.87, 0.4);  // Kp, Ki, Kd
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_powerIgnoreTime = 2000;  // time to ignore motor power (ms)
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_zeroSettleTime = 3000;   // how long (ms) to wait for motors to settle at zero speed
  m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_swapDir = 0; // inverse right motor direction?
  m_wheels.m_wheel[Wheel::RIGHT].m_encoder.setup(
      PIN_ODOMETER_RIGHT, ODOMETER_SWAP_DIR_RIGHT);

  // mower motor
  m_cutter.m_motor.config(2000.0,     // Acceleration
                      255,        // Max PWM
                      75,         // Max Power
                      false,      // Modulation
                      0,          // Max RPM
                      3300);      // Set RPM
  m_cutter.m_motor.setScale(3.25839);
  pwmMax = m_cutter.m_motor.m_pwmMax;
  m_cutter.m_motor.m_pid.setup(
      0.005, 0.01, 0.01, 0.0, pwmMax, pwmMax, 0.01, 1.0);  // Kp, Ki, Kd
  m_cutter.m_motor.m_pid.setSetPoint(m_cutter.m_motor.m_rpmSet);

  // lawn sensor
  m_lawnSensorArray[0].setup(PIN_LAWN_FRONT_SEND, PIN_LAWN_FRONT_RECV);
  m_lawnSensorArray[1].setup(PIN_LAWN_BACK_SEND, PIN_LAWN_BACK_RECV);

  // sonar
  m_sonarArray[static_cast<uint8_t>(SonarE::LEFT)].setup(
      PIN_SONAR_LEFT_TRIGGER,
      PIN_SONAR_LEFT_ECHO);
  m_sonarArray[static_cast<uint8_t>(SonarE::CENTER)].setup(
      PIN_SONAR_CENTER_TRIGGER,
      PIN_SONAR_CENTER_ECHO);
  m_sonarArray[static_cast<uint8_t>(SonarE::RIGHT)].setup(
      PIN_SONAR_RIGHT_TRIGGER,
      PIN_SONAR_RIGHT_ECHO);

  // rain
  m_rainSensor.setup(PIN_RAIN);

  // R/C
  pinMode(PIN_REMOTE_MOW, INPUT);
  pinMode(PIN_REMOTE_STEER, INPUT);
  pinMode(PIN_REMOTE_SPEED, INPUT);
  pinMode(PIN_REMOTE_SWITCH, INPUT);

  // user switches
  pinMode(PIN_USER_SWITCH_1, OUTPUT);
  pinMode(PIN_USER_SWITCH_2, OUTPUT);
  pinMode(PIN_USER_SWITCH_3, OUTPUT);


  // PWM frequency setup
  // For obstacle detection, motor torque should be detectable - torque can be computed by motor current.
  // To get consistent current values, PWM frequency should be 3.9 Khz
  // http://wiki.ardumower.de/index.php?title=Motor_driver
  // http://sobisource.com/arduino-mega-pwm-pin-and-frequency-timer-control/
  // http://www.atmel.com/images/doc2549.pdf
  TCCR3B = (TCCR3B & 0xF8) | 0x02;  // set PWM frequency 3.9 Khz (pin2,3,5)
  TCCR1B = (TCCR1B & 0xF8) | 0x02;  // set PWM frequency 3.9 Khz (pin11,12)

  // enable interrupts
  // R/C
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT4);
  PCMSK0 |= (1 << PCINT5);
  PCMSK0 |= (1 << PCINT6);
  PCMSK0 |= (1 << PCINT1);

  // odometer
  PCICR  |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT20);
  PCMSK2 |= (1 << PCINT21);
  PCMSK2 |= (1 << PCINT22);
  PCMSK2 |= (1 << PCINT23);

  // mower motor speed sensor interrupt
  //attachInterrupt(5, rpm_interrupt, CHANGE);
  PCMSK2 |= (1 << PCINT19);

  // ADC
  ADCMan.init();

  m_imu.init(&m_buzzer);
  m_gps.init();
}

void Robot::configureBluetooth(bool quick)
{
  BluetoothConfig bt;
  bt.setParams(m_name, PFOD_BLUETOOTH_PIN_CODE, PFOD_BAUDRATE, quick);
}

void Robot::printOdometer()
{
  Console.print(F("ODO,"));
  Console.print(m_odometer.getX());
  Console.print(", ");
  Console.println(m_odometer.getY());
}

void Robot::printInfo_perimeter(Stream &s)
{
  m_perimeters.printInfo(s);
  Streamprint(s, "  in %-2d  cnt %-4d  on %-1d\r\n", m_perimeterInside,
              m_perimeterCounter,
              !m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)].signalTimedOut());
}

void Robot::printInfo_odometer(Stream &s)
{
  Streamprint(s, "odo %4d %4d ",
      m_odometer.m_encoder.left.getCounter(),
      m_odometer.m_encoder.right.getCounter());
}

void Robot::printInfo_sensorValues(Stream &s)
{
  Streamprint(s, "sen %4d %4d %4d ",
              m_wheels.m_wheel[Wheel::LEFT].m_motor.getPowerMeas(),
              m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPowerMeas(),
              m_cutter.m_motor.getPowerMeas());

  Streamprint(s, "bum %4d %4d ",
              m_bumperArray[LEFT].isHit(),
              m_bumperArray[RIGHT].isHit());

  Streamprint(s, "dro %4d %4d ",
      m_dropSensorArray[LEFT].isDetected(),
      m_dropSensorArray[RIGHT].isDetected());

  Streamprint(s, "son %4u %4u %4u ",
      m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::LEFT)].getDistance_us(),
      m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::CENTER)].getDistance_us(),
      m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::RIGHT)].getDistance_us());

  Streamprint(s, "yaw %3d ", m_imu.getYawDegInt());
  Streamprint(s, "pit %3d ", m_imu.getPitchDegInt());
  Streamprint(s, "rol %3d ", m_imu.getRollDeg());

  if (m_perimeters.isUsed())
  {
    Streamprint(s, "per %3d ", m_perimeterInside);
  }

  if (m_lawnSensors.isUsed())
  {
    Streamprint(s, "lawn %3d %3d ",
                (int)m_lawnSensorArray[FRONT].getValue(),
                (int)m_lawnSensorArray[BACK].getValue());
  }
}

void Robot::printInfo_sensorCounters(Stream &s)
{
  Streamprint(s, "sen %4d %4d %4d ",
              m_wheels.m_wheel[Wheel::LEFT].m_motor.getOverloadCounter(),
              m_wheels.m_wheel[Wheel::RIGHT].m_motor.getOverloadCounter(),
              m_cutter.m_motor.getOverloadCounter());

  Streamprint(s, "bum %4d %4d ",
              m_bumperArray[LEFT].getCounter(),
              m_bumperArray[RIGHT].getCounter());

  Streamprint(s, "dro %4d %4d ",
      m_dropSensorArray[LEFT].getCounter(),
      m_dropSensorArray[RIGHT].getCounter());

  Streamprint(s, "son %3d ", m_sonars.getDistanceCounter());

  Streamprint(s, "yaw %3d ", m_imu.getYawDegInt());
  Streamprint(s, "pit %3d ", m_imu.getPitchDegInt());
  Streamprint(s, "rol %3d ", m_imu.getRollDegInt());

  //Streamprint(s, "per %3d ", perimeterLeft);

  if (m_perimeters.isUsed())
  {
    Streamprint(s, "per %3d ", m_perimeterCounter);
  }

  if (m_lawnSensors.isUsed())
  {
    Streamprint(s, "lawn %3d ", m_lawnSensors.getCounter());
  }

  if (m_gpsUse)
  {
    Streamprint(s, "gps %2d ", m_gps.satellites());
  }
}

void Robot::printInfo(Stream &s)
{
  if (m_consoleMode == CONSOLE_OFF)
  {
    return;
  }

  Streamprint(s, "t%4u ", (millis() - m_stateMachine.getStateStartTime()) / 1000);

  Streamprint(s, "l%5u ", m_loopsPerSec);

  //Streamprint(s, "r%4u ", freeRam());

  Streamprint(s, "v%1u ", m_consoleMode);

  Streamprint(s, "%8s ", m_stateMachine.getCurrentStateName());

  if (m_consoleMode == CONSOLE_PERIMETER)
  {
    printInfo_perimeter(s);
  }
  else if (m_consoleMode == CONSOLE_IMU)
  {
    m_imu.printInfo(s);
  }
  else
  {
    if (m_odometer.isUsed())
    {
      printInfo_odometer(s);
    }
    Streamprint(s, "spd %4d %4d %4d ",
                m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet,
                m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet,
                m_cutter.m_motor.getRpmMeas());

    if (m_consoleMode == CONSOLE_SENSOR_VALUES)
    {
      printInfo_sensorValues(s);
    }
    else
    {
      printInfo_sensorCounters(s);
    }

    int16_t batVolt_mV = m_battery.getBatVoltage_mV() + 50;
    Streamprint(s, "bat %2d.%01d V",
        batVolt_mV / 1000, (batVolt_mV % 1000) / 100);

    int16_t chgVolt_mV = m_battery.getChargeVoltage_mV() + 50;
    int16_t chgCurr_mA = m_battery.getChargeCurrent_mA();

    Streamprint(s, "chg %2d.%01d V @ %4d mA",
        chgVolt_mV / 1000, (chgVolt_mV % 1000) / 100, chgCurr_mA);

    Streamprint(s, "imu %3u ", m_imu.getAndClearCallCounter());

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
    printInfo(Console);
    delay(1000);
  }
}

void Robot::testOdometer()
{
  auto motorLeft_p = &m_wheels.m_wheel[Wheel::LEFT].m_motor;
  auto motorRight_p = &m_wheels.m_wheel[Wheel::RIGHT].m_motor;

  int16_t leftPwm = motorLeft_p->m_pwmMax / 2;
  int16_t rightPwm = motorRight_p->m_pwmMax / 2;

  motorLeft_p->setPwmCur(leftPwm);
  motorRight_p->setPwmCur(rightPwm);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  int lastLeft = 0;
  int lastRight = 0;

  for (;;)
  {
    resetIdleTime();

    int odoCountLeft = m_odometer.m_encoder.left.getCounter();
    int odoCountRight = m_odometer.m_encoder.right.getCounter();

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
        motorLeft_p->setPwmCur(leftPwm);
        motorRight_p->setPwmCur(rightPwm);
        setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);
      }

      if (ch == 'r')
      {
        motorLeft_p->setPwmCur(-leftPwm);
        motorRight_p->setPwmCur(-rightPwm);
        setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);
      }

      if (ch == 'z')
      {
        m_odometer.m_encoder.left.clearCounter();
        m_odometer.m_encoder.right.clearCounter();
      }
    }
  }

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);
}

void Robot::testMotors()
{
  auto motorLeft_p = &m_wheels.m_wheel[Wheel::LEFT].m_motor;
  auto motorRight_p = &m_wheels.m_wheel[Wheel::RIGHT].m_motor;

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(0, 0);

  Console.println(F("Testing left motor (forward) full speed..."));

  delay(1000);

  motorLeft_p->setPwmCur(motorLeft_p->m_pwmMax);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  delayInfo(5000);

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  Console.println(F("Testing left motor (reverse) full speed..."));

  delay(1000);

  motorLeft_p->setPwmCur(-motorLeft_p->m_pwmMax);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  delayInfo(5000);

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  Console.println(F("Testing right motor (forward) full speed..."));

  delay(1000);

  motorLeft_p->setPwmCur(0);
  motorRight_p->m_pwmCur = motorLeft_p->m_pwmMax;
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  delayInfo(5000);

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  Console.println(F("Testing right motor (reverse) full speed..."));

  delay(1000);

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(-motorLeft_p->m_pwmMax);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);

  delayInfo(5000);

  motorLeft_p->setPwmCur(0);
  motorRight_p->setPwmCur(0);
  setMotorPWMs(motorLeft_p->m_pwmCur, motorRight_p->m_pwmCur);
}

void Robot::menu()
{
  printMenu();

  for (;;)
  {
    resetIdleTime();
    m_imu.update();

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
          m_imu.calibrateAccelerometerNextAxis();
          break;

        case '6':
          m_imu.calibrateMagnetometerStartStop();
          break;

        case '7':
          m_imu.deleteCalibrationData();
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
          setNextState(StateMachine::STATE_OFF);
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
        m_consoleMode = (m_consoleMode + 1) % CONSOLE_END;
        Console.println(consoleModeName());
        break;

      case 'h': // drive home
        setNextState(StateMachine::STATE_PERI_FIND);
        break;

      case 't': // track perimeter
        setNextState(StateMachine::STATE_PERI_TRACK);
        break;

      case 'l': // simulate left bumper
        m_bumperArray[LEFT].simHit();
        break;

      case 'r': // simulate right bumper
        m_bumperArray[RIGHT].simHit();
        break;

      case 'j': // simulate left drop
        m_dropSensorArray[LEFT].simDetected();
        break;

      case 'k': // simulate right drop
        m_dropSensorArray[RIGHT].simDetected();
        break;

      case 's': // simulate lawn sensor
        m_lawnSensors.simDetected();
        break;

      case 'm': // toggle mower motor
        if (m_stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
            m_stateMachine.isCurrentState(StateMachine::STATE_MANUAL))
        {
          m_cutter.setEnableOverriden(false);
        }
        else
        {
          m_cutter.toggleEnableOverriden();
        }
        m_cutter.toggleEnabled();
        break;

      case 'c': // simulate in station
        setNextState(StateMachine::STATE_STATION);
        break;

      case 'a': // simulate in station charging
        setNextState(StateMachine::STATE_STATION_CHARGING);
        break;

      case '+': // rotate 90 degrees clockwise (IMU)
        setNextState(StateMachine::STATE_ROLL_WAIT);
        m_imuRollHeading = scalePI(m_imuRollHeading + PI / 2);
        break;

      case '-': // rotate 90 degrees anti-clockwise (IMU)
        setNextState(StateMachine::STATE_ROLL_WAIT);
        m_imuRollHeading = scalePI(m_imuRollHeading - PI / 2);
        break;

      case 'i': // toggle imu.use
        {
          auto settings_p = m_imu.getSettings();
          settings_p->use.value = !settings_p->use.value;
          break;
        }

      case '0': // turn OFF
        setNextState(StateMachine::STATE_OFF);
        break;

      case '1': // Automode
        m_cutter.enable();
        //motorMowModulate = false;
        setNextState(StateMachine::STATE_FORWARD);
        break;
    }
  }
}

void Robot::checkButton()
{
  bool buttonPressed = m_button.isPressed();

  if ((!buttonPressed && m_button.getCounter() > 0) ||
      (buttonPressed && m_button.isTimeToRun()))
  {
    if (buttonPressed)
    {
      Console.println(F("Button is pressed"));
      m_buzzer.beepLong();
      m_button.incCounter();
      resetIdleTime();
    }
    else
    {
      if ((!m_stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
           m_stateMachine.isCurrentState(StateMachine::STATE_ERROR)) &&
          !m_stateMachine.isCurrentState(StateMachine::STATE_STATION))
      {
        setNextState(StateMachine::STATE_OFF);
      }
      else
      {
        switch (m_button.getCounter())
        {
          case 1:
            // start normal with random mowing
            m_cutter.enable();
            m_mowPattern = MOW_RANDOM;
            setNextState(StateMachine::STATE_FORWARD);
            break;

          case 2:
            // start normal with bidir mowing
            m_cutter.enable();
            m_mowPattern = MOW_BIDIR;
            setNextState(StateMachine::STATE_FORWARD);
            break;

          case 4:
            // start normal without perimeter
            m_perimeters.getSettings()->use.value = false;
            setNextState(StateMachine::STATE_FORWARD);
            break;

          case 5:
            // drive home
            setNextState(StateMachine::STATE_PERI_FIND);
            break;

          case 6:
            // track perimeter
            setNextState(StateMachine::STATE_PERI_TRACK);
            break;

          case 7:
            // start normal with lanes mowing
            m_cutter.enable();
            m_mowPattern = MOW_LANES;
            setNextState(StateMachine::STATE_FORWARD);
            break;

          default:
            Console.print("Unknown number of button presses, ");
            Console.println(m_button.getCounter());
            break;
        }
      }

      m_button.clearCounter();
    }
  }
}

void Robot::readMotorCurrents()
{
  auto motorLeft_p = &m_wheels.m_wheel[Wheel::LEFT].m_motor;
  auto motorRight_p = &m_wheels.m_wheel[Wheel::RIGHT].m_motor;
  auto motorCutter_p = &m_cutter.m_motor;

  motorLeft_p->readCurrent();
  motorRight_p->readCurrent();
  motorCutter_p->readCurrent();

  // Conversion to power in Watts
  int16_t bat_mV = m_battery.getBatVoltage_mV();
  motorLeft_p->calcPower(bat_mV);
  motorRight_p->calcPower(bat_mV);
  motorCutter_p->calcPower(bat_mV);
}

void Robot::measureCutterMotorRpm()
{
  static uint32_t timeLastMeasure = 0;

  uint32_t curMillis = millis();
  uint32_t timeSinceLast = curMillis - timeLastMeasure;
  timeLastMeasure = curMillis;;

  if ((m_cutter.m_motor.getRpmMeas() == 0) &&
      (m_cutter.m_motor.getRpmCounter() != 0))
  {
    // rpm may be updated via interrupt
    m_cutter.m_motor.setRpmMeas(
        (int)(((float)m_cutter.m_motor.getRpmCounter() / (float)timeSinceLast)
            * 60000.0));
    m_cutter.m_motor.clearRpmCounter();
  }

  if (!ADCMan.calibrationDataAvail())
  {
    Console.println(F("Error: Missing ADC calibration data"));
    incErrorCounter(ERR_ADC_CALIB);
    setNextState(StateMachine::STATE_ERROR);
  }
}

void Robot::readPerimeters()
{
  unsigned long curMillis = millis();

  auto perimeterLeft_p = &m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)];
  //auto perimeterRight_p = &m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::RIGHT)];

  m_perimeterMag = perimeterLeft_p->calcMagnitude();
  bool inside = perimeterLeft_p->isInside();

  if (inside != m_perimeterInside)
  {
    m_perimeterCounter++;
    m_perimeterLastTransitionTime = curMillis;
    m_perimeterInside = inside;
  }

  m_ledPerimeter.set(m_perimeterInside);

  if (!m_perimeterInside && m_perimeterTriggerTime == 0)
  {
    // set perimeter trigger time
    // far away from perimeter?
    if (curMillis > m_stateMachine.getStateStartTime() + 2000)
    {
      m_perimeterTriggerTime = curMillis + m_perimeterTriggerTimeout;
    }
    else
    {
      m_perimeterTriggerTime = curMillis;
    }
  }

  if (perimeterLeft_p->signalTimedOut())
  {
    if (!m_stateMachine.isCurrentState(StateMachine::STATE_OFF) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_MANUAL) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHARGING) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHECK) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_REV) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_ROLL) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_FORW) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_PERI_OUT_FORW) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_PERI_OUT_REV) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_PERI_OUT_ROLL))
    {
      Console.println("Error: Perimeter too far away");
      incErrorCounter(ERR_PERIMETER_TIMEOUT);
      setNextState(StateMachine::STATE_ERROR);
    }
  }
}

void Robot::readImu()
{
  if (m_imu.getAndClearErrorCounter() > 0)
  {
    incErrorCounter(ERR_IMU_COMM);
    Console.println(F("IMU comm error"));
  }

  if (!m_imu.isCalibrationAvailable())
  {
    Console.println(F("Error: Missing IMU calibration data"));
    incErrorCounter(ERR_IMU_CALIB);
    setNextState(StateMachine::STATE_ERROR);
  }
}

void Robot::setDefaults()
{
  m_speed = 0;
  m_steer = 0;
  m_cutter.disable();
}

// set state machine new state
// http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
// called *ONCE* to set to a *NEW* state
void Robot::setNextState(StateMachine::StateE stateNew, uint8_t rollDir)
{
  if (m_stateMachine.isCurrentState(stateNew))
  {
    return;
  }

  unsigned long curMillis = millis();

  // state correction
  if (m_stateMachine.isCurrentState(StateMachine::STATE_PERI_FIND) ||
      m_stateMachine.isCurrentState(StateMachine::STATE_PERI_TRACK))
  {
    if (stateNew == StateMachine::STATE_ROLL)
    {
      stateNew = StateMachine::STATE_PERI_ROLL;
    }

    if (stateNew == StateMachine::STATE_REVERSE)
    {
      stateNew = StateMachine::STATE_PERI_REV;
    }
  }

  if (stateNew == StateMachine::STATE_FORWARD)
  {
    if (m_stateMachine.isCurrentState(StateMachine::STATE_STATION_REV) ||
        m_stateMachine.isCurrentState(StateMachine::STATE_STATION_ROLL) ||
        m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHECK))
    {
      return;
    }

    if (m_stateMachine.isCurrentState(StateMachine::STATE_STATION) ||
        m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHARGING))
    {
      stateNew = StateMachine::STATE_STATION_CHECK;
      m_battery.setChargeRelay(OFF);
      m_cutter.disable();
    }
  }

  // evaluate new state
  m_stateMachine.setNextState(stateNew);
  m_rollDir = rollDir;
  int zeroSettleTime = m_wheels.m_wheel[Wheel::LEFT].m_motor.m_zeroSettleTime;

  if (stateNew == StateMachine::STATE_STATION_REV)
  {
    m_speed = -100;
    m_steer = 0;
    m_stateMachine.setEndTime(curMillis + m_stationRevTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_STATION_ROLL)
  {
    m_speed = 0;
    m_steer = +100;
    m_stateMachine.setEndTime(curMillis + m_stationRollTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_STATION_FORW)
  {
    m_speed = +100;
    m_steer = 0;
    m_cutter.enable();
    m_stateMachine.setEndTime(curMillis + m_stationForwTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_STATION_CHECK)
  {
    m_speed = -25;
    m_steer = 0;
    m_stateMachine.setEndTime(curMillis + m_stationCheckTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_PERI_ROLL)
  {
    m_speed = 0;
    m_steer = (rollDir == LEFT) ? -25: +25;
    m_stateMachine.setEndTime(curMillis + m_perimeterTrackRollTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_PERI_REV)
  {
    m_speed = -25;
    m_steer = 0;
    m_stateMachine.setEndTime(curMillis + m_perimeterTrackRevTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_PERI_OUT_FORW)
  {
    m_speed = +100;
    m_steer = 0;
    m_stateMachine.setEndTime(curMillis + m_perimeterOutRevTime + zeroSettleTime + 1000);
  }
  else if (stateNew == StateMachine::STATE_PERI_OUT_REV)
  {
    m_speed = -75;
    m_steer = 0;
    m_stateMachine.setEndTime(curMillis + m_perimeterOutRevTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_PERI_OUT_ROLL)
  {
    m_speed = 0;
    m_steer = (rollDir == LEFT) ? -75: +75;
    m_stateMachine.setEndTime(
        curMillis + random(m_perimeterOutRollTimeMin, m_perimeterOutRollTimeMax)
            + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_FORWARD)
  {
    m_speed = 100;
    m_steer = 0;
    m_statsMowTimeTotalStart = true;
  }
  else if (stateNew == StateMachine::STATE_REVERSE)
  {
    m_speed = -75;
    m_steer = 0;
    m_stateMachine.setEndTime(
        curMillis + m_wheels.m_reverseTime + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_ROLL)
  {
    m_imuDriveHeading = scalePI(m_imuDriveHeading + PI); // Toggle heading 180 degree (IMU)
    if (m_imuRollDir == LEFT)
    {
      m_imuRollHeading = scalePI(m_imuDriveHeading - PI / 20);
      m_imuRollDir = RIGHT;
    }
    else
    {
      m_imuRollHeading = scalePI(m_imuDriveHeading + PI / 20);
      m_imuRollDir = LEFT;
    }
    m_speed = 0;
    m_steer = (rollDir == LEFT) ? -75: +75;
    m_stateMachine.setEndTime(
        curMillis + random(m_wheels.m_rollTimeMin, m_wheels.m_rollTimeMax)
            + zeroSettleTime);
  }
  else if (stateNew == StateMachine::STATE_STATION)
  {
    m_battery.setChargeRelay(OFF);
    setDefaults();
    m_statsMowTimeTotalStart = false;  // stop stats mowTime counter
    saveRobotStats();
  }
  else if (stateNew == StateMachine::STATE_STATION_CHARGING)
  {
    m_battery.setChargeRelay(ON);
    setDefaults();
  }
  else if (stateNew == StateMachine::STATE_OFF)
  {
    m_battery.setChargeRelay(OFF);
    setDefaults();
    m_statsMowTimeTotalStart = false; // stop stats mowTime counter
    saveRobotStats();
  }
  else if (stateNew == StateMachine::STATE_ERROR)
  {
    setDefaults();
    m_battery.setChargeRelay(OFF);
    m_statsMowTimeTotalStart = false;
    //saveRobotStats();
  }
  else if (stateNew == StateMachine::STATE_PERI_FIND)
  {
    // find perimeter  => drive half speed
    m_speed = 50;
    m_steer = 0;
    //motorMowEnable = false;     // FIXME: should be an option?
  }
  else if (stateNew == StateMachine::STATE_PERI_TRACK)
  {
    //motorMowEnable = false;     // FIXME: should be an option?
    m_speed = 50;
    m_steer = 0;
    m_battery.setChargeRelay(OFF);
    //m_buzzer.beepLong(6);
  }

  m_stateMachine.changeState();

  m_sonars.setObstacleTimeout(0);
  m_perimeterTriggerTime = 0;

  printInfo(Console);
}

void Robot::checkBattery()
{
  if (m_battery.isMonitored())
  {
    if (m_battery.isVoltageBelowSwitchOffLimit() &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_ERROR) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_OFF) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION) &&
        !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHARGING))
    {
      Console.println(F("Triggered batSwitchOffIfBelow"));
      incErrorCounter(ERR_BATTERY);
      m_buzzer.beepShort(2);
      setNextState(StateMachine::STATE_OFF);
    }
    else if (m_battery.isVoltageBelowGoHomeLimit() &&
             m_perimeters.isUsed() &&
             !m_stateMachine.isCurrentState(StateMachine::STATE_OFF) &&
             !m_stateMachine.isCurrentState(StateMachine::STATE_MANUAL) &&
             !m_stateMachine.isCurrentState(StateMachine::STATE_STATION) &&
             !m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHARGING) &&
             !m_stateMachine.isCurrentState(StateMachine::STATE_ERROR) &&
             !m_stateMachine.isCurrentState(StateMachine::STATE_PERI_TRACK))
    {
      Console.println(F("Triggered batGoHomeIfBelow"));
      m_buzzer.beepShort(2);
      setNextState(StateMachine::STATE_PERI_FIND);
    }
  }

  // Check if mower is idle and battery can be switched off
  if (m_stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
      m_stateMachine.isCurrentState(StateMachine::STATE_ERROR))
  {
    if (m_idleTimeSec != BATTERY_SW_OFF)
    {
      // battery already switched off?
      if (m_battery.hasIdleTimePassedSwitchOffLimit(++m_idleTimeSec))
      {
        Console.println(F("Triggered batSwitchOffIfIdle"));
        m_buzzer.beepShort();
        saveErrorCounters();
        saveRobotStats();
        m_idleTimeSec = BATTERY_SW_OFF; // flag to remember that battery is switched off
        Console.println(F("BATTERY switching OFF"));
        m_battery.setBatterySwitch(OFF);  // switch off battery
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
  if (!m_gpsUse)
  {
    return;
  }

  unsigned long chars = 0;
  unsigned short good_sentences = 0;
  unsigned short failed_cs = 0;
  m_gps.stats(&chars, &good_sentences, &failed_cs);

  if (good_sentences == 0)
  {
    // no GPS sentences received so far
    Console.println(F("GPS communication error!"));
    incErrorCounter(ERR_GPS_COMM);
  }
  Console.print(F("GPS sentences: "));
  Console.println(good_sentences);
  Console.print(F("GPS satellites in view: "));
  Console.println(m_gps.satellites());

  if (m_gps.satellites() == Gps::GPS_INVALID_SATELLITES)
  {
    // no GPS satellites received so far
    incErrorCounter(ERR_GPS_DATA);
  }

  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  m_gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths,
                     &age);

  if (age != Gps::GPS_INVALID_AGE)
  {
    Console.print(F("GPS date received: "));
    Console.println(date2str(m_datetime.date));
    m_datetime.date.dayOfWeek =
        getDayOfWeek(month, day, year, CalendarSystem::Gregorian);
    m_datetime.date.day = day;
    m_datetime.date.month = month;
    m_datetime.date.year = year;
    m_datetime.time.hour = hour;
    m_datetime.time.minute = minute;

    if (m_timerUse)
    {
      // set RTC using GPS data
      setRtc();
    }
  }
}

void Robot::readRtc()
{
  if (!readDS1307(m_datetime))
  {
    Console.println("RTC data error!");
    //addErrorCounter(ERR_RTC_DATA);
    //setNextState(STATE_ERROR, 0);
  }
  else
  {
    Console.print(F("RTC datetime read: "));
    Console.println(date2str(m_datetime.date));
  }
}

void Robot::setRtc()
{
  if (!setDS1307(m_datetime))
  {
    Console.println("RTC comm error!");
    incErrorCounter(ERR_RTC_COMM);
    setNextState(StateMachine::STATE_ERROR);
  }
  else
  {
    Console.print(F("RTC datetime set: "));
    Console.println(date2str(m_datetime.date));
  }
}

void Robot::checkRobotStats_mowTime()
{
  m_statsMowTimeHoursTotal = float(m_stats.mowTimeTotal_min) / 60;
  if (m_statsMowTimeTotalStart)
  {
    m_statsMowTimeMinutesTripCounter++;
    m_stats.mowTimeTrip_min = m_statsMowTimeMinutesTripCounter;
    m_stats.mowTimeTotal_min++;
  }
  else
  {
    m_statsMowTimeMinutesTripCounter = 0;
  }
}

void Robot::checkRobotStats_battery()
{
  // Count only if mower has charged more than 60 seconds.
  if (m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHARGING) &&
      m_stateMachine.getStateTime() >= 60000)
  {
    m_statsBatteryChargingCounter++; // temporary counter
    if (m_statsBatteryChargingCounter == 1)
    {
      m_stats.batteryChargingCounterTotal++;
    }
    m_stats.batteryChargingCapacityTrip_mAh = m_battery.getCapacity_mAh();
    // Sum up only the difference between actual batCapacity and last batCapacity
    m_stats.batteryChargingCapacityTotal_mAh +=
        (m_battery.getCapacity_mAh() - m_battery.getLastTimeCapacity_mAh());

    m_battery.updateLastTimeCapacity();
  }
  else
  {
    // Reset values to 0 when mower is not charging
    m_statsBatteryChargingCounter = 0;
    m_battery.clearCapacity();
  }

  if (isnan(m_stats.batteryChargingCapacityTrip_mAh))
  {
    m_stats.batteryChargingCapacityTrip_mAh = 0;
  }

  if (isnan(m_stats.batteryChargingCapacityTotal_mAh))
  {
    m_stats.batteryChargingCapacityTotal_mAh = 0;
  }

  if (m_stats.batteryChargingCapacityTotal_mAh <= 0 ||
      m_stats.batteryChargingCounterTotal == 0)
  {
    m_stats.batteryChargingCapacityAverage_mAh = 0; // Avoid divide by zero
  }
  else
  {
    m_stats.batteryChargingCapacityAverage_mAh =
        m_stats.batteryChargingCapacityTotal_mAh / m_stats.batteryChargingCounterTotal;
  }
}

void Robot::checkTimer()
{
  // TODO: Should these inits only be done once?
  // Initialize the pseudo-random number generator for c++ rand()
  srand(time2minutes(m_datetime.time));

  // Initialize the pseudo-random number generator for arduino random()
  randomSeed(time2minutes(m_datetime.time));

  receiveGPSTime();

  if (!m_timerUse)
  {
    return;
  }

  bool stopTimerTriggered = true;
  for (uint8_t i = 0; i < MAX_TIMERS; i++)
  {
    auto timerSettings_p = &m_settings.timer[i];

    if (!timerSettings_p->active.value)
    {
      continue;
    }

    if (timerSettings_p->daysOfWeek.value & (1 << m_datetime.date.dayOfWeek))
    {
      // Matched dayOfWeek
      // TODO: Improve this
      timehm_t time;
      time.hour = timerSettings_p->startTime.hour.value;
      time.minute = timerSettings_p->startTime.minute.value;
      int startMinute = time2minutes(time);

      time.hour = timerSettings_p->stopTime.hour.value;
      time.minute = timerSettings_p->stopTime.minute.value;
      int stopMinute = time2minutes(time);

      int currMinute = time2minutes(m_datetime.time);

      if (currMinute >= startMinute && currMinute < stopMinute)
      {
        // Start timer triggered
        stopTimerTriggered = false;
        if (m_stateMachine.isCurrentState(StateMachine::STATE_STATION) ||
            m_stateMachine.isCurrentState(StateMachine::STATE_OFF))
        {
          Console.println(F("Timer start triggered"));
          m_cutter.enable();
          setNextState(StateMachine::STATE_FORWARD);
        }
      }
    }

    if (stopTimerTriggered && m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD))
    {
      Console.println(F("Timer stop triggered"));
      if (m_perimeters.isUsed())
      {
        setNextState(StateMachine::STATE_PERI_FIND);
      }
      else
      {
        setNextState(StateMachine::STATE_OFF);
      }
    }
  }
}

void Robot::reverseOrChangeDirection(const uint8_t rollDir)
{
  if (m_mowPattern == MOW_BIDIR)
  {
    if (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD))
    {
      setNextState(StateMachine::STATE_REVERSE, RIGHT);
    }
    else if (m_stateMachine.isCurrentState(StateMachine::STATE_REVERSE))
    {
      setNextState(StateMachine::STATE_FORWARD, LEFT);
    }
  }
  else
  {
    setNextState(StateMachine::STATE_REVERSE, rollDir);
  }
}

void Robot::checkCutterMotorPower()
{
  if (m_cutter.m_motor.isOverpowered())
  {
    m_cutter.m_motor.incOverloadCounter();
  }
  else
  {
    m_errorCounterMax[ERR_CUTTER_SENSE] = 0;
    m_cutter.m_motor.clearOverloadCounter();

    // Wait some time before switching on again
    if (m_cutter.m_motor.isWaitAfterStuckEnd())
    {
      m_errorCounter[ERR_CUTTER_SENSE] = 0;
      m_cutter.enable();
    }
  }

  // Ignore motor cutter power overload for 3 seconds
  if (m_cutter.m_motor.getOverloadCounter() >= 30)
  {
    m_cutter.disable();
    Console.println("Error: Motor cutter current");
    incErrorCounter(ERR_CUTTER_SENSE);
    m_cutter.m_motor.gotStuck();
  }
}

void Robot::checkWheelMotorPower(Wheel::WheelE side)
{
  auto motor_p = &m_wheels.m_wheel[side].m_motor;

  bool hasPassedPowerIgnoreTime =
      millis() > (m_stateMachine.getStateStartTime() + motor_p->m_powerIgnoreTime);

  if (motor_p->isOverpowered() && hasPassedPowerIgnoreTime)
  {
    //m_buzzer.beepLong(1);
    motor_p->incOverloadCounter();
    setMotorPWMs(0, 0);
    // TODO: wheels.stop();

    if (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD) ||
        m_stateMachine.isCurrentState(StateMachine::STATE_PERI_FIND) ||
        m_stateMachine.isCurrentState(StateMachine::STATE_PERI_TRACK))
    {
      reverseOrChangeDirection(RIGHT);
    }
    else if (m_stateMachine.isCurrentState(StateMachine::STATE_REVERSE))
    {
      setNextState(StateMachine::STATE_ROLL, !side);
    }
    else if (m_stateMachine.isCurrentState(StateMachine::STATE_ROLL))
    {
      setNextState(StateMachine::STATE_FORWARD);
    }
  }
}

void Robot::checkMotorPower()
{
  if (m_cutter.m_motor.isTimeToCheckPower()) // Any motor could be the trigger
  {
    checkCutterMotorPower();
    checkWheelMotorPower(Wheel::LEFT);
    checkWheelMotorPower(Wheel::RIGHT);
  }
}

// check bumpers
void Robot::checkBumpers()
{
  if (m_mowPattern == MOW_BIDIR && millis() < (m_stateMachine.getStateStartTime() + 4000))
  {
    return;
  }

  if (m_bumperArray[LEFT].isHit())
  {
    reverseOrChangeDirection(RIGHT);
  }
  if (m_bumperArray[RIGHT].isHit())
  {
    reverseOrChangeDirection(LEFT);
  }
}

// check drop
void Robot::checkDrop()
{
  unsigned long curMillis = millis();
  if (m_mowPattern == MOW_BIDIR && curMillis < (m_stateMachine.getStateStartTime() + 4000))
  {
    return;
  }

  if (m_dropSensorArray[LEFT].isDetected())
  {
    reverseOrChangeDirection(RIGHT);
  }
  if (m_dropSensorArray[RIGHT].isDetected())
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
  if (m_bumpers.isAnyHit())
  {
    if (m_bumperArray[LEFT].isHit() ||
        m_stateMachine.isCurrentState(StateMachine::STATE_PERI_TRACK))
    {
      setNextState(StateMachine::STATE_PERI_REV, RIGHT);
    }
    else
    {
      setNextState(StateMachine::STATE_PERI_REV, LEFT);
    }
  }
}

// Check perimeter as a boundary
void Robot::checkPerimeterBoundary()
{
  unsigned long curMillis = millis();

  if (m_wheels.isTimeToRotationChange())
  {
    m_wheels.m_rotateDir = (Wheel::WheelE)!m_wheels.m_rotateDir; // Toggle rotation direction
  }

  if (m_mowPattern == MOW_BIDIR)
  {
    if (curMillis < m_stateMachine.getStateStartTime() + 3000)
    {
      return;
    }
    if (!m_perimeterInside)
    {
      reverseOrChangeDirection(rand() % 2); // Random direction
    }
  }
  else
  {
    if (m_perimeterTriggerTime > 0 && curMillis >= m_perimeterTriggerTime)
    {
      m_perimeterTriggerTime = 0;
      if (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD))
      {
        setNextState(StateMachine::STATE_PERI_OUT_REV, m_wheels.m_rotateDir);
      }
      else if (m_stateMachine.isCurrentState(StateMachine::STATE_ROLL))
      {
        m_speed = 0;
        m_steer = 0;
        setNextState(StateMachine::STATE_PERI_OUT_FORW, m_wheels.m_rotateDir);
      }
    }
  }
}

// Check perimeter while finding it
void Robot::checkPerimeterFind()
{
  if (m_stateMachine.isCurrentState(StateMachine::STATE_PERI_FIND))
  {
    if (m_perimeters.m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)].isInside())
    {
      if (m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet !=
          m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet)
      {
        // We just made an 'outside to inside' rotation, now track
        setNextState(StateMachine::STATE_PERI_TRACK);
      }
    }
    else
    {
      // We are outside, now roll to get inside
      m_speed = 0;
      m_steer = +50;
    }
  }
}

// check lawn
void Robot::checkLawn()
{
  if (m_lawnSensors.isUsed())
  {
    if (m_lawnSensors.isDetected() &&
        millis() - m_stateMachine.getStateStartTime() >= 3000)
    {
      reverseOrChangeDirection(!m_rollDir);
    }
    else
    {
      m_lawnSensors.clearDetected();
    }
  }
}

void Robot::checkRain()
{
  if (m_rainSensor.isRaining())
  {
    Console.println(F("RAIN"));

    if (m_perimeters.isUsed())
    {
      setNextState(StateMachine::STATE_PERI_FIND);
    }
    else
    {
      setNextState(StateMachine::STATE_OFF);
    }
  }
}

// check sonar
void Robot::checkSonar()
{
  unsigned long curMillis = millis();
  if (m_mowPattern == MOW_BIDIR &&
      curMillis < (m_stateMachine.getStateStartTime() + 4000))
  {
    return;
  }

  // slow down motor wheel speed near obstacles
  if (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD) ||
      (m_mowPattern == MOW_BIDIR &&
       m_stateMachine.isCurrentState(StateMachine::STATE_REVERSE)))
  {
    if (m_sonars.getObstacleTimeout() == 0)
    {
      if (m_sonars.isClose())
      {
        m_sonars.incTempDistanceCounter();
        if (m_sonars.getTempDistanceCounter() >= 5)
        {
          // Console.println("sonar slow down");

          // Avoid slowing down to 0, stop at -1 or +1, allows speedup again
          int8_t tmpSpeed = m_speed / 2;
          if (tmpSpeed == 0)
          {
            m_speed = sign(m_speed);
          }
          else
          {
            m_speed = tmpSpeed;
          }
          m_sonars.setObstacleTimeout(curMillis + 3000);
        }
      }
      else
      {
        m_sonars.clearTempDistanceCounter();
      }
    }
    else if (m_sonars.getObstacleTimeout() != 0 &&
             curMillis > m_sonars.getObstacleTimeout())
    {
      //Console.println("no sonar");
      m_sonars.setObstacleTimeout(0);
      m_sonars.clearTempDistanceCounter();
      m_speed = constrain(m_speed * 2, -100, 100);
    }
  }

  uint16_t distanceUs;
  const uint16_t triggerBelow = m_sonars.getSettings()->triggerBelow.value;

  distanceUs = m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::CENTER)].
      getDistance_us();
  if (distanceUs < triggerBelow)
  {
    m_sonars.incDistanceCounter();
    reverseOrChangeDirection(!m_rollDir);
  }

  distanceUs = m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::RIGHT)].
      getDistance_us();
  if (distanceUs < triggerBelow)
  {
    m_sonars.incDistanceCounter();
    reverseOrChangeDirection(LEFT);
  }

  distanceUs = m_sonars.m_sonarArray_p[static_cast<uint8_t>(SonarE::LEFT)].
      getDistance_us();
  if (distanceUs < triggerBelow)
  {
    m_sonars.incDistanceCounter();
    reverseOrChangeDirection(RIGHT);
  }
}

// check IMU (tilt)
void Robot::checkTilt()
{
  if (m_stateMachine.isCurrentState(StateMachine::STATE_OFF) &&
      m_stateMachine.isCurrentState(StateMachine::STATE_ERROR) &&
      m_stateMachine.isCurrentState(StateMachine::STATE_STATION))
  {
    int pitchAngle = (int)m_imu.getPitchDeg();
    int rollAngle = (int)m_imu.getRollDeg();
    if (abs(pitchAngle) > 40 || abs(rollAngle) > 40)
    {
      Console.println(F("Error: IMU tilt"));
      incErrorCounter(ERR_IMU_TILT);
      setNextState(StateMachine::STATE_ERROR);
    }
  }
}

// Check if mower is stuck
// TODO: Take HDOP into consideration if gpsSpeed is reliable
void Robot::checkIfStuck()
{
  if (m_gpsUse && m_gps.hdop() < 500)
  {
    float gpsSpeed = m_gps.f_speed_kmph();
    if (m_gpsSpeedIgnoreTime >= m_wheels.m_reverseTime)
    {
      m_gpsSpeedIgnoreTime = m_wheels.m_reverseTime - 500;
    }
    // low-pass filter
    // float accel = 0.1;
    // float gpsSpeed = (1.0-accel) * gpsSpeed + accel * gpsSpeedRead;
    // Console.println(gpsSpeed);
    // Console.println(robotIsStuckedCounter);
    // Console.println(errorCounter[ERR_STUCK]);
    if (m_stateMachine.isCurrentState(StateMachine::STATE_MANUAL) &&
        gpsSpeed < m_stuckIfGpsSpeedBelow &&
        m_odometer.m_encoder.left.getWheelRpmCurr() != 0 &&
        m_odometer.m_encoder.right.getWheelRpmCurr() != 0 &&
        millis() > (m_stateMachine.getStateStartTime() + m_gpsSpeedIgnoreTime))
    {
      m_robotIsStuckCounter++;
    }
    else
    {
      // If mower gets unstuck, errorCounterMax is reset to zero and
      // cutter motor is re-enabled.
      m_robotIsStuckCounter = 0;    // resets temporary counter to zero
      if (m_errorCounter[ERR_STUCK] == 0 &&
          m_stateMachine.isCurrentState(StateMachine::STATE_OFF) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_MANUAL) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_STATION) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHARGING) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_STATION_CHECK) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_STATION_REV) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_STATION_ROLL) &&
          m_stateMachine.isCurrentState(StateMachine::STATE_ERROR))
      {
        m_cutter.enable();
        m_errorCounterMax[ERR_STUCK] = 0;
      }
    }

    if (m_robotIsStuckCounter >= 5)
    {
      m_cutter.disable();
      if (m_errorCounterMax[ERR_STUCK] >= 3)
      {
        // robot is definitely stuck and unable to move
        Console.println(F("Error: Mower is stuck"));
        incErrorCounter(ERR_STUCK);
        setNextState(StateMachine::STATE_ERROR);
        //robotIsStuckedCounter = 0;
      }
      else if (m_errorCounter[ERR_STUCK] < 3)
      {
        // mower tries 3 times to get unstuck
        if (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD))
        {
          m_cutter.disable();
          incErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0);
          reverseOrChangeDirection(RIGHT);
        }
        else if (m_stateMachine.isCurrentState(StateMachine::STATE_ROLL))
        {
          m_cutter.disable();
          incErrorCounter(ERR_STUCK);
          setMotorPWMs(0, 0);
          setNextState(StateMachine::STATE_FORWARD);
        }
      }
    }
  }
}

void Robot::processGPSData()
{
  float nlat;
  float nlon;
  unsigned long age;
  m_gps.f_get_position(&nlat, &nlon, &age);

  if (nlat == m_gps.GPS_INVALID_F_ANGLE)
  {
    return;
  }

  if (m_gpsLon == 0)
  {
    m_gpsLon = nlon;  // this is xy (0,0)
    m_gpsLat = nlat;
    return;
  }

  m_gpsX = m_gps.distance_between(nlat, m_gpsLon, m_gpsLat, m_gpsLon);
  m_gpsY = m_gps.distance_between(m_gpsLat, nlon, m_gpsLat, m_gpsLon);
}

void Robot::checkTimeout()
{
  if (m_stateMachine.getStateTime() > m_wheels.m_forwardTimeMax)
  {
    setNextState(StateMachine::STATE_REVERSE, !m_rollDir);
  }
}


void Robot::runStateMachine()
{
  unsigned long curMillis = millis();

  // state machine - things to do *PERMANENTLY* for current state
  // robot state machine
  // http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
  switch (m_stateMachine.getCurrentState())
  {
    case StateMachine::STATE_ERROR:
      // fatal-error
      break;

    case StateMachine::STATE_OFF:
      // robot is turned off
      if (m_battery.isMonitored() &&
          curMillis - m_stateMachine.getStateStartTime() > 2000)
      {
        if (m_battery.getChargeVoltage_mV() > 5000 &&
            m_battery.getBatVoltage_mV() > 8000)
        {
          m_buzzer.beepShort(2);
          setNextState(StateMachine::STATE_STATION);
        }
      }
      m_imuDriveHeading = m_imu.getYaw();
      break;

    case StateMachine::STATE_MANUAL:
      break;

    case StateMachine::STATE_FORWARD:
      // driving forward
      if (m_mowPattern == MOW_BIDIR)
      {
        float ratio;

        if (m_stateMachine.getStateTime() > 4000)
        {
          ratio = m_wheels.m_biDirSpeedRatio2;
        }
        else
        {
          ratio = m_wheels.m_biDirSpeedRatio1;
        }

        auto motorLeft_p = &m_wheels.m_wheel[Wheel::LEFT].m_motor;
        auto motorRight_p = &m_wheels.m_wheel[Wheel::RIGHT].m_motor;

        if (m_rollDir == RIGHT)
        {
          motorRight_p->m_rpmSet = (float)motorLeft_p->m_rpmSet * ratio;
        }
        else
        {
          motorLeft_p->m_rpmSet = (float)motorRight_p->m_rpmSet * ratio;
        }
      }

      checkErrorCounter();
      checkTimer();

      if (m_rainSensor.isUsed())
      {
        checkRain();
      }

      checkMotorPower();
      checkBumpers();
      checkDrop();
      checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      checkTimeout();
      break;

    case StateMachine::STATE_ROLL:
      checkMotorPower();
      checkBumpers();
      checkDrop();
      //checkSonar();
      checkPerimeterBoundary();
      checkLawn();
      // making a roll (left/right)
      if (m_mowPattern == MOW_LANES)
      {
        if (abs(distancePI(m_imu.getYaw(), m_imuRollHeading)) < PI / 36)
        {
          setNextState(StateMachine::STATE_FORWARD);
        }
      }
      else
      {
        if (m_stateMachine.isStateEndTimeReached())
        {
          setNextState(StateMachine::STATE_FORWARD);
        }
      }
      break;

    case StateMachine::STATE_ROLL_WAIT:
      // making a roll (left/right)
      //if (abs(distancePI(imuYaw, imuRollHeading)) < PI/36) setNextState(StateMachine::STATE_OFF,0);
      break;

    case StateMachine::STATE_CIRCLE:
      // driving circles
      break;

    case StateMachine::STATE_REVERSE:
      // driving reverse
      checkErrorCounter();
      checkTimer();
      checkMotorPower();
      checkBumpers();
      checkDrop();
      //checkSonar();
      checkPerimeterBoundary();
      checkLawn();

      if (m_mowPattern == MOW_BIDIR)
      {
        float ratio;

        if (m_stateMachine.getStateTime() > 4000)
        {
          ratio = m_wheels.m_biDirSpeedRatio2;
        }
        else
        {
          ratio = m_wheels.m_biDirSpeedRatio1;
        }

        auto motorLeft_p = &m_wheels.m_wheel[Wheel::LEFT].m_motor;
        auto motorRight_p = &m_wheels.m_wheel[Wheel::RIGHT].m_motor;

        if (m_rollDir == RIGHT)
        {
          motorRight_p->m_rpmSet = (float)motorLeft_p->m_rpmSet * ratio;
        }
        else
        {
          motorLeft_p->m_rpmSet = (float)motorRight_p->m_rpmSet * ratio;
        }

        if (m_stateMachine.getStateTime() > m_wheels.m_forwardTimeMax)
        {
          // timeout
          setNextState(StateMachine::STATE_FORWARD, !m_rollDir);
        }
      }
      else
      {
        if (m_stateMachine.isStateEndTimeReached())
        {
          setNextState(StateMachine::STATE_ROLL, m_rollDir);
        }
      }
      break;

    case StateMachine::STATE_PERI_ROLL:
      // perimeter tracking roll
      if (m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_PERI_FIND);
      }
      break;

    case StateMachine::STATE_PERI_REV:
      // perimeter tracking reverse
      if (m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_PERI_ROLL, m_rollDir);
      }
      break;

    case StateMachine::STATE_PERI_FIND:
      // find perimeter
      if (m_wheels.getSteer() == 0)
      { // do not check during 'outside=>inside' rotation
        checkMotorPower();
        checkBumpersPerimeter();
        checkSonar();
      }
      checkPerimeterFind();
      checkTimeout();
      break;

    case StateMachine::STATE_PERI_TRACK:
      // track perimeter
      checkMotorPower();
      checkBumpersPerimeter();
      //checkSonar();
      if (m_battery.isMonitored())
      {
        if (m_battery.getChargeVoltage_mV() > 5000)
        {
          setNextState(StateMachine::STATE_STATION);
        }
      }
      break;

    case StateMachine::STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (m_battery.isMonitored())
      {
        if (m_battery.getChargeVoltage_mV() > 5000 &&
            m_battery.getBatVoltage_mV() > 8000)
        {
          if (m_battery.isVoltageBelowStartChargingLimit() &&
              m_stateMachine.getStateTime() > 2000)
          {
            setNextState(StateMachine::STATE_STATION_CHARGING);
          }
          else
          {
            checkTimer();
          }
        }
        else
        {
          setNextState(StateMachine::STATE_OFF);
        }
      }
      else
      {
        checkTimer();
      }
      break;

    case StateMachine::STATE_STATION_CHARGING:
      // waiting until charging completed
      if (m_battery.isMonitored())
      {
        if (m_battery.isChargeCurrentBelowBatFullLimit() &&
            m_stateMachine.getStateTime() > 2000)
        {
          setNextState(StateMachine::STATE_STATION);
        }
        else if (curMillis - m_stateMachine.getStateStartTime() > m_battery.m_chargingTimeout_ms)
        {
          incErrorCounter(ERR_BATTERY);
          setNextState(StateMachine::STATE_ERROR);
        }
      }
      break;

    case StateMachine::STATE_PERI_OUT_FORW:
      checkPerimeterBoundary();
      //if (millis() >= stateEndTime) setNextState(StateMachine::STATE_PERI_OUT_ROLL, rollDir);
      if (m_perimeterInside || m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_PERI_OUT_ROLL, m_rollDir);
      }
      break;

    case StateMachine::STATE_PERI_OUT_REV:
      checkPerimeterBoundary();
      // if (millis() >= stateEndTime) setNextState(StateMachine::STATE_PERI_OUT_ROLL, rollDir);
      if (m_perimeterInside || m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_PERI_OUT_ROLL, m_rollDir);
      }
      break;

    case StateMachine::STATE_PERI_OUT_ROLL:
      if (m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_FORWARD);
      }
      break;

    case StateMachine::STATE_STATION_CHECK:
      // check for charging voltage disappearing before leaving charging station
      if (m_stateMachine.isStateEndTimeReached())
      {
        if (m_battery.getChargeVoltage_mV() > 5000)
        {
          incErrorCounter(ERR_CHARGER);
          setNextState(StateMachine::STATE_ERROR);
        }
        else
        {
          setNextState(StateMachine::STATE_STATION_REV);
        }
      }
      break;

    case StateMachine::STATE_STATION_REV:
      // charging: drive reverse
      if (m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_STATION_ROLL);
      }
      break;

    case StateMachine::STATE_STATION_ROLL:
      // charging: roll
      if (m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_STATION_FORW);
      }
      break;

    case StateMachine::STATE_STATION_FORW:
      // forward (charge station)
      if (m_stateMachine.isStateEndTimeReached())
      {
        setNextState(StateMachine::STATE_FORWARD);
      }
      break;

    default:
      incErrorCounter(ERR_STATE_MACHINE);
      setNextState(StateMachine::STATE_ERROR);
      break;
  } // end switch
}

void Robot::tasks_continuous()
{
  ADCMan.run();
  readSerial();

  if (m_rc.readSerial())
  {
    resetIdleTime();
  }

  checkOdometerFaults();

  if (m_imu.isUsed())
  {
    m_imu.update();
  }

  runStateMachine();

  m_bumpers.clearHit();
  m_dropSensors.clearDetected();

  m_loopsPerSecCounter++;
}

void Robot::tasks_50ms()
{
  checkButton();
  readMotorCurrents();

  if (m_perimeters.isUsed())
  {
    readPerimeters();
  }
}

void Robot::tasks_100ms()
{
  if (m_bumpers.isUsed())
  {
    m_bumpers.check();
  }

  if (m_lawnSensors.isUsed())
  {
    m_lawnSensors.read();
  }

  if (m_dropSensors.isUsed())
  {
    m_dropSensors.check();
  }

  // Decide which motor control to use
  if (m_imu.isUsed() &&
      ((m_mowPattern == MOW_LANES &&
        m_stateMachine.isCurrentState(StateMachine::STATE_ROLL)) ||
       m_stateMachine.isCurrentState(StateMachine::STATE_ROLL_WAIT)))
  {
    wheelControl_imuRoll();
  }
  else if (m_perimeters.isUsed() &&
           m_stateMachine.isCurrentState(StateMachine::STATE_PERI_TRACK))
  {
    wheelControl_perimeter();
  }
  else if (m_imu.isUsed() &&
           (m_stateMachine.isCurrentState(StateMachine::STATE_FORWARD) &&
            (m_imu.isCorrectDir() || m_mowPattern == MOW_LANES)))
  {
    wheelControl_imuDir();
  }
  else
  {
    wheelControl_normal();
  }

  m_wheels.control();
  cutterControl();

  m_battery.read();
}

void Robot::tasks_200ms()
{
  m_rc.run();

  if (m_sonars.isUsed())
  {
    checkSonar();
  }

  if (m_imu.isUsed())
  {
    readImu();
    checkTilt();
  }
}

void Robot::tasks_250ms()
{
  if (m_sonars.isUsed())
  {
    m_sonars.ping();
  }
}

void Robot::tasks_300ms()
{
  checkIfStuck();
  if (m_odometer.isUsed())
  {
    m_odometer.calc();
  }

  if (m_stateMachine.isCurrentState(StateMachine::STATE_ERROR))
  {
    m_buzzer.beepShort();
  }
}

void Robot::tasks_500ms()
{
  measureCutterMotorRpm();
}

void Robot::tasks_1s()
{
  checkBattery();

  if (m_gpsUse)
  {
    m_gps.feed();
    processGPSData();
  }

  m_loopsPerSec = m_loopsPerSecCounter;
  m_loopsPerSecCounter = 0;

  printInfo(Console);
}

void Robot::tasks_2s()
{
  if (m_lawnSensors.isUsed())
  {
    m_lawnSensors.check();
  }
}

void Robot::tasks_5s()
{
  if (m_rainSensor.isUsed())
  {
    m_rainSensor.check();
  }
}

void Robot::tasks_1m()
{
  checkTimer();
  readRtc();
  checkRobotStats_mowTime();
  checkRobotStats_battery();
}
