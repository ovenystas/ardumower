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

// Android remote control (pfod App)
// For a detailed specification of the pfodApp protocol, please visit:  http://www.forward.com.au/pfod/
// example usage:
//   RemoteControl remote;
//   remote.initSerial(19200);
//   while (true){
//     remote.readSerial();
//     remote.run();
//  }
#pragma once

#include <Arduino.h>

#include "drivers.h"
#include "Pid.h"

// pfodApp state
enum
{
  PFOD_OFF,
  PFOD_MENU,
  PFOD_LOG_SENSORS,
  PFOD_PLOT_BAT,
  PFOD_PLOT_ODO2D,
  PFOD_PLOT_IMU,
  PFOD_PLOT_SENSOR_COUNTERS,
  PFOD_PLOT_SENSORS,
  PFOD_PLOT_PERIMETER,
  PFOD_PLOT_GPS,
  PFOD_PLOT_GPS2D,
  PFOD_PLOT_MOTOR
};

class Robot;

class RemoteControl
{
public:
  RemoteControl();
  void setRobot(Robot* robot_p);
  void initSerial(const int baudrate);
  bool readSerial();
  void run();

private:
  Robot* m_robot_p;
  bool m_pfodCmdComplete;
  String m_pfodCmd;
  byte m_pfodState;
  int m_testmode;
  unsigned long m_nextPlotTime;
  int8_t m_perimeterCapture[32];
  int m_perimeterCaptureIdx { 0 };

  float stringToFloat(const String& s);
  void parsePfodCmd();

  // generic
  void sendYesNo(const bool value);
  void sendOnOff(const bool value);

  // PID slider
  void sendPIDSlider(const String cmd, const String title, const Pid& pid,
      const double scale, const float maxvalue);
  void processPIDSlider(const String result, const String cmd, Pid& pid,
      const double scale, const float maxvalue);

  // generic slider
  void sendSlider(const String cmd, const String title, const float value,
      const String unit, const double scale, const float maxvalue,
      const float minvalue = 0);
  void processSlider(const String result, float& value, const double scale);
  void processSlider(const String result, long& value, const double scale);
  void processSlider(const String result, unsigned long& value,
      const double scale);
  void processSlider(const String result, int& value, const double scale);
  void processSlider(const String result, unsigned int& value,
      const double scale);
  void processSlider(const String result, byte& value, const double scale);
  void processSlider(const String result, short& value, const double scale);

  // send timer menu details
  void sendTimer(const ttimer_t timer);

  // main menu
  void sendMainMenu(const bool update);
  void sendErrorMenu(const bool update);
  void sendInfoMenu(const bool update);
  void sendCommandMenu(const bool update);
  void processCommandMenu(const String pfodCmd);
  void sendManualMenu(const bool update);
  void sendCompassMenu(const bool update);
  void processCompassMenu(const String pfodCmd);
  void processManualMenu(const String pfodCmd);
  void processSettingsMenu(const String pfodCmd);

  // plotting
  void sendPlotMenu(const bool update);

  // settings
  void sendSettingsMenu(const bool update);
  void sendMotorMenu(const bool update);
  void sendMowMenu(const bool update);
  void sendBumperMenu(const bool update);
  void sendDropMenu(const bool update);
  void sendSonarMenu(const bool update);
  void sendPerimeterMenu(const bool update);
  void sendLawnSensorMenu(const bool update);
  void sendImuMenu(const bool update);
  void sendRemoteMenu(const bool update);
  void sendBatteryMenu(const bool update);
  void sendStationMenu(const bool update);
  void sendOdometerMenu(const bool update);
  void sendRainMenu(const bool update);
  void sendGPSMenu(const bool update);
  void sendDateTimeMenu(const bool update);
  void sendFactorySettingsMenu(const bool update);
  void sendADCMenu(const bool update);

  void processMotorMenu(const String pfodCmd);
  void processErrorMenu(const String pfodCmd);
  void processMowMenu(const String pfodCmd);
  void processBumperMenu(const String pfodCmd);
  void processSonarMenu(const String pfodCmd);
  void processPerimeterMenu(const String pfodCmd);
  void processLawnSensorMenu(const String pfodCmd);
  void processRainMenu(const String pfodCmd);
  void processDropMenu(const String pfodCmd);
  void processGPSMenu(const String pfodCmd);
  void processImuMenu(const String pfodCmd);
  void processRemoteMenu(const String pfodCmd);
  void processBatteryMenu(const String pfodCmd);
  void processStationMenu(const String pfodCmd);
  void processOdometerMenu(const String pfodCmd);
  void processDateTimeMenu(const String pfodCmd);
  void processFactorySettingsMenu(const String pfodCmd);
  void processInfoMenu(const String pfodCmd);

  // timer
  void sendTimerDetailMenu(const int timerIdx, const bool update);
  void processTimerDetailMenu(const String pfodCmd);
  void sendTimerMenu(const bool update);
  void processTimerMenu(const String pfodCmd);
};
