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
enum class PfodState
{
  OFF,
  MENU,
  LOG_SENSORS,
  PLOT_BAT,
  PLOT_ODO2D,
  PLOT_IMU,
  PLOT_SENSOR_COUNTERS,
  PLOT_SENSORS,
  PLOT_PERIMETER,
  PLOT_GPS,
  PLOT_GPS2D,
  PLOT_MOTOR
};

class Robot;

class RemoteControl
{
public:
  RemoteControl(Robot* robot_p) : m_robot_p(robot_p) {}

  void initSerial(int baudrate);
  bool readSerial();
  void run();

private:
  float stringToFloat(const String& s);
  void parsePfodCmd();

  // generic
  void sendYesNo(bool value);
  void sendOnOff(bool value);

  // PID slider
  void sendPIDSlider(String cmd, String title, Pid& pid, float scale,
      float maxvalue);
  void processPIDSlider(String result, String cmd, Pid& pid, float scale,
      float maxvalue);

  // generic slider
  void sendSlider(String cmd, String title, float value, String unit,
      float scale, float maxvalue, float minvalue = 0);
  void processSlider(String result, float& value, float scale);
  void processSlider(String result, long& value, float scale);
  void processSlider(String result, unsigned long& value, float scale);
  void processSlider(String result, int& value, float scale);
  void processSlider(String result, unsigned int& value, float scale);
  void processSlider(String result, byte& value, float scale);
  void processSlider(String result, short& value, float scale);

  // send timer menu details
  void sendTimer(ttimer_t timer);

  // main menu
  void sendMainMenu(bool update);
  void sendErrorMenu(bool update);
  void sendInfoMenu(bool update);
  void sendCommandMenu(bool update);
  void processCommandMenu(String pfodCmd);
  void sendManualMenu(bool update);
  void sendCompassMenu(bool update);
  void processCompassMenu(String pfodCmd);
  void processManualMenu(String pfodCmd);
  void processSettingsMenu(String pfodCmd);

  // plotting
  void sendPlotMenu(bool update);

  // settings
  void sendSettingsMenu(bool update);
  void sendMotorMenu(bool update);
  void sendMowMenu(bool update);
  void sendBumperMenu(bool update);
  void sendDropMenu(bool update);
  void sendSonarMenu(bool update);
  void sendPerimeterMenu(bool update);
  void sendLawnSensorMenu(bool update);
  void sendImuMenu(bool update);
  void sendBatteryMenu(bool update);
  void sendStationMenu(bool update);
  void sendOdometerMenu(bool update);
  void sendRainMenu(bool update);
  void sendGPSMenu(bool update);
  void sendDateTimeMenu(bool update);
  void sendFactorySettingsMenu(bool update);
  void sendADCMenu(bool update);

  void processMotorMenu(String pfodCmd);
  void processErrorMenu(String pfodCmd);
  void processMowMenu(String pfodCmd);
  void processBumperMenu(String pfodCmd);
  void processSonarMenu(String pfodCmd);
  void processPerimeterMenu(String pfodCmd);
  void processLawnSensorMenu(String pfodCmd);
  void processRainMenu(String pfodCmd);
  void processDropMenu(String pfodCmd);
  void processGPSMenu(String pfodCmd);
  void processImuMenu(String pfodCmd);
  void processBatteryMenu(String pfodCmd);
  void processStationMenu(String pfodCmd);
  void processOdometerMenu(String pfodCmd);
  void processDateTimeMenu(String pfodCmd);
  void processFactorySettingsMenu(String pfodCmd);
  void processInfoMenu(String pfodCmd);

  // timer
  void sendTimerDetailMenu(int timerIdx, bool update);
  void processTimerDetailMenu(String pfodCmd);
  void sendTimerMenu(bool update);
  void processTimerMenu(String pfodCmd);

private:
  Robot* m_robot_p { nullptr };
  bool m_pfodCmdComplete { false };
  String m_pfodCmd { "" };
  PfodState m_pfodState { PfodState::OFF };
  int m_testmode {};
  unsigned long m_nextPlotTime {};
  int8_t m_perimeterCapture[32];
  int m_perimeterCaptureIdx {};
};
