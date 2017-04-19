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

// Android remote control (pfod App)
// For a detailed specification of the pfodApp protocol, please visit:  http://www.forward.com.au/pfod/
#include "AdcManager.h"
#include "Imu.h"
#include "Perimeter.h"
#include "RemoteControl.h"
#include "Robot.h"

#define TOGGLE(x) (x) = !(x)

RemoteControl::RemoteControl()
{
  pfodCmdComplete = false;
  pfodCmd = "";
  pfodState = PFOD_OFF;
  testmode = 0;
  nextPlotTime = 0;
  perimeterCaptureIdx = 0;
  robot_p = nullptr;
}

void RemoteControl::setRobot(Robot* robot_p)
{
  this->robot_p = robot_p;
}

void RemoteControl::initSerial(const int baudrate)
{
  Bluetooth.begin(baudrate);   // pfod app
}

float RemoteControl::stringToFloat(const String &s)
{
  char tmp[20];
  s.toCharArray(tmp, sizeof(tmp));
  float v = atof(tmp);
  return v;
}

void RemoteControl::sendYesNo(const bool value)
{
  if (value)
  {
    Bluetooth.print("YES");
  }
  else
  {
    Bluetooth.print("NO");
  }
}

void RemoteControl::sendOnOff(const bool value)
{
  if (value)
  {
    Bluetooth.print("ON");
  }
  else
  {
    Bluetooth.print("OFF");
  }
}

void RemoteControl::sendTimer(const ttimer_t timer)
{
  if (timer.active)
  {
    Bluetooth.print(F("(X)  "));
  }
  else
  {
    Bluetooth.print(F("(   )  "));
  }
  Bluetooth.print(time2str(timer.startTime));
  Bluetooth.print("-");
  Bluetooth.print(time2str(timer.stopTime));
  Bluetooth.println();
  if (timer.daysOfWeek == B1111111)
  {
    Bluetooth.print(F("every day"));
  }
  else
  {
    int counter = 0;
    for (int j = 0; j < 7; j++)
    {
      if ((timer.daysOfWeek >> j) & 1)
      {
        if (counter != 0)
        {
          Bluetooth.print(",");
        }
        Bluetooth.print(dayOfWeek[j]);
        counter++;
      }
    }
  }
}

void RemoteControl::sendSlider(const String cmd, const String title,
                               const float value, const String unit,
                               const double scale, const float maxvalue,
                               const float minvalue)
{
  (void)unit; //FIXME: Remove unused parameter unit or start using it

  Bluetooth.print("|");
  Bluetooth.print(cmd);
  Bluetooth.print("~");
  Bluetooth.print(title);
  Bluetooth.print(" `");
  Bluetooth.print(((int)(value / scale)));
  Bluetooth.print("`");
  Bluetooth.print(((int)(maxvalue / scale)));
  Bluetooth.print("`");
  Bluetooth.print(((int)(minvalue / scale)));
  Bluetooth.print("~ ~");
  if (scale == 10)
  {
    Bluetooth.print("10");
  }
  else if (scale == 1)
  {
    Bluetooth.print("1");
  }
  else if (scale == 0.1)
  {
    Bluetooth.print("0.1");
  }
  else if (scale == 0.01)
  {
    Bluetooth.print("0.01");
  }
  else if (scale == 0.001)
  {
    Bluetooth.print("0.001");
  }
  else if (scale == 0.0001)
  {
    Bluetooth.print("0.0001");
  }
}

void RemoteControl::sendPIDSlider(const String cmd, const String title,
                                  const Pid &pid, const double scale,
                                  const float maxvalue)
{
  sendSlider(cmd + "p", title + " P", pid.settings.Kp, "", scale, maxvalue);
  sendSlider(cmd + "i", title + " I", pid.settings.Ki, "", scale, maxvalue);
  sendSlider(cmd + "d", title + " D", pid.settings.Kd, "", scale, maxvalue);
}

void RemoteControl::processPIDSlider(const String result, const String cmd,
                                     Pid &pid, const double scale,
                                     const float maxvalue)
{
  (void)maxvalue; //FIXME: Warning unused parameter

  int idx = result.indexOf('`');
  String s = result.substring(idx + 1);
  //Console.println(tmp);
  float v = stringToFloat(s);
  if (pfodCmd.startsWith(cmd + "p"))
  {
    pid.settings.Kp = v * scale;
    if (pid.settings.Kp < scale)
    {
      pid.settings.Kp = 0.0;
    }
  }
  else if (pfodCmd.startsWith(cmd + "i"))
  {
    pid.settings.Ki = v * scale;
    if (pid.settings.Ki < scale)
    {
      pid.settings.Ki = 0.0;
    }
  }
  else if (pfodCmd.startsWith(cmd + "d"))
  {
    pid.settings.Kd = v * scale;
    if (pid.settings.Kd < scale)
    {
      pid.settings.Kd = 0.0;
    }
  }
}

void RemoteControl::processSlider(const String result, float &value,
                                  const double scale)
{
  int idx = result.indexOf('`');
  String s = result.substring(idx + 1);
  float v = stringToFloat(s);
  value = v * scale;
}

void RemoteControl::processSlider(const String result, long &value,
                                  const double scale)
{
  float v;
  processSlider(result, v, scale);
  value = v;
}

void RemoteControl::processSlider(const String result, unsigned long &value,
                                  const double scale)
{
  float v;
  processSlider(result, v, scale);
  value = v;
}

void RemoteControl::processSlider(const String result, int &value,
                                  const double scale)
{
  float v;
  processSlider(result, v, scale);
  value = v;
}

void RemoteControl::processSlider(const String result, unsigned int &value,
                                  const double scale)
{
  float v;
  processSlider(result, v, scale);
  value = v;
}

void RemoteControl::processSlider(const String result, byte &value,
                                  const double scale)
{
  float v;
  processSlider(result, v, scale);
  value = v;
}

void RemoteControl::processSlider(const String result, short &value,
                                  const double scale)
{
  float v;
  processSlider(result, v, scale);
  value = v;
}

void RemoteControl::sendMainMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Ardumower ("));
    Bluetooth.print(robot_p->name);
    Bluetooth.print(")");
  }
  Bluetooth.println(F("|r~Commands|n~Manual|s~Settings|in~Info|c~Test compass"
                      "|m1~Log sensors|yp~Plot|y4~Error counters"
                      "|y9~ADC calibration}"));
}

void RemoteControl::sendADCMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.ADC calibration`1000"));
  }
  Bluetooth.print(F("|c1~Calibrate (perimeter sender, charger must be off) "));
  for (uint8_t ch = 0; ch < 16; ch++)
  {
    const int16_t adcMin = ADCMan.getAdcMinCh(ch);
    const int16_t adcMax = ADCMan.getAdcMaxCh(ch);
    const int16_t adcZeroOffset = ADCMan.getAdcZeroOffsetCh(ch);
    Bluetooth.print(F("|zz~AD"));
    Bluetooth.print(ch);
    Bluetooth.print(" min=");
    Bluetooth.print(adcMin);
    Bluetooth.print(" max=");
    Bluetooth.print(adcMax);
    Bluetooth.print(" diff=");
    Bluetooth.print(adcMax - adcMin);
    Bluetooth.print(" ofs=");
    Bluetooth.print(adcZeroOffset);
  }
  Bluetooth.println("}");
}

void RemoteControl::sendPlotMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Plot"));
  }
  Bluetooth.println(F("|y7~Sensors|y5~Sensor counters|y3~IMU|y6~Perimeter"
                      "|y8~GPS|y1~Battery|y2~Odometer2D|y11~Motor control"
                      "|y10~GPS2D}"));
}

void RemoteControl::sendSettingsMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Settings"));
  }
  Bluetooth.println(F("|sz~Save settings|s1~Motor|s2~Mow|s3~Bumper|s4~Sonar"
                      "|s5~Perimeter|s6~Lawn sensor|s7~IMU|s8~R/C"
                      "|s9~Battery|s10~Station|s11~Odometer|s13~Rain"
                      "|s15~Drop sensor|s14~GPS|i~Timer|s12~Date/time"
                      "|sx~Factory settings}"));
}

void RemoteControl::sendErrorMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Error counters`1000"));
  }
  Bluetooth.print(F("|z00~Reset counters"));
  Bluetooth.print(F("|zz~ADC calibration "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_ADC_CALIB]);
  Bluetooth.print(F("|zz~Charger "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_CHARGER]);
  Bluetooth.print(F("|zz~Battery "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_BATTERY]);
  Bluetooth.print(F("|zz~Motor left "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_MOTOR_LEFT]);
  Bluetooth.print(F("|zz~Motor right "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_MOTOR_RIGHT]);
  Bluetooth.print(F("|zz~Motor mow "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_MOTOR_CUTTER]);
  Bluetooth.print(F("|zz~Mow sense "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_CUTTER_SENSE]);
  Bluetooth.print(F("|zz~Odometer left "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_ODOMETER_LEFT]);
  Bluetooth.print(F("|zz~Odometer right "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_ODOMETER_RIGHT]);
  Bluetooth.print(F("|zz~Perimeter timeout "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_PERIMETER_TIMEOUT]);
  Bluetooth.print(F("|zz~Perimeter tracking "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_TRACKING]);
  Bluetooth.print(F("|zz~IMU comm "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_IMU_COMM]);
  Bluetooth.print(F("|zz~IMU calibration "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_IMU_CALIB]);
  Bluetooth.print(F("|zz~IMU tilt "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_IMU_TILT]);
  Bluetooth.print(F("|zz~RTC comm "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_RTC_COMM]);
  Bluetooth.print(F("|zz~RTC data "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_RTC_DATA]);
  Bluetooth.print(F("|zz~GPS comm "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_GPS_COMM]);
  Bluetooth.print(F("|zz~GPS data "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_GPS_DATA]);
  Bluetooth.print(F("|zz~Robot stucked "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_STUCK]);
  Bluetooth.print(F("|zz~EEPROM data "));
  Bluetooth.print(robot_p->errorCounterMax[ERR_EEPROM_DATA]);
  Bluetooth.println("}");
}

void RemoteControl::processErrorMenu(const String pfodCmd)
{
  if (pfodCmd == "z00")
  {
    robot_p->resetErrorCounters();
    robot_p->setNextState(StateMachine::STATE_OFF, 0);
  }
  sendErrorMenu(true);
}

void RemoteControl::sendMotorMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Motor`1000"));
  }
  Bluetooth.println(F("|a00~Overload Counter l, r "));
  Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getOverloadCounter());
  Bluetooth.print(", ");
  Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getOverloadCounter());
  Bluetooth.println(F("|a01~Power in Watt l, r "));
  Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getPowerMeas());
  Bluetooth.print(", ");
  Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getPowerMeas());
  Bluetooth.println(F("|a05~motor current in mA l, r "));
  Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getAverageCurrent());
  Bluetooth.print(", ");
  Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getAverageCurrent());
  sendSlider("a02", F("Power max"),
             robot_p->wheels.wheel[Wheel::LEFT].motor.powerMax, "", 1, 100);
  sendSlider("a03", F("calibrate left motor "),
             robot_p->wheels.wheel[Wheel::LEFT].motor.getAverageCurrent(),
             "", 1, 1000, 0);
  sendSlider("a04", F("calibrate right motor"),
             robot_p->wheels.wheel[Wheel::RIGHT].motor.getAverageCurrent(),
             "", 1, 1000, 0);
  Bluetooth.print(F("|a05~Speed l, r"));
  Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getPwmCur());
  Bluetooth.print(", ");
  Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getPwmCur());
  sendSlider("a06", F("Speed max in rpm"),
             robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax, "", 1, 100);
  sendSlider("a15", F("Speed max in pwm"),
             robot_p->wheels.wheel[Wheel::LEFT].motor.pwmMax, "", 1, 255);
  sendSlider("a11", F("Accel"),
             robot_p->wheels.wheel[Wheel::LEFT].motor.acceleration,
             "", 1, 2000, 500);
  sendSlider("a18", F("Power ignore time"),
             robot_p->wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime,
             "", 1, 8000);
  sendSlider("a07", F("Roll time max"),
             robot_p->wheels.rollTimeMax, "", 1, 8000);
  sendSlider("a19", F("Roll time min"),
             robot_p->wheels.rollTimeMin, "", 1,
             (robot_p->wheels.rollTimeMax - 500));
  sendSlider("a08", F("Reverse time"),
             robot_p->wheels.reverseTime, "", 1, 8000);
  sendSlider("a09", F("Forw time max"),
             robot_p->wheels.forwardTimeMax, "", 10, 80000);
  sendSlider("a12", F("Bidir speed ratio 1"),
             robot_p->wheels.biDirSpeedRatio1, "", 0.01, 1.0);
  sendSlider("a13", F("Bidir speed ratio 2"),
             robot_p->wheels.biDirSpeedRatio2, "", 0.01, 1.0);
  sendPIDSlider("a14", "RPM",
                robot_p->wheels.wheel[Wheel::LEFT].motor.pid, 0.01, 3.0);
  Bluetooth.println(F("|a10~Testing is"));

  switch (testmode)
  {
    case 0:
      Bluetooth.print(F("OFF"));
      break;

    case 1:
      Bluetooth.print(F("Left motor forw"));
      break;

    case 2:
      Bluetooth.print(F("Right motor forw"));
      break;
  }

  Bluetooth.print(F("|a14~for config file:"));
  Bluetooth.print(F("motorSenseScale l, r"));
  Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getScale());
  Bluetooth.print(", ");
  Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getScale());
  Bluetooth.print(F("|a16~Swap left direction "));
  sendYesNo(robot_p->wheels.wheel[Wheel::LEFT].motor.swapDir);
  Bluetooth.print(F("|a17~Swap right direction "));
  sendYesNo(robot_p->wheels.wheel[Wheel::RIGHT].motor.swapDir);
  Bluetooth.println("}");
}

void RemoteControl::processMotorMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("a02"))
  {
    processSlider(pfodCmd,
                  robot_p->wheels.wheel[Wheel::LEFT].motor.powerMax, 1);
    processSlider(pfodCmd,
                  robot_p->wheels.wheel[Wheel::RIGHT].motor.powerMax, 1);
  }

  else if (pfodCmd.startsWith("a03"))
  {
    float currentMeas =
        robot_p->wheels.wheel[Wheel::LEFT].motor.getAverageCurrent();
    processSlider(pfodCmd, currentMeas, 1);
    robot_p->wheels.wheel[Wheel::LEFT].motor.setScale(
        robot_p->wheels.wheel[Wheel::LEFT].motor.getScale() /
        max(0, (float)robot_p->wheels.wheel[Wheel::LEFT].motor.getAverageSenseAdc()));
  }
  else if (pfodCmd.startsWith("a04"))
  {
    float currentMeas =
        robot_p->wheels.wheel[Wheel::RIGHT].motor.getAverageCurrent();
    processSlider(pfodCmd, currentMeas, 1);
    robot_p->wheels.wheel[Wheel::RIGHT].motor.setScale(
        robot_p->wheels.wheel[Wheel::RIGHT].motor.getScale() /
        max(0, (float)robot_p->wheels.wheel[Wheel::RIGHT].motor.getAverageSenseAdc()));
  }
  else if (pfodCmd.startsWith("a06"))
  {
    processSlider(pfodCmd, robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax, 1);
    processSlider(pfodCmd, robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmMax, 1);
  }
  else if (pfodCmd.startsWith("a15"))
  {
    processSlider(pfodCmd, robot_p->wheels.wheel[Wheel::LEFT].motor.pwmMax, 1);
    processSlider(pfodCmd, robot_p->wheels.wheel[Wheel::RIGHT].motor.pwmMax, 1);
  }
  else if (pfodCmd.startsWith("a07"))
  {
    processSlider(pfodCmd, robot_p->wheels.rollTimeMax, 1);
  }
  else if (pfodCmd.startsWith("a19"))
  {
    processSlider(pfodCmd, robot_p->wheels.rollTimeMin, 1);
  }
  else if (pfodCmd.startsWith("a08"))
  {
    processSlider(pfodCmd, robot_p->wheels.reverseTime, 1);
  }
  else if (pfodCmd.startsWith("a09"))
  {
    processSlider(pfodCmd, robot_p->wheels.forwardTimeMax, 10);
  }
  else if (pfodCmd.startsWith("a11"))
  {
    processSlider(pfodCmd,
                  robot_p->wheels.wheel[Wheel::LEFT].motor.acceleration, 1);
    processSlider(pfodCmd,
                  robot_p->wheels.wheel[Wheel::RIGHT].motor.acceleration, 1);
  }
  else if (pfodCmd.startsWith("a12"))
  {
    processSlider(pfodCmd, robot_p->wheels.biDirSpeedRatio1, 0.01);
  }
  else if (pfodCmd.startsWith("a13"))
  {
    processSlider(pfodCmd, robot_p->wheels.biDirSpeedRatio2, 0.01);
  }
  else if (pfodCmd.startsWith("a14"))
  {
    processPIDSlider(pfodCmd, "a14",
                     robot_p->wheels.wheel[Wheel::LEFT].motor.pid, 0.01, 3.0);
    processPIDSlider(pfodCmd, "a14",
                     robot_p->wheels.wheel[Wheel::RIGHT].motor.pid, 0.01, 3.0);
  }
  else if (pfodCmd.startsWith("a16"))
  {
    TOGGLE(robot_p->wheels.wheel[Wheel::LEFT].motor.swapDir);
  }
  else if (pfodCmd.startsWith("a17"))
  {
    TOGGLE(robot_p->wheels.wheel[Wheel::RIGHT].motor.swapDir);
  }
  else if (pfodCmd.startsWith("a18"))
  {
    processSlider(pfodCmd,
                  robot_p->wheels.wheel[Wheel::LEFT].motor.powerIgnoreTime, 1);
    processSlider(pfodCmd,
                  robot_p->wheels.wheel[Wheel::RIGHT].motor.powerIgnoreTime, 1);
  }
  else if (pfodCmd == "a10")
  {
    testmode = (testmode + 1) % 3;
    switch (testmode)
    {
      case 0:
        robot_p->setNextState(StateMachine::STATE_OFF, 0);
        robot_p->setSpeed(0);
        robot_p->setSteer(0);
        break;

      case 1:
        robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
        robot_p->setSpeed(+50);
        robot_p->setSteer(+50);
//        robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet = 0;
//        robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
//            robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;
        break;

      case 2:
        robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
        robot_p->setSpeed(+50);
        robot_p->setSteer(-50);
//        robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet = 0;
//        robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
//            robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmMax;
        break;
    }
  }
  sendMotorMenu(true);
}

void RemoteControl::sendMowMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Mow`1000"));
  }
  Bluetooth.print(F("|o00~Overload Counter "));
  Bluetooth.print(robot_p->cutter.motor.getOverloadCounter());
  Bluetooth.print(F("|o01~Power in Watt "));
  Bluetooth.print(robot_p->cutter.motor.getPowerMeas());
  Bluetooth.print(F("|o11~current in mA "));
  Bluetooth.print(robot_p->cutter.motor.getAverageCurrent());
  sendSlider("o02", F("Power max"), robot_p->cutter.motor.powerMax, "", 1, 100);
  sendSlider("o03", F("calibrate mow motor "),
             robot_p->cutter.motor.getAverageCurrent(), "", 1, 3000, 0);
  Bluetooth.print(F("|o04~Speed "));
  Bluetooth.print(robot_p->cutter.motor.getPwmCur());
  sendSlider("o05", F("Speed max"), robot_p->cutter.motor.pwmMax, "", 1, 255);
  if (robot_p->developerActive)
  {
    Bluetooth.print(F("|o06~Modulate "));
    sendYesNo(robot_p->cutter.motor.regulate);
  }
  Bluetooth.print(F("|o07~RPM "));
  Bluetooth.print(robot_p->cutter.motor.getRpmMeas());
  sendSlider("o08", F("RPM set"), robot_p->cutter.motor.rpmSet, "", 1, 4500);
  sendPIDSlider("o09", "RPM", robot_p->cutter.motor.pid, 0.01, 1.0);
  Bluetooth.println(F("|o10~Testing is"));

  switch (testmode)
  {
    case 0:
      Bluetooth.print(F("OFF"));
      break;

    case 1:
      Bluetooth.print(F("Motor ON"));
      break;
  }

  Bluetooth.println(F("|o04~for config file:"));
  Bluetooth.println(F("cutter.motor.getScale():"));
  Bluetooth.print(robot_p->cutter.motor.getScale());
  Bluetooth.println("}");
}

void RemoteControl::processMowMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("o02"))
  {
    processSlider(pfodCmd, robot_p->cutter.motor.powerMax, 1);
  }
  else if (pfodCmd.startsWith("o03"))
  {
    float currentMeas = robot_p->cutter.motor.getAverageCurrent();
    processSlider(pfodCmd, currentMeas, 1);
    robot_p->cutter.motor.setScale(currentMeas /
        max(0, (float )robot_p->cutter.motor.getAverageSenseAdc()));
  }
  else if (pfodCmd.startsWith("o05"))
  {
    processSlider(pfodCmd, robot_p->cutter.motor.pwmMax, 1);
  }
  else if (pfodCmd == "o06")
  {
    TOGGLE(robot_p->cutter.motor.regulate);
  }
  else if (pfodCmd.startsWith("o08"))
  {
    processSlider(pfodCmd, robot_p->cutter.motor.rpmSet, 1);
  }
  else if (pfodCmd.startsWith("o09"))
  {
    processPIDSlider(pfodCmd, "o09", robot_p->cutter.motor.pid, 0.01, 1.0);
  }
  else if (pfodCmd == "o10")
  {
    testmode = (testmode + 1) % 2;
    switch (testmode)
    {
      case 0:
        robot_p->setNextState(StateMachine::STATE_OFF, 0);
        robot_p->cutter.motor.setRpmMeas(0);
        robot_p->cutter.disable();
        break;

      case 1:
        robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
        robot_p->cutter.enable();
        break;
    }
  }
  sendMowMenu(true);
}

void RemoteControl::sendBumperMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Bumper`1000"));
  }
  Bluetooth.print(F("|b00~Use "));
  sendYesNo(robot_p->bumpers.use);
  Bluetooth.println(F("|b01~Counter l, r "));
  Bluetooth.print(bumper_getCounter(&robot_p->bumperArray[LEFT]));
  Bluetooth.print(", ");
  Bluetooth.print(bumper_getCounter(&robot_p->bumperArray[RIGHT]));
  Bluetooth.println(F("|b02~Value l, r "));
  Bluetooth.print(bumper_isHit(&robot_p->bumperArray[LEFT]));
  Bluetooth.print(", ");
  Bluetooth.print(bumper_isHit(&robot_p->bumperArray[RIGHT]));
  Bluetooth.println("}");
}

void RemoteControl::sendDropMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Drop`1000"));
  }
  Bluetooth.print(F("|u00~Use "));
  sendYesNo(robot_p->dropSensors.use);
  Bluetooth.println(F("|u01~Counter l, r "));
  Bluetooth.print(dropSensor_getCounter(
      &robot_p->dropSensors.dropSensorArray_p[LEFT]));
  Bluetooth.print(", ");
  Bluetooth.print(dropSensor_getCounter(
      &robot_p->dropSensors.dropSensorArray_p[RIGHT]));
  Bluetooth.println(F("|u02~Value l, r "));
  Bluetooth.print(dropSensor_isDetected(
      &robot_p->dropSensors.dropSensorArray_p[LEFT]));
  Bluetooth.print(", ");
  Bluetooth.print(dropSensor_isDetected(
      &robot_p->dropSensors.dropSensorArray_p[RIGHT]));
  Bluetooth.println("}");
}

void RemoteControl::processBumperMenu(const String pfodCmd)
{
  if (pfodCmd == "b00")
  {
    TOGGLE(robot_p->bumpers.use);
  }
  sendBumperMenu(true);
}

void RemoteControl::processDropMenu(const String pfodCmd)
{
  if (pfodCmd == "u00")
  {
    TOGGLE(robot_p->dropSensors.use);
  }
  sendDropMenu(true);
}

void RemoteControl::sendSonarMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Sonar`1000"));
  }
  Bluetooth.print(F("|d00~Use "));
  sendYesNo(robot_p->sonars.use);
  Bluetooth.print(F("|d04~Use left "));
  sendYesNo(robot_p->sonars.sonar[Sonars::LEFT].use);
  Bluetooth.print(F("|d05~Use center "));
  sendYesNo(robot_p->sonars.sonar[Sonars::CENTER].use);
  Bluetooth.print(F("|d06~Use right "));
  sendYesNo(robot_p->sonars.sonar[Sonars::RIGHT].use);
  Bluetooth.print(F("|d01~Counter "));
  Bluetooth.print(robot_p->sonars.getDistanceCounter());
  Bluetooth.println(F("|d02~Value [cm] l, c, r"));
  for (uint8_t i = 0; i < Sonars::END; i++)
  {
    Bluetooth.print(robot_p->sonars.sonar[i].getDistance_cm());
    if (i < Sonars::END - 1)
    {
      Bluetooth.print(", ");
    }
  }
  sendSlider("d03", F("Trigger below [us]"), robot_p->sonars.triggerBelow, "", 1, 3000);
  Bluetooth.println("}");
}

void RemoteControl::processSonarMenu(const String pfodCmd)
{
  if (pfodCmd == "d00")
  {
    TOGGLE(robot_p->sonars.use);
  }
  else if (pfodCmd.startsWith("d03"))
  {
    processSlider(pfodCmd, robot_p->sonars.triggerBelow, 1);
  }
  else if (pfodCmd == "d04")
  {
    TOGGLE(robot_p->sonars.sonar[Sonars::LEFT].use);
  }
  else if (pfodCmd == "d05")
  {
    TOGGLE(robot_p->sonars.sonar[Sonars::CENTER].use);
  }
  else if (pfodCmd == "d06")
  {
    TOGGLE(robot_p->sonars.sonar[Sonars::RIGHT].use);
  }
  sendSonarMenu(true);
}

void RemoteControl::sendPerimeterMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Perimeter`1000"));
  }
  Bluetooth.print(F("|e00~Use "));
  sendYesNo(robot_p->perimeters.use);
  Bluetooth.println(F("|e02~Value"));
  Bluetooth.print(robot_p->getPerimeterMag());
  if (robot_p->getPerimeterMag() < 0)
  {
    Bluetooth.print(" (inside)");
  }
  else
  {
    Bluetooth.print(" (outside)");
  }
  sendSlider("e08", F("Timed-out if below smag"),
             robot_p->perimeters.perimeter[Perimeter::LEFT].timedOutIfBelowSmag, "", 1, 2000);
  sendSlider("e14", F("Timeout (s) if not inside"),
             robot_p->perimeters.perimeter[Perimeter::LEFT].timeOutSecIfNotInside, "", 1, 20, 1);
  sendSlider("e04", F("Trigger timeout"),
             robot_p->perimeterTriggerTimeout, "", 1, 2000);
  sendSlider("e05", F("Perimeter out roll time max"),
             robot_p->perimeterOutRollTimeMax, "", 1, 8000);
  sendSlider("e06", F("Perimeter out roll time min"),
             robot_p->perimeterOutRollTimeMin, "", 1, 8000);
  sendSlider("e15", F("Perimeter out reverse time"),
             robot_p->perimeterOutRevTime, "", 1, 8000);
  sendSlider("e16", F("Perimeter tracking roll time"),
             robot_p->perimeterTrackRollTime, "", 1, 8000);
  sendSlider("e17", F("Perimeter tracking reverse time"),
             robot_p->perimeterTrackRevTime, "", 1, 8000);
  sendSlider("e11", F("Transition timeout"),
             robot_p->trackingPerimeterTransitionTimeOut, "", 1, 5000);
  sendSlider("e12", F("Track error timeout"),
             robot_p->trackingErrorTimeOut, "", 1, 10000);
  sendPIDSlider("e07", F("Track"), robot_p->perimeters.perimeter[Perimeter::LEFT].pid, 0.1, 100);
  Bluetooth.print(F("|e09~Use differential signal "));
  sendYesNo(robot_p->perimeters.perimeter[Perimeter::LEFT].useDifferentialPerimeterSignal);
  Bluetooth.print(F("|e10~Swap coil polarity "));
  sendYesNo(robot_p->perimeters.perimeter[Perimeter::LEFT].swapCoilPolarity);
  Bluetooth.print(F("|e13~Block inner wheel  "));
  sendYesNo(robot_p->trackingBlockInnerWheelWhilePerimeterStruggling);
  Bluetooth.println("}");
}

void RemoteControl::processPerimeterMenu(const String pfodCmd)
{
  if (pfodCmd == "e00")
  {
    TOGGLE(robot_p->perimeters.use);
  }
  else if (pfodCmd.startsWith("e04"))
  {
    processSlider(pfodCmd, robot_p->perimeterTriggerTimeout, 1);
  }
  else if (pfodCmd.startsWith("e05"))
  {
    processSlider(pfodCmd, robot_p->perimeterOutRollTimeMax, 1);
  }
  else if (pfodCmd.startsWith("e06"))
  {
    processSlider(pfodCmd, robot_p->perimeterOutRollTimeMin, 1);
  }
  else if (pfodCmd.startsWith("e15"))
  {
    processSlider(pfodCmd, robot_p->perimeterOutRevTime, 1);
  }
  else if (pfodCmd.startsWith("e16"))
  {
    processSlider(pfodCmd, robot_p->perimeterTrackRollTime, 1);
  }
  else if (pfodCmd.startsWith("e17"))
  {
    processSlider(pfodCmd, robot_p->perimeterTrackRevTime, 1);
  }
  else if (pfodCmd.startsWith("e07"))
  {
    processPIDSlider(pfodCmd, "e07", robot_p->perimeters.perimeter[Perimeter::LEFT].pid, 0.1, 100);
  }
  else if (pfodCmd.startsWith("e08"))
  {
    processSlider(pfodCmd, robot_p->perimeters.perimeter[Perimeter::LEFT].timedOutIfBelowSmag, 1);
  }
  else if (pfodCmd.startsWith("e09"))
  {
    TOGGLE(robot_p->perimeters.perimeter[Perimeter::LEFT].useDifferentialPerimeterSignal);
  }
  else if (pfodCmd.startsWith("e10"))
  {
    TOGGLE(robot_p->perimeters.perimeter[Perimeter::LEFT].swapCoilPolarity);
  }
  else if (pfodCmd.startsWith("e11"))
  {
    processSlider(pfodCmd, robot_p->trackingPerimeterTransitionTimeOut, 1);
  }
  else if (pfodCmd.startsWith("e12"))
  {
    processSlider(pfodCmd, robot_p->trackingErrorTimeOut, 1);
  }
  else if (pfodCmd.startsWith("e13"))
  {
    TOGGLE(robot_p->trackingBlockInnerWheelWhilePerimeterStruggling);
  }
  else if (pfodCmd.startsWith("e14"))
  {
    processSlider(pfodCmd, robot_p->perimeters.perimeter[Perimeter::LEFT].timeOutSecIfNotInside, 1);
  }
  sendPerimeterMenu(true);
}

void RemoteControl::sendLawnSensorMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Lawn sensor`1000"));
  }
  Bluetooth.print(F("|f00~Use "));
  sendYesNo(robot_p->lawnSensors.use);
  Bluetooth.print(F("|f01~Counter "));
  Bluetooth.print(lawnSensors_getCounter(&robot_p->lawnSensors));
  Bluetooth.println(F("|f02~Value f, b"));
  Bluetooth.print(lawnSensor_getValue(&robot_p->lawnSensorArray[FRONT]));
  Bluetooth.print(", ");
  Bluetooth.print(lawnSensor_getValue(&robot_p->lawnSensorArray[BACK]));
  Bluetooth.println("}");
}

void RemoteControl::processLawnSensorMenu(const String pfodCmd)
{
  if (pfodCmd == "f00")
  {
    TOGGLE(robot_p->lawnSensors.use);
  }
  sendLawnSensorMenu(true);
}

void RemoteControl::sendRainMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Rain`1000"));
  }
  Bluetooth.print(F("|m00~Use "));
  sendYesNo(robot_p->rainSensor.use);
  Bluetooth.print(F("|m01~Counter "));
  Bluetooth.print(robot_p->rainSensor.getCounter());
  Bluetooth.println(F("|m02~Value"));
  Bluetooth.print(robot_p->rainSensor.isRaining());
  Bluetooth.println("}");
}

void RemoteControl::processRainMenu(const String pfodCmd)
{
  if (pfodCmd == "m00")
  {
    TOGGLE(robot_p->rainSensor.use);
  }
  sendRainMenu(true);
}

void RemoteControl::sendGPSMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.GPS`1000"));
  }
  Bluetooth.print(F("|q00~Use "));
  sendYesNo(robot_p->gpsUse);
  sendSlider("q01", F("Stuck if GPS speed is below"),
             robot_p->stuckIfGpsSpeedBelow, "", 0.1, 3);
  sendSlider("q02", F("GPS speed ignore time"),
             robot_p->gpsSpeedIgnoreTime,
             "", 1, 10000, robot_p->wheels.reverseTime);
  Bluetooth.println("}");
}

void RemoteControl::processGPSMenu(const String pfodCmd)
{
  if (pfodCmd == "q00")
  {
    TOGGLE(robot_p->gpsUse);
  }
  else if (pfodCmd.startsWith("q01"))
  {
    processSlider(pfodCmd, robot_p->stuckIfGpsSpeedBelow, 0.1);
  }
  else if (pfodCmd.startsWith("q02"))
  {
    processSlider(pfodCmd, robot_p->gpsSpeedIgnoreTime, 1);
  }
  sendGPSMenu(true);
}

void RemoteControl::sendImuMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.IMU`1000"));
  }
  Bluetooth.print(F("|g00~Use "));
  sendYesNo(robot_p->imu.use);
  Bluetooth.print(F("|g10~Use accel calib"));
  sendYesNo(robot_p->imu.getUseAccelCalibration());
  Bluetooth.print(F("|g01~Yaw "));
  Bluetooth.print(robot_p->imu.getYawDeg());
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g09~DriveHeading "));
  Bluetooth.print(robot_p->imuDriveHeading / PI * 180);
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g02~Pitch "));
  Bluetooth.print(robot_p->imu.getPitchDeg());
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g03~Roll "));
  Bluetooth.print(robot_p->imu.getRollDeg());
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g04~Correct dir "));
  sendYesNo(robot_p->imu.correctDir);
  sendPIDSlider("g05", F("Dir"), robot_p->imu.pid[Imu::DIR], 0.1, 20);
  sendPIDSlider("g06", F("Roll"), robot_p->imu.pid[Imu::ROLL], 0.1, 30);
  Bluetooth.print(F("|g07~Acc cal next side"));
  Bluetooth.print(F("|g08~Com cal start/stop"));
  Bluetooth.println("}");
}

void RemoteControl::processImuMenu(const String pfodCmd)
{
  if (pfodCmd == "g00")
  {
    TOGGLE(robot_p->imu.use);
  }
  else if (pfodCmd == "g10")
  {
    robot_p->imu.toggleUseAccelCalibration();
  }
  else if (pfodCmd == "g04")
  {
    TOGGLE(robot_p->imu.correctDir);
  }
  else if (pfodCmd.startsWith("g05"))
  {
    processPIDSlider(pfodCmd, "g05", robot_p->imu.pid[Imu::DIR], 0.1, 20);
  }
  else if (pfodCmd.startsWith("g06"))
  {
    processPIDSlider(pfodCmd, "g06", robot_p->imu.pid[Imu::ROLL], 0.1, 30);
  }
  else if (pfodCmd == "g07")
  {
    robot_p->imu.calibrateAccelerometerNextAxis();
  }
  else if (pfodCmd == "g08")
  {
    robot_p->imu.calibrateMagnetometerStartStop();
  }
  sendImuMenu(true);
}

void RemoteControl::sendRemoteMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Remote R/C`1000"));
  }
  Bluetooth.print(F("|h00~Use "));
  sendYesNo(robot_p->remoteUse);
  Bluetooth.println("}");
}

void RemoteControl::processRemoteMenu(const String pfodCmd)
{
  if (pfodCmd == "h00")
  {
    TOGGLE(robot_p->remoteUse);
  }
  sendRemoteMenu(true);
}

void RemoteControl::sendBatteryMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Battery`1000"));
  }
  Bluetooth.print(F("|j00~Battery "));
  Bluetooth.print(robot_p->battery.getVoltage());
  Bluetooth.print(" V");
  Bluetooth.print(F("|j01~Monitor "));
  sendYesNo(robot_p->battery.monitored);
  if (robot_p->developerActive)
  {
    sendSlider("j05", F("Calibrate batFactor "), robot_p->battery.batFactor, "",
               0.01, 1.0);
  }
  //Console.print("batFactor=");
  //Console.println(robot->batFactor);
  sendSlider("j02", F("Go home if below Volt"),
             robot_p->battery.batGoHomeIfBelow, "", 0.1, robot_p->battery.batFull,
             (robot_p->battery.batFull * 0.72)); // for Sony Konion cells 4.2V * 0,72= 3.024V which is pretty safe to use
  sendSlider("j12", F("Switch off if idle minutes"),
             robot_p->battery.batSwitchOffIfIdle, "", 1, 300, 1);
  sendSlider("j03", F("Switch off if below Volt"),
             robot_p->battery.batSwitchOffIfBelow, "", 0.1, robot_p->battery.batFull,
             (robot_p->battery.batFull * 0.72));
  Bluetooth.print(F("|j04~Charge "));
  Bluetooth.print(robot_p->battery.getChargeVoltage());
  Bluetooth.print("V ");
  Bluetooth.print(robot_p->battery.getChargeCurrent());
  Bluetooth.print("A");
  sendSlider("j09", F("Calibrate batChgFactor"),
             robot_p->battery.batChgFactor, "", 0.01, 1.0);
  sendSlider("j06", F("Charge sense zero"),
             robot_p->battery.chgSenseZero, "", 1, 600, 400);
  sendSlider("j08", F("Charge factor"), robot_p->battery.chgFactor, "", 0.01, 80);
  sendSlider("j10", F("charging starts if Voltage is below"),
             robot_p->battery.startChargingIfBelow, "", 0.1, robot_p->battery.batFull);
  sendSlider("j11", F("Battery is fully charged if current is below"),
             robot_p->battery.batFullCurrent, "", 0.1, robot_p->battery.batChargingCurrentMax);
  Bluetooth.println("}");
}

void RemoteControl::processBatteryMenu(const String pfodCmd)
{
  if (pfodCmd == "j01")
  {
    TOGGLE(robot_p->battery.monitored);
  }
  else if (pfodCmd.startsWith("j02"))
  {
    processSlider(pfodCmd, robot_p->battery.batGoHomeIfBelow, 0.1);
    //Console.print("gohomeifbelow=");
    //Console.println(robot->batGoHomeIfBelow);
  }
  else if (pfodCmd.startsWith("j03"))
  {
    processSlider(pfodCmd, robot_p->battery.batSwitchOffIfBelow, 0.1);
  }
  else if (pfodCmd.startsWith("j05"))
  {
    processSlider(pfodCmd, robot_p->battery.batFactor, 0.01);
  }
  else if (pfodCmd.startsWith("j06"))
  {
    processSlider(pfodCmd, robot_p->battery.chgSenseZero, 1);
  }
  else if (pfodCmd.startsWith("j08"))
  {
    processSlider(pfodCmd, robot_p->battery.chgFactor, 0.01);
  }
  else if (pfodCmd.startsWith("j09"))
  {
    processSlider(pfodCmd, robot_p->battery.batChgFactor, 0.01);
  }
  else if (pfodCmd.startsWith("j10"))
  {
    processSlider(pfodCmd, robot_p->battery.startChargingIfBelow, 0.1);
  }
  else if (pfodCmd.startsWith("j11"))
  {
    processSlider(pfodCmd, robot_p->battery.batFullCurrent, 0.1);
  }
  else if (pfodCmd.startsWith("j12"))
  {
    processSlider(pfodCmd, robot_p->battery.batSwitchOffIfIdle, 1);
  }
  sendBatteryMenu(true);
}

void RemoteControl::sendStationMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Station`1000"));
  }
  sendSlider("k00", F("Reverse time"), robot_p->stationRevTime, "", 1, 8000);
  sendSlider("k01", F("Roll time"), robot_p->stationRollTime, "", 1, 8000);
  sendSlider("k02", F("Forw time"), robot_p->stationForwTime, "", 1, 8000);
  sendSlider("k03", F("Station reverse check time"),
             robot_p->stationCheckTime, "", 1, 8000);
  Bluetooth.println("}");
}

void RemoteControl::processStationMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("k00"))
  {
    processSlider(pfodCmd, robot_p->stationRevTime, 1);
  }
  else if (pfodCmd.startsWith("k01"))
  {
    processSlider(pfodCmd, robot_p->stationRollTime, 1);
  }
  else if (pfodCmd.startsWith("k02"))
  {
    processSlider(pfodCmd, robot_p->stationForwTime, 1);
  }
  else if (pfodCmd.startsWith("k03"))
  {
    processSlider(pfodCmd, robot_p->stationCheckTime, 1);
  }
  sendStationMenu(true);
}

void RemoteControl::sendOdometerMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Odometer2D`1000"));
  }
  Bluetooth.print(F("|l00~Use "));
  sendYesNo(robot_p->odometer.use);
  Bluetooth.print(F("|l01~Value l, r "));
  Bluetooth.print(robot_p->odometer.encoder.left_p->getCounter());
  Bluetooth.print(", ");
  Bluetooth.println(robot_p->odometer.encoder.right_p->getCounter());
  Bluetooth.println(F("|l03~RPM Motor l, r "));
  Bluetooth.print(robot_p->odometer.encoder.left_p->getWheelRpmCurr());
  Bluetooth.print(", ");
  Bluetooth.println(robot_p->odometer.encoder.right_p->getWheelRpmCurr());
  sendSlider("l04", F("Ticks per one full revolution"),
             robot_p->odometer.ticksPerRevolution, "", 1, 2000);
  sendSlider("l01", F("Ticks per cm"),
             robot_p->odometer.ticksPerCm, "", 0.1, 30);
  sendSlider("l02", F("Wheel base cm"),
             robot_p->odometer.wheelBaseCm, "", 0.1, 50);
  Bluetooth.print(F("|l08~Use two-way encoder "));
  sendYesNo(robot_p->odometer.encoder.left_p->twoWay);  // TODO: Only checking sensing left here
  Bluetooth.print(F("|l05~Swap left direction "));
  sendYesNo(robot_p->odometer.encoder.left_p->swapDir);
  Bluetooth.print(F("|l06~Swap right direction "));
  sendYesNo(robot_p->odometer.encoder.right_p->swapDir);
  Bluetooth.println("}");
}

void RemoteControl::processOdometerMenu(const String pfodCmd)
{
  if (pfodCmd == "l00")
  {
    TOGGLE(robot_p->odometer.use);
  }
  else if (pfodCmd.startsWith("l01"))
  {
    processSlider(pfodCmd, robot_p->odometer.ticksPerCm, 0.1);
  }
  else if (pfodCmd.startsWith("l02"))
  {
    processSlider(pfodCmd, robot_p->odometer.wheelBaseCm, 0.1);
  }
  else if (pfodCmd.startsWith("l04"))
  {
    processSlider(pfodCmd, robot_p->odometer.ticksPerRevolution, 1);
  }
  else if (pfodCmd.startsWith("l05"))
  {
    TOGGLE(robot_p->odometer.encoder.left_p->swapDir);
  }
  else if (pfodCmd.startsWith("l06"))
  {
    TOGGLE(robot_p->odometer.encoder.right_p->swapDir);
  }
  else if (pfodCmd.startsWith("l08"))
  {
    bool twoWay = robot_p->odometer.encoder.left_p->twoWay;
    robot_p->odometer.encoder.left_p->twoWay = !twoWay;
    robot_p->odometer.encoder.right_p->twoWay = !twoWay;
  }
  sendOdometerMenu(true);
}

void RemoteControl::sendDateTimeMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Date/time"));
  }
  Bluetooth.print("|t00~");
  Bluetooth.print(date2str(robot_p->datetime.date));
  Bluetooth.print(", ");
  Bluetooth.print(time2str(robot_p->datetime.time));
  sendSlider("t01", dayOfWeek[robot_p->datetime.date.dayOfWeek],
             robot_p->datetime.date.dayOfWeek, "", 1, 6, 0);
  sendSlider("t02", "Day ", robot_p->datetime.date.day, "", 1, 31, 1);
  sendSlider("t03", "Month ", robot_p->datetime.date.month, "", 1, 12, 1);
  sendSlider("t04", "Year ", robot_p->datetime.date.year, "", 1, 2020, 2013);
  sendSlider("t05", "Hour ", robot_p->datetime.time.hour, "", 1, 23, 0);
  sendSlider("t06", "Minute ", robot_p->datetime.time.minute, "", 1, 59, 0);
  Bluetooth.println("}");
}

void RemoteControl::processDateTimeMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("t01"))
  {
    processSlider(pfodCmd, robot_p->datetime.date.dayOfWeek, 1);
  }
  else if (pfodCmd.startsWith("t02"))
  {
    processSlider(pfodCmd, robot_p->datetime.date.day, 1);
  }
  else if (pfodCmd.startsWith("t03"))
  {
    processSlider(pfodCmd, robot_p->datetime.date.month, 1);
  }
  else if (pfodCmd.startsWith("t04"))
  {
    processSlider(pfodCmd, robot_p->datetime.date.year, 1);
  }
  else if (pfodCmd.startsWith("t05"))
  {
    processSlider(pfodCmd, robot_p->datetime.time.hour, 1);
  }
  else if (pfodCmd.startsWith("t06"))
  {
    processSlider(pfodCmd, robot_p->datetime.time.minute, 1);
  }
  sendDateTimeMenu(true);
  Console.print(F("setting RTC datetime: "));
  Console.println(date2str(robot_p->datetime.date));
  robot_p->setActuator(Robot::ACT_RTC, 0);
}

void RemoteControl::sendTimerDetailMenu(const int timerIdx,
                                        const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Details"));
  }
  Bluetooth.print("|p0");
  Bluetooth.print(timerIdx);
  Bluetooth.print("~Use ");
  sendYesNo(robot_p->timer[timerIdx].active);
  String sidx = String(timerIdx);
  sendSlider("p1" + sidx, F("Start hour "),
             robot_p->timer[timerIdx].startTime.hour, "", 1, 23, 0);
  sendSlider("p2" + sidx, F("Start minute "),
             robot_p->timer[timerIdx].startTime.minute, "", 1, 59, 0);
  sendSlider("p3" + sidx, F("Stop hour "),
             robot_p->timer[timerIdx].stopTime.hour, "", 1, 23, 0);
  sendSlider("p4" + sidx, F("Stop minute "),
             robot_p->timer[timerIdx].stopTime.minute, "", 1, 59, 0);
  for (int i = 0; i < 7; i++)
  {
    Bluetooth.print("|p5");
    Bluetooth.print(timerIdx);
    Bluetooth.print(i);
    Bluetooth.print("~");
    if ((robot_p->timer[timerIdx].daysOfWeek >> i) & 1)
    {
      Bluetooth.print("(X)  ");
    }
    else
    {
      Bluetooth.print("(  )  ");
    }
    Bluetooth.print(dayOfWeek[i]);
  }
  Bluetooth.print("|p9");
  Bluetooth.print(timerIdx);
  Bluetooth.print(F("~Set to current time"));
  Bluetooth.println("}");
}

void RemoteControl::processTimerDetailMenu(const String pfodCmd)
{
  timehm_t time;
  boolean checkStop = false;
  boolean checkStart = false;
  int startmin;
  int stopmin;
  int timerIdx = pfodCmd[2] - '0';
  if (pfodCmd.startsWith("p0"))
  {
    TOGGLE(robot_p->timer[timerIdx].active);
  }
  else if (pfodCmd.startsWith("p1"))
  {
    processSlider(pfodCmd, robot_p->timer[timerIdx].startTime.hour, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p2"))
  {
    processSlider(pfodCmd, robot_p->timer[timerIdx].startTime.minute, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p3"))
  {
    processSlider(pfodCmd, robot_p->timer[timerIdx].stopTime.hour, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p4"))
  {
    processSlider(pfodCmd, robot_p->timer[timerIdx].stopTime.minute, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p9"))
  {
    robot_p->timer[timerIdx].startTime = robot_p->datetime.time;
    checkStop = true;
    robot_p->timer[timerIdx].daysOfWeek =
        (1 << robot_p->datetime.date.dayOfWeek);
  }
  else if (pfodCmd.startsWith("p5"))
  {
    int day = pfodCmd[3] - '0';
    robot_p->timer[timerIdx].daysOfWeek =
        robot_p->timer[timerIdx].daysOfWeek ^ (1 << day);
  }
  if (checkStop)
  {
    // adjust start time
    startmin = min(1434, time2minutes(robot_p->timer[timerIdx].startTime));
    minutes2time(startmin, time);
    robot_p->timer[timerIdx].startTime = time;
    // check stop time
    stopmin = time2minutes(robot_p->timer[timerIdx].stopTime);
    stopmin = max(stopmin, startmin + 5);
    minutes2time(stopmin, time);
    robot_p->timer[timerIdx].stopTime = time;
  }
  else if (checkStart)
  {
    // adjust stop time
    stopmin = max(5, time2minutes(robot_p->timer[timerIdx].stopTime));
    minutes2time(stopmin, time);
    robot_p->timer[timerIdx].stopTime = time;
    // check start time
    startmin = time2minutes(robot_p->timer[timerIdx].startTime);
    startmin = min(startmin, stopmin - 5);
    minutes2time(startmin, time);
    robot_p->timer[timerIdx].startTime = time;
  }
  sendTimerDetailMenu(timerIdx, true);
}

void RemoteControl::sendTimerMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Timer"));
  }
  Bluetooth.print(F("|i99~Use "));
  sendYesNo(robot_p->timerUse);
  for (int i = 0; i < MAX_TIMERS; i++)
  {
    Bluetooth.print("|i0");
    Bluetooth.print(i);
    Bluetooth.print("~");
    sendTimer(robot_p->timer[i]);
  }
  Bluetooth.println("}");
}

void RemoteControl::processTimerMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("i0"))
  {
    int timerIdx = pfodCmd[2] - '0';
    sendTimerDetailMenu(timerIdx, false);
  }
  else
  {
    if (pfodCmd.startsWith("i99"))
    {
      TOGGLE(robot_p->timerUse);
    }
    sendTimerMenu(true);
  }
}

void RemoteControl::sendFactorySettingsMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.println(F("{.Factory settings"));
  }
  Bluetooth.println(F("|x0~Set factory settings (requires reboot)}"));
}

void RemoteControl::processFactorySettingsMenu(const String pfodCmd)
{
  if (pfodCmd == "x0")
  {
    robot_p->deleteUserSettings();
  }
  sendFactorySettingsMenu(true);
}

void RemoteControl::sendInfoMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Info` 1000"));
  }
  Bluetooth.print(F("|v00~Ardumower "));
  Bluetooth.print(VERSION);
  Bluetooth.print(F("|v01~Developer "));
  sendYesNo(robot_p->developerActive);
  Bluetooth.print(F("|v02~Mowing time trip (min) "));
  Bluetooth.print(robot_p->stats.mowTimeMinutesTrip);
  Bluetooth.print(F("|v03~Mowing time total (hrs) "));
  Bluetooth.print(robot_p->getStatsMowTimeHoursTotal());
  Bluetooth.print(F("|v05~Battery charging cycles "));
  Bluetooth.print(robot_p->stats.batteryChargingCounterTotal);
  Bluetooth.print(F("|v06~Battery recharged capacity trip (mAh)"));
  Bluetooth.print(robot_p->stats.batteryChargingCapacityTrip);
  Bluetooth.print(F("|v07~Battery recharged capacity total (Ah)"));
  Bluetooth.print(robot_p->stats.batteryChargingCapacityTotal / 1000);
  Bluetooth.print(F("|v08~Battery recharged capacity average (mAh)"));
  Bluetooth.print(robot_p->stats.batteryChargingCapacityAverage);
  //Bluetooth.print("|d01~Perimeter v");
  //Bluetooth.print(verToString(readPerimeterVer()));
  //Bluetooth.print("|d02~IMU v");
  //Bluetooth.print(verToString(readIMUver()));
  //Bluetooth.print("|d02~Stepper v");
  //Bluetooth.print(verToString(readStepperVer()));
  Bluetooth.println("}");
}

void RemoteControl::processInfoMenu(const String pfodCmd)
{
  if (pfodCmd == "v01")
  {
    TOGGLE(robot_p->developerActive);
  }
  robot_p->saveUserSettings();

  sendInfoMenu(true);
}

void RemoteControl::sendCommandMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Commands`5000"));
  }
  Bluetooth.print(F("|ro~OFF|ra~Auto mode|rc~RC mode|"));
  Bluetooth.print(F("rm~Mowing is "));
  sendOnOff(robot_p->cutter.isEnabled());
  Bluetooth.print(F("|rp~Pattern is "));
  Bluetooth.print(robot_p->mowPatternName());
  Bluetooth.print(F("|rh~Home|rk~Track|rs~State is "));
  Bluetooth.print(robot_p->stateMachine.getCurrentStateName());
  Bluetooth.print(F("|rr~Auto rotate is "));
  Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getPwmCur());
  Bluetooth.print(F("|r1~User switch 1 is "));
  sendOnOff(robot_p->userSwitch1);
  Bluetooth.print(F("|r2~User switch 2 is "));
  sendOnOff(robot_p->userSwitch2);
  Bluetooth.print(F("|r3~User switch 3 is "));
  sendOnOff(robot_p->userSwitch3);
  Bluetooth.println("}");
}

void RemoteControl::processCommandMenu(const String pfodCmd)
{
  if (pfodCmd == "ro")
  {
    // cmd: off
    robot_p->setNextState(StateMachine::STATE_OFF, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rh")
  {
    // cmd: home
    robot_p->setNextState(StateMachine::STATE_PERI_FIND, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rr")
  {
    robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet += 10;
    robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
        -robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet;
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rk")
  {
    // cmd: track perimeter
    robot_p->setNextState(StateMachine::STATE_PERI_TRACK, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "ra")
  {
    // cmd: start auto mowing
    robot_p->cutter.enable();
    robot_p->setNextState(StateMachine::STATE_FORWARD, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rc")
  {
    // cmd: start remote control (RC)
    robot_p->cutter.enable();
    robot_p->cutter.motor.regulate = false;
    robot_p->setNextState(StateMachine::STATE_REMOTE, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rm")
  {
    // cmd: mower motor on/off
    if (robot_p->stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
        robot_p->stateMachine.isCurrentState(StateMachine::STATE_MANUAL))
    {
      robot_p->cutter.setEnableOverriden(false);
    }
    else
    {
      robot_p->cutter.toggleEnableOverriden();
    }
    robot_p->cutter.toggleEnabled();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rs")
  {
    // cmd: state
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rp")
  {
    // cmd: pattern
    robot_p->mowPatternCurr = (robot_p->mowPatternCurr + 1) % 3;
    robot_p->setNextState(StateMachine::STATE_OFF, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r1")
  {
    TOGGLE(robot_p->userSwitch1);
    robot_p->setUserSwitches();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r2")
  {
    TOGGLE(robot_p->userSwitch2);
    robot_p->setUserSwitches();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r3")
  {
    TOGGLE(robot_p->userSwitch3);
    robot_p->setUserSwitches();
    sendCommandMenu(true);
  }
}

void RemoteControl::sendManualMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.println(F("{^Manual navigation`1000"));
  }
  Bluetooth.print(F("|nl~Left|nr~Right|nf~Forward"));
  if ((robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet < 5 &&
       robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet > -5) &&
      (robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet < 5 &&
       robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet > -5))
  {
    Bluetooth.print(F("|nb~Reverse"));
  }
  else
  {
    Bluetooth.print(F("|ns~Stop"));
  }
  Bluetooth.print(F("|nm~Mow is "));
  sendOnOff(robot_p->cutter.isEnabled());
  Bluetooth.println("}");
}

void RemoteControl::sendCompassMenu(const boolean update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.println(F("{^Compass`1000"));
  }
  Bluetooth.print(F("|cw~West|ce~East|cn~North "));
  Bluetooth.print(robot_p->imu.getYawDeg());
  Bluetooth.println(F("|cs~South|cm~Mow}"));
}

void RemoteControl::processCompassMenu(const String pfodCmd)
{
  if (pfodCmd == "cm")
  {
    robot_p->cutter.toggleEnabled();
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cn")
  {
    robot_p->imuRollHeading = 0;
    robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cs")
  {
    robot_p->imuRollHeading = PI;
    robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cw")
  {
    robot_p->imuRollHeading = -PI / 2;
    robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
  else if (pfodCmd == "ce")
  {
    robot_p->imuRollHeading = PI / 2;
    robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
}

void RemoteControl::processManualMenu(const String pfodCmd)
{
  if (pfodCmd == "nl")
  {
    // manual: left
    robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    int sign = 1;
    if (robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet < 0)
    {
      sign = -1;
    }
    if (sign * robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet >=
        sign * robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet)
    {
      robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
          sign * robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax / 2;
    }
    else
    {
      robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet /= 2;
    }
    robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
        sign * robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;

    sendManualMenu(true);
  }
  else if (pfodCmd == "nr")
  {
    // manual: right
    robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    int sign = 1;
    if (robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet < 0)
    {
      sign = -1;
    }
    if (sign * robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet >=
        sign * robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet)
    {
      robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
          sign * robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax / 2;
    }
    else
    {
      robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet /= 2;
    }
    robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
        sign * robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;

    sendManualMenu(true);
  }
  else if (pfodCmd == "nf")
  {
    // manual: forward
    robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
        robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;
    robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
        robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nb")
  {
    // manual: reverse
    robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
        -robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;
    robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
        -robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nm")
  {
    // manual: mower ON/OFF
    robot_p->cutter.toggleEnabled();
    sendManualMenu(true);
  }
  else if (pfodCmd == "ns")
  {
    // manual: stop
    //setNextState(STATE_OFF, 0);
    robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
        robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet = 0;
    sendManualMenu(true);
  }
}

void RemoteControl::processSettingsMenu(const String pfodCmd)
{
  if (pfodCmd == "s1")
  {
    sendMotorMenu(false);
  }
  else if (pfodCmd == "s2")
  {
    sendMowMenu(false);
  }
  else if (pfodCmd == "s3")
  {
    sendBumperMenu(false);
  }
  else if (pfodCmd == "s4")
  {
    sendSonarMenu(false);
  }
  else if (pfodCmd == "s5")
  {
    sendPerimeterMenu(false);
  }
  else if (pfodCmd == "s6")
  {
    sendLawnSensorMenu(false);
  }
  else if (pfodCmd == "s7")
  {
    sendImuMenu(false);
  }
  else if (pfodCmd == "s8")
  {
    sendRemoteMenu(false);
  }
  else if (pfodCmd == "s9")
  {
    sendBatteryMenu(false);
  }
  else if (pfodCmd == "s10")
  {
    sendStationMenu(false);
  }
  else if (pfodCmd == "s11")
  {
    sendOdometerMenu(false);
  }
  else if (pfodCmd == "s12")
  {
    sendDateTimeMenu(false);
  }
  else if (pfodCmd == "s13")
  {
    sendRainMenu(false);
  }
  else if (pfodCmd == "s15")
  {
    sendDropMenu(false);
  }
  else if (pfodCmd == "s14")
  {
    sendGPSMenu(false);
  }
  else if (pfodCmd == "sx")
  {
    sendFactorySettingsMenu(false);
  }
  else if (pfodCmd == "sz")
  {
    robot_p->saveUserSettings();
    sendSettingsMenu(true);
  }
  else
  {
    sendSettingsMenu(true);
  }
}

// process pfodState
void RemoteControl::run()
{
  unsigned long curMillis = millis();
  float elapsedSeconds = float(curMillis) / 1000.0f;

  if (pfodState == PFOD_LOG_SENSORS)
  {
    //robot->printInfo(Bluetooth);
    //Bluetooth.println("test");
    Bluetooth.print(elapsedSeconds);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getPowerMeas());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getPowerMeas());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->cutter.motor.getPowerMeas());
    Bluetooth.print(",");
    for (uint8_t i = 0; i < Sonars::END; i++)
    {
      Bluetooth.print(robot_p->sonars.sonar[i].getDistance_us());
      Bluetooth.print(",");
    }
    Bluetooth.print(robot_p->perimeters.perimeter[Perimeter::LEFT].isInside());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->getPerimeterMag());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->odometer.encoder.left_p->getCounter());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->odometer.encoder.right_p->getCounter());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.getYawDeg());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.getPitchDeg());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.getRollDeg());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.gyro.g.x / PI * 180);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.gyro.g.y / PI * 180);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.gyro.g.z / PI * 180);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.accel.x);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.accel.y);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.accel.z);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.mag.x);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.mag.y);
    Bluetooth.print(",");
    Bluetooth.print(robot_p->imu.mag.z);
    Bluetooth.print(",");
    float lat, lon;
    unsigned long age;
    robot_p->gps.f_get_position(&lat, &lon, &age);
    Bluetooth.print(robot_p->gps.hdop());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->gps.satellites());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->gps.f_speed_kmph());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->gps.f_course());
    Bluetooth.print(",");
    Bluetooth.print(robot_p->gps.f_altitude());
    Bluetooth.print(",");
    Bluetooth.print(lat);
    Bluetooth.print(",");
    Bluetooth.print(lon);
    Bluetooth.println();
  }
  else if (pfodState == PFOD_PLOT_BAT)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 60000;
      Bluetooth.print(curMillis / 60000);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->battery.getVoltage());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->battery.getChargeVoltage());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->battery.getChargeCurrent());
      Bluetooth.print(",");
      Bluetooth.println(robot_p->battery.getCapacity());
    }
  }
  else if (pfodState == PFOD_PLOT_ODO2D)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 500;
      Bluetooth.print(robot_p->odometer.getX());
      Bluetooth.print(",");
      Bluetooth.println(robot_p->odometer.getY());
    }
  }
  else if (pfodState == PFOD_PLOT_IMU)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 200;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.getYawDeg());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.getPitchDeg());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.getRollDeg());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.gyro.g.x / PI * 180);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.gyro.g.y / PI * 180);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.gyro.g.z / PI * 180);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.accel.x);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.accel.y);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.accel.z);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.mag.x);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->imu.mag.y);
      Bluetooth.print(",");
      Bluetooth.println(robot_p->imu.mag.z);
    }
  }
  else if (pfodState == PFOD_PLOT_SENSOR_COUNTERS)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 200;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->stateMachine.getCurrentState());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getOverloadCounter());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getOverloadCounter());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->cutter.motor.getOverloadCounter());
      Bluetooth.print(",");
      Bluetooth.print(bumper_getCounter(&robot_p->bumperArray[LEFT]));
      Bluetooth.print(",");
      Bluetooth.print(bumper_getCounter(&robot_p->bumperArray[RIGHT]));
      Bluetooth.print(",");
      Bluetooth.print(robot_p->sonars.getDistanceCounter());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->getPerimeterCounter());
      Bluetooth.print(",");
      Bluetooth.print(lawnSensors_getCounter(&robot_p->lawnSensors));
      Bluetooth.print(",");
      Bluetooth.print(robot_p->rainSensor.getCounter());
      Bluetooth.print(",");
      Bluetooth.print(dropSensor_getCounter(&robot_p->dropSensorArray[LEFT]));
      Bluetooth.print(",");
      Bluetooth.println(dropSensor_getCounter(&robot_p->dropSensorArray[RIGHT]));
    }
  }
  else if (pfodState == PFOD_PLOT_SENSORS)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 200;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->stateMachine.getCurrentState());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getPowerMeas());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getPowerMeas());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->cutter.motor.getPowerMeas());
      Bluetooth.print(",");
      for (uint8_t i = 0; i < Sonars::END; i++)
      {
        Bluetooth.print(robot_p->sonars.sonar[i].getDistance_cm());
        Bluetooth.print(",");
      }
      Bluetooth.print(robot_p->perimeters.perimeter[Perimeter::LEFT].isInside());
      Bluetooth.print(",");
      Bluetooth.print(lawnSensors_isDetected(&robot_p->lawnSensors));
      Bluetooth.print(",");
      Bluetooth.print(robot_p->rainSensor.isRaining());
      Bluetooth.print(",");
      Bluetooth.print(dropSensor_isDetected(&robot_p->dropSensorArray[LEFT]));
      Bluetooth.print(",");
      Bluetooth.println(dropSensor_isDetected(&robot_p->dropSensorArray[RIGHT]));
    }
  }
  else if (pfodState == PFOD_PLOT_PERIMETER)
  {
    if (curMillis >= nextPlotTime)
    {
      if (perimeterCaptureIdx >= 32 * 3)
      {
        if (ADCMan.isCaptureComplete(A2))  //FIXME: Use define PIN_PERIMETER_LEFT
        {
          const int8_t* samples_p = ADCMan.getCapture(A2);  //FIXME: Use define PIN_PERIMETER_LEFT
          memcpy(perimeterCapture, samples_p, 32);
          perimeterCaptureIdx = 0;
        }
      }
      if (perimeterCaptureIdx < 32 * 3)
      {
        nextPlotTime = curMillis + 200;
        Bluetooth.print(perimeterCapture[perimeterCaptureIdx / 3]);
        Bluetooth.print(",");
        Bluetooth.print(robot_p->getPerimeterMag());
        Bluetooth.print(",");
        Bluetooth.print(robot_p->perimeters.perimeter[Perimeter::LEFT].getSmoothMagnitude());
        Bluetooth.print(",");
        Bluetooth.print(robot_p->perimeters.perimeter[Perimeter::LEFT].isInside());
        Bluetooth.print(",");
        Bluetooth.print(robot_p->getPerimeterCounter());
        Bluetooth.print(",");
        Bluetooth.print(!robot_p->perimeters.perimeter[Perimeter::LEFT].signalTimedOut());
        Bluetooth.print(",");
        Bluetooth.println(robot_p->perimeters.perimeter[Perimeter::LEFT].getFilterQuality());
        perimeterCaptureIdx++;
      }
    }
  }
  else if (pfodState == PFOD_PLOT_GPS)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 200;
      float lat, lon;
      unsigned long age;
      robot_p->gps.f_get_position(&lat, &lon, &age);
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->gps.hdop());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->gps.satellites());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->gps.f_speed_kmph());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->gps.f_course());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->gps.f_altitude());
      Bluetooth.print(",");
      Bluetooth.print(lat);
      Bluetooth.print(",");
      Bluetooth.println(lon);
    }
  }
  else if (pfodState == PFOD_PLOT_GPS2D)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 500;
      Bluetooth.print(robot_p->getGpsX());
      Bluetooth.print(",");
      Bluetooth.println(robot_p->getGpsY());
    }
  }
  else if (pfodState == PFOD_PLOT_MOTOR)
  {
    if (curMillis >= nextPlotTime)
    {
      nextPlotTime = curMillis + 50;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(robot_p->odometer.encoder.left_p->getWheelRpmCurr());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->odometer.encoder.right_p->getWheelRpmCurr());
      Bluetooth.print(",");
      //      Bluetooth.print(robot->motorLeftSpeedRpmSet);
      Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.pid.getSetpoint());
      Bluetooth.print(",");
      //      Bluetooth.print(robot->motorRightSpeedRpmSet);
      Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.pid.getSetpoint());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.getPwmCur());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::RIGHT].motor.getPwmCur());
      Bluetooth.print(",");
      Bluetooth.print(robot_p->wheels.wheel[Wheel::LEFT].motor.pid.getErrorOld());
      Bluetooth.print(",");
      Bluetooth.println(robot_p->wheels.wheel[Wheel::RIGHT].motor.pid.getErrorOld());
    }
  }
}

// process serial input from pfod App
bool RemoteControl::readSerial()
{
  bool res = false;
  while (Bluetooth.available() > 0)
  {
    res = true;
    if (Bluetooth.available() > 0)
    {
      char ch = Bluetooth.read();
      //Console.print("pfod ch=");
      //Console.println(ch);
      if (ch == '}')
      {
        pfodCmdComplete = true;
      }
      else if (ch == '{')
      {
        pfodCmd = "";
      }
      else
      {
        pfodCmd += ch;
      }
    }
    if (pfodCmdComplete)
    {
      parsePfodCmd();
      pfodCmd = "";
      pfodCmdComplete = false;
    }
  }
  return res;
}

void RemoteControl::parsePfodCmd()
{
  Console.print("pfod cmd=");
  Console.println(pfodCmd);
  pfodState = PFOD_MENU;
  if (pfodCmd == ".")
  {
    sendMainMenu(false);
  }
  else if (pfodCmd == "m1")
  {
    // log raw sensors
    Bluetooth.println(F("{=Log sensors}"));
    Bluetooth.println(F("time,"
                        "leftsen,rightsen,mowsen,"
                        "sonleft,soncenter,sonright,"
                        "perinside,permag,"
                        "odoleft,odoright,"
                        "yaw,pitch,roll,"
                        "gyrox,gyroy,gyroz,"
                        "accx,accy,accz,"
                        "comx,comy,comz,"
                        "hdop,sats,gspeed,gcourse,galt,lat,lon"));
    pfodState = PFOD_LOG_SENSORS;
  }
  else if (pfodCmd == "y1")
  {
    // plot battery
    Bluetooth.println(F("{=battery|time min`0|battery V`1|"
                        "charge V`1|charge A`2|capacity Ah`3}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_BAT;
  }
  else if (pfodCmd == "y2")
  {
    // plot odometer 2d
    Bluetooth.println(F("{=odometer2d|position`0~~~x|`~~~y}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_ODO2D;
  }
  else if (pfodCmd == "y3")
  {
    // plot IMU
    Bluetooth.println(F("{=IMU`60|time s`0|yaw`1~180~-180|pitch`1|roll`1"
                        "|gyroX`2~90~-90|gyroY`2|gyroZ`2"
                        "|accX`3~2~-2|accY`3|accZ`3"
                        "|comX`4~2~-2|comY`4|comZ`4}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_IMU;
  }
  else if (pfodCmd == "y5")
  {
    // plot sensor counters
    Bluetooth.println(F("{=Sensor counters`300"
                        "|time s`0|state`1"
                        "|motL`2|motR`3|motM`4"
                        "|bumL`5|bumR`6"
                        "|son`7|peri`8|lawn`9|rain`10"
                        "|dropL`11|dropR`12}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_SENSOR_COUNTERS;
  }
  else if (pfodCmd == "y6")
  {
    // plot perimeter spectrum
    /*Bluetooth.print(F("{=Perimeter spectrum`"));
     Bluetooth.print(Perimeter.getFilterBinCount());
     Bluetooth.print(F("|freq (Hz)`0|magnitude`0~60~-1|selected band`0~60~-1}"));*/
    Bluetooth.println(F("{=Perimeter`128|sig`1|mag`2|smag`3"
                        "|in`4|cnt`5|on`6|qty`7}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_PERIMETER;
  }
  else if (pfodCmd == "y7")
  {
    // plot sensor values
    Bluetooth.println(F("{=Sensors`300"
                        "|time s`0|state`1"
                        "|motL`2|motR`3|motM`4"
                        "|sonL`5|sonC`6|sonR`7"
                        "|peri`8|lawn`9|rain`10"
                        "|dropL`11|dropR`12}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_SENSORS;
  }
  else if (pfodCmd == "y8")
  {
    // plot GPS
    Bluetooth.println(F("{=GPS`300"
                        "|time s`0|hdop`1|sat`2|spd`3|course`4|alt`5"
                        "|lat`6|lon`7}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_GPS;
  }
  else if (pfodCmd == "y10")
  {
    // plot GPS 2d
    Bluetooth.println(F("{=gps2d|position`0~~~x|`~~~y}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_GPS2D;
  }
  else if (pfodCmd == "c1")
  {
    // ADC calibration
    ADCMan.calibrate();
    robot_p->beep(2, false);
  }
  else if (pfodCmd == "y11")
  {
    // motor control
    Bluetooth.println(F("{=Motor control`300"
                        "|time s`0"
                        "|lrpm_curr`1|rrpm_curr`2"
                        "|lrpm_set`3|rrpm_set`4"
                        "|lpwm`5|rpwm`6"
                        "|lerr`7|rerr`8}"));
    nextPlotTime = 0;
    pfodState = PFOD_PLOT_MOTOR;
  }
  else if (pfodCmd == "yp")
  {
    sendPlotMenu(false);
  }
  else if (pfodCmd == "y4")
  {
    sendErrorMenu(false);
  }
  else if (pfodCmd == "y9")
  {
    sendADCMenu(false);
  }
  else if (pfodCmd == "n")
  {
    sendManualMenu(false);
  }
  else if (pfodCmd == "s")
  {
    sendSettingsMenu(false);
  }
  else if (pfodCmd == "r")
  {
    sendCommandMenu(false);
  }
  else if (pfodCmd == "c")
  {
    sendCompassMenu(false);
  }
  else if (pfodCmd == "t")
  {
    sendDateTimeMenu(false);
  }
  else if (pfodCmd == "i")
  {
    sendTimerMenu(false);
  }
  else if (pfodCmd == "in")
  {
    sendInfoMenu(false);
  }
  else if (pfodCmd.startsWith("s"))
  {
    processSettingsMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("r"))
  {
    processCommandMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("c"))
  {
    processCompassMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("n"))
  {
    processManualMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("a"))
  {
    processMotorMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("o"))
  {
    processMowMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("b"))
  {
    processBumperMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("d"))
  {
    processSonarMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("e"))
  {
    processPerimeterMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("f"))
  {
    processLawnSensorMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("g"))
  {
    processImuMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("h"))
  {
    processRemoteMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("j"))
  {
    processBatteryMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("k"))
  {
    processStationMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("l"))
  {
    processOdometerMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("m"))
  {
    processRainMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("q"))
  {
    processGPSMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("t"))
  {
    processDateTimeMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("i"))
  {
    processTimerMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("p"))
  {
    processTimerDetailMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("x"))
  {
    processFactorySettingsMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("u"))
  {
    processDropMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("v"))
  {
    processInfoMenu(pfodCmd);
  }
  else if (pfodCmd.startsWith("z"))
  {
    processErrorMenu(pfodCmd);
  }
  else
  {
    // no match
    Bluetooth.println("{}");
  }
  Bluetooth.flush();
}
