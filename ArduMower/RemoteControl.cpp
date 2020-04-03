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
  m_pfodCmdComplete = false;
  m_pfodCmd = "";
  m_pfodState = PFOD_OFF;
  m_testmode = 0;
  m_nextPlotTime = 0;
  m_perimeterCaptureIdx = 0;
  m_robot_p = nullptr;
}

void RemoteControl::setRobot(Robot* robot_p)
{
  m_robot_p = robot_p;
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
  Bluetooth.print(value ? F("YES") : F("NO"));
}

void RemoteControl::sendOnOff(const bool value)
{
  Bluetooth.print(value ? F("ON") : F("OFF"));
}

void RemoteControl::sendTimer(const ttimer_t timer)
{
  Bluetooth.print(timer.active ? F("(X)  ") : F("(   )  "));
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
                                  Pid& pid, const double scale,
                                  const float maxvalue)
{
  const Pid_settingsT* pidSettings_p = pid.getSettings();
  sendSlider(cmd + "p", title + " P", pidSettings_p->Kp, "", scale, maxvalue);
  sendSlider(cmd + "i", title + " I", pidSettings_p->Ki, "", scale, maxvalue);
  sendSlider(cmd + "d", title + " D", pidSettings_p->Kd, "", scale, maxvalue);
}

void RemoteControl::processPIDSlider(const String result, const String cmd,
                                     Pid& pid, const double scale,
                                     const float maxvalue)
{
  (void)maxvalue; //FIXME: Warning unused parameter

  int idx = result.indexOf('`');
  String s = result.substring(idx + 1);
  //Console.println(tmp);
  float v = stringToFloat(s);

  Pid_settingsT* pidSettings_p = pid.getSettings();
  float* ptr = nullptr;

  if (m_pfodCmd.startsWith(cmd + "p"))
  {
    ptr = &pidSettings_p->Kp;
  }
  else if (m_pfodCmd.startsWith(cmd + "i"))
  {
    ptr = &pidSettings_p->Ki;
  }
  else if (m_pfodCmd.startsWith(cmd + "d"))
  {
    ptr = &pidSettings_p->Kd;
  }

  if (ptr)
  {
    *ptr = v * scale;
    if (*ptr < scale)
    {
      *ptr = 0.0;
    }
  }
}

// TODO: Make template out of processSlider()
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

void RemoteControl::sendMainMenu(const bool update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Ardumower ("));
    Bluetooth.print(m_robot_p->m_name);
    Bluetooth.print(")");
  }
  Bluetooth.println(F("|r~Commands|n~Manual|s~Settings|in~Info|c~Test compass"
                      "|m1~Log sensors|yp~Plot|y4~Error counters"
                      "|y9~ADC calibration}"));
}

void RemoteControl::sendADCMenu(const bool update)
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

void RemoteControl::sendPlotMenu(const bool update)
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

void RemoteControl::sendSettingsMenu(const bool update)
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

void RemoteControl::sendErrorMenu(const bool update)
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
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_ADC_CALIB]);
  Bluetooth.print(F("|zz~Charger "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_CHARGER]);
  Bluetooth.print(F("|zz~Battery "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_BATTERY]);
  Bluetooth.print(F("|zz~Motor left "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_MOTOR_LEFT]);
  Bluetooth.print(F("|zz~Motor right "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_MOTOR_RIGHT]);
  Bluetooth.print(F("|zz~Motor mow "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_MOTOR_CUTTER]);
  Bluetooth.print(F("|zz~Mow sense "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_CUTTER_SENSE]);
  Bluetooth.print(F("|zz~Odometer left "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_ODOMETER_LEFT]);
  Bluetooth.print(F("|zz~Odometer right "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_ODOMETER_RIGHT]);
  Bluetooth.print(F("|zz~Perimeter timeout "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_PERIMETER_TIMEOUT]);
  Bluetooth.print(F("|zz~Perimeter tracking "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_TRACKING]);
  Bluetooth.print(F("|zz~IMU comm "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_IMU_COMM]);
  Bluetooth.print(F("|zz~IMU calibration "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_IMU_CALIB]);
  Bluetooth.print(F("|zz~IMU tilt "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_IMU_TILT]);
  Bluetooth.print(F("|zz~RTC comm "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_RTC_COMM]);
  Bluetooth.print(F("|zz~RTC data "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_RTC_DATA]);
  Bluetooth.print(F("|zz~GPS comm "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_GPS_COMM]);
  Bluetooth.print(F("|zz~GPS data "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_GPS_DATA]);
  Bluetooth.print(F("|zz~Robot stucked "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_STUCK]);
  Bluetooth.print(F("|zz~EEPROM data "));
  Bluetooth.print(m_robot_p->m_errorCounterMax[ERR_EEPROM_DATA]);
  Bluetooth.println("}");
}

void RemoteControl::processErrorMenu(const String pfodCmd)
{
  if (pfodCmd == "z00")
  {
    m_robot_p->resetErrorCounters();
    m_robot_p->setNextState(StateMachine::STATE_OFF, 0);
  }
  sendErrorMenu(true);
}

void RemoteControl::sendMotorMenu(const bool update)
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
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getOverloadCounter());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getOverloadCounter());
  Bluetooth.println(F("|a01~Power in Watt l, r "));
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getPowerMeas());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPowerMeas());
  Bluetooth.println(F("|a05~motor current in mA l, r "));
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getAverageCurrent());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getAverageCurrent());
  sendSlider("a02", F("Power max"),
             m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_powerMax, "", 1, 100);
  sendSlider("a03", F("calibrate left motor "),
             m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getAverageCurrent(),
             "", 1, 1000, 0);
  sendSlider("a04", F("calibrate right motor"),
             m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getAverageCurrent(),
             "", 1, 1000, 0);
  Bluetooth.print(F("|a05~Speed l, r"));
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getPwmCur());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPwmCur());
  sendSlider("a06", F("Speed max in rpm"),
             m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax, "", 1, 100);
  sendSlider("a15", F("Speed max in pwm"),
             m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pwmMax, "", 1, 255);
  sendSlider("a11", F("Accel"),
             m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_acceleration,
             "", 1, 2000, 500);
  sendSlider("a18", F("Power ignore time"),
             m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_powerIgnoreTime,
             "", 1, 8000);
  sendSlider("a07", F("Roll time max"),
             m_robot_p->m_wheels.m_rollTimeMax, "", 1, 8000);
  sendSlider("a19", F("Roll time min"),
             m_robot_p->m_wheels.m_rollTimeMin, "", 1,
             (m_robot_p->m_wheels.m_rollTimeMax - 500));
  sendSlider("a08", F("Reverse time"),
             m_robot_p->m_wheels.m_reverseTime, "", 1, 8000);
  sendSlider("a09", F("Forw time max"),
             m_robot_p->m_wheels.m_forwardTimeMax, "", 10, 80000);
  sendSlider("a12", F("Bidir speed ratio 1"),
             m_robot_p->m_wheels.m_biDirSpeedRatio1, "", 0.01, 1.0);
  sendSlider("a13", F("Bidir speed ratio 2"),
             m_robot_p->m_wheels.m_biDirSpeedRatio2, "", 0.01, 1.0);
  sendPIDSlider("a14", "RPM",
                m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid, 0.01, 3.0);
  Bluetooth.println(F("|a10~Testing is"));

  switch (m_testmode)
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
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getScale());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getScale());
  Bluetooth.print(F("|a16~Swap left direction "));
  sendYesNo(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_swapDir);
  Bluetooth.print(F("|a17~Swap right direction "));
  sendYesNo(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_swapDir);
  Bluetooth.println("}");
}

void RemoteControl::processMotorMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("a02"))
  {
    processSlider(pfodCmd,
                  m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_powerMax, 1);
    processSlider(pfodCmd,
                  m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_powerMax, 1);
  }

  else if (pfodCmd.startsWith("a03"))
  {
    float currentMeas =
        m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getAverageCurrent();
    processSlider(pfodCmd, currentMeas, 1);
    m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.setScale(
        m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getScale() /
        max(0, (float)m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getAverageSenseAdc()));
  }
  else if (pfodCmd.startsWith("a04"))
  {
    float currentMeas =
        m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getAverageCurrent();
    processSlider(pfodCmd, currentMeas, 1);
    m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.setScale(
        m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getScale() /
        max(0, (float)m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getAverageSenseAdc()));
  }
  else if (pfodCmd.startsWith("a06"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax, 1);
    processSlider(pfodCmd, m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmMax, 1);
  }
  else if (pfodCmd.startsWith("a15"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pwmMax, 1);
    processSlider(pfodCmd, m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pwmMax, 1);
  }
  else if (pfodCmd.startsWith("a07"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_rollTimeMax, 1);
  }
  else if (pfodCmd.startsWith("a19"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_rollTimeMin, 1);
  }
  else if (pfodCmd.startsWith("a08"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_reverseTime, 1);
  }
  else if (pfodCmd.startsWith("a09"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_forwardTimeMax, 10);
  }
  else if (pfodCmd.startsWith("a11"))
  {
    processSlider(pfodCmd,
                  m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_acceleration, 1);
    processSlider(pfodCmd,
                  m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_acceleration, 1);
  }
  else if (pfodCmd.startsWith("a12"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_biDirSpeedRatio1, 0.01);
  }
  else if (pfodCmd.startsWith("a13"))
  {
    processSlider(pfodCmd, m_robot_p->m_wheels.m_biDirSpeedRatio2, 0.01);
  }
  else if (pfodCmd.startsWith("a14"))
  {
    processPIDSlider(pfodCmd, "a14",
                     m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid, 0.01, 3.0);
    processPIDSlider(pfodCmd, "a14",
                     m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid, 0.01, 3.0);
  }
  else if (pfodCmd.startsWith("a16"))
  {
    TOGGLE(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_swapDir);
  }
  else if (pfodCmd.startsWith("a17"))
  {
    TOGGLE(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_swapDir);
  }
  else if (pfodCmd.startsWith("a18"))
  {
    processSlider(pfodCmd,
                  m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_powerIgnoreTime, 1);
    processSlider(pfodCmd,
                  m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_powerIgnoreTime, 1);
  }
  else if (pfodCmd == "a10")
  {
    m_testmode = (m_testmode + 1) % 3;
    switch (m_testmode)
    {
      case 0:
        m_robot_p->setNextState(StateMachine::STATE_OFF, 0);
        m_robot_p->setSpeed(0);
        m_robot_p->setSteer(0);
        break;

      case 1:
        m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
        m_robot_p->setSpeed(+50);
        m_robot_p->setSteer(+50);
//        robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet = 0;
//        robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet =
//            robot_p->wheels.wheel[Wheel::LEFT].motor.rpmMax;
        break;

      case 2:
        m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
        m_robot_p->setSpeed(+50);
        m_robot_p->setSteer(-50);
//        robot_p->wheels.wheel[Wheel::LEFT].motor.rpmSet = 0;
//        robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmSet =
//            robot_p->wheels.wheel[Wheel::RIGHT].motor.rpmMax;
        break;
    }
  }
  sendMotorMenu(true);
}

void RemoteControl::sendMowMenu(const bool update)
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
  Bluetooth.print(m_robot_p->m_cutter.m_motor.getOverloadCounter());
  Bluetooth.print(F("|o01~Power in Watt "));
  Bluetooth.print(m_robot_p->m_cutter.m_motor.getPowerMeas());
  Bluetooth.print(F("|o11~current in mA "));
  Bluetooth.print(m_robot_p->m_cutter.m_motor.getAverageCurrent());
  sendSlider("o02", F("Power max"), m_robot_p->m_cutter.m_motor.m_powerMax, "", 1, 100);
  sendSlider("o03", F("calibrate mow motor "),
             m_robot_p->m_cutter.m_motor.getAverageCurrent(), "", 1, 3000, 0);
  Bluetooth.print(F("|o04~Speed "));
  Bluetooth.print(m_robot_p->m_cutter.m_motor.getPwmCur());
  sendSlider("o05", F("Speed max"), m_robot_p->m_cutter.m_motor.m_pwmMax, "", 1, 255);
  if (m_robot_p->m_developerActive)
  {
    Bluetooth.print(F("|o06~Modulate "));
    sendYesNo(m_robot_p->m_cutter.m_motor.m_regulate);
  }
  Bluetooth.print(F("|o07~RPM "));
  Bluetooth.print(m_robot_p->m_cutter.m_motor.getRpmMeas());
  sendSlider("o08", F("RPM set"), m_robot_p->m_cutter.m_motor.m_rpmSet, "", 1, 4500);
  sendPIDSlider("o09", "RPM", m_robot_p->m_cutter.m_motor.m_pid, 0.01, 1.0);
  Bluetooth.println(F("|o10~Testing is"));

  switch (m_testmode)
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
  Bluetooth.print(m_robot_p->m_cutter.m_motor.getScale());
  Bluetooth.println("}");
}

void RemoteControl::processMowMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("o02"))
  {
    processSlider(pfodCmd, m_robot_p->m_cutter.m_motor.m_powerMax, 1);
  }
  else if (pfodCmd.startsWith("o03"))
  {
    float currentMeas = m_robot_p->m_cutter.m_motor.getAverageCurrent();
    processSlider(pfodCmd, currentMeas, 1);
    m_robot_p->m_cutter.m_motor.setScale(currentMeas /
        max(0, (float )m_robot_p->m_cutter.m_motor.getAverageSenseAdc()));
  }
  else if (pfodCmd.startsWith("o05"))
  {
    processSlider(pfodCmd, m_robot_p->m_cutter.m_motor.m_pwmMax, 1);
  }
  else if (pfodCmd == "o06")
  {
    TOGGLE(m_robot_p->m_cutter.m_motor.m_regulate);
  }
  else if (pfodCmd.startsWith("o08"))
  {
    processSlider(pfodCmd, m_robot_p->m_cutter.m_motor.m_rpmSet, 1);
  }
  else if (pfodCmd.startsWith("o09"))
  {
    processPIDSlider(pfodCmd, "o09", m_robot_p->m_cutter.m_motor.m_pid, 0.01, 1.0);
  }
  else if (pfodCmd == "o10")
  {
    m_testmode = (m_testmode + 1) % 2;
    switch (m_testmode)
    {
      case 0:
        m_robot_p->setNextState(StateMachine::STATE_OFF, 0);
        m_robot_p->m_cutter.m_motor.setRpmMeas(0);
        m_robot_p->m_cutter.disable();
        break;

      case 1:
        m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
        m_robot_p->m_cutter.enable();
        break;
    }
  }
  sendMowMenu(true);
}

void RemoteControl::sendBumperMenu(const bool update)
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
  sendYesNo(m_robot_p->m_bumpers.isUsed());
  Bluetooth.println(F("|b01~Counter l, r "));
  Bluetooth.print(m_robot_p->m_bumperArray[LEFT].getCounter());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_bumperArray[RIGHT].getCounter());
  Bluetooth.println(F("|b02~Value l, r "));
  Bluetooth.print(m_robot_p->m_bumperArray[LEFT].isHit());
  Bluetooth.print(", ");
  Bluetooth.print(m_robot_p->m_bumperArray[RIGHT].isHit());
  Bluetooth.println("}");
}

void RemoteControl::sendDropMenu(const bool update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Drop`1000"));
  }

  DropSensor* dropSensorLeft_p =
      &m_robot_p->m_dropSensors.m_dropSensorArray_p[LEFT];
  DropSensor* dropSensorRight_p =
      &m_robot_p->m_dropSensors.m_dropSensorArray_p[RIGHT];

  Bluetooth.print(F("|u00~Use "));
  sendYesNo(m_robot_p->m_dropSensors.m_use);
  Bluetooth.println(F("|u01~Counter l, r "));
  Bluetooth.print(dropSensorLeft_p->getCounter());
  Bluetooth.print(", ");
  Bluetooth.print(dropSensorRight_p->getCounter());
  Bluetooth.println(F("|u02~Value l, r "));
  Bluetooth.print(dropSensorLeft_p->isDetected());
  Bluetooth.print(", ");
  Bluetooth.print(dropSensorRight_p->isDetected());
  Bluetooth.println("}");
}

void RemoteControl::processBumperMenu(const String pfodCmd)
{
  if (pfodCmd == "b00")
  {
    TOGGLE(m_robot_p->m_bumpers.m_use);
  }
  sendBumperMenu(true);
}

void RemoteControl::processDropMenu(const String pfodCmd)
{
  if (pfodCmd == "u00")
  {
    TOGGLE(m_robot_p->m_dropSensors.m_use);
  }
  sendDropMenu(true);
}

void RemoteControl::sendSonarMenu(const bool update)
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
  sendYesNo(m_robot_p->m_sonars.use);
  Bluetooth.print(F("|d04~Use left "));
  sendYesNo(m_robot_p->m_sonars.sonarArray_p[LEFT].use);
  Bluetooth.print(F("|d05~Use center "));
  sendYesNo(m_robot_p->m_sonars.sonarArray_p[CENTER].use);
  Bluetooth.print(F("|d06~Use right "));
  sendYesNo(m_robot_p->m_sonars.sonarArray_p[RIGHT].use);
  Bluetooth.print(F("|d01~Counter "));
  Bluetooth.print(sonars_getDistanceCounter(&m_robot_p->m_sonars));
  Bluetooth.println(F("|d02~Value [cm] l, c, r"));
  for (uint8_t i = 0; i < SONARS_NUM; i++)
  {
    Bluetooth.print(sonar_getDistance_cm(&m_robot_p->m_sonars.sonarArray_p[i]));
    if (i < SONARS_NUM - 1)
    {
      Bluetooth.print(", ");
    }
  }
  sendSlider("d03", F("Trigger below [us]"), m_robot_p->m_sonars.triggerBelow, "", 1, 3000);
  Bluetooth.println("}");
}

void RemoteControl::processSonarMenu(const String pfodCmd)
{
  if (pfodCmd == "d00")
  {
    TOGGLE(m_robot_p->m_sonars.use);
  }
  else if (pfodCmd.startsWith("d03"))
  {
    processSlider(pfodCmd, m_robot_p->m_sonars.triggerBelow, 1);
  }
  else if (pfodCmd == "d04")
  {
    TOGGLE(m_robot_p->m_sonars.sonarArray_p[LEFT].use);
  }
  else if (pfodCmd == "d05")
  {
    TOGGLE(m_robot_p->m_sonars.sonarArray_p[CENTER].use);
  }
  else if (pfodCmd == "d06")
  {
    TOGGLE(m_robot_p->m_sonars.sonarArray_p[RIGHT].use);
  }
  sendSonarMenu(true);
}

void RemoteControl::sendPerimeterMenu(const bool update)
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
  sendYesNo(m_robot_p->m_perimeters.isUsed());
  Bluetooth.println(F("|e02~Value"));
  Bluetooth.print(m_robot_p->getPerimeterMag());
  if (m_robot_p->getPerimeterMag() < 0)
  {
    Bluetooth.print(" (inside)");
  }
  else
  {
    Bluetooth.print(" (outside)");
  }
  sendSlider("e08", F("Timed-out if below smag"),
             m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_timedOutIfBelowSmag, "", 1, 2000);
  sendSlider("e14", F("Timeout (s) if not inside"),
             m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_timeOutSecIfNotInside, "", 1, 20, 1);
  sendSlider("e04", F("Trigger timeout"),
             m_robot_p->m_perimeterTriggerTimeout, "", 1, 2000);
  sendSlider("e05", F("Perimeter out roll time max"),
             m_robot_p->m_perimeterOutRollTimeMax, "", 1, 8000);
  sendSlider("e06", F("Perimeter out roll time min"),
             m_robot_p->m_perimeterOutRollTimeMin, "", 1, 8000);
  sendSlider("e15", F("Perimeter out reverse time"),
             m_robot_p->m_perimeterOutRevTime, "", 1, 8000);
  sendSlider("e16", F("Perimeter tracking roll time"),
             m_robot_p->m_perimeterTrackRollTime, "", 1, 8000);
  sendSlider("e17", F("Perimeter tracking reverse time"),
             m_robot_p->m_perimeterTrackRevTime, "", 1, 8000);
  sendSlider("e11", F("Transition timeout"),
             m_robot_p->m_trackingPerimeterTransitionTimeOut, "", 1, 5000);
  sendSlider("e12", F("Track error timeout"),
             m_robot_p->m_trackingErrorTimeOut, "", 1, 10000);
  sendPIDSlider("e07", F("Track"), m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_pid, 0.1, 100);
  Bluetooth.print(F("|e09~Use differential signal "));
  sendYesNo(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_useDifferentialPerimeterSignal);
  Bluetooth.print(F("|e10~Swap coil polarity "));
  sendYesNo(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_swapCoilPolarity);
  Bluetooth.print(F("|e13~Block inner wheel  "));
  sendYesNo(m_robot_p->m_trackingBlockInnerWheelWhilePerimeterStruggling);
  Bluetooth.println("}");
}

void RemoteControl::processPerimeterMenu(const String pfodCmd)
{
  if (pfodCmd == "e00")
  {
    TOGGLE(m_robot_p->m_perimeters.m_use);
  }
  else if (pfodCmd.startsWith("e04"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeterTriggerTimeout, 1);
  }
  else if (pfodCmd.startsWith("e05"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeterOutRollTimeMax, 1);
  }
  else if (pfodCmd.startsWith("e06"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeterOutRollTimeMin, 1);
  }
  else if (pfodCmd.startsWith("e15"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeterOutRevTime, 1);
  }
  else if (pfodCmd.startsWith("e16"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeterTrackRollTime, 1);
  }
  else if (pfodCmd.startsWith("e17"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeterTrackRevTime, 1);
  }
  else if (pfodCmd.startsWith("e07"))
  {
    processPIDSlider(pfodCmd, "e07", m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_pid, 0.1, 100);
  }
  else if (pfodCmd.startsWith("e08"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_timedOutIfBelowSmag, 1);
  }
  else if (pfodCmd.startsWith("e09"))
  {
    TOGGLE(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_useDifferentialPerimeterSignal);
  }
  else if (pfodCmd.startsWith("e10"))
  {
    TOGGLE(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_swapCoilPolarity);
  }
  else if (pfodCmd.startsWith("e11"))
  {
    processSlider(pfodCmd, m_robot_p->m_trackingPerimeterTransitionTimeOut, 1);
  }
  else if (pfodCmd.startsWith("e12"))
  {
    processSlider(pfodCmd, m_robot_p->m_trackingErrorTimeOut, 1);
  }
  else if (pfodCmd.startsWith("e13"))
  {
    TOGGLE(m_robot_p->m_trackingBlockInnerWheelWhilePerimeterStruggling);
  }
  else if (pfodCmd.startsWith("e14"))
  {
    processSlider(pfodCmd, m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].m_timeOutSecIfNotInside, 1);
  }
  sendPerimeterMenu(true);
}

void RemoteControl::sendLawnSensorMenu(const bool update)
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
  sendYesNo(m_robot_p->m_lawnSensors.use);
  Bluetooth.print(F("|f01~Counter "));
  Bluetooth.print(lawnSensors_getCounter(&m_robot_p->m_lawnSensors));
  Bluetooth.println(F("|f02~Value f, b"));
  Bluetooth.print(lawnSensor_getValue(&m_robot_p->m_lawnSensorArray[FRONT]));
  Bluetooth.print(", ");
  Bluetooth.print(lawnSensor_getValue(&m_robot_p->m_lawnSensorArray[BACK]));
  Bluetooth.println("}");
}

void RemoteControl::processLawnSensorMenu(const String pfodCmd)
{
  if (pfodCmd == "f00")
  {
    TOGGLE(m_robot_p->m_lawnSensors.use);
  }
  sendLawnSensorMenu(true);
}

void RemoteControl::sendRainMenu(const bool update)
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
  sendYesNo(m_robot_p->m_rainSensor.use);
  Bluetooth.print(F("|m01~Counter "));
  Bluetooth.print(rainSensor_getCounter(&m_robot_p->m_rainSensor));
  Bluetooth.println(F("|m02~Value"));
  Bluetooth.print(rainSensor_isRaining(&m_robot_p->m_rainSensor));
  Bluetooth.println("}");
}

void RemoteControl::processRainMenu(const String pfodCmd)
{
  if (pfodCmd == "m00")
  {
    TOGGLE(m_robot_p->m_rainSensor.use);
  }
  sendRainMenu(true);
}

void RemoteControl::sendGPSMenu(const bool update)
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
  sendYesNo(m_robot_p->m_gpsUse);
  sendSlider("q01", F("Stuck if GPS speed is below"),
             m_robot_p->m_stuckIfGpsSpeedBelow, "", 0.1, 3);
  sendSlider("q02", F("GPS speed ignore time"),
             m_robot_p->m_gpsSpeedIgnoreTime,
             "", 1, 10000, m_robot_p->m_wheels.m_reverseTime);
  Bluetooth.println("}");
}

void RemoteControl::processGPSMenu(const String pfodCmd)
{
  if (pfodCmd == "q00")
  {
    TOGGLE(m_robot_p->m_gpsUse);
  }
  else if (pfodCmd.startsWith("q01"))
  {
    processSlider(pfodCmd, m_robot_p->m_stuckIfGpsSpeedBelow, 0.1);
  }
  else if (pfodCmd.startsWith("q02"))
  {
    processSlider(pfodCmd, m_robot_p->m_gpsSpeedIgnoreTime, 1);
  }
  sendGPSMenu(true);
}

void RemoteControl::sendImuMenu(const bool update)
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
  sendYesNo(m_robot_p->m_imu.m_use);
  Bluetooth.print(F("|g10~Use accel calib"));
  sendYesNo(m_robot_p->m_imu.getUseAccelCalibration());
  Bluetooth.print(F("|g01~Yaw "));
  Bluetooth.print(m_robot_p->m_imu.getYawDeg());
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g09~DriveHeading "));
  Bluetooth.print(m_robot_p->m_imuDriveHeading / PI * 180);
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g02~Pitch "));
  Bluetooth.print(m_robot_p->m_imu.getPitchDeg());
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g03~Roll "));
  Bluetooth.print(m_robot_p->m_imu.getRollDeg());
  Bluetooth.print(F(" deg"));
  Bluetooth.print(F("|g04~Correct dir "));
  sendYesNo(m_robot_p->m_imu.m_correctDir);
  sendPIDSlider("g05", F("Dir"), m_robot_p->m_imu.m_pid[Imu::DIR], 0.1, 20);
  sendPIDSlider("g06", F("Roll"), m_robot_p->m_imu.m_pid[Imu::ROLL], 0.1, 30);
  Bluetooth.print(F("|g07~Acc cal next side"));
  Bluetooth.print(F("|g08~Com cal start/stop"));
  Bluetooth.println("}");
}

void RemoteControl::processImuMenu(const String pfodCmd)
{
  if (pfodCmd == "g00")
  {
    TOGGLE(m_robot_p->m_imu.m_use);
  }
  else if (pfodCmd == "g10")
  {
    m_robot_p->m_imu.toggleUseAccelCalibration();
  }
  else if (pfodCmd == "g04")
  {
    TOGGLE(m_robot_p->m_imu.m_correctDir);
  }
  else if (pfodCmd.startsWith("g05"))
  {
    processPIDSlider(pfodCmd, "g05", m_robot_p->m_imu.m_pid[Imu::DIR], 0.1, 20);
  }
  else if (pfodCmd.startsWith("g06"))
  {
    processPIDSlider(pfodCmd, "g06", m_robot_p->m_imu.m_pid[Imu::ROLL], 0.1, 30);
  }
  else if (pfodCmd == "g07")
  {
    m_robot_p->m_imu.calibrateAccelerometerNextAxis();
  }
  else if (pfodCmd == "g08")
  {
    m_robot_p->m_imu.calibrateMagnetometerStartStop();
  }
  sendImuMenu(true);
}

void RemoteControl::sendRemoteMenu(const bool update)
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
  sendYesNo(m_robot_p->m_remoteUse);
  Bluetooth.println("}");
}

void RemoteControl::processRemoteMenu(const String pfodCmd)
{
  if (pfodCmd == "h00")
  {
    TOGGLE(m_robot_p->m_remoteUse);
  }
  sendRemoteMenu(true);
}

void RemoteControl::sendBatteryMenu(const bool update)
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
  Bluetooth.print(m_robot_p->m_battery.getVoltage());
  Bluetooth.print(" V");
  Bluetooth.print(F("|j01~Monitor "));
  sendYesNo(m_robot_p->m_battery.isMonitored());
  if (m_robot_p->m_developerActive)
  {
    sendSlider("j05", F("Calibrate batFactor "), m_robot_p->m_battery.m_batFactor, "",
               0.01, 1.0);
  }
  //Console.print("batFactor=");
  //Console.println(robot->batFactor);
  sendSlider("j02", F("Go home if below Volt"),
             m_robot_p->m_battery.m_batGoHomeIfBelow, "", 0.1, m_robot_p->m_battery.m_batFull,
             (m_robot_p->m_battery.m_batFull * 0.72)); // for Sony Konion cells 4.2V * 0,72= 3.024V which is pretty safe to use
  sendSlider("j12", F("Switch off if idle minutes"),
             m_robot_p->m_battery.m_batSwitchOffIfIdle, "", 1, 300, 1);
  sendSlider("j03", F("Switch off if below Volt"),
             m_robot_p->m_battery.m_batSwitchOffIfBelow, "", 0.1, m_robot_p->m_battery.m_batFull,
             (m_robot_p->m_battery.m_batFull * 0.72));
  Bluetooth.print(F("|j04~Charge "));
  Bluetooth.print(m_robot_p->m_battery.getChargeVoltage());
  Bluetooth.print("V ");
  Bluetooth.print(m_robot_p->m_battery.getChargeCurrent());
  Bluetooth.print("A");
  sendSlider("j09", F("Calibrate batChgFactor"),
             m_robot_p->m_battery.m_batChgFactor, "", 0.01, 1.0);
  sendSlider("j06", F("Charge sense zero"),
             m_robot_p->m_battery.m_chgSenseZero, "", 1, 600, 400);
  sendSlider("j08", F("Charge factor"), m_robot_p->m_battery.m_chgFactor, "", 0.01, 80);
  sendSlider("j10", F("charging starts if Voltage is below"),
             m_robot_p->m_battery.m_startChargingIfBelow, "", 0.1, m_robot_p->m_battery.m_batFull);
  sendSlider("j11", F("Battery is fully charged if current is below"),
             m_robot_p->m_battery.m_batFullCurrent, "", 0.1, m_robot_p->m_battery.m_batChargingCurrentMax);
  Bluetooth.println("}");
}

void RemoteControl::processBatteryMenu(const String pfodCmd)
{
  if (pfodCmd == "j01")
  {
    TOGGLE(m_robot_p->m_battery.m_monitored);
  }
  else if (pfodCmd.startsWith("j02"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_batGoHomeIfBelow, 0.1);
    //Console.print("gohomeifbelow=");
    //Console.println(robot->batGoHomeIfBelow);
  }
  else if (pfodCmd.startsWith("j03"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_batSwitchOffIfBelow, 0.1);
  }
  else if (pfodCmd.startsWith("j05"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_batFactor, 0.01);
  }
  else if (pfodCmd.startsWith("j06"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_chgSenseZero, 1);
  }
  else if (pfodCmd.startsWith("j08"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_chgFactor, 0.01);
  }
  else if (pfodCmd.startsWith("j09"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_batChgFactor, 0.01);
  }
  else if (pfodCmd.startsWith("j10"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_startChargingIfBelow, 0.1);
  }
  else if (pfodCmd.startsWith("j11"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_batFullCurrent, 0.1);
  }
  else if (pfodCmd.startsWith("j12"))
  {
    processSlider(pfodCmd, m_robot_p->m_battery.m_batSwitchOffIfIdle, 1);
  }
  sendBatteryMenu(true);
}

void RemoteControl::sendStationMenu(const bool update)
{
  if (update)
  {
    Bluetooth.print("{:");
  }
  else
  {
    Bluetooth.print(F("{.Station`1000"));
  }
  sendSlider("k00", F("Reverse time"), m_robot_p->m_stationRevTime, "", 1, 8000);
  sendSlider("k01", F("Roll time"), m_robot_p->m_stationRollTime, "", 1, 8000);
  sendSlider("k02", F("Forw time"), m_robot_p->m_stationForwTime, "", 1, 8000);
  sendSlider("k03", F("Station reverse check time"),
             m_robot_p->m_stationCheckTime, "", 1, 8000);
  Bluetooth.println("}");
}

void RemoteControl::processStationMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("k00"))
  {
    processSlider(pfodCmd, m_robot_p->m_stationRevTime, 1);
  }
  else if (pfodCmd.startsWith("k01"))
  {
    processSlider(pfodCmd, m_robot_p->m_stationRollTime, 1);
  }
  else if (pfodCmd.startsWith("k02"))
  {
    processSlider(pfodCmd, m_robot_p->m_stationForwTime, 1);
  }
  else if (pfodCmd.startsWith("k03"))
  {
    processSlider(pfodCmd, m_robot_p->m_stationCheckTime, 1);
  }
  sendStationMenu(true);
}

void RemoteControl::sendOdometerMenu(const bool update)
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
  sendYesNo(m_robot_p->m_odometer.m_use);
  Bluetooth.print(F("|l01~Value l, r "));
  Bluetooth.print(m_robot_p->m_odometer.m_encoder.left_p->getCounter());
  Bluetooth.print(", ");
  Bluetooth.println(m_robot_p->m_odometer.m_encoder.right_p->getCounter());
  Bluetooth.println(F("|l03~RPM Motor l, r "));
  Bluetooth.print(m_robot_p->m_odometer.m_encoder.left_p->getWheelRpmCurr());
  Bluetooth.print(", ");
  Bluetooth.println(m_robot_p->m_odometer.m_encoder.right_p->getWheelRpmCurr());
  sendSlider("l04", F("Ticks per one full revolution"),
             m_robot_p->m_odometer.m_ticksPerRevolution, "", 1, 2000);
  sendSlider("l01", F("Ticks per cm"),
             m_robot_p->m_odometer.m_ticksPerCm, "", 0.1, 30);
  sendSlider("l02", F("Wheel base cm"),
             m_robot_p->m_odometer.m_wheelBaseCm, "", 0.1, 50);
  Bluetooth.print(F("|l05~Swap left direction "));
  sendYesNo(m_robot_p->m_odometer.m_encoder.left_p->m_swapDir);
  Bluetooth.print(F("|l06~Swap right direction "));
  sendYesNo(m_robot_p->m_odometer.m_encoder.right_p->m_swapDir);
  Bluetooth.println("}");
}

void RemoteControl::processOdometerMenu(const String pfodCmd)
{
  if (pfodCmd == "l00")
  {
    TOGGLE(m_robot_p->m_odometer.m_use);
  }
  else if (pfodCmd.startsWith("l01"))
  {
    processSlider(pfodCmd, m_robot_p->m_odometer.m_ticksPerCm, 0.1);
  }
  else if (pfodCmd.startsWith("l02"))
  {
    processSlider(pfodCmd, m_robot_p->m_odometer.m_wheelBaseCm, 0.1);
  }
  else if (pfodCmd.startsWith("l04"))
  {
    processSlider(pfodCmd, m_robot_p->m_odometer.m_ticksPerRevolution, 1);
  }
  else if (pfodCmd.startsWith("l05"))
  {
    TOGGLE(m_robot_p->m_odometer.m_encoder.left_p->m_swapDir);
  }
  else if (pfodCmd.startsWith("l06"))
  {
    TOGGLE(m_robot_p->m_odometer.m_encoder.right_p->m_swapDir);
  }
  sendOdometerMenu(true);
}

void RemoteControl::sendDateTimeMenu(const bool update)
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
  Bluetooth.print(date2str(m_robot_p->m_datetime.date));
  Bluetooth.print(", ");
  Bluetooth.print(time2str(m_robot_p->m_datetime.time));
  sendSlider("t01", dayOfWeek[m_robot_p->m_datetime.date.dayOfWeek],
             m_robot_p->m_datetime.date.dayOfWeek, "", 1, 6, 0);
  sendSlider("t02", "Day ", m_robot_p->m_datetime.date.day, "", 1, 31, 1);
  sendSlider("t03", "Month ", m_robot_p->m_datetime.date.month, "", 1, 12, 1);
  sendSlider("t04", "Year ", m_robot_p->m_datetime.date.year, "", 1, 2020, 2013);
  sendSlider("t05", "Hour ", m_robot_p->m_datetime.time.hour, "", 1, 23, 0);
  sendSlider("t06", "Minute ", m_robot_p->m_datetime.time.minute, "", 1, 59, 0);
  Bluetooth.println("}");
}

void RemoteControl::processDateTimeMenu(const String pfodCmd)
{
  if (pfodCmd.startsWith("t01"))
  {
    processSlider(pfodCmd, m_robot_p->m_datetime.date.dayOfWeek, 1);
  }
  else if (pfodCmd.startsWith("t02"))
  {
    processSlider(pfodCmd, m_robot_p->m_datetime.date.day, 1);
  }
  else if (pfodCmd.startsWith("t03"))
  {
    processSlider(pfodCmd, m_robot_p->m_datetime.date.month, 1);
  }
  else if (pfodCmd.startsWith("t04"))
  {
    processSlider(pfodCmd, m_robot_p->m_datetime.date.year, 1);
  }
  else if (pfodCmd.startsWith("t05"))
  {
    processSlider(pfodCmd, m_robot_p->m_datetime.time.hour, 1);
  }
  else if (pfodCmd.startsWith("t06"))
  {
    processSlider(pfodCmd, m_robot_p->m_datetime.time.minute, 1);
  }
  sendDateTimeMenu(true);
  Console.print(F("setting RTC datetime: "));
  Console.println(date2str(m_robot_p->m_datetime.date));
  m_robot_p->setActuator(Robot::ACT_RTC, 0);
}

void RemoteControl::sendTimerDetailMenu(const int timerIdx,
                                        const bool update)
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
  sendYesNo(m_robot_p->m_timer[timerIdx].active);
  String sidx = String(timerIdx);
  sendSlider("p1" + sidx, F("Start hour "),
             m_robot_p->m_timer[timerIdx].startTime.hour, "", 1, 23, 0);
  sendSlider("p2" + sidx, F("Start minute "),
             m_robot_p->m_timer[timerIdx].startTime.minute, "", 1, 59, 0);
  sendSlider("p3" + sidx, F("Stop hour "),
             m_robot_p->m_timer[timerIdx].stopTime.hour, "", 1, 23, 0);
  sendSlider("p4" + sidx, F("Stop minute "),
             m_robot_p->m_timer[timerIdx].stopTime.minute, "", 1, 59, 0);
  for (int i = 0; i < 7; i++)
  {
    Bluetooth.print("|p5");
    Bluetooth.print(timerIdx);
    Bluetooth.print(i);
    Bluetooth.print("~");
    if ((m_robot_p->m_timer[timerIdx].daysOfWeek >> i) & 1)
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
  bool checkStop = false;
  bool checkStart = false;
  int startmin;
  int stopmin;
  int timerIdx = pfodCmd[2] - '0';
  if (pfodCmd.startsWith("p0"))
  {
    TOGGLE(m_robot_p->m_timer[timerIdx].active);
  }
  else if (pfodCmd.startsWith("p1"))
  {
    processSlider(pfodCmd, m_robot_p->m_timer[timerIdx].startTime.hour, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p2"))
  {
    processSlider(pfodCmd, m_robot_p->m_timer[timerIdx].startTime.minute, 1);
    checkStop = true;
  }
  else if (pfodCmd.startsWith("p3"))
  {
    processSlider(pfodCmd, m_robot_p->m_timer[timerIdx].stopTime.hour, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p4"))
  {
    processSlider(pfodCmd, m_robot_p->m_timer[timerIdx].stopTime.minute, 1);
    checkStart = true;
  }
  else if (pfodCmd.startsWith("p9"))
  {
    m_robot_p->m_timer[timerIdx].startTime = m_robot_p->m_datetime.time;
    checkStop = true;
    m_robot_p->m_timer[timerIdx].daysOfWeek =
        (1 << m_robot_p->m_datetime.date.dayOfWeek);
  }
  else if (pfodCmd.startsWith("p5"))
  {
    int day = pfodCmd[3] - '0';
    m_robot_p->m_timer[timerIdx].daysOfWeek =
        m_robot_p->m_timer[timerIdx].daysOfWeek ^ (1 << day);
  }
  if (checkStop)
  {
    // adjust start time
    startmin = min(1434, time2minutes(m_robot_p->m_timer[timerIdx].startTime));
    minutes2time(startmin, time);
    m_robot_p->m_timer[timerIdx].startTime = time;
    // check stop time
    stopmin = time2minutes(m_robot_p->m_timer[timerIdx].stopTime);
    stopmin = max(stopmin, startmin + 5);
    minutes2time(stopmin, time);
    m_robot_p->m_timer[timerIdx].stopTime = time;
  }
  else if (checkStart)
  {
    // adjust stop time
    stopmin = max(5, time2minutes(m_robot_p->m_timer[timerIdx].stopTime));
    minutes2time(stopmin, time);
    m_robot_p->m_timer[timerIdx].stopTime = time;
    // check start time
    startmin = time2minutes(m_robot_p->m_timer[timerIdx].startTime);
    startmin = min(startmin, stopmin - 5);
    minutes2time(startmin, time);
    m_robot_p->m_timer[timerIdx].startTime = time;
  }
  sendTimerDetailMenu(timerIdx, true);
}

void RemoteControl::sendTimerMenu(const bool update)
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
  sendYesNo(m_robot_p->m_timerUse);
  for (int i = 0; i < MAX_TIMERS; i++)
  {
    Bluetooth.print("|i0");
    Bluetooth.print(i);
    Bluetooth.print("~");
    sendTimer(m_robot_p->m_timer[i]);
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
      TOGGLE(m_robot_p->m_timerUse);
    }
    sendTimerMenu(true);
  }
}

void RemoteControl::sendFactorySettingsMenu(const bool update)
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
    m_robot_p->deleteUserSettings();
  }
  sendFactorySettingsMenu(true);
}

void RemoteControl::sendInfoMenu(const bool update)
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
  sendYesNo(m_robot_p->m_developerActive);
  Bluetooth.print(F("|v02~Mowing time trip (min) "));
  Bluetooth.print(m_robot_p->m_stats.mowTimeMinutesTrip);
  Bluetooth.print(F("|v03~Mowing time total (hrs) "));
  Bluetooth.print(m_robot_p->getStatsMowTimeHoursTotal());
  Bluetooth.print(F("|v05~Battery charging cycles "));
  Bluetooth.print(m_robot_p->m_stats.batteryChargingCounterTotal);
  Bluetooth.print(F("|v06~Battery recharged capacity trip (mAh)"));
  Bluetooth.print(m_robot_p->m_stats.batteryChargingCapacityTrip);
  Bluetooth.print(F("|v07~Battery recharged capacity total (Ah)"));
  Bluetooth.print(m_robot_p->m_stats.batteryChargingCapacityTotal / 1000);
  Bluetooth.print(F("|v08~Battery recharged capacity average (mAh)"));
  Bluetooth.print(m_robot_p->m_stats.batteryChargingCapacityAverage);
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
    TOGGLE(m_robot_p->m_developerActive);
  }
  m_robot_p->saveUserSettings();

  sendInfoMenu(true);
}

void RemoteControl::sendCommandMenu(const bool update)
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
  sendOnOff(m_robot_p->m_cutter.isEnabled());
  Bluetooth.print(F("|rp~Pattern is "));
  Bluetooth.print(m_robot_p->mowPatternName());
  Bluetooth.print(F("|rh~Home|rk~Track|rs~State is "));
  Bluetooth.print(m_robot_p->m_stateMachine.getCurrentStateName());
  Bluetooth.print(F("|rr~Auto rotate is "));
  Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getPwmCur());
  Bluetooth.print(F("|r1~User switch 1 is "));
  sendOnOff(m_robot_p->m_userSwitch1);
  Bluetooth.print(F("|r2~User switch 2 is "));
  sendOnOff(m_robot_p->m_userSwitch2);
  Bluetooth.print(F("|r3~User switch 3 is "));
  sendOnOff(m_robot_p->m_userSwitch3);
  Bluetooth.println("}");
}

void RemoteControl::processCommandMenu(const String pfodCmd)
{
  if (pfodCmd == "ro")
  {
    // cmd: off
    m_robot_p->setNextState(StateMachine::STATE_OFF, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rh")
  {
    // cmd: home
    m_robot_p->setNextState(StateMachine::STATE_PERI_FIND, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rr")
  {
    m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet += 10;
    m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet =
        -m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet;
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rk")
  {
    // cmd: track perimeter
    m_robot_p->setNextState(StateMachine::STATE_PERI_TRACK, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "ra")
  {
    // cmd: start auto mowing
    m_robot_p->m_cutter.enable();
    m_robot_p->setNextState(StateMachine::STATE_FORWARD, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rc")
  {
    // cmd: start remote control (RC)
    m_robot_p->m_cutter.enable();
    m_robot_p->m_cutter.m_motor.m_regulate = false;
    m_robot_p->setNextState(StateMachine::STATE_REMOTE, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "rm")
  {
    // cmd: mower motor on/off
    if (m_robot_p->m_stateMachine.isCurrentState(StateMachine::STATE_OFF) ||
        m_robot_p->m_stateMachine.isCurrentState(StateMachine::STATE_MANUAL))
    {
      m_robot_p->m_cutter.setEnableOverriden(false);
    }
    else
    {
      m_robot_p->m_cutter.toggleEnableOverriden();
    }
    m_robot_p->m_cutter.toggleEnabled();
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
    m_robot_p->m_mowPatternCurr = (m_robot_p->m_mowPatternCurr + 1) % 3;
    m_robot_p->setNextState(StateMachine::STATE_OFF, 0);
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r1")
  {
    TOGGLE(m_robot_p->m_userSwitch1);
    m_robot_p->setUserSwitches();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r2")
  {
    TOGGLE(m_robot_p->m_userSwitch2);
    m_robot_p->setUserSwitches();
    sendCommandMenu(true);
  }
  else if (pfodCmd == "r3")
  {
    TOGGLE(m_robot_p->m_userSwitch3);
    m_robot_p->setUserSwitches();
    sendCommandMenu(true);
  }
}

void RemoteControl::sendManualMenu(const bool update)
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
  if ((m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet < 5 &&
       m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet > -5) &&
      (m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet < 5 &&
       m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet > -5))
  {
    Bluetooth.print(F("|nb~Reverse"));
  }
  else
  {
    Bluetooth.print(F("|ns~Stop"));
  }
  Bluetooth.print(F("|nm~Mow is "));
  sendOnOff(m_robot_p->m_cutter.isEnabled());
  Bluetooth.println("}");
}

void RemoteControl::sendCompassMenu(const bool update)
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
  Bluetooth.print(m_robot_p->m_imu.getYawDeg());
  Bluetooth.println(F("|cs~South|cm~Mow}"));
}

void RemoteControl::processCompassMenu(const String pfodCmd)
{
  if (pfodCmd == "cm")
  {
    m_robot_p->m_cutter.toggleEnabled();
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cn")
  {
    m_robot_p->m_imuRollHeading = 0;
    m_robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cs")
  {
    m_robot_p->m_imuRollHeading = PI;
    m_robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
  else if (pfodCmd == "cw")
  {
    m_robot_p->m_imuRollHeading = -PI / 2;
    m_robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
  else if (pfodCmd == "ce")
  {
    m_robot_p->m_imuRollHeading = PI / 2;
    m_robot_p->setNextState(StateMachine::STATE_ROLL_WAIT, 0);
    sendCompassMenu(true);
  }
}

void RemoteControl::processManualMenu(const String pfodCmd)
{
  if (pfodCmd == "nl")
  {
    // manual: left
    m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    int sign = 1;
    if (m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet < 0)
    {
      sign = -1;
    }
    if (sign * m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet >=
        sign * m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet)
    {
      m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet =
          sign * m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax / 2;
    }
    else
    {
      m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet /= 2;
    }
    m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet =
        sign * m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax;

    sendManualMenu(true);
  }
  else if (pfodCmd == "nr")
  {
    // manual: right
    m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    int sign = 1;
    if (m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet < 0)
    {
      sign = -1;
    }
    if (sign * m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet >=
        sign * m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet)
    {
      m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet =
          sign * m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax / 2;
    }
    else
    {
      m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet /= 2;
    }
    m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet =
        sign * m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax;

    sendManualMenu(true);
  }
  else if (pfodCmd == "nf")
  {
    // manual: forward
    m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet =
        m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax;
    m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet =
        m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nb")
  {
    // manual: reverse
    m_robot_p->setNextState(StateMachine::STATE_MANUAL, 0);
    m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet =
        -m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax;
    m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet =
        -m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmMax;
    sendManualMenu(true);
  }
  else if (pfodCmd == "nm")
  {
    // manual: mower ON/OFF
    m_robot_p->m_cutter.toggleEnabled();
    sendManualMenu(true);
  }
  else if (pfodCmd == "ns")
  {
    // manual: stop
    //setNextState(STATE_OFF, 0);
    m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_rpmSet =
        m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_rpmSet = 0;
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
    m_robot_p->saveUserSettings();
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

  if (m_pfodState == PFOD_LOG_SENSORS)
  {
    //robot->printInfo(Bluetooth);
    //Bluetooth.println("test");
    Bluetooth.print(elapsedSeconds);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getPowerMeas());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPowerMeas());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_cutter.m_motor.getPowerMeas());
    Bluetooth.print(",");
    for (uint8_t i = 0; i < SONARS_NUM; i++)
    {
      Bluetooth.print(sonar_getDistance_us(&m_robot_p->m_sonars.sonarArray_p[i]));
      Bluetooth.print(",");
    }
    Bluetooth.print(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].isInside());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->getPerimeterMag());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_odometer.m_encoder.left_p->getCounter());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_odometer.m_encoder.right_p->getCounter());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.getYawDeg());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.getPitchDeg());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.getRollDeg());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_gyro.m_g.x / PI * 180);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_gyro.m_g.y / PI * 180);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_gyro.m_g.z / PI * 180);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_acc.x);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_acc.y);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_acc.z);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_mag.x);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_mag.y);
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_imu.m_mag.z);
    Bluetooth.print(",");
    float lat, lon;
    unsigned long age;
    m_robot_p->m_gps.f_get_position(&lat, &lon, &age);
    Bluetooth.print(m_robot_p->m_gps.hdop());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_gps.satellites());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_gps.f_speed_kmph());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_gps.f_course());
    Bluetooth.print(",");
    Bluetooth.print(m_robot_p->m_gps.f_altitude());
    Bluetooth.print(",");
    Bluetooth.print(lat);
    Bluetooth.print(",");
    Bluetooth.print(lon);
    Bluetooth.println();
  }
  else if (m_pfodState == PFOD_PLOT_BAT)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 60000;
      Bluetooth.print(curMillis / 60000);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_battery.getVoltage());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_battery.getChargeVoltage());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_battery.getChargeCurrent());
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->m_battery.getCapacity());
    }
  }
  else if (m_pfodState == PFOD_PLOT_ODO2D)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 500;
      Bluetooth.print(m_robot_p->m_odometer.getX());
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->m_odometer.getY());
    }
  }
  else if (m_pfodState == PFOD_PLOT_IMU)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 200;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.getYawDeg());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.getPitchDeg());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.getRollDeg());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_gyro.m_g.x / PI * 180);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_gyro.m_g.y / PI * 180);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_gyro.m_g.z / PI * 180);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_acc.x);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_acc.y);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_acc.z);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_mag.x);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_imu.m_mag.y);
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->m_imu.m_mag.z);
    }
  }
  else if (m_pfodState == PFOD_PLOT_SENSOR_COUNTERS)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 200;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_stateMachine.getCurrentState());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getOverloadCounter());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getOverloadCounter());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_cutter.m_motor.getOverloadCounter());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_bumperArray[LEFT].getCounter());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_bumperArray[RIGHT].getCounter());
      Bluetooth.print(",");
      Bluetooth.print(sonars_getDistanceCounter(&m_robot_p->m_sonars));
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->getPerimeterCounter());
      Bluetooth.print(",");
      Bluetooth.print(lawnSensors_getCounter(&m_robot_p->m_lawnSensors));
      Bluetooth.print(",");
      Bluetooth.print(rainSensor_getCounter(&m_robot_p->m_rainSensor));
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_dropSensorArray[LEFT].getCounter());
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->m_dropSensorArray[RIGHT].getCounter());
    }
  }
  else if (m_pfodState == PFOD_PLOT_SENSORS)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 200;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_stateMachine.getCurrentState());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getPowerMeas());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPowerMeas());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_cutter.m_motor.getPowerMeas());
      Bluetooth.print(",");
      for (uint8_t i = 0; i < SONARS_NUM; i++)
      {
        Bluetooth.print(sonar_getDistance_cm(&m_robot_p->m_sonars.sonarArray_p[i]));
        Bluetooth.print(",");
      }
      Bluetooth.print(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].isInside());
      Bluetooth.print(",");
      Bluetooth.print(lawnSensors_isDetected(&m_robot_p->m_lawnSensors));
      Bluetooth.print(",");
      Bluetooth.print(rainSensor_isRaining(&m_robot_p->m_rainSensor));
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_dropSensorArray[LEFT].isDetected());
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->m_dropSensorArray[RIGHT].isDetected());
    }
  }
  else if (m_pfodState == PFOD_PLOT_PERIMETER)
  {
    if (curMillis >= m_nextPlotTime)
    {
      if (m_perimeterCaptureIdx >= 32 * 3)
      {
        if (ADCMan.isCaptureComplete(A2))  //FIXME: Use define PIN_PERIMETER_LEFT
        {
          const int8_t* samples_p = ADCMan.getCapture(A2);  //FIXME: Use define PIN_PERIMETER_LEFT
          memcpy(m_perimeterCapture, samples_p, 32);
          m_perimeterCaptureIdx = 0;
        }
      }
      if (m_perimeterCaptureIdx < 32 * 3)
      {
        m_nextPlotTime = curMillis + 200;
        Bluetooth.print(m_perimeterCapture[m_perimeterCaptureIdx / 3]);
        Bluetooth.print(",");
        Bluetooth.print(m_robot_p->getPerimeterMag());
        Bluetooth.print(",");
        Bluetooth.print(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].getSmoothMagnitude());
        Bluetooth.print(",");
        Bluetooth.print(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].isInside());
        Bluetooth.print(",");
        Bluetooth.print(m_robot_p->getPerimeterCounter());
        Bluetooth.print(",");
        Bluetooth.print(!m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].signalTimedOut());
        Bluetooth.print(",");
        Bluetooth.println(m_robot_p->m_perimeters.m_perimeterArray_p[PERIMETER_LEFT].getFilterQuality());
        m_perimeterCaptureIdx++;
      }
    }
  }
  else if (m_pfodState == PFOD_PLOT_GPS)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 200;
      float lat, lon;
      unsigned long age;
      m_robot_p->m_gps.f_get_position(&lat, &lon, &age);
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_gps.hdop());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_gps.satellites());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_gps.f_speed_kmph());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_gps.f_course());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_gps.f_altitude());
      Bluetooth.print(",");
      Bluetooth.print(lat);
      Bluetooth.print(",");
      Bluetooth.println(lon);
    }
  }
  else if (m_pfodState == PFOD_PLOT_GPS2D)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 500;
      Bluetooth.print(m_robot_p->getGpsX());
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->getGpsY());
    }
  }
  else if (m_pfodState == PFOD_PLOT_MOTOR)
  {
    if (curMillis >= m_nextPlotTime)
    {
      m_nextPlotTime = curMillis + 50;
      Bluetooth.print(elapsedSeconds);
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_odometer.m_encoder.left_p->getWheelRpmCurr());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_odometer.m_encoder.right_p->getWheelRpmCurr());
      Bluetooth.print(",");
      //      Bluetooth.print(robot->motorLeftSpeedRpmSet);
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid.getSetPoint());
      Bluetooth.print(",");
      //      Bluetooth.print(robot->motorRightSpeedRpmSet);
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid.getSetPoint());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.getPwmCur());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.getPwmCur());
      Bluetooth.print(",");
      Bluetooth.print(m_robot_p->m_wheels.m_wheel[Wheel::LEFT].m_motor.m_pid.getErrorOld());
      Bluetooth.print(",");
      Bluetooth.println(m_robot_p->m_wheels.m_wheel[Wheel::RIGHT].m_motor.m_pid.getErrorOld());
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
        m_pfodCmdComplete = true;
      }
      else if (ch == '{')
      {
        m_pfodCmd = "";
      }
      else
      {
        m_pfodCmd += ch;
      }
    }
    if (m_pfodCmdComplete)
    {
      parsePfodCmd();
      m_pfodCmd = "";
      m_pfodCmdComplete = false;
    }
  }
  return res;
}

void RemoteControl::parsePfodCmd()
{
  Console.print("pfod cmd=");
  Console.println(m_pfodCmd);
  m_pfodState = PFOD_MENU;
  if (m_pfodCmd == ".")
  {
    sendMainMenu(false);
  }
  else if (m_pfodCmd == "m1")
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
    m_pfodState = PFOD_LOG_SENSORS;
  }
  else if (m_pfodCmd == "y1")
  {
    // plot battery
    Bluetooth.println(F("{=battery|time min`0|battery V`1|"
                        "charge V`1|charge A`2|capacity Ah`3}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_BAT;
  }
  else if (m_pfodCmd == "y2")
  {
    // plot odometer 2d
    Bluetooth.println(F("{=odometer2d|position`0~~~x|`~~~y}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_ODO2D;
  }
  else if (m_pfodCmd == "y3")
  {
    // plot IMU
    Bluetooth.println(F("{=IMU`60|time s`0|yaw`1~180~-180|pitch`1|roll`1"
                        "|gyroX`2~90~-90|gyroY`2|gyroZ`2"
                        "|accX`3~2~-2|accY`3|accZ`3"
                        "|comX`4~2~-2|comY`4|comZ`4}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_IMU;
  }
  else if (m_pfodCmd == "y5")
  {
    // plot sensor counters
    Bluetooth.println(F("{=Sensor counters`300"
                        "|time s`0|state`1"
                        "|motL`2|motR`3|motM`4"
                        "|bumL`5|bumR`6"
                        "|son`7|peri`8|lawn`9|rain`10"
                        "|dropL`11|dropR`12}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_SENSOR_COUNTERS;
  }
  else if (m_pfodCmd == "y6")
  {
    // plot perimeter spectrum
    /*Bluetooth.print(F("{=Perimeter spectrum`"));
     Bluetooth.print(Perimeter.getFilterBinCount());
     Bluetooth.print(F("|freq (Hz)`0|magnitude`0~60~-1|selected band`0~60~-1}"));*/
    Bluetooth.println(F("{=Perimeter`128|sig`1|mag`2|smag`3"
                        "|in`4|cnt`5|on`6|qty`7}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_PERIMETER;
  }
  else if (m_pfodCmd == "y7")
  {
    // plot sensor values
    Bluetooth.println(F("{=Sensors`300"
                        "|time s`0|state`1"
                        "|motL`2|motR`3|motM`4"
                        "|sonL`5|sonC`6|sonR`7"
                        "|peri`8|lawn`9|rain`10"
                        "|dropL`11|dropR`12}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_SENSORS;
  }
  else if (m_pfodCmd == "y8")
  {
    // plot GPS
    Bluetooth.println(F("{=GPS`300"
                        "|time s`0|hdop`1|sat`2|spd`3|course`4|alt`5"
                        "|lat`6|lon`7}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_GPS;
  }
  else if (m_pfodCmd == "y10")
  {
    // plot GPS 2d
    Bluetooth.println(F("{=gps2d|position`0~~~x|`~~~y}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_GPS2D;
  }
  else if (m_pfodCmd == "c1")
  {
    // ADC calibration
    ADCMan.calibrate();
    m_robot_p->m_buzzer.beepLong(2);
  }
  else if (m_pfodCmd == "y11")
  {
    // motor control
    Bluetooth.println(F("{=Motor control`300"
                        "|time s`0"
                        "|lrpm_curr`1|rrpm_curr`2"
                        "|lrpm_set`3|rrpm_set`4"
                        "|lpwm`5|rpwm`6"
                        "|lerr`7|rerr`8}"));
    m_nextPlotTime = 0;
    m_pfodState = PFOD_PLOT_MOTOR;
  }
  else if (m_pfodCmd == "yp")
  {
    sendPlotMenu(false);
  }
  else if (m_pfodCmd == "y4")
  {
    sendErrorMenu(false);
  }
  else if (m_pfodCmd == "y9")
  {
    sendADCMenu(false);
  }
  else if (m_pfodCmd == "n")
  {
    sendManualMenu(false);
  }
  else if (m_pfodCmd == "s")
  {
    sendSettingsMenu(false);
  }
  else if (m_pfodCmd == "r")
  {
    sendCommandMenu(false);
  }
  else if (m_pfodCmd == "c")
  {
    sendCompassMenu(false);
  }
  else if (m_pfodCmd == "t")
  {
    sendDateTimeMenu(false);
  }
  else if (m_pfodCmd == "i")
  {
    sendTimerMenu(false);
  }
  else if (m_pfodCmd == "in")
  {
    sendInfoMenu(false);
  }
  else if (m_pfodCmd.startsWith("s"))
  {
    processSettingsMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("r"))
  {
    processCommandMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("c"))
  {
    processCompassMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("n"))
  {
    processManualMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("a"))
  {
    processMotorMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("o"))
  {
    processMowMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("b"))
  {
    processBumperMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("d"))
  {
    processSonarMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("e"))
  {
    processPerimeterMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("f"))
  {
    processLawnSensorMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("g"))
  {
    processImuMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("h"))
  {
    processRemoteMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("j"))
  {
    processBatteryMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("k"))
  {
    processStationMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("l"))
  {
    processOdometerMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("m"))
  {
    processRainMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("q"))
  {
    processGPSMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("t"))
  {
    processDateTimeMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("i"))
  {
    processTimerMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("p"))
  {
    processTimerDetailMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("x"))
  {
    processFactorySettingsMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("u"))
  {
    processDropMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("v"))
  {
    processInfoMenu(m_pfodCmd);
  }
  else if (m_pfodCmd.startsWith("z"))
  {
    processErrorMenu(m_pfodCmd);
  }
  else
  {
    // no match
    Bluetooth.println("{}");
  }
  Bluetooth.flush();
}
