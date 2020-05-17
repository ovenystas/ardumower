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

/*
 perimeter v2 receiver for Arduino sound sensors/LM386 using digital filter:
 matched filter - evaluates signal polarity of 'pulse3' signal on one ADC pin
 (for one coil)
 (for details see    http://wiki.ardumower.de/index.php?title=Perimeter_wire )

 How to use it (example):
 1. initialize ADC:        ADCMan.init();
 2. set perimeter pins:    perimeter.setup(PIN_PERIMETER);
 3. read perimeter:        int16_t value = Perimeter.calcMagnitude();
 */
#pragma once

#include <Arduino.h>
#include "Pid.h"
#include "Setting.h"
#include "CrossCorrelationFilter.h"
#include "Drivers.h"

struct PerimeterSettings
{
  Setting<bool> use;                // Use the perimeter sensors or not
  Setting<uint16_t> timedOutIfBelowSmag;
  Setting<bool> useDifferentialPerimeterSignal;
  Setting<bool> swapCoilPolarity;
  Setting<uint16_t> timeOutSecIfNotInside;
};

class Perimeter
{
public:
  Perimeter() = default;
  Perimeter(uint8_t idxPin)
  {
    setup(idxPin);
  }

  void setup(const uint8_t idxPin);

  bool isUsed()
  {
    return m_use;
  }

  int16_t calcMagnitude();

  int16_t getMagnitude()
  {
    return m_mag;
  }

  int16_t getSmoothMagnitude()
  {
    return static_cast<int16_t>(round(m_smoothMag));
  }

  bool signalTimedOut();

  float getFilterQuality()
  {
    return m_filterQuality;
  }

  bool isInside()
  {
    return m_inside;
  }

  void printInfo(Stream &s);

  void calcStatistics(const int8_t* samples_p);

  void matchedFilter();

  PerimeterSettings* getSettings()
  {
    return &m_settings;
  }

  void setSettings(PerimeterSettings* settings_p)
  {
    m_settings.use.value = settings_p->use.value;
    m_settings.timedOutIfBelowSmag.value = settings_p->timedOutIfBelowSmag.value;
    m_settings.useDifferentialPerimeterSignal.value = settings_p->useDifferentialPerimeterSignal.value;
    m_settings.swapCoilPolarity.value = settings_p->swapCoilPolarity.value;
    m_settings.timeOutSecIfNotInside.value = settings_p->timeOutSecIfNotInside.value;
 }

public:
  Pid m_pid;             // perimeter PID controller

private:
  uint32_t m_lastInsideTime {};
  uint8_t m_idxPin {}; // channel for idx
  int16_t m_mag {}; // perimeter magnitude per channel
  float m_smoothMag {};
  float m_filterQuality {};
  int8_t m_signalMin { INT8_MAX };
  int8_t m_signalMax { INT8_MIN };
  int8_t m_signalAvg {};
  int8_t m_signalCounter {};
  uint8_t m_subSample {};
  bool m_inside { true };

  int16_t m_sumMaxTmp {};
  int16_t m_sumMinTmp {};

  uint8_t m_callCounter = 0;
  uint32_t m_callCounter2 = 0;

  CrossCorrelationFilter m_corrFilter { Console };

  PerimeterSettings m_settings
  {
    { "Use", false },
    { "Timeout if below smag", "", 300, 0, 2000 },
    { "Use differential signal", true },
    { "Swap coil polarity", false },
    { "Timeout if not inside", "s", 8, 1, 20 },
  };

  // Shorter convenient variables for settings variables
  bool& m_use = m_settings.use.value;
  uint16_t& m_timedOutIfBelowSmag = m_settings.timedOutIfBelowSmag.value;
  bool& m_useDifferentialPerimeterSignal = m_settings.useDifferentialPerimeterSignal.value;
  bool& m_swapCoilPolarity = m_settings.swapCoilPolarity.value;
  uint16_t& m_timeOutSecIfNotInside = m_settings.timeOutSecIfNotInside.value;
};
