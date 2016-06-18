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
 2. set perimeter pins:    Perimeter.setup(PIN_PERIMETER_LEFT, PIN_PERIMETER_RIGHT);
 3. read perimeter:        int value = Perimeter.getMagnitude(0);
 */

#ifndef PERIMETER_H
#define PERIMETER_H

#include <Arduino.h>

#include "Pid.h"

class Perimeter
{
  public:
    Perimeter() { };
    // set ADC pins
    void setup(const uint8_t idx0Pin, const uint8_t idx1Pin);
    // get perimeter magnitude
    int16_t getMagnitude(const uint8_t idx);

    const int16_t getSmoothMagnitude(const uint8_t idx) const
    {
      return (int16_t)smoothMag[idx];
    }

    // inside perimeter (true) or outside (false)?
    // perimeter signal timed out? (e.g. due to broken wire)
    boolean signalTimedOut(const uint8_t idx);

    const int16_t getSignalMin(const uint8_t idx) const
    {
      return signalMin[idx];
    }

    const int16_t getSignalMax(const uint8_t idx) const
    {
      return signalMax[idx];
    }

    const int16_t getSignalAvg(const uint8_t idx) const
    {
      return signalAvg[idx];
    }

    const float getFilterQuality(const uint8_t idx) const
    {
      return filterQuality[idx];
    }

    const bool isInside(const uint8_t idx) const
    {
      return (signalCounter[idx] < 0);
    }

    bool isTimeToControl(void);


    Pid pid;             // perimeter PID controller
    int16_t timedOutIfBelowSmag {300};
    int16_t timeOutSecIfNotInside {8};
    // use differential perimeter signal as input for the matched filter?
    bool useDifferentialPerimeterSignal {true};
    // swap coil polarity?
    bool swapCoilPolarity {false};

  private:
    static const uint8_t timeBetweenControl {100};

    uint32_t lastInsideTime[2] {};
    uint8_t idxPin[2]; // channel for idx
    int16_t mag[2] {}; // perimeter magnitude per channel
    float smoothMag[2] {};
    float filterQuality[2] {};
    int16_t signalMin[2];
    int16_t signalMax[2];
    int16_t signalAvg[2];
    int16_t signalCounter[2] {};
    uint8_t subSample {};
    uint32_t nextTimeControl {};

    void matchedFilter(const uint8_t idx);
    int16_t corrFilter(const int8_t* H_p,
                       const int8_t subsample,
                       const int16_t M,
                       const int8_t* ip_p,
                       const int16_t nPts,
                       float &quality);
    void printADCMinMax(const int8_t* samples_p);
};

#endif
