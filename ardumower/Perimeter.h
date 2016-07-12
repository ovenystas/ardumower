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

#ifndef PERIMETER_H
#define PERIMETER_H

#include <Arduino.h>

#include "Pid.h"

class Perimeter
{
  public:
    enum perimeterE
    {
      LEFT,
      //RIGHT,
      END
    };

    Perimeter()
    {
    }
    ;
    // set ADC pins
    void setup(const uint8_t idxPin);
    // get perimeter magnitude
    int16_t calcMagnitude(void);

    const int16_t getMagnitude(void) const
    {
      return mag;
    }

    const int16_t getSmoothMagnitude(void) const
    {
      return (int16_t)smoothMag;
    }

    // inside perimeter (true) or outside (false)?
    // perimeter signal timed out? (e.g. due to broken wire)
    boolean signalTimedOut(void);

    const float getFilterQuality(void) const
    {
      return filterQuality;
    }

    const bool isInside(void) const
    {
      return inside;
    }

    void printInfo(Stream &s);

    Pid pid;             // perimeter PID controller
    unsigned int timedOutIfBelowSmag { 300 };
    unsigned int timeOutSecIfNotInside { 8 };
    // use differential perimeter signal as input for the matched filter?
    bool useDifferentialPerimeterSignal { true };
    // swap coil polarity?
    bool swapCoilPolarity { false };

  private:
    uint32_t lastInsideTime {};
    uint8_t idxPin {}; // channel for idx
    int16_t mag {}; // perimeter magnitude per channel
    float smoothMag {};
    float filterQuality {};
    int8_t signalMin { INT8_MAX };
    int8_t signalMax { INT8_MIN };
    int8_t signalAvg {};
    int8_t signalCounter {};
    uint8_t subSample {};bool inside { true };

    int16_t sumMaxTmp {};
    int16_t sumMinTmp {};

    void calcStatistics(const int8_t* const samples_p);
    void matchedFilter(void);
    int16_t corrFilter(const int8_t* H_p,
                       const uint8_t subsample,
                       const uint8_t M,
                       const uint8_t Hsum,
                       const int8_t* ip_p,
                       const uint8_t nPts,
                       float &quality,
                       bool print);
};

class Perimeters
{
  public:
    Perimeter perimeter[Perimeter::END];
    bool use { false };

    bool isTimeToControl(void);
    void printInfo(Stream &s);

  private:
    static const uint8_t timeBetweenControl { 100 };
    uint32_t nextTimeControl {};
};

#endif
