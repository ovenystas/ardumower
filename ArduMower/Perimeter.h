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

typedef enum
{
  PERIMETER_LEFT,
  PERIMETER_RIGHT,
  PERIMETER_END
} perimeterE;

typedef struct
{
  Pid pid;             // perimeter PID controller
  unsigned int timedOutIfBelowSmag { 300 };
  unsigned int timeOutSecIfNotInside { 8 };
  // use differential perimeter signal as input for the matched filter?
  bool useDifferentialPerimeterSignal { true };
  // swap coil polarity?
  bool swapCoilPolarity { false };

  uint32_t lastInsideTime;
  uint8_t idxPin; // channel for idx
  int16_t mag; // perimeter magnitude per channel
  float smoothMag;
  float filterQuality;
  int8_t signalMin { INT8_MAX };
  int8_t signalMax { INT8_MIN };
  int8_t signalAvg;
  int8_t signalCounter;
  uint8_t subSample;
  bool inside { true };

  int16_t sumMaxTmp;
  int16_t sumMinTmp;
} Perimeter;

typedef struct
{
  bool use { false };
  Perimeter* perimeterArray_p;
} Perimeters;

// set ADC pins
void perimeter_setup(const uint8_t idxPin, Perimeter* perimeter_p);

// get perimeter magnitude
int16_t perimeter_calcMagnitude(Perimeter* perimeter_p);

static inline
int16_t perimeter_getMagnitude(Perimeter* perimeter_p)
{
  return perimeter_p->mag;
}

static inline
int16_t perimeter_getSmoothMagnitude(Perimeter* perimeter_p)
{
  return (int16_t)perimeter_p->smoothMag;
}

// inside perimeter (true) or outside (false)?
// perimeter signal timed out? (e.g. due to broken wire)
bool perimeter_signalTimedOut(Perimeter* perimeter_p);

static inline
float perimeter_getFilterQuality(Perimeter* perimeter_p)
{
  return perimeter_p->filterQuality;
}

static inline
bool perimeter_isInside(Perimeter* perimeter_p)
{
  return perimeter_p->inside;
}

void perimeter_printInfo(Stream &s, Perimeter* perimeter_p);

void perimeter_calcStatistics(
    const int8_t* const samples_p,
    Perimeter* perimeter_p);

void perimeter_matchedFilter(Perimeter* perimeter_p);

int16_t perimeter_corrFilter(
    const int8_t* H_p,
    const uint8_t subsample,
    const uint8_t M,
    const uint8_t Hsum,
    const int8_t* ip_p,
    const uint8_t nPts,
    float &quality,
    bool print,
    Perimeter* perimeter_p);




void perimeters_printInfo(Stream &s, Perimeters* perimeters_p);

#endif /* PERIMETER_H */
