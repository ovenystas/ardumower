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

#include "Perimeter.h"

#include <Arduino.h>
#include <limits.h>

#include "AdcManager.h"
#include "drivers.h"
#include "Config.h"

// developer test to be activated in mower.cpp:
#if PERIMETER_USE_DEVELOPER_TEST
// more motor driver friendly signal (receiver)
const int8_t sigcode_norm[] =
{
  1, -1, 0, 0, 0, 1, -1, 0, 0, 0,
  -1, 1, 0, 0, 0, 1, -1, 0, 0, 0
};
#else
#if !PERIMETER_USE_BARKER_CODE
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal
// if using reconstructed sender signal, use this
const int8_t sigcode_norm[] =
{
  1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1,
  1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1
};
// "pseudonoise4_pw" signal (differential)
// if using the coil differential signal, use this
const int8_t sigcode_diff[] =
{
  1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1,
  0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1
};
#else
const int8_t sigcode_norm[]  = { 1,  1,  1, -1, -1, -1,  1, -1, -1,  1, -1 };
const uint8_t hSum_norm = 44;
//const int8_t sigcode_diff[]  = { 1,  0,  0, -1,  0,  0,  1, -1,  0,  1, -1 };
const int8_t sigcode_diff[] = { 1,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0,  1,  0, -1,  0,  0,  0,  1,  0, -1,  0 };
const uint8_t hSum_diff = 12;
#endif
#endif

void perimeter_setup(const uint8_t idxPin, Perimeter* perimeter_p)
{
  perimeter_p->idxPin = idxPin;

  pinMode(perimeter_p->idxPin, INPUT);

  switch (ADCMan.getSampleRate())
  {
    case SRATE_9615:
      perimeter_p->subSample = 1;
      break;
    case SRATE_19231:
      perimeter_p->subSample = 2;
      break;
    case SRATE_38462:
      perimeter_p->subSample = 4;
      break;
  }

  // Use max. 255 samples and multiple of signal size
  unsigned int adcSampleCount { sizeof(sigcode_norm) * perimeter_p->subSample };
  uint8_t samplecount = (255 / adcSampleCount) * adcSampleCount;

  ADCMan.setCapture(perimeter_p->idxPin, samplecount, true);

  Console.print(F("matchSignal size="));
  Console.print(sizeof(sigcode_norm));
  Console.print(F(" subSample="));
  Console.print(perimeter_p->subSample);
  Console.print(F(" samplecount="));
  Console.print(samplecount);
  Console.print(F(" capture size="));
  Console.println(ADCMan.getCaptureSize(perimeter_p->idxPin));
}

int16_t perimeter_calcMagnitude(Perimeter* perimeter_p)
{
  if (ADCMan.isCaptureComplete(perimeter_p->idxPin))
  {
    perimeter_matchedFilter(perimeter_p);
  }
  return perimeter_p->mag;
}

void perimeter_calcStatistics(
    const int8_t* const samples_p,
    Perimeter* perimeter_p)
{
  int8_t vMin = INT8_MAX;
  int8_t vMax = INT8_MIN;
  int16_t sum = 0;
  uint8_t numSamples = ADCMan.getCaptureSize(perimeter_p->idxPin);
  for (uint8_t i = 0; i < numSamples; i++)
  {
    const int8_t value = samples_p[i];
    sum += value;
    vMin = min(vMin, value);
    vMax = max(vMax, value);
  }
  perimeter_p->signalMin = vMin;
  perimeter_p->signalMax = vMax;
  perimeter_p->signalAvg = (int8_t)(sum / numSamples);
}

// perimeter V2 uses a digital matched filter
void perimeter_matchedFilter(Perimeter* perimeter_p)
{
  static uint8_t callCounter = 0;
  static uint32_t callCounter2 = 0;
  const uint8_t numSamples = ADCMan.getCaptureSize(perimeter_p->idxPin);
  const int8_t* const samples_p = ADCMan.getCapture(perimeter_p->idxPin);

  // Calculate some statistics every 100 th's call
  if (callCounter == 100)
  {
    callCounter = 0;
    perimeter_calcStatistics(samples_p, perimeter_p);
  }

  // magnitude for tracking (fast but inaccurate)
  const int8_t* sigcode_p;
  uint8_t sigcode_size;
  uint8_t hSum;
  if (perimeter_p->useDifferentialPerimeterSignal)
  {
    sigcode_p = sigcode_diff;
    sigcode_size = sizeof(sigcode_diff);
    hSum = hSum_diff;
  }
  else
  {
    sigcode_p = sigcode_norm;
    sigcode_size = sizeof(sigcode_norm);
    hSum = hSum_norm;
  }
  bool print { false };
  if (callCounter2 >= 200UL)
  {
    callCounter2 = 0;
    //print = true;
  }
  perimeter_p->mag =
      perimeter_corrFilter(
          sigcode_p,
          perimeter_p->subSample / 2,
          sigcode_size,
          hSum,
          samples_p,
          numSamples - sigcode_size * perimeter_p->subSample / 2,
          perimeter_p->filterQuality,
          print,
          perimeter_p);

  if (perimeter_p->swapCoilPolarity)
  {
    perimeter_p->mag = -perimeter_p->mag;
  }

  // smoothed magnitude used for signal-off detection
  perimeter_p->smoothMag = 0.99 * perimeter_p->smoothMag + 0.01 * (float)abs(perimeter_p->mag);

  // perimeter inside/outside detection
  if (perimeter_p->mag > 0)
  {
    perimeter_p->signalCounter = min(perimeter_p->signalCounter + 1, 3);
  }
  else
  {
    perimeter_p->signalCounter = max(perimeter_p->signalCounter - 1, -3);
  }

  if (perimeter_p->signalCounter < 0)
  {
    perimeter_p->inside = true;
    perimeter_p->lastInsideTime = millis();
  }
  else
  {
    perimeter_p->inside = false;
  }

  ADCMan.restart(perimeter_p->idxPin);
  callCounter++;
  callCounter2++;
}

bool perimeter_signalTimedOut(Perimeter* perimeter_p)
{
  if ((uint16_t)perimeter_p->smoothMag < perimeter_p->timedOutIfBelowSmag)
  {
    return true;
  }

  if (millis() - perimeter_p->lastInsideTime > perimeter_p->timeOutSecIfNotInside * 1000)
  {
    return true;
  }

  return false;
}

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs
// subsample is the number of times for each filter coeff to repeat
// M = H.length (number of points in FIR)
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data

int16_t perimeter_corrFilter(
    const int8_t* H_p,
    const uint8_t subsample,
    const uint8_t M,
    const uint8_t Hsum,
    const int8_t* ip_p,
    const uint8_t nPts,
    float &quality,
    bool print,
    Perimeter* perimeter_p)
{
  if (print)
  {
    Console.print(F("cF: H="));
    for (uint8_t i = 0; i < M; i++)
    {
      Console.print(H_p[i]);
      Console.print(",");
    }
    Console.print(F(" subsample="));
    Console.print(subsample);
    Console.print(F(" M="));
    Console.print(M);
    Console.print(F(" ip="));
    for (uint8_t i = 0; i < nPts; i++)
    {
      Console.print(ip_p[i]);
      Console.print(",");
    }
    Console.print(F(" nPts="));
    Console.print(nPts);
  }

  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  uint8_t Ms = M * subsample; // number of filter coeffs including subsampling
  if (print)
  {
    Console.print(F(" Ms="));
    Console.print(Ms);
    Console.print(F(" Hsum="));
    Console.print(Hsum);
  }

  // compute correlation
  // for each input value
  for (uint8_t j = 0; j < nPts; j++)
  {
    int16_t sum = 0;
    int8_t* Hi_p = (int8_t*)H_p;
    uint8_t ss = 0;
    int8_t* ipi_p = (int8_t*)ip_p;

    // for each filter coeffs
    for (uint8_t i = 0; i < Ms; i++)
    {
      sum += ((int16_t)*Hi_p) * ((int16_t)*ipi_p);
      ss++;
      if (ss == subsample)
      {
        ss = 0;
        Hi_p++; // next filter coeffs
      }
      ipi_p++;
    }
    if (sum > sumMax)
    {
      sumMax = sum;
    }
    if (sum < sumMin)
    {
      sumMin = sum;
    }
    ip_p++;
  }
  perimeter_p->sumMaxTmp = sumMax;
  perimeter_p->sumMinTmp = sumMin;
  if (print)
  {
    Console.print(F(" sumMax="));
    Console.print(sumMax);
    Console.print(F(" sumMin="));
    Console.print(sumMin);
  }

  // normalize to 4095
  sumMin = (float)sumMin / (float)((uint16_t)Hsum * 127) * 4095.0;
  sumMax = (float)sumMax / (float)((uint16_t)Hsum * 127) * 4095.0;
  if (print)
  {
    Console.print(F(" sumMaxNorm="));
    Console.print(sumMax);
    Console.print(F(" sumMinNorm="));
    Console.print(sumMin);
  }

  // compute ratio min/max
  if (sumMax > -sumMin)
  {
    quality = (float)sumMax / (float)-sumMin;
    if (print)
    {
      Console.print(F(" quality="));
      Console.print((int16_t)(quality * 100));
      Console.print(F(" mag="));
      Console.println(sumMax);
    }
    return sumMax;
  }
  else
  {
    quality = (float)-sumMin / (float)sumMax;
    if (print)
    {
      Console.print(F(" quality="));
      Console.print((int16_t)(quality * 100));
      Console.print(F(" mag="));
      Console.println(sumMin);
    }
    return sumMin;
  }
}

void perimeter_printInfo(Stream &s, Perimeter* perimeter_p)
{
  Streamprint(s, "sig min %-4d max %-4d avg %-4d",
              perimeter_p->signalMin, perimeter_p->signalMax, perimeter_p->signalAvg);

  Streamprint(s, " mag %-5d smag %-4d qty %-3d",
              perimeter_p->mag, (int16_t )perimeter_p->smoothMag, (int8_t)(perimeter_p->filterQuality * 100.0));

  Streamprint(s, " (sum min %-6d max %-6d)",
              perimeter_p->sumMinTmp, perimeter_p->sumMaxTmp);
}

//-----------------------------------------------------------------------------

void perimeters_printInfo(Stream &s, Perimeters* perimeters_p)
{
//  Streamprint(s, "perL: ");
  perimeter_printInfo(s, &perimeters_p->perimeterArray_p[PERIMETER_LEFT]);
//  Streamprint(s, "perR: ");
//  perimeter_printInfo(s, &perimeters_p->perimeterArray_p[PERIMETER_RIGHT]);

}
