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

// developer test to be activated in mower.cpp:
#ifdef USE_DEVELOPER_TEST
// more motor driver friendly signal (receiver)
const int8_t sigcode_norm[] = {
  1, -1, 0, 0, 0, 1, -1, 0, 0, 0,
  -1, 1, 0, 0, 0, 1, -1, 0, 0, 0
};
#else
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal
// if using reconstructed sender signal, use this
const int8_t sigcode_norm[] = {
  1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1,
  1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1
};
// "pseudonoise4_pw" signal (differential)
// if using the coil differential signal, use this
const int8_t sigcode_diff[] = {
  1, 0, -1, 0, 1, -1, 1, -1, 0, 1, -1, 1,
  0, -1, 0, 1, -1, 0, 1, -1, 0, 1, 0, -1
};
#endif

Perimeter::Perimeter()
{
  useDifferentialPerimeterSignal = true;
  swapCoilPolarity = false;
  timedOutIfBelowSmag = 300;
  timeOutSecIfNotInside = 8;
  callCounter = 0;
  mag[0] = 0;
  mag[1] = 0;
  smoothMag[0] = 0;
  smoothMag[1] = 0;
  filterQuality[0] = 0;
  filterQuality[1] = 0;
  signalCounter[0] = 0;
  signalCounter[1] = 0;
  lastInsideTime[0] = 0;
  lastInsideTime[1] = 0;
  subSample = 0;
}

void Perimeter::setup(const byte idx0Pin, const byte idx1Pin)
{
  pinMode(idx0Pin, INPUT);
  pinMode(idx1Pin, INPUT);

  idxPin[0] = idx0Pin;
  idxPin[1] = idx1Pin;

  switch (ADCMan.sampleRate)
  {
    case SRATE_9615:
      subSample = 1;
      break;
    case SRATE_19231:
      subSample = 2;
      break;
    case SRATE_38462:
      subSample = 4;
      break;
  }

  // use max. 255 samples and multiple of signalsize
  int adcSampleCount = sizeof sigcode_norm * subSample;
  ADCMan.setCapture(idx0Pin,
                    (255 / adcSampleCount) * adcSampleCount,
                    true);
  ADCMan.setCapture(idx1Pin,
                    (255 / adcSampleCount) * adcSampleCount,
                    true);
  // ADCMan.setCapture(idx0Pin, adcSampleCount*2, true);
  // ADCMan.setCapture(idx1Pin, adcSampleCount*2, true);

  Console.print(F("matchSignal size="));
  Console.println(sizeof sigcode_norm);
  Console.print(F("subSample="));
  Console.println((int) subSample);
  Console.print(F("capture size="));
  Console.println(ADCMan.getCaptureSize(idx0Pin));
}

void Perimeter::speedTest()
{
  uint16_t loops = 0;
  unsigned long endTime = millis() + 1000;
  while (millis() < endTime)
  {
    matchedFilter(0);
    loops++;
  }
  Console.print(F("speedTest="));
  Console.println(loops);
}

int Perimeter::getMagnitude(const byte idx)
{
  if (ADCMan.isCaptureComplete(idxPin[idx]))
  {
    matchedFilter(idx);
  }
  return mag[idx];
}

int Perimeter::getSmoothMagnitude(const byte idx)
{
  return smoothMag[idx];
}

void Perimeter::printADCMinMax(const int8_t* samples_p)
{
  int8_t vmax = SCHAR_MIN;
  int8_t vmin = SCHAR_MAX;
  for (byte i = 0; i < ADCMan.getCaptureSize(idxPin[0]); i++)
  {
    vmax = max(vmax, samples_p[i]);
    vmin = min(vmin, samples_p[i]);
  }
  Console.print(F("perimter min,max="));
  Console.print((int) vmin);
  Console.print(F(","));
  Console.println((int) vmax);
}

// perimeter V2 uses a digital matched filter
void Perimeter::matchedFilter(const byte idx)
{
  int16_t sampleCount = ADCMan.getCaptureSize(idxPin[0]);
  int8_t* samples_p = ADCMan.getCapture(idxPin[idx]);
  if (callCounter == 100)
  {
    // statistics only
    callCounter = 0;
    signalMin[idx] = 9999;
    signalMax[idx] = -9999;
    signalAvg[idx] = 0;

    for (int i = 0; i < sampleCount; i++)
    {
      int8_t v = samples_p[i];
      signalAvg[idx] += v;
      signalMin[idx] = min(signalMin[idx], v);
      signalMax[idx] = max(signalMax[idx], v);
    }
    signalAvg[idx] = (double)signalAvg[idx] / (double)sampleCount;
  }

  // magnitude for tracking (fast but inaccurate)
  const int8_t* sigcode_p;
  if (useDifferentialPerimeterSignal)
  {
    sigcode_p = sigcode_diff;
  }
  else
  {
    sigcode_p = sigcode_norm;
  }
  int16_t sigcode_size = sizeof(*sigcode_p);
  mag[idx] = corrFilter(sigcode_p,
                        subSample,
                        sigcode_size,
                        samples_p,
                        sampleCount - sigcode_size * subSample,
                        filterQuality[idx]);
  if (swapCoilPolarity)
  {
    mag[idx] = -mag[idx];
  }
  // smoothed magnitude used for signal-off detection
  smoothMag[idx] = 0.99 * smoothMag[idx] + 0.01 * (float)abs(mag[idx]);

  // perimeter inside/outside detection
  if (mag[idx] > 0)
  {
    signalCounter[idx] = min(signalCounter[idx] + 1, 3);
  }
  else
  {
    signalCounter[idx] = max(signalCounter[idx] - 1, -3);
  }
  if (signalCounter[idx] < 0)
  {
    lastInsideTime[idx] = millis();
  }

  ADCMan.restart(idxPin[idx]);
  if (idx == 0)
  {
    callCounter++;
  }
}

int16_t Perimeter::getSignalMin(const byte idx)
{
  return signalMin[idx];
}

int16_t Perimeter::getSignalMax(const byte idx)
{
  return signalMax[idx];
}

int16_t Perimeter::getSignalAvg(const byte idx)
{
  return signalAvg[idx];
}

float Perimeter::getFilterQuality(const byte idx)
{
  return filterQuality[idx];
}

boolean Perimeter::isInside(const byte idx)
{
  return (signalCounter[idx] < 0);
}

boolean Perimeter::signalTimedOut(const byte idx)
{
  if (getSmoothMagnitude(idx) < timedOutIfBelowSmag)
  {
    return true;
  }

  if (millis() - lastInsideTime[idx] > timeOutSecIfNotInside * 1000)
  {
    return true;
  }

  return false;
}

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs, M = H.length (number of points in FIR)
// subsample is the number of times for each filter coeff to repeat
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data

int16_t Perimeter::corrFilter(const int8_t* H_p, const int8_t subsample,
                              const int16_t M, const int8_t* ip_p,
                              const int16_t nPts, float &quality)
{
  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  int16_t Ms = M * subsample; // number of filter coeffs including subsampling

  // compute sum of absolute filter coeffs
  int16_t Hsum = 0;
  for (int16_t i = 0; i < M; i++)
  {
    Hsum += abs(H_p[i]);
  }
  Hsum *= subsample;

  // compute correlation
  // for each input value
  for (int16_t j = 0; j < nPts; j++)
  {
    int16_t sum = 0;
    const int8_t* Hi_p = H_p;
    int8_t ss = 0;
    const int8_t* ipi_p = ip_p;
    // for each filter coeffs
    for (int16_t i = 0; i < Ms; i++)
    {
      sum += ((int16_t)(*Hi_p)) * ((int16_t)(*ipi_p));
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
  // normalize to 4095
  sumMin = ((float)sumMin) / ((float)(Hsum * 127)) * 4095.0;
  sumMax = ((float)sumMax) / ((float)(Hsum * 127)) * 4095.0;

  // compute ratio min/max
  if (sumMax > -sumMin)
  {
    quality = ((float)sumMax) / ((float)-sumMin);
    return sumMax;
  }
  else
  {
    quality = ((float)-sumMin) / ((float)sumMax);
    return sumMin;
  }
}

bool Perimeter::isTimeToControl(void)
{
  unsigned long curMillis = millis();
  if (curMillis >= nextTimeControl)
  {
    nextTimeControl = curMillis + timeBetweenControl;
    return true;
  }
  return false;
}
