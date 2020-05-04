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

#include <limits.h>

#include "AdcManager.h"
#include "Config.h"
#include "Drivers.h"

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

void Perimeter::setup(const uint8_t idxPin)
{
  m_idxPin = idxPin;

  pinMode(m_idxPin, INPUT);

  switch (ADCMan.getSampleRate())
  {
    case SRATE_9615:
      m_subSample = 1;
      break;
    case SRATE_19231:
      m_subSample = 2;
      break;
    case SRATE_38462:
      m_subSample = 4;
      break;
  }

  // Use max. 255 samples and multiple of signal size
  unsigned int adcSampleCount { sizeof(sigcode_norm) * m_subSample };
  uint8_t samplecount = (255 / adcSampleCount) * adcSampleCount;

  ADCMan.setCapture(m_idxPin, samplecount, true);

  Console.print(F("matchSignal size="));
  Console.print(sizeof(sigcode_norm));
  Console.print(F(" subSample="));
  Console.print(m_subSample);
  Console.print(F(" samplecount="));
  Console.print(samplecount);
  Console.print(F(" capture size="));
  Console.println(ADCMan.getCaptureSize(m_idxPin));
}

int16_t Perimeter::calcMagnitude()
{
  if (ADCMan.isCaptureComplete(m_idxPin))
  {
    matchedFilter();
  }
  return m_mag;
}

void Perimeter::calcStatistics(const int8_t* samples_p)
{
  int8_t vMin = INT8_MAX;
  int8_t vMax = INT8_MIN;
  int16_t sum = 0;
  uint8_t numSamples = ADCMan.getCaptureSize(m_idxPin);
  for (uint8_t i = 0; i < numSamples; i++)
  {
    const int8_t value = samples_p[i];
    sum += value;
    vMin = min(vMin, value);
    vMax = max(vMax, value);
  }
  m_signalMin = vMin;
  m_signalMax = vMax;
  m_signalAvg = (int8_t)(sum / numSamples);
}

// perimeter V2 uses a digital matched filter
void Perimeter::matchedFilter()
{
  static uint8_t callCounter = 0;
  static uint32_t callCounter2 = 0;
  const uint8_t numSamples = ADCMan.getCaptureSize(m_idxPin);
  const int8_t* const samples_p = ADCMan.getCapture(m_idxPin);

  // Calculate some statistics every 100 th's call
  if (callCounter == 100)
  {
    callCounter = 0;
    calcStatistics(samples_p);
  }

  // magnitude for tracking (fast but inaccurate)
  const int8_t* sigcode_p;
  uint8_t sigcode_size;
  uint8_t hSum;
  if (m_useDifferentialPerimeterSignal)
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
  m_mag = corrFilter(
      sigcode_p,
      m_subSample / 2,
      sigcode_size,
      hSum,
      samples_p,
      numSamples - sigcode_size * m_subSample / 2,
      m_filterQuality,
      print);

  if (m_swapCoilPolarity)
  {
    m_mag = -m_mag;
  }

  // smoothed magnitude used for signal-off detection
  m_smoothMag = 0.99 * m_smoothMag + 0.01 * (float)abs(m_mag);

  // perimeter inside/outside detection
  if (m_mag > 0)
  {
    m_signalCounter = min(m_signalCounter + 1, 3);
  }
  else
  {
    m_signalCounter = max(m_signalCounter - 1, -3);
  }

  if (m_signalCounter < 0)
  {
    m_inside = true;
    m_lastInsideTime = millis();
  }
  else
  {
    m_inside = false;
  }

  ADCMan.restart(m_idxPin);
  callCounter++;
  callCounter2++;
}

bool Perimeter::signalTimedOut()
{
  if ((uint16_t)m_smoothMag < m_timedOutIfBelowSmag)
  {
    return true;
  }

  if (millis() - m_lastInsideTime > m_timeOutSecIfNotInside * 1000)
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

int16_t Perimeter::corrFilter(
    const int8_t* H_p,
    const uint8_t subsample,
    const uint8_t M,
    const uint8_t Hsum,
    const int8_t* ip_p,
    const uint8_t nPts,
    float &quality,
    bool print)
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
  m_sumMaxTmp = sumMax;
  m_sumMinTmp = sumMin;
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

void Perimeter::printInfo(Stream &s)
{
  Streamprint(s, "sig min %-4d max %-4d avg %-4d",
              m_signalMin, m_signalMax, m_signalAvg);

  Streamprint(s, " mag %-5d smag %-4d qty %-3d",
              m_mag, (int16_t)m_smoothMag, (int8_t)(m_filterQuality * 100.0));

  Streamprint(s, " (sum min %-6d max %-6d)",
              m_sumMinTmp, m_sumMaxTmp);
}

//-----------------------------------------------------------------------------

void Perimeters::printInfo(Stream &s)
{
//  Streamprint(s, "perL: ");
  m_perimeterArray_p[static_cast<uint8_t>(PerimeterE::LEFT)].printInfo(s);
//  Streamprint(s, "perR: ");
//  perimeterArray_p[static_cast<uint8_t>(PerimeterE::RIGHT)].printInfo(s);
}
