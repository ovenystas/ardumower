/*
 * CrossCorrelationFilter.cpp
 *
 *  Created on: May 17, 2020
 *      Author: ove
 */

#include "CrossCorrelationFilter.h"

// digital matched filter (cross correlation)
// http://en.wikipedia.org/wiki/Cross-correlation
// H[] holds the double sided filter coeffs
// subsample is the number of times for each filter coeff to repeat
// M = H.length (number of points in FIR)
// ip[] holds input data (length > nPts + M )
// nPts is the length of the required output data

int16_t CrossCorrelationFilter::update(
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
    m_outStream.print(F("cF: H="));

    for (uint8_t i = 0; i < M; i++)
    {
      m_outStream.print(H_p[i]);
      m_outStream.print(",");
    }

    m_outStream.print(F(" subsample="));
    m_outStream.print(subsample);
    m_outStream.print(F(" M="));
    m_outStream.print(M);
    m_outStream.print(F(" ip="));

    for (uint8_t i = 0; i < nPts; i++)
    {
      m_outStream.print(ip_p[i]);
      m_outStream.print(",");
    }

    m_outStream.print(F(" nPts="));
    m_outStream.print(nPts);
  }

  int16_t sumMax = 0; // max correlation sum
  int16_t sumMin = 0; // min correlation sum
  uint8_t Ms = static_cast<uint8_t>(M * subsample); // number of filter coeffs including subsampling

  if (print)
  {
    m_outStream.print(F(" Ms="));
    m_outStream.print(Ms);
    m_outStream.print(F(" Hsum="));
    m_outStream.print(Hsum);
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
      sum = static_cast<int16_t>(sum +
          static_cast<int16_t>(*Hi_p) *
          static_cast<int16_t>(*ipi_p));

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
    m_outStream.print(F(" sumMax="));
    m_outStream.print(sumMax);
    m_outStream.print(F(" sumMin="));
    m_outStream.print(sumMin);
  }

  // normalize to 4095
  float tmp = static_cast<float>(static_cast<uint16_t>(Hsum) * 127) / 4095.0f;
  sumMin = static_cast<int16_t>(static_cast<float>(sumMin) / tmp);
  sumMax = static_cast<int16_t>(static_cast<float>(sumMax) / tmp);

  if (print)
  {
    m_outStream.print(F(" sumMaxNorm="));
    m_outStream.print(sumMax);
    m_outStream.print(F(" sumMinNorm="));
    m_outStream.print(sumMin);
  }

  // compute ratio min/max
  if (sumMax > -sumMin)
  {
    quality = (float)sumMax / (float)-sumMin;

    if (print)
    {
      m_outStream.print(F(" quality="));
      m_outStream.print((int16_t)(quality * 100));
      m_outStream.print(F(" mag="));
      m_outStream.println(sumMax);
    }

    return sumMax;
  }
  else
  {
    quality = static_cast<float>(-sumMin) / static_cast<float>(sumMax);

    if (print)
    {
      m_outStream.print(F(" quality="));
      m_outStream.print(static_cast<int16_t>(quality * 100));
      m_outStream.print(F(" mag="));
      m_outStream.println(sumMin);
    }

    return sumMin;
  }
}
