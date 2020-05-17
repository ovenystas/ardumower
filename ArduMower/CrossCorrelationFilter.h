/*
 * CrossCorrelationFilter.h
 *
 *  Created on: May 17, 2020
 *      Author: ove
 */

#pragma once

#include "Arduino.h"

class CrossCorrelationFilter
{
public:
  CrossCorrelationFilter(Stream& outStream) :
    m_outStream(outStream) {};

  int16_t update(
      const int8_t* H_p,
      const uint8_t subsample,
      const uint8_t M,
      const uint8_t Hsum,
      const int8_t* ip_p,
      const uint8_t nPts,
      float &quality,
      bool print);

private:
  Stream& m_outStream;
  int16_t m_sumMaxTmp {};
  int16_t m_sumMinTmp {};
};


