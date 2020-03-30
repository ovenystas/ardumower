/*
 * Bumper.cpp
 *
 *  Created on: Apr 19, 2016
 *      Author: ove
 */

#include "Bumper.h"

void Bumper::check()
{
  if (digitalRead(m_pin) == LOW)
  {
    m_hit = true;
    m_counter++;
  }
}

void Bumpers::setup(const uint8_t* pins, Bumper* bumperArray_p, uint8_t len)
{
  m_use = false;
  m_len = len;
  m_bumperArray_p = bumperArray_p;
  for (uint8_t i = 0; i < len; i++)
  {
    m_bumperArray_p[i].setup(pins[i]);
  }
}

void Bumpers::check()
{
  for (uint8_t i = 0; i < m_len; i++)
  {
    m_bumperArray_p[i].check();
  }
}

void Bumpers::clearHit()
{
  for (uint8_t i = 0; i < m_len; i++)
  {
    m_bumperArray_p[i].clearHit();
  }
}

bool Bumpers::isAnyHit()
{
  for (uint8_t i = 0; i < m_len; i++)
  {
    if (m_bumperArray_p[i].isHit())
    {
      return true;
    }
  }
  return false;
}
