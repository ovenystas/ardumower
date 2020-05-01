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

void Bumper::simHit()
{
  m_hit = true;
  m_counter++;
}

void Bumpers::check() const
{
  for (uint8_t i = 0; i < m_len; i++)
  {
    m_bumperArray_p[i].check();
  }
}

void Bumpers::clearHit() const
{
  for (uint8_t i = 0; i < m_len; i++)
  {
    m_bumperArray_p[i].clearHit();
  }
}

bool Bumpers::isAnyHit() const
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
