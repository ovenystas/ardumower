/*
 * encoder.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#include "Encoder.h"

// Local function prototypes --------------------------------------------------


// Public functions -----------------------------------------------------------

void Encoder::setup(uint8_t pin, bool swapDir)
{
  m_pin = pin;
  m_swapDir = swapDir;

  pinMode(m_pin, INPUT_PULLUP);
}

void Encoder::read()
{
  m_curState = digitalRead(m_pin);

  int16_t step = m_swapDir ? -1 : 1;
  if (m_curState != m_lastState)
  {
    if (m_wheelRpmCurr >= 0)
    {
      m_counter = (int16_t)(m_counter + step);
    }
    else
    {
      m_counter = (int16_t)(m_counter - step);
    }
    m_lastState = m_curState;
  }
}

int16_t Encoder::getCounter()
{
  int16_t counter;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    counter = m_counter;
  }
  return counter;
}

void Encoder::clearCounter()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    m_counter = 0;
  }
}

int16_t Encoder::getWheelRpmCurr()
{
  int16_t rpm;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    rpm = m_wheelRpmCurr;
  }
  return rpm;
}

void Encoder::setWheelRpmCurr(int16_t wheelRpmCurr)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    m_wheelRpmCurr = wheelRpmCurr;
  }
}

// Private functions ----------------------------------------------------------
