/*
 * StateMachine.cpp
 *
 *  Created on: Sep 3, 2016
 *      Author: ove
 */

#include "StateMachine.h"

constexpr const char* const StateMachine::stateNames[];

const char* StateMachine::getCurrentStateName() const
{
  return stateNames[m_stateCurr];
}

void StateMachine::init()
{
  m_stateStartTime = millis();
}

void StateMachine::changeState()
{
  m_stateStartTime = millis();
  m_stateLast = m_stateCurr;
  m_stateCurr = m_stateNext;
}

unsigned long StateMachine::getStateTime() const
{
  return millis() - m_stateStartTime;
}

bool StateMachine::isStateEndTimeReached() const
{
  return millis() >= m_stateEndTime;
}
