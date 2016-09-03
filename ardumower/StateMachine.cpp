/*
 * StateMachine.cpp
 *
 *  Created on: Sep 3, 2016
 *      Author: ove
 */

#include "StateMachine.h"
#include <Arduino.h>

constexpr const char* const StateMachine::stateNames[];

const char* StateMachine::getCurrentStateName()
{
  return stateNames[stateCurr];
}

bool StateMachine::isCurrentState(stateE state)
{
  return stateCurr == state;
}

void StateMachine::init()
{
  stateStartTime = millis();
}

void StateMachine::changeState()
{
  stateStartTime = millis();
  stateLast = stateCurr;
  stateCurr = stateNext;
}

unsigned long StateMachine::getStateTime()
{
  return millis() - stateStartTime;
}

bool StateMachine::isStateEndTimeReached()
{
  return millis() >= stateEndTime;
}
