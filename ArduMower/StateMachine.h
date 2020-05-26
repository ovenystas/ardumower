/*
 * StateMachine.h
 *
 *  Created on: Sep 3, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>

class StateMachine
{
  public:
    StateMachine() {};

    enum StateE
    {
      STATE_OFF,              // off
      STATE_FORWARD,          // drive forward
      STATE_ROLL,             // drive roll right/left
      STATE_REVERSE,          // drive reverse
      STATE_CIRCLE,           // drive circle
      STATE_ERROR,            // error
      STATE_PERI_FIND,        // perimeter find
      STATE_PERI_TRACK,       // perimeter track
      STATE_PERI_ROLL,        // perimeter roll
      STATE_PERI_REV,         // perimeter reverse
      STATE_STATION,          // in station
      STATE_STATION_CHARGING, // in station charging
      STATE_STATION_CHECK,    // checks if station is present
      STATE_STATION_REV,      // charge reverse
      STATE_STATION_ROLL,     // charge roll
      STATE_STATION_FORW,     // charge forward
      STATE_MANUAL,           // manual navigation
      STATE_ROLL_WAIT,        // drive roll right/left
      STATE_PERI_OUT_FORW,    // outside perimeter forward driving without checkPerimeterBoundary()
      STATE_PERI_OUT_REV,     // outside perimeter reverse driving without checkPerimeterBoundary()
      STATE_PERI_OUT_ROLL,    // outside perimeter rolling driving without checkPerimeterBoundary()
    };

    const char* getCurrentStateName() const;

    void init();

    uint32_t getStateStartTime() const
    {
      return m_stateStartTime;
    }

    bool isCurrentState(StateE state) const
    {
      return m_stateCurr == state;
    }


    StateE getCurrentState() const
    {
      return m_stateCurr;
    }

    void setEndTime(uint32_t endTime)
    {
      m_stateEndTime = endTime;
    }

    void changeState();

    void setNextState(StateE state)
    {
      m_stateNext = state;
    }

    unsigned long getStateTime() const;

    bool isStateEndTimeReached() const;

  private:
    static const constexpr char* const stateNames[] =
    {
      "OFF ", "FORW", "ROLL", "REV ", "CIRC", "ERR ",
      "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK",
      "STREV", "STROL", "STFOR", "MANU", "ROLW", "POUTFOR",
      "POUTREV", "POUTROLL"
    };

    StateE m_stateCurr { STATE_OFF };
    StateE m_stateLast { STATE_OFF };
    StateE m_stateNext { STATE_OFF };
    uint32_t m_stateTime {};
    uint32_t m_stateStartTime {};
    uint32_t m_stateEndTime {};
};
