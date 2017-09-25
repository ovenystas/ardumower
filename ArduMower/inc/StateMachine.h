/*
 * StateMachine.h
 *
 *  Created on: Sep 3, 2016
 *      Author: ove
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

class StateMachine
{
  public:
    typedef enum stateE
    {
      STATE_OFF,              // off
      STATE_REMOTE,           // model remote control (R/C)
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
    } stateE;

    const char* getCurrentStateName();

    void init();

    unsigned long getStateStartTime() const
    {
      return stateStartTime;
    }

    bool isCurrentState(stateE state);

    stateE getCurrentState() const
    {
      return stateCurr;
    }

    unsigned long getEndTime() const
    {
      return stateEndTime;
    }

    void setEndTime(unsigned long endTime)
    {
      this->stateEndTime = endTime;
    }

    void changeState();

    void setNextState(stateE state)
    {
      this->stateNext = state;
    }

    unsigned long getStateTime();

    bool isStateEndTimeReached();

  private:
    static const constexpr char* const stateNames[] =
    {
      "OFF ", "RC  ", "FORW", "ROLL", "REV ", "CIRC", "ERR ",
      "PFND", "PTRK", "PROL", "PREV", "STAT", "CHARG", "STCHK",
      "STREV", "STROL", "STFOR", "MANU", "ROLW", "POUTFOR",
      "POUTREV", "POUTROLL"
    };

    stateE stateCurr { STATE_OFF };
    stateE stateLast { STATE_OFF };
    stateE stateNext { STATE_OFF };
    unsigned long stateTime {};
    unsigned long stateStartTime;
    unsigned long stateEndTime;

};

#endif /* STATEMACHINE_H_ */
