/*
 * encoder.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder
{
  public:
    void setup(const uint8_t pin, const uint8_t pin2, const bool swapDir);
    void read();
    void setState(unsigned long timeMicros);

    int getCounter() const
    {
      return counter;
    }

    void clearCounter()
    {
      counter = 0;
    }

    float getWheelRpmCurr() const
    {
      return wheelRpmCurr;
    }

    void setWheelRpmCurr(float wheelRpmCurr)
    {
      this->wheelRpmCurr = wheelRpmCurr;
    }

    bool twoWay;                // using 2-pin encoder
    bool swapDir { false };     // inverse encoder direction?

  private:
    uint8_t pin;
    uint8_t pin2;
    boolean curState { LOW };   // current state
    boolean curState2 { LOW };  // current state on 2nd pin
    boolean lastState { LOW };  // last state
    boolean lastState2 { LOW }; // last state on 2nd pin
    int counter {};             // wheel counter
    float wheelRpmCurr {};      // wheel rpm
};

#endif /* ENCODER_H */
