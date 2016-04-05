/*
 * odometer.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#ifndef ODOMETER_H
#define ODOMETER_H

#include "encoder.h"

class Odometer
{
  public:
    enum odometerE
    {
      LEFT,
      RIGHT,
      END
    };

    /**
     * Setup function to use with 1-pin encoders.
     */
    void setup(const int ticksPerRevolution,
               const float ticksPerCm,
               const float wheelBaseCm,
               const uint8_t pin[2],
               const bool swapDir[2]);

    /**
     * Setup function to use with 2-pin encoders.
     */
    void setup(const int ticksPerRevolution,
               const float ticksPerCm,
               const float wheelBaseCm,
               const uint8_t pin[2],
               const uint8_t pin2[2],
               const bool swapDir[2]);

    void read();
    void setState(unsigned long timeMicros);

    int ticksPerRevolution; // encoder ticks per one full resolution
    float ticksPerCm;       // encoder ticks per cm
    float wheelBaseCm;      // wheel-to-wheel distance (cm)
    float theta {};         // theta angle (radiant)
    float x {};             // X map position (cm)
    float y {};             // Y map position (cm)
    Encoder encoder[END];

  private:
    void setOdometerState(unsigned long timeMicros,
                          boolean odometerLeftState,
                          boolean odometerRightState,
                          boolean odometerLeftState2,
                          boolean odometerRightState2);
};

#endif /* ODOMETER_H */
