/*
 * odometer.h
 *
 *  Created on: Apr 4, 2016
 *      Author: ove
 */

#ifndef ODOMETER_H
#define ODOMETER_H

#include "Encoder.h"
#include "Imu.h"

class Odometer
{
  public:
    enum odometerE
    {
      LEFT,
      RIGHT,
      END
    };

    void setup(const int ticksPerRevolution,
               const float ticksPerCm,
               const float wheelBaseCm,
               const uint8_t pin[2],
               const uint8_t pin2[2],
               const bool swapDir[2]);
    void read();
    void setState(unsigned long timeMicros);
    void calc(const Imu &imu);

    float getX() const
    {
      return x;
    }

    float getY() const
    {
      return y;
    }

    bool use { false };      // Use this odometer or not
    int ticksPerRevolution;  // encoder ticks per one full resolution
    float ticksPerCm;        // encoder ticks per cm
    float wheelBaseCm;       // wheel-to-wheel distance (cm)
    Encoder encoder[END];

  private:
    unsigned long nextTime {};          // when to trigger next time
    unsigned long lastWheelRpmTime {};  // last time it was updated
    float theta {};                     // theta angle (radiant)
    float x {};                         // X map position (cm)
    float y {};                         // Y map position (cm)
};

#endif /* ODOMETER_H */
