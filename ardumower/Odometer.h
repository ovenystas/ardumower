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

typedef struct encoderS
{
  Encoder* left_p { nullptr };
  Encoder* right_p { nullptr };
} encoderS;

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
               Encoder* encoderLeft,
               Encoder* encoderRight,
               Imu* imu);
    void loop(void);
    void readAndSetState(void);

    const float getX() const
    {
      return x;
    }

    const float getY() const
    {
      return y;
    }

    bool use { false };      // Use this odometer or not
    int ticksPerRevolution;  // encoder ticks per one full resolution
    float ticksPerCm;        // encoder ticks per cm
    float wheelBaseCm;       // wheel-to-wheel distance (cm)
    encoderS encoder;
    Imu* imu_p;

  private:
    const int16_t TIME_BETWEEN_CALCS { 300 }; // (ms)

    unsigned long nextTime {};          // when to trigger next time
    unsigned long lastWheelRpmTime {};  // last time it was updated
    int16_t lastOdoLeft {};
    int16_t lastOdoRight {};
    float theta {};                     // theta angle (radiant)
    float x {};                         // X map position (cm)
    float y {};                         // Y map position (cm)

    void calc();
};

#endif /* ODOMETER_H */
