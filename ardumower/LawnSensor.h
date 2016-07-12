/*
 * lawnSensor.h
 *
 *  Created on: Apr 3, 2016
 *      Author: ove
 */

#ifndef LAWNSENSOR_H
#define LAWNSENSOR_H

class LawnSensor
{
  public:
    bool use { false };
    enum lawnSensorE
    {
      FRONT,
      BACK,
      END
    };

    void setup(const uint8_t pinSendFront, const uint8_t pinReceiveFront,
               const uint8_t pinSendBack, const uint8_t pinReceiveBack);

    float getValue(uint8_t index)
    {
      return value[index];
    }

    boolean isDetected(void)
    {
      return detected;
    }

    void clearDetected(void)
     {
       detected = false;
     }

    void simDetected(void);

    uint16_t getCounter(void)
    {
      return counter;
    }

    void read();
    void check();
    bool isTimeToRead();
    bool isTimeToCheck();

  private:
    uint8_t pinSend[2]{};
    uint8_t pinReceive[2]{};
    uint16_t counter{};
    boolean detected{false};
    float value[2]{};     // lawn sensor capacity (time)
    float valueOld[2]{};  // lawn sensor capacity (time)
    unsigned long nextTimeRead {};
    unsigned long nextTimeCheck {};
    unsigned int timeBetweenRead { 100 };
    unsigned int timeBetweenCheck { 2000 };

    uint16_t measureLawnCapacity(const uint8_t pinSend, const uint8_t pinReceive);

};

#endif /* LAWNSENSOR_H */
