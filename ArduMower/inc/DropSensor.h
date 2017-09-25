#ifndef DROP_SENSOR_H
#define DROP_SENSOR_H

#include <Arduino.h>

typedef enum
{
  DROPSENSOR_NO = 0,
  DROPSENSOR_NC = 1
} dropSensorContactE;

typedef struct
{
  uint8_t pin;
  boolean detected;
  uint16_t counter;
} DropSensor;

typedef struct
{
  bool use;
  dropSensorContactE contactType; // contact 1=NC 0=NO against GND
  unsigned long lastRun;
  uint8_t len;
  DropSensor* dropSensorArray_p;
} DropSensors;

void dropSensor_setup(const uint8_t pin, DropSensor* dropSensor_p);
void dropSensor_check(DropSensor* dropSensor_p, dropSensorContactE contactType);

static inline
void dropSensor_simDetected(DropSensor* dropSensor_p)
{
  dropSensor_p->detected = true;
  dropSensor_p->counter++;
}

static inline
bool dropSensor_isDetected(const DropSensor* dropSensor_p)
{
  return dropSensor_p->detected;
}

static inline
void dropSensor_clearDetected(DropSensor* dropSensor_p)
{
  dropSensor_p->detected = false;
}

static inline
uint16_t dropSensor_getCounter(const DropSensor* dropSensor_p)
{
  return dropSensor_p->counter;
}

static inline
void dropSensor_clearCounter(DropSensor* dropSensor_p)
{
  dropSensor_p->counter = 0;
}

static inline
void dropSensors_setup(const uint8_t* pins,
                       const dropSensorContactE contactType,
                       DropSensor* dropSensorArray_p,
                       DropSensors* dropSensors_p,
                       const uint8_t len)
{
  dropSensors_p->use = false;
  dropSensors_p->contactType = contactType;
  dropSensors_p->len = len;
  dropSensors_p->lastRun = 0;
  dropSensors_p->dropSensorArray_p = dropSensorArray_p;
  for (uint8_t i = 0; i < len; i++)
  {
    dropSensor_setup(pins[i], &dropSensorArray_p[i]);
  }
}

static inline
uint8_t dropSensors_getContactType(const DropSensors* dropSensors_p)
{
  return dropSensors_p->contactType;
}

static inline
void dropSensors_check(DropSensors* dropSensors_p)
{
  dropSensors_p->lastRun = millis();
  for (uint8_t i = 0; i < dropSensors_p->len; i++)
  {
    dropSensor_check(&dropSensors_p->dropSensorArray_p[i],
                     dropSensors_p->contactType);
  }
}

static inline
void dropSensors_clearDetected(DropSensors* dropSensors_p)
{
  for (uint8_t i = 0; i < dropSensors_p->len; i++)
  {
    dropSensor_clearDetected(&dropSensors_p->dropSensorArray_p[i]);
  }
}

#endif /* DROP_SENSOR_H */
