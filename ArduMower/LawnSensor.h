/*
 * lawnSensor.h
 *
 *  Created on: Apr 3, 2016
 *      Author: ove
 */
#pragma once

#include <Arduino.h>

typedef struct
{
  uint8_t sendPin;
  uint8_t receivePin;
  float value;     // lawn sensor capacity (time)
  float valueOld;  // lawn sensor capacity (time)
} LawnSensor;

typedef struct
{
  bool use;
  uint8_t len;
  LawnSensor* lawnSensorArray_p;
  bool detected;
  uint16_t counter;
} LawnSensors;


void lawnSensor_setup(const uint8_t sendPin, const uint8_t receivePin,
    LawnSensor* lawnSensor_p);

void lawnSensor_read(LawnSensor* lawnSensor_p);

static inline
float lawnSensor_getValue(LawnSensor* lawnSensor_p)
{
  return lawnSensor_p->value;
}

uint16_t lawnSensor_measureLawnCapacity(LawnSensor* lawnSensor_p);

static inline
void lawnSensors_setup(const uint8_t* sendPins, const uint8_t* receivePins,
                        LawnSensor* lawnSensorArray_p,
                        LawnSensors* lawnSensors_p, const uint8_t len)
{
  lawnSensors_p->use = false;
  lawnSensors_p->len = len;
  lawnSensors_p->lawnSensorArray_p = lawnSensorArray_p;
  for (uint8_t i = 0; i < len; i++)
  {
    lawnSensor_setup(sendPins[i], receivePins[i], &lawnSensorArray_p[i]);
  }
  lawnSensors_p->detected = false;
  lawnSensors_p->counter = 0;
}

static inline
void lawnSensors_read(LawnSensors* lawnSensors_p)
{
  for (uint8_t i = 0; i < lawnSensors_p->len; i++)
  {
    lawnSensor_read(&lawnSensors_p->lawnSensorArray_p[i]);
  }
}

static inline
bool lawnSensors_isDetected(LawnSensors* lawnSensors_p)
{
  return lawnSensors_p->detected;
}

static inline
void lawnSensors_clearDetected(LawnSensors* lawnSensors_p)
 {
  lawnSensors_p->detected = false;
 }

static inline
void lawnSensors_simDetected(LawnSensors* lawnSensors_p)
{
  lawnSensors_p->detected = true;
  lawnSensors_p->counter++;
}

static inline
uint16_t lawnSensors_getCounter(LawnSensors* lawnSensors_p)
{
  return lawnSensors_p->counter;
}

void lawnSensors_check(LawnSensors* lawnSensors_p);
