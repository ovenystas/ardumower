/** \file Scheduler.h
 * \brief This module contains definitions for a task scheduler.
 */
#pragma once

// Includes
#include <Arduino.h>

// Constants
#define INTERVAL_CONTINUOUS 0UL
#define INTERVAL_10MS       10UL
#define INTERVAL_50MS       50UL
#define INTERVAL_100MS      100UL
#define INTERVAL_200MS      200UL
#define INTERVAL_250MS      250UL
#define INTERVAL_300MS      300UL
#define INTERVAL_500MS      500UL
#define INTERVAL_1S         1000UL
#define INTERVAL_2S         2000UL
#define INTERVAL_5S         5000UL
#define INTERVAL_1M         60000UL

/**
 * Struct TaskType
 * TaskType structure is used to define the parameters required in order to
 * configure a task.
 */
struct TaskType
{
  const uint16_t interval;   // Defines how often a task will run
  uint32_t lastTick;         // Stores the last tick task was ran
  void (*func)(void);  // Function pointer to the task
};
