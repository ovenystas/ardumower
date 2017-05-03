/** \file Scheduler.h
 * \brief This module contains definitions for a task scheduler.
 */
#ifndef SCHEDULER_H_
#define SCHEDULER_H_

// Includes
#include <Arduino.h>

// Constants
#define INTERVAL_10MS   10UL
#define INTERVAL_50MS   50UL
#define INTERVAL_100MS  100UL
#define INTERVAL_200MS  200UL
#define INTERVAL_250MS  250UL
#define INTERVAL_500MS  500UL
#define INTERVAL_1000MS 1000UL
#define INTERVAL_2000MS 2000UL

/**
 * Struct TaskType
 * TaskType structure is used to define the parameters required in order to
 * configure a task.
 */
typedef struct
{
  unsigned int interval;    // Defines how often a task will run
  unsigned long lastTick;    // Stores the last tick task was ran
  void (*func)(void);   // Function pointer to the task
} TaskType;

#endif // SCHEDULER_H_
