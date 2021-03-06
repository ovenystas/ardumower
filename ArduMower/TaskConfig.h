/** \file tsk_cfg.h
 * \brief This is the header file for the configuration of the application tasks.
 *  Including this file will include all tasks and can also be used to turn off
 *  tasks that will not be used in an application.
 */
#pragma once

#include <Arduino.h>

#include "Scheduler.h"

TaskType* Task_getConfig(void);
uint8_t Task_getNumTasks(void);
