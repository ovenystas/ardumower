/** \file tsk_cfg.c
 * \brief This is source file for the configuration of the application tasks.
 *  Including this file will include all tasks and can also be used to turn off
 *  tasks that will not be used in an application.
 */

#include "tsk_cfg.h"
#include "Robot.h"      // Where the tasks to run are declared
#include "scheduler.h"  // For task interval definitions

void run_task_continuous(void)
{
  robot.tasks_continuous();
}

void run_task_50ms(void)
{
  robot.tasks_50ms();
}

void run_task_100ms(void)
{
  robot.tasks_100ms();
}

void run_task_200ms(void)
{
  robot.tasks_200ms();
}

void run_task_250ms(void)
{
  robot.tasks_250ms();
}

void run_task_300ms(void)
{
  robot.tasks_300ms();
}

void run_task_500ms(void)
{
  robot.tasks_500ms();
}

void run_task_1s(void)
{
  robot.tasks_1s();
}

void run_task_2s(void)
{
  robot.tasks_2s();
}

void run_task_5s(void)
{
  robot.tasks_5s();
}

void run_task_1m(void)
{
  robot.tasks_1m();
}

/**
 * Task configuration table. Holds the task interval, last time executed, and
 * the function to be executed. A continuous task is defined as a task with
 * an interval of 0. Last time executed is set to 0.
 */
static TaskType Tasks[] =
{
  { 0,              0, run_task_continuous },
  { INTERVAL_50MS,  0, run_task_50ms },
  { INTERVAL_100MS, 0, run_task_100ms },
  { INTERVAL_200MS, 0, run_task_200ms },
  { INTERVAL_250MS, 0, run_task_250ms },
  { INTERVAL_300MS, 0, run_task_300ms },
  { INTERVAL_500MS, 0, run_task_500ms },
  { INTERVAL_1S,    0, run_task_1s },
  { INTERVAL_2S,    0, run_task_2s },
  { INTERVAL_5S,    0, run_task_5s },
  { INTERVAL_1M,    0, run_task_1m },
};

TaskType *Tsk_GetConfig(void)
{
  return Tasks;
}

byte Tsk_GetNumTasks(void)
{
  return sizeof(Tasks) / sizeof(*Tasks);
}
