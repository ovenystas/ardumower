/** \file tsk_cfg.c
 * \brief This is source file for the configuration of the application tasks.
 *  Including this file will include all tasks and can also be used to turn off
 *  tasks that will not be used in an application.
 */

#include "tsk_cfg.h"
#include "Mower.h"      // Where the tasks to run are
#include "scheduler.h"  // For task interval definitions

void run_task_continious(void)
{
  robot.tasks_continious();
}

void run_task_200ms(void)
{
  robot.tasks_200ms();
}

void run_task_1000ms(void)
{
  robot.tasks_1000ms();
}

/**
 * Task configuration table. Holds the task interval, last time executed, and
 * the function to be executed. A continuous task is defined as a task with
 * an interval of 0. Last time executed is set to 0.
 */
static TaskType Tasks[] =
{
  { 0              , 0, run_task_continious },
  { INTERVAL_200MS , 0, run_task_200ms },
  { INTERVAL_1000MS, 0, run_task_1000ms },
};

TaskType *Tsk_GetConfig(void)
{
  return Tasks;
}

byte Tsk_GetNumTasks(void)
{
  return sizeof(Tasks) / sizeof(*Tasks);
}
