/*
 * scheduler_2.c
 *
 *  Created on: 19.05.2022
 *      Author: symon
 */

#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "global_functions.h"
#include "setup.h"
#include "tasks.h"
#include "adc1.h"
#include "battery.h"
#include "telemetry.h"
#include "scheduler.h"

static void scheduler_reschedule(scheduler_t* scheduler);
static bool is_in_queue(const task_t* const task, scheduler_t* scheduler);


scheduler_t main_scheduler = { .system_load = 100 };


bool scheduler_initialization(scheduler_t* scheduler)
{
	scheduler->current_task = NULL;
	scheduler->task_queue[0] = NULL;
	scheduler->task_queue_size = 0;

	add_to_queue(&all_tasks[TASK_GYRO_CALIBRATION], scheduler);
	add_to_queue(&all_tasks[TASK_GYRO_UPDATE], scheduler);
	add_to_queue(&all_tasks[TASK_MAIN_LOOP], scheduler);
	add_to_queue(&all_tasks[TASK_UPDATE_MOTORS], scheduler);
	add_to_queue(&all_tasks[TASK_RX_HANDLING], scheduler);
	add_to_queue(&all_tasks[TASK_SYSTEM], scheduler);
	add_to_queue(&all_tasks[TASK_ACC_UPDATE], scheduler);
	add_to_queue(&all_tasks[TASK_STABILIZATION_LOOP], scheduler);
	add_to_queue(&all_tasks[TASK_OSD_INIT], scheduler);
#if defined(USE_BARO)
	add_to_queue(&all_tasks[TASK_BARO_INIT], scheduler);
	add_to_queue(&all_tasks[TASK_ALT_HOLD], scheduler);
#endif
	add_to_queue(&all_tasks[TASK_BLACKBOX_INIT], scheduler);
	add_to_queue(&all_tasks[TASK_BLACKBOX], scheduler);
#if defined(USE_TELEMETRY)
	add_to_queue(&all_tasks[TASK_TELEMETRY], scheduler);
#endif
#if defined(USE_BUZZER)
	add_to_queue(&all_tasks[TASK_BUZZER], scheduler);
#endif
	scheduler_reset_tasks_statistics(scheduler);

	return true;
}

void scheduler_reset_tasks_statistics(scheduler_t* scheduler)
{
	for (uint8_t i = 0; i < scheduler->task_queue_size;i++) {
		scheduler->task_queue[i]->dynamic_priority = 0;
		scheduler->task_queue[i]->last_execution = get_Global_Time();
		scheduler->task_queue[i]->avg_execution_time = 0;
		scheduler->task_queue[i]->avg_delayed_cycles = 0;
	}
}

void scheduler_execute(scheduler_t* scheduler)
{
	// reschedule scheduler to get next task:
	scheduler_reschedule(scheduler);

	// execute current task:
	if (scheduler->current_task != NULL)
	{
		timeUs_t execution_time = get_Global_Time();
		// execute task function:
		scheduler->current_task->task_fun(execution_time, execution_time - scheduler->current_task->last_execution);
		// update task stats:
		float delayed_cycles = (float)(execution_time - scheduler->current_task->next_execution) / scheduler->current_task->desired_period;
		float period = execution_time - scheduler->current_task->last_execution;
		scheduler->current_task->avg_delayed_cycles = scheduler->current_task->avg_delayed_cycles * 0.95f + ((float)period / scheduler->current_task->desired_period - 1) * 0.05f;
		scheduler->current_task->next_execution += (uint64_t)(1 + delayed_cycles) * scheduler->current_task->desired_period;
		scheduler->current_task->last_execution = execution_time;
		scheduler->current_task->dynamic_priority = 0;
		scheduler->current_task->avg_execution_time = scheduler->current_task->avg_execution_time * 0.95f + (get_Global_Time() - execution_time) * 0.05f;
	}
	// if nothing to do -> wait 10 [us]:
	else
	{
		delay_micro(10);
		scheduler->idle_time_counter += 10;
	}
}

static void scheduler_reschedule(scheduler_t* scheduler)
{
	//	reset variables:
	timeUs_t current_time = get_Global_Time();
	scheduler->current_task = NULL;

	//	iterate throw all tasks update dynamic priority and choose current task:
	uint8_t i = 0;
	for (task_t* task = scheduler->task_queue[i]; task != NULL; task = scheduler->task_queue[++i])
	{

		if (task->check_fun)		//	if task.check_fun() exists:
		{
			if (task->check_fun(current_time, current_time - (task->last_execution + task->avg_execution_time)))//	if .check_fun() return true (if not dynamic priority will stay 0):
			{
				//	if time for next execution has come update dynamic priority:
				if (current_time >= task->next_execution) {

					task->dynamic_priority = ((float)(current_time - task->next_execution) / (task->desired_period) + 1) * task->static_priority;

					if (task->static_priority == TASK_PRIORITY_REALTIME)
					{
						//	If realtime tasks occurs don't care which one will be first.
						//	They can overwrite each other, just set current_task as realtime:
						scheduler->current_task = task;
						// break FOR LOOP and execute this task (don't bother updating rest of the tasks): 
						break;
					}
				}
			}
		}
		// tasks without check_fun():
		else
		{
			//	if time for next execution has come update dynamic priority:
			if (current_time >= task->next_execution) {

				task->dynamic_priority = ((float)(current_time - task->next_execution) / (task->desired_period) + 1) * task->static_priority;
				//	realtime tasks handling:
				if (task->static_priority == TASK_PRIORITY_REALTIME)
				{
					//	If realtime tasks occurs don't care which one will be first.
					//	They can overwrite each other, just set current_task as realtime:
					scheduler->current_task = task;
					// break FOR LOOP and execute this task (don't bother updating rest of the tasks): 
					break;
				}
			}
		}

		// update current task:
		if ((scheduler->current_task == NULL && task->dynamic_priority > 0) || (scheduler->current_task != NULL && task->dynamic_priority > scheduler->current_task->dynamic_priority))
		{
			scheduler->current_task = task;
		}
	}
}

static bool is_in_queue(const task_t* const task, scheduler_t* scheduler)
{
	for (uint8_t i = 0; i < scheduler->task_queue_size; i++)
		if (scheduler->task_queue[i] == task)
			return true;
	return false;
}

bool add_to_queue(task_t* task, scheduler_t* scheduler)
{
	if (is_in_queue(task, scheduler) || scheduler->task_queue_size >= TASKS_COUNT) // make sure that we have space
		return false;
	for (uint8_t i = 0; i <= scheduler->task_queue_size; i++)
	{
		// set tasks in order of static priority (during rescheduling it will speed up process)
		if (scheduler->task_queue[i] == NULL || (scheduler->task_queue[i]->static_priority < task->static_priority))
		{
			memmove(&scheduler->task_queue[i + 1], &scheduler->task_queue[i],
				sizeof(task) * (scheduler->task_queue_size - i));
			scheduler->task_queue[i] = task;
			scheduler->task_queue_size++;
			return true;
		}
	}
	return false;
}

bool remove_from_queue(task_t* task, scheduler_t* scheduler)
{
	for (int i = 0; i < scheduler->task_queue_size; i++)
	{
		if (scheduler->task_queue[i] == task)
		{
			memmove(&(scheduler->task_queue[i]),
				&(scheduler->task_queue[i + 1]),
				sizeof(task) * (scheduler->task_queue_size - i));
			scheduler->task_queue_size--;

			scheduler->task_queue[scheduler->task_queue_size] = NULL;

			return true;
		}
	}
	return false;
}