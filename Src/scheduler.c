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
#include "connection.h"
#include "scheduler.h"

bool scheduler_initialization(scheduler_t *scheduler);
void scheduler_execute(scheduler_t *scheduler);
void scheduler_reschedule(scheduler_t *scheduler);
void add_all_tasks(scheduler_t *scheduler);
bool is_in_queue(task_t *task, scheduler_t *scheduler);
bool add_to_queue(task_t *task, scheduler_t *scheduler);
bool remove_from_queue(task_t *task, scheduler_t *scheduler);
static task_t *queue_first(scheduler_t *scheduler);
static task_t *queue_next(scheduler_t *scheduler);

scheduler_t main_scheduler;
static uint8_t task_queue_pos;
static timeUs_t idle_time_counter = 0;

void task_system_fun(timeUs_t current_time)
{
	static timeUs_t last_time;
	main_scheduler.system_load = main_scheduler.system_load * 0.9f + 10 * (1 - (float)(idle_time_counter) / (current_time - last_time));
	idle_time_counter = 0;
	last_time = current_time;
	//	read MCU temp. and battery voltage:
	ADC1_start(current_time);
	battery_manage();
	//	blackbox management:
	if (BLACKBOX_STATUS == BLACKBOX_SEND_DATA)
	{
		print_blackbox();
	}
}

bool scheduler_initialization(scheduler_t *scheduler)
{
	scheduler->current_task = NULL;
	scheduler->task_queue[0] = NULL;
	scheduler->task_queue_size = 0;

	add_to_queue(&all_tasks[TASK_GYRO_ACC_FILTER], scheduler);
	add_to_queue(&all_tasks[TASK_IBUS_SAVE], scheduler);
	add_to_queue(&all_tasks[TASK_MAIN_LOOP], scheduler);
	add_to_queue(&all_tasks[TASK_STABILIZATION_LOOP], scheduler);
	add_to_queue(&all_tasks[TASK_SYSTEM], scheduler);
	add_to_queue(&all_tasks[TASK_TELEMETRY], scheduler);
	add_to_queue(&all_tasks[TASK_UPDATE_MOTORS], scheduler);

	return true;
}

void scheduler_execute(scheduler_t *scheduler)
{
	static timeUs_t time_before_execution;

	// reschedule scheduler to get next task:
	scheduler_reschedule(scheduler);

	// execute current task:
	if (scheduler->current_task != NULL)
	{
		time_before_execution = get_Global_Time();
		// execute task function:
		scheduler->current_task->task_fun(time_before_execution);
		// update task params:
		scheduler->current_task->last_execution = time_before_execution;
		scheduler->current_task->dynamic_priority = 0;
		scheduler->current_task->avg_execution_time = scheduler->current_task->avg_execution_time * 0.95f + (get_Global_Time() - time_before_execution) * 0.05f;
	}
	// if nothing to do wait 10 [us]:
	else
	{
		delay_micro(10);
		idle_time_counter += 10;
	}
}

void scheduler_reschedule(scheduler_t *scheduler)
{
	//	reset variables:
	timeUs_t current_time = get_Global_Time();
	bool real_time_task_lock = false;
	scheduler->current_task = NULL;

	//	iterate throw all tasks update dynamic priority and choose current task:
	for (task_t *task = queue_first(scheduler); task != NULL; task = queue_next(scheduler))
	{
		//	if task.check_fun() exists:
		if (task->check_fun)
		{ //	if .check_fun() return true (if not dynamic priority will stay 0):
			if (task->check_fun(current_time, current_time - task->last_execution))
			{
				if (task->static_priority == TASK_PRIORITY_REALTIME && (current_time - task->last_execution) > task->desired_period)
				{
					//	If realtime tasks occurs don't care which one will be first.
					//	They can overwrite each other, just set current_task as realtime:
					scheduler->current_task = task;
					// lock scheduling since realtime task will be done:
					real_time_task_lock = true;
				}
				//	update dynamic priority:
				else
				{
					task->dynamic_priority = (current_time - task->last_execution) / (task->desired_period) * task->static_priority;
				}
			}
		}
		//	realtime tasks handling:
		else if (task->static_priority == TASK_PRIORITY_REALTIME)
		{
			if ((current_time - task->last_execution) > task->desired_period)
			{
				//	If realtime tasks occurs don't care which one will be first.
				//	They can overwrite each other, just set current_task as realtime:
				scheduler->current_task = task;
				// lock scheduling since realtime task will be done:
				real_time_task_lock = true;
			}
		}
		//	all other tasks handling:
		else
		{ // update dynamic priority (if time hasn't come yet will stay 0):
			task->dynamic_priority = (current_time - task->last_execution) / (task->desired_period) * task->static_priority;
		}

		// update current task (if there is no realtime tasks):
		if (!real_time_task_lock)
		{
			if ((scheduler->current_task == NULL && task->dynamic_priority > 0) || (task->dynamic_priority > scheduler->current_task->dynamic_priority))
			{
				scheduler->current_task = task;
			}
		}
	}
}

void add_all_tasks(scheduler_t *scheduler)
{
}

bool is_in_queue(task_t *task, scheduler_t *scheduler)
{
	for (uint8_t i = 0; i < scheduler->task_queue_size; i++)
		if (scheduler->task_queue[i] == task)
			return true;
	return false;
}

bool add_to_queue(task_t *task, scheduler_t *scheduler)
{

	if (is_in_queue(task, scheduler) || scheduler->task_queue_size >= TASKS_COUNT) // make sure that we have space
		return false;
	for (uint8_t i = 0; i <= scheduler->task_queue_size; i++)
	{
		if (scheduler->task_queue[i] == NULL || (scheduler->task_queue[i]->static_priority < task->static_priority))
		{
			memmove(&scheduler->task_queue[i + 1], &(scheduler->task_queue[i]),
					sizeof(task) * (scheduler->task_queue_size - i));
			scheduler->task_queue[i] = task;
			scheduler->task_queue_size++;
			return true;
		}
	}
	return false;
}

bool remove_from_queue(task_t *task, scheduler_t *scheduler)
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

static task_t *queue_first(scheduler_t *scheduler)
{
	task_queue_pos = 0;
	return scheduler->task_queue[0];
}

static task_t *queue_next(scheduler_t *scheduler)
{
	return scheduler->task_queue[++task_queue_pos];
}