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
#include "scheduler_2.h"
#include "setup.h"
#include "PID.h"

#define DEFINE_TASK(task_name_param, task_fun_param, check_fun_param, static_priority_param, desired_period_param) {  \
	.task_name = task_name_param, \
	.task_fun = task_fun_param, \
	.check_fun = check_fun_param, \
	.static_priority = static_priority_param, \
	.dynamic_priority=0,\
	.last_execution=0,\
	.desired_period = 0,\
	.avg_execution_time = 0\
	}

bool scheduler_initialization(scheduler_t *scheduler);
void scheduler_execute(scheduler_t *scheduler);
void add_all_tasks(scheduler_t *scheduler);
bool is_in_queue(task_t *task, scheduler_t *scheduler);
bool add_to_queue(task_t *task, scheduler_t *scheduler);
bool remove_from_queue(task_t *task, scheduler_t *scheduler);

scheduler_t main_scheduler;

// ------DEFINE ALL TASKS--------
task_t all_tasks[TASKS_COUNT] =
		{ 		[TASK_SYSTEM ] = DEFINE_TASK("SYSTEM_SETUP", setup, NULL, TASK_PRIORITY_MAX, 10000),
				[TASK_PID] = DEFINE_TASK("PID", PID_fun, NULL, TASK_PRIORITY_MAX, TASK_PERIOD_HZ(FREQUENCY_PID_LOOP)),
		};



bool scheduler_initialization(scheduler_t *scheduler) {

	scheduler->current_task_name = NULL;
	scheduler->task_queue[0] = NULL;
	scheduler->task_queue_size = 0;

	return true;
}

void scheduler_execute(scheduler_t *scheduler) {
	timeUs_t currentTime = SEC_TO_US(get_Global_Time());
}

void add_all_tasks(scheduler_t *scheduler) {

}

bool is_in_queue(task_t *task, scheduler_t *scheduler) {
	for (uint8_t i = 0; i < scheduler->task_queue_size; i++)
		if (scheduler->task_queue[i] == task)
			return true;
	return false;
}

bool add_to_queue(task_t *task, scheduler_t *scheduler) {

	if (is_in_queue(task, scheduler)
			|| scheduler->task_queue_size >= TASKS_COUNT) //make sure that we have place
		return false;
	for (uint8_t i = 0; i <= scheduler->task_queue_size; i++) {
		if (scheduler->task_queue[i] == NULL
				|| (scheduler->task_queue[i]->static_priority
						< task->static_priority)) {
			memmove(&scheduler->task_queue[i + 1], &(scheduler->task_queue[i]),
					sizeof(task) * (scheduler->task_queue_size - i));
			scheduler->task_queue[i] = task;
			scheduler->task_queue_size++;
			return true;
		}
	}
	return false;
}

bool remove_from_queue(task_t *task, scheduler_t *scheduler) {
	for (int i = 0; i < scheduler->task_queue_size; i++) {
		if (scheduler->task_queue[i] == task) {
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

