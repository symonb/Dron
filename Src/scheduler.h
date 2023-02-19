/*
 * scheduler_2.h
 *
 *  Created on: 19.05.2022
 *      Author: symon
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include "tasks.h"
#include "stdbool.h"

typedef struct
{
	task_t* current_task;				 // current task
	task_t* task_queue[TASKS_COUNT + 1]; // table of pointers for tasks and one more for NULL
	uint8_t task_queue_size;			 // counter of tasks in queue
	float system_load;					 // how much system is loaded in %
} scheduler_t;

bool scheduler_initialization(scheduler_t* scheduler);
void scheduler_reset_tasks_statistics(scheduler_t* scheduler);
void scheduler_execute(scheduler_t* scheduler);
void task_system_fun(timeUs_t currentTime);
bool remove_from_queue(task_t* task, scheduler_t* scheduler);
extern scheduler_t main_scheduler;

#endif /* SCHEDULER_H_ */
