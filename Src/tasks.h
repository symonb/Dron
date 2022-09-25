///*
// * tasks.h

// *
// *  Created on: 04.03.2022
// *      Author: symon
// */

#ifndef TASKS_H_
#define TASKS_H_

#include "global_variables.h"
#include "stdbool.h"

#define DEFINE_TASK(task_name_param, task_fun_param, check_fun_param, static_priority_param, desired_period_param) \
	{                                                                                                              \
		.task_name = task_name_param,                                                                              \
		.task_fun = task_fun_param,                                                                                \
		.check_fun = check_fun_param,                                                                              \
		.static_priority = static_priority_param,                                                                  \
		.dynamic_priority = 0,                                                                                     \
		.last_execution = 0,                                                                                       \
		.desired_period = desired_period_param,                                                                    \
		.avg_execution_time = 0                                                                                    \
	}

typedef enum
{
	TASK_SYSTEM,
	TASK_IBUS_SAVE,
	TASK_GYRO_ACC_FILTER,
	TASK_MAIN_LOOP,
	TASK_STABILIZATION_LOOP,
	TASK_UPDATE_MOTORS,
	TASK_TELEMETRY,
	TASKS_COUNT
} taskID_e;

typedef enum
{
	TASK_PRIORITY_IDLE = 1,
	TASK_PRIORITY_LOW = 2,
	TASK_PRIORITY_MEDIUM = 3,
	TASK_PRIORITY_HIGH = 4,
	TASK_PRIORITY_REALTIME = 5,
	TASK_PRIORITY_MAX = 255
} taskPriority_e;

typedef struct
{
	const char *task_name;										   // task name
	void (*task_fun)(timeUs_t current_time);					   // main task function
	bool (*check_fun)(timeUs_t current_time, timeUs_t delta_time); // function for checking event occurrence
	taskPriority_e static_priority;								   // task basic priority
	uint16_t dynamic_priority;									   // prevent task from not being executed
	timeUs_t last_execution;									   // last time when this task was done
	timeUs_t desired_period;									   // period that wishes to be achieved in [us]
	float avg_execution_time;									   // average execution time of this task

} task_t;

extern task_t all_tasks[];

#endif /* TASKS_H_ */
