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
	TASK_GYRO_UPDATE,
	TASK_MAIN_LOOP,
	TASK_UPDATE_MOTORS,
	TASK_RX_HANDLING,
	TASK_SYSTEM,
	TASK_ACC_UPDATE,
	TASK_STABILIZATION_LOOP,
	TASK_BARO,
	TASK_ALT_HOLD,
	TASK_BLACKBOX,
	TASK_OSD,
	TASK_TELEMETRY,
	TASK_BUZZER,
	TASK_USB_HANDLING,
	TASK_OSD_INIT,
	TASK_BLACKBOX_INIT,
	TASK_BARO_INIT,
	TASK_GYRO_CALIBRATION,
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


/**
 *@brief task structure
*@param task_name name of the function
*@param task_fun pointer to task function with 2 arguments (current time and time from last execution)
*@param check_fun pointer to function for checking event occurrence, delta_time is a time from the (last execution - average_execution_time)
*@param static_priority basic priority of the task
*@param dynamic_priority priority used for recheduling tasks, set as static_priority*delayed_cycles
*@param	last_execution last time when task_fun was started (updated before fun not after)
*@param next_execution time of the next execution
*@param desired_period period wished to be achieved in [us]
*@param avg_execution_time average execution time of this task
*@param avg_delayed_cycles  average delayed cycles for this task
*/
typedef struct
{
	const char* task_name;
	void (*task_fun)(timeUs_t current_time, timeUs_t dt_us);
	bool (*check_fun)(timeUs_t current_time, timeUs_t delta_time);
	taskPriority_e static_priority;
	uint16_t dynamic_priority;
	timeUs_t last_execution;
	timeUs_t next_execution;
	timeUs_t desired_period;
	float avg_execution_time;
	float avg_delayed_cycles;
} task_t;

extern task_t all_tasks[];

#endif /* TASKS_H_ */
