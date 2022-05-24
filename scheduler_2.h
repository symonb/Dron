/*
 * scheduler_2.h
 *
 *  Created on: 19.05.2022
 *      Author: symon
 */

#ifndef SCHEDULER_2_H_
#define SCHEDULER_2_H_


#include <stdint.h>
#include <stdbool.h>





typedef enum {
	TASK_SYSTEM,
	TASK_GYRO_ACC_FILTER,
	TASK_PID,
	TASK_TELEMETRY,
	TASKS_COUNT
} taskID_e;

typedef enum {
	TASK_PRIORITY_IDLE = 1,
	TASK_PRIORITY_LOW = 2,
	TASK_PRIORITY_MEDIUM = 3,
	TASK_PRIORITY_HIGH = 4,
	TASK_PRIORITY_REALTIME = 5,
	TASK_PRIORITY_MAX = 255
} taskPriority_e;


typedef struct {
	const char* task_name;						// task name
	void (*task_fun)(timeUs_t current_time);		// main task function
	bool (*check_fun)();						// function for checking event occurrence
	taskPriority_e static_priority;				// task basic priority
	uint16_t dynamic_priority;					// prevent task from not being executed
	double last_execution;						// last time when this task was done
	timeUs_t desired_period;					// period that wishes to be achieved in [us]
	timeUs_t avg_execution_time;				// average execution time of this task

} task_t;


typedef struct {
	char* current_task_name;					// current task name
	void (*execute_fun)(timeUs_t current_time);	// function which will execute current_task main function
	task_t *task_queue[TASKS_COUNT];			// table of pointers for tasks
	uint8_t task_queue_size;					// counter of tasks in queue

} scheduler_t;


bool scheduler_initialization(scheduler_t *scheduler);





#endif /* SCHEDULER_2_H_ */
