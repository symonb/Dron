///*
// * scheduler.h
// *
// *  Created on: 28.02.2022
// *      Author: symon
// */
//
//#ifndef SCHEDULER_H_
//#define SCHEDULER_H_
//
//#include <stdint.h>
//#include <stdbool.h>
//
//#define TASK_PERIOD_KHZ(khz) (1000000000 / (khz))
//#define TASK_PERIOD_HZ(hz) (1000000 / (hz))
//#define TASK_PERIOD_MS(ms) ((ms) * 1000)
//#define TASK_PERIOD_US(us) (us)
//#define MAX(a, b) ((a>b)?a:b)
//#define SEC_TO_US(s) (s*1000000)
//
//#define SCHEDULER_DELAY_LIMIT 100 		// minimum period of task
//
//typedef uint64_t timeUs_t;
//typedef uint32_t timeMs_t;
//
//typedef enum {
//	TASK_PRIORITY_IDLE = 1,
//	TASK_PRIORITY_LOW = 2,
//	TASK_PRIORITY_MEDIUM = 3,
//	TASK_PRIORITY_HIGH = 4,
//	TASK_PRIORITY_REALTIME = 5,
//	TASK_PRIORITY_MAX = 255
//} taskPriority_e;
//
////typedef enum {
////	TASK_SYSTEM,
////	TASK_GYRO_ACC_FILTER,
////	TASK_PID,
////	TASK_TELEMETRY_ACRO,
////	TASK_COUNT
////} taskID_e;
//
////typedef struct {
////	const char* taskName;								//for debuging purpose
////	bool (*checkFun)(timeUs_t currentTime, timeUs_t deltaTime);	//check function for event driven tasks
////	void (*taskFun)(timeUs_t currentTime);					//main task function
////	taskPriority_e staticPriority;								//task priority
////	uint16_t dynamicPriority;			//prevent task from not being executed
////	timeUs_t lastExecution;
////	timeUs_t lastSignaled;
////	timeUs_t desiredPeriod;
////	timeUs_t maxExecutionTime;					//just for stats and debuging
////	timeUs_t totalExecutionTime;
////	timeUs_t movingSumExTime;	//moving sum of time consumed by 32 invokations
////	uint32_t ageCycles;
////} task_t;
//
//extern task_t tasks[TASK_COUNT];
//void task_System_Load(timeUs_t currentTime);
//bool remove_from_Queue(task_t *task);
//bool add_to_Queue(task_t *task);
//void enable_Task(taskID_e taskID, bool enabled);
//void init_Scheduler(void);
//void scheduler(void);
//#endif /* SCHEDULER_H_ */
