///*
// * tasks.c
// *
// *  Created on: 04.03.2022
// *      Author: symon
// */
//
//
//#include "stm32f4xx.h"
//#include "global_constants.h"
//#include "global_variables.h"
//#include "global_functions.h"
//#include "setup.h"
//#include "acro.h"
//#include "stabilize.h"
//#include "MPU6000.h"
//#include "PID.h"
//#include "scheduler.h"
//#include "tasks.h"
//
//
//task_t * get_Task(unsigned taskId)
//{
//    return &tasks[taskId];
//}
//
//#define DEFINE_TASK(taskNameParam, checkFuncParam, taskFuncParam, desiredPeriodParam, staticPriorityParam) {  \
//	.taskName = taskNameParam, \
//	.checkFun = checkFuncParam, \
//    .taskFun = taskFuncParam, \
//	.staticPriority = staticPriorityParam, \
//	.dynamicPriority=0,\
//	.lastExecution=0,\
//	.lastSignaled=0,\
//    .desiredPeriod = desiredPeriodParam, \
//	.maxExecutionTime=0,\
//	.totalExecutionTime=0,\
//	.movingSumExTime=0,\
//	.ageCycles=0\
//}
//
//	// ADD all tasks in this array also add TASK_*NAME* in "scheduler.h"
//task_t tasks[TASK_COUNT] = {
//    [TASK_SYSTEM] = DEFINE_TASK("SYSTEM", NULL, setup, TASK_PERIOD_HZ(10), TASK_PRIORITY_MEDIUM),
//    [TASK_GYRO_ACC_FILTER] = DEFINE_TASK("FILTER_GYRO_ACC", NULL,rewrite_Gyro_Acc_data, TASK_PERIOD_HZ(FREQUENCY_IMU_READING), TASK_PRIORITY_REALTIME),
//    [TASK_PID] = DEFINE_TASK("PID", NULL, PID_fun, TASK_PERIOD_HZ(FREQUENCY_PID_LOOP), TASK_PRIORITY_REALTIME),
//	[TASK_TELEMETRY_ACRO] = DEFINE_TASK("TELEMETRY_ACRO", NULL, send_telemetry_acro, TASK_PERIOD_HZ(FREQUENCY_TELEMETRY_UPDATE), TASK_PRIORITY_LOW),
//};
//
//
//void init_Tasks(void)
//{
//	init_Scheduler();
//	enable_Task(TASK_SYSTEM, true);
//	enable_Task(TASK_GYRO_ACC_FILTER, true);
//	enable_Task(TASK_PID, true);
//}
//
