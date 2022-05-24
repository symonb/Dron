///*
// * scheduler.c
//
// *
// *  Created on: 28.02.2022
// *      Author: symon
// */
//#include <math.h>
//#include <string.h>
//#include <stdbool.h>
//#include "stm32f4xx.h"
//#include "global_constants.h"
//#include "global_variables.h"
//#include "global_functions.h"
//#include "tasks.h"
//#include "scheduler.h"
//
//task_t *taskQueue[TASK_COUNT + 1];	//extra item for null pointer at the end
//static task_t *currentTask = NULL;
//static uint8_t taskQueueSize = 0;
//static uint8_t taskQueuePos = 0;
//
//static uint32_t totalWaitingTasks;
//static uint32_t totalWaitingTasksSamples;
//uint16_t averageSystemLoadPercent;
//
//void task_System_Load(timeUs_t currentTime)
//{
//	if (totalWaitingTasksSamples > 0)
//	{
//		averageSystemLoadPercent = 100 * totalWaitingTasks / totalWaitingTasksSamples;
//		totalWaitingTasksSamples = 0;
//		totalWaitingTasks = 0;
//	}
//}
//
//static void clear_Queue(void)
//{
//	// NIE wiem czy nie powinno byc NULL zamiast 0?
//
//	memset(taskQueue, 0, sizeof(taskQueue));
//	taskQueuePos = 0;
//	taskQueueSize = 0;
//}
//
// static bool is_In_Queue(task_t *task)
//{
//	for (uint8_t i = 0; i < taskQueueSize; i++)
//		if (taskQueue[i] == task)
//			return true;
//	return false;
//}
//
// bool add_to_Queue(task_t *task){
//
//	if (is_In_Queue(task) || taskQueueSize >= TASK_COUNT) //make sure that we have place
//		return false;
//	for (uint8_t i = 0; i <= taskQueueSize; i++)
//	{
//	    if (taskQueue[i] == NULL || taskQueue[i]->staticPriority < task->staticPriority) {
//	            memmove(&taskQueue[i+1], &taskQueue[i], sizeof(task) * (taskQueueSize - i));
//	            taskQueue[i] = task;
//	            taskQueueSize++;
//	            return true;
//		}
//	}
//	return false;
//}
//
// bool remove_from_Queue(task_t *task)
//{
//	for (int i = 0; i < taskQueueSize; i++)
//	{
//		if (taskQueue[i] == task)
//		{
//			memmove(&taskQueue[i], &taskQueue[i + 1], sizeof(task) * (taskQueueSize - i));
//			taskQueueSize--;
//			//modyfikacja: w ten sposób nieu¿ywana kolejka ma zawsze NULL:
//			/* teoretycznie po usuniêciu jest NULL ale to zalezy od kompilatora
//			 *
//			 * taskQueue[taskQueueSize]=NULL;
//			 */
//			return true;
//		}
//	}
//	return false;
//}
//
//static task_t *queue_First(void)
//{
//	taskQueuePos = 0;
//	return taskQueue[0];
//}
//
//static task_t *queue_Next(void)
//{
//	return taskQueue[++taskQueuePos];
//}
//
//void enable_Task(taskID_e taskID, bool enabled)
//{
//	if (taskID < TASK_COUNT)
//	{
//		task_t *task = &tasks[taskID];
//		enabled ? add_To_Queue(task) : remove_from_Queue(task);
//	}
//}
//
//void reschedule_Task(taskID_e taskID, timeUs_t newPeriodUs)
//{
//    if (taskID < TASK_COUNT) {
//        task_t *task = getTask(taskID);
//        task->desiredPeriod = MAX(SCHEDULER_DELAY_LIMIT, newPeriodUs);  // Limit delay to 100us (10 kHz) to prevent scheduler clogging
//    }
//}
//
//void init_Scheduler(void)
//{
//	clear_Queue();
//	enable_Task(TASK_SYSTEM, true);
//}
//void execute_Task(task_t* selectedTask,timeUs_t currentTime){
//
//	const timeUs_t timeBeforeTask = SEC_TO_US(get_Global_Time());
//	selectedTask->taskFun(SEC_TO_US(get_Global_Time()));
//	const timeUs_t timeAfterTask = SEC_TO_US(get_Global_Time());
//	timeUs_t dt_local = timeAfterTask - timeBeforeTask;
//
//	selectedTask->totalExecutionTime += dt_local;
//	selectedTask->maxExecutionTime = MAX(dt_local, selectedTask->maxExecutionTime);
//	selectedTask->movingSumExTime += dt_local - selectedTask->movingSumExTime / 32;
//	selectedTask->lastExecution = currentTime;
//	selectedTask->dynamicPriority = 0;
//}
//
//void scheduler(void)
//{
//	timeUs_t currentTime = SEC_TO_US(get_Global_Time());
//	static task_t *selectedTask = NULL; // need to remember old one
//	uint16_t selectedTaskDynamicPriority = 0;
//	uint8_t waitingTasks = 0;
//
//	        // some tasks: gyro/filtering/PID get complete priority
///*	if(gyro_ready){
//		execute_Task(get_Task(TASK_GYRO));
//		remove_from_Queue(get_Task(TASK_GYRO));
//	}
//	if(filter_ready){
//		execute_Task(get_Task(TASK_FILTER));
//		remove_from_Queue(get_Task(TASK_FILTER));
//	}
//	if(PID_ready){
//		execute_Task(get_Task(TASK_PID));
//		remove_from_Queue(get_Task(TASK_PID));
//	}
//*/
//
//	// view all tasks in queue and update their dynamic_priorities and parameters:
//	for (task_t *task = queue_First(); task != NULL; task = queue_Next())
//	{
//		if (task->checkFun)
//		{
//			/*
//				TODO: Configure event driven task
//			*/
//		}
//
//		else //update dynamic_priority
//		{
//			task->ageCycles = (currentTime - task->lastExecution) / task->desiredPeriod;
//			if (task->ageCycles > 0)
//			{
//				task->dynamicPriority = 1 + task->staticPriority * task->ageCycles;
//				waitingTasks++;
//			}
//		}
//
//		//update task which will be done now if dynamic_priority is higher than selected so far:
//		if ((task->dynamicPriority > selectedTaskDynamicPriority))
//		{
//			selectedTaskDynamicPriority = task->dynamicPriority;
//			selectedTask = task;
//			waitingTasks++;
//		}
//
//	}
//		totalWaitingTasks += waitingTasks;
//		totalWaitingTasksSamples++;
//		currentTask = selectedTask;
//
//	if (selectedTask)
//	{
//		execute_Task(currentTask,currentTime);
//		remove_from_Queue(currentTask);
//	}
//}
//
//
