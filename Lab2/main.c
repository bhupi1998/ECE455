// SET memory scheme to use heac 4!!!
// increase heap size too
// configUSE_QUEUE_SET must be 1 for createSet to work!
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

// including libraries for GPIO and ADC Set up
// not needed, artifact from lab 1
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h"
#include "../Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h"

/*---------------DEFINE-----------------------------------*/
#define amber  	0
#define green  	1
#define red  	2
#define blue  	3

#define amber_led	LED3
#define green_led	LED4
#define red_led		LED5
#define blue_led	LED6

#define BENCH 1 // used to change test bench setup

#define TASK_HIGH_PRIORITY 2
#define TASK_LOW_PRIORITY 1

#define DDS_PRIORITY 5
#define GEN_PRIORITY 4
#define MONITOR_PRIORITY 3
// Executiong delays
#define DELAY95 95
#define DELAY100 100
#define DELAY150 150
#define DELAY200 200
#define DELAY250 250

// How many values combined queue can hold
#define COMBINED_QUEUE_LENGTH 12
/*--------------------------------------------------------*/
 /* TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );



QueueHandle_t  dd_task_queue, completed_task_queue, overdue_task_queue, request_list_queue, generate_queue;
static QueueSetHandle_t DDS_Queue_Set;

/*-----------------------------------------------------------*/

/*------------------------Structs-----------------------*/
enum task_type {PERIODIC, APERIODIC};
enum list_request_type{ACTIVE_LIST, COMPLETED_LIST, OVERDUE_LIST};

struct dd_task {
	TaskHandle_t t_handle;
	char type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	TimerHandle_t timer_Handle; // This is used for overdue timer handling.
	uint32_t period; // used by task generator. Allows to use one struct
};

// using doubly linked lists for easy deletion.
struct dd_task_list {
	struct dd_task task;
	struct dd_task_list *next_task;
	struct dd_task_list *prev_task;

};

struct list_Request {
	uint8_t list_Type; // which list is being requested
	QueueHandle_t response_Queue; //queue handler where the dds should send the
};

/*------------------Functions-----------------------------*/
/*Function declarations*/
void create_dd_task( TaskHandle_t t_handle,	char type,	uint32_t task_id, uint32_t absolute_deadline);
void complete_dd_task(uint32_t task_id);
// return pointer of the head of the list
struct dd_task_list* get_active_dd_task_list(void);
struct dd_task_list* get_complete_dd_task_list(void);
struct dd_task_list* get_overdue_dd_task_list(void);

void F_task1(void *pvParameters);
void F_task2(void *pvParameters);
void F_task3(void *pvParameters);
// Setting up gpio
void GPIO_SetUp();

// Sorting algorithm
struct dd_task_list* EDF_Sort(struct dd_task_list* head);

// set task priorities
void set_dds_Task_Priority(struct dd_task_list* active_List);

// handle overdue task
void overdue_Timer_Callback(TimerHandle_t xTimer);
// handle task generator scheduling
void generate_Timer_Callback(TimerHandle_t xTimer);

/*-----------------------------------------------------------*/
/*----------------------------DDS Functions----------------------*/
static void deadline_Driven_Scheduler_Task(void *pvParameters);
static void DDS_Task_Gen_Task(void *pvParameters);
static void monitor_Task(void *pvParameters);
/*-----------------------------------------------------------*/

int main(void)
{
	GPIO_SetUp(); // will need to be configured for led testing


	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	DDS_Queue_Set = xQueueCreateSet(COMBINED_QUEUE_LENGTH);
	// space for 10 tasks might be excessive since the DDS will clear it out right away
	dd_task_queue = xQueueCreate(3, sizeof(struct dd_task));

	// contains taskID
	completed_task_queue = xQueueCreate(3, sizeof(uint32_t));

	// timer will write to this queue if task is overdue.
	overdue_task_queue = xQueueCreate(3, sizeof(uint32_t));

	// Queue for requesting lists from DDS
	request_list_queue = xQueueCreate(3,sizeof(struct list_Request));

	// add queues to set
	xQueueAddToSet(dd_task_queue,DDS_Queue_Set);
	xQueueAddToSet(completed_task_queue,DDS_Queue_Set);
	xQueueAddToSet(overdue_task_queue,DDS_Queue_Set);
	xQueueAddToSet(request_list_queue,DDS_Queue_Set);

	configASSERT(DDS_Queue_Set);
	// task id of tasks to release by the generator here
	generate_queue = xQueueCreate(3,sizeof(uint32_t));
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( dd_task_queue, "DD_TASK_Q" );
	vQueueAddToRegistry( completed_task_queue, "Comp_Q" );
	vQueueAddToRegistry( overdue_task_queue, "Over_Q" );
	vQueueAddToRegistry(request_list_queue,"Req_Q");


	// DDS Core
	xTaskCreate(deadline_Driven_Scheduler_Task, "DDS",configMINIMAL_STACK_SIZE*2,NULL,DDS_PRIORITY,NULL);
	xTaskCreate(DDS_Task_Gen_Task, "Task Generator",configMINIMAL_STACK_SIZE,NULL,GEN_PRIORITY,NULL);
	xTaskCreate(monitor_Task, "Monitor Task",configMINIMAL_STACK_SIZE,NULL,MONITOR_PRIORITY,NULL);

	// F tasks created in task generator.


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*---------------------DDS-Tasks----------------------------------------*/

void deadline_Driven_Scheduler_Task(void *pvParameters){
	// these lists contain the head of the list
	static struct dd_task_list *active_List = NULL;
	static struct dd_task_list *completed_List = NULL;
	static struct dd_task_list *overdue_List = NULL;

	struct dd_task received_task;
	struct list_Request request_List;

	uint32_t completed_task_ID;
	uint32_t overdue_task_ID;
	TimerHandle_t overdue_Timer;

	QueueSetMemberHandle_t xActivatedMember;
	while(1){
		// only blocks for 200ms. This could be changed to forever.
		xActivatedMember = xQueueSelectFromSet(DDS_Queue_Set,200/portTICK_PERIOD_MS);
		// a new task was added

		if(xActivatedMember == dd_task_queue )
		{
			xQueueReceive(xActivatedMember, &received_task, 0);
			// NEED TO ASSIGN RELEASE TIME
			received_task.release_time = xTaskGetTickCount();
			// Create timer to check if task is overdue
			TickType_t timerPeriod = received_task.absolute_deadline-received_task.release_time;
			if(timerPeriod > 0){
				overdue_Timer= xTimerCreate("Overdue Timer",timerPeriod,pdFALSE,received_task.task_id,overdue_Timer_Callback);
				received_task.timer_Handle= overdue_Timer; // the handle will be unique to each instance of timer

				// ADD TO ACTIVE LIST
				struct dd_task_list *new_Node = pvPortMalloc(sizeof(struct dd_task_list)); // never freed since they just get moved in completed or overdue later

				new_Node->task = received_task; // make new linked list item
				new_Node->next_task = active_List; // make it point to the previous head of the list
				new_Node->prev_task = NULL; // since it's the head it has nothing to point back to

				if(active_List != NULL){
					active_List->prev_task = new_Node;
				}
				active_List = new_Node; // update the head
				// turn timer on
				xTimerStart(overdue_Timer,0);
				// SORT BY DEADLINE
				active_List = EDF_Sort(active_List);
				// SET USER TASK PRIORITY
				set_dds_Task_Priority(active_List);
				printf("\ntask Created ID %d at %d\n",new_Node->task.task_id,new_Node->task.release_time);
			}else{
				// task could not run be released on time, add it to overdue.
				uint32_t task_id = received_task.task_id; // the ID of the timer is set the same as the task
				if (xQueueSend(overdue_task_queue, &task_id, portMAX_DELAY) != pdPASS){
					printf("Could not send to overdue task Queue");
					}
			}

		}
		// a task has completed. Receives the ID of the task
		if(xActivatedMember == completed_task_queue  )
		{
			xQueueReceive(xActivatedMember, &completed_task_ID, 0) ;
			struct dd_task_list *curr = active_List;
			// find task with ID
			while(curr != NULL){

				if(curr->task.task_id == completed_task_ID){
					break;
				}
				curr = curr->next_task;
			}
			if(curr == NULL){
				printf("Invalid ID-completed_dd_task\n");
			}else{
				// delete the timer since the task has completed
				xTimerDelete(curr->task.timer_Handle,0);
        		// Remove from active_List and move to completed
        		if (curr->prev_task != NULL)
            		curr->prev_task->next_task = curr->next_task;
        		else
            		active_List = curr->next_task; // was head

       			if (curr->next_task != NULL)
            		curr->next_task->prev_task = curr->prev_task;

				curr->task.completion_time = xTaskGetTickCount();
				// Insert into completed_List at the head
				curr->next_task = completed_List;
				curr->prev_task = NULL;

				if (completed_List != NULL)
					completed_List->prev_task = curr;

				completed_List = curr;
				vTaskDelete(curr->task.t_handle); // Delete Task
				printf("\ntask completed ID %d at %d\n",curr->task.task_id,curr->task.completion_time);
			}
			// SET USER TASK PRIORITY AGAIN
			set_dds_Task_Priority(active_List);
		}

		// Message from timer about an overdue task received
		if(xActivatedMember == overdue_task_queue)
		{
			xQueueReceive(xActivatedMember, &overdue_task_ID, 0);
			struct dd_task_list *curr = active_List;
			// find task with ID
			while(curr != NULL){
				if(curr->task.task_id == overdue_task_ID){
					break;
				}
				curr = curr->next_task;
			}
			if(curr == NULL){
				printf("Invalid overdue task ID-\n");
			}else{
				// delete the timer since the task has completed
				xTimerDelete(curr->task.timer_Handle,0);
        		// Remove from active_List and move to completed
        		if (curr->prev_task != NULL)
            		curr->prev_task->next_task = curr->next_task;
        		else
            		active_List = curr->next_task; // was head

       			if (curr->next_task != NULL)
            		curr->next_task->prev_task = curr->prev_task;

				curr->task.completion_time = xTaskGetTickCount();
				// Insert into completed_List at the head
				curr->next_task = overdue_List;
				curr->prev_task = NULL;

				if (overdue_List != NULL)
				overdue_List->prev_task = curr;

				overdue_List = curr;
				vTaskDelete(curr->task.t_handle); // delete task
				printf("\ntask overdue ID %d at %d\n",curr->task.task_id,curr->task.completion_time);
			}
			// SET USER TASK PRIORITY AGAIN
			set_dds_Task_Priority(active_List);
		}

		if(xActivatedMember == request_list_queue)
		{
			xQueueReceive(xActivatedMember, &request_List, 0);
			struct dd_task_list *response = NULL;
			switch (request_List.list_Type)
			{
			case ACTIVE_LIST:
				response = active_List;
				break;
			case COMPLETED_LIST:
				response = completed_List;
				break;
			case OVERDUE_LIST:
				response = overdue_List;
			break;
			default:
				printf("Invalid list request");
				break;
			}
		// send back response
		if(xQueueSend(request_List.response_Queue, &response, portMAX_DELAY)!=pdPASS){
			printf("Could not send to response queue");
			}
		}
	}

}

// generates a new task based on period.
static void DDS_Task_Gen_Task(void *pvParameters){
	// initial set up.
	static struct dd_task task1_info, task2_info, task3_info; // stores information for timers
	// for easy switching between test benches
	switch(BENCH){
		case 1:
			task1_info.period = pdMS_TO_TICKS(500);
			task2_info.period = pdMS_TO_TICKS(500);
			task3_info.period = pdMS_TO_TICKS(750);
		break;
		case 2:
			task1_info.period = pdMS_TO_TICKS(250);
			task2_info.period = pdMS_TO_TICKS(500);
			task3_info.period = pdMS_TO_TICKS(750);
		break;
		case 3:
			task1_info.period = pdMS_TO_TICKS(500);
			task2_info.period = pdMS_TO_TICKS(500);
			task3_info.period = pdMS_TO_TICKS(500);
		break;
	}
	// assign ID for each task
	task1_info.task_id=1;
	task2_info.task_id=2;
	task3_info.task_id=3;
	// assign task type
	task1_info.type = PERIODIC;
	task2_info.type = PERIODIC;
	task3_info.type = PERIODIC;
	// Ftasks // These tasks just turn an led on for the requested period of time.
	xTaskCreate(F_task1,"Task1",configMINIMAL_STACK_SIZE,NULL,1,&task1_info.t_handle); // generator has higher priority than tasks when is created so it won't be preempted.
	xTaskCreate(F_task2,"Task2",configMINIMAL_STACK_SIZE,NULL,1,&task2_info.t_handle);
	xTaskCreate(F_task3,"Task3",configMINIMAL_STACK_SIZE,NULL,1,&task3_info.t_handle);
	uint32_t current_Time = xTaskGetTickCount();


	// Create timers for each task period
	TimerHandle_t timer1 = xTimerCreate("Task1 Timer",task1_info.period,pdTRUE,task1_info.task_id,generate_Timer_Callback);
	TimerHandle_t timer2 = xTimerCreate("Task2 Timer",task2_info.period,pdTRUE,task2_info.task_id,generate_Timer_Callback);
	TimerHandle_t timer3 = xTimerCreate("Task3 Timer",task3_info.period,pdTRUE,task3_info.task_id,generate_Timer_Callback);

	// Initial set up with absolute deadline
	create_dd_task(task1_info.t_handle, task1_info.type, task1_info.task_id, current_Time + task1_info.period);
	create_dd_task(task2_info.t_handle, task2_info.type, task2_info.task_id, current_Time + task2_info.period);
	create_dd_task(task3_info.t_handle, task3_info.type, task3_info.task_id, current_Time + task3_info.period);
	//start the timers
	xTimerStart(timer1, 0);
	xTimerStart(timer2, 0);
	xTimerStart(timer3, 0);
	while(1){
		uint32_t taskID;
		if (xQueueReceive(generate_queue, &taskID, portMAX_DELAY) == pdPASS) // task is blocked until at timer goes off.
		{
			switch (taskID)
			{
			case 1:
			current_Time = xTaskGetTickCount(); // update current time
			xTaskCreate(F_task1,"Task1",configMINIMAL_STACK_SIZE,NULL,1,&task1_info.t_handle);
			create_dd_task(task1_info.t_handle, task1_info.type, task1_info.task_id, current_Time + task1_info.period);
				break;
			case 2:
			current_Time = xTaskGetTickCount(); // update current time
			xTaskCreate(F_task2,"Task2",configMINIMAL_STACK_SIZE,NULL,1,&task2_info.t_handle);
			create_dd_task(task2_info.t_handle, task2_info.type, task2_info.task_id, current_Time + task2_info.period);
				break;
			case 3:
			current_Time = xTaskGetTickCount(); // update current time
			xTaskCreate(F_task3,"Task3",configMINIMAL_STACK_SIZE,NULL,1,&task3_info.t_handle);
			create_dd_task(task3_info.t_handle, task3_info.type, task3_info.task_id, current_Time + task3_info.period);
				break;
			default:
				printf("invalid task id for generation");
				break;
			}
		}
	}
}
// prints the number of tasks in active, overdue and completed
static void monitor_Task(void *pvParameters){
	const TickType_t xDelay = pdMS_TO_TICKS(1500); // updates every hyper period
	struct dd_task_list *active, *completed, *overdue;
	while(1){
		int activeCount=0, completedCount=0, overdueCount=0;
		active = get_active_dd_task_list();
		completed = get_complete_dd_task_list();
		overdue = get_overdue_dd_task_list();
		while(active != NULL){
			activeCount++;
			active = active->next_task;
		}
		while(completed != NULL){
			completedCount++;
			completed = completed->next_task;
		}
		while(overdue != NULL){
			overdueCount++;
			overdue = overdue->next_task;
		}
		printf("\nCurrent system state: \nActive Count:%d\nCompleted Cound:%d\nOverDue Count:%d\n",activeCount,completedCount,overdueCount);
		vTaskDelay(xDelay);
	}
}
/*--------------------User Tasks---------------------------------------*/
// need to do testing to determine correct for loop value for each execution time.
// can simply use an oscillosope. Use GPIO set up from previous lab and select one pin as output.
void F_task1(void *pvParameters){
	while(1){
		STM_EVAL_LEDOn(amber_led);
		if(BENCH == 1){
			uint32_t delay = pdMS_TO_TICKS(DELAY95); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}else if(BENCH==2){
			uint32_t delay = pdMS_TO_TICKS(DELAY95); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}else if(BENCH==3){
			uint32_t delay = pdMS_TO_TICKS(DELAY100); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}
		STM_EVAL_LEDOff(amber_led);
		// send task is completed to dds
		complete_dd_task(1); // task id 1
	}
}

void F_task2(void *pvParameters){
	while(1){
		STM_EVAL_LEDOn(blue_led);
		if(BENCH == 1){
			uint32_t delay = pdMS_TO_TICKS(DELAY150); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}else if(BENCH==2){
			uint32_t delay = pdMS_TO_TICKS(DELAY150); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}

		}else if(BENCH==3){
			uint32_t delay = pdMS_TO_TICKS(DELAY200); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}
		STM_EVAL_LEDOff(blue_led);
		complete_dd_task(2);
	}
}

void F_task3(void *pvParameters){
	while(1){
		STM_EVAL_LEDOn(red_led);
		if(BENCH == 1){
			uint32_t delay = pdMS_TO_TICKS(DELAY250); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}else if(BENCH==2){
			uint32_t delay = pdMS_TO_TICKS(DELAY250); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}else if(BENCH==3){
			uint32_t delay = pdMS_TO_TICKS(DELAY200); // time to execute in ticks
			uint32_t now = xTaskGetTickCount();
			uint32_t executeUntil = now+delay;
			while(executeUntil>now){
				now = xTaskGetTickCount();
			}
		}
		STM_EVAL_LEDOff(red_led);
		complete_dd_task(3);
	}
}
/*-----------------------Setup-------------------------*/
/*
 * Function sets up the ports for outputs and ADC
 * PC0 -> Red Light
 * PC1 -> Amber Light
 * PC2 -> Green Light
 * PC8 -> Shift Register Reset
 * PC7 -> Shift Register Clock
 * PC6 -> Shift Register Data
 * PC3 -> Potentionmeter Input
 * */
void GPIO_SetUp(){
	/* Initialize LEDs */
	STM_EVAL_LEDInit(amber_led);
	STM_EVAL_LEDInit(green_led);
	STM_EVAL_LEDInit(red_led);
	STM_EVAL_LEDInit(blue_led);
}



/*--------------------------Functions DDS---------------------*/
// creates dd_task struct and sends it to DDS to handle
void create_dd_task( TaskHandle_t t_handle,	char type,	uint32_t task_id, uint32_t absolute_deadline){
	struct dd_task newDD;

	newDD.t_handle=t_handle;
	newDD.type=type;
	newDD.task_id=task_id;
	newDD.release_time=NULL; // set by DDS
	newDD.absolute_deadline=absolute_deadline; // This will be in ticks. Must be calculated by the task generator function
	newDD.completion_time=NULL; // Set by DDS once task has run

	// Send the new task to the DDS task (e.g., through a FreeRTOS queue)
	if (xQueueSend(dd_task_queue, &newDD, portMAX_DELAY) != pdPASS){
		printf("Could not send to dd task queue");
	}
}

void complete_dd_task(uint32_t task_id){
	// Send the task ID of the completed task to the DDS through a queue
	//! portMax delay will block the function indefinetely till space opens. This should never occurr in the current setup
	if (xQueueSend(completed_task_queue, &task_id, portMAX_DELAY) != pdPASS){
		printf("Could not send to completed task Queue");
		}
}

struct dd_task_list* get_active_dd_task_list(void){
	QueueHandle_t sendBack_Queue = xQueueCreate(1,sizeof(struct dd_task_list*)); // temporary queue to receive information in
	struct dd_task_list *list = NULL;
	struct list_Request request = {ACTIVE_LIST, sendBack_Queue};
	if(xQueueSend(request_list_queue, &request, portMAX_DELAY) == pdPASS){ // send request
		xQueueReceive(sendBack_Queue, &list, portMAX_DELAY);
		vQueueDelete(sendBack_Queue);
	}else{
		printf("Could not send to sendback queue");
	}
	return list;
}
struct dd_task_list* get_complete_dd_task_list(void){
	QueueHandle_t sendBack_Queue = xQueueCreate(1,sizeof(struct dd_task_list*)); // temporary queue to receive information in
	struct dd_task_list *list = NULL;
	struct list_Request request = {COMPLETED_LIST, sendBack_Queue};
	xQueueSend(request_list_queue, &request, portMAX_DELAY); // send request
	xQueueReceive(sendBack_Queue, &list, portMAX_DELAY);
	vQueueDelete(sendBack_Queue);
	return list;
}
struct dd_task_list* get_overdue_dd_task_list(void){
	QueueHandle_t sendBack_Queue = xQueueCreate(1,sizeof(struct dd_task_list*)); // temporary queue to receive information in
	struct dd_task_list *list = NULL;
	struct list_Request request = {OVERDUE_LIST, sendBack_Queue};
	xQueueSend(request_list_queue, &request, portMAX_DELAY); // send request
	xQueueReceive(sendBack_Queue, &list, portMAX_DELAY);
	vQueueDelete(sendBack_Queue);
	return list;
}
/*--------------------------Helper Functions--------------------*/
// Sorts a doubly linked lists by earliest absolute deadline.
// The head of the list will always be the highest priority.
// Earliest to latest deadline by using insertion sort
// Reference: https://www.geeksforgeeks.org/insertion-sort-doubly-linked-list/
struct dd_task_list* EDF_Sort(struct dd_task_list* head){
	if(head == NULL) return head;

	struct dd_task_list* sorted = NULL; // head for sorted list
	struct dd_task_list* curr = head;

	while (curr != NULL){
		struct dd_task_list* next = curr->next_task;
		// need to handle cases where the absolute deadline is the same
		if(sorted == NULL || sorted->task.absolute_deadline > curr->task.absolute_deadline ||
			(sorted->task.absolute_deadline == curr->task.absolute_deadline &&
			 sorted->task.task_id > curr->task.task_id)){
			curr->next_task = sorted;

			if(sorted != NULL)
				sorted->prev_task = curr;

			sorted = curr;
			sorted->prev_task=NULL;
		}else{
			struct dd_task_list* current_sorted = sorted;

			//insert between elements
			while(current_sorted->next_task != NULL &&
				(current_sorted->next_task->task.absolute_deadline < curr->task.absolute_deadline ||
				(current_sorted->next_task->task.absolute_deadline == curr->task.absolute_deadline &&
				 current_sorted->next_task->task.task_id <= curr->task.task_id))){

				current_sorted = current_sorted->next_task;
			}

			curr->next_task = current_sorted->next_task;
			if(current_sorted->next_task != NULL){
				current_sorted->next_task->prev_task=curr;
			}

			current_sorted->next_task = curr;
			curr->prev_task = current_sorted;
		}
		curr = next;
	}
	return sorted;
}

// Takes in sorted active list. Sets head to high priority and the rest to low.
void set_dds_Task_Priority(struct dd_task_list* active_List)
{
	// EDF task gets highest priority
	if(active_List != NULL)
		vTaskPrioritySet(active_List->task.t_handle, TASK_HIGH_PRIORITY);
	// Lower priority of all other task
	struct dd_task_list *curr =active_List->next_task;
	while(curr != NULL){
		vTaskPrioritySet(curr->task.t_handle, TASK_LOW_PRIORITY);
		curr = curr->next_task;
	}
}
// sends message to overdue queue so that DDS can handle overdue task
void overdue_Timer_Callback(TimerHandle_t xTimer){
	uint32_t task_id = (uint32_t)pvTimerGetTimerID(xTimer); // the ID of the timer is set the same as the task
	if (xQueueSend(overdue_task_queue, &task_id, portMAX_DELAY) != pdPASS){
		printf("Could not send to overdue task Queue");
		}
}

// send id of timer to gen to indicate that a new instance of the task should be sent
void generate_Timer_Callback(TimerHandle_t xTimer){
	uint32_t task_id = (uint32_t)pvTimerGetTimerID(xTimer);
	if (xQueueSend(generate_queue, &task_id, portMAX_DELAY) != pdPASS){
		printf("Could not send to generate task Queue");
		}
}
void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	printf("Insufficient heap");
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
