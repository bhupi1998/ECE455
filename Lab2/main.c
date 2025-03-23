

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

#define TASK_HIGH_PRIORITY 10
#define TASK_LOW_PRIORITY 1

#define DDS_PRIORITY 11
#define GEN_PRIORITY 1
#define MONITOR_PRIORITY 1
// Executiong delays
#define DELAY95 10000
#define DELAY100 10000
#define DELAY150 10000
#define DELAY200 10000
#define DELAY250 10000
/*--------------------------------------------------------*/
 /* TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );



QueueHandle_t  dd_task_queue, completed_task_queue, overdue_task_queue;


/*-----------------------------------------------------------*/

/*------------------------Structs-----------------------*/
enum task_type {PERIODIC, APERIODIC};

struct dd_task {
	TaskHandle_t t_handle;
	task_type type;	
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	TimerHandle_t timer_Handle; // added so that associated timer can be found.
};

// using doubly linked lists for easy deletion.
struct dd_task_list {
	dd_task task;
	struct dd_task_list *next_task;
	struct dd_task_list *prev_task;

};
/*-------------------Functions-----------------------------*/
/*Function declarations*/
void create_dd_task( TaskHandle_t t_handle,	task_type type,	uint32_t task_id, uint32_t absolute_deadline);
void complete_dd_task(uint32_t task_id);
// return pointer of the head of the list
struct dd_task_list* get_active_dd_task_list(void);
struct dd_task_list* get_complete_dd_task_list(void);
struct dd_task_list* get_overdue_dd_task_list(void);
// Setting up gpio
void GPIO_SetUp();

// Sorting algorithm
struct dd_task_list* EDF_Sort(struct dd_task_list* head);

// set task priorities
void set_dds_Task_Priority(struct dd_task_list* active_List);

// handle overdue task
void overdue_Timer_Callback(TimerHandle_t xTimer);

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

	// space for 10 tasks might be excessive since the DDS will clear it out right away
	dd_task_queue = xQueueCreate(10, sizeof(struct dd_task));
	
	// contains taskID
	completed_task_queue = xQueueCreate(10, sizeof(uint32_t));

	// timer will write to this queue if task is overdue.
	overdue_task_queue = xQueueCreate(10, sizeof(uint32_t));
	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( dd_task_queue, "DD_TASK_Q" );
	vQueueAddToRegistry( completed_task_queue, "Comp_Q" );
	vQueueAddToRegistry( overdue_task_queue, "Over_Q" );


	// DDS Core
	xTaskCreate(deadline_Driven_Scheduler_Task, "DDS",configMINIMAL_STACK_SIZE,NULL,DDS_PRIORITY,NULL);
	xTaskCreate(DDS_Task_Gen_Task, "Task Generator",configMINIMAL_STACK_SIZE,NULL,GEN_PRIORITY,NULL);
	xTaskCreate(monitor_Task, "Monitor Task",configMINIMAL_STACK_SIZE,NULL,MONITOR_PRIORITY,NULL);

	// USER Defined Tasks
	// Ftasks // These tasks just turn an led on for the requested period of time.
	xTaskCreate(F_task1,"Task1",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(F_task2,"Task2",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(F_task3,"Task3",configMINIMAL_STACK_SIZE,NULL,1,NULL);


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*---------------------DDS-Tasks----------------------------------------*/

static void deadline_Driven_Scheduler_Task(void *pvParameters){
	// these lists contain the head of the list
	static dd_task_list *active_List = NULL;
	static dd_task_list *completed_List = NULL;
	static dd_task_list *overdue_List = NULL;
	
	struct dd_task received_task;

	uint32_t completed_task_ID;
	uint32_t overdue_task_ID;
	TimerHandle_t overdue_Timer;
	while(1){
		// a new task was added
		if (xQueueReceive(dd_task_queue, &received_task, portMAX_DELAY) == pdPASS)
		{
			// NEED TO ASSIGN RELEASE TIME 
			received_task.release_time = xTaskGetTickCount();
			// Create timer to check if task is overdue
			TickType_t timerPeriod = received_task.absolute_deadline-received_task.release_time;
			overdue_Timer= xTimerCreate("Overdue Timer",timerPeriod,pdFALSE,received_task.task_id,overdue_Timer_Callback);
			received_task.timer_Handle= overdue_Timer; // the handle will be unique to each instance of timer

			// ADD TO ACTIVE LIST
			struct dd_task_list *new_Node = pvPortMalloc(sizeof(dd_task_node)); // never freed since they just get moved in completed or overdue later

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
		}
		// a task has completed. Receives the ID of the task
		if(xQueueReceive(completed_task_queue, &completed_task_ID, portMAX_DELAY) == pdPASS){
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
			}
			// SET USER TASK PRIORITY AGAIN
			set_dds_Task_Priority(active_List);

		}
		// Message from timer about an overdue task received
		if (xQueueReceive(overdue_task_queue, &overdue_task_ID, portMAX_DELAY) == pdPASS)
		{
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
			}
			// SET USER TASK PRIORITY AGAIN
			set_dds_Task_Priority(active_List);
		}
	}

}
static void DDS_Task_Gen_Task(void *pvParameters){

}
static void monitor_Task(void *pvParameters){

}
/*--------------------User Tasks---------------------------------------*/
// need to do testing to determine correct for loop value for each execution time.
// can simply use an oscillosope. Use GPIO set up from previous lab and select one pin as output.
static void F_task1(void *pvParameters){
	while(1){
		STM_EVAL_LEDOn(amber_led);
		if(BENCH == 1){
			for(i=0;i<DELAY95;i++){}
		}else if(BENCH==2){
			for(i=0;i<DELAY95;i++){}
		}else if(BENCH==3){
			for(i=0;i<DELAY100;i++){}
		}
		STM_EVAL_LEDOff(amber_led);
	}
}

static void F_task2(void *pvParameters){
	while(1){
		STM_EVAL_LEDOn(blue_led);
		if(BENCH == 1){
			for(i=0;i<DELAY150;i++){}
		}else if(BENCH==2){
			for(i=0;i<DELAY150;i++){}
		}else if(BENCH==3){
			for(i=0;i<DELAY200;i++){}
		}
		STM_EVAL_LEDOff(blue_led);
	}
}

static void F_task3(void *pvParameters){
	while(1){
		STM_EVAL_LEDOn(red_led);
		if(BENCH == 1){
			for(i=0;i<DELAY250;i++){}
		}else if(BENCH==2){
			for(i=0;i<DELAY250;i++){}
		}else if(BENCH==3){
			for(i=0;i<DELAY200;i++){}
		}
		STM_EVAL_LEDOff(red_led);
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
void create_dd_task( TaskHandle_t t_handle,	task_type type,	uint32_t task_id, uint32_t absolute_deadline){
	struct dd_task newDD;

	newDD.t_handle=t_handle;
	newDD.type=type;	
	newDD.task_id=task_id;
	newDD.release_time; // set by DDS
	newDD.absolute_deadline=absolute_deadline; // This will be in ticks. Must be calculated by the task generator function
	newDD.completion_time; // Set by DDS once task has run

	// Send the new task to the DDS task (e.g., through a FreeRTOS queue)
	if (xQueueSend(dd_task_queue, &newDD, portMAX_DELAY) != pdPASS){
		printf("Could not send to dd task queue");
	}
}

void complete_dd_task(uint32_t task_id){
	// Send the task ID of the completed task to the DDS through a queue
	//! portMax delay will block the function indefinetely till space opens. This should never occurr in the current setup
	if (xQueueSend(completed_task_queue, task_id, portMAX_DELAY) != pdPASS){
		printf("Could not send to completed task Queue");
		}
}

// idk what the * are for
// will probably return a pointer
**dd_task_list get_active_dd_task_list(void);
**dd_task_list get_complete_dd_task_list(void);
**dd_task_list get_overdue_dd_task_list(void);
/*--------------------------Helper Functions--------------------*/
// Sorts a single linked lists by earliest absolute deadline.
// The head of the list will always be the highest priority.
// Earliest to latest deadline by using insertion sort
// Reference: https://www.geeksforgeeks.org/insertion-sort-doubly-linked-list/
struct dd_task_list* EDF_Sort(struct dd_task_list* head){
	if(head == NULL) return head;

	struct dd_task_list* sorted = NULL; // head for sorted list
	struct dd_task_list* curr = head;

	while (curr != NULL){
		struct dd_task_list* next = curr->next_task;

		if(sorted == NULL || sorted->task.absolute_deadline > curr->task.absolute_deadline){
			curr->next_task = sorted;

			if(sorted != NULL) 
				sorted->prev_task = curr;

			sorted = curr;
			sorted->prev_task=NULL;
		}else{
			struct dd_task_list* current_sorted = sorted;

			//insert between elements
			while(current_sorted->next_task != NULL && current_sorted->next_task->task.absolute_deadline < curr->task.absolute_deadline)
			{
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
void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
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
