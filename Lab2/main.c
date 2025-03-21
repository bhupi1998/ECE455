

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



QueueHandle_t  dd_task_queue, completed_task_queue;


/*-----------------------------------------------------------*/

/*------------------------Structs-----------------------*/
enum task_type {PERIODIC, APERIODIC}

struct dd_task {
	TaskHandle_t t_handle;
	task_type type;	
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
}

struct dd_task_list {
	dd_task task;
	struct dd_task_list *next_task
}
/*-------------------Functions-----------------------------*/
/*Function declarations*/
void create_dd_task( TaskHandle_t t_handle,	task_type type,	uint32_t task_id, uint32_t absolute_deadline);
void complete_dd_task(uint32_t task_id);
// idk what the * are for
// will probably return a pointer
**dd_task_list get_active_dd_task_list(void);
**dd_task_list get_complete_dd_task_list(void);
**dd_task_list get_overdue_dd_task_list(void);
// Setting up gpio
void GPIO_SetUp();




/*-----------------------------------------------------------*/
/*----------------------------DDS Functions----------------------*/
static void deadline_Driven_Scheduler_Task(void *pvParameters)
static void DDS_Task_Gen_Task(void *pvParameters)
static void monitor_Task(void *pvParameters)
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

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( dd_task_queue, "DD_TASK_Q" );


	// DDS Core
	xTaskCreate(deadline_Driven_Scheduler_Task, "DDS",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(DDS_Task_Gen_Task, "Task Generator",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(monitor_Task, "Monitor Task",configMINIMAL_STACK_SIZE,NULL,1,NULL);

	// Ftasks // These tasks just turn an led on for the requested period of time.
	xTaskCreate(F_task1,"Task1",configMINIMAL_STACK_SIZE,NULL,1,NULL)
	xTaskCreate(F_task2,"Task2",configMINIMAL_STACK_SIZE,NULL,1,NULL)
	xTaskCreate(F_task3,"Task3",configMINIMAL_STACK_SIZE,NULL,1,NULL)

	/*
	led usage
	STM_EVAL_LEDOn(amber_led);
	STM_EVAL_LEDOff(blue_led);
	*/

	// USER Defined Tasks
	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

/*---------------------DDS-Tasks----------------------------------------*/

static void deadline_Driven_Scheduler_Task(void *pvParameters){
	// these lists contain the head of the list
	static dd_task_list active_List = NULL;
	static dd_task_list completed_List = NULL;
	static dd_task_list overdue_List = NULL;

	struct dd_task received_task;

	while(1){
		// a new task was added
		if (xQueueReceive(dd_task_queue, &received_task, portMAX_DELAY) == pdPASS)
		{
			// NEED TO ASSIGN RELEASE TIME 
			received_task.release_time = xTaskGetTickCount();
			// ADD TO ACTIVE LIST
			//! i don't think a linked list is being created here!
			dd_task_list *new_Node = pvPortMalloc(sizeof(dd_task_node));
			new_Node.task = received_task; // make new linked list item
			new_Node.next_task = NULL; // make it point to the previous tail of the list
			active_List = new_Node; // update the head

			// SORT BY DEADLINE

			// SET USER TASK PRIORITY
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

static void F_task1(void *pvParameters){
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
	newDD.release_time; // current tick count
	newDD.absolute_deadline=absolute_deadline; // This will be in ticks. Must be calculated by the task generator function
	newDD.completion_time; // leave empty for now

	// Send the new task to the DDS task (e.g., through a FreeRTOS queue)
	if (xQueueSend(dd_task_queue, &newDD, portMAX_DELAY) != pdPASS){
		printf("Could not send to dd task queue");
	}
}

void complete_dd_task(uint32_t task_id){
	// Send the task ID of the completed task to the DDS through a queue
	//! portMax delay will block the function indefinetely till space opens. This should never occurr in the current setup
	if (xQueueSend(completed_task_queue, task_id, portMAX_DELAY) != pdPASS){
		printf("Could not send to completed task Queue")
		}
}

// idk what the * are for
// will probably return a pointer
**dd_task_list get_active_dd_task_list(void);
**dd_task_list get_complete_dd_task_list(void);
**dd_task_list get_overdue_dd_task_list(void);
/*--------------------------Helper Functions--------------------*/

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
