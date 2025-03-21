

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

/*--------------------------------------------------------*/
 /* TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );



xQueueHandle xQueue_Traffic_State = 0;
xQueueHandle xQueue_Pot_Val = 0;
xQueueHandle xQueue_Add_Traffic = 0;
xQueueHandle xQueue_Traffic_Timing = 0;
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
/*-----------------------------------------------------------*/
/*Function declarations*/
void create_dd_task( TaskHandle_t t_handle,
	task_type type,
	uint32_t task_id,
	uint32_t absolute_deadline,
	);
void delete_dd_task(uint32_t task_id);

// idk what the * are for
// will probably return a pointer
**dd_task_list get_active_dd_task_list(void);
**dd_task_list get_complete_dd_task_list(void);
**dd_task_list get_overdue_dd_task_list(void);
// Setting up gpio
void GPIO_SetUp();




/*-----------------------------------------------------------*/
/*----------------------------TLS Functions----------------------*/
//static void traffic_Flow_Adjustment_Task(void *pvParameters);

/*-----------------------------------------------------------*/
int main(void)
{
	GPIO_SetUp(); // will need to be configured for led testing


	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

	//xQueue_Traffic_State = xQueueCreate(MAX_TRAFFIC_LIGHT_QUEUE,sizeof(uint8_t)); // just need to represent 0,1,2 for uint8 should be sufficient

	/* Add to the registry, for the benefit of kernel aware debugging. */
	//vQueueAddToRegistry( xQueue_Traffic_State, "TrafficLightStateQueue" );


	// DDS Core
	xTaskCreate(deadline_Driven_Scheduler_Task, "DDS",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(DDS_Task_Gen_Task, "Task Generator",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(monitor_Task, "Monitor Task",configMINIMAL_STACK_SIZE,NULL,1,NULL);

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

/*---------------------Tasks----------------------------------------*/

static void deadline_Driven_Scheduler_Task(void *pvParameters){

}
static void DDS_Task_Gen_Task(void *pvParameters){

}
static void monitor_Task(void *pvParameters){

}

/*-----------------Helper Functions & Setup-------------------------*/
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



/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

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
