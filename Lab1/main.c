/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
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

/*-----------------------------------------------------------*/
#define MAX_TRAFFIC_LIGHT_QUEUE 2 // 1 would be fine
#define MAX_POT_QUEUE 2 //1 would be fine

#define RED_LIGHT GPIO_Pin_0
#define AMBER_LIGHT GPIO_Pin_1
#define GREEN_LIGHT GPIO_Pin_2


#define CLR GPIOC,GPIO_Pin_8 // Reset Pin
#define CLK GPIOC,GPIO_Pin_7 // Clock Pin
#define DATA GPIOC,GPIO_Pin_6 // Data Pin
#define LED_GREEN GPIOC,GPIO_Pin_0
#define LED_AMBER GPIOC,GPIO_Pin_1
#define LED_RED GPIOC,GPIO_Pin_2
/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Manager_Task( void *pvParameters );
static void Blue_LED_Controller_Task( void *pvParameters );
static void Green_LED_Controller_Task( void *pvParameters );
static void Red_LED_Controller_Task( void *pvParameters );
static void Amber_LED_Controller_Task( void *pvParameters );

xQueueHandle xQueue_handle = 0;


/*-----------------------------------------------------------*/
/*Function declarations*/
// Setting up gpio
void GPIO_SetUp();

// Send High onto data line. Indicates a car is present
void data_High();
// Send Low onto data line. indicates no car is present
void data_Low();
// Clears all lights.
void data_Rst(); // !!Currently untested

/*-----------------------------------------------------------*/
/*----------------------------TLS Functions----------------------*/
static void traffic_Flow_Adjustment_Task(void *pvParameters);
static void traffic_Generator_Task(void *pvParameters);
static void traffic_Light_State_Task(void *pvParameters);
static void system_Display_Task(void *pvParameters);
/*-----------------------------------------------------------*/
int main(void)
{
	GPIO_SetUp();
	printf("GPIO Set up Done\n");
	GPIO_SetBits(LED_RED);
	GPIO_SetBits(LED_AMBER);
	GPIO_SetBits(LED_GREEN);

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();

    xQueue_Traffic_Lights = xQueueCreate(MAX_TRAFFIC_LIGHT_QUEUE,sizeof(uint8_t)); // just need to represent 0,1,2 for uint8 should be sufficient
    xQueue_Pot_Val = xQueueCreate(MAX_POT_QUEUE,sizeof(uint16_t));

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue_Traffic_Lights, "TrafficQueue" );
    vQueueAddToRegistry( xQueue_Pot_Val, "PotentiometerQueue" );

	xTaskCreate(traffic_Flow_Adjustment_Task, "Potentiometer Read",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
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
	// Defining typedef for all outputs at once
	GPIO_InitTypeDef GPIO_Output_Conf;
	GPIO_StructInit(&GPIO_Output_Conf);
	GPIO_Output_Conf.GPIO_Pin = (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8);
	GPIO_Output_Conf.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_Output_Conf.GPIO_OType = GPIO_OType_PP; //push pull
	GPIO_Output_Conf.GPIO_PuPd = GPIO_PuPd_NOPULL; // no pull ups since push pull is enabled
	GPIO_Output_Conf.GPIO_Speed = GPIO_Speed_2MHz; // Low Speed. High speed introduces noise and used too much current.

	// Defining typedef for ADC
	GPIO_InitTypeDef GPIO_ADC_Conf;
	GPIO_StructInit(&GPIO_ADC_Conf);
	GPIO_ADC_Conf.GPIO_Pin = GPIO_Pin_3;
	GPIO_ADC_Conf.GPIO_Mode = GPIO_Mode_AN;
	GPIO_ADC_Conf.GPIO_PuPd = GPIO_PuPd_NOPULL; // no pull ups since push pull is enabled
	GPIO_ADC_Conf.GPIO_Speed = GPIO_Speed_2MHz; // Low Speed

	ADC_InitTypeDef ADC_Conf;
	ADC_StructInit(&ADC_Conf);
	ADC_Conf.ADC_Resolution = ADC_Resolution_12b;
	ADC_Conf.ADC_DataAlign =  ADC_DataAlign_Right;


	// Enable Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	// GPIO Init
	// Set Pins
	// PC0,1,2,6,7,8 as digital outputs
	GPIO_Init(GPIOC,&GPIO_Output_Conf);
	// PC3 as analog input for ADC
	GPIO_Init(GPIOC,&GPIO_ADC_Conf);
	//ADC_DeInit(); //reset ADC to default
	ADC_Init(ADC1,&ADC_Conf); // Initialize ADC
	ADC_Cmd(ADC1,ENABLE); //enable ADC
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,1,ADC_SampleTime_15Cycles); //set 15 cycles randomly
}

// Send High onto data
void data_High(){
	GPIO_ResetBits(CLK); // set clock low
	GPIO_SetBits(CLR); // set reset to high
	GPIO_SetBits(DATA); // set data pin high
	GPIO_SetBits(CLK); // set clock high
	for(int i = 0; i<10;i++){} // delay
	GPIO_ResetBits(CLK); // set clock low
	for(int i = 0; i<10;i++){} // delay
}
// Send Low onto data line
void data_Low(){
	GPIO_ResetBits(CLK); // set clock low
	GPIO_SetBits(CLR); // set reset to high
	GPIO_ResetBits(DATA); // set data pin low
	GPIO_SetBits(CLK); // set clock high
	for(int i = 0; i<10;i++){}// delay
	GPIO_ResetBits(CLK); // set clock low
	for(int i = 0; i<10;i++){} // delay
}
// Reset all LEDS off
void data_Rst(){
	GPIO_ResetBits(CLR); // drives the reset line low. All other values are irrelevant
	for(int i = 0; i<10;i++){} // delay
	GPIO_SetBits(CLR); // set reset to high
}
/*-----------------------------------------------------------*/

/*-----------------------TLS Tasks---------------------------*/
// Reads the state of the potentiometer and sends it to a queue 
static void traffic_Flow_Adjustment_Task(void *pvParameters){
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(250); // setting a 250ms delay for now
	// need to initialize with the current tick time. Managed by vTaskDelayUntil() afterwards
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
	// Read ADC
	ADC_SoftwareStartConv(ADC1);
	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)){} //wait for conversion to finish
	int ADC_Value = ADC_GetConversionValue(ADC1);
	printf(" %d \n", ADC_Value); // for testing
	// Sends value onto a queue
    // Potentiometer values should be sent as the come and should never be full
    int xStatus = xQueueSend(xQueue_Pot_Val,&ADC_Value,0);
    if(xStatus != pdPASS)
    {
        printf("Could not sent to the queue. \n");
    }
	// go into suspended for 250 ms
	vTaskDelayUntil(&xLastWakeTime, xDelay);
	}
}
// Randomly generates new traffic
static void traffic_Generator_Task(void *pvParameters){
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(250); // setting a 250ms delay for now
	// need to initialize with the current tick time. Managed by vTaskDelayUntil() afterwards
	xLastWakeTime = xTaskGetTickCount();
    uint16_t ADC_Val;
    uint16_t random_Val;
    while(1){
        xQueueReceive(xQueue_Pot_Val,&ADC_Val,0); // get ADC Value
        ADC_Val = (float)ADC_Val/(float)4096 * 100; // results in value between 0 and 100
        // Reference for rand() : https://www.geeksforgeeks.org/c-rand-function/        
        random_Val = rand() % 101; // generates a random value between 0 and 100
        if(random_Val<ADC_Val){
            // add a car to the traffic
            // send it to the traffic queue
        }
        else{
            // don't add a car to the traffic
        }

        // go into suspended for 250 ms
	    vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

/*-----------------------------------------------------------*/

static void Manager_Task( void *pvParameters )
{
	uint16_t tx_data = amber;


	while(1)
	{

		if(tx_data == amber)
			STM_EVAL_LEDOn(amber_led);
		if(tx_data == green)
			STM_EVAL_LEDOn(green_led);
		if(tx_data == red)
			STM_EVAL_LEDOn(red_led);
		if(tx_data == blue)
			STM_EVAL_LEDOn(blue_led);

		if( xQueueSend(xQueue_handle,&tx_data,1000))
		{
			printf("Manager: %u ON!\n", tx_data);
			if(++tx_data == 4)
				tx_data = 0;
			vTaskDelay(1000);
		}
		else
		{
			printf("Manager Failed!\n");
		}
	}
}

/*-----------------------------------------------------------*/

static void Blue_LED_Controller_Task( void *pvParameters )
{
	uint16_t rx_data;
	while(1)
	{
		if(xQueueReceive(xQueue_handle, &rx_data, 500))
		{
			if(rx_data == blue)
			{
				vTaskDelay(250);
				STM_EVAL_LEDOff(blue_led);
				printf("Blue Off.\n");
			}
			else
			{
				if( xQueueSend(xQueue_handle,&rx_data,1000))
					{
						printf("BlueTask GRP (%u).\n", rx_data); // Got wwrong Package
						vTaskDelay(500);
					}
			}
		}
	}
}


/*-----------------------------------------------------------*/

static void Green_LED_Controller_Task( void *pvParameters )
{
	uint16_t rx_data;
	while(1)
	{
		if(xQueueReceive(xQueue_handle, &rx_data, 500))
		{
			if(rx_data == green)
			{
				vTaskDelay(250);
				STM_EVAL_LEDOff(green_led);
				printf("Green Off.\n");
			}
			else
			{
				if( xQueueSend(xQueue_handle,&rx_data,1000))
					{
						printf("GreenTask GRP (%u).\n", rx_data); // Got wrong Package
						vTaskDelay(500);
					}
			}
		}
	}
}

/*-----------------------------------------------------------*/

static void Red_LED_Controller_Task( void *pvParameters )
{
	uint16_t rx_data;
	while(1)
	{
		if(xQueueReceive(xQueue_handle, &rx_data, 500))
		{
			if(rx_data == red)
			{
				vTaskDelay(250);
				STM_EVAL_LEDOff(red_led);
				printf("Red off.\n");
			}
			else
			{
				if( xQueueSend(xQueue_handle,&rx_data,1000))
					{
						printf("RedTask GRP (%u).\n", rx_data); // Got wrong Package
						vTaskDelay(500);
					}
			}
		}
	}
}


/*-----------------------------------------------------------*/

static void Amber_LED_Controller_Task( void *pvParameters )
{
	uint16_t rx_data;
	while(1)
	{
		if(xQueueReceive(xQueue_handle, &rx_data, 500))
		{
			if(rx_data == amber)
			{
				vTaskDelay(250);
				STM_EVAL_LEDOff(amber_led);
				printf("Amber Off.\n");
			}
			else
			{
				if( xQueueSend(xQueue_handle,&rx_data,1000))
					{
						printf("AmberTask GRP (%u).\n", rx_data); // Got wrong Package
						vTaskDelay(500);
					}
			}
		}
	}
}


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
