/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Modified for use with NXT by Kristian Lien, Spring 2017
//                  Mapping functionality added by Geir Eikeland, Spring 2018
//                  Credit is given where credit is due.
// Purpose:
// NXT Robot with FreeRTOS implementation in the collaborating robots
// project.
// In FreeRTOS each thread of execution is known as tasks
// and are written as C functions
//
//
// TASKS IMPLEMENTED
// Communication:               vMainCommunicationTask
// Sensors:                     vMainSensorTowerTask
// Robot control:               vMainPoseControllerTask
// Position estimator:          vMainPoseEstimatorTask
// Mapping:						vMainMappingTask
// Stack overflow handling:     vApplicationStackOverflowHook
//
// See FreeRTOSConfig.h for scheduler settings
// See defines.h for all definitions
//
/************************************************************************/


/* KERNEL INCLUDES */
#include "FreeRTOS.h" /* Must come first */
#include "FreeRTOSConfig.h"
#include "task.h"     /* RTOS task related API prototypes */
#include "semphr.h"   /* Semaphore related API prototypes */
#include "queue.h"    /* RTOS queue related API prototypes */

#include "display.h"
#include "types.h"
#include "defines.h"
#include "nxt.h"
#include "led.h"
#include "io.h"
#include "arq.h"
#include "simple_protocol.h"
#include "network.h"
#include "communication.h"
#include "sensor_tower.h"
#include "pose_estimator.h"
#include "pose_controller.h"
//#include "navigation.h"
#include "mapping.h"
#include "motor.h"

/* Semaphore handles */
SemaphoreHandle_t xCommandReadyBSem;

/* Queue handles */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t movementStatusQ = 0;
QueueHandle_t globalWheelTicksQ = 0;
QueueHandle_t globalPoseQ = 0;
QueueHandle_t mappingMeasurementQ = 0;
//QueueHandle_t navigationmappingMeasurementQ = 0;

/* Task handles */
TaskHandle_t xPoseCtrlTask = NULL;
TaskHandle_t xMappingTask = NULL;

/* Global flags to indicate system status */
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

#ifdef DEBUG
	#warning DEBUG IS ACTIVE
#endif

/* Defines for enabling system tasks and functionality */
//#define COMPASS_CALIBRATE		// Compass calibration task
//#define SENSOR_CALIBRATE		// Sensor calibration task
#define MAPPING 		// Mapping task
//#define NAVIGATION 		// Navigation task 
#define SEND_LINE 		// Sending of lines to server in mapping task
//#define SEND_UPDATE	// Sending of IR data to server in sensor tower task
//#define MANUAL		// Manual drive mode


/**
In case of stack overflow, disable all interrupts and handle it.
*/
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	led_set(LED_GREEN);
	led_set(LED_YELLOW);
	led_set(LED_RED);

	#ifdef DEBUG
	//debug("s", "Stack overflow!\n");
		//#warning STACK OVERFLOW
	#endif
	
	for (;;);
}

// Function executed when configASSERT is called. Used for debugging.
// Enabled in FreeRTOSConfig.h
void vAssertCalled(void)
{
	//static portBASE_TYPE xPrinted = pdFALSE;
	volatile uint32_t ulSetToNonZeroInDebuggerToContinue = 0;

		/* Parameters are not used. */
		//( void ) ulLine;
		//( void ) pcFileName;

		//gHandshook = FALSE;
		uint8_t dir = 0;
		vMotorMovementSwitch(0, 0, &dir, &dir);
		vTaskDelay(10);
		led_set(LED_YELLOW);
		vTaskDelay(10);

		taskENTER_CRITICAL();
		{
				/* You can step out of this function to debug the assertion by using
				the debugger to set ulSetToNonZeroInDebuggerToContinue to a non-zero
				value. */
				while( ulSetToNonZeroInDebuggerToContinue == 0 )
				{
				}
		}
		taskEXIT_CRITICAL();
		led_clear(LED_YELLOW);
		//taskDISABLE_INTERRUPTS();
		//for (;;);
}


/* The main function */
int main(void)
{
	nxt_init();
	network_init();
	arq_init();
	simple_p_init(server_receiver);

	/* Set red LED on to indicate INIT is ongoing */
	led_set(LED_RED);
	
	/* Initialize queues and semaphores */
	movementQ = xQueueCreate(2, sizeof(uint8_t)); // For sending movements to vMainMovementTask (used in compass task only)
	poseControllerQ = xQueueCreate(1, sizeof(point_t)); // For setpoints to controller
	movementStatusQ = xQueueCreate(1, sizeof(uint8_t)); // For robot status
	globalWheelTicksQ = xQueueCreate(1, sizeof(wheel_ticks_t));
	globalPoseQ = xQueueCreate(1, sizeof(pose_t)); // For storing and passing the global pose estimate
	mappingMeasurementQ = xQueueCreate(3, sizeof(measurement_t));
//  actuationQ = xQueueCreate(2, sizeof)

	xCommandReadyBSem = xSemaphoreCreateBinary();

	/* For debugging using the FreeRTOS-aware plugin in IAR embedded studio. */
	vQueueAddToRegistry(movementQ, "Movement queue");
	vQueueAddToRegistry(poseControllerQ, "Pose controller queue");
	vQueueAddToRegistry(movementStatusQ, "Scan status queue");
	vQueueAddToRegistry(globalWheelTicksQ, "Global wheel ticks queue");
	vQueueAddToRegistry(globalPoseQ, "Global pose queue");
	vQueueAddToRegistry(mappingMeasurementQ, "Measurement queue");
	vQueueAddToRegistry(xCommandReadyBSem, "Command ready semaphore");

	/* Create tasks */
	BaseType_t ret;
	#ifndef DEBUG
	xTaskCreate(vMainCommunicationTask, "Comm", 256, NULL, 3, NULL);  // Dependant on IO, sends instructions to other tasks
	#endif /* DEBUG */

#ifndef COMPASS_CALIBRATE
	xTaskCreate(vMainPoseControllerTask, "PoseCon", 256, NULL, 2, &xPoseCtrlTask);// Dependant on estimator, sends instructions to movement task //2
	xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 256, NULL, 2, NULL); // Independent task,

	#ifdef MAPPING
	xTaskCreate(vMainMappingTask, "Mapping", 256, NULL, 1, &xMappingTask);
	#endif /* MAPPING */

	#ifdef NAVIGATION
	xTaskCreate(vMainNavigationTask, "Navigation", 128, NULL, 1, NULL);
	#endif /* NAVIGATION */

	ret = xTaskCreate(vMainSensorTowerTask,"Tower", 128, NULL, 3, NULL); // Independent task, but use pose updates from estimator //1
	//ret = pdPASS;
#endif /* COMPASS_CALIBRATE */
	if (ret != pdPASS) {
		display_goto_xy(0,2);
		display_string("Error");
		display_update();
	}
	
#ifdef COMPASS_CALIBRATE
	#include "calibration.h"
	display_goto_xy(0,1);
	display_string("\n \t WARNING \t !\n");
	display_string("COMPASS CALIBRATION!\n");
	display_string("{S,CON} to begin\n");
	xTaskCreate(compassTask, "compasscal", 2000, NULL, 4, NULL); // Task used for compass calibration, dependant on communication and movement task
#endif /* COMPASS_CALIBRATE */
	
	/* Indicate that init is complete */
	led_clear(LED_RED);
	
	/* Start scheduler */
	display_goto_xy(0,0);
	display_string("Init complete");
	display_update();

	vTaskStartScheduler();
	
	/*  MCU is out of RAM if the program comes here */
	while(1) {
		led_toggle(LED_GREEN);
		led_toggle(LED_RED);
		led_toggle(LED_YELLOW);
	}

}