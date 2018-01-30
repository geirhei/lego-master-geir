/************************************************************************/
// File:			main.c
// Author:			Erlend Ese, NTNU Spring 2016
//                  Modified for use with NXT by Kristian Lien, Spring 2017
//                  Modified by Geir Eikeland, Fall 2017
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
// Stack overflow handling:     vApplicationStackOverflowHook
//
// See FreeRTOSConfig.h for scheduler settings
// See defines.h for all definitions
//
/************************************************************************/

/* KERNEL INCLUDES */
#include "FreeRTOS.h" /* Must come first */
#include "task.h"     /* RTOS task related API prototypes */
#include "semphr.h"   /* Semaphore related API prototypes */
#include "queue.h"    /* RTOS queue related API prototypes */
//#include "event_groups.h" /* RTOS event group related APT prototypes */

//#include <stdlib.h>         // For itoa();
//#include <string.h>         // For stringstuff
//#include <stdio.h>
//#include <math.h>

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

/* Semaphore handles */
SemaphoreHandle_t xCommandReadyBSem;
//SemaphoreHandle_t xBeginMergeBSem;

/* Queues */
QueueHandle_t movementQ = 0;
QueueHandle_t poseControllerQ = 0;
QueueHandle_t scanStatusQ = 0;
QueueHandle_t globalWheelTicksQ = 0;
QueueHandle_t globalPoseQ = 0;
QueueHandle_t measurementQ = 0;
//QueueHandle_t actuationQ = 0;

/* Task handles */
TaskHandle_t xPoseCtrlTask = NULL;
TaskHandle_t xMappingTask = NULL;

// Flag to indicate connection status.
volatile uint8_t gHandshook = FALSE;
volatile uint8_t gPaused = FALSE;

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );

#ifdef DEBUG
	#warning DEBUG IS ACTIVE
#endif

//#define COMPASS_CALIBRATE
//#define SENSOR_CALIBRATE

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

/* The main function */
int main(void) {
  nxt_init();
  network_init();
  arq_init();
  simple_p_init(server_receiver);

  // Set red LED on to indicate INIT is ongoing
  led_set(LED_RED);

  // assert test
  //uint8_t a = 0;
  //configASSERT(a == 1);

  /* Init and start tracing */
  //vTraceEnable(TRC_START);
  
  /* Initialize RTOS utilities  */
  movementQ = xQueueCreate(2, sizeof(uint8_t)); // For sending movements to vMainMovementTask (used in compass task only)
  poseControllerQ = xQueueCreate(1, sizeof(cartesian_t)); // For setpoints to controller
  scanStatusQ = xQueueCreate(1, sizeof(uint8_t)); // For robot status
  globalWheelTicksQ = xQueueCreate(1, sizeof(wheel_ticks_t));
  globalPoseQ = xQueueCreate(1, sizeof(pose_t)); // For storing and passing the global pose estimate
  measurementQ = xQueueCreate(3, sizeof(measurement_t));
//  actuationQ = xQueueCreate(2, sizeof)

  xCommandReadyBSem = xSemaphoreCreateBinary();
  //xBeginMergeBSem = xSemaphoreCreateBinary();

  // For debugging
  vQueueAddToRegistry(movementQ, "Movement queue");
  vQueueAddToRegistry(poseControllerQ, "Pose controller queue");
  vQueueAddToRegistry(scanStatusQ, "Scan status queue");
  vQueueAddToRegistry(globalWheelTicksQ, "Global wheel ticks queue");
  vQueueAddToRegistry(globalPoseQ, "Global pose queue");
  vQueueAddToRegistry(measurementQ, "Measurement queue");

  vQueueAddToRegistry(xCommandReadyBSem, "Command ready semaphore");
  //vQueueAddToRegistry(xBeginMergeBSem, "Begin merge semaphore");

  BaseType_t ret;
  #ifndef DEBUG
  xTaskCreate(vMainCommunicationTask, "Comm", 150, NULL, 3, NULL);  // Dependant on IO, sends instructions to other tasks
  #endif
#ifndef COMPASS_CALIBRATE
  xTaskCreate(vMainPoseControllerTask, "PoseCon", 150, NULL, 2, &xPoseCtrlTask);// Dependant on estimator, sends instructions to movement task //2
  xTaskCreate(vMainPoseEstimatorTask, "PoseEst", 150, NULL, 2, NULL); // Independent task,
  xTaskCreate(vMainMappingTask, "Mapping", 200, NULL, 1, &xMappingTask);
  //xTaskCreate(vMainNavigationTask, "Navigation", 500, NULL, 1, NULL);
  ret = xTaskCreate(vMainSensorTowerTask,"Tower", 100, NULL, 3, NULL); // Independent task, but use pose updates from estimator //1
  //ret = pdPASS;
#endif
  if(ret != pdPASS) {
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
#endif
  
  led_clear(LED_RED);
  
  /*  Start scheduler */
  display_goto_xy(0,0);
  display_string("Init complete");
  display_update();

  vTaskStartScheduler();
  
  /*  MCU is out of RAM if the program comes here */
  while(1){
	led_toggle(LED_GREEN);
	led_toggle(LED_RED);
	led_toggle(LED_YELLOW);
  }

	
}