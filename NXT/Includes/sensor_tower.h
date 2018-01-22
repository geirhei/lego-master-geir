#ifndef SENSOR_TOWER_H_
#define SENSOR_TOWER_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "types.h"
#include "defines.h"
#include "functions.h"
#include "motor.h"
#include "io.h"
#include "server_communication.h"

extern volatile uint8_t gHandshook;
extern volatile uint8_t gPaused;

extern QueueHandle_t scanStatusQ;
extern QueueHandle_t globalPoseQ;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t measurementQ;

extern SemaphoreHandle_t xBeginMergeBSem;

extern TaskHandle_t xMappingTask;

void vMainSensorTowerTask( void *pvParameters );

#endif