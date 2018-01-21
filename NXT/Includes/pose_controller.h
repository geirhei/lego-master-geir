#ifndef POSE_CONTROLLER_H_
#define POSE_CONTROLLER_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <math.h>

#include "types.h"
#include "functions.h"
#include "motor.h"
#include "server_communication.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t globalPoseQ;
extern QueueHandle_t globalWheelTicksQ;
extern QueueHandle_t poseControllerQ;
extern QueueHandle_t scanStatusQ;

extern TaskHandle_t xPoseCtrlTask;

/*  Calculates new settings for the movement task */
void vMainPoseControllerTask( void *pvParameters );

#endif