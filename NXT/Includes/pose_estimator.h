#ifndef POSE_ESTIMATOR_H_
#define POSE_ESTIMATOR_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <math.h>

#include "defines.h"
#include "types.h"
#include "functions.h"
#include "io.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t globalPoseQ;
extern QueueHandle_t globalWheelTicksQ;

extern TaskHandle_t xPoseCtrlTask;

/* Pose estimator task */
// New values and constants should be calibrated for the NXT
void vMainPoseEstimatorTask( void *pvParameters );

#endif