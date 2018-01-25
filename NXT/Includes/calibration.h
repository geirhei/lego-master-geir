#ifndef CALIBRATION_H_
#define CALIBRATION_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "defines.h"
#include "types.h"
#include "motor.h"
#include "display.h"
#include "io.h"
#include "led.h"
//#include "server_communication.h"
#include "communication.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t globalWheelTicksQ;

void compassTask(void *par);

void vSensorCalibrationTask(void *pvParams);

#endif