#include "navigation.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdint.h>

#include "defines.h"
#include "types.h"

extern volatile uint8_t gHandshook;

void vMainNavigationTask( void *pvParameters )
{	

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{	
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		if (gHandshook) {

		}

		else {
			// Do nothing?
		}

	}
}


/*
void navigation_get_measurement_headings(float robotHeading, uint8_t servoStep, float *headings) {
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		uint8_t heading = (int) robotHeading + servoStep + (i-1)*90;
		if (heading >= 360) {
			heading -= 360;
		} else if (heading < 0) {
			heading +=360;
		}
		headings[i] = heading;
	}
}
*/