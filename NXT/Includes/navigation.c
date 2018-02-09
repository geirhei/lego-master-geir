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