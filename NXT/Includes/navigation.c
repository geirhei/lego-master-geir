#include "navigation.h"

#include <stdint.h>

#include "defines.h"

/*
void vMainNavigationTask( void *pvParameters )
{	
	float *distances;
	distances = pvPortMalloc(360 * sizeof(float));
	for (uint8_t i = 0; i < 360; i++) {
		distances[i] = INFINITY;
	}

	float headings[4];

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{	
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		// Receive a measurement
		measurement_t Measurement = {0};
		xQueueReceive(measurementQ, &Measurement, 0);

		// Read the global pose
		pose_t Pose = {0};
		xQueuePeek(globalPoseQ, &Pose, 0);

		float robotHeading = Pose.theta;
		uint8_t servoStep = Measurement.servoStep;
		navigation_get_measurement_headings(robotHeading, servoStep, headings);

		
		LinkedList *frontierLocations = emlist_create();
		for (uint8_t i = 0; i < 1000; i++) {
			uint32_t a = 1;
			uint32_t *ptr = &a;
			emlist_insert(frontierLocations, ptr);
		}
		while(!emlist_is_empty(frontierLocations)) {
			emlist_pop(frontierLocations);
		}
		emlist_destroy(frontierLocations);
		
		
	}
}
*/


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