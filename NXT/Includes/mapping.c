#include "mapping.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <stdint.h>
#include <math.h>
#include <stdlib.h>

#include "types.h"
#include "defines.h"
#include "functions.h"
#include "communication.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t measurementQ;
extern QueueHandle_t globalPoseQ;
extern TaskHandle_t xMappingTask;

void vMainMappingTask( void *pvParameters )
{
	// Initialize the buffers used for mapping operations. Each sensor has its
	// own buffers that are allocated on the heap.
	point_buffer_t *PointBuffers;
	line_buffer_t *LineBuffers;
	line_buffer_t *LineRepo;
	
	PointBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(point_buffer_t));
	LineBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(line_buffer_t));
	
	// Allocate space for the desired buffer length and set initial length to 0.
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		PointBuffers[i].buffer = pvPortMalloc(PB_SIZE * sizeof(point_t));
		PointBuffers[i].len = 0;

		LineBuffers[i].buffer = pvPortMalloc(LB_SIZE * sizeof(line_t));
		LineBuffers[i].len = 0;
	}

	LineRepo = pvPortMalloc(sizeof(line_buffer_t));
	LineRepo->buffer = pvPortMalloc(L_SIZE * sizeof(line_t));
	LineRepo->len = 0;

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		// Runs with a fixed frequency
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		if (gHandshook)
		{
			// Retrieve a measurement sent by the sensor tower.
			// Do not wait if there is none available
			measurement_t Measurement;
			if (xQueueReceive(measurementQ, &Measurement, 0) == pdTRUE) {
				// Read the robots current pose
				pose_t Pose;
				xQueuePeek(globalPoseQ, &Pose, 0);
				// Convert to centimeters
				Pose.x /= 10;
				Pose.y /= 10;
				// Put inside [0,2pi)
				vFunc_wrapTo2Pi(&Pose.theta);

				// Append new IR measurements to the end of each PB
				vMappingUpdatePointBuffers(PointBuffers, Measurement, Pose);
			}
			
			// Check for notification from sensor tower task.
			// Do not wait.
			if (ulTaskNotifyTake(pdTRUE, 0) == 1) {
				for (uint8_t j = 0; j < NUMBER_OF_SENSORS; j++) {

					//vTaskSuspendAll();
					vMappingLineCreate(&PointBuffers[j], &LineBuffers[j]);
					vMappingLineMerge(&LineBuffers[j], LineRepo);
					//xTaskResumeAll();

				}

				for (uint8_t k = 0; k < LineRepo->len; k++) {
					line_t line = LineRepo->buffer[k];
					send_line(line);
				}
			}
			
		} 
		else {
			// do nothing
		}
		
	}
}

static void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t Measurement, pose_t Pose) {
	float towerAngle = (float) Measurement.servoStep * DEG2RAD; //[0,pi/2]
	float theta = towerAngle + Pose.theta;

	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		if (i > 0) {
			theta += 0.5 * M_PI;
		}
		vFunc_wrapTo2Pi(&theta); //[0,2pi)
		uint8_t r = Measurement.data[i];
		// Abort if the measurement is outside the valid range
		if (r <= 0 || r > 40) {
			continue;
		}
		point_t Pos = vFunc_polar2Cart(theta, (float) r);
		// Get the coordinates relative to the global coordinate system
		Pos.x += Pose.x;
		Pos.y += Pose.y;
		uint8_t currentLength = Buffers[i].len;
		Buffers[i].buffer[currentLength] = Pos;
		Buffers[i].len++;

		// Notify itself to call LineCreate if one of the buffers are full
		if (Buffers[i].len >= PB_SIZE) {
			xTaskNotifyGive(xMappingTask);
		}
	}
}

static void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer) {
	configASSERT(PointBuffer != NULL && LineBuffer != NULL);

	LineBuffer->len = 0;

	if (PointBuffer->len < 3) {
		PointBuffer->len = 0;
		return;
	}
	
	point_t A = PointBuffer->buffer[0];
	point_t B = PointBuffer->buffer[1];

	for (uint8_t i = 2; i < PointBuffer->len; i++) {
		line_t Line;
		if (vFunc_areCollinear(&A, &B, &PointBuffer->buffer[i])) {
			if (i == PointBuffer->len-1) {
				Line = (line_t) {
					{A.x, A.y},
					{PointBuffer->buffer[i].x, PointBuffer->buffer[i].y}
				};
			} else {
				continue;
			}
		} else {
			Line = (line_t) {
				{A.x, A.y},
				{PointBuffer->buffer[i-1].x, PointBuffer->buffer[i-1].y}
			};
			if (i > PointBuffer->len-3) {
				break;
			} else {
				A = PointBuffer->buffer[i];
				B = PointBuffer->buffer[i+1];
				i++;
			}
		}
		LineBuffer->buffer[LineBuffer->len] = Line;
		LineBuffer->len++;
	}

	// Reset PB length
	PointBuffer->len = 0;
}

static void vMappingLineMerge(line_buffer_t *LineBuffer, line_buffer_t *LineRepo) {
	configASSERT(LineBuffer != NULL && LineRepo != NULL);

	if (LineRepo->len == 0) {
		for (uint8_t i = 0; i < LineBuffer->len; i++) {
			LineRepo->buffer[LineRepo->len] = LineBuffer->buffer[i];
			LineRepo->len++;
		}
	} else {
		uint8_t index = LineRepo->len;
		for (uint8_t j = 0; j < LineBuffer->len; j++) {
			uint8_t merged = FALSE;
			for (uint8_t k = 0; k < LineRepo->len; k++) {
				if (vMappingIsMergeable(&LineBuffer->buffer[j], &LineRepo->buffer[k]) == 1) {
					LineRepo->buffer[k] = vMappingMergeSegments(&LineBuffer->buffer[j], &LineRepo->buffer[k]);
					merged = TRUE;
					break;
				}
			}
			if (merged == FALSE) {
				LineRepo->buffer[index] = LineBuffer->buffer[j];
				index++;
			}
		}
		LineRepo->len = index;
	}

	// Reset LineBuffer length
	LineBuffer->len = 0;
}

static int8_t vMappingIsMergeable(line_t *Line1, line_t *Line2) {
    const float MU = 0.01; // slope
    const float DELTA = 1.0; // cm

    float m1 = vFunc_getSlope(Line1);
    float m2 = vFunc_getSlope(Line2);

    // Test slope
    if (fabs(m1 - m2) > MU) {
        // Slope test failed
        return -1;
    }

    // Test distance between endpoints
    float d1 = vFunc_distanceBetween(&Line1->P, &Line2->P);
    float d2 = vFunc_distanceBetween(&Line1->P, &Line2->Q);
    float d3 = vFunc_distanceBetween(&Line1->Q, &Line2->P);
    float d4 = vFunc_distanceBetween(&Line1->Q, &Line2->Q);

    if ( (d1 <= DELTA) || (d2 <= DELTA) || (d3 <= DELTA) || (d4 <= DELTA) ) {
        return 1;
    }
    return -1;
}

static line_t vMappingMergeSegments(line_t *Line1, line_t *Line2) {
	configASSERT(Line1 != NULL && Line2 != NULL);

	float a1 = vFunc_getSlope(Line1);
	float a2 = vFunc_getSlope(Line2);
	float b1 = Line1->P.y - a1 * Line1->P.x;
	float b2 = Line2->P.y - a2 * Line2->P.x;
	float l1 = vFunc_distanceBetween(&Line1->P, &Line1->Q);
	float l2 = vFunc_distanceBetween(&Line2->P, &Line2->Q);

	// Find parameters for the merged line
	float a = (l1 * a1 + l2 * a2) / (l1 + l2);
	float b = (l1 * b1 + l2 * b2) / (l1 + l2);

	// Find projections of all 4 points onto the merged line
	point_t projectedPoints[4];
	projectedPoints[0] = vFunc_getProjectedPoint(Line1->P, a, b);
	projectedPoints[1] = vFunc_getProjectedPoint(Line2->P, a, b);
	projectedPoints[2] = vFunc_getProjectedPoint(Line1->Q, a, b);
	projectedPoints[3] = vFunc_getProjectedPoint(Line2->Q, a, b);

	// Find the points farthest away from each other
	point_t P = projectedPoints[0];
	point_t Q = projectedPoints[0];
	for (uint8_t i = 1; i < 4; i++) {
		if (projectedPoints[i].x < P.x) {
			P = projectedPoints[i];
		}
		if (projectedPoints[i].x > P.x) {
			Q = projectedPoints[i];
		}
	}

	return (line_t) { P, Q };
}