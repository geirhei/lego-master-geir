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
extern volatile uint8_t gPaused;

extern QueueHandle_t mappingMeasurementQ;
extern QueueHandle_t globalPoseQ;
extern TaskHandle_t xMappingTask;

void vMainMappingTask( void *pvParameters )
{
	// Initialize the buffers used for mapping operations. Each sensor has its
	// own buffers that are allocated on the heap.
	point_buffer_t *PointBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(point_buffer_t));
	line_buffer_t *LineBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(line_buffer_t));
	
	// Set initial lengths to 0
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		PointBuffers[i].len = 0;
		LineBuffers[i].len = 0;
	}

	// Initialize the repo for storing the completely merged line segments
	line_repo_t *LineRepo = pvPortMalloc(sizeof(line_repo_t));
	LineRepo->len = 0;

	// Verify allocation
	configASSERT(PointBuffers && LineBuffers && LineRepo );

	// Set task to run at a fixed frequency
	/*
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();
	*/

	while (1)
	{
		//vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		if (gHandshook == TRUE && gPaused == FALSE)
		{
			// Read the robots current pose
			pose_t Pose;
			xQueuePeek(globalPoseQ, &Pose, 10 / portTICK_PERIOD_MS);

			// Convert to centimeters
			Pose.x /= 10;
			Pose.y /= 10;

			// Put inside [0,2pi)
			func_wrap_to_2pi(&Pose.theta);

			// Block here waiting for measurement from the sensor tower.
			measurement_t Measurement;
			if (xQueueReceive(mappingMeasurementQ, &Measurement, 200 / portTICK_PERIOD_MS) == pdTRUE) {
				
				// Append new IR measurements to the end of each PB
				mapping_update_point_buffers(PointBuffers, Measurement, Pose);
			}
			
			// Check for notification from sensor tower task. Do not wait.
			if (ulTaskNotifyTake(pdTRUE, 0) == 1) {

				// Run create and merge functions for each of the sensors' buffers
				for (uint8_t j = 0; j < NUMBER_OF_SENSORS; j++) {
					mapping_line_create(&PointBuffers[j], &LineBuffers[j]);
					mapping_line_merge(&LineBuffers[j], LineRepo);
				}

				// Merge the repo with itself until it cannot be reduced further
				uint8_t lastRepoLen;
				do {
					lastRepoLen = LineRepo->len;
					LineRepo = mapping_repo_merge(LineRepo);
				} while (LineRepo->len < lastRepoLen);	
			}

			line_t LineOut = { 0 };
			if (LineRepo->len > 0) {
				LineOut = LineRepo->buffer[LineRepo->len-1];
				LineRepo->len--;
			}

			// Send update to server. LineOut contains all zeroes if one was not available from the LineRepo.
			send_line(ROUND(Pose.x), ROUND(Pose.y), ROUND(Pose.theta*RAD2DEG), LineOut);
		}

		else {
			// If disconnected or paused
			vTaskDelay(200 / portTICK_PERIOD_MS);
		}
		
	}
}

static void mapping_update_point_buffers(point_buffer_t *Buffers, measurement_t Measurement, pose_t Pose) {
	float towerAngle = (float) Measurement.servoStep * DEG2RAD; //[0,pi/2]
	float theta = towerAngle + Pose.theta;

	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		if (i > 0)
			theta += 0.5 * M_PI;

		func_wrap_to_2pi(&theta); //[0,2pi)
		uint8_t r = Measurement.data[i];
		// Abort if the measurement is outside the valid range
		if (r <= 0 || r > 40)
			continue;
		
		point_t Pos = func_polar2cart(theta, (float) r);
		// Get the coordinates relative to the global coordinate system
		Pos.x += Pose.x;
		Pos.y += Pose.y;
		uint8_t currentLength = Buffers[i].len;
		Buffers[i].buffer[currentLength] = Pos;
		Buffers[i].len++;

		configASSERT(Buffers[i].len <= PB_SIZE);

		// Notify itself to call LineCreate if one of the buffers are full
		if (Buffers[i].len >= PB_SIZE)
			xTaskNotifyGive(xMappingTask);
	}
}

static void mapping_line_create(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer) {
	configASSERT(PointBuffer && LineBuffer);
	configASSERT(LineBuffer->buffer);

	LineBuffer->len = 0;

	if (PointBuffer->len < 3) {
		PointBuffer->len = 0;
		return;
	}
	
	point_t A = PointBuffer->buffer[0];
	point_t B = PointBuffer->buffer[1];

	for (uint8_t i = 2; i < PointBuffer->len; i++) {
		line_t Line;
		if (mapping_are_collinear(&A, &B, &PointBuffer->buffer[i])) {
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
		configASSERT(LineBuffer->buffer);
	}

	configASSERT(LineBuffer->len <= LB_SIZE);

	// Reset PB length
	PointBuffer->len = 0;
}

static void mapping_line_merge(line_buffer_t *LineBuffer, line_repo_t *LineRepo) {
	configASSERT(LineBuffer&& LineRepo);
	configASSERT(LineBuffer->buffer);
	configASSERT(LineRepo->buffer);

	if (LineRepo->len == 0) {
		for (uint8_t i = 0; i < LineBuffer->len; i++) {
			LineRepo->buffer[i] = LineBuffer->buffer[i];
			//configASSERT(LineRepo->buffer != NULL);
			LineRepo->len++;
		}
	} else {
		uint8_t newLen = LineRepo->len;
		for (uint8_t j = 0; j < LineBuffer->len; j++) {
			uint8_t merged = FALSE;
			for (uint8_t k = 0; k < LineRepo->len; k++) {
				if (mapping_is_mergeable(&LineBuffer->buffer[j], &LineRepo->buffer[k])) {
					LineRepo->buffer[k] = mapping_merge_segments(&LineBuffer->buffer[j], &LineRepo->buffer[k]);
					merged = TRUE;
					break;
				}
			}
			if (!merged) {
				LineRepo->buffer[newLen] = LineBuffer->buffer[j];
				newLen++;
			}
		}
		LineRepo->len = newLen;
		configASSERT(LineRepo->len <= L_SIZE);
	}

	// Reset LineBuffer length
	LineBuffer->len = 0;
}

static uint8_t mapping_is_mergeable(line_t *Line1, line_t *Line2) {
	configASSERT(Line1 && Line2);

    //MU = 0.5; // slope
    //DELTA = 1.0; // cm

    float m1 = func_get_slope(Line1);
    float m2 = func_get_slope(Line2);

    // Test slope
    if (fabs(m1 - m2) > MU)
        return 0; // Slope-test failed

    // Test distance between endpoints
    float d1 = func_distance_between(&Line1->P, &Line2->P);
    float d2 = func_distance_between(&Line1->P, &Line2->Q);
    float d3 = func_distance_between(&Line1->Q, &Line2->P);
    float d4 = func_distance_between(&Line1->Q, &Line2->Q);

    if ( (d1 <= DELTA) || (d2 <= DELTA) || (d3 <= DELTA) || (d4 <= DELTA) )
        return 1;
    
    return 0;
}

static line_t mapping_merge_segments(line_t *Line1, line_t *Line2) {
	configASSERT(Line1 && Line2);

	float a1 = func_get_slope(Line1);
	float a2 = func_get_slope(Line2);
	float b1 = Line1->P.y - a1 * Line1->P.x;
	float b2 = Line2->P.y - a2 * Line2->P.x;
	float l1 = func_distance_between(&Line1->P, &Line1->Q);
	float l2 = func_distance_between(&Line2->P, &Line2->Q);

	// Find parameters for the merged line
	float a = (l1 * a1 + l2 * a2) / (l1 + l2);
	float b = (l1 * b1 + l2 * b2) / (l1 + l2);

	// Find projections of all 4 points onto the merged line
	point_t projectedPoints[] = { 
		func_get_projected_point(Line1->P, a, b),
		func_get_projected_point(Line2->P, a, b),
		func_get_projected_point(Line1->Q, a, b),
		func_get_projected_point(Line2->Q, a, b)
	};

	// Find the points farthest away from each other
	point_t P = projectedPoints[0];
	point_t Q = P;

	for (uint8_t i = 1; i < 4; i++) {
		if (projectedPoints[i].x < P.x)
			P = projectedPoints[i];
		
		if (projectedPoints[i].x > P.x)
			Q = projectedPoints[i];
	}

	return (line_t) { P, Q };
}

static line_repo_t* mapping_repo_merge(line_buffer_t *Repo) {
	configASSERT(Repo);

	// Allocate memory on the heap for the resulting repo
	line_repo_t *MergedRepo = pvPortMalloc(sizeof(line_repo_t));
	MergedRepo->len = 0;

	for (uint8_t i = 0; i < Repo->len; i++) {

		if (!mapping_is_empty(Repo->buffer[i])) {
			uint8_t merged = FALSE;
			for (uint8_t j = i + 1; j < Repo->len; j++) {

				if (!mapping_is_empty(Repo->buffer[j])) {
					if (mapping_is_mergeable(&Repo->buffer[i], &Repo->buffer[j])) {
						// Add merged line to MergedRepo
						MergedRepo->buffer[MergedRepo->len] = mapping_merge_segments(&Repo->buffer[i], &Repo->buffer[j]);
						MergedRepo->len++;

						//Set value of buffer entry to 0
						Repo->buffer[j] = (line_t) { 0 };
						merged = TRUE;
					}
				}

			}

			// If the line is not merged with another, add it to the end of MergedRepo
			if (!merged) {
				MergedRepo->buffer[MergedRepo->len] = Repo->buffer[i];
				MergedRepo->len++;
			}
		}
		
	}

	// Free the memory from the original repo and return a pointer to the merged one
	vPortFree(Repo);
	Repo = NULL;
	return MergedRepo;
}

static uint8_t mapping_are_collinear(point_t *a, point_t *b, point_t *c) {
    return fabs( (a->y - b->y) * (a->x - c->x) - (a->y - c->y) * (a->x - b->x) ) <= COLLINEAR_TOLERANCE;
}

static uint8_t mapping_is_empty(line_t line) {
	return ( line.P.x == 0 && line.P.y == 0 && line.Q.x == 0 && line.Q.y == 0 );
}