#include "mapping.h"

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <stdint.h>
#include <math.h>

#include "types.h"
#include "defines.h"
#include "functions.h"
#include "communication.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t measurementQ;
extern QueueHandle_t globalPoseQ;
extern SemaphoreHandle_t xBeginMergeBSem;
//extern TaskHandle_t xMappingTask;

static message_t vMappingGetLineMessage(line_t *Line);

/* Mapping task */
void vMainMappingTask( void *pvParameters )
{
	// init:
	point_buffer_t **PointBuffers;
	line_buffer_t **LineBuffers;
	line_buffer_t *LineRepo;
	
	PointBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(point_buffer_t*));
	LineBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(line_buffer_t*));
	
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		PointBuffers[i] = pvPortMalloc(sizeof(point_buffer_t));
		PointBuffers[i]->buffer = pvPortMalloc(PB_SIZE * sizeof(point_t));
		PointBuffers[i]->len = 0;

		LineBuffers[i] = pvPortMalloc(sizeof(line_buffer_t));
		LineBuffers[i]->buffer = pvPortMalloc(LB_SIZE * sizeof(line_t));
		LineBuffers[i]->len = 0;
	}

	LineRepo = pvPortMalloc(sizeof(line_buffer_t));
	LineRepo->buffer = pvPortMalloc(L_SIZE * sizeof(line_t));
	LineRepo->len = 0;

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
		
		if (gHandshook)
		{
			// Wait up to 200ms for a measurement. This is the period of the
			// sensor tower sampling.
			measurement_t Measurement;
			if (xQueueReceive(measurementQ, &Measurement, 0) == pdTRUE) {
				uint8_t sensorValues[] = { Measurement.forward, Measurement.left, Measurement.rear, Measurement.right };

				pose_t Pose;
				xQueuePeek(globalPoseQ, &Pose, 0);
				// Convert to centimeters
				Pose.x /= 10;
				Pose.y /= 10;
				// Put inside [0,2pi)
				vFunc_wrapTo2Pi(&Pose.theta);

				// Add new IR-measurements to end of PB
				vMappingUpdatePointBuffers(PointBuffers, sensorValues, Measurement.servoStep, Pose);
			}
			
			// Check for notification from sensor tower task.
			// Do not wait.
			if (ulTaskNotifyTake(pdTRUE, 0) == 1) {
				vTaskSuspendAll();
				for (uint8_t j = 0; j < NUMBER_OF_SENSORS; j++) {
					//line_t Line = { PointBuffers[j]->buffer[0], PointBuffers[j]->buffer[PointBuffers[j]->len] };
					//sendLine(&Line);
					vMappingLineCreate(PointBuffers[j], LineBuffers[j]);
					//message_t LineMsg = vMappingGetLineMessage(&LineBuffers[0]->buffer[0]);
					//line_t testLine = LineBuffers[0]->buffer[0];
					//vMappingLineMerge(LineBuffers[j], LineRepo);
					for (uint8_t k = 0; k < LineBuffers[j]->len; k++) {
						line_t line = LineBuffers[j]->buffer[k];
						//send_line(line);
					}
					// Prevent overflow while testing
					LineBuffers[j]->len = 0;
				}
				xTaskResumeAll();
			}
			
			
		} 
		else {
			// do nothing
		}
		
	}
}

static void vMappingUpdatePointBuffers(point_buffer_t **Buffers, uint8_t *sensorValues, uint8_t servoStep, pose_t Pose) {
	float towerAngle = (float) servoStep * DEG2RAD; //[0,pi/2]
	float theta = towerAngle + Pose.theta;

	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		if (i > 0) {
			theta += 0.5 * M_PI;
		}
		vFunc_wrapTo2Pi(&theta); //[0,2pi)
		uint8_t r = sensorValues[i];
		// Abort if the measurement is outside the valid range
		if (r <= 0 || r > 40) {
			continue;
		}
		point_t Pos = vFunc_polar2Cart(theta, (float) r);
		// Get the coordinates relative to the global coordinate system
		Pos.x += Pose.x;
		Pos.y += Pose.y;
		uint8_t currentLength = Buffers[i]->len;
		Buffers[i]->buffer[currentLength] = Pos;
		Buffers[i]->len++;
	}
}

static void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer) {
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
				point_t P = { A.x, A.y };
				point_t Q = { PointBuffer->buffer[i].x, PointBuffer->buffer[i].y };
				Line.P = P;
				Line.P = Q;
			} else {
				continue;
			}
		} else {
			point_t P = { A.x, A.y };
			point_t Q = { PointBuffer->buffer[i-1].x, PointBuffer->buffer[i-1].y };
			Line.P = P;
			Line.Q = Q;
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



	LineBuffer->len = 0;
}