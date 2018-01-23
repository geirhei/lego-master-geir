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
#include "server_communication.h"

extern volatile uint8_t gHandshook;

extern QueueHandle_t measurementQ;
extern QueueHandle_t globalPoseQ;
extern QueueHandle_t sendingQ;
extern SemaphoreHandle_t xBeginMergeBSem;
extern TaskHandle_t xMappingTask;

static message_t vMappingGetLineMessage(line_t *Line);

/* Mapping task */
void vMainMappingTask( void *pvParameters )
{
	// init:
	point_buffer_t *PointBuffers;
	line_buffer_t *LineBuffers;
	line_t *Lines;
	PointBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(point_buffer_t));
	LineBuffers = pvPortMalloc(NUMBER_OF_SENSORS * sizeof(line_buffer_t));
	Lines = pvPortMalloc(L_SIZE * sizeof(line_t));
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		PointBuffers[i].buffer = pvPortMalloc(PB_SIZE * sizeof(point_t));
		LineBuffers[i].buffer = pvPortMalloc(LB_SIZE * sizeof(line_t));
	}

	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1000 / portTICK_PERIOD_MS;
	xLastWakeTime = xTaskGetTickCount();

	while (1)
	{
		vTaskDelayUntil(&xLastWakeTime, xFrequency);

		//test
		line_t testLine = { {-10, 0}, {10, 0} };
		message_t LineMsg = vMappingGetLineMessage(&testLine);
		xQueueSendToBack(sendingQ, &LineMsg, 100 / portTICK_PERIOD_MS);
		//end

		if (gHandshook)
		{
			measurement_t Measurement = {0};
			if (xQueueReceive(measurementQ, &Measurement, 100) == pdTRUE) {
				pose_t Pose = {0};
				xQueuePeek(globalPoseQ, &Pose, 0);

				// Add new IR-measurements to end of PB
				vMappingUpdatePointBuffers(PointBuffers, &Measurement, &Pose);
			}

			// Check semaphore for synchronization from sensor tower
			if (xSemaphoreTake(xBeginMergeBSem, 10) == pdTRUE) {
				for (uint8_t j = 0; j < NUMBER_OF_SENSORS; j++) {
					vMappingLineCreate(&PointBuffers[j], &LineBuffers[j]);
					// merge
				}
			}

			
		} else {

		}

	}
}

void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t *Measurement, pose_t *Pose) {
	float towerAngle = Measurement->servoStep * DEG2RAD; //[0,pi/2]
	float theta = towerAngle + Pose->theta;
	vFunc_wrapTo2Pi(&theta); //[0,2pi)

	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		if (i > 0) {
			theta += M_PI/2;
		}
		vFunc_wrapTo2Pi(&theta); //[0,2pi)
		float r = Measurement->data[i];
		if (r <= 0 || r > 40) {
			continue;
		}
		point_t Pos;
		Pos = vFunc_polar2Cart(theta, r);
		// Get the coordinates relative to the global coordinate system
		Pos.x += Pose->x;
		Pos.y += Pose->y;
		uint8_t currentLength = Buffers[i].len;
		Buffers[i].buffer[currentLength] = Pos;
		Buffers[i].len++;
	}
}

void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer) {
	if (PointBuffer->len < 3) {
		return;
	}

	LineBuffer->len = 0;
	point_t A = PointBuffer->buffer[0];
	point_t B = PointBuffer->buffer[1];

	for (uint8_t i = 2; i < PointBuffer->len; i++) {
		line_t Line;
		if (vFunc_areCollinear(&A, &B, &PointBuffer->buffer[i])) {
			if (i == PointBuffer->len - 1) {
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
			if (i > PointBuffer->len - 3) {
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

void vMappingLineMerge(point_buffer_t *PointBuffer, line_buffer_t *LineRepo) {

}

/**
 * Creates and returns a message for sending to the server from the input
 * Line structure.
 */
static message_t vMappingGetLineMessage(line_t *Line) {
	message_t msg;
	msg.type = TYPE_LINE;
	msg.message.line.x_p = Line->P.x;
	msg.message.line.y_p = Line->P.y;
	msg.message.line.x_q = Line->Q.x;
	msg.message.line.y_q = Line->Q.y;
	return msg;
}