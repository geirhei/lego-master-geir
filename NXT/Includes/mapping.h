#ifndef MAPPING_H_
#define MAPPING_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include <stdint.h>
#include <math.h>

#include "types.h"
#include "defines.h"
#include "functions.h"

extern volatile uint8_t gHandshook;

extern SemaphoreHandle_t xBeginMergeBSem;

extern QueueHandle_t measurementQ;
extern QueueHandle_t globalPoseQ;

void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t *Measurement, pose_t *Pose);

void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer);

void vMappingLineMerge(point_buffer_t *PointBuffer, line_buffer_t *LineRepo);

#endif