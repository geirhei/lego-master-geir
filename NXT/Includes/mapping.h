#ifndef MAPPING_H_
#define MAPPING_H_

#include "types.h"

void vMainMappingTask( void *pvParameters );

static void vMappingUpdatePointBuffers(point_buffer_t **Buffers, uint8_t *sensorValues, uint8_t servoStep, pose_t Pose);

static void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer);

static void vMappingLineMerge(line_buffer_t *LineBuffer, line_buffer_t *LineRepo);

#endif