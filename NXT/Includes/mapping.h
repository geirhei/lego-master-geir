#ifndef MAPPING_H_
#define MAPPING_H_

#include "types.h"

void vMainMappingTask( void *pvParameters );

static void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t Measurement, pose_t Pose);

static void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer);

static void vMappingLineMerge(line_buffer_t *LineBuffer, line_buffer_t *LineRepo);

static int8_t vMappingIsMergeable(line_t *Line1, line_t *Line2);

static line_t vMappingMergeSegments(line_t *Line1, line_t *Line2);

#endif