#ifndef MAPPING_H_
#define MAPPING_H_

#include "types.h"

void vMainMappingTask( void *pvParameters );

void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t *Measurement, pose_t *Pose);

void vMappingLineCreate(point_buffer_t *PointBuffer, line_buffer_t *LineBuffer);

void vMappingLineMerge(point_buffer_t *PointBuffer, line_buffer_t *LineRepo);

#endif