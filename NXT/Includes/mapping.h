#ifndef MAPPING_H_
#define MAPPING_H_

#include "FreeRTOS.h"
#include <stdint.h>
#include <math.h>

#include "types.h"
#include "defines.h"
#include "functions.h"

void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t *Measurement, pose_t *Pose);

#endif