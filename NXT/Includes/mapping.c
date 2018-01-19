#include "mapping.h"

void vMappingUpdatePointBuffers(point_buffer_t *Buffers, measurement_t *Measurement, pose_t *Pose) {
	float theta = Measurement->servoStep + Pose->theta;
	if (theta >= 360) {
		theta -= 360;
	}
	for (uint8_t i = 0; i < NUMBER_OF_SENSORS; i++) {
		if (i > 0) {
			theta += 90;
		}
		if (theta >= 360) {
			theta -= 360;
		}
	}
}