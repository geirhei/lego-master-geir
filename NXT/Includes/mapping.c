#include "mapping.h"

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
		point_t pos;
		pos = vFunc_polar2Cart(theta, r);

	}
}
