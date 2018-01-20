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