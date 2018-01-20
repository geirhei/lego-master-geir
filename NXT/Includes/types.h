#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>

/**
 * Type representing the heading and position of the robot
 */
typedef struct {
	float theta;
	float x;
	float y;
} pose_t;

/**
 * Definition for type representing the coordinates of a measurement
 */
typedef struct {
	float x;
	float y;
} point_t;

/**
 * Definition for type representing a line segment
 */
typedef struct {
	point_t P;
	point_t Q;
} line_t;

/**
 * Type containing a buffer of pointers to coordinates and the current length of the buffer
 */
typedef struct {
	point_t* buffer;
	uint8_t len;
} point_buffer_t;

/**
 * Type containing a buffer of pointers to lines and the current length of the buffer
 */
typedef struct {
	line_t* buffer;
	uint8_t len;
} line_buffer_t;

/**
 * Definition for type storing the IR data from a measurement
 */
typedef struct {
	uint8_t data[4];
	uint8_t servoStep;
} measurement_t;

/**
 * Type for storing wheel ticks
 */
typedef struct {
	int16_t rightWheel;
	int16_t leftWheel;
} wheel_ticks_t;

/**
 * Type for representing a position in cartesian coordinates
 */
typedef struct {
	float x;
	float y;
} cartesian_t;

#endif