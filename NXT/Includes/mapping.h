#include <stdint.h>

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
	point_t p;
	point_t q;
} line_t;

typedef struct {
	point_t* buffer;
	uint8_t len;
} point_buffer_t;

typedef struct {
	line_t* buffer;
	uint8_t len;
} line_buffer_t;

/**
 * Definition for type storing the IR data from a measurement
 */
typedef struct {
	uint8_t forward;
	uint8_t left;
	uint8_t rear;
	uint8_t right;
	uint8_t servoStep;
} measurement_t;