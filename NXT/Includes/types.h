#ifndef _TYPES_H_
#define _TYPES_H_

#include <stdint.h>

#include "defines.h"

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
/*
typedef struct {
	uint8_t data[4];
	uint8_t servoStep;
} measurement_t;
*/

typedef struct {
	uint8_t forward;
	uint8_t left;
	uint8_t rear;
	uint8_t right;
	uint8_t servoStep;
} measurement_t;

/**
 * Type for storing wheel ticks
 */
typedef struct {
	int16_t left;
	int16_t right;
} wheel_ticks_t;

/**
 * Type for representing a position in cartesian coordinates
 */
typedef struct {
	float x;
	float y;
} cartesian_t;

/* Communication types */
typedef struct {
  uint8_t name_length;
  uint8_t name[ROBOT_NAME_LENGTH];
  uint16_t width;
  uint16_t length;
  uint8_t tower_offset_x;
  uint8_t tower_offset_y;
  uint8_t axel_offset;
  uint8_t sensor_offset1;
  uint8_t sensor_offset2;
  uint8_t sensor_offset3;
  uint8_t sensor_offset4;
  uint16_t sensor_heading1;
  uint16_t sensor_heading2;
  uint16_t sensor_heading3;
  uint16_t sensor_heading4;
  uint16_t deadline;
} __attribute__((packed)) handshake_message_t;

typedef struct {
  int16_t x;
  int16_t y;
  int16_t heading;
  int16_t tower_angle;
  uint8_t sensor1;
  uint8_t sensor2;
  uint8_t sensor3;
  uint8_t sensor4;
} __attribute__((packed)) update_message_t;

typedef struct {
  int16_t x;
  int16_t y;
} __attribute__((packed)) order_message_t;

typedef struct {
  int16_t x_p;
  int16_t y_p;
  int16_t x_q;
  int16_t y_q;
} __attribute__((packed)) line_message_t;

union Message {
  update_message_t update;
  handshake_message_t handshake;
  order_message_t order;
  line_message_t line;
};

typedef struct {
  uint8_t type;
  union Message message;
} __attribute__((packed)) message_t;

#endif