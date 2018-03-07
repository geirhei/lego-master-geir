/************************************************************************/
// File:      types.h
// Author:    - Geir Eikeland, NTNU Spring 2018  
//
// Contains all the custom type definitions used in the application for easy
// inclusion where they are needed.
//
// /************************************************************************/

#ifndef TYPES_H_
#define TYPES_H_

#include <stdint.h>

#include "defines.h"

/**
 * Type for storing the pose of a robot.
 */
typedef struct {
	float theta;
	float x;
	float y;
} pose_t;

/**
 * Type for storing the coordinates of a point in the environment.
 */
typedef struct {
	float x;
	float y;
} point_t;

/**
 * Type for storing the endpoints of a line segment.
 */
typedef struct {
	point_t P;
	point_t Q;
} line_t;

/**
 * Type for storing IR-measurements.
 */
typedef struct {
	uint8_t data[4];
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
 * Type for storing a point buffer
 */
typedef struct {
  uint8_t len;
  point_t buffer[PB_SIZE];
} point_buffer_t;

/**
 * Type for storing a line buffer
 */
typedef struct {
  uint8_t len;
  line_t buffer[LB_SIZE];
} line_buffer_t;

/**
 * Type for storing a line repo
 */
typedef struct {
  uint8_t len;
  line_t buffer[L_SIZE];
} line_repo_t;

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
  int16_t x;
  int16_t y;
  uint16_t heading;
  int16_t p_x;
  int16_t p_y;
  int16_t q_x;
  int16_t q_y;
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