/************************************************************************/
// File:			defines.h
// Author:			Erlend Ese, NTNU Spring 2016
//					Modifed and adapted to NXT by Kristian Lien, NTNU Fall 2016
// Defines located in one file
//
//
/************************************************************************/

#ifndef DEFINES_H_
#define DEFINES_H_

#define ROBOT_NAME          "NXT"
#define ROBOT_NAME_LENGTH   3
/************************************************************************/
/* PHYSICAL CONSTANTS - If the robot is changed these need to be changed
 Some of these will be sent to server during the start-up-handshake
 Wheel factor is the circumference divided by ticks per rotation
 -> pi*56/360 = 0.48mm Length the robot travels per tick              */
#define WHEELBASE_MM             170  /* Length between wheel centers  */
#define ROBOT_TOTAL_WIDTH_MM     195 /* From outer rim to outer rim   */
#define ROBOT_TOTAL_LENGTH_MM    175 /* From front to aft, total      */
#define ROBOT_AXEL_OFFSET_MM     30  /* From center of square         */
#define SENSOR_TOWER_OFFSET_X_MM 30  /* From center of square         */
#define SENSOR_TOWER_OFFSET_Y_MM 20   /* From center of square         */
#define SENSOR_OFFSET_RADIUS_MM  20  /* From center of tower          */
#define ROBOT_DEADLINE_MS        200 /* Interval between measurements */
#define SENSOR1_HEADING_DEG      0   /* Sensor angle relative to body */
#define SENSOR2_HEADING_DEG      90
#define SENSOR3_HEADING_DEG      180
#define SENSOR4_HEADING_DEG      270
#define NUMBER_OF_SENSORS		 4

#define WHEEL_FACTOR_MM 0.469   /* Calculated, measured   */

/************************************************************************/
/* Program settings                                                     */
//#define NUMBER_OF_TASKS			7
#define PERIOD_MOTOR_MS         20
#define PERIOD_ESTIMATOR_MS     40
#define PERIOD_SENSORS_MS       200
#define moveStop                0
#define moveForward             1
#define moveBackward            2
#define moveClockwise           3
#define moveCounterClockwise    4
#define moveLeftForward         5
#define moveRightForward        6
#define moveLeftBackward        7
#define moveRightBackward       8
//#define moveArrived             9

/************************************************************************/
/* COMMANDS                                                             */
#define TRUE 1
#define FALSE 0

/************************************************************************/
/* Macros                                                               */
#define M_PI 3.14159265358979323846
#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI
#define ROUND(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))

/************************************************************************/
/* Event group defines */
//#define HANDSHOOK_BIT	(1 << 0)
//define PAUSED_BIT		(1 << 1)

/************************************************************************/
/* Mapping defines */
#define PB_SIZE 				50
#define LB_SIZE					50
#define L_SIZE          		50
#define MAX_IR_DISTANCE			40
#define COLLINEAR_TOLERANCE		1
#define MU 						0.5 // slope
#define DELTA					0.5 	// cm

/************************************************************************/
/* Communication defines */
#define TYPE_HANDSHAKE      0
#define TYPE_UPDATE         1
#define TYPE_ORDER          2
//#define TPYE_PRIORITY_ORDER 3
#define TYPE_IDLE           3
#define TYPE_PAUSE          4
#define TYPE_UNPAUSE        5
#define TYPE_CONFIRM        6
#define TYPE_FINISH         7
#define TYPE_PING           8
#define TYPE_PING_RESPONSE  9
#define TYPE_LINE           10
#define TYPE_DEBUG          11

#define SERVER_ADDRESS       0

#endif /* DEFINES_H_ */
