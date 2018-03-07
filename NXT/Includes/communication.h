/************************************************************************/
// File:			communication.h
// Author:			Erlend Ese, NTNU Spring 2016
// 					ARQ connection protocols written by Kristian Lien, NTNU spring 2017
// 					Server communication files and communication task merged by
// 					Geir Eikeland, NTNU spring 2018
// 
// Contains the main communication task, and functions used by this and other
// tasks for sending to and receiving data from the server.
//
/************************************************************************/

#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "arq.h"
#include "defines.h"
#include "types.h"

extern arq_connection server_connection;
extern uint8_t use_arq[];
extern uint8_t connected;

/**
 * @brief      The main communication task responsible for processing messages
 *             received from the server. Sets global status variables and relays
 *             setpoints to the pose controller.
 *
 * @param      pvParameters  The pv parameters
 */
void vMainCommunicationTask( void *pvParameters );

/**
 * @brief      Init function for the connection to the server. Used in the task setup.
 */
static void server_communication_init(void);

/**
 * @brief      Attempts to connect to the server.
 *
 * @return     1 if successfull
 */
static uint8_t server_connect(void);

/**
 * @brief      Sends a handshake message to the server which contains parameters
 *             that the server needs to know for the specific robot, including
 *             physical measurements, initial pose etc.
 *
 * @return     1 if succesfull
 */
static uint8_t send_handshake(void);

/**
 * @brief      Send an acknowledgement of a ping message received from the
 *             server.
 */
static void send_ping_response(void);

/**
 * @brief      Sends pose and distance measurement updates to the server using
 *             the original message format for exchanging data with the server.
 *
 * @param[in]  x_cm            The x centimeters
 * @param[in]  y_cm            The y centimeters
 * @param[in]  heading_deg     The heading degrees
 * @param[in]  towerAngle_deg  The tower angle degrees
 * @param[in]  S1_cm           The s 1 centimeters
 * @param[in]  S2_cm           The s 2 centimeters
 * @param[in]  S3_cm           The s 3 centimeters
 * @param[in]  S4_cm           The s 4 centimeters
 */
void send_update(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm);

/**
 * @brief      Notifies the server that the robot is idle and ready for new
 *             commands.
 */
void send_idle(void);

/**
 * @brief      Sends pose and line data to the server in the message format used
 *             by the mapping task.
 *
 * @param[in]  x        x-coordinate
 * @param[in]  y        y-coordinate
 * @param[in]  heading  The heading
 * @param[in]  line     The line
 */
void send_line(int16_t x, int16_t y, uint16_t heading, line_t line);

/**
 * @brief      Callback function passed to the simple protocol init in main.
 *
 * @param      data  The data
 * @param[in]  len   The length
 */
void server_receiver(uint8_t *data, uint16_t len);

//void debug(const char *fmt, ...);

#endif