#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "arq.h"
#include "defines.h"
#include "types.h"

extern arq_connection server_connection;
extern uint8_t use_arq[];
extern uint8_t connected;

/**
 * Task responsible for processing messages received from the server.
 * Sets global status variables and relays setpoints to the pose controller.
 */
void vMainCommunicationTask( void *pvParameters );

static void server_communication_init(void);

static uint8_t server_connect(void);

static uint8_t send_handshake(void);

static void send_ping_response(void);

void send_update(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm);

void send_idle(void);

/**
 * Sends the coordinates of the start and endpoints of a line. [cm]
 */
void send_line(int16_t x, int16_t y, uint16_t heading, line_t line);

//void debug(const char *fmt, ...);

void server_receiver(uint8_t *data, uint16_t len);

#endif