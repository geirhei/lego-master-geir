#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include "arq.h"
#include "defines.h"
#include "types.h"

extern arq_connection server_connection;
extern uint8_t use_arq[];
extern uint8_t connected;


static void server_communication_init(void);

static uint8_t server_connect(void);

//uint8_t send_handshake(void);
void send_update(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm);
//void send_idle(void);
//void send_ping_response(void);

/* Send the coordinates of the start and endpoints of a line. [cm] */
//void send_line(line_t *Line);

void debug(const char *fmt, ...);


/**
 * @brief      Task responsible for processing messages received from the
 *             server. Sets global status variables and relays setpoints to the
 *             pose controller.
 *             
 *             Sends to poseControllerQ
 *
 * @param      pvParameters  The pv parameters
 */
void vMainCommunicationTask( void *pvParameters );

/**
 * @brief      Task that reads message from the sendingQ and sends them to the server.
 * 				Will block if the queue is empty.
 */
void vSenderTask( void *pvParameters );

void server_receiver(uint8_t *data, uint16_t len);

#endif