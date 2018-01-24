#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "server_communication.h"

extern volatile message_t message_in;

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

#endif