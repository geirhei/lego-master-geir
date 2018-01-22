#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

/* Kernel includes */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "types.h"
#include "led.h"
#include "server_communication.h"
#include "arq.h"
#include "display.h"

extern volatile uint8_t gHandshook;
extern volatile uint8_t gPaused;
extern volatile message_t message_in;

extern QueueHandle_t globalPoseQ;
extern QueueHandle_t poseControllerQ;

extern SemaphoreHandle_t xCommandReadyBSem;

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

#endif