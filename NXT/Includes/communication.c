#include "communication.h"

#include "types.h"
#include "led.h"
#include "server_communication.h"
#include "arq.h"
#include "display.h"

extern volatile uint8_t gHandshook;
extern volatile uint8_t gPaused;
volatile message_t message_in;

extern QueueHandle_t globalPoseQ;
extern QueueHandle_t poseControllerQ;
extern SemaphoreHandle_t xCommandReadyBSem;

QueueHandle_t sendingQ = 0;

void vMainCommunicationTask( void *pvParameters ) {
	// Setup for the communication task
	//struct sPolar Setpoint = {0}; // Struct for setpoints from server
	cartesian_t Target = {0}; // Structs for target coordinates from server
	message_t command_in; // Buffer for recieved messages

	server_communication_init();
	sendingQ = xQueueCreate(10, sizeof(message_t));
	xTaskCreate(vSenderTask, "Sender", 125, NULL, 3, NULL);

	uint8_t success = 0;
	while (!success) {
		success = server_connect();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		led_toggle(LED_GREEN);
	}

	xTaskCreate(vARQTask, "ARQ", 250, NULL, 3, NULL);
	led_clear(LED_GREEN);
	send_handshake();

	while(1) {
		if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE) {
			// We have a new command from the server, copy it to the memory
			vTaskSuspendAll ();       // Temporarily disable context switching
			taskENTER_CRITICAL();
			command_in = message_in;
			taskEXIT_CRITICAL();
			xTaskResumeAll();      // Enable context switching
			  
			switch (command_in.type)
			{
				case TYPE_CONFIRM:
					taskENTER_CRITICAL();
					gHandshook = TRUE; // Set start flag true
					taskEXIT_CRITICAL();
					
					display_goto_xy(0,1);
					display_string("Connected");
					display_update();
					break;
				case TYPE_PING:
					send_ping_response();
					break;
				case TYPE_ORDER:
					// Coordinates received in cm, convert to mm for internal use in the robot.
					Target.x = (float) command_in.message.order.x * 10;
					Target.y = (float) command_in.message.order.y * 10;
					/* Relay new coordinates to position controller */
					xQueueSendToFront(poseControllerQ, &Target, 0);
					break;
				case TYPE_PAUSE:
					//led_set(LED_YELLOW);
					// Stop sending update messages
					taskENTER_CRITICAL();
					gPaused = TRUE;
					taskEXIT_CRITICAL();
					// Stop controller - pass the current position
					xQueuePeek(globalPoseQ, &Target, 0);
					xQueueOverwrite(poseControllerQ, &Target);
					break;
				case TYPE_UNPAUSE:
					//led_set(LED_RED);
					taskENTER_CRITICAL();
					gPaused = FALSE;
					taskEXIT_CRITICAL(); 
					//led_clear(LED_RED);
					break;
				case TYPE_FINISH:
					taskENTER_CRITICAL();
					gHandshook = FALSE;
					taskEXIT_CRITICAL();
					// Stop controller - pass the current position
					xQueuePeek(globalPoseQ, &Target, 0);
					xQueueOverwrite(poseControllerQ, &Target);
					break;
			}
		}
		
	}
}


void vSenderTask( void *pvParameters ) {
	
	while (1) {
		message_t Msg = { 0 };
		if (xQueueReceive(sendingQ, &Msg, portMAX_DELAY) == pdTRUE) {
			switch (Msg.type) {
				case TYPE_UPDATE:

					break;
				case TYPE_HANDSHAKE:
					break;
				case TYPE_LINE:
					if (!connected) return;
					uint8_t data[sizeof(line_message_t)+1];
					memcpy(data, (uint8_t*) &Msg, sizeof(data));
					if(use_arq[TYPE_LINE]) arq_send(server_connection, data, sizeof(data));
					else simple_p_send(SERVER_ADDRESS, data, sizeof(data));
					break;
			}
		}
	}
}
