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
extern QueueHandle_t sendingQ;
extern SemaphoreHandle_t xCommandReadyBSem;

void vMainCommunicationTask( void *pvParameters ) {
	// Setup for the communication task
	message_t command_in; // Buffer for recieved messages

	server_communication_init();

	uint8_t success = 0;
	while (!success) {
		success = server_connect();
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		led_toggle(LED_GREEN);
	}

	xTaskCreate(vARQTask, "ARQ", 250, NULL, 3, NULL);
	led_clear(LED_GREEN);
	message_t msg = { .type = TYPE_HANDSHAKE };
	xQueueSendToBack(sendingQ, &msg, 0);
	//send_handshake();

	while(1) {
		if (xSemaphoreTake(xCommandReadyBSem, portMAX_DELAY) == pdTRUE) {
			// We have a new command from the server, copy it to the memory
			vTaskSuspendAll ();       // Temporarily disable context switching
			taskENTER_CRITICAL();
			command_in = message_in;
			taskEXIT_CRITICAL();
			xTaskResumeAll();      // Enable context switching
			  
			cartesian_t Target; // Stores coordinates receives from server

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
				case TYPE_PING: {
					message_t msg = { .type = TYPE_PING_RESPONSE };
					xQueueSendToBack(sendingQ, &msg, 0);
					break;
				}
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
		message_t msg;
		if (xQueueReceive(sendingQ, &msg, portMAX_DELAY) == pdTRUE) {
			if (!connected) continue;
			switch (msg.type) {
				case TYPE_PING_RESPONSE: {
					uint8_t status = TYPE_PING_RESPONSE;
					if(use_arq[TYPE_PING_RESPONSE]) arq_send(server_connection, &status, 1);
					else simple_p_send(SERVER_ADDRESS, &status, 1);
					break;
				}

				case TYPE_UPDATE:
					break;

				case TYPE_HANDSHAKE: {
					msg.message.handshake.name_length = ROBOT_NAME_LENGTH;
					strcpy((char*)msg.message.handshake.name, ROBOT_NAME);
					msg.message.handshake.width = ROBOT_TOTAL_WIDTH_MM;
					msg.message.handshake.length = ROBOT_TOTAL_LENGTH_MM;
					msg.message.handshake.axel_offset = ROBOT_AXEL_OFFSET_MM;
					msg.message.handshake.tower_offset_x = SENSOR_TOWER_OFFSET_X_MM;
					msg.message.handshake.tower_offset_y = SENSOR_TOWER_OFFSET_Y_MM;
					msg.message.handshake.sensor_offset1 = SENSOR_OFFSET_RADIUS_MM;
					msg.message.handshake.sensor_offset2 = SENSOR_OFFSET_RADIUS_MM;
					msg.message.handshake.sensor_offset3 = SENSOR_OFFSET_RADIUS_MM;
					msg.message.handshake.sensor_offset4 = SENSOR_OFFSET_RADIUS_MM;
					msg.message.handshake.sensor_heading1 = SENSOR1_HEADING_DEG;
					msg.message.handshake.sensor_heading2 = SENSOR2_HEADING_DEG;
					msg.message.handshake.sensor_heading3 = SENSOR3_HEADING_DEG;
					msg.message.handshake.sensor_heading4 = SENSOR4_HEADING_DEG;
					msg.message.handshake.deadline = ROBOT_DEADLINE_MS;

					uint8_t data[sizeof(handshake_message_t)+1];
					memcpy(data, (uint8_t*) &msg, sizeof(data));
					if(use_arq[TYPE_HANDSHAKE]) arq_send(server_connection, data, sizeof(data));
					else simple_p_send(server_connection, data, sizeof(data));
					break;
				}

				case TYPE_LINE:
					uint8_t data[sizeof(line_message_t)+1];
					memcpy(data, (uint8_t*) &msg, sizeof(data));
					if(use_arq[TYPE_LINE]) arq_send(server_connection, data, sizeof(data));
					else simple_p_send(SERVER_ADDRESS, data, sizeof(data));
					break;

				case TYPE_IDLE: {
					uint8_t status = TYPE_IDLE;
					if(use_arq[TYPE_IDLE]) arq_send(server_connection, &status, 1);
					else simple_p_send(SERVER_ADDRESS, &status, 1);
					break;
				}
			}
		}
	}
}
