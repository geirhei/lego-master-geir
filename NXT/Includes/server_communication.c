#include "server_communication.h"


//#define SERVER_ADDRESS  0

arq_connection server_connection;
uint8_t connected = 0;
extern message_t message_in;
extern SemaphoreHandle_t xCommandReadyBSem;
extern uint8_t gHandshook;
//extern QueueHandle_t sendingQ;

uint8_t use_arq[12] = { 
  [TYPE_HANDSHAKE] = 1,
  [TYPE_UPDATE] = 0, 
  [TYPE_IDLE] = 1, 
  [TYPE_PING_RESPONSE] = 0, 
  [TYPE_LINE] = 0,
  [TYPE_DEBUG] = 0
};

void server_communication_init(void) {
  if(connected) return;
  server_connection = arq_new_connection();
}

uint8_t server_connect(void) {
  connected = arq_connect(server_connection, SERVER_ADDRESS, server_receiver, 1000);
  return connected;
}

uint8_t send_handshake(void) {
  if(!connected) return 0;
  message_t msg;
  msg.type = TYPE_HANDSHAKE;
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
  return 1;
}

void send_update(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm){
  if(!connected) return;
  message_t msg;
  msg.type = TYPE_UPDATE;
  msg.message.update.x = x_cm;
  msg.message.update.y = y_cm;
  msg.message.update.heading = heading_deg;
  msg.message.update.tower_angle = towerAngle_deg;
  msg.message.update.sensor1 = S1_cm;
  msg.message.update.sensor2 = S2_cm;
  msg.message.update.sensor3 = S3_cm;
  msg.message.update.sensor4 = S4_cm;
  uint8_t data[sizeof(update_message_t)+1];
  memcpy(data, (uint8_t*) &msg, sizeof(data));
  if(use_arq[TYPE_UPDATE]) arq_send(server_connection, data, sizeof(data));
  else simple_p_send(SERVER_ADDRESS, data, sizeof(data));
}

/*
void send_idle(void) {
  if(!connected) return;
  uint8_t status = TYPE_IDLE;
  if(use_arq[TYPE_IDLE]) arq_send(server_connection, &status, 1);
  else simple_p_send(SERVER_ADDRESS, &status, 1);
}
*/

void debug(const char *fmt, ...) {
	uint8_t buf[100];
	va_list ap;
	buf[0] = TYPE_DEBUG;
	va_start(ap, fmt);
	uint8_t ret = vsprintf((char*)buf+1, fmt, ap);
	va_end(ap);
	if (ret > 0) {
		if(use_arq[TYPE_DEBUG]) arq_send(server_connection, buf, ret+1);
		else simple_p_send(SERVER_ADDRESS, buf, ret+1);
	}
}

/*
void send_ping_response(void) {
  if(!connected) return;
  uint8_t status = TYPE_PING_RESPONSE;
  if(use_arq[TYPE_PING_RESPONSE]) arq_send(server_connection, &status, 1);
  else simple_p_send(SERVER_ADDRESS, &status, 1);
}
*/

/*
void send_line(line_t *Line) {
  message_t msg;
  msg.type = TYPE_LINE;
  msg.message.line.x_p = Line->P.x;
  msg.message.line.y_p = Line->P.y;
  msg.message.line.x_q = Line->Q.x;
  msg.message.line.y_q = Line->Q.y;
  
}
*/

void server_receiver(uint8_t *data, uint16_t len) {
  if(data == NULL) { // ARQ passes NULL to the callback when connection is lost
      gHandshook = 0;
  }
  memcpy(&message_in, data, len);
  
  xSemaphoreGive(xCommandReadyBSem);
}

