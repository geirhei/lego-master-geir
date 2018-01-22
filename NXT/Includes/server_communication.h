/*
 * server_communication.h
 *
 *  Created on: 21. feb. 2017
 *      Author: krilien
 */

#ifndef INCLUDES_SERVER_COMMUNICATION_H_
#define INCLUDES_SERVER_COMMUNICATION_H_

#include "defines.h"
#include "types.h"

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

/*
typedef struct {
  int16_t orientation;
  int16_t distance;
} __attribute__((packed)) order_message_t;
*/

typedef struct {
  int16_t x;
  int16_t y;
} __attribute__((packed)) order_message_t;

typedef struct {
  int16_t x_p;
  int16_t y_p;
  int16_t x_q;
  int16_t y_q;
} __attribute__((packed)) line_message_t;

/*
typedef struct {
  int16_t heading;
  int16_t distance;
} __attribute__((packed)) priority_order_message_t;
*/

union Message {
  update_message_t update;
  handshake_message_t handshake;
  order_message_t order;
  //priority_order_message_t priority_order;
};

typedef struct {
  uint8_t type;
  union Message message;
} __attribute__((packed)) message_t;

void server_communication_init(void);
uint8_t server_connect(void);
uint8_t send_handshake(void);
void send_update(int16_t x_cm, int16_t y_cm, int16_t heading_deg, int16_t towerAngle_deg, uint8_t S1_cm, uint8_t S2_cm, uint8_t S3_cm, uint8_t S4_cm);
void send_idle(void);
void send_ping_response(void);
void send_line(line_t *line);
void server_receiver(uint8_t *data, uint16_t len);
void debug(const char *fmt, ...);

#endif /* INCLUDES_SERVER_COMMUNICATION_H_ */
