/*
 * io_nxt.h
 *
 * Created: 16.02.2017 17:23:27
 *  Author: krilien
 */ 


#ifndef IO_NXT_H_
#define IO_NXT_H_

struct to_nxt {
	uint8_t		dist_0;
	uint8_t		dist_90;
	uint8_t		dist_180;
	uint8_t		dist_270;
	int16_t		gyro_x;
	int16_t		gyro_y;
	int16_t		gyro_z;
	int16_t		compass_x;
	int16_t		compass_y;
	int16_t		compass_z;
	uint8_t		dongle_status;
};

typedef enum
{
	SET_LED				= 0x01,
	SEND_BT				= 0x02,
	RECEIVE_BT			= 0x03,
	RECEIVE_SENSORS		= 0x04,
	ALIVE_TEST			= 0x05
} nxt_message_type;

typedef enum
{
	SENSOR_DATA			= 0x01,
	BT_DATA				= 0x02,
	ALIVE_RESPONSE		= 0x03
} io_message_type;

uint8_t io_nxt_init(uint16_t buffer_size);
void io_nxt_send(io_message_type type, uint8_t *data, uint8_t len);
uint8_t io_nxt_get_message(uint8_t *data);

#endif /* IO_NXT_H_ */