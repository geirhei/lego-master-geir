/*
 * bt.h
 *
 * Created: 30.01.2017 14:20:28
 *  Author: krilien
 */ 


#ifndef BT_H_
#define BT_H_

typedef enum
{
	BT_OK			= 0x00,
	BT_FAIL			= 0x01
} bt_status;

typedef struct
{
	uint8_t         num_received_bytes;
	bt_status		status;
} bt_receive_result;

bt_status bt_init(uint16_t buffer_size);
bt_receive_result bt_receive(uint8_t *data, uint16_t len);
bt_status bt_send(uint8_t *data, uint16_t len);
bt_receive_result bt_receive_token(uint8_t *data, uint8_t token);

#endif /* BT_ */