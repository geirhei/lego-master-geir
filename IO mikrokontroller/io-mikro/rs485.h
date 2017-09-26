/*
 * rs485.h
 *
 * Created: 30.01.2017 14:20:43
 *  Author: krilien
 */ 


#ifndef RS485_H_
#define RS485_H_

typedef enum
{
	RS485_OK			= 0x00,
	RS485_FAIL			= 0x01
} rs485_status;

typedef struct
{
	uint8_t             num_received_bytes;
	rs485_status		status;
} rs485_receive_result;

rs485_status rs485_init(uint8_t buffer_size);
rs485_receive_result rs485_receive(uint8_t *data, uint8_t len);
rs485_receive_result rs485_receive_token(uint8_t *data, uint8_t token);
rs485_status rs485_send(uint8_t *data, uint8_t len);

#endif /* RS485_H_ */