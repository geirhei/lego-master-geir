/*
 * uart.h
 *
 * Created: 16.02.2017 17:10:20
 *  Author: krilien
 */ 


#ifndef UART_H_
#define UART_H_

typedef enum
{
	UART_OK			= 0x00,
	UART_FAIL		= 0x01
} uart_status;

typedef struct
{
	uint8_t             num_received_bytes;
	uart_status			status;
} uart_receive_result;

uart_status uart0_init(uint16_t buffer_size);
uart_status uart1_init(uint16_t buffer_size);

uart_status uart0_send(uint8_t *data, uint16_t len);
uart_status uart1_send(uint8_t *data, uint16_t len);

uart_receive_result uart0_receive(uint8_t *data, uint16_t len);
uart_receive_result uart1_receive(uint8_t *data, uint16_t len);

uart_receive_result uart0_receive_token(uint8_t *data, uint8_t token);
uart_receive_result uart1_receive_token(uint8_t *data, uint8_t token);


#endif /* UART_H_ */