/*
 * rs485.c
 *
 * Created: 30.01.2017 14:20:55
 *  Author: krilien
 */ 

#define DRIVER_ENABLE_PIN		PD4
#define DRIVER_ENABLE_MASK	(uint8_t) ~(1<<PD4)
#define DRIVER_ENABLE		PORTD = (PORTD & DRIVER_ENABLE_MASK) | 1<<DRIVER_ENABLE_PIN	
#define RECIEVER_ENABLE		PORTD = (PORTD & DRIVER_ENABLE_MASK) | 0<<DRIVER_ENABLE_PIN	

#include <avr/io.h>
#include "rs485.h"
#include "uart.h"

void driver_enable();
void receiver_enable();

rs485_status rs485_init(uint8_t buffer_size) {
	DDRD = 0xFF;
	RECIEVER_ENABLE;
	
	return (rs485_status) uart0_init(buffer_size);
}

rs485_status rs485_send(uint8_t *data, uint8_t len) {
	uart_status status;
	
	DRIVER_ENABLE;
	
	status = uart0_send(data, len);
	
	if(status == UART_OK){
		while (!( UCSR0A & (1<<TXC0))); //Make sure the last byte is sent before setting the RS485 transceiver into receive mode
		UCSR0A |= (1<<TXC0); //Clear the flag
	}

	RECIEVER_ENABLE;
	
	return (rs485_status) status;
}

rs485_receive_result rs485_receive(uint8_t *data, uint8_t len) {
	rs485_receive_result rs_result;
	uart_receive_result result = uart0_receive(data, len);
	
	rs_result.num_received_bytes = result.num_received_bytes;
	rs_result.status = result.status;
	
	return rs_result;
}

rs485_receive_result rs485_receive_token(uint8_t *data, uint8_t token) {
	rs485_receive_result rs_result;
	uart_receive_result result = uart0_receive_token(data, token);
	
	rs_result.num_received_bytes = result.num_received_bytes;
	rs_result.status = result.status;
	
	return rs_result;
}
