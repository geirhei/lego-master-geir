/*
 * bluetooth.c
 *
 * Created: 30.01.2017 14:20:17
 *  Author: krilien
 */ 

#include <avr/io.h>
#include "bt.h"
#include "uart.h"

bt_status bt_init(uint16_t buffer_size) {
	return (bt_status) uart1_init(buffer_size);
}

bt_status bt_send(uint8_t *data, uint16_t len) {
	return (bt_status) uart1_send(data, len);
}

bt_receive_result bt_receive(uint8_t *data, uint16_t len) {
	bt_receive_result bt_result;
	uart_receive_result result = uart1_receive(data, len);
	
	bt_result.num_received_bytes = result.num_received_bytes;
	bt_result.status = result.status;
	
	return bt_result;
}

bt_receive_result bt_receive_token(uint8_t *data, uint8_t token) {
	bt_receive_result bt_result;
	uart_receive_result result = uart1_receive_token(data, token);
	
	bt_result.num_received_bytes = result.num_received_bytes;
	bt_result.status = result.status;
	
	return bt_result;
}