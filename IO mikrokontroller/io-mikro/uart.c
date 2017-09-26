/*
 * uart.c
 *
 * Created: 16.02.2017 17:10:04
 *  Author: krilien
 */ 
#define F_CPU		7372800UL

#define BAUD0			230400
#define BAUD_REG0		((F_CPU/(16*BAUD0)) - 1)
#define SEND_LIMIT0		100

#define BAUD1			38400
#define BAUD_REG1		((F_CPU/(16*BAUD1)) - 1)
#define SEND_LIMIT1		100

#include <stdlib.h>
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include "fifo.h"
#include "uart.h"

fifo_t uart0_fifo;
fifo_t uart1_fifo;

uart_status uart0_init(uint16_t buffer_size) {
	UBRR0L = (uint8_t) BAUD_REG0;
	UBRR0H = (uint8_t) (BAUD_REG0 >> 8);

	UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
	
	char *buf = malloc(buffer_size);
	if(buf == NULL) return UART_FAIL;
	fifo_init(&uart0_fifo, buf, buffer_size);
	return UART_OK;
}

uart_status uart1_init(uint16_t buffer_size) {
	UBRR1L = (uint8_t) BAUD_REG1;
	UBRR1H = (uint8_t) (BAUD_REG1 >> 8);
	
	UCSR1B = _BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1);
	UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);
	
	char *buf = malloc(buffer_size);
	if(buf == NULL) return UART_FAIL;
	
	fifo_init(&uart1_fifo, buf, buffer_size);
	return UART_OK;
}

uart_status uart0_send(uint8_t *data, uint16_t len) {
	if(len > SEND_LIMIT0 || data == NULL) return UART_FAIL;
	
	uint16_t i=0;
	while(i<len) {
		while (!( UCSR0A & (1<<UDRE0)));
		UDR0 = data[i];
		i++;
	}
	
	return UART_OK;
}

uart_status uart1_send(uint8_t *data, uint16_t len) {
	if(len > SEND_LIMIT1 || data == NULL) return UART_FAIL;
	
	uint16_t i=0;
	while(i<len) {
		while (!( UCSR1A & (1<<UDRE1)));
		UDR1 = data[i];
		i++;
	}
	return UART_OK;
}

uart_receive_result uart0_receive(uint8_t *data, uint16_t len) {
	uart_receive_result result;
	
	if(data == NULL) {
		result.status = UART_FAIL;
		result.num_received_bytes = 0;
	} else {
		if(len > uart0_fifo.size) len = uart0_fifo.size;
		result.status = UART_OK;
		result.num_received_bytes = fifo_read(&uart0_fifo, data, len);
	}

	return result;
}

uart_receive_result uart1_receive(uint8_t *data, uint16_t len) {
	uart_receive_result result;
	
	if(data == NULL) {
		result.status = UART_FAIL;
		result.num_received_bytes = 0;
	} else {
		if(len > uart1_fifo.size) len = uart1_fifo.size;
		result.status = UART_OK;
		result.num_received_bytes = fifo_read(&uart1_fifo, data, len);
	}

	return result;
}

uart_receive_result uart0_receive_token(uint8_t *data, uint8_t token) {
	uart_receive_result result;
	
	if(data == NULL) {
		result.status = UART_FAIL;
		result.num_received_bytes = 0;
		} else {
		result.status = UART_OK;
		result.num_received_bytes = fifo_read_token(&uart0_fifo, data, token);
	}

	return result;
}

uart_receive_result uart1_receive_token(uint8_t *data, uint8_t token) {
	uart_receive_result result;
	
	if(data == NULL) {
		result.status = UART_FAIL;
		result.num_received_bytes = 0;
		} else {
		result.status = UART_OK;
		result.num_received_bytes = fifo_read_token(&uart1_fifo, data, token);
	}

	return result;
}

ISR(USART0_RX_vect) {
	fifo_write(&uart0_fifo, UDR0);
}

ISR(USART1_RX_vect) {
	fifo_write(&uart1_fifo, UDR1);
}