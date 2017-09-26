/*
 * io_nxt.c
 * 
 * This library handles the communication between the IO microcontroller and the NXT brick
*
 * Created: 16.02.2017 17:23:42
 *  Author: krilien
 */ 

#ifndef FALSE
#define FALSE       (0)
#endif

#ifndef TRUE
#define TRUE        (!FALSE)
#endif

#include <stdlib.h>
#include <string.h>
#include <util/crc16.h>
#include "io_nxt.h"
#include "rs485.h"
#include "cobs.h"
volatile uint8_t number_of_errors = 0;
uint16_t _buffer_size;

uint8_t io_nxt_init(uint16_t buffer_size) {
	_buffer_size = buffer_size;
	rs485_status result = rs485_init(_buffer_size);
	return result == RS485_OK;
}

void io_nxt_send(io_message_type type, uint8_t *data, uint8_t len) {
	int i;
	uint8_t crc = 0;
	uint8_t max_overhead = (len/254)+1;
	uint8_t *tmp = (uint8_t*) malloc(len+2); //Space for data + one byte for CRC + one byte for message type
	uint8_t *encoded_data = (uint8_t*) malloc(len+2+max_overhead); //Space for encoded data (data + CRC+ type) and the worst case overhead for COBS (one byte for every 254 bytes)
	if(tmp == NULL || encoded_data == NULL) {
		free(tmp);
		free(encoded_data);
		return;
	}
	tmp[0] = type;
	memcpy(tmp+1, data, len);
	
	for(i=0;i<len+1;i++) {
		crc = _crc_ibutton_update(crc, tmp[i]);
	}
	tmp[len+1] = crc;

	cobs_encode_result result = cobs_encode(encoded_data, len+2+max_overhead, tmp, len+2);
	if(result.status != COBS_ENCODE_OK) {
		free(tmp);
		free(encoded_data);
		return;
	}
	encoded_data[result.out_len] = 0x00; //Message delimiter
	
	rs485_send(encoded_data, result.out_len+1); //+1 for message delimiter

	free(tmp);
	free(encoded_data);
}

// Gets a message from the RS485 input-buffer
// Messages are sent and stored in the buffer as COBS-encoded. 
// This function finds the first message and decodes it. The decoded message has the format
// | Type (1 byte) | Raw data (variable bytes) | CRC (1 byte) |
uint8_t io_nxt_get_message(uint8_t *data) {
	if(data == NULL) return 0;
	
	uint8_t message[100];

	cobs_decode_result cobs_result = {0,0};
	uint8_t crc; 
	rs485_receive_result rs_result = rs485_receive_token(message, 0x00);
	if(rs_result.status == RS485_FAIL) return 0;
	
	if(rs_result.num_received_bytes > 0) {
		cobs_result = cobs_decode(data, _buffer_size, message, rs_result.num_received_bytes-1);	
				
		if(cobs_result.status != COBS_DECODE_OK) {
			return 0;
		}
		uint8_t i;
		crc = 0;
		for(i=0;i<cobs_result.out_len-1;i++) { //-1 for å ikke ta med CRC feltet
			crc = _crc_ibutton_update(crc, data[i]);
		}
		if(crc != data[cobs_result.out_len-1]) {
			number_of_errors++;
			return 0;
		} 
		else return cobs_result.out_len-1;
	}
	return 0;
	
}