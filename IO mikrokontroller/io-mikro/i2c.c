/*
 * i2c.c
 *
 * Created: 18.01.2017 11:56:27
 *  Author: Kristian Lien
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "i2c.h"

#define TW_CONTROL_MASK (_BV(TWINT)|_BV(TWEA)|_BV(TWSTA)|_BV(TWSTO)) //Mask for the TWCR bits that are often used during an I2C transfer

struct I2C_message {
	uint8_t mode; //0 = write, 1 = read
	uint8_t address; //Slave address
	uint8_t *data;
	uint8_t length; //Message length (bytes)
	uint8_t reg; //Address of register to be read or written to
	uint8_t count; //Number of bytes read/written so far
};

struct I2C_message message;

void i2c_init(void) {
	TWCR = 1<<TWEN | 1<<TWIE;
	TWBR = 29; //For 100 kHz (Prescaler = 1) 7,3728MHz klokke
}

void i2c_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length) {
	if(length <= 10) { //Message size is max 10 bytes
		message.mode = TW_WRITE;
		message.address = address;
		message.count = 0;
		message.length = length;
		TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10100000; 
	}
}

void i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length) {
	if(length <= 10) { //Message size is max 10 bytes
		message.mode = TW_READ;
		message.address = address;
		message.count = 0;
		message.length = length;
		TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10100000;
	}
}

ISR(TWI_vect) {
	switch(TW_STATUS) {
		case TW_START: //START has been transmitted
			TWDR = 0x54;//message.address; //7 bit address (write mode)
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10000000;
			break;
		case TW_REP_START:
			TWDR = message.address | message.mode; //7 bit address and R/W-bit
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10000000;
		case TW_MT_SLA_ACK: //Received ACK for address + write bit
			TWDR = message.reg;
			//TWDR = *(message.data+message.count);
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10000000;
			//message.count++;
			//message.data++;
			break;
		case TW_MT_DATA_ACK:
			if(message.mode == TW_READ) {
				TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10100000;
			}
			else {
				if(message.count < message.length) { //Send next byte if there is more data
					TWDR = *(message.data+message.count);
					TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10000000;
					message.count++;
					message.data++;
					} else { //Stop
						TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10010000;
					}
				}
			break;
		case TW_MR_SLA_ACK:
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10000000;
			break;
		case TW_MR_DATA_ACK:
			*message.data = TWDR;
			message.count++;
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10000000 | (message.count + 1 == message.length ? 0x00 : 0b01000000);
			break;
		case TW_MR_DATA_NACK: //Received a byte and have NACKed it (last byte)
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10010000;
			break;
		case TW_MT_SLA_NACK:
			TWCR = (TWCR & ~TW_CONTROL_MASK) | 0b10010000;
			break;
		case TW_MT_DATA_NACK:
			break;
		case TW_MR_SLA_NACK:
			break;
	}
}