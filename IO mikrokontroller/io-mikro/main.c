/*
 * io-mikro.c
 *
 * Created: 17.01.2017 13:31:20
 * Author : Kristian Lien
 */ 
#define F_CPU 7372800UL
#define NRF19 PINC7
#define NRF20 PINC6
#define DONGLE_CONNECTED !(PINC & (1<<NRF19))
#define IO_NXT_BUFFER_SIZE 128
#define IO_DONGLE_BUFFER_SIZE 128

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <avr/cpufunc.h>

#include "adc.h"
#include "bt.h"
#include "io_nxt.h"
#include "com_HMC5883L.h"
#include "LSM6DS3.h"
#include "uart.h"
uint8_t moving_average(uint8_t avg, uint8_t sample);

int main(void)
{
	DDRB = 0x07;
	DDRC = 0x00;
	
 	PORTB = 0x04;
	_delay_ms(100);
	
	uint8_t init_success = 1;
	adc_init();
	init_success *= gyro_init();
	init_success *= vCOM_init();
	init_success *= io_nxt_init(IO_NXT_BUFFER_SIZE);
	init_success *= (bt_init(IO_DONGLE_BUFFER_SIZE) == BT_OK);
	
	if(!init_success) {
		PORTB = 0x10;
		while(1);
	}
	sei();
	
	struct to_nxt values;
	uint8_t message[IO_NXT_BUFFER_SIZE];
	uint8_t num_bytes = 0;
	PORTB = 0x10;
	values.dist_0 = 100;
	values.dist_90 = 100;
	values.dist_180 = 100;
	values.dist_270 = 100;
	uint8_t count = 0;
	PORTB = 0x00;

	while (1) 
    {
		if(++count > 50) {
			count = 0;
			vCOM_getData(&values.compass_x,&values.compass_y,&values.compass_z);
			gyro_getData(&values.gyro_x,&values.gyro_y,&values.gyro_z);
			values.dist_0 = moving_average(values.dist_0, adc_read(0));
			values.dist_90 = moving_average(values.dist_90, adc_read(1));
			values.dist_180 = moving_average(values.dist_180, adc_read(2));
			values.dist_270 = moving_average(values.dist_270, adc_read(3));
			values.dongle_status = DONGLE_CONNECTED;
		}

		num_bytes = io_nxt_get_message(message);
		if(num_bytes > 0){
			if(message[0] == ALIVE_TEST) {
				io_nxt_send(ALIVE_RESPONSE, NULL, 0);
			}
			if(message[0] == SET_LED) { //Format: 0b1tcccryg (c=change, g=green, r=red, y=yellow, t=toggle)
				if(message[1] & 0x40) PORTB ^= (0b00000111 & (message[1] & (message[1] >> 3))); //Toggle
				else PORTB = (PORTB & (0b11111000 | ~(message[1] >> 3))) | message[1];
			} else if(message[0] == RECEIVE_SENSORS) {
				io_nxt_send(SENSOR_DATA, (uint8_t*) &values, sizeof(values));
			} else if(message[0] == SEND_BT) {
				bt_send(&message[1], num_bytes-1);
			} else if(message[0] == RECEIVE_BT) {
				uint8_t bt_msg[IO_DONGLE_BUFFER_SIZE];
				bt_receive_result bt_result = bt_receive(bt_msg, IO_DONGLE_BUFFER_SIZE);
				io_nxt_send(BT_DATA, bt_msg, bt_result.num_received_bytes);
			}
		}
		_delay_us(500);
    }
}

uint8_t moving_average(uint8_t avg, uint8_t sample) {
	avg -= avg/3;
	avg += sample/3;
	return avg;
}