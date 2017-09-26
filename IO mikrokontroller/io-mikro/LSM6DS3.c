/*
 * LSM6DS3.c
 *
 * Created: 26.01.2017 11:59:52
 *  Author: krilien
 */ 

/************************************************************************/
// File:			com_HMC5883L.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver for HMC588L magnetic compass
//
/************************************************************************/

#include "twi_master.h"
#include "LSM6DS3.h"

uint8_t gyro_init(void){
	vTWI_init();
	ui8TWI_start(LSM6DS3_WRITE);
	ui8TWI_write(0x11); // set pointer to CTRL2_G
	ui8TWI_write(0x42); // write 0x82 to CTRL2_G (104 Hz sample rate, 125 dps)
	vTWI_stop();
	if(gyro_getID() != 0x69) return 0;
	return 1;
}

uint8_t gyro_getID(void) {
	uint8_t id;
	ui8TWI_start(LSM6DS3_WRITE);
	ui8TWI_write(0x0F);
	vTWI_stop();
	ui8TWI_start(LSM6DS3_READ);
	id = ((uint8_t)ui8TWI_read_nack());
	vTWI_stop();
	return id;
}
void gyro_getData(int16_t *x, int16_t *y, int16_t *z){
	ui8TWI_start(LSM6DS3_WRITE);
	ui8TWI_write(0x22); // set pointer to X axis MSB
	vTWI_stop();
	ui8TWI_start(LSM6DS3_READ);
	*x = ((uint8_t)ui8TWI_read_ack());
	*x |= ui8TWI_read_ack() << 8;
	*y = ((uint8_t)ui8TWI_read_ack());
	*y |= ui8TWI_read_ack() << 8;
	*z = ((uint8_t)ui8TWI_read_ack());
	*z |= ui8TWI_read_nack() << 8;
	vTWI_stop();
}
