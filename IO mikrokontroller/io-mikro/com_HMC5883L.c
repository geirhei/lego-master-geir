/************************************************************************/
// File:			com_HMC5883L.c
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver for HMC588L magnetic compass
// 
/************************************************************************/

#include "twi_master.h"
#include "com_HMC5883L.h"

uint8_t vCOM_init(void){
    vTWI_init();
    ui8TWI_start(HMC5883L_WRITE);
    ui8TWI_write(0x00); // set pointer to CRA
    ui8TWI_write(0x70); // write 0x70 to CRA
    vTWI_stop();

    ui8TWI_start(HMC5883L_WRITE);
    ui8TWI_write(0x01); // set pointer to CRB
    ui8TWI_write(0xA0); // Set bit 7 and 5 (GN2, GN0)
    vTWI_stop();

    ui8TWI_start(HMC5883L_WRITE);
    ui8TWI_write(0x02); // set pointer to measurement mode
    ui8TWI_write(0x00); // continous measurement
    vTWI_stop();
	
	return vCOM_test_alive();
}

//Registers 10 to 12 contain the compass' ID: 'H43'
uint8_t vCOM_test_alive(void) {
	uint8_t id[3] = {0};
    ui8TWI_start(HMC5883L_WRITE);
    ui8TWI_write(0x0A); // set pointer to ID register A
    vTWI_stop();
    ui8TWI_start(HMC5883L_READ);
    id[0] = ui8TWI_read_ack();
    id[1] = ui8TWI_read_ack();
    id[2] = ui8TWI_read_nack();
    vTWI_stop();	
	return (id[0] == 'H' && id[1] == '4' && id[2] == '3');
}
void vCOM_getData(int16_t *xCom, int16_t *yCom, int16_t *zCom){
    ui8TWI_start(HMC5883L_WRITE);
    ui8TWI_write(0x03); // set pointer to X axis MSB
    vTWI_stop();
    ui8TWI_start(HMC5883L_READ);
    *xCom = ((uint8_t)ui8TWI_read_ack())<<8;
    *xCom |= ui8TWI_read_ack();
    *zCom = ((uint8_t)ui8TWI_read_ack())<<8;
    *zCom |= ui8TWI_read_ack();
    *yCom = ((uint8_t)ui8TWI_read_ack())<<8;
    *yCom |= ui8TWI_read_nack();
    vTWI_stop();
}
