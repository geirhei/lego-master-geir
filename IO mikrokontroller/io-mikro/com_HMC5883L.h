/************************************************************************/
// File:			com_HMC5883L.h
// Author:			Erlend Ese, NTNU Spring 2016
// Purpose:         Driver for HMC588L magnetic compass
//
/************************************************************************/


#ifndef COM_HMC5883L_H_
#define COM_HMC5883L_H_

#define HMC5883L_WRITE 0x3C
#define HMC5883L_READ 0x3D

uint8_t vCOM_init(void);
uint8_t vCOM_test_alive(void);
void vCOM_getData(int16_t *xCom, int16_t *yCom, int16_t *zCom);

#endif /* COM_HMC5883L_H_ */