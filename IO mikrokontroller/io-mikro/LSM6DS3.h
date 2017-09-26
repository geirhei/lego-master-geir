/*
 * LSM6DS3.h
 *
 * Created: 26.01.2017 11:58:00
 *  Author: krilien
 */ 


#ifndef LSM6DS3_H_
#define LSM6DS3_H_

#define LSM6DS3_WRITE 0xD6
#define LSM6DS3_READ 0xD7

uint8_t gyro_init(void);
uint8_t gyro_getID(void);
void gyro_getData(int16_t *xCom, int16_t *yCom, int16_t *zCom);

#endif /* LSM6DS3_H_ */