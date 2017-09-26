/*
 * io.h
 *
 *  Created on: 21. feb. 2017
 *      Author: krilien
 */

#ifndef _IO_H_
#define _IO_H_

#include <stdint.h>

#define BAUD_RATE       230400

uint8_t io_init(void);
void vIOTask(void *pvParamters);
uint8_t io_send_bluetooth(uint8_t *data, uint16_t len);
uint8_t io_send_bluetooth_string(char *str);
void io_set_bluetooth_receive_callback(void (*cb)(uint8_t*, uint16_t));
uint8_t io_send_led_command(uint8_t leds);
void gyro_get_raw(int16_t *xGyro, int16_t *yGyro, int16_t *zGyro);
float gyro_get_dps_x(void);
float gyro_get_dps_y(void);
float gyro_get_dps_z(void);
void compass_get(int16_t *xCom, int16_t *yCom, int16_t *zCom);
uint8_t dongle_connected(void);
uint8_t distance_get_cm(uint8_t direction);
uint8_t distance_get_volt(uint8_t direction);

#endif /*_IO_H_ */
