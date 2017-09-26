#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

/* Servos */
#define servoRight  	0   /* NXT_PORT_A */
#define servoLeft    	1   /* NXT_PORT_B */
#define servoTower      2   /* NXT_PORT_C */

void vMotorMovementSwitch(uint8_t movement, int16_t tmp_leftWheelTicks, int16_t tmp_rightWheelTicks);
void vMotorCountUpdate(void);
void vMotor_init(void);
void vMotorSetAngle(uint8_t motor, int16_t angle);

#endif
