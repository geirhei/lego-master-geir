/*
 * bat.h
 *
 * Created: 14.03.2017 13:50:51
 *  Author: Lars Marius Strande
 */ 

/*	AVR includes	*/
#include <avr/io.h>

#ifndef BAT_H_
#define BAT_H_

void vBat_init();
int16_t adc_readBat();
int16_t averageBat();
#endif /* BAT_H_ */