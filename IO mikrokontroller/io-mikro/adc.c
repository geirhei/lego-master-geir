/*
 * adc.c
 *
 * Created: 18.01.2017 11:57:30
 *  Author: Kristian Lien
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "adc.h"

#define ADC_INTERVAL_MS 25

void adc_init(void) {
	ADMUX = 1<<REFS1 | 1<<REFS0 | 1<<ADLAR; //Internal 2,56V reference, left adjust result
	ADCSRA = 1<<ADEN | 0<<ADATE | 0<<ADIE | 1<<ADPS2 | 0<<ADPS1 | 1<<ADPS0;
	//ADCSRB = 0<<ADTS2 | 1<<ADTS1 | 1<<ADTS0;
	DIDR0 = 0xFF;
}

uint8_t adc_read(uint8_t channel) {
	uint8_t val = 0;
	if(channel >= 0 && channel <= 7) {
		ADMUX = (0b11100000 & ADMUX) | channel;
		ADCSRA |= 1<<ADSC;
		while(!(ADCSRA & _BV(ADIF))) {}
		ADCSRA |= _BV(ADIF);
		val = ADCH; 
	}
	return val;
}

