/*
 * adc.h
 *
 * Created: 18.01.2017 11:57:55
 *  Author: Kristian Lien
 */ 


#ifndef ADC_H_
#define ADC_H_

void adc_init(void);
uint8_t adc_read(uint8_t channel);


#endif /* ADC_H_ */