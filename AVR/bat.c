/*/*
 * bat.c
 *
 * Created: 14.03.2017 13:50:35
 *  Author: Lars Marius Strande
 */ 

/*	AVR includes	*/
#include <avr/io.h>

/*	Custom includes	*/
#include <bat.h>
#include <defines.h>

void vBat_init(){
	/*!	Initiate as input	*/
	batReg &= ~(1<<batInput);
	
	/*!	Internal 2.56 V ref	*/	
	ADMUX |= (1<<REFS1) | (1<<REFS0);
	
	ADCSRA |= (1<<ADEN);
	
	ADCSRA |= (1<<ADPS2) | (0<<ADPS1) | (0<<ADPS0);
}

int16_t adc_readBat()
{
	unsigned int adc_value;

	ADMUX=0x07;
	
	/* Enable internal 2,54V AREF */
	ADMUX |= (1<<REFS1) | (1<<REFS0);
	
	ADCSRA |= (1<<ADSC);					//Start
		//while(ADCSRA & (1<<ADSC))			//Wait for complete
		loop_until_bit_is_clear(ADCSRA, ADSC);
	//adc_value += (ADCL >> 2) | (ADCH << 6);
	adc_value = ADCW;						//Store value
	
	return (adc_value);
}
int16_t averageBat(){
	int16_t b_buff[20];
	int count = 0;
	for (int i = 0 ; i < 20 ; i++)
	{
		b_buff[i] = adc_readBat();
		count ++;
	}
	int16_t sum;
	for (int i = 0 ; i < sizeof(b_buff)/sizeof(int16_t) ; i++)
	{
		sum += b_buff[i];
	}
	return sum/count;
}