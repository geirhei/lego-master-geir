//************************************************************************/
// File:			led.c
// Author:			Kristian Lien, NTNU Spring 2017              
//
// Functions for controlling the LEDs on the IO-PCB
// 
// One byte is sent to the IO-microcontroller to change the leds. The bits in the 
// byte follow this format: 0b1tcccryg (c=change, g=green, r=red, y=yellow, t=toggle)
//
// 'ccc' is the mask for which leds are to be changed, e.g. using '101' would result in
// only the red and green leds being changed to the levels set in 'ryg'
//
// If 't' is set, then the leds selected with 'ccc' and 'ryg' are toggled
//
// MSB is always high
/************************************************************************/

#ifndef _LED_H_
#define _LED_H_

#define LED_GREEN       0  
#define LED_YELLOW      1
#define LED_RED         2

//Set og clear any combination of LEDs
//'1' led is set
//'0' led is cleared
uint8_t led_change_multiple(uint8_t red, uint8_t yellow, uint8_t green);

//The selected LEDs are toggled
uint8_t led_toggle_multiple(uint8_t red, uint8_t yellow, uint8_t green);

//Set a single LED
uint8_t led_set(uint8_t led);

//Clear a single LED
uint8_t led_clear(uint8_t led);

//Toggle a single LED
uint8_t led_toggle(uint8_t led);

#endif

