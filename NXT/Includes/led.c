//************************************************************************/
// File:			io_led.c
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

#include "io.h"
#include "led.h"

//Set og clear any combination of LEDs
//'1' led is set
//'0' led is cleared
uint8_t led_change_multiple(uint8_t red, uint8_t yellow, uint8_t green) {
  if(red > 1 || yellow > 1 || green > 1) return 0; //Make sure the arguments are 0 or 1
  return io_send_led_command((green << LED_GREEN) | (red << LED_RED) | (red << LED_YELLOW) | 0xB8);
}

//The selected LEDs are toggled
uint8_t led_toggle_multiple(uint8_t red, uint8_t yellow, uint8_t green) {
  if(red > 1 || yellow > 1 || green > 1) return 0; //Make sure the arguments are 0 or 1
  return io_send_led_command((green << LED_GREEN) | (red << LED_RED) | (red << LED_YELLOW) | 0xF8);
}

//Set a single LED
uint8_t led_set(uint8_t led) {
  if(led != LED_GREEN && led != LED_YELLOW && led != LED_RED) return 0;
  return io_send_led_command( (1 << led) | (1 << (led+3)) | 0x80 );
}

//Clear a single LED
uint8_t led_clear(uint8_t led) {
  if(led != LED_GREEN && led != LED_YELLOW && led != LED_RED) return 0;
  return io_send_led_command( (0 << led) | (1 << (led+3)) | 0x80 );
}

//Toggle a single LED
uint8_t led_toggle(uint8_t led) {
  if(led != LED_GREEN && led != LED_YELLOW && led != LED_RED) return 0;
  return io_send_led_command( (1 << led) | (1 << (led+3)) | 0xC0 );
}