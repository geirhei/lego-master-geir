#ifndef __NXT_AVR_H__
#  define __NXT_AVR_H__

/* Main user interface */
void nxt_avr_init(void);

void nxt_avr_1kHz_update(void);

void nxt_avr_set_motor(unsigned long n, int power_percent, int brake);

void nxt_avr_power_down(void);

void nxt_avr_test_loop(void);

void nxt_avr_update(void);

unsigned long buttons_get(void);

unsigned long battery_voltage(void);

unsigned long sensor_adc(unsigned long n);

void nxt_avr_set_input_power(unsigned long n, unsigned long power_type);

void nxt_avr_firmware_update_mode(void);

#endif
