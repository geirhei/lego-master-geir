#ifndef __NXT_MOTORS_H__
#  define __NXT_MOTORS_H__

#  define NXT_N_MOTORS 3

int nxt_motor_get_count(unsigned long n);
int nxt_motor_get_speed(unsigned long n);
void nxt_motor_set_count(unsigned long n, int count);
void nxt_motor_set_speed(unsigned long n, int speed_percent, int brake);
void nxt_motor_command(unsigned long n, int target_count, int speed_percent);
void nxt_motor_init(void);

// ISR points used by motor processing
void nxt_motor_1kHz_process(void);

#endif
