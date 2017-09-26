#ifndef __AT91_TWI_H__
#  define __AT91_TWI_H__

/* Main user interface */
int twi_init(void);

void twi_start_write(unsigned long dev_addr, const unsigned char *data, unsigned long nBytes);
void twi_start_read(unsigned long dev_addr, unsigned char *data, unsigned long nBytes);
int twi_status(void);
void twi_reset(void);

#endif
