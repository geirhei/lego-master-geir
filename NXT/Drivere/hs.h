#ifndef HS_H_
#define HS_H_

#define   HS_RX_PIN  AT91C_PIO_PA5
#define   HS_TX_PIN  AT91C_PIO_PA6
#define   HS_RTS_PIN AT91C_PIO_PA7

void hs_init(void);
//int hs_enable(void); // 10/04/16 takashic
int hs_enable(unsigned long baud_rate);
void hs_disable(void);
unsigned long hs_write(unsigned char *buf, unsigned long off, unsigned long len);
unsigned long hs_read(unsigned char * buf, unsigned long off, unsigned long len);
unsigned long hs_read_delimiter(unsigned char * buf, unsigned long off, unsigned long len, unsigned char delimiter);
unsigned long hs_pending(void);

#endif /*HS_H_*/
