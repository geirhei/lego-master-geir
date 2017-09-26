#ifndef __NXT_SPI_H__
#  define __NXT_SPI_H__

/*
 * Note that this is not a normal SPI interface, 
 * it is a bodged version as used by the NXT's 
 * display.
 *
 * The display does not use MISO because you can
 * only write to it in serial mode.
 *
 * Instead, the MISO pin is not used by the SPI
 * and is instead driven as a PIO pin for controlling CD.
 */

void nxt_spi_init(void);
void nxt_spi_write(unsigned long CD, const unsigned char *data, unsigned long nBytes);
void nxt_spi_set_display(const unsigned char *disp);
void nxt_spi_refresh(void);

#endif
