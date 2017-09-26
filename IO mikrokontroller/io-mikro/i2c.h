/*
 * i2c.h
 *
 * Created: 18.01.2017 11:53:52
 *  Author: Kristian Lien
 */ 


#ifndef I2C_H_
#define I2C_H_

void i2c_init();
void i2c_write(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);
void i2c_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);

#endif /* I2C_H_ */