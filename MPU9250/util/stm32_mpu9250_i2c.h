/*
 * stm32_MPU9250_i2c.h
 *
 *  Created on: 2017/09/14
 *      Author: feunoir
 */

#ifndef UTIL_STM32_MPU9250_I2C_H_
#define UTIL_STM32_MPU9250_I2C_H_

void stm32_i2c_init(I2C_HandleTypeDef *hi2cx);
int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data);
int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data);


#endif /* UTIL_STM32_MPU9250_I2C_H_ */
