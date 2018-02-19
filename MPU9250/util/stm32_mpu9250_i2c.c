/*
 * stm32_MPU9250_i2c.c
 *
 *  Created on: 2017/09/14
 *      Author: feunoir
 */

#include "stm32F3xx_hal.h"

#include "stm32_mpu9250_i2c.h"

I2C_HandleTypeDef hi2c;

void stm32_i2c_init(I2C_HandleTypeDef *hi2cx)
{
	hi2c = *hi2cx;

}
int stm32_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	HAL_I2C_Mem_Write(&hi2c, slave_addr << 1, reg_addr, 1, data, length, 0xFFFF);

	return 0;
}
int stm32_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	HAL_I2C_Mem_Read(&hi2c, slave_addr << 1, reg_addr, 1, data, length, 0xFFFF);

	return 0;
}
