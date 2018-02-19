/*
 * stm32_mpu9250_clk.c
 *
 *  Created on: 2017/09/14
 *      Author: feunoir
 */

#include "stm32F3xx_hal.h"
#include "stm32_mpu9250_clk.h"

volatile uint32_t counter = 0;

void HAL_SYSTICK_Callback()
{
	counter++;
}

int stm32_get_clock_ms(unsigned long *count)
{
	*count = counter;
	return 0;
}
