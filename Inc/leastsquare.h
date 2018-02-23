/*
 * leastsquare.h
 *
 *  Created on: 2018/02/23
 *      Author: feunoir
 */

#ifndef LEASTSQUARE_H_
#define LEASTSQUARE_H_

/**
 * Calculate slope from arrayed data
 */

float calc_leastsquare_uint32in(uint32_t *time, uint32_t *data, uint8_t pos, uint8_t n)
{
    double  sum_xy = 0,
            sum_x = 0,
            sum_y = 0,
            sum_x2 = 0;
    uint8_t i = pos - n;

    do {
        sum_xy += (double)time[i] * (double)data[i];
        sum_x += (double)time[i];
        sum_y += (double)data[i];
        sum_x2 += (double)time[i] * (double)time[i];
        i++;
    } while(i != pos);

	return ( (float)((n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)) );
}


float calc_leastsquare_floatin(uint32_t *time, float *data, uint8_t pos, uint8_t n)
{
    double  sum_xy = 0,
            sum_x = 0,
            sum_y = 0,
            sum_x2 = 0;
    uint8_t i = pos - n;

    do {
        sum_xy += (double)time[i] * (double)data[i];
        sum_x += (double)time[i];
        sum_y += (double)data[i];
        sum_x2 += (double)time[i] * (double)time[i];
        i++;
    } while(i != pos);

	return ( (float)((n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)) );
}

float calc_leastsquare_doublein(uint32_t *time, double *data, uint8_t pos, uint8_t n)
{
    double  sum_xy = 0,
            sum_x = 0,
            sum_y = 0,
            sum_x2 = 0;
    uint8_t i = pos - n;

    do {
        sum_xy += (double)time[i] * data[i];
        sum_x += (double)time[i];
        sum_y += data[i];
        sum_x2 += (double)time[i] * (double)time[i];
        i++;
    } while(i != pos);

	return ( (float)((n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x)) );
}





#endif /* LEASTSQUARE_H_ */
