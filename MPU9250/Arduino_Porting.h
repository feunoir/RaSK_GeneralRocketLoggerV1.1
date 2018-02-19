/*
 * Arduino_Porting.h
 *
 *  Created on: 2017/09/14
 *      Author: feunoir
 */

#ifndef ARDUINO_PORTING_H_
#define ARDUINO_PORTING_H_

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define sq(x) ((x)*(x))

#endif /* ARDUINO_PORTING_H_ */
