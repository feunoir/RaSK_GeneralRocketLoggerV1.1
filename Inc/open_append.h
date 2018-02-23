/*
 * open_append.h
 *
 *  Created on: 2018/02/23
 *      Author: feunoir
 */

#ifndef OPEN_APPEND_H_
#define OPEN_APPEND_H_

#include "ff.h"

FRESULT open_append(
	    FIL* fp,            /* [OUT] File object to create */
	    const char* path    /* [IN]  File name to be opened */
		);


#endif /* OPEN_APPEND_H_ */
