/*
 * open_append.c
 *
 *  Created on: 2018/02/23
 *      Author: feunoir
 */

#include "open_append.h"

FRESULT open_append(
	    FIL* fp,            /* [OUT] File object to create */
	    const char* path    /* [IN]  File name to be opened */
		)
{
    FRESULT fr;

    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(fp, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK) {
        /* Seek to end of the file to append data */
        fr = f_lseek(fp, f_size(fp));
        if (fr != FR_OK)
            f_close(fp);
    }
    return fr;
}
