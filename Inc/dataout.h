/*
 * dataout.h
 *
 *  Created on: Sep 20, 2020
 *      Author: meredithcaveney
 */

#include "fatfs.h"

#ifndef INC_DATAOUT_H_
#define INC_DATAOUT_H_

void mountDrive(FATFS *FatFs, FIL *fil, FRESULT *fres);
void unmountDrive(FATFS *FatFs, FIL *fil, FRESULT *fres);
void readSDcard(FATFS *FatFs, FIL *fil, FRESULT *fres, int *userdata);
void writeSDcard(FATFS *FatFs, FIL *fil, FRESULT *fres, int *outdata);

#endif /* INC_DATAOUT_H_ */
