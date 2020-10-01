/*
 * dataout.h
 *
 *  Created on: Sep 20, 2020
 *      Author: meredithcaveney
 */

#include "fatfs.h"

#ifndef INC_DATAOUT_H_
#define INC_DATAOUT_H_

void dataout(FATFS *FatFs, FIL *fil, FRESULT *fres, char write_data[]);

#endif /* INC_DATAOUT_H_ */
