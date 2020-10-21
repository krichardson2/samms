
#include "dataout.h"
#include <string.h>

void dataout(FATFS *FatFs, FIL *fil, FRESULT *fres)
{
	  //Mount drive
	      *fres = f_mount(FatFs, "", 1); //1=mount now
	      if (*fres != FR_OK) {
	      	//error
	    	f_mount(NULL, "", 0);
	        while(1);
	      }

	      DWORD free_clusters, free_sectors, total_sectors;

	      FATFS* getFreeFs;

	      *fres = f_getfree("", &free_clusters, &getFreeFs);
	      if (*fres != FR_OK) {
	      	//error
	    	f_mount(NULL, "", 0);
	        while(1);
	      }

	      total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	      free_sectors = free_clusters * getFreeFs->csize;



	      //Try to open file
	      *fres = f_open(fil, "test.txt", FA_READ);
	      if (*fres != FR_OK) {
	        //error
	    	f_mount(NULL, "", 0);
	        while(1);
	      }


	      BYTE readBuf[30];

	      //We can either use f_read OR f_gets to get data out of files
	      //f_gets is a wrapper on f_read that does some string formatting for us
	      TCHAR* rres = f_gets((TCHAR*)readBuf, 30, fil);
	      if(rres != 0) {
	        //read
	      } else {
	        //error
	      }

	      //Close file, don't forget this!
	      f_close(fil);

	      *fres = f_open(fil, "file1.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
	      if(*fres == FR_OK) {
	        //opened
	      } else {
	        //error
	      }

	      strncpy((char*)readBuf, "litty", 5); // what to write out + character amt
	      UINT bytesWrote;
	      *fres = f_write(fil, readBuf, 5, &bytesWrote);
	      if(*fres == FR_OK) {
	        //wrote
	      } else {
	        //error
	      }

	      //Close file, don't forget this!
	      f_close(fil);

	      //De-mount drive
	      f_mount(NULL, "", 0);

}
