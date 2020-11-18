#include "dataout.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
void readSDcard(FATFS *FatFs, FIL *fil, FRESULT *fres, int *userdata)//arr[2], h/l, SPL
{
	  //Mount drive
	      *fres = f_mount(FatFs, "", 1); //1=mount now
	      if (*fres != FR_OK) {
	      	//error
	    	f_mount(NULL, "", 0);
	      }

	      //open file
	      *fres = f_open(fil, "th.txt", FA_READ);
	      if (*fres != FR_OK) {
	        //error
	    	f_mount(NULL, "", 0);
	      }

	      BYTE readBuf[10];

	      TCHAR* rres = f_gets((TCHAR*)readBuf, 10, fil);
	      if(rres != 0) {
	    	  char *ptr;
	    	  ptr = strtok (rres,",");
	    	  //if low threshold, store 0, if high, 1
	    	  int type;
	    	  if(*ptr == 'L')
	    	  {
	    		  type = 0;//set low thresh
	    	  }
	    	  else
	    	  {
	    		  type = 1;//set high thresh
	    	  }
	    	  // value
	    	  ptr = strtok(NULL, ",");
	    	  int value = atoi(ptr);

	    	  userdata[0]= type;
	    	  userdata[1]= value;
	      }

	      //Close file, don't forget this!
	      f_close(fil);

	      //De-mount drive
	      f_mount(NULL, "", 0);


}

void writeSDcard(FATFS *FatFs, FIL *fil, FRESULT *fres, int *outdata)//arr[19], date, SPL
{
	  //Mount drive
	      *fres = f_mount(FatFs, "", 1); //1=mount now
	      if (*fres != FR_OK) {
	      	//error
	    	f_mount(NULL, "", 0);
	      }

	      //open file based on week
	      char filename[14];
	      sprintf(filename,"data_%i%i_%i%i.txt",outdata[0],outdata[1],outdata[6],outdata[7]);
	      *fres = f_open(fil, filename, FA_WRITE | FA_OPEN_APPEND);
	      if(*fres != FR_OK){
	    	f_mount(NULL, "", 0);
	      }


	      UINT bytesWrote;
	      BYTE writeBuff[19];
	      strncpy((char*)writeBuff, outdata, strlen(outdata));

	      *fres = f_write(fil, writeBuff, 19, &bytesWrote);
	      f_sync(fil);//flush buffer
	      if(*fres == FR_OK) {
	        //wrote
	      } else {
	    	f_mount(NULL, "", 0);
	        //error
	      }

	      //Close file, don't forget this!
	      f_close(fil);

	      //De-mount drive
	      f_mount(NULL, "", 0);

}
