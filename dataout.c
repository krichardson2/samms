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
	        while(1);
	      }

	      //open file
	      *fres = f_open(fil, "th.txt", FA_READ);
	      if (*fres != FR_OK) {
	        //error
	    	f_mount(NULL, "", 0);
	        while(1);
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

void writeSDcard(FATFS *FatFs, FIL *fil, FRESULT *fres, int *outdata)//arr[24], date, SPL
{
	  //Mount drive
	      *fres = f_mount(FatFs, "", 1); //1=mount now
	      if (*fres != FR_OK) {
	      	//error
	    	f_mount(NULL, "", 0);
	        while(1);
	      }

	      //open file
	      *fres = f_open(fil, "data.txt", FA_WRITE | FA_OPEN_APPEND);
	      if(*fres != FR_OK){
	    	f_mount(NULL, "", 0);
	        while(1);
	      }


	      UINT bytesWrote;
	      BYTE writeBuff[24];
	      strncpy((char*)writeBuff, outdata, strlen(outdata));

	      *fres = f_write(fil, writeBuff, 24, &bytesWrote);
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
