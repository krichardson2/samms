# samms

Notes for files included in repo:

dataout.c
dataout.h

- the SPI2 module must be configured with FATFS Middleware enabled as a User-defined Configuration. Use an external GPIO for CS output. call it SD_CS
- SPI2 is configured as Full-Duplex Master
- Include driver files user_diskio_spi.h and user_diskio_spi.c found here: https://github.com/kiwih/cubemx-mmc-sd-card
-include this in user code 1:

//Fatfs object

FATFS FatFs;>

//File object

FIL fil;

-in user code 2: 

FRESULT fres;

-to call function:

dataout(&FatFs, &fil, &fres);

- if there is a bug in the code before unmounting, remove the SD card and insert it again before launching another debug session
- file output names must be less than 9 characters
