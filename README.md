# ADC -> DMA Source Code

Apparently the .IOC files do not open properly in the STM32CubeIDE if pulled from here. The main source code has all the proper initialization within. The initialization steps are also outlined [here](https://docs.google.com/document/d/1qkgSoZE3QrpV7r2WLNOdz0vMn0RVLeRlGni07C8RmEA/edit?usp=sharing)

I was using 2/4 leds to show the half and complete buffer fills.

## Files

- main.c
- nlms_filter.c , nlms_filter.h
- arm_math.h


## Using the CMSIS Libraries

The ARM_MATH_CM4 library is located in \libs and needs to be appended to the library search path in the cubeIDE. 
Click project -> properties-> C/C++ Build -> Settings -> Tool Settings -> MCU GCC Linker -> Libraries

under libraries, add arm_cortexM4lf_math and under library search path add "../libs"
