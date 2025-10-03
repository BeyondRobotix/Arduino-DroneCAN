#ifndef APP
#define APP

extern "C"
{
    #ifdef CANL431
        #include "stm32l4xx.h" // or stm32l431xx.h if needed
    #endif
    #ifdef CANH7
        #include "stm32h743xx.h"
    #endif
}
#include "Arduino.h"


void app_setup();

#endif // APP