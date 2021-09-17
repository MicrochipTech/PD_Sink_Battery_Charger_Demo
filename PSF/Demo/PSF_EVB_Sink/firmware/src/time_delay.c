#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


void timer_ms_delay(uint32_t ms)
{
    SYS_TIME_HANDLE tmrHandle = SYS_TIME_HANDLE_INVALID;

    if (SYS_TIME_SUCCESS != SYS_TIME_DelayMS(ms, &tmrHandle))
    {
        return;
    }

    while (true != SYS_TIME_DelayIsComplete(tmrHandle))
    {
    }
}

SYS_TIME_RESULT set_timer_ms(SYS_TIME_HANDLE *timer, uint32_t ms)
{
    *timer = SYS_TIME_HANDLE_INVALID;
    
    return(SYS_TIME_DelayMS(ms, timer));
}

bool timer_ms_done(SYS_TIME_HANDLE *timer)
{
    return(SYS_TIME_DelayIsComplete(*timer));
}


