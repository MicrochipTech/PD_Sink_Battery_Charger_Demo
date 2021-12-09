/*******************************************************************************
Copyright ©  [2019] Microchip Technology Inc. and its subsidiaries.

Subject to your compliance with these terms, you may use Microchip software and
any derivatives exclusively with Microchip products. It is your responsibility
to comply with third party license terms applicable to your use of third party
software (including open source software) that may accompany Microchip software.

THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER EXPRESS,
IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES
OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR PURPOSE. IN
NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN
ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST
EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, THAT YOU
HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
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


