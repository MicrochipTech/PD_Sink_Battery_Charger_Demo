/* 
 * File:   time_delay.h
 * Author: C41076
 *
 * Created on September 23, 2020, 9:47 AM
 */

#ifndef TIME_DELAY_H
#define	TIME_DELAY_H

#include "definitions.h"

#ifdef	__cplusplus
extern "C" {
#endif
void timer_ms_delay(uint32_t ms);
SYS_TIME_RESULT set_timer_ms(SYS_TIME_HANDLE *timer, uint32_t ms);
bool timer_ms_done(SYS_TIME_HANDLE *timer);




#ifdef	__cplusplus
}
#endif

#endif	/* TIME_DELAY_H */

