/*
 * scheduler.h
 *
 *  Created on: 15/05/2023
 *  Author: Adam Jonkers , Laurence Watson
 */

#ifndef GROUP_MINUS_2_SCHEDULER_H_
#define GROUP_MINUS_2_SCHEDULER_H_

#include <stdint.h>
#include <stdbool.h>

/*******************************************************
    Converts tick rates and signals a tick event with set frequencys
********************************************************/
void SysTickIntHandler(void);


/*******************************************************
    Initialises functions on the clock used in systick ADC and display
********************************************************/
void initClock (void);


#endif /* GROUP_MINUS_2_SCHEDULER_H_ */
