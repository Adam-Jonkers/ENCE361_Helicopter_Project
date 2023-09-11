/*
 * scheduler.c
 *
 *  Created on: 15/05/2023
<<<<<<< HEAD
 *      Author: Adam Jonkers , Laurence Watson

>>>>>>> 52256ac958bee3e8f1a5435fc2cc391a9ed17f43
 */

#include "scheduler.h"
#include "Config.h"
#include "Control.h"

#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"


#define UART_RATE_HZ 6
#define DISPLAY_RATE_HZ 20
#define CONTROL_RATE_HZ 100
#define SAMPLE_RATE_HZ 200
#define TICK_RATE_HZ 200

<<<<<<< HEAD
/*******************************************************
    Converts tick rates and signals a tick event with set frequencys
********************************************************/
void
SysTickIntHandler(void)

=======
void SysTickIntHandler(void)
>>>>>>> 52256ac958bee3e8f1a5435fc2cc391a9ed17f43
{
    //
    // Initiate a conversion
    //
    static uint8_t UARTtickCount = 0;
    const uint8_t ticksPerUART = TICK_RATE_HZ / UART_RATE_HZ;

    static uint8_t displayTickCount = 0;
    const uint8_t ticksPerDisplay = TICK_RATE_HZ / DISPLAY_RATE_HZ;

    static uint8_t tickControl = 0;
    const uint8_t ticksPerControl = TICK_RATE_HZ / CONTROL_RATE_HZ;

    static uint8_t ADCTickCount = 0;
    const uint8_t ticksPerSample = TICK_RATE_HZ / SAMPLE_RATE_HZ;

    if (++UARTtickCount >= ticksPerUART)
    {                       // Signal a slow tick
        UARTtickCount = 0;
        UARTTick = true;
    }
    if (++displayTickCount >= ticksPerDisplay)
    {                       // Signal a slow tick
        displayTickCount = 0;
        dispTick = true;
    }
    if(++tickControl >= ticksPerControl)
    {                       // Signals a control tick
        tickControl = 0;
        controlTick = true;
    }
    if(++ADCTickCount >= ticksPerSample)
    {                       // signals a ADC tick
        ADCTickCount = 0;
        ADCProcessorTrigger(ADC0_BASE, 3);
        g_ulSampCnt++;
    }
}

//*****************************************************************************
// Initialisation functions for the clock (incl. SysTick), ADC, display
//*****************************************************************************
void
initClock (void)
{
    // Set the clock rate to 20 MHz
    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ);
    //
    // Set up the period for the SysTick timer.  The SysTick timer period is
    // set as a function of the system clock.
    SysTickPeriodSet(SysCtlClockGet() / TICK_RATE_HZ);
    //
    // Register the interrupt handler
    SysTickIntRegister(SysTickIntHandler);

    IntPrioritySet(INT_GPIOC, 0x03);
    //
    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();
}



