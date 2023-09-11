/*
 * quadDec.c
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/adc.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

#include "circBufT.h"
#include "buttons4.h"

#include "Config.h"

// Quadrature Decoding Phases
#define PHASE_1 0x00000000
#define PHASE_2 0x00000002
#define PHASE_3 0x00000003
#define PHASE_4 0x00000001

#define BUF_SIZE 10

/*********************************************************
 * Interrupt Handler for quadratic encoder
 *********************************************************/

void QuadIntHandler (void)
{
    uint32_t intStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    GPIOIntClear(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

    //Change on channel A
    if (intStatus & GPIO_INT_PIN_0) {
        switch (quadState)
        {
        case PHASE_1:
            quadState = PHASE_4;
            heading--;
            break;
        case PHASE_2:
            quadState = PHASE_3;
            heading++;
            break;
        case PHASE_3:
            quadState = PHASE_2;
            heading--;
            break;
        case PHASE_4:
            quadState = PHASE_1;
            heading++;
            break;
        }
    }
    //Change on channel B
    else if (intStatus & GPIO_INT_PIN_1) {
        switch (quadState)
        {
        case PHASE_1:
            quadState = PHASE_2;
            heading++;
            break;
        case PHASE_2:
            quadState = PHASE_1;
            heading--;
            break;
        case PHASE_3:
            quadState = PHASE_4;
            heading++;
            break;
        case PHASE_4:
            quadState = PHASE_3;
            heading--;
            break;
        }
    }

    // lock the heading between -179 and 180 degrees
    if (heading > 224)
    {
        heading = -223;
    }
    else if (heading < -223)
    {
        heading = 224;
    }

}

/*********************************************************
 * Interrupt Handler for quadratic encoder reference point
 *********************************************************/

void RefInitHandler (void)
{
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    heading = 0;
    headingSet = true;
}

/*********************************************************
 * Initialize the quadratic encoder
 *********************************************************/

void initQuadDec (void)
{
    // Initialize channel A & B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
        continue;
    }

    GPIOIntRegister(GPIO_PORTB_BASE, QuadIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_BOTH_EDGES);

    quadState = GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

    IntPrioritySet(INT_GPIOB, 0x01);

    // Initialize reference
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
        continue;
    }

    GPIOIntRegister(GPIO_PORTC_BASE, RefInitHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);

    IntPrioritySet(INT_GPIOC, 0x01);

    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);

}

/*********************************************************
 * Initialize switch A7 as input
 *********************************************************/

void initSwitch (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
        continue;
    }
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);
}

/*********************************************************
 * Interrupt Handler for reset
 *********************************************************/

void ResetIntHandler (void)
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_6);
    SysCtlReset();
}

/*********************************************************
 * Initialize reset
 *********************************************************/

void initReset (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
        continue;
    }
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);

    GPIOIntRegister(GPIO_PORTA_BASE, ResetIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_6);

    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_FALLING_EDGE);

    IntPrioritySet(INT_GPIOA, 0x00);

    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_6);
}

/*********************************************************
 * Initialize inputs
 *********************************************************/

void initInputs (void)
{
    initButtons ();
    initQuadDec ();
    initSwitch  ();
    initReset   ();
}


// ADC FUNCTIONS

//*****************************************************************************
//
// The handler for the ADC conversion complete interrupt.
// Writes to the circular buffer.
//
//*****************************************************************************
void ADCIntHandler(void)
{
    uint32_t ulValue;

    //
    // Get the single sample from ADC0.  ADC_BASE is defined in
    // inc/hw_memmap.h
    ADCSequenceDataGet(ADC0_BASE, 3, &ulValue);
    //
    // Place it in the circular buffer (advancing write index)
    writeCircBuf (&g_inBuffer, ulValue);
    //
    // Clean up, clearing the interrupt
    ADCIntClear(ADC0_BASE, 3);
}

/*********************************************************
 * Initialize ADC
 *********************************************************/

void initADC (void)
{
    //
    // The ADC0 peripheral must be enabled for configuration and use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // single-ended mode (default) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.  For more
    // on the ADC sequences and steps, refer to the LM3S1968 datasheet.
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE |
                             ADC_CTL_END);

    //
    // Since sample sequence 3 is now configured, it must be enabled.
    ADCSequenceEnable(ADC0_BASE, 3);

    //
    // Register the interrupt handler
    ADCIntRegister (ADC0_BASE, 3, ADCIntHandler);

    IntPrioritySet(INT_ADC0SS3, 0x02);

    //
    // Enable interrupts for ADC0 sequence 3 (clears any outstanding interrupts)
    ADCIntEnable(ADC0_BASE, 3);
}

/*********************************************************
 * Fill sample buffer with initial values
 *********************************************************/

void setUpBuffer()
{
    uint16_t i;
    int32_t sum = 0;
    while (g_ulSampCnt < 100) {
        continue;
    }

    for (i = 0; i < BUF_SIZE; i++) {
        sum = sum + readCircBuf (&g_inBuffer);
    }
}

/*********************************************************
 * Return mean value from buffer
 *********************************************************/

int32_t getBufferMean()
{
    uint16_t i;
    int32_t sum = 0;

    sum = 0;
    for (i = 0; i < BUF_SIZE; i++) {
        sum = sum + readCircBuf (&g_inBuffer);
    }
    // Calculate and display the rounded mean of the buffer contents
    return ((2 * sum + BUF_SIZE) / 2 / BUF_SIZE);
}

/*********************************************************
 * Returns value from switch A7
 *********************************************************/

bool readSwitch()
{
    if ((GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_7) & GPIO_PIN_7) == 0) {
        return false;
    } else {
        return true;
    }
}



/*********************************************************
 * Polls buttons for user inputs and handles any input received
 *********************************************************/
void GetUserControl()
{
    uint8_t butState;
    updateButtons();

    butState = checkButton (LEFT);
    if (butState == PUSHED) {
        notIntergralResetBlock = true;
        tailControl.intergral = 0;
        targetHeading -= 15;
        if (targetHeading < -180) {
            targetHeading += 360;
        }
    }

    butState = checkButton (RIGHT);
    if (butState == PUSHED) {
        notIntergralResetBlock = true;
        tailControl.intergral = 0;
        targetHeading += 15;
        if (targetHeading > 180) {
            targetHeading -= 360;
        }
    }

    butState = checkButton (UP);
    if (butState == PUSHED) {
        mainControl.intergral = 0;
        targetAltitude += 10;
        if (targetAltitude > 100){
            targetAltitude=100;
        }
    }

    butState = checkButton (DOWN);
    if (butState == PUSHED) {
        mainControl.intergral = 0;
        targetAltitude -= 10;
        if(targetAltitude <0){
            targetAltitude=0;
        }
    }
}
