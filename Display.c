/*
 * Display.c
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"
#include "driverlib/pwm.h"

#include "OrbitOLED/OrbitOLEDInterface.h"

#include "Config.h"
#include "Control.h"


//---USB Serial comms: UART0, Rx:PA0 , Tx:PA1
#define BAUD_RATE 9600
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX


void initDisplay (void)
{
    // Intialise the Orbit OLED display
    OLEDInitialise ();
}

//*****************************************************************************
//
// Functions for displaying to the on board display
//
//*****************************************************************************
// display screen for the orbit OLED
// contains altitude and desired altitude, yaw and desired yaw , main PWM , tail PWM
void displayScreen(int32_t skyVolt, int32_t groundVolt)
{
    char string[17];  // 16 characters across the display
    float headFloat = ((360 * heading) / 448);  // remove this line
    int head = (int)headFloat;
    int headDec  = abs(((36000 * heading) / 448) - (head * 100));

    // Form a new string for the line.  The maximum width specified for the
    //  number field ensures it is displayed right justified.

    usnprintf (string, sizeof(string), "Alt= %2d%% [%2d%%]  ", (((altitude - groundVolt) * 100) / (skyVolt - groundVolt)),targetAltitude);
    OLEDStringDraw (string, 0, 0);

    usnprintf (string, sizeof(string), "yaw= %4d [%2d]    " , head,targetHeading);
    OLEDStringDraw (string, 0, 1);
    usnprintf (string, sizeof(string), "Main Duty: %3d%%   ", mainDuty);
    OLEDStringDraw (string, 0, 2);
    usnprintf (string, sizeof(string), "Tail Duty: %3d%%   ", tailDuty);
    OLEDStringDraw (string, 0, 3);
}

//********************************************************
// initialiseUSB_UART - 8 bits, 1 stop bit, no parity
//********************************************************
void
initialiseUSB_UART (void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    //
    SysCtlPeripheralEnable(UART_USB_PERIPH_UART);
    SysCtlPeripheralEnable(UART_USB_PERIPH_GPIO);
    //
    // Select the alternate (UART) function for these pins.
    //
    GPIOPinTypeUART(UART_USB_GPIO_BASE, UART_USB_GPIO_PINS);
    GPIOPinConfigure (GPIO_PA0_U0RX);
    GPIOPinConfigure (GPIO_PA1_U0TX);

    UARTConfigSetExpClk(UART_USB_BASE, SysCtlClockGet(), BAUD_RATE,
            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
            UART_CONFIG_PAR_NONE);
    UARTFIFOEnable(UART_USB_BASE);
    UARTEnable(UART_USB_BASE);
}

//**********************************************************************
// Transmit a string via UART0
//**********************************************************************
void UARTSend (char *pucBuffer)
{
    // Loop while there are more characters to send.
    while(*pucBuffer)
    {
        // Write the next character to the UART Tx FIFO.
        UARTCharPut(UART_USB_BASE, *pucBuffer);
        pucBuffer++;
    }
}


void UARTDisplayInfo(int32_t skyVolt, int32_t groundVolt, char state[10])
{
    int32_t Heading_degrees = (heading*180)/224;
    int32_t AltitudePercent = (((altitude - groundVolt) * 100) / (skyVolt - groundVolt));

    usnprintf (statusStr, sizeof(statusStr), "Alt %%%2d | [%%%2d]\r\n", AltitudePercent, targetAltitude);
    UARTSend (statusStr);
    usnprintf (statusStr, sizeof(statusStr), "Heading: %5d | [%5d] \r\n", Heading_degrees, targetHeading);
    UARTSend (statusStr);
    usnprintf (statusStr, sizeof(statusStr), "Tail Duty:%%%3d | Main:%%%3d\r\n", tailDuty, mainDuty);
    UARTSend (statusStr);
    usnprintf (statusStr, sizeof(statusStr), "State Duty: %s \r\n\n", state);
    UARTSend (statusStr);
}

