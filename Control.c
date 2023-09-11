/*
 * Control.c
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#include "Config.h"
#include "Control.h"
#include "circBufT.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

// PWM Main Motor configuration
#define PWM_MAIN_MOTOR_START_DUTY     2
#define PWM_MAIN_MOTOR_DIVIDER_CODE   SYSCTL_PWMDIV_4

// PWM Main Motor Limits
#define PWM_MAX_DUTY  70
#define PWM_MIN_DUTY  2

// PWM Duty Offset to get duty value for landing
#define PWM_LANDING_OFFSET 90

// PWM  Tail Motor configuration
#define PWM_TAIL_MOTOR_START_DUTY     2
#define PWM_TAIL_MOTOR_DIVIDER_CODE   SYSCTL_PWMDIV_4

// PWM Tail Motor Limits
#define PWM_TAIL_MOTOR_MAX_DUTY       70
#define PWM_TAIL_MOTOR_MIN_DUTY       2

// PWM Frequency
#define PWM_FREQUENCY 50 //Actual frequency will be 4x this value
#define PWM_DIVIDER   4


//  PWM Hardware Details M0PWM7 (gen 3)
//  ---Main Rotor PWM: PC5
#define PWM_MAIN_MOTOR_BASE        PWM0_BASE
#define PWM_MAIN_MOTOR_GEN         PWM_GEN_3
#define PWM_MAIN_MOTOR_OUTNUM      PWM_OUT_7
#define PWM_MAIN_MOTOR_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_MOTOR_PERIPH_PWM  SYSCTL_PERIPH_PWM0
#define PWM_MAIN_MOTOR_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_MOTOR_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_MOTOR_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_MOTOR_GPIO_PIN    GPIO_PIN_5

// PWM Hardware Details M1PWM5 (gen 2)
// tail Rotor PWM: PF1
#define PWM_TAIL_MOTOR_BASE        PWM1_BASE
#define PWM_TAIL_MOTOR_GEN         PWM_GEN_2
#define PWM_TAIL_MOTOR_OUTNUM      PWM_OUT_5
#define PWM_TAIL_MOTOR_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_MOTOR_PERIPH_PWM  SYSCTL_PERIPH_PWM1
#define PWM_TAIL_MOTOR_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_MOTOR_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_MOTOR_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_MOTOR_GPIO_PIN    GPIO_PIN_1

// Main motor gains
#define MAIN_MOTOR_KP 100
#define MAIN_MOTOR_KI 40
#define MAIN_MOTOR_KD 50

// Tail motor gains
#define TAIL_MOTOR_KP 150
#define TAIL_MOTOR_KI 70
#define TAIL_MOTOR_KD 300

// steady value constants
#define TAILSTEADY 30

// Buffer size for calculating derivative control
#define DERIVATIVE_BUFF_SIZE 10

// time constant
#define PI_TIME_STEP 1/200

/*********************************************************
 * Returns proportional response for given error and Kp
 *********************************************************/

int32_t KpControl(int32_t error, int32_t Kp)
{
    return(error * Kp);
}

/*********************************************************
 * Returns integral response for given error and Ki
 *********************************************************/

int32_t KiControl(int32_t error, int32_t* intergral, int32_t frequency, int32_t Ki)
{
    *intergral += (error * Ki) / frequency;
    return *intergral;
}

/*********************************************************
 * Returns derivative response for given Kd
 * using the difference between the current error and the 10th last error for the slope
 *********************************************************/

int32_t KdControl(int32_t error, circBufSigned_t* prevErrors, int32_t frequency, int32_t Kd)
{
    int32_t KdOut;
    KdOut = ((error - prevErrors->data[prevErrors->windex]) * Kd) / frequency ;
    swriteCircBuf(prevErrors, error);
    return (KdOut);
}

/*********************************************************
 * Initialize tail motor control struct
 *********************************************************/

void initTailControl()
{
    tailControl.Kd = TAIL_MOTOR_KD;
    tailControl.Ki = TAIL_MOTOR_KI;
    tailControl.Kp = TAIL_MOTOR_KP;
    tailControl.frequency = PWM_FREQUENCY;
    tailControl.intergral = 0;
    sinitCircBuf (&tailControl.prevErrors, DERIVATIVE_BUFF_SIZE);
    tailControl.target = 0;
    tailControl.hoverDuty = TAILSTEADY;
}

/*********************************************************
 * Initialize main motor control struct
 *********************************************************/

void initMainControl()
{
    mainControl.Kd = MAIN_MOTOR_KD;
    mainControl.Ki = MAIN_MOTOR_KI;
    mainControl.Kp = MAIN_MOTOR_KP;
    mainControl.frequency = PWM_FREQUENCY;
    mainControl.intergral = 0;
    sinitCircBuf (&mainControl.prevErrors, DERIVATIVE_BUFF_SIZE);
    mainControl.target = 0;
    mainControl.hoverDuty = 0;
}

/*********************************************************
 * Initialize main motor PWM output on (PC5)
 *********************************************************/

void initialiseMainMotorPWM (void)
{
    SysCtlPeripheralEnable(PWM_MAIN_MOTOR_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_MOTOR_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_MOTOR_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_MOTOR_GPIO_BASE, PWM_MAIN_MOTOR_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY;

    PWMGenPeriodSet(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_OUTNUM,
        ui32Period * PWM_MAIN_MOTOR_START_DUTY / 100);

    PWMGenEnable(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_OUTBIT, false);
}

/*********************************************************
 * Enables main motor PWM output
 *********************************************************/

void enableMainMotorPWM (void)
{
    PWMOutputState(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_OUTBIT, true);
}

/*********************************************************
 * Disables main motor PWM output
 *********************************************************/

void disableMainMotorPWM (void)
{
    PWMOutputState(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_OUTBIT, false);
}

/*********************************************************
 * Initialize main motor PWM output on (PF1)
 *********************************************************/
void initialiseTailMotorPWM (void)
{
    SysCtlPeripheralEnable(PWM_TAIL_MOTOR_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_MOTOR_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_MOTOR_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_MOTOR_GPIO_BASE, PWM_TAIL_MOTOR_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the initial PWM parameters
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / PWM_FREQUENCY;

    PWMGenPeriodSet(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_OUTNUM,
        ui32Period * PWM_TAIL_MOTOR_START_DUTY / 100);


    PWMGenEnable(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_GEN);

    // Disable the output.  Repeat this call with 'true' to turn O/P on.
    PWMOutputState(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_OUTBIT, false);
}

/*********************************************************
 * Enables tail motor PWM output
 *********************************************************/

void enableTailMotorPWM (void)
{
    PWMOutputState(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_OUTBIT, true);
}

/*********************************************************
 * Enables tail motor PWM output
 *********************************************************/

void disableTailMotorPWM (void)
{
    PWMOutputState(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_OUTBIT, false);
}

/********************************************************
 * Function to set the freq, duty cycle of tail motor
 ********************************************************/

void setTailMotorPWM (uint32_t ui32Freq, int32_t i32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    if (i32Duty > PWM_TAIL_MOTOR_MAX_DUTY) {
        i32Duty = PWM_TAIL_MOTOR_MAX_DUTY;
    } else if (i32Duty < PWM_TAIL_MOTOR_MIN_DUTY) {
        i32Duty = PWM_TAIL_MOTOR_MIN_DUTY;
    }

    tailDuty = i32Duty;

    PWMGenPeriodSet(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_MOTOR_BASE, PWM_TAIL_MOTOR_OUTNUM,
        ui32Period * i32Duty / 100);
}

/********************************************************
 * Finds the minimum absolute value of two numbers
 ********************************************************/

static int32_t abs_min(int32_t num1, int32_t num2)
{
    if (abs(num1) <= abs(num2)) {
        return (num1);
    } else {
        return (num2);
    }
}

/********************************************************
 * Converts yaw angle from degrees to raw value
 ********************************************************/

static int32_t YawPercentageConversion(int32_t PercentValue)
{
    return (PercentValue*448)/360;
}

/********************************************************
 * Finds yaw error and returns the smallest error
 ********************************************************/

static int32_t getYawError()
{
    int32_t distance;
    int32_t splitdistance;
    int32_t RawHeadingTarget = YawPercentageConversion(targetHeading);
    if (RawHeadingTarget > heading) {
        distance = RawHeadingTarget - heading;
        splitdistance = -(448 - distance);
    } else {
        distance = -(heading - RawHeadingTarget);
        splitdistance = (448 + distance);
    }

    return(abs_min(distance, splitdistance));
}

/********************************************************
 * Calculates PID Control for the tail motor
 ********************************************************/

void TailMotor_PID (bool intergralReset)
{
    int32_t error;
    error = getYawError();

    if ((abs(error) < 2) && intergralReset && notIntergralResetBlock) {
        tailControl.intergral = 0;
        notIntergralResetBlock = false;
    }
    int32_t response;
    int32_t Kp = KpControl(error, tailControl.Kp);
    int32_t Ki = KiControl(error, &tailControl.intergral, tailControl.frequency, tailControl.Ki );

    int32_t Kd = KdControl(error, &tailControl.prevErrors, tailControl.frequency, tailControl.Kd);
    response = tailControl.hoverDuty + (Kp + Ki + Kd) / 1000;
    setTailMotorPWM (PWM_FREQUENCY , response);
}

/********************************************************
 * Drive the tail motor rotating the helicopter clockwise
 ********************************************************/

void TailMotorFindRef ()
{
    int32_t error = 100;
    int32_t response;
    int32_t proportional = TAIL_MOTOR_KP * error;
    static int32_t intergral = 0;
    intergral += (error * TAIL_MOTOR_KI) / 200;
    response = TAILSTEADY + (proportional + intergral) / 1000;
    setTailMotorPWM (PWM_FREQUENCY , response);
}

/********************************************************
 * Function to set the freq, duty cycle of main motor
 ********************************************************/

void setMainMotorPWM (uint32_t ui32Freq, int32_t i32Duty)
{
    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period =
        SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    if (i32Duty > PWM_MAX_DUTY) {
        i32Duty = PWM_MAX_DUTY;
    } else if (i32Duty < PWM_MIN_DUTY) {
        i32Duty = PWM_MIN_DUTY;
    }

    mainDuty = i32Duty;
    PWMGenPeriodSet(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_MOTOR_BASE, PWM_MAIN_MOTOR_OUTNUM,
        ui32Period * i32Duty / 100);
}

/********************************************************
 * Convert altitude from percentage to raw value
 ********************************************************/

int32_t AltitudePercentageConversion(int32_t percent_value){
    int32_t raw = (groundVolt-(((groundVolt-skyVolt)*percent_value)/100));
    return raw;
}

/********************************************************
 * Calculates PID Control for the main motor
 ********************************************************/

void MainMotor_PID ()
{

    int32_t error = altitude - AltitudePercentageConversion(targetAltitude) ;
    int32_t response;
    int32_t Kp = KpControl(error, mainControl.Kp);
    int32_t Ki = KiControl(error, &mainControl.intergral, mainControl.frequency, mainControl.Ki );
    int32_t Kd = KdControl(error, &mainControl.prevErrors, mainControl.frequency, mainControl.Kd);
    response = mainControl.hoverDuty + (Kp + Ki + Kd) / 1000;
    setMainMotorPWM (PWM_FREQUENCY , response);
}

/********************************************************
 * Sets main motor duty to landing value
 ********************************************************/

void MainMotorLand()
{
    setMainMotorPWM (PWM_FREQUENCY, (mainControl.hoverDuty * PWM_LANDING_OFFSET) / 100);
}

/********************************************************
 * Slowly increases tail and main motor duty cycle to find hover values
 ********************************************************/

void findHoverVals(bool* complete)
{
    static uint32_t tickRise = 0;
    static int16_t lastHeading = 0;
    const uint32_t ticksPerRise = 1000;
    if (++tickRise >= ticksPerRise)
    {
        tickRise = 0;
        mainControl.hoverDuty++;
    }
    if (lastHeading - heading < 0) {
        tailControl.hoverDuty -= 2;
        if (tailControl.hoverDuty > PWM_TAIL_MOTOR_MAX_DUTY) {
            tailControl.hoverDuty = PWM_TAIL_MOTOR_MAX_DUTY;
        }
    } else if (lastHeading - heading > 0) {
        tailControl.hoverDuty += 2;
        if (tailControl.hoverDuty < PWM_TAIL_MOTOR_MIN_DUTY) {
            tailControl.hoverDuty = PWM_TAIL_MOTOR_MIN_DUTY;
        }
    }
    lastHeading = heading;
    setTailMotorPWM (PWM_FREQUENCY, tailControl.hoverDuty);
    setMainMotorPWM (PWM_FREQUENCY, mainControl.hoverDuty);
    if (altitude <= groundVolt - 10) {
        *complete = true;
        mainControl.hoverDuty-=5;
        tailControl.hoverDuty-=4;
    }
}
