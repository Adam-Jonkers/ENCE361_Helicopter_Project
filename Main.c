//*****************************************************************************
//
// Heli.c - Control a Helicopter
//
// Author:  Adam Jonkers , Laurence Watson
// Last modified:	3.5.2023
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/interrupt.h"

#include "Config.h"
#include "Control.h"
#include "Display.h"
#include "Input.h"
#include "scheduler.h"

//*****************************************************************************
// Constants
//*****************************************************************************
#define BUF_SIZE 10
#define ONE_VOLT 1241
            //1241 @1v
            //2482 @2v

//*****************************************************************************
// Global Variables
//*****************************************************************************
enum state {landed = 0, takeoff = 1, flying = 2, landing = 3};
enum state heliState = landed;
uint8_t displayState;
bool    switchStatus;
bool    previousSwitch;

#define INTERGRAL_RESET_ON true
#define INTERGRAL_RESET_OFF false

//********************************************************
// Prototypes
//********************************************************
void SysTickIntHandler (void);
void initClock (void);
void initSysTick (void);

//********************************************************
// state functions
//********************************************************

//landed state
// will remain in the landed state until the take off switch changes.
// this converts the state to takeoff
void landedFunc()
{
    if (UARTTick) {
        UARTTick = false;
        UARTDisplayInfo(skyVolt, groundVolt, "Landed");
    }
    switchStatus = readSwitch();
    if (switchStatus && !previousSwitch) {
        heliState = takeoff;
        enableMainMotorPWM();
        enableTailMotorPWM();
    }
    previousSwitch = switchStatus;
}

// take off state
// will stay in this state until the hover value is found and the facing forward point is found.
// while in take off state the helicopter will increase its altitude at a constant rate until
// take off is achieved it will then set the hover value. The yaw value will will interpret a
// constant error of 100 causing it to begin to rotate which it will continue to do until
// forwards is found. then it will stay pointed to the front. when both conditions are met it
// will switch states to flying
void takeOffFunc()
{
    static bool hoverValSet = false;
    if (UARTTick == true) {
        UARTTick = false;
        UARTDisplayInfo(skyVolt, groundVolt, "Take_Off");
    }
    if (controlTick == true) {
        if (hoverValSet == true && headingSet == false) {
            controlTick = false;
            TailMotorFindRef ();
            setMainMotorPWM (mainControl.frequency , mainControl.hoverDuty);
        } else if (hoverValSet == false){
            findHoverVals(&hoverValSet);
        } else if (hoverValSet == true && headingSet == true) {
            targetHeading = 0;
            TailMotor_PID (INTERGRAL_RESET_ON);
            setMainMotorPWM (mainControl.frequency , mainControl.hoverDuty);
        }
    }
    if (headingSet == true && hoverValSet == true && abs(heading) < 7) {
        heliState = flying;
        hoverValSet = false;
        tailControl.intergral = 0;
    }
    switchStatus = readSwitch();
    if (switchStatus != true) {
        heliState = landing;
        notIntergralResetBlock = true;
    }
}

// flying state
// PID control has full control of the movement of the helicopter to move towards the target.
// when passing by the target yaw the first time the target integral component of PID is
// reset to avoid a excessive buildup of integral control
// when the switch is changed it will change the current state to be landing.
// only flying state interacts with buttons
void flyingFunc()
{
    GetUserControl();
    if (UARTTick == true) {
        UARTTick = false;
        UARTDisplayInfo(skyVolt, groundVolt, "Flying");
    }
    if (controlTick == true) {
        controlTick = false;
        TailMotor_PID (INTERGRAL_RESET_ON);
        MainMotor_PID ();
    }
    switchStatus = readSwitch();
    if (switchStatus != true) {
        heliState = landing;
        notIntergralResetBlock = true;
    }
}
// landing state
// first orientates the helicopter to forwards using PID control with a reset to integral
// when facing forwards. once the helicopter has been facing forwards within +- 7 intervals
// for 2 seconds then it will change the PWM supplied to 90% of the hover value allowing for a
// smooth descent once the helicoper is on the ground then the state will switch to Landed.
void landingFunc()
{
    static bool atRef = false;
    static uint32_t atRefCount = 0;

    if (UARTTick == true) {
        UARTTick = false;
        UARTDisplayInfo(skyVolt, groundVolt, "Landing");
    }

    if (atRef == true) {
        if (controlTick) {
            controlTick = false;
            TailMotor_PID (INTERGRAL_RESET_OFF);
            MainMotorLand();
        }
    } else {
        if (controlTick == true) {
            controlTick = false;
            targetHeading = 0;
            TailMotor_PID (INTERGRAL_RESET_ON);
            MainMotor_PID ();

            if (abs(heading) < 7) {
                atRefCount++;
            } else {
                atRefCount = 0;
            }
            if (atRefCount > 200 /*CONTROL_RATE_HZ*/) {
                atRef = true;
            }
        }
    }


    if (altitude > groundVolt - 10 && atRef) {
        heliState = landed;
        targetAltitude = 0;
        atRef = false;
        disableMainMotorPWM();
        disableTailMotorPWM();
    }
}

//********************************************************
// Helicopter Main
//********************************************************
int main(void)

{
	initClock ();
	initADC ();
	initDisplay ();
	initCircBuf (&g_inBuffer, BUF_SIZE);

	initialiseMainMotorPWM ();
	initialiseTailMotorPWM ();
	initialiseUSB_UART ();

	initInputs();

	initTailControl();
	initMainControl();

    previousSwitch = readSwitch();

    // Enable interrupts to the processor.
    IntMasterEnable();


    setUpBuffer();


    // Calculate and display the rounded mean of the buffer contents
    groundVolt = getBufferMean();
    skyVolt = groundVolt - ONE_VOLT;

	while (1)
	{

	    altitude = getBufferMean();
        if (dispTick) {
            dispTick = false;
            displayScreen (skyVolt, groundVolt);
        }

        //Switch Statement for changing between states

        switch (heliState)
        {
        case landed:
            landedFunc();
            break;
        case takeoff:
            takeOffFunc();
            break;
        case flying:
            flyingFunc();
            break;
        case landing:
            landingFunc();
            break;
        }
	}
}

