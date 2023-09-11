/*
 * Config.h
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#ifndef GROUP_MINUS_2_CONFIG_H_
#define GROUP_MINUS_2_CONFIG_H_

#define MAX_STR_LEN 40

#include <stdint.h>
#include <stdbool.h>
#include "circBufT.h"
#include "signedCircBufT.h"

// Struct to store motor control information

typedef struct {
    circBufSigned_t prevErrors;
    int32_t intergral;
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
    int32_t target;
    int32_t frequency;
    int32_t hoverDuty;
} control_t;

extern volatile int32_t quadState;

extern char statusStr[MAX_STR_LEN + 1];
extern volatile uint8_t UARTTick;
extern volatile uint8_t dispTick;
extern volatile uint8_t controlTick;

extern int32_t targetAltitude;
extern int32_t targetHeading;

extern int32_t altitude;
extern volatile int16_t heading;

extern circBuf_t g_inBuffer;
extern uint32_t g_ulSampCnt;

extern int32_t tailDuty;
extern int32_t mainDuty;

extern bool headingSet;

extern bool notIntergralResetBlock;

extern volatile int32_t groundVolt;
extern volatile int32_t skyVolt;

extern control_t tailControl;
extern control_t mainControl;


#endif /* GROUP_MINUS_2_CONFIG_H_ */
