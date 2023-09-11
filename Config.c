/*
 * Gonfig.c
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#include <stdint.h>
#include <stdbool.h>
#include "circBufT.h"
#include "Config.h"

#define MAX_STR_LEN 40

volatile int32_t quadState;    // Current State of the quadratic decoder
volatile int16_t heading = 0;    // Heading from -223 to 224

char statusStr[MAX_STR_LEN + 1];
volatile uint8_t UARTTick = false;
volatile uint8_t dispTick = false;
volatile uint8_t controlTick = false;

int32_t targetAltitude = 0;
int32_t targetHeading = 0;

int32_t altitude;

circBuf_t g_inBuffer;        // Buffer of size BUF_SIZE integers (sample values)
uint32_t g_ulSampCnt = 0;    // Counter for the interrupts

int32_t tailDuty;
int32_t mainDuty;

bool headingSet = false;

bool notIntergralResetBlock = true;

volatile int32_t groundVolt;    // Voltage reading from ADC at minimum ALT
volatile int32_t skyVolt;    // Voltage at maximum ALT

control_t tailControl;    // Tail motor control
control_t mainControl;    // Main motor control
