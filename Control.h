/*
 * Control.h
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#ifndef GROUP_MINUS_2_CONTROL_H_
#define GROUP_MINUS_2_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

#include "Config.h"
#include "circBufT.h"

int32_t KpControl(int32_t error, int32_t Kp);
int32_t KiControl(int32_t error, int32_t* intergral, int32_t frequency, int32_t Ki);
int32_t KdControl(int32_t error, circBufSigned_t* prevErrors, int32_t frequency, int32_t Kd);

void initTailControl(void);
void initMainControl(void);

void initialiseMainMotorPWM (void);
void enableMainMotorPWM (void);
void disableMainMotorPWM (void);

void initialiseTailMotorPWM (void);
void enableTailMotorPWM (void);
void disableTailMotorPWM (void);

void setTailMotorPWM (uint32_t ui32Freq, int32_t i32Duty);

void TailMotor_PID (bool intergralReset);
void TailMotorFindRef (void);

void setMainMotorPWM (uint32_t ui32Freq, int32_t i32Duty);
void MainMotor_PID (void);
void MainMotorLand (void);

void findHoverVals(bool* complete);

#endif /* GROUP_MINUS_2_CONTROL_H_ */
