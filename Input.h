/*
 * quadDec.h
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#ifndef GROUP_MINUS_2_INPUT_H_
#define GROUP_MINUS_2_INPUT_H_


void QuadIntHandler (void);
void RefInitHandler (void);
void initQuadDec (void);

void initSwitch (void);

void ResetIntHandler (void);
void initReset (void);
void initInputs (void);

void ADCIntHandler(void);
void initADC (void);

void setUpBuffer(void);
int32_t getBufferMean(void);

bool readSwitch(void);
void GetUserControl(void);

#endif /* GROUP_MINUS_2_INPUT_H_ */
