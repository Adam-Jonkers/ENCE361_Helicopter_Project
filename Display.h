/*
 * Display.h
 *
 *  Created on: 13/05/2023
 *      Author: Adam Jonkers
 */

#ifndef GROUP_MINUS_2_DISPLAY_H_
#define GROUP_MINUS_2_DISPLAY_H_

#define NUMBER_OF_SCREENS 4

void initDisplay (void);

void displayScreen(int32_t skyVolt, int32_t groundVolt);

void initialiseUSB_UART (void);

void UARTSend (char *pucBuffer);

void UARTDisplayInfo(int32_t skyVolt, int32_t groundVolt, char state[10]);

#endif /* GROUP_MINUS_2_DISPLAY_H_ */
