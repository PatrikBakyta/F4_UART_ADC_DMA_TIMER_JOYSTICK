/*
 * lib.h
 *
 *  Created on: 17. 12. 2016
 *      Author: Patrik Bakyta
 */

#ifndef LIB_H_
#define LIB_H_


void initSYSTEMCLOCK(void);

void initTIMERwithINTERRUPT(uint16_t TimerVal);
extern "C" void TIM2_IRQHandler(void);

void initUSART(void);

void initADCwithDMA(void);

int *remap(uint8_t ADC_Values[2]);
char *INTconversionCHAR(uint8_t value);


#endif /* LIB_H_ */
