/*
 * joystick.cpp
 *
 *  Created on: 16. 12. 2016
 *      Author: Patrik Bakyta
 */

#include "stm32f4xx.h"
#include <lib.h>

/* Globalne pole pre zapis meranych hodnot z ADC pomocu DMA,
 * hodnoty su inicializovane na defaultnu polohu joysticku */
uint8_t ADC_Values[2] = {128, 128};

int default_values[6] = {128,128,128,128,128,128}; // x_min, x_stred, x_max, y...
int q = 0; // pocitadlo strednej hodnoty
extern bool map; // premenna zapnuteho/vypnuteho mapovania

void InitSYSTEMCLOCK(void) {

	RCC_HSICmd(ENABLE);
	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	RCC_SYSCLKConfig(RCC_CFGR_SW_HSI);
	SystemCoreClockUpdate();

	return;
}

void EnableInterrupt(uint8_t IRQChannel, uint8_t Preemption, uint8_t Sub) {

    NVIC_InitTypeDef nvicStructure;
    nvicStructure.NVIC_IRQChannel = IRQChannel;
    nvicStructure.NVIC_IRQChannelPreemptionPriority = Preemption;
    nvicStructure.NVIC_IRQChannelSubPriority = Sub;
    nvicStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvicStructure);

    return;
}

void InitTIM(TIM_TypeDef* Timer, uint16_t Prescaler, uint32_t Period) {

	if (Timer==TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	} else if (Timer==TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	} else if (Timer==TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	} else if (Timer==TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	} else if (Timer==TIM6) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	} else if (Timer==TIM7) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	} else if (Timer==TIM12) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	} else if (Timer==TIM13) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	} else if (Timer==TIM14) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	}

	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = Prescaler-1;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = Period-1;
	timerInitStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(Timer, &timerInitStructure);

	TIM_Cmd(Timer, ENABLE);

	// Enable TIM update Interrupt source
	TIM_ITConfig(Timer, TIM_IT_Update, ENABLE);
	TIM_ClearITPendingBit(Timer, TIM_IT_Update);

	return;
}

extern "C" void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		/* Code below is executed when TIM update interrupt is set */

		ADC_print(ADC_Values); // vypis meranych hodnot ADC

	}

	return;
}

void InitUART1viaCONVERTER(void) {

	 GPIO_InitTypeDef GPIO_InitStruct;
	 USART_InitTypeDef USART_InitStruct;

	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	 GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	 GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	 GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	 GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 GPIO_Init(GPIOB, &GPIO_InitStruct);

	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);
	 GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	 USART_InitStruct.USART_BaudRate = 9600;
	 USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	 USART_InitStruct.USART_StopBits = USART_StopBits_1;
	 USART_InitStruct.USART_Parity = USART_Parity_No;
	 USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	 USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	 USART_Init(USART1, &USART_InitStruct);

	 USART_Cmd(USART1, ENABLE);

	 // Enable Receive Data register not empty interrupt
	 USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	 USART_ClearITPendingBit(USART1, USART_IT_RXNE);

	 return;
}

extern "C" void USART1_IRQHandler(void) {

	if (USART_GetITStatus(USART1, USART_IT_RXNE)==SET) {

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);

		//uint8_t received_data = USART_ReceiveData(USART1);

		/* Code below is executed when UART receives data
		 * and stores it in variable received_data */

	}

	return;
}

void InitADC1withDMA2(void) {

    ADC_InitTypeDef       ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    DMA_InitTypeDef       DMA_InitStruct;
    GPIO_InitTypeDef      GPIO_InitStruct;

    // Enable DMA2 clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    // Enable GPIOC clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // Enable ADC1 clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // DMA2 Stream0 channel0 configuration
    DMA_InitStruct.DMA_Channel = DMA_Channel_0;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // ADC1's data register
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)&ADC_Values;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = 2;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Reads 8 bit values
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // Stores 8 bit values
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream0, ENABLE);

    // Configure GPIO pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; // PC0 - Channel 10, PC1 - Channel 11
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // The pins are configured in analog mode
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // We don't need any pull up or pull down
    GPIO_Init(GPIOC, &GPIO_InitStruct); // Initialize GPIOC pins with the configuration

    // ADC Common Init
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStruct);

    // ADC1 Init
    ADC_DeInit();
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_8b; // Input voltage is converted into a 8bit int (max 255)
    ADC_InitStruct.ADC_ScanConvMode = ENABLE; // The scan is configured in multiple channels
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE; // Continuous conversion: input signal is sampled more than once
    ADC_InitStruct.ADC_ExternalTrigConv = 0;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right; // Data converted will be shifted to right
    ADC_InitStruct.ADC_NbrOfConversion = 2;
    ADC_Init(ADC1, &ADC_InitStruct); // Initialize ADC with the configuration

    // Select the channels to be read from
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_144Cycles); // PC0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_144Cycles); // PC1

    // Enable DMA request after last transfer (Single-ADC mode)
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    // Enable ADC1 DMA
    ADC_DMACmd(ADC1, ENABLE);
    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    // Start ADC1 Software Conversion
    ADC_SoftwareStartConv(ADC1);

    return;
}

void ADC_print(uint8_t array_vals[2]) {

	int *int_pointer;
	char *char_pointer;

	// premapovanie hodnot
	if (map==true) {
		int_pointer = remap(array_vals);
	}

	// najprv sa posle x potom y
	for (int j=0; j<2; j++) {

		// konverzia hodnoty z ADC na pole charov, funkcia vracia smernik
		if (map==true) {
			char_pointer = INTconversionCHAR(*(int_pointer+j));
		} else {
			char_pointer = INTconversionCHAR(*(array_vals+j));
		}

		int i = *(char_pointer); // na 1. mieste je pocet cifier
		while (i>0) {
			while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
			USART_SendData(USART1,*(char_pointer+i));
			i--;
		}

		// najprv space a pootm new line
		while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
		if (j==0) {
			USART_SendData(USART1,32);
		} else {
			USART_SendData(USART1,13);
		}

	}

	/*
	// priame posielanie hodnot
	while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
	USART_SendData(USART1,array_vals[0]);
	while (USART_GetFlagStatus(USART1,USART_FLAG_TXE)==0) {};
	USART_SendData(USART1,array_vals[1]);
	*/

	return;
}

int *remap(uint8_t array_values[2]) {

	static int int_array[2];
	int rozsah;

	if (q<10) {
		q++;
	}

	for (int j=0; j<2; j++) {

		// nastavenie defaultnych strednych hodnot
		if (q==10) {
			default_values[j*3+1] = array_values[j];
			if (j==1) {
				q = 11;
			}
		}

		// nastavenie novych minimalnych/maximalnych hodnot
		if (array_values[j]<default_values[j*3+0]) {
			default_values[j*3+0] = array_values[j];
		} else if (array_values[j]>default_values[j*3+2]) {
			default_values[j*3+2] = array_values[j];
		}

		// premapovanie
		if (array_values[j]>default_values[j*3+1]) {
			rozsah = default_values[j*3+2]-default_values[j*3+1]; // max-stred
		} else if (array_values[j]<default_values[j*3+1]) {
			rozsah = default_values[j*3+1]-default_values[j*3+0]; // stred-min
		}

		int_array[j] = 128+(array_values[j]-default_values[j*3+1])*(128/(double)rozsah);

		// pre istotu obmedzenie
		if (int_array[j]>255) {
			int_array[j] = 255;
		} else if (int_array[j]<0) {
			int_array[j] = 0;
		}

	}

	return int_array;
}

char *INTconversionCHAR(uint8_t value) {

	int j = 1; // index pola, zacina na 1
	static char char_array[4]; // pole, na 1. mieste pocet cifier hodnoty z ADC (1-3)

	do {
		*(char_array+j) = (char)(value % 10) + '0'; // konverzia z INT na CHAR
		value /= 10;
		j++;
	} while (value);

	*(char_array) = j-1; // teraz uz vieme pocet cifier, zapis na 1. miesto v poli

	return char_array;
}
