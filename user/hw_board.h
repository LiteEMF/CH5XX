/* 
*   BSD 2-Clause License
*   Copyright (c) 2022, LiteEMF
*   All rights reserved.
*   This software component is licensed by LiteEMF under BSD 2-Clause license,
*   the "License"; You may not use this file except in compliance with the
*   License. You may obtain a copy of the License at:
*       opensource.org/licenses/BSD-2-Clause
* 
*/


#ifndef _hw_board_h
#define _hw_board_h
#include "emf_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************************************
** Defined
*******************************************************************************************************/
#if PROJECT_KM

#elif PROJECT_GAMEPAD
	#if GAMEPAD1
		//timer
		#define HW_TIMER_MAP			{TIMER0, VAL2FLD(TIMER_FREQ,1000)|VAL2FLD(TIMER_PRI,1)}

		//uart
		#define HW_UART_MAP {\
			{PD_00, PIN_NULL,	0, 0, UART0, VAL2FLD(UART_BAUD,1000000)},	\
			{PC_07, PC_06, 		0, 64, UART0, VAL2FLD(UART_BAUD,1000000)|VAL2FLD(UART_PRI,1)},		\
			}

		//adc
		#define HW_ADC_MAP {	\
			{PB_00,0,VAL2FLD(ADC_CHANNEL,0)},		\
			{PA_01,0,VAL2FLD(ADC_CHANNEL,1) | VAL2FLD(ADC_PULL,1)}			\
			}
		//spi
		#define HW_SPI_HOST_MAP {\
			{PB_07,PB_05,PB_06,PB_04,SPI0,VAL2FLD(SPI_BADU,1000000)},	\
			{PC_03,PC_01,PC_02,PC_01,SPI0,VAL2FLD(SPI_BADU,1000000)},	\
			}

		//pwm
		#define HW_PWM_MAP {	\
			{PC_04,PWM1,VAL2FLD(PWM_FREQ,200000)|VAL2FLD(PWM_ACTIVE,1)},		\
			{PC_05,PWM2,VAL2FLD(PWM_FREQ,200000)|VAL2FLD(PWM_ACTIVE,0)}			\
			}

	#endif
#endif




#ifdef __cplusplus
}
#endif
#endif





