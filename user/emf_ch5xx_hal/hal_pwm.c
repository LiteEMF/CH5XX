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

/************************************************************************************************************
**	Description:	
************************************************************************************************************/
#include "hw_config.h"
#ifdef HW_PWM_MAP
#include  "api/api_pwm.h"
#include  "api/api_gpio.h"
#include  "api/api_system.h"

/******************************************************************************************************
** Defined
PWM1/2 使用P2.4/P2.5；1:PWM1/2 使用 P4.3/P4.5	
*******************************************************************************************************/
// #define HW_PWM_MAP {	\
// 			{PC_04,PWM1,VAL2FLD(PWM_FREQ,200000)|VAL2FLD(PWM_ACTIVE,1)},		\
// 			{PC_05,PWM2,VAL2FLD(PWM_FREQ,200000)|VAL2FLD(PWM_ACTIVE,0)}			\
// 			}
/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/
#define SetPWMClk(CK_SE) (PWM_CK_SE = CK_SE)                                  //分频,默认时钟Fsys            
#define SetPWMCycle(Cycle) (PWM_CYCLE = Cycle)                                //设置循环周期
#define SetPWM1Dat(dat) (PWM_DATA = dat)                                      //设置PWM输出占空比
#define SetPWM2Dat(dat) (PWM_DATA2 = dat)


/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_pwm_set_duty(uint16_t id, uint8_t duty)
{
	if(PWM1 == m_pwm_map[id].peripheral){
		SetPWM1Dat(duty); 
	}else if(PWM2 == m_pwm_map[id].peripheral){
		SetPWM2Dat(duty);	
	}

	return true;
}

/*******************************************************************
** Parameters:		
** Returns:	
** Description:	PWM_FREQ_ATT: 188Khz ~ 48M
注意: PWM1, PWM2 频率和IO是相同不能单独配置
*******************************************************************/
bool hal_pwm_init(uint16_t id, uint8_t duty)
{
	uint8_t div;
	uint32_t freq = PWM_FREQ_ATT(id);

	if(PWM_CH_ATT(id) > 4) return false;

	if(0 == freq) freq = PWM_FREQ_DEFAULT;
	if(freq < 188000){
		div =  0xFF;
	}else{
		div = HAL_SYS_FREQ / freq;
	}
	SetPWMClk(div);
	SetPWMCycle(255);       			
	
	PWM_CTRL &= ~bPWM_CLR_ALL;                                                //清空FIFO和计数                                                      
    PWM_CTRL &= ~bPWM_MOD_MFM;
    PWM_CTRL &= ~bPWM_IE_END;                                                  //使能PWM计数周期完成中断
    
	api_gpio_dir(m_pwm_map[id].pin, PIN_OUT, PIN_PULLNONE);
	if((PE_04 == m_pwm_map[id].pin) || (PE_05 == m_pwm_map[id].pin)){
		PIN_FUNC |= bPWM1_PIN_X;		//0:PWM1/2 使用P2.4/P2.5；1:PWM1/2 使用 P4.3/P4.5	
	}

	if(PWM1 == m_pwm_map[id].peripheral){
		PWM_CTRL |= bPWM_OUT_EN;                                            //PWM2输出使能	
		if(PWM_ACTIVE_ATT(id)){	
			PWM_CTRL &= ~bPWM_POLAR;                                        //高电平有效  
		}else{
			PWM_CTRL |= bPWM_POLAR;                                         //低电平有效
		}	
	}else if(PWM2 == m_pwm_map[id].peripheral){
		PWM_CTRL |= bPWM2_OUT_EN;											//PWM2输出使能	
		if(PWM_ACTIVE_ATT(id)){
			PWM_CTRL &= ~bPWM2_POLAR;										//高电平有效  
		}else{
			PWM_CTRL |= bPWM2_POLAR;										//低电平有效
		}	
	}

	return true;
}

bool hal_pwm_deinit(uint16_t id)
{
	if(PWM1 == m_pwm_map[id].peripheral){
		PWM_CTRL &= ~bPWM_OUT_EN;
	}else if(PWM2 == m_pwm_map[id].peripheral){
		PWM_CTRL &= ~bPWM2_OUT_EN;
	}
	return false;
}


#endif





