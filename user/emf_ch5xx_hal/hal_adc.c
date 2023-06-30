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
#include "hw_board.h"
#ifdef HW_ADC_MAP
#include  "api/api_adc.h"
#include  "api/api_tick.h"
#include  "api/api_system.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
// #define HW_ADC_MAP {	\
// 			{PB_00,0,VAL2FLD(ADC_CH,0)},		\
// 			{PA_01,0,VAL2FLD(ADC_CH,1) | VAL2FLD(ADC_PULL,1)}			\
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
uint8_t adc_id_index = 0;
uint16x_t	ADCbuf[ 8 ];                                       //存储ADC采样数据


/*******************************************************************************
* Function Name  : InitADCInterrupt()
* Description    : ADC中断初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitADCInterrupt()
{
	ADC_SETUP |= bADC_IE_FIFO_OV;                                             //使能FIFO溢出中断
	//  ADC_SETUP |= bADC_IE_AIN7_LOW;                                            //使能AIN7低电平中断
	ADC_SETUP |= bADC_IE_ACT;                                                 //ADC完成中断
    IE_ADC = 1;                                                               //使能ADC中断
}




/*******************************************************************************
* Function Name  : ADCInterrupt(void)
* Description    : ADC 中断服务程序
*******************************************************************************/
void ADCInterrupt( void ) interrupt INT_NO_ADC using 1                       //ADC中断服务程序,使用寄存器组1
{ 
    UINT16 ADCValue = 0;
    if(ADC_STAT & bADC_IF_ACT) {                                                //ADC完成中断
    	ADC_STAT |= bADC_IF_ACT;                                                //清中断                                         
    }
    ADCbuf[adc_id_index] = ADC_FIFO;
	if(++adc_id_index >= m_adc_num) adc_id_index = 0;
	ADC_CHANN = ADC_CH_ATT(adc_id_index);                                                          //切换ADC通道
    hal_adc_start_scan();
}

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
uint16_t hal_adc_to_voltage(uint16_t adc)
{
	return adc * ADC_REF_MV / ADC_RES_MAX;
}
bool hal_adc_value(uint8_t id, uint16_t* valp)
{
	*valp = ADCbuf[id];
	return true;
}
bool hal_adc_start_scan(void)
{
	ADC_CTRL |= bADC_SAMPLE;                                                  //手动产生采样脉冲
    delay_us(2);
    ADC_CTRL &= ~bADC_SAMPLE;	
	return true;
}
bool hal_adc_init(void)
{
	uint8_t i;
	
	for(i=0; i<m_adc_num; i++){
		if(ADC_PULL_ATT(i)){
			hal_gpio_mode(m_adc_map[i].pin,8);
		}else{
			hal_gpio_mode(m_adc_map[i].pin,7);
		}
	}

	ADC_SETUP |= bADC_POWER_EN;          	//ADC电源使能
    ADC_CK_SE |= (MASK_ADC_CK_SE & 0x0C);	//设置12分频
    ADC_CTRL &= ~MASK_ADC_CYCLE;
    ADC_CTRL |= 0x0C;                    	//设置ADC自动采样周期                                   
    ADC_CTRL &= ~(bADC_CHANN_MOD0 | bADC_CHANN_MOD1);         	//手工选择通道模式
    ADC_EX_SW |= bADC_RESOLUTION;        	//采样位数11bit
//	ADC_EX_SW &= ~bADC_RESOLUTION;       	//采样位数10bit

	adc_id_index = 0;
	ADC_CHANN = ADC_CH_ATT(0);       
    delay_us(100);                       	//确保ADC正常启动	

	InitADCInterrupt();                                                        //ADC中断初始化
    EA = 1;      

	return true;
}
bool hal_adc_deinit(void)
{
	ADC_SETUP &= ~bADC_POWER_EN;
	return true;
}


#endif




