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
#include  "api/api_system.h"
#define NO_XSFR_DEFINE
#include "CH559.H"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define	 FREQ_SYS	HAL_SYS_FREQ		//系统主频


/*****************************************************************************************************
**  Function
******************************************************************************************************/
#define  OSC_EN_XT 	0

/*******************************************************************************
* Function Name  : CfgFsys( )
* Description    : CH559时钟选择和配置函数,默认使用内部晶振12MHz，如果定义了FREQ_SYS可以
                   根据PLL_CFG和CLOCK_CFG配置得到，公式如下：
                   Fsys = (Fosc * ( PLL_CFG & MASK_PLL_MULT ))/(CLOCK_CFG & MASK_SYS_CK_DIV);
                   具体时钟需要自己配置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
void CfgFsys( )  
{
#if OSC_EN_XT
	SAFE_MOD = 0x55;                                                           //开启安全模式
    SAFE_MOD = 0xAA;                                                 
	CLOCK_CFG |= bOSC_EN_XT;                                                   //使能外部晶振
    mDelaymS(10);
	SAFE_MOD = 0x55;                                                           //开启安全模式
    SAFE_MOD = 0xAA;   
	CLOCK_CFG &= ~bOSC_EN_INT;
	SAFE_MOD = 0x00; 
#endif 

	SAFE_MOD = 0x55;                                                           //开启安全模式
    SAFE_MOD = 0xAA;     
	CLOCK_CFG &= ~MASK_SYS_CK_DIV;
#if	FREQ_SYS == 48000000
	CLOCK_CFG |= 6;                                                            //配置系统时钟48MHz
#endif
#if	FREQ_SYS == 36000000
	CLOCK_CFG |= 8;                                                            //配置系统时钟36MHz
#endif
#if	FREQ_SYS == 24000000
	CLOCK_CFG |= 12;                                                           //配置系统时钟24MHz
#endif
#if	FREQ_SYS == 18000000
	CLOCK_CFG |= 16;                                                           //配置系统时钟18MHz  
#endif
#if	FREQ_SYS == 56000000
    CLOCK_CFG |= 6;                                                            //配置系统时钟56MHz    
    PLL_CFG = 0xFC;
#endif
    SAFE_MOD = 0xFF;                                                           //关闭安全模式  
}

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_set_sysclk(emf_clk_t clk, uint32_t freq)
{
	CfgFsys();
	return true;
}
/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
uint32_t hal_get_sysclk(emf_clk_t clk)
{
	return HAL_SYS_FREQ;
}

bool hal_get_uuid(uint8_t *uuid, uint8_t len)
{
	return false;
}






