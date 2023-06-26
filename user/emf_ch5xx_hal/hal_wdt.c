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
#if API_WDT_ENABLE

#include  "api/api_system.h"
#include  "api/api_wdt.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************************
* Function Name  : WDOGInit(UINT8 n,UINT8 value)
* Description    : 看门狗中断初始化
* Input          : UINT8 n,计时时长
                   UINT8 value,选择看门狗计时完成后操作
                   value=1芯片复位;
                   value=0产生看门狗中断;
系统主频 Fsys/262144，当计满 0FFh 转向 00h 时产生溢出信号, (5.4ms)
*******************************************************************************/
void WDOGInit(UINT8 n,UINT8 value)
{	
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                          //开启安全模式
    GLOBAL_CFG |= value;    
    SAFE_MOD = 0xFF;                                                          //关闭安全模式
    
    WDOG_COUNT = n;                                                           //看门狗超时时间
    if(!value)
    {
        IE_WDOG = 1;                                                          //使能看门狗中断
        EA = 1;                                                               //开总中断
    }
}

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
//hal

void hal_wdt_feed(void)
{
	WDOG_COUNT = 0;
}

bool hal_wdt_init(uint32_t ms)
{
	WDOGInit(0,1);
	return true;
}
bool hal_wdt_deinit(void)
{
	return false;
}


#endif







