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


#if	defined HW_TIMER_MAP && API_TIMER_BIT_ENABLE
#include  "api/api_timer.h"
#include  "api/api_system.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
// #define HW_TIMER_MAP \
//     {TIMER0, VAL2FLD(TIMER_FREQ,1000)|VAL2FLD(TIMER_PRI,1)}
/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

//CH559 Timer0时钟选择   
//bTMR_CLK同时影响Timer0&1&2,使用时要注意                                                       
#define mTimer0ClkFsys( ) (T2MOD |= bTMR_CLK | bT0_CLK)                     //定时器,时钟=Fsys
#define mTimer0Clk4DivFsys( ) (T2MOD &= ~bTMR_CLK; T2MOD |= bT0_CLK)          //定时器,时钟=Fsys/4
#define mTimer0Clk12DivFsys( ) (T2MOD &= ~(bTMR_CLK | bT0_CLK))             //定时器,时钟=Fsys/12
#define mTimer0CountClk( ) (TMOD |= bT0_CT)                                 //计数器,T0引脚的下降沿有效

//CH559 Timer0 开始(SS=1)/结束(SS=0)
#define mTimer0RunCTL( SS ) (TR0 = SS ? 1 : 0)
#define TIMER0_T(freq)		(0X10000 - (HAL_SYS_FREQ/12)/freq)
	

uint8_t get_timer_id (uint8_t timer)
{
    uint8_t i;

    for(i=0; i<m_timer_num; i++){
        if(m_timer_map[i].peripheral == (uint32_t)timer){
            return i;
        }
    }
    return ID_NULL;
}

/*******************************************************************************
* Function Name  : mTimer0ModSetup(UINT8 mode)
* Description    : CH559定时计数器0模式0设置
* Input          : UINT8 mode,Timer0模式选择
                   0：模式0，13位定时器，TL0的高3位无效
                   1：模式1，16位定时器
                   2：模式2，8位自动重装定时器
                   3：模式3，两个8位定时器
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer0ModSetup(UINT8 mode)
{
    TMOD &= 0xf0;
    TMOD |= mode; 
}


void mTimer0Init(UINT16 freq)
{
	mTimer0ModSetup(1);				    //方式1
    mTimer0Clk12DivFsys( );				//时钟选择Fsys定时器方式

    TL0 = TIMER0_T(freq) & 0xff;
    TH0 = (TIMER0_T(freq) >> 8) & 0xff;

    mTimer0RunCTL( 1 );				//启动定时器
    ET0 = 1;						//使能定时计数器0中断
    EA = 1;							//使能全局中断
}



/*******************************************************************************
* Function Name  : mTimer0Interrupt()
* Description    : CH559定时计数器0定时计数器中断处理函数
*******************************************************************************/
#if API_TIMER_BIT_ENABLE & BIT(0)
void	mTimer0Interrupt( void ) interrupt INT_NO_TMR0 using 1                //timer0中断服务程序,使用寄存器组1
{    
    uint16_t reload;        
	uint8_t id = get_timer_id(TIMER0);   
    reload = TIMER0_T(TIMER_FREQ_ATT(id));                                                           //方式3时，TH0使用Timer1的中断资源
	TL0 = reload & 0xff;
    TH0 = (reload >> 8 ) & 0xff;//非自动重载方式需重新给TH0和TL0赋值  
    api_timer_hook(id);  
}
#endif


/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_timer_init(uint8_t id)
{
	if(TIMER0 == m_timer_map->peripheral){
		mTimer0Init(TIMER_FREQ_ATT(id));
		return true;
	}
	return false;
}
bool hal_timer_deinit(uint8_t id)
{
	if(TIMER0 == m_timer_map->peripheral){
		mTimer0RunCTL( 0 );				//启动定时器
    	ET0 = 0;						//使能定时计数器0中断
		return true;
	}
	return false;
}

#endif






