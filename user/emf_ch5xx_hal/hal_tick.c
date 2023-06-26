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
#include  "api/api_tick.h"
#include  "api/api_system.h"
#include <intrins.h>       // 声明了void _nop_(void);

/******************************************************************************************************
** Defined
*******************************************************************************************************/

/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/

/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:	//TODO 时间不准确没有调试	
*******************************************************************/
void hal_delay_ns(uint32_t ns)
{
	if(ns < 500){
	}else{
		ns /= 500;
		while ( ns-- ) {  // total = 12~13 Fsys cycles, 1uS @Fsys=12MHz 250ns@48M
			_nop_();
		}
	}
}
void hal_delay_us(uint32_t us)
{
	while ( us ) {  // total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
		++ SAFE_MOD;  // 2 Fsys cycles, for higher Fsys, add operation here

		#if	FREQ_SYS >= 14000000
		++ SAFE_MOD;
		#endif
		#if FREQ_SYS >= 16000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 18000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 20000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 22000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 24000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 26000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 28000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 30000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 32000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 34000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 36000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 38000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 40000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 42000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 44000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 46000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 48000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 50000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 52000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 54000000
		++ SAFE_MOD;
		#endif
		#if	FREQ_SYS >= 56000000
		++ SAFE_MOD;
		#endif
		-- us;
	}
}
void hal_delay_ms(uint32_t ms)
{
	hal_delay_us(1000*ms);
}
void hal_tick_init(void)
{	

}







