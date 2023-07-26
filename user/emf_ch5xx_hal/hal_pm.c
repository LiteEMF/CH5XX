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
#include  "api/api_pm.h"
#include  "api/api_system.h"

#include  "api/api_log.h"
/******************************************************************************************************
** Defined
*******************************************************************************************************/
static uint8_t reset_keep=0xff;
/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
void hal_weakup_init(void)
{
	
}

pm_reson_t hal_get_reset_reson(void)
{
	if(0xff == reset_keep){
		reset_keep = RESET_KEEP;
		logd("read RESET_KEEP=%x\n",(uint16_t)reset_keep);
		RESET_KEEP = 1;
	}
	return reset_keep? PM_RESON_VCM:PM_RESON_POR;
}
void hal_boot(uint8_t index)
{
}
void hal_reset(void)
{
}
void hal_sleep(void)
{
}










