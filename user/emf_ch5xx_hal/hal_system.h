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


#ifndef _hal_system_h
#define _hal_system_h
#include "utils/emf_typedef.h" 

#define NO_XSFR_DEFINE
#include "CH559.H"

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************************************************************
** Defined
*******************************************************************************************************/
#ifndef HAL_SYS_FREQ			//系统时钟
#define HAL_SYS_FREQ			(48 * 1000000L)			
#endif

#ifndef API_ENTER_CRITICAL
#define API_ENTER_CRITICAL()		EA = 0
#endif
#ifndef API_EXIT_CRITICAL
#define API_EXIT_CRITICAL()			EA = 1
#endif


/******************************************************************************************************
**	Parameters
*******************************************************************************************************/



#ifdef __cplusplus
}
#endif
#endif





