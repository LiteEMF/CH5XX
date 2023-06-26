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
#include  "api/api_flash.h"
#include  "api/api_system.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define DATAFLASH_ADD		0xF000
#define DATAFLASH_LEN		0X400

/*****************************************************************************************************
**  Function
******************************************************************************************************/


/*******************************************************************************
* Function Name  : EraseBlock(UINT16 Addr)
* Description    : CodeFlash块擦除(1KB)，全部数据位写1
* Input          : UINT16 Addr
* Output         : None
* Return         : None
*******************************************************************************/
UINT8	EraseBlock( UINT16 Addr )
{
	ROM_ADDR = Addr;
	if ( ROM_STATUS & bROM_ADDR_OK ) {                                          // 操作地址有效
		ROM_CTRL = ROM_CMD_ERASE;
		return( ( ROM_STATUS ^ bROM_ADDR_OK ) & 0x7F );                           // 返回状态,0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
	}
	else return( 0x40 );
}

/*******************************************************************************
* Function Name  : ProgWord( UINT16 Addr, UINT16 Data )
* Description    : 写EEPROM，双字节写
* Input          : UNIT16 Addr,写地址
                   UINT16 Data,数据
* Output         : None
* Return         : SUCESS 
*******************************************************************************/
UINT8	ProgWord( UINT16 Addr, UINT16 Data )
{
	ROM_ADDR = Addr;
	ROM_DATA = Data;
	if ( ROM_STATUS & bROM_ADDR_OK ) {                                           // 操作地址有效
		ROM_CTRL = ROM_CMD_PROG;
		return( ( ROM_STATUS ^ bROM_ADDR_OK ) & 0x7F );                            // 返回状态,0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
	}
	else return( 0x40 );
}

/*******************************************************************************
* Function Name  : EraseDataFlash(UINT16 Addr)
* Description    : DataFlash块擦除(1KB)，全部数据位写1
* Input          : UINT16 Addr
* Output         : None
* Return         : UINT8 status
*******************************************************************************/
UINT8 EraseDataFlash(UINT16 Addr)
{
    UINT8 status;

    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG |= bDATA_WE;                                                    //使能DataFlash写
    SAFE_MOD = 0;	
    status = EraseBlock(Addr);	
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG &= ~bDATA_WE;                                                   //开启DataFlash写保护
    SAFE_MOD = 0;	
    return status;
}

/*******************************************************************************
* Function Name  : WriteDataFlash(UINT16 Addr,PUINT8 buf,UINT16 len)
* Description    : DataFlash写
* Input          : UINT16 Addr，PUINT16 buf,UINT16 len
* Output         : None
* Return         : 
*******************************************************************************/
UINT8  WriteDataFlash( UINT16 Addr, PUINT8X Buf,UINT16 len )
{
    UINT16 j,tmp;                                           

    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG |= bDATA_WE;                                                    //使能DataFlash写
    SAFE_MOD = 0;
    for(j=0;j<len;j=j+2)
    {
        tmp = Buf[j+1];
        tmp <<= 8;
        tmp += Buf[j];			
        ProgWord(Addr,tmp);
        Addr = Addr + 2;
    }
    SAFE_MOD = 0x55;
    SAFE_MOD = 0xAA;                                                           //进入安全模式
    GLOBAL_CFG &= ~bDATA_WE;                                                   //开启DataFlash写保护
    SAFE_MOD = 0;
    return 0;
}

/*******************************************************************************
* Function Name  : FlashReadBuf(UINT16 Addr,PUINT8 buf,UINT16 len)
* Description    : 读Flash（包含data和code）
* Input          : UINT16 Addr,PUINT8 buf,UINT16 len
* Output         : None
* Return         : 返回实际读出长度
*******************************************************************************/
UINT16 FlashReadBuf(UINT16 Addr,PUINT8 buf,UINT16 len)
{
    memcpy(buf,(PUINT8C)Addr,len);
	return len;
}





/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
//hal
bool hal_flash_write(uint16_t offset,uint8_t *buf,uint16_t len)
{
    return !WriteDataFlash( DATAFLASH_ADD+offset, buf, len );
}
bool hal_flash_read(uint16_t offset,uint8_t *buf,uint16_t len)
{
	return FlashReadBuf(DATAFLASH_ADD+offset,buf,len);
}
bool hal_flash_erase(uint16_t offset)
{
    return !EraseDataFlash( DATAFLASH_ADD + offset);
}
bool hal_flash_init(void)
{
	return false;
}










