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
#ifdef HW_SPI_HOST_MAP

#include  "api/api_spi_host.h"
#include  "api/api_gpio.h"
#include  "api/api_system.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
// #define HW_SPI_HOST_MAP {\
// 			{PB_07,PB_05,PB_06,PB_04,SPI0,VAL2FLD(SPI_BADU,1000000)},	\
// 			{PC_03,PC_01,PC_02,PC_01,SPI0,VAL2FLD(SPI_BADU,1000000)},	\
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
#define SENDBYTE_SPI( d )    {  SPI0_DATA = d;while(S0_FREE == 0); }
#define RECVBYTE_SPI( d )    { SPI0_DATA = 0xff;while(S0_FREE == 0);d = SPI0_DATA;}
#define SENDBYTE_SPI1( d )    { SPI1_DATA = d;while((SPI1_STAT & 0x08) == 0);}
#define RECVBYTE_SPI1( d )    { SPI1_DATA = 0xff; while((SPI1_STAT & 0x08) == 0);d = SPI1_DATA;}

/*******************************************************************************
* Function Name  : InitHostSPI0( void )
* Description    : SPI0主机模式初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitHostSPI0( UINT8 id )
{
    SPI0_SETUP &=~(bS0_MODE_SLV | bS0_BIT_ORDER);                              /*设置成主机模式*/
    SPI0_CTRL = bS0_SCK_OE | bS0_MOSI_OE;                                      /*主机写，默认不启动写传输，如果使能bS0_DATA_DIR*/
                                                                               /*那么发送数据后自动产生一个字节的时钟，用于快速数据收发*/
	P1_DIR |= (bMOSI | bSCK | bSCS| bPWM3 );                                   /*bMOSI 、bSCK 、bSCS置为输出方向*/
    P1_DIR &= ~bMISO;

    SPI0_CK_SE = HAL_SYS_FREQ / SPI_BADU_ATT(id);
	//  SPI0_STAT = 0xFF;                                                          /*清中断标志*/
	//  IE_SPI0 = 1;
}


/*******************************************************************************
* Function Name  : CH559SPI1Init()
* Description    : CH559SPI1初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void InitHostSPI1( UINT8 id )
{
	P1_DIR |= bSCS;
	P2_DIR |= bMOSI1 | bSCK1;                                                  //设置SPI接口的引脚方向
	SPI1_CTRL |= bS1_SCK_OE | bS1_AUTO_IF ;                                    //MISO输出使能，SCK输出使能
	SPI1_CTRL &= ~(bS1_DATA_DIR | bS1_2_WIRE);                                 //使用3线SPI，读数据不启动传输
	                                                                           //如果使能bS1_DATA_DIR，那么发送数据后自动产生一个字节的时钟，用于快速数据收发
	SPI1_CK_SE = HAL_SYS_FREQ / SPI_BADU_ATT(id);                                                         //设置SPI工作时钟，可以自己配置
	SPI1_CTRL &= ~bS1_CLR_ALL;                                                 //清空SPI1的FIFO,默认是1，必须置零才能发送数据
}


/***************************************************************************** **
* Function Name  : CH559SPI1Write(UINT8 dat)
* Description    : SPI1输出数据
* Input          : UINT8 dat 数据
* Output         : None
* Return         : None
*******************************************************************************/
void CH559SPIWrite(UINT8 SPI, UINT8 dat)
{
	if(SPI0 == SPI){
		RECVBYTE_SPI(dat);
	}else{
		SENDBYTE_SPI1(dat);
	}														
}

/*******************************************************************************
* Function Name  : CH559SPI1Read()
* Description    : SPI1读数据
* Input          : None
* Output         : None
* Return         : UINT8 ret   
*******************************************************************************/
UINT8 CH559SPIRead( UINT8 SPI )
{
	UINT8 d;
	if(SPI0 == SPI){
		SENDBYTE_SPI(d);
	}else{
		SENDBYTE_SPI1(d);
	}
	return d;
}

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_spi_host_write(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
	uint16_t i;
	int8_t spi = m_spi_map[id].peripheral;

	if(0XFF00 & addr){
		CH559SPIWrite(id, addr>>8 );
	}
	CH559SPIWrite(spi, addr );
	for(i=0; i<len; i++){
		CH559SPIWrite(spi, buf[i] );
	}
	
	return true;
}
bool hal_spi_host_read(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
	uint16_t i;
	int8_t spi = m_spi_map[id].peripheral;

	if(0XFF00 & addr){
		CH559SPIWrite(spi, addr>>8 );
	}
	CH559SPIWrite(spi, addr );

	for(i=0; i<len; i++){
		buf[i] = CH559SPIRead(spi);
	}
	return true;
}
bool hal_spi_host_isr_write(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
	return hal_spi_host_write(id,addr,buf,len);
}
bool hal_spi_host_isr_read(uint8_t id,uint16_t addr, uint8_t * buf, uint16_t len)
{
	return hal_spi_host_read(id,addr,buf,len);
}
bool hal_spi_host_init(uint8_t id)
{
	if(SPI0 == m_spi_map[id].peripheral){
		InitHostSPI0( id );
	}else{
		InitHostSPI1( id );
	}
	return true;
}
bool hal_spi_host_deinit(uint8_t id)
{
	return true;
}

#endif




