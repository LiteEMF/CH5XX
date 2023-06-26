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
#include  "api/api_gpio.h"
#include  "api/api_system.h"

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


/*******************************************************************************
* Function Name  : CH559GPIODrivCap(UINT8 Port,UINT8 Cap)
* Description    : 端口0、1、2、3驱动能力设置
* Input          : UINT8 Port端口选择(0、1、2、3)
                   UINT8 Cap驱动能力选择((0)5mA、(1)20mA(注意:P1口是10mA))
* Output         : None
* Return         : SUCCESS成功
                   FAIL失败
*******************************************************************************/
bool CH559GPIODrivCap(UINT8 Port,UINT8 Cap)
{
	if(Port >= 4){
		return false;
	}

	if(Cap == 0){                                                               //驱动电流最大5mA
    	PORT_CFG &= ~(bP0_DRV << Port);
 	}else{		
    	PORT_CFG |= (bP0_DRV << Port);                                             //驱动电流最大20mA
  	}
  	return true;
}

/*******************************************************************************
* Function Name  : CH559GPIOModeSelt(UINT8 Port,UINT8 Mode,UINT8 PinNum)
* Description    : 端口0、1、2、3引脚模式设置
* Input          : UINT8 Port端口选择(0、1、2、3)
                   UINT8 Cap驱动方式选择(bPn_OC & Pn_DIR & Pn_PU)
                   0(000)：仅输入，无上拉；
                   1(001)：仅输入，带上拉；
                   2(01x)：推挽输出，高低电平强驱动；
                   3(100)：开漏输出，无上拉，支持输入；
                   4(110)：开漏输出，无上拉,当转变输出由低到高时，仅驱动2个时钟的高电平
                   5(101)：准双向(标准51模式)，开漏输出，带上拉
                   6(110)：准双向(标准51模式)，开漏输出，带上拉，当转变输出由低到高时，仅驱动2个时钟的高电平
                   7(111): 模拟输入
				   7(111): 模拟输入,带上拉
				   UINT8 PinNum(引脚选择0-7)
* Output         : None
* Return         : SUCCESS成功
                   FAIL失败
*******************************************************************************/
bool CH559GPIOModeSelt(UINT8 Port,UINT8 Mode,UINT8 PinNum)
{
  	UINT8 Pn_DIR_set=0,Pn_PU_set=0;
	UINT8 Pn_DIR_c=0xff,Pn_PU_c=0xff;
	if(Port >= 4){
		return false;
	}

  switch (Mode){
	case 0:                                                                //仅输入，无上拉
	case 7:
		PORT_CFG &= ~(bP0_OC << Port);
		Pn_DIR_c &= ~(1<<PinNum);
		Pn_PU_c &= ~(1<<PinNum);
		break;
	case 1:   
	case 8:                                                             //仅输入，带上拉
		PORT_CFG &= ~(bP0_OC << Port);
		Pn_DIR_c &= ~(1<<PinNum);
		Pn_PU_set |= 1<<PinNum;
		break;
	case 2:                                                                //推挽输出，高低电平强驱动
		PORT_CFG &= ~(bP0_OC << Port);
		Pn_DIR_set |= (1<<PinNum);
		break;
	case 3:                                                                //开漏输出，无上拉，支持输入
		PORT_CFG |= (bP0_OC << Port);
		Pn_DIR_c &= ~(1<<PinNum);
		Pn_PU_c &= ~(1<<PinNum);
		break;
	case 4:                                                                //开漏输出，无上拉,当转变输出由低到高时，仅驱动2个时钟的高电平
		PORT_CFG |= (bP0_OC << Port);
		Pn_DIR_set |= 1<<PinNum;
		Pn_PU_c &= ~(1<<PinNum);
		break;
	case 5:                                                                //弱准双向(标准51模式)，开漏输出，带上拉
		PORT_CFG |= (bP0_OC << Port);
		Pn_DIR_c &= ~(1<<PinNum);
		Pn_PU_set |= 1<<PinNum;
		break;
	case 6:                                                                //准双向(标准51模式)，开漏输出，带上拉，当转变输出由低到高时，仅驱动2个时钟的高电平
		PORT_CFG |= (bP0_OC << Port);
		Pn_DIR_set |= 1<<PinNum;
		Pn_PU_set |= 1<<PinNum;
		break;
	default:
		break;
	}

	switch(Port){
	case 0:
		P0_DIR = P0_DIR & Pn_DIR_c | Pn_DIR_set;
		P0_PU = P0_PU & Pn_PU_c | Pn_PU_set;
		break;
	case 1:
		P1_DIR = P1_DIR & Pn_DIR_c | Pn_DIR_set;
		P1_PU = P1_PU & Pn_PU_c | Pn_PU_set;
		if(7 == Mode){			//P1口选择模拟输入
			P1_IE &= P1_IE & Pn_DIR_c | Pn_DIR_set;
		}
		break;
	case 2:
		P2_DIR = P2_DIR & Pn_DIR_c | Pn_DIR_set;
		P2_PU = P2_PU & Pn_PU_c | Pn_PU_set;
		break;
	case 3:
		P3_DIR = P3_DIR & Pn_DIR_c | Pn_DIR_set;
		P3_PU = P3_PU & Pn_PU_c | Pn_PU_set;
		break;
	case 4:
		P4_DIR = P4_DIR & Pn_DIR_c | Pn_DIR_set;
		P4_PU = P4_PU & Pn_PU_c | Pn_PU_set;
		break;
	}
  return true;
}

/*******************************************************************************
* Function Name  : CH559P4Mode()
* Description    : CH559的P4端口初始化，P4默认是输入口
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH559P4Mode( )
{
	P4_DIR |= 0xff;                                                            //置1设置为输出
	P4_PU |= 0xff;                                                             //启动p4口内部上拉
	P4_CFG |= bP4_DRV;                                                         //该位为0则P4口驱动能力5mA,为1时为20mA
}

/*******************************************************************************
* Function Name  : CH559GPIOInterruptInit()
* Description    : CH559GPIO中断初始化，其他引脚如P5.5\P1.4\P0.3\P5.7\P4.1\RXD0设置同理
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH559GPIOInterruptInit()
{                                                      
	GPIO_IE &= ~bIE_IO_EDGE;                                                   //中断方式选择，该位为0则表示IO口电平中断，该位为1则表示IO口边沿中断
	GPIO_IE |= bIE_RXD1_LO;                                                    //使能RXD1引脚的中断,其他引脚中断设置同理
}


/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
void hal_gpio_mode(pin_t pin, uint8_t mode)
{
	CH559GPIOModeSelt(pin>>4, mode, pin&0x0f);
}

void hal_gpio_dir(pin_t pin, pin_dir_t dir, pin_pull_t pull)
{
	if(PIN_OUT == dir){
		if(PIN_PULL_OD == pull){
			CH559GPIOModeSelt(pin>>4, 5, pin&0x0f);		//准双向IO 带上拉
		}else{
			CH559GPIOModeSelt(pin>>4, 2, pin&0x0f);
		}
	}else{
		switch(pull){
		case PIN_PULLNONE:
		case PIN_PULLDOWN:				//不支持下拉
			CH559GPIOModeSelt(pin>>4, 0, pin&0x0f);
			break;
		case PIN_PULLUP:
			CH559GPIOModeSelt(pin>>4, 1, pin&0x0f);
			break;	
		}
	}	
}

uint32_t hal_gpio_in(pin_t pin)
{
    uint32_t value = 0;
	uint8_t io_bit = BIT(pin&0X0F);

	switch(pin>>4){
        case 0:
            value = P0 & io_bit;
            break;
        case 1:
            value = P1 & io_bit;
            break;
        case 2:
            value = P2 & io_bit;
            break;
        case 3:
            value = P3 & io_bit;
            break;
        default :
			break;
	}

	if(value) value = 1;

	return value;
}

void hal_gpio_out(pin_t pin, uint8_t value)
{
	uint8_t io = pin&0X0F;
	uint8_t io_bit = BIT(pin&0X0F);

	switch(pin>>4){
        case 0:
			P0 = ( P0 & io_bit ) | (value << io);
            break;
        case 1:
            P1 = ( P1 & io_bit ) | (value << io);
            break;
        case 2:
            P2 = ( P2 & io_bit ) | (value << io);
            break;
        case 3:
            P3 = ( P3 & io_bit ) | (value << io);
            break;
        default :
			break;
	}
}








