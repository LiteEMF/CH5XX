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
#ifdef HW_UART_MAP

#include  "api/api_system.h"
#include  "api/api_uart.h"
#include  "api/api_gpio.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
// #define HW_UART_MAP {\
// 			{PD_00, PIN_NULL,	0, 0, UART0, VAL2FLD(UART_BAUD,1000000)},	\
// 			{PC_07, PC_06, 		0, 0, UART0, VAL2FLD(UART_BAUD,1000000)|VAL2FLD(UART_PRI,1)},		\
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

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************************
* Function Name  : CH559UART0Alter()
* Description    : CH559串口0引脚映射,串口映射到RX:P0.2和TX:P0.3
					默认RX:P3.0, TX:P3.1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH559UART0Alter()
{
    PORT_CFG |= bP0_OC;
    P0_DIR |= bTXD_;
    P0_PU |= bTXD_ | bRXD_;
    PIN_FUNC |= bUART0_PIN_X;                                                  //串口映射到P0.2和P0.3
}

/*******************************************************************************
* Function Name  : mInitSTDIO()
* Description    : CH559串口0初始化,默认使用T1作UART0的波特率发生器,也可以使用T2
                   作为波特率发生器
注意: 如果映射成P0口, p0port将设置为开漏输出
*******************************************************************************/
void CH559UART0Init( UINT8 id ,UINT32 buad)
{
    UINT32 x;
    UINT8 x2; 

	if(PA_03 == m_uart_map[id].tx){
		CH559UART0Alter();
	}
	
    SM0 = 0;
    SM1 = 1;
    SM2 = 0;                                                                   //串口0使用模式1
                                                                               //使用Timer1作为波特率发生器
    RCLK = 0;                                                                  //UART0接收时钟
    TCLK = 0;                                                                  //UART0发送时钟
    PCON |= SMOD;
    x = 10 * HAL_SYS_FREQ / buad / 16;                                             //如果更改主频，注意x的值不要溢出                            
    x2 = x % 10;
    x /= 10;
    if ( x2 >= 5 ) x ++;                                                       //四舍五入

    TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1;              //0X20，Timer1作为8位自动重载定时器
    T2MOD = T2MOD | bTMR_CLK | bT1_CLK;                                        //Timer1时钟选择
    TH1 = 0-x;                                                                 //12MHz晶振,buad/12为实际需设置波特率
    TR1 = 1;                                                                   //启动定时器1
    TI = 1;
	if(((pin_t)PIN_NULL != m_uart_map[id].rx)){
    	REN = 1;                                                                   //串口0接收使能
	}
}

/*******************************************************************************
* Function Name  : CH559UART1Init(UINT8 DIV,UINT8 mode,UINT8 pin)
* Description    : CH559 UART1初始化设置
* Input          :
                   UINT8 DIV设置分频系数，时钟频率=Fsys/DIV,DIV不能为0
                   UINT8 mode，模式选择，1：普通串口模式；0:485模式
                   UINT8 pin，串口引脚选择；
                   当mode=1时
                   0：RXD1=P4.0,TXD1关闭；
                   1：RXD1&TXD1=P4.0&P4.4；
                   2：RXD1&TXD1=P2.6&P2.7；
                   3：RXD1&TXD1&TNOW=P2.6&P2.7&P2.5；
                   当mode=0时
                   0：无意义
                   1：P5.4&P5.5连接485,TNOW=P4.4；
                   2：P5.4&P5.5连接485；
                   3：P5.4&P5.5连接485,TNOW=P2.5；
* Output         : None
* Return         : None
SER1_DLL_SER1_DLM除数=Fsys/8/SER1_DIV/波特率
*******************************************************************************/
void CH559UART1Init(UINT8 id, UINT32 buad)
{
    UINT32 x;
    UINT8 x2;
	UINT8 mode = 1,pin = 2;

	if((PE_04 == m_uart_map[id].tx) || (PE_00 == m_uart_map[id].rx)){
		pin = 1;
	}else{		//rx:PC_06, tx:PC_07
		pin = 2;
	}

    SER1_LCR |= bLCR_DLAB; // DLAB位置1，写DLL、DLM和DIV寄存器
    SER1_DIV = 1; // 预分频 FSYS/8	6M
    x = 10 * HAL_SYS_FREQ / 8 / buad;
    x2 = x % 10;
    x /= 10;
    if (x2 >= 5)
        x++; // 四舍五入
    SER1_DLM = x >> 8;
    SER1_DLL = x & 0xff;
    SER1_LCR &= ~bLCR_DLAB; // DLAB位置0,防止修改UART1波特率和时钟
    if (mode == 1){ 		// 关闭RS485模式 RS485_EN = 0,不能省略
        XBUS_AUX |= bALE_CLK_EN;
    } else if (mode == 0){ // 开启RS485模式 RS485_EN = 1;
        UHUB1_CTRL |= bUH1_DISABLE;
        PIN_FUNC &= ~bXBUS_CS_OE;
        PIN_FUNC |= bXBUS_AL_OE;
        XBUS_AUX &= ~bALE_CLK_EN;
        SER1_MCR |= bMCR_HALF; // 485模式只能使用半双工模式
    }
    SER1_LCR |= MASK_U1_WORD_SZ; // 线路控制
    SER1_LCR &= ~(bLCR_PAR_EN | bLCR_STOP_BIT); // 无线路间隔，无校验，1位停止位，8位数据位

    SER1_MCR &= ~bMCR_TNOW;
    SER1_IER |= bIER_EN_MODEM_O;
    SER1_IER |= ((pin << 4) & MASK_U1_PIN_MOD); // 串口模式配置

    if(UART_PRI_ATT(id)){
        SER1_IER |= /*bIER_MODEM_CHG | */ bIER_LINE_STAT | /*bIER_THR_EMPTY |*/ bIER_RECV_RDY; // 中断使能配置
    }

    SER1_FCR |= MASK_U1_FIFO_TRIG | bFCR_T_FIFO_CLR | bFCR_R_FIFO_CLR | bFCR_FIFO_EN; // FIFO控制器 7bit fifo
    SER1_MCR |= bMCR_OUT2; // MODEM控制寄存器
    x2 = SER1_IIR; // 读IIR清中断
                   // 中断请求输出，不产生实际中断
    SER1_ADDR |= 0xff; // 关闭多机通信
}



/*******************************************************************************
* Function Name  : CH559UART0RcvByte()
* Description    : CH559UART0接收一个字节
* Input          : None
* Output         : None
* Return         : SBUF
*******************************************************************************/
UINT8  CH559UART0RcvByte( )
{
    while(RI == 0);                                                            //查询接收，中断方式可不用
    RI = 0;
    return SBUF;
}

/*******************************************************************************
* Function Name  : CH559UART0SendByte(UINT8 SendDat)
* Description    : CH559UART0发送一个字节
* Input          : UINT8 SendDat；要发送的数据
* Output         : None
* Return         : None
*******************************************************************************/
void CH559UART0SendByte(UINT8 SendDat)
{
	SBUF = SendDat;                                                              //查询发送，中断方式可不用下面2条语句,但发送前需TI=0
	while(TI ==0);
	TI = 0;
}
void UART0Send(PUINT8 Sendbuf, UINT8 len)
{
    while (len--) {
        SBUF = (*Sendbuf++);
        while(TI ==0);
		TI = 0;
    }
}




/*******************************************************************************
 * Function Name  : CH559UART1RcvByte()
 * Description    : CH559UART1接收一个字节
 * Input          : None
 * Output         : None
 * Return         : 正确：UINT8 Rcvdat;接收数据
 *******************************************************************************/
UINT8 CH559UART1RcvByte()
{
    while ((SER1_LSR & bLSR_DATA_RDY) == 0); // 等待数据准备好
    return SER1_RBR;
}

/*******************************************************************************
 * Function Name  : CH559UART1SendByte(UINT8 SendDat)
 * Description    : CH559UART1发送一个字节
 * Input          : UINT8 SendDat；要发送的数据
 * Output         : None
 * Return         : None
 *******************************************************************************/
void CH559UART1SendByte(UINT8 SendDat)
{
    while ((SER1_LSR & bLSR_T_ALL_EMP) == 0); // 没开FIFO，等待1字节发送完成
    SER1_THR = SendDat;
}

void UART1Send(PUINT8 Sendbuf, UINT8 len)
{
    while (len--) {
        SER1_THR = (*Sendbuf++);
        while ((SER1_LSR & bLSR_T_FIFO_EMP) == 0);
    }
}


uint8_t get_uart_id (uint8_t uart)
{
    uint8_t i;

    for(i=0; i<m_uart_num; i++){
        if(m_uart_map[i].peripheral == (uint32_t)uart){
            return i;
        }
    }

    return ID_NULL;
}

/*******************************************************************************
 * Function Name  : UART1Interrupt(void)
 * Description    : UART1中断服务程序
 *******************************************************************************/
void UART1Interrupt(void) interrupt INT_NO_UART1 using 1 // UART1中断服务程序,使用寄存器组1
{
	uint8_t id; 
    UINT8D InterruptStatus, i;
	UINT8D buf[8];

    InterruptStatus = SER1_IIR & 0x0f; // 获取中断状态
    switch (InterruptStatus) {
    case U1_INT_RECV_RDY: // 接收数据可用中断，可以先读取指定字节数触发中断的数据个数
    case U1_INT_RECV_TOUT: // 接收超时中断
		i = 0;
		while((SER1_LSR & bLSR_DATA_RDY) == 0);                                   //等待数据准备好
		while(SER1_LSR & bLSR_DATA_RDY){
			buf[i++] = SER1_RBR; 
		}          
		id = get_uart_id (UART1);
		api_uart_rx_hook(id, (uint8_t*)buf, i);
        break;
    case U1_INT_LINE_STAT: // 线路状态中断
        i = SER1_LSR;
        break;
    case U1_INT_SLV_ADDR: // 设备地址match中断
        i = SER1_IIR;
        break;
    case U1_INT_NO_INTER: // 无中断
        break;
    case U1_INT_MODEM_CHG: // MODEM中断
        i = SER1_MSR;
        break;
    case U1_INT_THR_EMPTY: // 发送空中断，可以启动下次发送或者等待接收
        i = SER1_IIR;
        break;
    default:
        break;
    }
}

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
bool hal_uart_set_baud(uint8_t id, uint32_t baud)
{
	return true;
}
bool hal_uart_tx(uint8_t id,void * buf,uint16_t len)
{
	if(UART0 == m_uart_map[id].peripheral){
		UART0Send(buf,len);
		return true;
	}else if(UART1 == m_uart_map[id].peripheral){
		UART0Send(buf,len);
		return true;
	}

	return false;
}
bool hal_uart_init(uint8_t id,uint32_t baudrate)
{
	if(UART0 == m_uart_map[id].peripheral){
		CH559UART0Init(id,baudrate);
		return true;
	}else if(UART1 == m_uart_map[id].peripheral){
		CH559UART1Init(id,baudrate);
		return true;
	}

	return false;
}
bool hal_uart_deinit(uint8_t id)
{
	return false;
}


#endif





