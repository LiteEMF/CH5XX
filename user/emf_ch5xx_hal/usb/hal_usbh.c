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
#if API_USBH_BIT_ENABLE
#include  "api/usb/usb_typedef.h"
#include  "api/usb/host/usbh.h"
#include  "api/api_tick.h"
#include  "api/api_system.h"

#include "api/api_log.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/

    
/* USB Communication Status Code */
#define ERR_SUCCESS                 0x00
#define ERR_USB_CONNECT             0x15
#define ERR_USB_DISCON              0x16
#define ERR_USB_BUF_OVER            0x17
#define ERR_USB_DISK_ERR            0x1F
#define ERR_USB_TRANSFER            0x20
#define ERR_USB_UNSUPPORT           0xFB
#define ERR_USB_UNAVAILABLE         0xFC
#define ERR_USB_UNKNOWN             0xFE
                
/* USB Communication Time */
#define DEF_BUS_RESET_TIME          11          // USB bus reset time
#define DEF_RE_ATTACH_TIMEOUT       100         // Wait for the USB device to reconnect after reset, 100mS timeout
#define DEF_WAIT_USB_TOUT_200US     200
#define DEF_CTRL_TRANS_TIMEOVER_CNT 60000       // Control transmission delay timing


/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/
/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/
#if !defined(__C51__)
__ALIGN(4) uint8x_t  RxBuffer[ 0x40];  // IN, must even address
__ALIGN(4) uint8x_t  TxBuffer[ 0x40 ];  // OUT, must even address
#else
uint8x_t  RxBuffer[ 0x40] _at_ 0x0000;  // IN, must even address, 错开usbd buf
uint8x_t  TxBuffer[ 0x40 ] _at_ (0x0040);  // OUT, must even address
#endif


/*****************************************************************************************************
**	static Function
******************************************************************************************************/

/*******************************************************************************
 * Function Name  : RootHUB_Detect_USB_Plug
 * Description    : 检测ROOTHUB端口设备插拔
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void RootHUB_Detect_USB_Plug(void)
{
    UINT8 s;
    if (UIF_DETECT) // 如果有USB主机检测中断则处理
    {
        UIF_DETECT = 0; // 清中断标志

    	if ( USB_HUB_ST & bUHS_H0_ATTACH ){                       // 设备存在
			usbh_det_event(0, 1);
		}else{
			usbh_det_event(0, 0);
		}

		if ( USB_HUB_ST & bUHS_H1_ATTACH ){                       // 设备存在
			usbh_det_event(1, 1);
		}else{
			usbh_det_event(1, 0);
		}
    }
}




/*******************************************************************************
* Function Name  : SetHostUsbAddr
* Description    : 设置USB主机当前操作的USB设备地址
* Input          : UINT8 addr
* Output         : None
* Return         : None
*******************************************************************************/
void    SetHostUsbAddr( UINT8 addr )
{
    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | addr & 0x7F;
}



/*******************************************************************************
* Function Name  : SetUsbSpeed
* Description    : 设置当前USB速度
* Input          : UINT8 FullSpeed
* Output         : None
* Return         : None
*******************************************************************************/
void    SetUsbSpeed( usb_speed_t speed )
{
    if (USB_SPEED_FULL == speed) { // full speed
		USB_CTRL &= ~bUC_LOW_SPEED; // 全速
		UH_SETUP &= ~bUH_PRE_PID_EN; // 禁止PRE PID
    } else {
    	USB_CTRL |= bUC_LOW_SPEED; // 低速
    }
}

/*******************************************************************************
* Function Name  : ResetRootHubPort( UINT8 RootHubIndex )
* Description    : 检测到设备后,复位总线,为枚举设备准备,设置为默认为全速
* Input          : UINT8 RootHubIndex 指定端口
* Output         : None
* Return         : None
*******************************************************************************/
UINT8 ResetRootHubPort( UINT8 id )
{
    UINT16 n = 30;
    SetHostUsbAddr( 0x00 );	
    SetUsbSpeed( 1 );                                            // 默认为全速
    if ( (id>>4) == 1 )
    {
        UHUB1_CTRL = UHUB1_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// 默认为全速,开始复位
		while( n )
		{
			delay_ms(1);
			if( USB_HUB_ST & ( bUHS_HP_PIN | bUHS_HM_PIN ) )
			{
				UHUB1_CTRL = UHUB1_CTRL & ~bUH_BUS_RESET;  						/* 结束复位 */
				return( 0x01 );
			}
			--n;
		}
        UHUB1_CTRL = UHUB1_CTRL & ~ bUH_BUS_RESET;               // 结束复位
    }
    else
    {
        UHUB0_CTRL = UHUB0_CTRL & ~ bUH_LOW_SPEED | bUH_BUS_RESET;// 默认为全速,开始复位
        while( n ) 
		{  
			delay_ms(1);
			if( USB_HUB_ST & ( bUHS_DP_PIN | bUHS_DM_PIN ) )
			{
				UHUB0_CTRL = UHUB0_CTRL & ~bUH_BUS_RESET;  						/* 结束复位 */
				return( 0x01 );
			}
			-- n;
		}
        UHUB0_CTRL = UHUB0_CTRL & ~ bUH_BUS_RESET;               // 结束复位
    }
    delay_us( 200 );
    UIF_DETECT = 0;											     // 清连接中断标志
	return ( 0x00 );
}

/*******************************************************************************
* Function Name  : EnableRootHubPort( UINT8 RootHubIndex )
* Description    : 使能ROOT-HUB端口,相应的bUH_PORT_EN置1开启端口,设备断开可能导致返回失败
* Input          : UINT8 RootHubIndex 指定端口
* Output         : None
*******************************************************************************/
UINT8 EnableRootHubPort( UINT8 id,usb_speed_t* pspeed )
{
	if ((id >> 4) == 1) {
		if (USB_HUB_ST & bUHS_H1_ATTACH){ // HUB1有设备
			if ((UHUB1_CTRL & bUH_PORT_EN) == 0x00){ // 尚未使能
				*pspeed = USB_HUB_ST & bUHS_HM_LEVEL ? USB_SPEED_LOW : USB_SPEED_FULL;
				if (USB_SPEED_LOW == *pspeed) {
					UHUB1_CTRL |= bUH_LOW_SPEED; // 低速
				}
			}
			UHUB1_CTRL |= bUH_PORT_EN; // 使能HUB1端口
			return (ERR_SUCCESS);
		}
	} else {
		if (USB_HUB_ST & bUHS_H0_ATTACH){ // HUB0有设备
			if ((UHUB0_CTRL & bUH_PORT_EN) == 0x00){ // 尚未使能
				*pspeed = USB_HUB_ST & bUHS_DM_LEVEL ? USB_SPEED_LOW : USB_SPEED_FULL;
				if (USB_SPEED_LOW == *pspeed) {
					UHUB0_CTRL |= bUH_LOW_SPEED; // 低速
				}
			}
			UHUB0_CTRL |= bUH_PORT_EN; // 使能HUB0端口
			return (ERR_SUCCESS);
		}
	}
    return( ERR_USB_DISCON );
}


/*******************************************************************************
* Function Name  : DisableRootHubPort(UINT8 RootHubIndex)
* Description    : 关闭HUB端口
* Input          : UINT8 RootHubIndex 指定ROOT_HUB口
* Output         : None
* Return         : None
*******************************************************************************/
void DisableRootHubPort( UINT8 id )          // 关闭指定的ROOT-HUB端口,实际上硬件已经自动关闭,此处只是清除一些结构状态
{
    if ( (id >> 4) == 1 )
    {
        UHUB1_CTRL = 0x00;                      // 清除有关HUB1的控制数据,实际上不需要清除
    }
    else
    {
        UHUB0_CTRL = 0x00;                      // 清除有关HUB0的控制数据,实际上不需要清除
    }
}



/*******************************************************************************
* Function Name  : USBHostTransact
* Description    : CH559传输事务,输入目的端点地址/PID令牌,同步标志,以20uS为单位的NAK重试总时间(0则不重试,0xFFFF无限重试),返回0成功,超时/出错重试
                   本子程序着重于易理解,而在实际应用中,为了提供运行速度,应该对本子程序代码进行优化
* Input          : UINT8 endp_pid 令牌和地址  endp_pid: 高4位是token_pid令牌, 低4位是端点地址
                   UINT8 tog      同步标志
                   UINT16 timeout 超时时间
* Output         : None
* Return         : ERR_USB_UNKNOWN 超时，可能硬件异常
                   ERR_USB_DISCON  设备断开
                   ERR_USB_CONNECT 设备连接
                   ERR_SUCCESS     传输完成
*******************************************************************************/
UINT8   USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT16 timeout )
{
    UINT8   TransRetry;
    UINT8   r;
    UINT16  i;
    UH_RX_CTRL = tog;
    UH_TX_CTRL =tog ;
    TransRetry = 0;
    do
    {
        UH_EP_PID = endp_pid;                                    // 指定令牌PID和目的端点号
        UIF_TRANSFER = 0;                                        // 允许传输
        for ( i = DEF_WAIT_USB_TOUT_200US; i != 0 && UIF_TRANSFER == 0; i -- )
        {
            delay_us( 1 );
        }
        UH_EP_PID = 0x00;                                        // 停止USB传输
        if ( UIF_TRANSFER == 0 )
        {
            return( ERR_USB_UNKNOWN );
        }
        if ( UIF_TRANSFER )                                      // 传输完成
        {
            if ( U_TOG_OK )
            {
                return( ERR_SUCCESS );
            }
            r = USB_INT_ST & MASK_UIS_H_RES;                     // USB设备应答状态
            if ( r == USB_PID_STALL )
            {
                return( r | ERR_USB_TRANSFER );
            }
            if ( r == USB_PID_NAK )
            {
                if ( timeout == 0 )
                {
                    return( r | ERR_USB_TRANSFER );
                }
                if ( timeout < 0xFFFF )
                {
                    timeout --;
                }
                -- TransRetry;
            }
            else switch ( endp_pid >> 4 )
                {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if ( U_TOG_OK )
                    {
                        return( ERR_SUCCESS );
                    }
                    if ( r == USB_PID_ACK )
                    {
                        return( ERR_SUCCESS );
                    }
                    if ( r == USB_PID_STALL || r == USB_PID_NAK )
                    {
                        return( r | ERR_USB_TRANSFER );
                    }
                    if ( r )
                    {
                        return( r | ERR_USB_TRANSFER );          // 不是超时/出错,意外应答
                    }
                    break;                                       // 超时重试
                case USB_PID_IN:
                    if ( U_TOG_OK )
                    {
                        return( ERR_SUCCESS );
                    }
                    if ( tog ? r == USB_PID_DATA1 : r == USB_PID_DATA0 )
                    {
                        return( ERR_SUCCESS );
                    }
                    if ( r == USB_PID_STALL || r == USB_PID_NAK )
                    {
                        return( r | ERR_USB_TRANSFER );
                    }
                    if ( r == USB_PID_DATA0 && r == USB_PID_DATA1 )// 不同步则需丢弃后重试
                    {
                    }                                            // 不同步重试
                    else if ( r )
                    {
                        return( r | ERR_USB_TRANSFER );          // 不是超时/出错,意外应答
                    }
                    break;                                       // 超时重试
                default:
                    return( ERR_USB_UNKNOWN );                   // 不可能的情况
                    break;
                }
        }
        else                                                     // 其它中断,不应该发生的情况
        {
            USB_INT_FG = 0xFF;                                   //清中断标志
        }
        delay_us( 15 );
    }
    while ( ++ TransRetry < 3 );
    return( ERR_USB_TRANSFER );                                  // 应答超时
}
/*******************************************************************************
* Function Name  : HostCtrlTransfer
* Description    : 执行控制传输,8字节请求码在pSetupReq中,DataBuf为可选的收发缓冲区
* Input          : PUINT8X DataBuf 如果需要接收和发送数据,那么DataBuf需指向有效缓冲区用于存放后续数据
                   PUINT8 RetLen  实际成功收发的总长度保存在RetLen指向的字节变量中
* Output         : None
* Return         : ERR_USB_BUF_OVER IN状态阶段出错
                   ERR_SUCCESS     数据交换成功
                   其他错误状态
*******************************************************************************/
UINT8   HostCtrlTransfer( uint8_t ep0_size, usb_control_request_t *preq, uint8_t *pbuf, uint16_t *plen )
{
    UINT16  RemLen  = 0;
    UINT8   s, RxLen, RxCnt, TxCnt;
    PUINT8  pBuf;
    PUINT16  xdata   pLen;

    pBuf = pbuf;
    pLen = plen;
    delay_us( 200 );
    if ( pLen )
    {
        *pLen = 0;                                                // 实际成功收发的总长度
    }
    UH_TX_LEN = sizeof( USB_SETUP_REQ );

	memcpy(TxBuffer,preq,sizeof(USB_SETUP_REQ));
    s = USBHostTransact( USB_PID_SETUP << 4 | 0x00, 0x00, 200000/20 );// SETUP阶段,200mS超时
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    UH_RX_CTRL = UH_TX_CTRL = bUH_R_TOG | bUH_R_AUTO_TOG | bUH_T_TOG | bUH_T_AUTO_TOG;// 默认DATA1
    UH_TX_LEN = 0x01;                                            // 默认无数据故状态阶段为IN
    RemLen = SWAP16_L( preq -> wLength );
    if ( RemLen && pBuf )                                        // 需要收发数据
    {
        if ( preq->bmRequestType.val & USB_REQ_TYP_IN )        // 收
        {
            while ( RemLen )
            {
                delay_us( 200 );
                s = USBHostTransact( USB_PID_IN << 4 | 0x00, UH_RX_CTRL, 200000/20 );// IN数据
                if ( s != ERR_SUCCESS )
                {
                    return( s );
                }
                RxLen = USB_RX_LEN < RemLen ? USB_RX_LEN : RemLen;
                RemLen -= RxLen;
                if ( pLen )
                {
                    *pLen += RxLen;                             // 实际成功收发的总长度
                }
                for ( RxCnt = 0; RxCnt != RxLen; RxCnt ++ )
                {
                    *pBuf = RxBuffer[ RxCnt ];
                    pBuf ++;
                }
                if ( USB_RX_LEN == 0 || ( USB_RX_LEN & ( ep0_size - 1 ) ) )
                {
                    break;                                      // 短包
                }
            }
            UH_TX_LEN = 0x00;                                   // 状态阶段为OUT
        }
        else                                                    // 发
        {
            while ( RemLen )
            {
                delay_us( 200 );
                UH_TX_LEN = RemLen >= ep0_size ? ep0_size : RemLen;
                for ( TxCnt = 0; TxCnt != UH_TX_LEN; TxCnt ++ )
                {
                    TxBuffer[ TxCnt ] = *pBuf;
                    pBuf ++;
                }
                s = USBHostTransact( USB_PID_OUT << 4 | 0x00, UH_TX_CTRL, 200000/20 );// OUT数据
                if ( s != ERR_SUCCESS )
                {
                    return( s );
                }
                RemLen -= UH_TX_LEN;
                if ( pLen )
                {
                    *pLen += UH_TX_LEN;                        // 实际成功收发的总长度
                }
            }
        }
    }
    delay_us( 200 );
    s = USBHostTransact( ( UH_TX_LEN ? USB_PID_IN << 4 | 0x00: USB_PID_OUT << 4 | 0x00 ), bUH_R_TOG | bUH_T_TOG, 200000/20 );  // STATUS阶段
    if ( s != ERR_SUCCESS )
    {
        return( s );
    }
    if ( UH_TX_LEN == 0 )
    {
        return( ERR_SUCCESS );                                  // 状态OUT
    }
    if ( USB_RX_LEN == 0 )
    {
        return( ERR_SUCCESS );                                  // 状态IN,检查IN状态返回数据长度
    }
    return( ERR_USB_BUF_OVER );                                 // IN状态阶段错误
}


/*********************************************************************
 * @fn      USBFSH_GetEndpData
 *
 * @brief   Get data from USB device input endpoint.
 *
 * @para    endp_num: Endpoint number.
 *          endp_tog: Endpoint toggle.
 *          *pbuf: Data Buffer.
 *          *plen: Data length.
 *
 * @return  The result of getting data.
 */
uint8_t USBHDH_GetEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;

    s = USBHostTransact( ( USB_PID_IN << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *plen = USB_RX_LEN;
        memcpy( pbuf, RxBuffer, *plen );
        
        *pendp_tog ^= bUH_R_TOG;
    }
    
    return s;
}

/*********************************************************************
 * @fn      USBHDH_SendEndpData
 *
 * @brief   Send data to the USB device output endpoint.
 *
 * @para    endp_num: Endpoint number
 *          endp_tog: Endpoint toggle
 *          *pbuf: Data Buffer
 *          *plen: Data length
 *
 * @return  The result of sending data.
 */
uint8_t USBHDH_SendEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t len )
{
    uint8_t  s;
    
    memcpy( TxBuffer, pbuf, len );
    UH_TX_LEN = len;
    
    s = USBHostTransact( ( USB_PID_OUT << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )  
    {
        *pendp_tog ^= bUH_T_TOG;
    }
    
    return s;
}



/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:		
** Returns:	
** Description:		
*******************************************************************/
error_t hal_usbh_port_en(uint8_t id,uint8_t en, usb_speed_t* pspeed)
{
	uint8_t i,s;
	if(en){
        for( i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++ ){
            if( EnableRootHubPort(id, (uint8_t*)pspeed ) == ERR_SUCCESS ){
                i = 0;
                s++;
                if( s > 6 ){
                    break;
                }
            }
            delay_ms( 1 );
        }
        if(0 == i){
            return ERROR_SUCCESS;
        }else{
            return ERROR_DISCON;
        }
	}else{
		DisableRootHubPort(id);
		return ERROR_SUCCESS;
	}
}
error_t hal_usbh_set_speed(uint8_t id, usb_speed_t speed)
{
	SetUsbSpeed(speed);
	return ERROR_SUCCESS;
}
error_t hal_usbh_port_reset(uint8_t id)
{
	return ResetRootHubPort( id );
}
error_t hal_usbh_set_addr(uint8_t id,uint8_t addr)
{
	SetHostUsbAddr(addr);
	return ERROR_SUCCESS;
}
error_t hal_usbh_endp_unregister(uint8_t id,usb_endp_t *endpp)
{
	return ERROR_SUCCESS;
}
error_t hal_usbh_endp_register(uint8_t id,usb_endp_t *endpp)
{
	return ERROR_SUCCESS;
}
error_t hal_usbh_ctrl_transfer( uint8_t id, usb_control_request_t* preq,uint8_t* buf, uint16_t* plen)
{
	uint8_t err;
    usbh_dev_t* pdev = get_usbh_dev(id);

    err = HostCtrlTransfer( pdev->endp0_mtu,  (USB_SETUP_REQ *)preq, buf, plen );
logd("HostCtrlTransfer len=%x\n",*plen);
    err =  err ?  ERROR_FAILE: ERROR_SUCCESS;
	return err;
}

error_t hal_usbh_in(uint8_t id, usb_endp_t *endpp, uint8_t* buf,uint16_t* plen,uint16_t timeout_ms)
{
    error_t err;
    uint8_t tog = endpp->sync? bUH_R_TOG : 0;
    uint8_t endp_num = endpp->addr;

    err = USBHDH_GetEndpData(endp_num, &tog, buf, plen );
    endpp->sync = BOOL_SET(tog);

    err =  err ?  ERROR_FAILE: ERROR_SUCCESS;
	return err;
}
error_t hal_usbh_out(uint8_t id, usb_endp_t *endpp, uint8_t* buf, uint16_t len)
{
    error_t err;
    uint8_t tog = endpp->sync? bUH_T_TOG : 0;
    uint8_t endp_num = endpp->addr;

    err = USBHDH_SendEndpData( endp_num, &tog, buf, len );
    err =  err ?  ERROR_FAILE: ERROR_SUCCESS;

	return err;
}
error_t hal_usbh_driver_init(uint8_t id)
{
	UINT8   i;
    IE_USB = 0;
    USB_CTRL = bUC_HOST_MODE;                                                       // 先设定模式
    USB_DEV_AD = 0x00;
    UH_EP_MOD = bUH_EP_TX_EN | bUH_EP_RX_EN ;
    UH_RX_DMA = RxBuffer;
    UH_TX_DMA = TxBuffer;
    UH_RX_CTRL = 0x00;
    UH_TX_CTRL = 0x00;
    USB_CTRL = bUC_HOST_MODE | bUC_INT_BUSY | bUC_DMA_EN;                           // 启动USB主机及DMA,在中断标志未清除前自动暂停
    UH_SETUP = bUH_SOF_EN;
    USB_INT_FG = 0xFF;   
	                                                           // 清中断标志
    DisableRootHubPort( id );   
    USB_INT_EN = bUIE_TRANSFER | bUIE_DETECT;

	return ERROR_SUCCESS;
}
error_t hal_usbh_driver_deinit(uint8_t id)
{
	DisableRootHubPort(id);
	return ERROR_SUCCESS;
}
void hal_usbh_driver_task(uint32_t dt_ms)
{
	RootHUB_Detect_USB_Plug();
}

#endif


