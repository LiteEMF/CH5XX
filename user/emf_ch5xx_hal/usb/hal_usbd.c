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
#if API_USBD_BIT_ENABLE
#include  "api/usb/usb_typedef.h"
#include "api/usb/device/usbd.h"
#include  "api/api_system.h"

#include "api/api_log.h"

/******************************************************************************************************
** Defined
*******************************************************************************************************/
#define USBD_ID         0

/******************************************************************************************************
**	public Parameters
*******************************************************************************************************/


/******************************************************************************************************
**	static Parameters
*******************************************************************************************************/
#if !defined(__C51__)
__ALIGN(4) uint8x_t Ep0Buffer[(64 + 2) + (64 + 2) * 2]; // 端点0+4 OUT&IN缓冲区，必须是偶地址
__ALIGN(4) uint8x_t Ep1Buffer[(64 + 2) * 2]; // 端点1 IN/OUT
__ALIGN(4) uint8x_t Ep2Buffer[(64 + 2) * 2]; // 端点2 IN/OUT
__ALIGN(4) uint8x_t Ep3Buffer[(64 + 2) * 2]; // 端点3 IN/OUT
#else
uint8x_t Ep0Buffer[(64 + 2) + (64 + 2) * 2] _at_(0x0080); // 端点0+4 OUT&IN缓冲区，必须是偶地址
uint8x_t Ep1Buffer[(64 + 2) * 2] _at_(0x80 + (64 + 2) + (64 + 2) * 2); // 端点1 IN/OUT
uint8x_t Ep2Buffer[(64 + 2) * 2] _at_(0x80 + (64 + 2) + (64 + 2) * 4); // 端点2 IN/OUT
uint8x_t Ep3Buffer[(64 + 2) * 2] _at_(0x80 + (64 + 2) + (64 + 2) * 6); // 端点3 IN/OUT
#endif

/*****************************************************************************************************
**	static Function
******************************************************************************************************/



/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description    : CH559USB中断处理函数
*******************************************************************************/
void USBHD_IRQHandler( void )	interrupt INT_NO_USB using 1               /* USB中断服务程序,使用寄存器组1 */
{
    uint16_t len;
    uint8_t ep;
    uint8_t  intst;

    intst = USB_INT_ST;
    ep = intst & MASK_UIS_ENDP;

    if (UIF_TRANSFER) { // USB传输完成标志
        switch (intst & MASK_UIS_TOKEN) {

        case UIS_TOKEN_IN:
            usbd_endp_in_event(USBD_ID, TUSB_DIR_IN_MASK | ep);
            if(0 == ep){
                UEP0_CTRL ^= bUEP_T_TOG; // 同步标志位翻转
            }else if(4 == ep){
                UEP4_CTRL ^= bUEP_T_TOG; // 同步标志位翻转
            }
            break;
        case UIS_TOKEN_OUT:
            if (U_TOG_OK) {                     // 不同步的数据包将丢弃
                usbd_endp_out_event(USBD_ID, ep, USB_RX_LEN);
                if(0 == ep){
                    UEP0_CTRL ^= bUEP_R_TOG; // 同步标志位翻转
                }else if(4 == ep){
                    UEP4_CTRL ^= bUEP_R_TOG; // 同步标志位翻转
                }
            }
            break;
        case UIS_TOKEN_SETUP: // SETUP
            UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_NAK | UEP_T_RES_NAK;
            len = USB_RX_LEN;
            if (len == sizeof(usb_control_request_t)) {
                UEP0_T_LEN = 0;
                usbd_setup_event(USBD_ID, (usb_control_request_t*)Ep0Buffer , sizeof(usb_control_request_t));
            } else {
                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
            }
            break;
        default:
            break;
        }
        UIF_TRANSFER = 0;
    } else if (UIF_BUS_RST){
        /* usb reset interrupt processing */
        usbd_reset_event(USBD_ID);
        UIF_BUS_RST = 0;
    } else if (UIF_SUSPEND){
        usbd_suspend_event(USBD_ID);
        UIF_SUSPEND = 0;
    } else { 
        USB_INT_FG = 0XFF;
    }
}

/*****************************************************************************************************
**  Function
******************************************************************************************************/

/*******************************************************************
** Parameters:
** Returns:
** Description:
*******************************************************************/
error_t hal_usbd_endp_dma_init(uint8_t id)
{
	return ERROR_SUCCESS;
}
error_t hal_usbd_endp_open(uint8_t id, usb_endp_t* pendp)
{
    uint8d_t mode = 0, ctrl = 0, mask = 0;
    // logd("enp init %d %d\n",(uint16_t)(endp), (uint16_t)(in_out));

    if (0 == pendp->addr) {
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK; // OUT事务返回ACK，IN事务返回NAK
        return ERROR_SUCCESS;
    }

    if (pendp->dir) {
        mode |= bUEP2_TX_EN;
        ctrl |= UEP_T_RES_NAK;
        mask = MASK_UEP_T_RES;
    } else {
        mode |= bUEP2_RX_EN;
        ctrl |= UEP_R_RES_ACK;
        mask = MASK_UEP_R_RES;
    }
    if ((pendp->addr == 1) || (pendp->addr == 3)) {
        mode <<= 4;
    }

    switch (pendp->addr) {
    case 1:
        UEP4_1_MOD |= mode;
        UEP1_CTRL = (UEP1_CTRL & ~mask) | ctrl | bUEP_AUTO_TOG;   // 端点1自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
        break;
    case 2:
        UEP2_3_MOD |= mode;
        UEP2_CTRL = (UEP2_CTRL & ~mask) | ctrl | bUEP_AUTO_TOG; // 端点2自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
        break;
    case 3:
        UEP2_3_MOD |= mode;
        UEP3_CTRL = (UEP3_CTRL & ~mask) | ctrl | bUEP_AUTO_TOG; // 端点3自动翻转同步标志位，IN事务返回NAK，OUT返回ACK
        break;
    case 4:
        UEP4_1_MOD |= mode; 
        UEP4_CTRL = (UEP4_CTRL & ~mask) | ctrl;               // 端点4不支持自动翻转同步标志位 
        break;
    }

    return ERROR_SUCCESS;
}

error_t hal_usbd_endp_close(uint8_t id, uint8_t ep)
{
    uint8_t ep_addr = ep & 0x7f;

    if (0 == ep_addr) {
        return ERROR_SUCCESS;
    }

    switch (ep_addr) {
    case 1:
        UEP1_T_LEN = 0;
        UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP4_1_MOD &= ~(bUEP1_TX_EN | bUEP1_RX_EN | bUEP1_BUF_MOD);
        break;
    case 2:
        UEP2_T_LEN = 0;
        UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP2_3_MOD &= ~(bUEP2_TX_EN | bUEP2_RX_EN | bUEP2_BUF_MOD);
        break;
    case 3:
        UEP3_T_LEN = 0;
        UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP2_3_MOD &= ~(bUEP3_TX_EN | bUEP3_RX_EN | bUEP3_BUF_MOD);
        break;
    case 4:
        UEP4_T_LEN = 0;
        UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
        UEP4_1_MOD &= ~(bUEP4_TX_EN | bUEP4_RX_EN);     //端点4 dma buf和端点0 共用
        break;
    }

    return ERROR_SUCCESS;
}
error_t hal_usbd_endp_ack(uint8_t id, uint8_t ep, uint16_t len)
{
    switch (ep) {
    case 0x80:
        UEP0_T_LEN = len;
		UEP0_CTRL = (UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x81:
        UEP1_T_LEN = len;
		UEP1_CTRL = (UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x82:
        UEP2_T_LEN = len;
        UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x83:
        UEP3_T_LEN = len;
        UEP3_CTRL = (UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
    case 0x84:
        UEP4_T_LEN = len;
        UEP4_CTRL = (UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_ACK;
        break;
        
    case 0x00:
        UEP0_CTRL = UEP0_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;  //重新开启接收
        break;
    default:
        break;
    }
	return ERROR_SUCCESS;
}
error_t hal_usbd_endp_nak(uint8_t id, uint8_t ep)
{

     // 关闭接收,等待数据处理
    switch (ep) {
    case 0x00:
        UEP0_CTRL = (UEP0_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK;
        break;
    case 0x80:
        UEP0_T_LEN = 0;
        UEP0_CTRL = (UEP0_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x81:
        UEP1_T_LEN = 0;
        UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x82:
        UEP2_T_LEN = 0;
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x83:
        UEP3_T_LEN = 0;
        UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x84:
        UEP4_T_LEN = 0;
        UEP4_CTRL = UEP4_CTRL & ~MASK_UEP_T_RES | UEP_T_RES_NAK; // 默认应答NAK
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & ~MASK_UEP_R_RES | UEP_R_RES_NAK; /* 设置端点2 OUT NAK */
        break;
    default:
        break;
    }
    return ERROR_SUCCESS;
}
       



error_t hal_usbd_clear_endp_stall(uint8_t id, uint8_t ep)
{
    switch (ep) {
    case 0x00:
    case 0x80:
        UEP0_T_LEN = 0; // 虽然尚未到状态阶段，但是提前预置上传0长度数据包以防主机提前进入状态阶段
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK; // 默认数据包是DATA1,返回应答ACK
        break;
    case 0x81:
        UEP1_CTRL = UEP1_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x82:
        UEP2_CTRL = UEP2_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x83:
        UEP3_CTRL = UEP3_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x84:
        UEP4_CTRL = UEP4_CTRL & ~(bUEP_T_TOG | MASK_UEP_T_RES) | UEP_T_RES_NAK;
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & ~(bUEP_R_TOG | MASK_UEP_R_RES) | UEP_R_RES_ACK;
        break;
    default:
        break;
    }
    return ERROR_SUCCESS;
}
error_t hal_usbd_endp_stall(uint8_t id, uint8_t ep)
{
    switch (ep) {
    case 0x00:
    case 0x80:
        UEP0_T_LEN = 0;
        UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL; // STALL
        break;
    case 0x81:
        UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
        break;
    case 0x82:
        UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点2 IN STALL */
        break;
    case 0x83:
        UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
        break;
    case 0x84:
        UEP4_CTRL = UEP4_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL; /* 设置端点1 IN STALL */
        break;
    case 0x01:
        UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    case 0x02:
        UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    case 0x03:
        UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    case 0x04:
        UEP4_CTRL = UEP4_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL; /* 设置端点2 OUT Stall */
        break;
    default:
        break;
    }

    return ERROR_SUCCESS;
}


uint8_t* hal_usbd_get_endp_buffer(uint8_t id, uint8_t ep)
{
    uint8_t *pbuf;
    uint8_t ep_addr = ep & 0x7f;

    switch (ep_addr) {
    case 0:
        return Ep0Buffer;
    case 1:
        pbuf = Ep1Buffer;
        if((ep & 0x80) && (UEP4_1_MOD & bUEP1_TX_EN) && (UEP4_1_MOD & bUEP1_RX_EN)){
            pbuf += 64;
        }

        break;
    case 2:
        pbuf = Ep2Buffer;
        if((ep & 0x80) && (UEP2_3_MOD & bUEP2_TX_EN) && (UEP2_3_MOD & bUEP2_RX_EN)){
            pbuf += 64;
        }
        break;
    case 3:
        pbuf = Ep3Buffer;
        if((ep & 0x80) && (UEP2_3_MOD & bUEP3_TX_EN) && (UEP2_3_MOD & bUEP3_RX_EN)){
            pbuf += 64;
        }
        break;
    case 4:
        pbuf = Ep0Buffer + USBD_ENDP0_MTU;
        if((ep & 0x80) && (UEP4_1_MOD & bUEP4_TX_EN) && (UEP4_1_MOD & bUEP4_RX_EN)){
            pbuf += 64;
        }
        break;
    default:
        pbuf = NULL;
        break;
    }

    return pbuf;
}



/*******************************************************************
** Parameters: buf: ch32f103平台端点0时参数无效,使用usbd_req_t中数据发送
** Returns:
** Description: 注意: 端点0 发送需要处理 usbd_req_t,usbd_free_setup_buffer释放空间
//TODO 后面优化这一部分代码
    调试记录:
        1. 设置nak后, in消息不会进中断,只有有效传送才会进中断
        2. 设置地址必须在ack 请求后再进行地址设置
*******************************************************************/
error_t hal_usbd_in(uint8_t id, uint8_t ep, uint8_t* buf, uint16_t len)
{
    error_t err = ERROR_FAILE;
    uint8_t ep_addr = ep & 0x7f;
    uint16_t send_len;
    uint8_t* endp_buf = hal_usbd_get_endp_buffer(id, TUSB_DIR_IN_MASK | ep);

    if(0 == ep_addr){
        usbd_req_t *preq = usbd_get_req(id);

        if(preq->setup_index <= preq->setup_len){
            send_len = preq->setup_len - preq->setup_index;
            send_len = (send_len >= USBD_ENDP0_MTU) ? USBD_ENDP0_MTU : send_len; //本次传输长度
            memcpy(Ep0Buffer, (void*)(preq->setup_buf+preq->setup_index), send_len);						          //加载上传数据
            preq->setup_index += send_len;

            err = hal_usbd_endp_ack(id, ep, send_len);
            if((preq->setup_index == preq->setup_len)){
                if(USBD_ENDP0_MTU != send_len){             //判断发送最后一包数据
                    usbd_free_setup_buffer(preq);           //发送完成释放内存
                    hal_usbd_endp_ack(id, 0x00, 0);         //开始接收
                }
            }
        }else{
            return ERROR_FAILE;
        }
    }else{
        memcpy(endp_buf, buf, len);
        err = hal_usbd_endp_ack(id, ep, len);
    }

    return err;
}

error_t hal_usbd_out(uint8_t id, uint8_t ep, uint8_t* buf, uint16_t* plen)
{
    uint8_t* p;

    p = hal_usbd_get_endp_buffer(id, ep);
    if(NULL != buf) memcpy(buf, p,*plen);
    hal_usbd_endp_ack(id, ep, 0);


    return ERROR_SUCCESS;
}
error_t hal_usbd_reset(uint8_t id)
{
    return hal_usbd_init(id);
}
error_t hal_usbd_set_address(uint8_t id,uint8_t address)
{
    USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | address;
    return ERROR_SUCCESS;
}


error_t hal_usbd_init(uint8_t id)
{
	IE_USB = 0;
    USB_CTRL = 0x00;                                                                // 先设定模式
    UEP4_1_MOD = bUEP1_TX_EN;                                                       // 端点1上传IN
    UEP2_3_MOD = bUEP2_RX_EN | bUEP2_TX_EN;                                         // 端点2下传OUT和上传IN
    UEP0_DMA = Ep0Buffer;
    UEP1_DMA = Ep1Buffer;
    UEP2_DMA = Ep2Buffer;
    UEP3_DMA = Ep3Buffer;
	USB_DEV_AD = 0x00;
    UDEV_CTRL = bUD_DP_PD_DIS | bUD_DM_PD_DIS;                                      // 禁止DP/DM下拉电阻
    USB_CTRL = bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                           // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
    UDEV_CTRL |= bUD_PORT_EN;                                                       // 允许USB端口
    USB_INT_FG = 0xFF;                                                              // 清中断标志
    USB_INT_EN = bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
    IE_USB = 1;

	return ERROR_SUCCESS;
}
error_t hal_usbd_deinit(uint8_t id)
{
	IE_USB = 0; 
    USB_CTRL = 0x00;		
	return ERROR_SUCCESS;
}

#endif



