
/********************************** (C) COPYRIGHT *******************************
* File Name          : SPI0Flash.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559 SPI0 ��д�ⲿFlash
*******************************************************************************/
#include ".\DEBUG.C"                                                          //������Ϣ��ӡ
#include ".\DEBUG.H"

#pragma  NOAREGS

sbit CHIP_SELECT = P1^4;
#define SENDBYTE_SPI( d )    {  SPI0_DATA = d;while(S0_FREE == 0); }
#define RECVBYTE_SPI( d )    { SPI0_DATA = 0xff;while(S0_FREE == 0);d = SPI0_DATA;}

UINT8 buf[50]; 

/*******************************************************************************
* Function Name  : InitHostSPI0( void )
* Description    : SPI0����ģʽ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void    InitHostSPI0( void )
{
    SPI0_SETUP &=~(bS0_MODE_SLV | bS0_BIT_ORDER);                              /*���ó�����ģʽ*/
    SPI0_CTRL = bS0_SCK_OE | bS0_MOSI_OE;                                      /*����д��Ĭ�ϲ�����д���䣬���ʹ��bS0_DATA_DIR*/
                                                                               /*��ô�������ݺ��Զ�����һ���ֽڵ�ʱ�ӣ����ڿ��������շ�*/
    P1_DIR |= (bMOSI | bSCK | bSCS| bPWM3 );                                   /*bMOSI ��bSCK ��bSCS��Ϊ�������*/
    P1_DIR &= ~bMISO;
    SPI0_CK_SE = 0x02;                                                         /*��ƵΪ6M*/
//  SPI0_STAT = 0xFF;                                                          /*���жϱ�־*/
//  IE_SPI0 = 1;
}

/*******************************************************************************
* Function Name  : ReadExternalFlashStatusReg_SPI
* Description    : ������ȡ״̬�Ĵ���,������״̬�Ĵ�����ֵ
* Input          : None
* Output         : None
* Return         : ExFlashRegStatus
*******************************************************************************/
UINT8 ReadExternalFlashStatusReg_SPI( void )
{
    UINT8 ExFlashRegStatus;
    CHIP_SELECT = 0;
    SENDBYTE_SPI(0x05);                                                        /*���Ͷ�״̬�Ĵ��������� */
    RECVBYTE_SPI(ExFlashRegStatus);                                            /*��ȡ״̬�Ĵ���*/
    CHIP_SELECT = 1 ;
    return ExFlashRegStatus;
}
   
/*******************************************************************************
* Function Name  : WaitExternalFlashIfBusy
* Description    : �ȴ�оƬ����(��ִ��Byte-Program, Sector-Erase, Block-Erase, Chip-Erase������)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WaitExternalFlashIfBusy( void )
{
    while ((ReadExternalFlashStatusReg_SPI())&0x01 == 0x01 )                   /*�ȴ�ֱ��Flash����*/
    {
        ReadExternalFlashStatusReg_SPI( );
    }
}

/*******************************************************************************
* Function Name  : WriteExternalFlashStatusReg_SPI
* Description    : ��״̬�Ĵ�����дһ���ֽ�
* Input          : status -д�������
* Output         : None
* Return         : None
*******************************************************************************/
void WriteExternalFlashStatusReg_SPI( UINT8 status )
{
    CHIP_SELECT = 0 ;
    SENDBYTE_SPI(0x01);                                                       /*����д״̬�Ĵ���*/
    SENDBYTE_SPI(status);                                                     /*�ı�Ĵ�����BPx����BPL (ֻ��2,3,4,5,7λ���Ը�д)*/
    CHIP_SELECT = 1 ;
}
 
/*******************************************************************************
* Function Name  : WriteExternalFlashEnable_SPI
* Description    : дʹ��,ͬ����������ʹ��д״̬�Ĵ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WriteExternalFlashEnable_SPI( void )
{
    CHIP_SELECT = 0 ;
    SENDBYTE_SPI(0x06);                                                       /*����дʹ������*/
    CHIP_SELECT = 1 ;
}

/*******************************************************************************
* Function Name  : CheckExternalFlashWriteEnable_SPI
* Description    : ����д����ǰWELλ�Ƿ�Ϊ1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CheckExternalFlashWriteEnable_SPI( void )
{
    UINT8 WRENStatus;
    WRENStatus = ReadExternalFlashStatusReg_SPI();                            /*��ȡ״̬register*/
    if ((WRENStatus&0x02) != 0x02)                                            /*���WELλ��λ*/
    {
        WriteExternalFlashEnable_SPI( );                                      /*���δ��1������Ӧ����,��������дʹ�ܲ���*/
    }
}

/*******************************************************************************
* Function Name  : EraseExternalFlash_SPI
* Description    : �����ⲿFlash
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EraseExternalFlash_SPI( void )
{
    CheckExternalFlashWriteEnable_SPI();
    CHIP_SELECT = 0 ;
    SENDBYTE_SPI(0x60);                                                        /*���� Chip Erase���� (60h or C7h)*/
    CHIP_SELECT = 1 ;
    WaitExternalFlashIfBusy();
}

/*******************************************************************************
* Function Name  : ByteReadExternalFlash_SPI
* Description    : ��ȡһ����ַ��һ���ֽڵ�����.���ض�ȡ������ 
* Input          : UINT32 StarAddr -Destination Address 000000H - 1FFFFFH
* Output         : None
* Return         : byte -��ȡ������
*******************************************************************************/
UINT8 ByteReadExternalFlash_SPI(UINT32 StarAddr)    
{   
    UINT8 dat = 0;    
    CHIP_SELECT = 0 ;                                                           //enable device  
    SENDBYTE_SPI(0x03);                                                         //read command 
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16));                                //send 3 address bytes  
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
    RECVBYTE_SPI(dat);   
    CHIP_SELECT = 1 ;                                                           //disable device   
    return dat;                                                                 //return one byte read
} 

/*******************************************************************************
* Function Name  : ByteWriteExternalFlash_SPI
* Description    : д����
* Input          : StarAddr  -Destination Address 000000H - 1FFFFFH
*                  dat -Ҫд�������
* Output         : None
* Return         : None
*******************************************************************************/
void ByteWriteExternalFlash_SPI(UINT32 StarAddr, UINT8 dat)
{
    WriteExternalFlashEnable_SPI();
    CHIP_SELECT = 0 ;                                                          //оƬʹ�� 
    SENDBYTE_SPI(0x02);                                                        //����д����ָ�� 
    SENDBYTE_SPI(((StarAddr & 0xFFFFFF) >> 16));                                    //����3�ֽڵ�ַ 
    SENDBYTE_SPI(((StarAddr & 0xFFFF) >> 8));
    SENDBYTE_SPI(StarAddr & 0xFF);
    SENDBYTE_SPI(dat);                                                         //����Ҫд������
    CHIP_SELECT = 1 ;
    WaitExternalFlashIfBusy();
}

/*******************************************************************************
* Function Name  : BlukReadExternalFlash_SPI
* Description    : ��ȡ��ʼ��ַ(StarAddr)�ڶ���ֽ�(Len)������.���뻺����RcvBuffer��
* Input          : StarAddr -Destination Address 000000H - 1FFFFFH
                   Len ��ȡ���ݳ���
                   RcvBuffer ���ջ�������ʼ��ַ
* Output         : None
* Return         : None
*******************************************************************************/
void BlukReadExternalFlash_SPI(UINT32 StarAddr,UINT16 Len,PUINT8 RcvBuffer)
{
    UINT16 i;
    for(i=0; i<Len; i++)                                                        /*��������*/
    {
        RcvBuffer[i] = ByteReadExternalFlash_SPI(StarAddr);
        StarAddr++;                                                             /*��ȡ��һ��ַ*/
    }
}

/*******************************************************************************
* Function Name  : BlukWriteExternalFlash_SPI
* Description    : ������д���ⲿFlash
* Input          : StarAddr  -Destination Address 000000H - 1FFFFFH
                   Len �������ݳ���
*                  SendBuffer -�������ݻ�����
* Output         : None
* Return         : None
*******************************************************************************/
void BlukWriteExternalFlash_SPI(UINT32 StarAddr,UINT16 Len,PUINT8 SendBuffer)
{
    UINT16 i;
    for(i=0; i<Len; i++)                                                        /*����Ҫд������*/
    {
        ByteWriteExternalFlash_SPI(StarAddr,*(SendBuffer+i)); 
        StarAddr++;			
    }
}

void main( ) 
{
    UINT32  i;
//  mDelaymS(30);                                                                  //�ϵ���ʱ,�ȴ��ڲ������ȶ�
    mInitSTDIO( );                                                                 /* Ϊ���ü����ͨ�����ڼ����ʾ���� */
    printf( "Start SPI FLASH @ChipID=%02X\n", (UINT16)CHIP_ID );

    InitHostSPI0( );
#if  0	
    printf("Address(0xF8) = %02x\n",(UINT16)SPI0_STAT);
    printf("Address(0xF9) = %02x\n",(UINT16)SPI0_DATA);
    printf("Address(0xFA) = %02x\n",(UINT16)SPI0_CTRL);
    printf("Address(0xFB) = %02x\n",(UINT16)SPI0_CK_SE);
    printf("Address(0xFC) = %02x\n",(UINT16)SPI0_SETUP);	
#endif
    WriteExternalFlashEnable_SPI( );                                               //FLASHдʹ��
    WriteExternalFlashStatusReg_SPI( 0x00 );                                       //д�Ĵ��� 
    EraseExternalFlash_SPI( );                                                     //FLASH��Ƭ����
    printf("Chip_Erase over\n");
    for(i = 0 ;i < 50 ;i ++)
    {
        buf[i] = i;                                                
    }
    BlukWriteExternalFlash_SPI(0,50,&buf[0]);                                      //��FLASH��ַ0x00000000д������
    printf("Write over\n");
    BlukReadExternalFlash_SPI(0,50,&buf[0]);                                       //��FLASH��ַ0x00000000��������
    for(i = 0 ;i < 50 ;i ++)
    {
        printf(" %02x  ",(UINT16)buf[i]);                                               
    }
	  while(1);
}