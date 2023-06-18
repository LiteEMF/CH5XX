
/********************************** (C) COPYRIGHT *******************************
* File Name          : SPI0Mater.C
* Author             : WCH
* Version            : V1.3
* Date               : 2019/07/22
* Description        : CH559�ṩSPI0����ģʽ�����ӿں���             				   
*******************************************************************************/
#include "..\DEBUG.C"                                                          //������Ϣ��ӡ
#include "..\DEBUG.H"

#define SPI0Interrupt   0                                                      //�趨SPI0�����շ��жϷ�ʽ���߲�ѯ��ʽ
UINT8 flag;
UINT8 TmpBuf;

#pragma  NOAREGS
#define SET_SPI0_CK( d )   { SPI0_CK_SE = d; }                                 //d>=2

/*Ӳ���ӿڶ���*/
/******************************************************************************
ʹ��CH559 Ӳ��SPI�ӿ� 
         CH559        DIR       
         P1.4        <==>       SCS
         P1.5        <==>       MOSI
         P1.6        <==>       MISO
         P1.7        <==>       SCK
*******************************************************************************/


/*******************************************************************************
* Function Name  : CH559SPI0HostInit()
* Description    : CH559SPI0��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH559SPI0HostInit(void)
{
    PORT_CFG &= ~bP1_OC;
    P1_DIR |= (bSCK | bMOSI | bSCS);
    P1_IE |= bMISO;                                                            //��������
	
    SPI0_SETUP &= ~(bS0_MODE_SLV | bS0_BIT_ORDER);                             //����Ϊ����ģʽ���ֽ�˳��Ϊ���ģʽ		
    SPI0_CTRL |=  bS0_MOSI_OE  | bS0_SCK_OE ;                                  //MISO���ʹ�ܣ�SCK���ʹ��
    SPI0_CTRL &= ~(bS0_MST_CLK | bS0_2_WIRE);
    SPI0_CTRL &=  ~(bS0_DATA_DIR);                                             //����д��Ĭ�ϲ�����д���䣬���ʹ��bS0_DATA_DIR��
	                                                                             //��ô�������ݺ��Զ�����һ���ֽڵ�ʱ�ӣ����ڿ��������շ�	
    SET_SPI0_CK(6);                                                              //6��Ƶ
    SPI0_CTRL &= ~bS0_CLR_ALL;                                                 //���SPI0��FIFO,Ĭ����1������������ܷ�������
}

/*******************************************************************************
* Function Name  : CH559SPI0Write(UINT8 dat)
* Description    : CH559Ӳ��SPIд����
* Input          : UINT8 dat   ����
* Output         : None
* Return         : None
*******************************************************************************/
void CH559SPI0Write(UINT8 dat)
{
    SPI0_DATA = dat;                                                           
    while(S0_FREE == 0);													   //�ȴ��������		
//���bS0_DATA_DIRΪ1���˴�����ֱ�Ӷ�ȡһ���ֽڵ��������ڿ��ٶ�д	
}

/*******************************************************************************
* Function Name  : CH559SPI0Read( )
* Description    : CH559Ӳ��SPI0������
* Input          : None
* Output         : None
* Return         : UINT8 ret   
*******************************************************************************/
UINT8 CH559SPI0Read()
{
    SPI0_DATA = 0xff;
    while(S0_FREE == 0);
    return SPI0_DATA;
}



void main()
{
    UINT8 ret,i=0;
    mDelaymS(30);                                                              //�ϵ���ʱ,�ȴ��ڲ������ȶ�,�ؼ� 
//  CfgFsys( );     
    mInitSTDIO( );                                                             //����0,�������ڵ���
    printf("start ...\n");  
    CH559SPI0HostInit();                                                       //SPI0����ģʽ��ʼ��
	mDelaymS(100);
    while(1)
    {   
	     SCS = 0;                                                               //SPI������������
        CH559SPI0Write(i);
        mDelaymS(5);
        ret = CH559SPI0Read();                                            //����SPI�ӻ����ص����ݣ�ȡ������
        SCS = 1;
        if(ret != (i^0xff))
        {
            printf("Err: %02X  %02X  \n",(UINT16)i,(UINT16)ret);               //��������ڷ������ݵ�ȡ������ӡ������Ϣ
        }
        else
        {
            printf("success %02x\n",(UINT16)i);                               
        }
        i = i+1;
        mDelaymS(50);
    }
}