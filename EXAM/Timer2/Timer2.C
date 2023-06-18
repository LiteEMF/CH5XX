/********************************** (C) COPYRIGHT *******************************
* File Name          : Timer2.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559 TIME2�ӿں���                				   
*******************************************************************************/
#include "..\DEBUG.C"                                                          //������Ϣ��ӡ
#include "..\DEBUG.H"

#ifndef TIMER
#define TIMER    0                                                             //T2��Ϊ��ʱ��
#define T2EX_CAP 0                                                             //T2ex��׽���ŵ�ƽ
#define T2_CAP   1                                                             //T2��׽���ŵ�ƽ
#endif

#pragma  NOAREGS

//CH559 Timer2ʱ��ѡ��   
//bTMR_CLKͬʱӰ��Timer0&1&2,ʹ��ʱҪע��                                                       
#define mTimer2ClkFsys( )      {T2MOD |= (bTMR_CLK | bT2_CLK);C_T2=0;}         //��ʱ��,ʱ��=Fsys
#define mTimer2Clk4DivFsys( )  {T2MOD &= ~bTMR_CLK;T2MOD |=  bT2_CLK;C_T2 = 0;}//��ʱ��,ʱ��=Fsys/4
#define mTimer2Clk12DivFsys( ) {T2MOD &= ~(bTMR_CLK | bT2_CLK);C_T2 = 0;}      //��ʱ��,ʱ��=Fsys/12
#define mTimer2CountClk( )     {C_T2 = 1;}                                     //������,T2���ŵ��½�����Ч

//CH559 Timer2 ��ʼ(SS=1)/����(SS=0)
#define mTimer2RunCTL( SS )    {TR2 = SS ? START : STOP;}

UINT8 FLAG;
UINT16 Cap[8] = {0};

/*******************************************************************************
* Function Name  : mTimer2Setup(UINT8 T2Out)
* Description    : CH559��ʱ2��ʼ��
* Input          : UINT8 T2Out,�Ƿ�����T2���ʱ��
                   0�����������
                   1���������  
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer2Setup(UINT8 T2Out)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 0;                                                                //�����Զ����ض�ʱ������
    if(T2Out)
    {
	      T2MOD |= T2OE;                                                        //�Ƿ�����T2���ʱ��,�������ʱ��=1/2��ʱ��2�����
    }
    else
    {
	      T2MOD &= ~T2OE;
    }
}

/*******************************************************************************
* Function Name  : mTimer2Init(UINT16 Tim)
* Description    : CH559 T2��ʱ������ֵ                   
* Input          : UINT16 Tim,��ʱ����ֵ
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer2Init(UINT16 Tim)
{
    UINT16 tmp;
    tmp = 65536 - Tim;
    RCAP2L = TL2 = tmp & 0xff;
    RCAP2H = TH2 = (tmp >> 8) & 0xff;
}

/*******************************************************************************
* Function Name  : T2exCaptureSetup(UINT8 mode)
* Description    : CH559��ʱ������2 T2EX���Ų�׽���ܳ�ʼ��
                   UINT8 mode,���ز�׽ģʽѡ��
                   0:T2ex���½��ص���һ���½���
                   1:T2ex�������֮��
                   3:T2ex�������ص���һ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void T2exCaptureSetup(UINT8 mode)
{
    C_T2  = 0;
    EXEN2 = 1; 
    CP_RL2 = 1;                                                                //����T2ex�Ĳ�׽����
    T2MOD |= mode << 2;                                                        //���ز�׽ģʽѡ��
}

/*******************************************************************************
* Function Name  : T2CaptureSetup(UINT8 mode)
* Description    : CH559��ʱ������2 T2���Ų�׽���ܳ�ʼ��T2
                   UINT8 mode,���ز�׽ģʽѡ��
                   0:T2ex���½��ص���һ���½���
                   1:T2ex�������֮��
                   3:T2ex�������ص���һ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void T2CaptureSetup(UINT8 mode)
{
    RCLK = 0;
    TCLK = 0;
    CP_RL2 = 1;
    C_T2 = 0;
    T2MOD &= ~T2OE;                                                            //ʹ��T2���Ų�׽����
    T2MOD |= (mode << 2) | bT2_CAP1_EN;                                        //���ز�׽ģʽѡ��
}

/*******************************************************************************
* Function Name  : mTimer2Interrupt()
* Description    : CH559��ʱ������2��ʱ�������жϴ�������
*******************************************************************************/
void	mTimer2Interrupt( void ) interrupt INT_NO_TMR2 using 2                //timer2�жϷ������,ʹ�üĴ�����1
{
    mTimer2RunCTL( 0 );                                                       //�ض�ʱ��
#if T2EX_CAP
    if(EXF2)                                                                  //T2ex��ƽ�仯�ж��жϱ�־
    {
        MOSI1 = !MOSI1;                                                       //P2.1��ƽָʾ���
        Cap[FLAG++] = RCAP2;                                                  //T2EX
        EXF2 = 0;                                                             //���T2ex��׽�жϱ�־		
    }
#endif

#if T2_CAP
    if(CAP1F)                                                                  //T2��ƽ��׽�жϱ�־
    {
        Cap[FLAG++] = T2CAP1;                                                  //T2;	  	
        CAP1F = 0;                                                             //���T2��׽�жϱ�־
    }
#endif
	
#if TIMER
    if(TF2)
    {
        TF2 = 0;                                                               //��ն�ʱ��2����ж�	                                                      
        UDTR = !UDTR;                                                          //P0.2��ƽָʾ���
    }
#endif
    mTimer2RunCTL( 1 );                                                        //����ʱ��
}

main( ) 
{
    UINT8 i;
//  CfgFsys( );                                                                //CH559ʱ��ѡ������   
    mDelaymS(5);                                                               //�ȴ��ⲿ�����ȶ�	

    mInitSTDIO( );                                                             //����0,�������ڵ���
    printf("start ...\n"); 

    FLAG = 0;
    PORT_CFG |= bP0_OC;                                                        //P0.2����Ϊ��׼˫��IO
    P0_DIR |= bUDTR;
    P0_PU |= bUDTR;


    mTimer2ClkFsys( );                                                         //ʱ��ѡ��Fsys��ʱ����ʽ
#if TIMER
    mTimer2Setup(0);                                                           //��ʱ��������ʾ
    mTimer2Init(0x2000);                                                       //��ʱ������ֵ
    ET2 = 1;                                                                   //ʹ�ܶ�ʱ������2�ж�
    EA = 1;                                                                    //ʹ��ȫ���ж�
    mTimer2RunCTL( 1 );                                                        //������ʱ��
#endif

#if T2EX_CAP
    T2exCaptureSetup(1);                                                       //T2ex���Ų�׽��ʾ
    ET2 = 1;                                                                   //ʹ�ܶ�ʱ������2�ж�
    EA = 1;                                                                    //ʹ��ȫ���ж�
    mTimer2RunCTL( 1 );                                                        //������ʱ��
    T2EX = 0;                                                                  //ģ��T2ex���ŵ�ƽ�仯
    mDelayuS(500);
    T2EX = 1;
    mDelayuS(500);
    T2EX = 0;
    mDelayuS(500);
    T2EX = 1;
    mDelaymS(1);                                                                //ȷ�����һ�βɵ�����
    mTimer2RunCTL( 0 );                                                         //�رն�ʱ��
#endif

#if T2_CAP
    T2CaptureSetup(1);                                                         //T2���Ų�׽��ʾ
    ET2 = 1;                                                                   //ʹ�ܶ�ʱ������2�ж�
    EA = 1;                                                                    //ʹ��ȫ���ж�
    mTimer2RunCTL( 1 );                                                        //������ʱ��
    T2 = 0;	                                                                   //ģ��T2���ŵ�ƽ�仯
    mDelayuS(90);
    T2 = 1;
    mDelayuS(200);
    T2 = 0;
    mDelaymS(1);                                                                //ȷ�����һ�βɵ�����
    mTimer2RunCTL( 0 );                                                         //�رն�ʱ��
#endif

#if T2EX_CAP|T2_CAP                                                             //��׽���ݴ�ӡ
    EA = 0;                                                                     //ʹ��ȫ���ж�
    ET2 = 0;                                                                    //ʹ�ܶ�ʱ������2�ж�
    printf("FLAG %02X\n",(UINT16)FLAG);
    for(i=0;i<FLAG;i++)
    {
        printf("%04X  ",(UINT16)Cap[i]);
    }
#endif
    while(1);
}