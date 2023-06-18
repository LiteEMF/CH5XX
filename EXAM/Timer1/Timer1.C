/********************************** (C) COPYRIGHT *******************************
* File Name          : Timer1.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559 TIME1�ӿں���                				   
*******************************************************************************/

#include "..\DEBUG.C"                                                          //������Ϣ��ӡ
#include "..\DEBUG.H"

#pragma  NOAREGS

//CH559 Timer1ʱ��ѡ��   
//bTMR_CLKͬʱӰ��Timer0&1&2,ʹ��ʱҪע��                                                       
#define mTimer1ClkFsys( ) (T2MOD |= bTMR_CLK | bT1_CLK)                     //��ʱ��,ʱ��=Fsys
#define mTimer1Clk4DivFsys( ) (T2MOD &= ~bTMR_CLK;T2MOD |=  bT1_CLK)        //��ʱ��,ʱ��=Fsys/4
#define mTimer1Clk12DivFsys( ) (T2MOD &= ~(bTMR_CLK | bT1_CLK))             //��ʱ��,ʱ��=Fsys/12
#define mTimer1CountClk( ) (TMOD |= bT1_CT)                                 //������,T1���ŵ��½�����Ч

//CH559 Timer1 ��ʼ(SS=1)/����(SS=0)
#define mTimer1RunCTL( SS ) (TR1 = SS ? START : STOP)

/*******************************************************************************
* Function Name  : mTimer1ModSetup(UINT8 mode)
* Description    : CH559��ʱ������1ģʽ����
* Input          : UINT8 mode,Timer1ģʽѡ��
                   0��ģʽ0��13λ��ʱ����TL1�ĸ�3λ��Ч
                   1��ģʽ1��16λ��ʱ��
                   2��ģʽ2��8λ�Զ���װ��ʱ��
                   3��ģʽ3��ֹͣTimer1
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer1ModSetup(UINT8 mode)
{
    TMOD &= 0x0f;
    TMOD |= mode << 4; 
}

/*******************************************************************************
* Function Name  : mTimer1SetData(UINT16 dat)
* Description    : CH559Timer1 TH1��TL1��ֵ
* Input          : UINT16 dat;��ʱ����ֵ
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer1SetData(UINT16 dat)
{
  UINT16 tmp;
  tmp = 65536 - dat;
	TL1 = tmp & 0xff;
	TH1 = (tmp>>8) & 0xff;
}

/*******************************************************************************
* Function Name  : mTimer1Interrupt()
* Description    : CH559��ʱ������1��ʱ�������жϴ�������
*******************************************************************************/
void	mTimer1Interrupt( void ) interrupt INT_NO_TMR1 using 1                   //timer1�жϷ������,ʹ�üĴ�����1
{                                                                              //��ʽ3ʱ��ֹͣTimer1
    RXD_ = !RXD_;
//	mTimer1SetData(0x3737);                                                    //���Զ����ط�ʽ�����¸�TH1��TL1��ֵ        
}

main( ) 
{
//  CfgFsys( );                                                                //CH559ʱ��ѡ������ 
    mDelaymS(5);                                                               //�ȴ��ⲿ�����ȶ�
    mInitSTDIO( );                                                             //����0,�������ڵ���
    printf("start ...\n");

    mTimer1Clk12DivFsys( );                                                    //ʱ��ѡ��Fsys��ʱ����ʽ
    mTimer1ModSetup(2);	                                                       //��ʽ2���Զ�����8Ϊ��ʱ��                                                      
    mTimer1SetData(0x8080);                                                    //��ʱ������ֵ
    mTimer1RunCTL(1);                                                          //������ʱ��
    ET1 = 1;                                                                   //ʹ�ܶ�ʱ������1�ж�
    EA = 1;                                                                    //ʹ��ȫ���ж�
	    
    while(1);
}
