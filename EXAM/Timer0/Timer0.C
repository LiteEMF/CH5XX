/********************************** (C) COPYRIGHT *******************************
* File Name          : Timer0.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559 Timer0�ӿں���             				   
*******************************************************************************/

#include "..\DEBUG.C"                                                          //������Ϣ��ӡ
#include "..\DEBUG.H"

#pragma  NOAREGS

//CH559 Timer0ʱ��ѡ��   
//bTMR_CLKͬʱӰ��Timer0&1&2,ʹ��ʱҪע��                                                       
#define mTimer0ClkFsys( ) (T2MOD |= bTMR_CLK | bT0_CLK)                     //��ʱ��,ʱ��=Fsys
#define mTimer0Clk4DivFsys( ) (T2MOD &= ~bTMR_CLK;T2MOD |=  bT0_CLK)        //��ʱ��,ʱ��=Fsys/4
#define mTimer0Clk12DivFsys( ) (T2MOD &= ~(bTMR_CLK | bT0_CLK))             //��ʱ��,ʱ��=Fsys/12
#define mTimer0CountClk( ) (TMOD |= bT0_CT)                                 //������,T0���ŵ��½�����Ч

//CH559 Timer0 ��ʼ(SS=1)/����(SS=0)
#define mTimer0RunCTL( SS ) (TR0 = SS ? START : STOP)

/*******************************************************************************
* Function Name  : mTimer0ModSetup(UINT8 mode)
* Description    : CH559��ʱ������0ģʽ0����
* Input          : UINT8 mode,Timer0ģʽѡ��
                   0��ģʽ0��13λ��ʱ����TL0�ĸ�3λ��Ч
                   1��ģʽ1��16λ��ʱ��
                   2��ģʽ2��8λ�Զ���װ��ʱ��
                   3��ģʽ3������8λ��ʱ��
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer0ModSetup(UINT8 mode)
{
    TMOD &= 0xf0;
    TMOD |= mode; 
}

/*******************************************************************************
* Function Name  : mTimer0SetData(UINT16 dat)
* Description    : CH559Timer0 TH0��TL0��ֵ
* Input          : UINT16 dat;��ʱ����ֵ
* Output         : None
* Return         : None
*******************************************************************************/
void mTimer0SetData(UINT16 dat)
{
    UINT16 tmp;
    tmp = 65536 - dat;	
    TL0 = tmp & 0xff;
    TH0 = (tmp>>8) & 0xff;
}

/*******************************************************************************
* Function Name  : mTimer0Interrupt()
* Description    : CH559��ʱ������0��ʱ�������жϴ�������
*******************************************************************************/
void	mTimer0Interrupt( void ) interrupt INT_NO_TMR0 using 1                //timer0�жϷ������,ʹ�üĴ�����1
{                                                                             //��ʽ3ʱ��TH0ʹ��Timer1���ж���Դ
    CAP1 = !CAP1;
//  mTimer0SetData(0x2000)                                                    //���Զ����ط�ʽ�����¸�TH0��TL0��ֵ      
}

main( ) 
{
//  CfgFsys( );                                                               //CH559ʱ��ѡ������  
    mDelaymS(5);                                                              //�ȴ��ⲿ�����ȶ�	
    mInitSTDIO();                                                             //����0,�������ڵ���,Ĭ�ϲ�����57600bps
    printf("start ...\n");

    mTimer0ModSetup(2);	                                                      //��ʽ2���Զ�����8Ϊ��ʱ��
    mTimer0ClkFsys( );                                                        //ʱ��ѡ��Fsys��ʱ����ʽ
    mTimer0SetData(0x2323);                                                   //��ʱ������ֵ
    mTimer0RunCTL( 1 );				                                                //������ʱ��
//  printf("%02X  %02X",(UINT16)TH0,(UINT16)TL0);
    ET0 = 1;                                                                  //ʹ�ܶ�ʱ������0�ж�
    EA = 1;                                                                   //ʹ��ȫ���ж�
	    
    while(1);
}