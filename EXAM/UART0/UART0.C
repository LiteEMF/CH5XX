
/********************************** (C) COPYRIGHT *******************************
* File Name          : UART0.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559 ����0�Է�������ʾ
                     (1)������0�շ����ݣ������ʿɵ�;              				   
*******************************************************************************/

#include "..\DEBUG.C"                                                          //������Ϣ��ӡ
#include "..\DEBUG.H"

#pragma  NOAREGS
                                        
 UINT8 DAT,FLAG;

/*******************************************************************************
* Function Name  : CH559UART0Interrupt()
* Description    : CH559UART0�жϴ�������
*******************************************************************************/
void CH559UART0Interrupt( )  interrupt INT_NO_UART0 using 1                    //���Ź��жϷ������,ʹ�üĴ�����1
{
    if(TI)
    {
        TI = 0;                                                                //��շ����ж�                        
    }
    if(RI)
    {
        FLAG = 1;
        RI = 0;                                                                //��ս����ж�
        DAT = SBUF;
    }
}

main( ) 
{
    UINT8 i;
//  CfgFsys( );                                                                  //CH559ʱ��ѡ������    
    mDelaymS(5);                                                                 //�ȴ��ⲿ�����ȶ�		  
//  CH559UART0Alter();    
    FLAG = 0;                                                                    //��־λ���
    mInitSTDIO( );                                                               //����0��ʼ������
    ES = 1;                                                                      //����UART0�ж�
    EA = 1;                                                                      //���жϿ���
    while(1)
	  {
		    if(FLAG == 1)
		    {
			     SBUF = DAT;
			     FLAG = 0;
		    }
	  }
}
