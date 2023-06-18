/********************************** (C) COPYRIGHT *******************************
* File Name          : Encryption.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : ʹ����ֵ������ֵ������ʵ�ֵ�Ƭ��HEX����
                       ���磬ʹ��Encoded_0()��Encoded_1()�ֱ����0��1������ͬ�����������Ʒ���������Կ
*******************************************************************************/

#include "./DEBUG.C"
#include "./DEBUG.H"
#define  PI  3.141592657

#pragma  NOAREGS

UINT8  ID;
UINT8  IDX;

/*******************************************************************************
* Function Name  : EraseBlock(UNIT16 Addr)
* Description    : Dataflash���������
* Input          : UINT16 Addr
* Output         : None
* Return         : ״̬status
*******************************************************************************/ 
UINT8	EraseBlock( UINT16 Addr )
{
	ROM_ADDR = Addr;
	if ( ROM_STATUS & bROM_ADDR_OK ) {  // ������ַ��Ч
		ROM_CTRL = ROM_CMD_ERASE;
		return( ( ROM_STATUS ^ bROM_ADDR_OK ) & 0x7F );  // ����״̬,0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
	}
	else return( 0x40 );
}

/*******************************************************************************
* Function Name  : ProgWord( UINT16 Addr, UINT16 Data )
* Description    : Dataflashд�뺯��
* Input          : UNIT16 Addr,UINT16 Data
* Output         : None
* Return         : ״̬status
*******************************************************************************/ 
UINT8	ProgWord( UINT16 Addr, UINT16 Data )
{
	ROM_ADDR = Addr;
	ROM_DATA = Data;
	if ( ROM_STATUS & bROM_ADDR_OK ) {  // ������ַ��Ч
		ROM_CTRL = ROM_CMD_PROG;
		return( ( ROM_STATUS ^ bROM_ADDR_OK ) & 0x7F );  // ����״̬,0x00=success, 0x01=time out(bROM_CMD_TOUT), 0x02=unknown command(bROM_CMD_ERR)
	}
	else return( 0x40 );
}

/*******************************************************************************
* Function Name  : EncodedID_AndWR_ToDataflash()
* Description    : IDת�����ܺ�����ʹ�ò��������㣨�������ѡ���ĸ����ӵ��㷨������ID��
                   ���һ������IDX������Dataflash
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/ 
UINT8 EncodedID_AndWR_ToDataflash( )
{
	double i;
	UINT8 status;
	i=(double)ID/PI;
	IDX=(UINT8)i;
	
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;/*���밲ȫģʽ*/
	GLOBAL_CFG |= bDATA_WE;/*Dataflashдʹ��*/
	status=EraseBlock(0xF000);/*����1K��Dataflash*/
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
  GLOBAL_CFG &= ~ bDATA_WE;/*Dataflashдʹ�ܹر�*/
	SAFE_MOD = 0xFF;/*�˳���ȫģʽ*/
	
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;/*���밲ȫģʽ*/
	GLOBAL_CFG |= bDATA_WE;/*Dataflashдʹ��*/
	status=ProgWord( 0xF000,(UINT16)IDX);/*���������д��Dataflash*/
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;
	GLOBAL_CFG &= ~ bDATA_WE;/*Dataflashдʹ�ܹر�*/
	SAFE_MOD = 0xFF;/*�˳���ȫģʽ*/
	
	return IDX;
}

/*******************************************************************************
* Function Name  : GetIDXFromDataflash()
* Description    : IDX��ȡ����
* Input          : None
* Output         : None
* Return         : IDX
*******************************************************************************/ 
UINT8 GetIDXFromDataflash()
{
	return (UINT16)*((PUINT8C)(0xF000));
}

/*******************************************************************************
* Function Name  : Encoded_0()
* Description    : ��ֵ0�������
* Input          : None
* Output         : None
* Return         : 0
*******************************************************************************/ 
UINT8 Encoded_0()
{
	return (GetIDXFromDataflash()-EncodedID_AndWR_ToDataflash());
}

/*******************************************************************************
* Function Name  : Encoded_1()
* Description    : ��ֵ1�������
* Input          : None
* Output         : None
* Return         : 1
*******************************************************************************/ 
UINT8 Encoded_1()
{
	return (GetIDXFromDataflash()-EncodedID_AndWR_ToDataflash()+1);
}

void main()
{
	UINT8 i;
	mDelaymS(50);
	mInitSTDIO( );
	printf("start...\n");                                                
  ID=CHIP_ID;/*��ȡоƬID*/ 

/*������*/ 	
// 	i=Encoded_0();
//  printf("0�����ֵ  %02X\n",(UINT16)i);  
// 	i=Encoded_1();
// 	printf("1�����ֵ  %02X\n",(UINT16)i);
	
	//���ܲ��֣�ʹ����ֵ�����forѭ��
	for(i= Encoded_0();i<10*Encoded_1();i=i+Encoded_1())
	{
		printf("%02X\n",(UINT16)i);
	}
	
	while(1);
}