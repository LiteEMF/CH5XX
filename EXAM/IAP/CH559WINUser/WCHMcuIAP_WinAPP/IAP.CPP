/*
Module Name:Main.cpp
*****************************************
**  Copyright  (C)  WCH  2001-2015     **
**  Web:  http://wch.cn                **
*****************************************
Environment:
    user mode only,VC6.0
Revision History:
    9/10/2015: TECH
Descriptor:
    WCH MCU IAP�������룬֧��CH56X��CH55XоƬ
*/

#include "resource.h"
#include "Main.h"
#include "IAP.H"

#define MAX_FILE_SIZE 60*1024 //�����ļ���󳤶�

extern BOOL IsDownloading,IsDeviceChanged;
extern ULONG AfxDnInterface;
extern DnDevInforS AfxDnDev[]; //�豸�б�
extern HANDLE AfxDnDnDevH;
extern HWND AfxMainHwnd;

//HEX��ʽ�ļ�ת����BIN��ʽ
BOOL HexToBin( PVOID Hexbuf,     //HexbufΪHEX�ļ�������
			  ULONG iHexBufLen,  //iHexBufLenΪHEX����������
			  PVOID Binbuf,      //BinbufΪBIN�ļ�������
			  PULONG iBinLen );   //iBinLenΪת����BIN����Ч����
//ͨ��CMD_IAP_PROM(ISP�������)������д��FLASH��
BOOL WCHMcuIAP_PROM(HANDLE DnDevH,PUCHAR DnDataBuf,ULONG DnDataLen);
//CMD_IAP_VERIFY(ISPУ������)
BOOL WCHMcuIAP_VERIFY(HANDLE DnDevH,PUCHAR DnDataBuf,ULONG DnDataLen);
//CMD_IAP_END(ISP������־):
BOOL WCHMcuIAP_END(HANDLE DnDevH);
//CMD_IAP_ERASE��ͨ��CMD_IAP_END����������������������������ͬ
BOOL WCHMcuIAP_ERASE(HANDLE DnDevH,
					USHORT StartAddr); //��ַ������1024��λ��
//�رմ������ؽӿ�
VOID DnInterface_Close(HANDLE DnDevH);
//�򿪴������ؽӿ�
HANDLE DnInterface_Open();
//�������ļ���ȡ��ȫ����������������.����hex�ļ�Ҫת����bin�ļ�;����������4K���룬����4096�Ĳ�����д0XFF
BOOL ReadDataFromDnFile(PUCHAR DnDataBuf, //�����ļ����ݻ�����
						ULONG *DnDataLen);  //�����ļ������ֽ���
//CMD_IAP_VERIFY��ͨ��CMD_IAP_PROM����������������������������ͬ
BOOL WCHMcuIAP_PROM_VERIFY(HANDLE DnDevH,
						  UCHAR CmdCode,
						  USHORT StartAddr,
						  PUCHAR DnDataBuf,
						  ULONG DnDataLen);
//����IAP��������
BOOL WCHMcuIAP_SendCmd(HANDLE DnDevH,PUCHAR CmdPktBuf,PULONG oLen);
//����IAP�����
BOOL WCHMcuIAP_RecvCmdRet(HANDLE DnDevH,PUCHAR CmdPktBuf,PULONG iLen);



//ͨ��USB�ӿں�MCU��IAP�̼�������IMCU�û�����
DWORD WINAPI IAPFlashDownloadThread(LPVOID lpParameter)
{
	HANDLE DnDevH;
	UCHAR* buffer = NULL;												//���ݻ�����ָ��
	BOOL RetVal = FALSE;
	PUCHAR DnDataBuf = NULL; //�����ļ����ݻ�����
	ULONG DnDataLen = 0;      //�����ļ������ֽ���

	EnableWindow(GetDlgItem(AfxMainHwnd,IDC_Download),FALSE); //��ֹ�ظ��������
	IsDownloading = TRUE;

	if( AfxDnInterface == 0 ) //USB�ӿ�����
		DbgPrint("\r\n****��ʼͨ��USB�ӿڽ���IAP����:");
	else
		DbgPrint("\r\n****��ʼͨ�����ڽӿڽ���IAP����:");

	//1.��ȡ�����ļ��ڵ���������������,����HEX�ļ�����ת��BIN�ļ�
	DbgPrint("1.��ȡ�����ļ�����>>");
	DnDataLen = MAX_FILE_SIZE; //��󳤶�60K
	DnDataBuf = (PUCHAR)malloc(DnDataLen);
	ZeroMemory(DnDataBuf,DnDataLen);
    if( !ReadDataFromDnFile(DnDataBuf,    //�����ļ����ݻ�����
		&DnDataLen) )   //�����ļ������ֽ���	
	{
		DbgPrint("  �������ļ�ʧ��");
		goto ExitDownload;
	}
	else
		DbgPrint("  �Ѷ�ȡ��������%d�ֽ�",DnDataLen);
	
	//2.���豸���ؽӿ�	
	DbgPrint("2.�����ؽӿ�>>");;
	DnDevH = DnInterface_Open();
	if( DnDevH == INVALID_HANDLE_VALUE )
		goto ExitDownload;
			
	//��1�������Ͳ�������	
	DbgPrint("3.����FLASH>>");
	if( !WCHMcuIAP_ERASE(DnDevH,0) )
	{
		//isTrue = FALSE;
		DbgPrint("  ��������.");
		goto ExitDownload;
	}
	else
		DbgPrint("  OK");
	//��2�������ͱ����������б��	
	DbgPrint("4.дFLASH>>");
	if( !WCHMcuIAP_PROM(DnDevH,DnDataBuf,DnDataLen) )
	{
		//isTrue = FALSE;
		DbgPrint("  ʧ��.");
		goto ExitDownload;
	}
	else
		DbgPrint("  �ɹ�");
	//��3��������У������
	//У����������ͷ���
	DbgPrint("5.���ؼ���>>");
	if (!WCHMcuIAP_VERIFY(DnDevH,DnDataBuf,DnDataLen))
		
	{
		DbgPrint("  ����.");
		goto ExitDownload;
	}
	DbgPrint("  �ɹ�");	
	RetVal = TRUE;
	
ExitDownload:
	DbgPrint("6.���ؽ���>>");
	if( WCHMcuIAP_END(DnDevH) )
		DbgPrint("  ���óɹ�");
	else
		DbgPrint("  ����ʧ��");

	DbgPrint("7.�ر����ؽӿ�>>");
	DnInterface_Close(DnDevH);
	DnDevH = INVALID_HANDLE_VALUE;

	if( RetVal )
		DbgPrint("****IAP���سɹ�.");
	else
		DbgPrint("****IAP����ʧ��!\r\n   1.���ͨѶ�ӿ��Լ������Ƿ���ȷ;\r\n    2.MCU����û������IAP����;\r\n    3.MCU��û�н��뵽IAP����ģʽ;");	

	if( DnDataBuf != NULL )
	{		
		free(DnDataBuf);
		DnDataBuf = NULL;
	}

	EnableWindow(GetDlgItem(AfxMainHwnd,IDC_Download),TRUE); //�ָ����ذ�ť״̬
	IsDownloading = FALSE;
	if( IsDeviceChanged ) //�����ڼ����豸���
	{
		IsDeviceChanged = FALSE;
		PostMessage(GetDlgItem(AfxMainHwnd,IDC_ScanDev),BM_CLICK,0,0); //ˢ���豸�б�
	}
	return 0;	
}


//ͨ��CMD_IAP_PROM(ISP�������)������д��FLASH��
BOOL WCHMcuIAP_PROM(HANDLE DnDevH,PUCHAR DnDataBuf,ULONG DnDataLen)
{
	return WCHMcuIAP_PROM_VERIFY(DnDevH,CMD_IAP_PROM,0,DnDataBuf,DnDataLen);
}

//CMD_IAP_VERIFY(ISPУ������)
BOOL WCHMcuIAP_VERIFY(HANDLE DnDevH,PUCHAR DnDataBuf,ULONG DnDataLen)
{
	return WCHMcuIAP_PROM_VERIFY(DnDevH,CMD_IAP_VERIFY,0,DnDataBuf,DnDataLen);
}

//CMD_IAP_ERASE��ͨ��CMD_IAP_END����������������������������ͬ
BOOL WCHMcuIAP_ERASE(HANDLE DnDevH,
					USHORT StartAddr) //��ַ������1024��λ��
{
	USHORT FlashAddr;
	IAPCMDS IAPCmd = {0};
	ULONG  IAPCmdLen;
	UCHAR CmdDataLen;
	BOOL RetVal = FALSE;
	
	FlashAddr = StartAddr;	//ÿ���´�ʱ�Ļ�������ƫ�Ƶ�ַ,��0x00000000��ʼ
	CmdDataLen = sizeof(FlashAddr); //�޺�������
	
	//��������
	IAPCmd.program.cmd = CMD_IAP_ERASE;          //������
	IAPCmd.program.len = CmdDataLen;       //�޺�������
	*((PUSHORT)&IAPCmd.program.addr[0]) = FlashAddr; //дFLASH���׵�ַ
	
	IAPCmdLen = CmdDataLen + 2; //�����´����ݵ��ܳ���		
	
	//����IAP�����
	if( !WCHMcuIAP_SendCmd(DnDevH,(PUCHAR)&IAPCmd,&IAPCmdLen) )
	{
		DbgPrint("  WCHMcuIAP_ERASE send data failure.[%X>%dB].",FlashAddr,CmdDataLen);
		goto Exit_WCHMcuIAP_ERASE;
	}
	//���ܲ������2�ֽ�:
	IAPCmdLen = 2;
	memset(&IAPCmd,0xFF,5);
	if( !WCHMcuIAP_RecvCmdRet(DnDevH,(PUCHAR)&IAPCmd,&IAPCmdLen) )
	{
		DbgPrint("  WCHMcuIAP_ERASE read status failure.[%X>%dB].",FlashAddr,CmdDataLen);
		goto Exit_WCHMcuIAP_ERASE;
	}
	if( (IAPCmd.other.buf[0] != 0x00) || (IAPCmd.other.buf[1] != 0x00) )//״̬��ʾ����
	{
		DbgPrint("  WCHMcuIAP_ERASE failure.[%X.%X].",IAPCmd.other.buf[0],IAPCmd.other.buf[1]);
		goto Exit_WCHMcuIAP_ERASE;
	}	
	else
		RetVal = TRUE;

Exit_WCHMcuIAP_ERASE:
	return RetVal;
}

//CMD_IAP_END(ISP������־):
BOOL WCHMcuIAP_END(HANDLE DnDevH)
{
	IAPCMDS IAPCmd = {0};
	ULONG  IAPCmdLen;
	UCHAR CmdDataLen;
	BOOL RetVal = FALSE;
	
	CmdDataLen = sizeof(IAPCmd.end.status); //�޺�������
	
	//��������
	IAPCmd.end.cmd = CMD_IAP_END;    //������
	IAPCmd.end.len = CmdDataLen;     //��������Ϊstatus����
	IAPCmd.end.status[0] = IAPCmd.end.status[1] = 0;
	
	IAPCmdLen = 1 + 1+ CmdDataLen; //������+����+��������status
	
	//����IAP�����
	if( !WCHMcuIAP_SendCmd(DnDevH,(PUCHAR)&IAPCmd,&IAPCmdLen) )
	{
		DbgPrint("  WCHMcuIAP_END send data failure");
		goto Exit_WCHMcuIAP_END;
	}
	else 
		RetVal = TRUE;
	//�޷�������	

Exit_WCHMcuIAP_END:
	return RetVal;
}


//CMD_IAP_VERIFY��ͨ��CMD_IAP_PROM����������������������������ͬ
BOOL WCHMcuIAP_PROM_VERIFY(HANDLE DnDevH,
						  UCHAR CmdCode,
						  USHORT StartAddr,
						  PUCHAR DnDataBuf,
						  ULONG DnDataLen)
{
	USHORT FlashAddr;

	ULONG RemainLen = DnDataLen;
	IAPCMDS IAPCmd = {0};
	ULONG  IAPCmdLen;
	UCHAR CmdDataLen;
	
	FlashAddr = StartAddr;	//ÿ���´�ʱ�Ļ�������ƫ�Ƶ�ַ,��0x00000000��ʼ
	RemainLen = DnDataLen;
	
	while (RemainLen > 0)
	{		
		if (RemainLen > mMAX_CMDBUF_LENGTH )
			CmdDataLen = mMAX_CMDBUF_LENGTH;  //����
		else
			CmdDataLen = (UCHAR)RemainLen;           //��ͷ��
		//��������
		IAPCmd.program.cmd = CmdCode;          //������
		IAPCmd.program.len = CmdDataLen;       //д��FLASH���ֽ���
		*((PUSHORT)&IAPCmd.program.addr[0]) = FlashAddr; //дFLASH���׵�ַ			
		memcpy(IAPCmd.program.buf,&DnDataBuf[FlashAddr],CmdDataLen);	//д��FLASH������

		IAPCmdLen = CmdDataLen + 4; //�����´����ݵ��ܳ���		

		//����IAP�����
		if( !WCHMcuIAP_SendCmd(DnDevH,(PUCHAR)&IAPCmd,&IAPCmdLen) )
		{			
			DbgPrint("    ���������ʧ��.[%X>%dB].",FlashAddr,CmdDataLen);
			break;
		}
		//���ܲ������2�ֽ�:
		IAPCmdLen = 2;
		memset(&IAPCmd,0xFF,5);
		if( !WCHMcuIAP_RecvCmdRet(DnDevH,(PUCHAR)&IAPCmd,&IAPCmdLen) )
		{
			DbgPrint("  Read status failure.[%X>%dB].",FlashAddr,CmdDataLen);
			break;
		}
		if( (IAPCmd.other.buf[0] != 0x00) || (IAPCmd.other.buf[1] != 0x00) )//״̬��ʾ����
		{
			DbgPrint("  ״̬�뷵�ش���.[%X.%X].",IAPCmd.other.buf[0],IAPCmd.other.buf[1]);
			break;
		}

		RemainLen -= CmdDataLen;                   //ʣ�����ݳ���
		FlashAddr += CmdDataLen;                   //�´�дFLASHʱ���׵�ַ
	}	
	return (RemainLen == 0);
}


//�򿪴������ؽӿ�
HANDLE DnInterface_Open()
{
	ULONG InterfSel;
	BOOL RetVal = FALSE;
	HANDLE DnDnDevH = INVALID_HANDLE_VALUE;

	InterfSel = SendDlgItemMessage(AfxMainHwnd,IDC_DeviceList,CB_GETCURSEL,0,0);
	if( InterfSel == CB_ERR )
	{
		DbgPrint("    û��ѡ�������豸");
		return DnDnDevH;
	}	
	if( AfxDnInterface == 0 )//USB�ӿ�
	{
		//DbgPrint("��USB���ؽӿ�:");

		DnDnDevH = CreateFile( AfxDnDev[InterfSel].DevName, GENERIC_READ | GENERIC_WRITE,  // ���豸,��֧���ص�����
			FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );  // ����
		
		if (INVALID_HANDLE_VALUE == DnDnDevH)
		{
			DbgPrint("    %s��ʧ��",AfxDnDev[InterfSel].DevName);
			return DnDnDevH;
		}
		else
		{
			DbgPrint("    %s�򿪳ɹ�",AfxDnDev[InterfSel].DevName);
		}

		CH375SetExclusive((ULONG)DnDnDevH,1);
		CH375SetTimeout((ULONG)DnDnDevH,1000,1000);		
	}
	else //����
	{
		CHAR portname[64] = {0};
		DCB dcb;
		COMMTIMEOUTS TimeOuts;
		
		sprintf(portname, "\\\\.\\%s",AfxDnDev[InterfSel].DevName);		
		
		//�򿪴���
		DnDnDevH = CreateFile(portname,GENERIC_READ|GENERIC_WRITE,
			0,NULL,OPEN_EXISTING,
			NULL,
			NULL);
		if (INVALID_HANDLE_VALUE == DnDnDevH)
		{
			//���ڴ�ʧ��
			//DbgPrint("            ʧ��.");
			DbgPrint("    %s��ʧ��",AfxDnDev[InterfSel].DevName);
			ShowLastError("�򿪴���:");
			return DnDnDevH;
		}
		else
		{
			DbgPrint("    %s�򿪳ɹ�",AfxDnDev[InterfSel].DevName);
		}
		SetupComm(DnDnDevH,8192,8192);
		GetCommTimeouts(DnDnDevH,&TimeOuts);
		//�趨��ʱ
		TimeOuts.ReadIntervalTimeout = 0;
		TimeOuts.ReadTotalTimeoutMultiplier=1000;
		TimeOuts.ReadTotalTimeoutConstant=2000;
		TimeOuts.WriteTotalTimeoutMultiplier=50;
		TimeOuts.WriteTotalTimeoutConstant=500;
		SetCommTimeouts(DnDnDevH,&TimeOuts);//���ó�ʱ
		PurgeComm(DnDnDevH,PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);   //������롢���������
		
		GetCommState(DnDnDevH,&dcb);
		GetCommState(DnDnDevH,&dcb);
		dcb.BaudRate = IAP_UART_BAUDRATE;
		dcb.ByteSize = 8;					//��������λ
		dcb.Parity = NOPARITY;				//����У��λ
		dcb.StopBits = ONESTOPBIT;			//����ֹͣλ		
		SetCommState(DnDnDevH,&dcb);	
	}
	return DnDnDevH;
}

//�رմ������ؽӿ�
VOID DnInterface_Close(HANDLE DnDevH)
{
	//DbgPrint("�ر����ؽӿ�:");
	if( DnDevH != INVALID_HANDLE_VALUE )
	{
		if( AfxDnInterface == 0 ) //USB�ӿ�����
		{
		}
		else
		{
		}
		CloseHandle(DnDevH);
		DnDevH = INVALID_HANDLE_VALUE;		
		DbgPrint("    �ѹرա�");
	}
	else
		DbgPrint("    ���ؽӿ�δ�򿪣�����ر�");
}

//����IAP�����
BOOL WCHMcuIAP_RecvCmdRet(HANDLE DnDevH,PUCHAR CmdPktBuf,PULONG iLen)
{
	ULONG DataLen;
	BOOL RetVal = FALSE;

	DataLen = *iLen;
	if( AfxDnInterface == 0 ) //USB����
	{
		if( !CH375ReadData( (ULONG)DnDevH,CmdPktBuf,&DataLen) )
		{
			*iLen = 0;
			DbgPrint("    USB��ʧ��.");
		}
		else
		{
			if( DataLen == *iLen )
				RetVal = TRUE;
			else
				DbgPrint("    USB���������س��ȴ���(%d-%d).",DataLen,*iLen);
			*iLen = DataLen;
		}
	}
	else //��������
	{
		if( !ReadFile(DnDevH,CmdPktBuf,DataLen,&DataLen,NULL) )
		{
			*iLen = 0;
			DbgPrint("    ���ڶ�ʧ��.");
		}
		else
		{
			if( DataLen == *iLen )
				RetVal = TRUE;	
			else
				DbgPrint("    ���ڶ��������س��ȴ���(%d-%d).",DataLen,*iLen);

			*iLen = DataLen;
		}		
	}
	return RetVal;
}

//����IAP��������
BOOL WCHMcuIAP_SendCmd(HANDLE DnDevH,PUCHAR CmdPktBuf,PULONG oLen)
{
	BOOL RetVal = FALSE;
	ULONG DataLen = *oLen;

	if( AfxDnInterface == 0 ) //USB����
	{
		if( !CH375WriteData((ULONG)DnDevH,CmdPktBuf,&DataLen) )
		{		
			*oLen = 0;
			DbgPrint("    USBд����ʧ��");

		}
		else
		{
			if( DataLen != *oLen )
			{
				DbgPrint("    USBд��ʱ.(%d-%d) ",DataLen,*oLen);				
			}
			else
				RetVal = TRUE;
			*oLen = DataLen;
		}
	}
	else //��������
	{//�򴮿ڽ�USB������У�鹦�ܣ����������ݰ�����3���ֽڣ�0x57+0xAB+����+����͡�
		PUCHAR NewCmdBuf;
		UCHAR CheckSum = 0;
		ULONG i;

		PurgeComm(DnDevH,PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR);   //������롢���������

		DataLen += 3;
		NewCmdBuf = (PUCHAR)malloc(DataLen);
		ZeroMemory(NewCmdBuf,DataLen);
		for(i=0;i<*oLen;i++)
		{
			CheckSum += CmdPktBuf[i];
		}
		NewCmdBuf[0] = Uart_Sync_Head1; //����ǰ����1
		NewCmdBuf[1] = Uart_Sync_Head2; //����ǰ����2
		memcpy(&NewCmdBuf[2],CmdPktBuf,*oLen);
		NewCmdBuf[2+*oLen] = CheckSum; //����У���
		
		//ͨ������д����
		if( !WriteFile(DnDevH,NewCmdBuf,DataLen,&DataLen,NULL) ) //дʧ��
		{
			ShowLastError("WriteFile");
			DbgPrint("    ����дʧ��");
			*oLen = 0;
		}
		else
		{
			if( DataLen == (*oLen+3) ) //д�ɹ�
				RetVal = TRUE;
			else
			{
				DbgPrint("    ����д��ʱ(%d-%d)",DataLen,*oLen);
			}
			*oLen = DataLen;
		}
		free(NewCmdBuf);
	}
	return RetVal;
}

//�������ļ���ȡ��ȫ����������������.����hex�ļ�Ҫת����bin�ļ�;����������4K���룬����4096�Ĳ�����д0XFF
BOOL ReadDataFromDnFile(PUCHAR DnDataBuf, //�����ļ����ݻ�����
						ULONG *DnDataLen)  //�����ļ������ֽ���
{
	HANDLE FileH;
	ULONG FileSize;
	CHAR FileName[MAX_PATH] = "";
	PUCHAR FileBuf = NULL;
	BOOL RetVal = FALSE;
	
	GetDlgItemText(AfxMainHwnd,IDC_DownloadFile,FileName,sizeof(FileName)); //�����ļ���
	FileH = CreateFile(FileName,GENERIC_READ,FILE_SHARE_READ,NULL,OPEN_ALWAYS,FILE_ATTRIBUTE_ARCHIVE,NULL);
	if(FileH==INVALID_HANDLE_VALUE)
	{
		DbgPrint("    �޷��������ļ�%s",FileName);
		goto ExitReadDataFromDnFile;
	}
	FileSize = GetFileSize(FileH,NULL);
	if( FileSize == 0 ) //�ļ�Ϊ��
	{
		DbgPrint("    �հ������ļ�%s",FileName);
		goto ExitReadDataFromDnFile;
	}
	
	FileBuf = (PUCHAR)malloc(FileSize);		
	ZeroMemory(FileBuf,FileSize);

	if( !ReadFile(FileH,FileBuf,FileSize,&FileSize,NULL) )
	{
		ShowLastError("    �������ļ� ");
		goto ExitReadDataFromDnFile;
	}
	
	CharUpper(FileName); //�ļ���ȫ����д
	if( (strstr(FileName,".HEX") != NULL) ) //��HEX��ʽ�����ļ�,Ҫת����BIN�ļ�
	{
		DbgPrint("    HEX��ʽת����BIN��ʽ.");
		if (!HexToBin(FileBuf, FileSize, DnDataBuf, &FileSize))
		{
			DbgPrint("    ��Ч��HEX�ļ�.");
			goto ExitReadDataFromDnFile;
		}
		if (FileSize > *DnDataLen)
		{
			DbgPrint("    �����ļ����ܳ���%d�ֽ�",MAX_FILE_SIZE);
			goto ExitReadDataFromDnFile;
		}
		else
			*DnDataLen = FileSize;
	}
	else //BIN�ļ�
	{
		if (FileSize > MAX_FILE_SIZE)
		{
			DbgPrint("    �����ļ����ܳ���%d�ֽ�",MAX_FILE_SIZE);
			goto ExitReadDataFromDnFile;
		}
		
		memcpy(DnDataBuf,FileBuf,FileSize);  //������������
		*DnDataLen = FileSize;
	}	
		
	if( (*DnDataLen %4) ) //����������4���룬���㲿�ֲ�0
	{	
		ULONG FillLen;

		FillLen = 4-(*DnDataLen%4);		
		memset(&DnDataBuf[*DnDataLen],0,FillLen);
		*DnDataLen += FillLen; //������ĳ���
	}
	RetVal = TRUE;

ExitReadDataFromDnFile:
	if( FileH != INVALID_HANDLE_VALUE )
		CloseHandle(FileH);	
	if( FileBuf )
		free(FileBuf);

	return RetVal;
}

//�������ַ�ת��һ���ֽڵ�ʮ��������ֵ
BOOL Char2ToHex(PCHAR  pInChar,  //��ת����2���ַ�������
			   PUCHAR pOutChar) //ת�����ʮ��������ֵ
{
	UCHAR h,l;	

	h = pInChar[0]; //��4λ
	l = pInChar[1]; //��4λ
	if(l>='0' && l<='9')
	{
		l = l - '0';	
	}
	else if(l>='a' && l<='f')
	{
		l = l - 'a' + 0xa;
	}
	else if(l>='A' && l<='F')
	{
		l = l - 'A' + 0xa;
	}
	else
	{	
		return FALSE;
	}	
	if(h>='0' && h<='9')
	{
		h = h - '0';
	}
	else if(h>='a' && h<='f')
	{
		h = h - 'a' + 0xa;
	}
	else if(h>='A' &&h <='F')
	{
		h = h - 'A' + 0xa;
	}
	else
	{
		return FALSE;
	}	
	h <<= 4;
	h |= l;
	*pOutChar = h;
	return TRUE;
}

//HEX��ʽ�ļ�ת����BIN��ʽ
BOOL HexToBin( PVOID Hexbuf,     //HexbufΪHEX�ļ�������
			  ULONG iHexBufLen,  //iHexBufLenΪHEX����������
			  PVOID Binbuf,      //BinbufΪBIN�ļ�������
			  PULONG iBinLen )   //iBinLenΪת����BIN����Ч����
{
	PUCHAR hp;              // �ݴ�Hexbufָ��
	ULONG StartPos;         // ��¼�ڿ�ʼλ��
	UCHAR DataBuf[256];     // ��ʱ���ݴ�Ż���
	USHORT ExtAdr;          // ��չ��ַ
	USHORT SegAdr;          // ��չ�ε�ַ
	USHORT OfsAdr;          // ƫ�Ƶ�ַ
	ULONG WriteAdr;         // д���ַ

	__try
	{						// ������0XFF
		memset( Binbuf, 0xff, iHexBufLen);
	}
	__except( EXCEPTION_EXECUTE_HANDLER )
	{
		return FALSE;   // ���ʷ����쳣,ת��ʧ��
	}
	// ���HEX�ļ��������Ƿ�ɶ�
	if( IsBadReadPtr( Hexbuf, iHexBufLen ) )
		return FALSE;
	
	StartPos = 0;
	ExtAdr = 0;
	SegAdr = 0;
	hp = (PUCHAR)Hexbuf;   //�ݴ�HEX��������ʼָ��
	*iBinLen = 0; 

	//LINE ���ݸ�ʽ��0x3A(1���ֽ�) + �������ݳ���(2���ֽ�) + ����ƫ��ַַ(4���ֽ�) + ��������(1���ֽ�) + ��������(2���ֽ�Ϊ��λ) + �����(2���ֽ�) + 0x0D + 0x0A

	while( 1 )
	{
		if( (StartPos + 2) > iHexBufLen ) //�������ݵ����ٵð���3���ֽ� //if( (StartPos + 2) > iHexBufLen ) //�������ݵ����ٵð���3���ֽ�
			return FALSE;
		
		//ȡ������ͷ��־0x3A
		if( hp[StartPos] == ':' )
		{
			UCHAR i;
			ULONG Len = 0;          // ��Ч���ݳ���
			UCHAR CheckSum;         // У���		

			//ȡ��HEX�е����ݳ���,2���ֽ�
			Char2ToHex((char*)(hp + StartPos + 1),(PUCHAR)&Len); 
			if( (StartPos + Len*2 + 13) > iHexBufLen ) //һ��HEX�����ݳ�������13�ֽ�
				return FALSE;                          //��ֹ�Ƿ������ڴ�

			//LINE OFFSET3-4-5-6:��ʼ��ַ��4�ֽ�
			//LINE OFFSET7...��������,2�ı���			
			for( CheckSum=(UCHAR)Len, i=0; i < (Len+4); ++i ) //��־ͷ����ͼ����֮�������ֵ����ӣ��������ۼӺ���ӣ����Ϊ0x100�����������������ͳ���
			{// ��������ת������������
				Char2ToHex((char *)(hp+StartPos+3+i+i),(PUCHAR)(DataBuf+i)); //DataBuf��Ŵ���ʼ��ַ��ʼ��ת������ֵ������
				CheckSum += DataBuf[i];
			}
			if( CheckSum != 0 )   // У���ʧ��,�����HEX�ļ�
				return FALSE;

			//�������ͷ���
			switch ( DataBuf[2] ) 
			{
			case  0: // ���ݼ�¼
				{
					OfsAdr = DataBuf[0]*256 + DataBuf[1];
					WriteAdr = ExtAdr*65536 + SegAdr*16 + OfsAdr;

					if ((WriteAdr + Len) > iHexBufLen)
						return FALSE;

					memcpy(&(((PUCHAR)Binbuf)[WriteAdr]),&DataBuf[3],Len);
					StartPos += Len*2 + 13;         // ����һ���еĿ�ʼ

					if( (WriteAdr + Len) > *iBinLen )
						*iBinLen = WriteAdr + Len;  // �õ���������ַ
				}
				break;				
			case  2: // ��չ�ε�ַ��¼
				{
					SegAdr = DataBuf[3]*256 + DataBuf[4];
					StartPos += 17;
				}
				break;				
			case  4: // ��չ���Ե�ַ��¼
				{
					ExtAdr = DataBuf[3]*256 + DataBuf[4];
					StartPos += 17;
				}
				break;				
			case 5:  //��ʼ���Ե�ַ��¼
				{
					StartPos += 21;
				}
				break;				
			case 3: //��ʼ�ε�ַ��¼
				{
					StartPos += 21;
				}
				break;				
			case  1:  // HEX�ļ�������־
				return TRUE;				
			default:
				return FALSE;
			}
		}
		else
		{
			return FALSE;                 // �����ܳ���
		}
	}
}