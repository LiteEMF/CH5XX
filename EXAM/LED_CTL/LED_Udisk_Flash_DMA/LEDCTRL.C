
/********************************** (C) COPYRIGHT *********************************
* File Name          : LEDCTRL.C
* Author             : WCH
* Version            : V1.3
* Date               : 2016/06/24
* Description        : CH559��DMA��ʽʵ��LED���ƣ�����ʹ��U�̸�����������
                       ֧��U���Ȳ��
**********************************************************************************/

/*********************************ͷ�ļ�����**************************************/
#include <stdio.h>
#include <string.h>
#include "../../CH559.H"                                                    //CH559ͷ�ļ�
#include "../../DEBUG.H"                                                    //����ģ��
#include "../../DEBUG.C"
#include "ROOT_UDISKIF.H"                                                   //U�̽ӿ�ģ��
#include "ROOT_UDISKIF.C"

#pragma  NOAREGS

/*********************************ͷ�ļ�����**************************************/
#define   ScreenLength      1024                                            //��Ļ�ĳ��ȣ�������(bit)
#define   ScreenWidth       16                                              //��Ļ�Ŀ��ȣ�������(bit)
#define   UnicodeSize       2048                                            //ScreenLength*ScreenWidth/8 ��U���ж�ȡ���ַ���
#define   LargeUnicodeSize  1024                                            //�ֽ�
#define   SingleSendSize    128                                             //ScreenLength/8 LED��DMA���η��͵����ݳ���

#ifndef DEBUG                                                               //��ӡ������Ϣ
#define DEBUG 1
#endif

// #if LargeUnicodeSize>=UnicodeSize
// UINT8X LEDBuffer[CountUnicode]  _at_ 0x0000;                             //LED DMA���ͻ�����
// #else
UINT8X LEDBuffer[LargeUnicodeSize]  _at_ 0x0000;                            //LED DMA���ͻ�����
UINT8X LEDBuffer1[LargeUnicodeSize]  _at_ (0x0000+LargeUnicodeSize);        //LED DMA���ͻ�����
// #endif

sbit LA   = P2^0;
sbit LB   = P2^1;
sbit LC   = P2^2;
sbit LD   = P2^3;
sbit EN   = P2^4;
sbit STB  = P2^5;
#define EN_L( )       { EN  = 0; }                             
#define EN_H( )       { EN  = 1; }  
#define STB_L( )      { STB = 0; }                            
#define STB_H( )      { STB = 1; }
#define LINE0         { LD=0;LC=0;LB=0;LA=0; } 
#define LINE1         { LD=0;LC=0;LB=0;LA=1; }
#define LINE2         { LD=0;LC=0;LB=1;LA=0; }
#define LINE3         { LD=0;LC=0;LB=1;LA=1; }
#define LINE4         { LD=0;LC=1;LB=0;LA=0; }
#define LINE5         { LD=0;LC=1;LB=0;LA=1; }
#define LINE6         { LD=0;LC=1;LB=1;LA=0; }
#define LINE7         { LD=0;LC=1;LB=1;LA=1; }
#define LINE8         { LD=1;LC=0;LB=0;LA=0; }
#define LINE9         { LD=1;LC=0;LB=0;LA=1; }
#define LINE10        { LD=1;LC=0;LB=1;LA=0; }
#define LINE11        { LD=1;LC=0;LB=1;LA=1; }
#define LINE12        { LD=1;LC=1;LB=0;LA=0; }
#define LINE13        { LD=1;LC=1;LB=0;LA=1; }
#define LINE14        { LD=1;LC=1;LB=1;LA=0; }
#define LINE15        { LD=1;LC=1;LB=1;LA=1; }


/******************************SPI0 Flash**************************************/
/*******************************************************************************
* Function Name  : InitSPI_Host( void )
* Description    : SPI0����ģʽ��ʼ��
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void	InitSPI_Host( void )
{
    SPI0_SETUP &=~(bS0_MODE_SLV | bS0_BIT_ORDER) ;                            // ���ó�����ģʽ
    SPI0_CTRL = bS0_SCK_OE | bS0_MOSI_OE;                                     //����д��Ĭ�ϲ�����д���䣬���ʹ��bS0_DATA_DIR��
                                                                              //��ô�������ݺ��Զ�����һ���ֽڵ�ʱ�ӣ����ڿ��������շ�	
    P1_DIR |= (bMOSI | bSCK | bSCS| bPWM3 );			                            //bMOSI ��bSCK ��bSCS��Ϊ�������
    P1_DIR &= ~bMISO;
    SPI0_CK_SE = 0x02;	                                                      //��ƵΪ6M
//	SPI0_STAT = 0xFF;                                                         // ���жϱ�־
//	IE_SPI0 = 1;
}

/*******************************************************************************
* Function Name  : SPI0_MASTER_Trans
* Description    : ����һ�ֽ�����
* Input          : buffer -���������� 
* Output         : None
* Return         : None
*******************************************************************************/
void SPI0_MASTER_Trans( UINT8 dat )
{
    SPI0_DATA = dat;
    while( S0_FREE == 0 );                                                    //�ȴ����ݷ������
}

/*******************************************************************************
* Function Name  : SPI0_MASTER_Recv
* Description    : ����һ�ֽ�����
* Input          : None
* Output         : None
* Return         : ���յ�����
*******************************************************************************/
UINT8 SPI0_MASTER_Recv( void )
{
    SPI0_DATA = 0xFF;
    while(  S0_FREE == 0 );                                                   //�ȴ����ݻ���	
    return SPI0_DATA;
}

/*******************************************************************************
* Function Name  : Read_Status_Register
* Description    : ������ȡ״̬�Ĵ���,������״̬�Ĵ�����ֵ,Ϊ��֤�ٶȣ����Բ����ú���
* Input          : None
* Output         : None
* Return         : SPI0_DATA -�Ĵ���״ֵ̬
*******************************************************************************/
UINT8 Read_Status_Register( void )   
{   
    UINT8 byte = 0;
    SCS = 0 ;                                                                  //ʹ���豸 
    SPI0_DATA = 0x05;                                                          //���Ͷ�״̬�Ĵ��������� 
    while( S0_FREE == 0 );                                                     //�ȴ����ݷ������  
    SPI0_DATA = 0xFF; 
    while(  S0_FREE == 0 );                                                    //��ȡ״̬�Ĵ���  
    SCS = 1 ;                                                                  //��ֹ�豸    
    return SPI0_DATA;   
}
   
/*******************************************************************************
* Function Name  : Wait_Busy
* Description    : �ȴ�оƬ����(��ִ��Byte-Program, Sector-Erase, Block-Erase, Chip-Erase������)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Wait_Busy( void )   
{   
    while ((Read_Status_Register())&0x01 == 0x01 )                              //waste time until not busy   
          Read_Status_Register( );   
}

/*******************************************************************************
* Function Name  : WRSR
* Description    : ��״̬�Ĵ�����дһ���ֽ�
* Input          : byte -д�������
* Output         : None
* Return         : None
*******************************************************************************/
void WRSR( UINT8 byte )   
{   
    SCS = 0 ;                                                                  //ʹ���豸 
    SPI0_MASTER_Trans(0x01);                                                   //����д״̬�Ĵ���   
    SPI0_MASTER_Trans(byte);                                                   //�ı�Ĵ�����BPx����BPL (ֻ��2,3,4,5,7λ���Ը�д)   
    SCS = 1 ;                                                                  //��ֹ�豸  
}
 
/*******************************************************************************
* Function Name  : WREN
* Description    : дʹ��,ͬ����������ʹ��д״̬�Ĵ��� 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WREN( void )   
{   																	                                         //SCS�����س�ͻʹ��AIN3���
   SCS = 0 ;         
   SPI0_MASTER_Trans(0x06);                                                    //����WREN����  
   SCS = 1 ;             
}

/*******************************************************************************
* Function Name  : WREN_Check
* Description    : ����д����ǰWELλ�Ƿ�Ϊ1 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void WREN_Check( void )   
{   
    UINT8 byte;   
    byte = Read_Status_Register( );                                             //��ȡ״̬register   
    if ((byte&0x02) != 0x02)                                                    //���WELλ��λ   
    {   
        WREN( );                                                                //���δ��1������Ӧ����,��������дʹ�ò��� 
    }   
}

/*******************************************************************************
* Function Name  : Chip_Erase
* Description    : ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Chip_Erase( void )   
{                          
    WREN_Check();   
    SCS = 0 ;            
    SPI0_MASTER_Trans(0x60);                                                    //���� Chip Erase���� (60h or C7h)   
    SCS = 1 ;  
    Wait_Busy();   
}

/*******************************************************************************
* Function Name  : FlashRead
* Description    : ��ȡ��ʼ��ַ��SingleSendSize���ֽڵ�����.���ض�ȡ������,���ٶ� 
* Input          : Dst -Destination Address 000000H - 1FFFFFH
                   buffer ���ջ�������ʼ��ַ
* Output         : None
* Return         : None
*******************************************************************************/
void FlashRead(UINT32 Dst,PUINT8 buffer)    
{   
    UINT8 i;    
    SCS = 0 ;                                                                   //enable device  
    SPI0_DATA = 0x03;                                                           //read command 
    while( S0_FREE == 0 );                                                      //�ȴ����ݷ������ 
    SPI0_DATA = ((Dst & 0xFFFFFF) >> 16);                                       //send 3 address bytes
    while( S0_FREE == 0 );                                                      //�ȴ����ݷ������ 
    SPI0_DATA = ((Dst & 0xFFFF) >> 8);                                       
    while( S0_FREE == 0 );                                                      	
    SPI0_DATA = Dst & 0xFF;                                       
    while( S0_FREE == 0 ); 
    for(i=0;i<SingleSendSize;i++)
    {                                                                           //�ȴ����ݷ��� 
        SPI0_DATA = 0xFF;
        while(  S0_FREE == 0 );                                                  
        *(buffer+i) = SPI0_DATA;			
    }				 
    SCS = 1 ;                                                                   //disable device   
} 

/*******************************************************************************
* Function Name  : Byte_Program
* Description    : д����
* Input          : Dst  -Destination Address 000000H - 1FFFFFH
*                  byte -Ҫд�������
* Output         : None
* Return         : None
*******************************************************************************/
void Byte_Program(UINT32 Dst, UINT8 byte)
{
    WREN();
    SCS = 0 ;                                                                    //оƬʹ�� 
    SPI0_MASTER_Trans(0x02);                                                     //����д����ָ�� 
    SPI0_MASTER_Trans(((Dst & 0xFFFFFF) >> 16));                                 //����3�ֽڵ�ַ 
    SPI0_MASTER_Trans(((Dst & 0xFFFF) >> 8));
    SPI0_MASTER_Trans(Dst & 0xFF);
    SPI0_MASTER_Trans(byte);                                                     //����Ҫд������
    SCS = 1 ;
    Wait_Busy();
}

/*******************************************************************************
* Function Name  : CopyData2Flash(UINT16 Num,UINT16 Addr)
* Description    : �������ֶ�Ӧ���������Flash
* Input          : UINT16 Num ����д��Flash���ֽ���
                   UINT16 Addr ����д��Flash����ʼ��ַ
* Output         : None
* Return         : None
*******************************************************************************/
void CopyData2Flash(UINT16 Num,UINT16 Addr)
{
		UINT16 i;

		for(i=0;i<Num;i++)
		{
            Byte_Program( Addr+i ,LEDBuffer1[i] );                                  
		}
}	
/****************************SPI0 Flash END*************************************/

/*****************************��ѡ��IO����**************************************/
/*******************************************************************************
* Function Name  : InitLED(void)
* Description    : LED
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
void InitLED() 
{
    P2_DIR = 0xff;                                                    
    P3_DIR = 0xff;                                                      
    //P4_DIR = 0xff;                                                     
    PORT_CFG |= bP2_DRV | bP3_DRV;
    EN_H( );
    STB_L( );
    LED_CTRL  =  0x00|bLED_OUT_EN |bLED_BIT_ORDER;      
    LED_CK_SE=0x06;
}

/*******************************************************************************
* Function Name  : Showline(UINT8 LineNum) 
* Description    : LED
* Input          : UINT8 LineNum 
* Output         : None
* Return         : None
*******************************************************************************/
void Showline(UINT8 LineNum)                                     
{
    switch(LineNum)                                                     
	  {
         case  0: LINE0;break;
         case  1: LINE1;break;
         case  2: LINE2;break;
         case  3: LINE3;break;
         case  4: LINE4;break;
         case  5: LINE5;break;
         case  6: LINE6;break;
         case  7: LINE7;break;
         case  8: LINE8;break;
         case  9: LINE9;break;
         case 10:LINE10;break;
         case 11:LINE11;break;
         case 12:LINE12;break;
         case 13:LINE13;break;
         case 14:LINE14;break;
         case 15:LINE15;break;
         default:break;
    }
}
/******************************IO end*******************************************/

/*******************************************************************************
* Function Name  : ReadDisplayFile()
* Description    : ��ȡU������Ҫ��ʾ�ĺ����ַ�(ÿ�����ֶ�Ӧ2���ַ�)���洢��Flash
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ReadDisplayFile( )        
{
    UINT8 s,i;
    UINT16 len;
	
	WREN( );                                                                     //FLASHдʹ��
	WRSR( 0x00 );                                                                //д�Ĵ��� 
	Chip_Erase( );                                                               //FLASH��Ƭ����

    strcpy( mCmdParam.Open.mPathName, "/COMMAND.TXT" );                          //������Ҫ�򿪵��ļ��� 
    s = CH559FileOpen( );  
    if ( s == ERR_MISS_DIR || s == ERR_MISS_FILE )    
    {
        printf( "Miss File,Please Check the file name...\n" );
    }
    else    
    {
      mCmdParam.ByteLocate.mByteOffset = 602;                                  //����ƫ��ָ��
      s = CH559ByteLocate();
      mStopIfError( s );
      for(i=0;i<(UnicodeSize/LargeUnicodeSize);i++)                            //���ȴ���LargeUnicodeSize
      {                                                                        //ÿ�ζ�LargeUnicodeSize
          mCmdParam.ByteRead.mByteCount = LargeUnicodeSize;  
          mCmdParam.ByteRead.mByteBuffer = LEDBuffer1;                         //���ö��ļ���������ַ
          s = CH559ByteRead( );
          mStopIfError( s );
          printf("%02X  \n",(UINT16)i);
#if DEBUG
          for(len=0;len<LargeUnicodeSize;len++)
          {
              printf("%02X  ",(UINT16)LEDBuffer1[len]);
          }	
#endif					
          CopyData2Flash(LargeUnicodeSize,i*LargeUnicodeSize);                 //д��Flash         
      }	
      mCmdParam.ByteRead.mByteCount = UnicodeSize%LargeUnicodeSize;            //��ʣ���ֽ�  
      mCmdParam.ByteRead.mByteBuffer = LEDBuffer1;                             //���ö��ļ���������ַ
      s = CH559ByteRead( );
      mStopIfError( s );
#if DEBUG
      for(len=0;len<(UnicodeSize%LargeUnicodeSize);len++)
      {
          printf("%02X  ",(UINT16)LEDBuffer1[len]);
      }	
#endif			
      CopyData2Flash(UnicodeSize%LargeUnicodeSize,i*LargeUnicodeSize);         //д��Flash
  } 	
  mCmdParam.Close.mUpdateLen = 0;                                              //��ֹ�����ļ�����
  s = CH559FileClose( );                                                       //�ر��ļ�
  mStopIfError( s );		    
}	

/*******************************************************************************
* Function Name  : SendLeddata(UINT8 lineNum)
* Description    : DMA��ʽ����������
* Input          : UINT8 lineNum
* Output         : None
* Return         : None
*******************************************************************************/	
void SendLeddata(UINT8 lineNum)  
{
    UINT8 i,len;
    len = SingleSendSize/2;
    i=0;
	
    if(lineNum%2 == 0)
    {
        LED_DMA = LEDBuffer1; 
        LED_DMA_CN = len;                                                      //����DMA��˫�ֵģ������������2
        LED_CTRL |=  bLED_DMA_EN;  
        			
        FlashRead((UINT16)SingleSendSize*((lineNum+1)%ScreenWidth),LEDBuffer); //��FLASH��������	
    }
    else
    {
        LED_DMA = LEDBuffer; 
        LED_DMA_CN = len;                                                       //����DMA��˫�ֵģ������������2 
        LED_CTRL |=  bLED_DMA_EN;	  

        FlashRead((UINT16)SingleSendSize*((lineNum+1)%ScreenWidth),LEDBuffer1);//��FLASH��������				
    }
    while(LED_FIFO_CN||!(LED_STAT&bLED_FIFO_EMPTY));		
    LED_CTRL &= ~ bLED_DMA_EN ;
#if 0
    for(i=0;i<SingleSendSize;i++)
    {
        if(lineNum%2 == 0)
        {
            printf("%02X  ",(UINT16)LEDBuffer[i]);
        }
        else
        {
            printf("%02X  ",(UINT16)LEDBuffer1[i]);            
        }	
    }				
    printf("\n");	
#endif		
}


/*******************************************************************************
* Function Name  : Leddisplay(void)
* Description    : ��Ļ��ʾ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/	
void Leddisplay( )  
{
    UINT8 i;
#if 0 	
    printf("benin\n");	
#endif
    for(i=0;i<16;i++)
    { 		
		STB_H();
		mDelayuS(2);
		STB_L(); 
		SendLeddata(i);
		EN_L();		 
		Showline(i);
		mDelayuS(500);
        EN_H();			
    } 
}

/**********************************������***************************************/
void main( )  
{
    UINT8 s,i;
    mDelaymS(30);                                                              //�ϵ���ʱ,�ȴ��ڲ������ȶ�,�ؼ�          
    mInitSTDIO( );                                                             //Ϊ���ü����ͨ�����ڼ����ʾ����
    printf("Start LED contol....\n");
    InitUSB_Host( );
    InitLED( );
    InitSPI_Host( );
    CH559LibInit( );                                                            //��ʼ��CH559�������֧��U���ļ�
    FoundNewDev = 0;  
    UIF_DETECT=0;
    FlashRead(0,LEDBuffer1);                                                    //��FLASH������һ�е�����
    while(1)
    {
        if ( UIF_DETECT )                                                       //���U���豸���
        {
            UIF_DETECT = 0;                                                     //���жϱ�־
            s = AnalyzeRootHub( );                                              //����ROOT-HUB״̬
            if( s == ERR_USB_CONNECT )
            {
                FoundNewDev = 1;
            }
            if( FoundNewDev || s == ERR_USB_CONNECT )                           //���µ�USB�豸����
            {
                FoundNewDev = 0;
                mDelaymS( 200 );                                                //����USB�豸�ղ�����δ�ȶ�,�ʵȴ�USB�豸���ٺ���,������ζ���
                s = InitRootDevice( );                                          //��ʼ��USB�豸
                if( s == ERR_SUCCESS )
                {
                    // U�̲������̣�USB���߸�λ��U�����ӡ���ȡ�豸������������USB��ַ����ѡ�Ļ�ȡ������������֮�󵽴�˴�����CH559�ӳ���������ɺ�������
                    CH559DiskStatus = DISK_USB_ADDR;
                    for( i = 0; i != 10; i ++ )
                    {
                        s = CH559DiskReady( );
                        if ( s == ERR_SUCCESS )
                        {
                            break;
                        }
                        mDelaymS( 50 );
                    }
                    if( CH559DiskStatus >= DISK_MOUNTED )                       //U��׼����
                    {
                        printf("Read Command File....\n");
                        ReadDisplayFile();
                        printf("Finish Reading File\n");
                    }
                    else
                    {
                        printf( "U_Disk not ready ERR =%02X\n", (UINT16)s );
                    }
                 }
                 else
                 {
                     printf("U_Disk Init Failed,Please retry again\n");
                 }
             } 
             SetUsbSpeed( 1 );                                                  // Ĭ��Ϊȫ��	
        }
  		Leddisplay( );                                                          //��̬��ʾ����
    }
}


