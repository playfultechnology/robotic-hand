#include "include.h"


static bool fUartRxComplete = FALSE;
static uint8 UartRxBuffer[50];
uint8 Uart1RxBuffer[50];

static bool UartBusy = FALSE;

uint8 frameIndexSumSum[256];


void InitUart1(void)
{
  //bitSet(UCSR0A,U2X0);
  bitSet(UCSR0B,RXCIE0);
  bitSet(UCSR0B,RXEN0); 					
  bitSet(UCSR0B,TXEN0); 					
  bitSet(UCSR0C,UCSZ01);
  bitSet(UCSR0C,UCSZ00);					
  UBRR0=(F_CPU/16/9600-1);//波特率9600
}



void Uart1SendData(BYTE dat)
{
	loop_until_bit_is_set(UCSR0A,UDRE0);
	UDR0=dat;
}

void UART1SendDataPacket(uint8 dat[],uint8 count)
{
	uint8 i;
	for(i = 0; i < count; i++)
	{
		Uart1SendData(dat[i]);
	}

}


ISR(USART_RX_vect) 
{
	uint8 i;
	uint8 rxBuf;

	static uint8 startCodeSum = 0;
	static bool fFrameStart = FALSE;
	static uint8 messageLength = 0;
	static uint8 messageLengthSum = 2;
	
    rxBuf=UDR0;
	if(!fFrameStart)
	{
		if(rxBuf == 0x55)
		{
			startCodeSum++;
			if(startCodeSum == 2)
			{
				startCodeSum = 0;
				fFrameStart = TRUE;
				messageLength = 1;
			}
		}
		else
		{

			fFrameStart = FALSE;
			messageLength = 0;

			startCodeSum = 0;
		}
		
	}
	if(fFrameStart)
	{
		Uart1RxBuffer[messageLength] = rxBuf;
		if(messageLength == 2)
		{
			messageLengthSum = Uart1RxBuffer[messageLength];
			if(messageLengthSum < 2)// || messageLengthSum > 30
			{
				messageLengthSum = 2;
				fFrameStart = FALSE;
				
			}
				
		}
		messageLength++;

		if(messageLength == messageLengthSum + 2) 
		{
			if(fUartRxComplete == FALSE)
			{
				fUartRxComplete = TRUE;
				for(i = 0;i < messageLength;i++)
				{
					UartRxBuffer[i] = Uart1RxBuffer[i];
				}
			}
			

			fFrameStart = FALSE;
		}
	}

}

void McuToPCSendData(uint8 cmd,uint8 prm1,uint8 prm2)
{
	uint8 dat[8];
	uint8 datlLen = 2;
	switch(cmd)
	{

//		case CMD_ACTION_DOWNLOAD:
//			datlLen = 2;
//			break;

		default:
			datlLen = 2;
			break;
	}

	dat[0] = 0x55;
	dat[1] = 0x55;
	dat[2] = datlLen;
	dat[3] = cmd;
	dat[4] = prm1;
	dat[5] = prm2;
	UART1SendDataPacket(dat,datlLen + 2);
}

static bool UartRxOK(void)
{
	if(fUartRxComplete)
	{
		fUartRxComplete = FALSE;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
void FlashEraseAll();
void SaveAct(uint8 fullActNum,uint8 frameIndexSum,uint8 frameIndex,uint8* pBuffer);
void TaskPCMsgHandle(void)
{

	uint16 i;
	uint8 cmd;
	uint8 id;
	uint8 servoCount;
	uint16 time;
	uint16 pos;
	uint16 times;
	uint8 fullActNum;
	if(UartRxOK())
	{
    LedFlip();
		cmd = UartRxBuffer[3];
 		switch(cmd)
 		{
 			case CMD_MULT_SERVO_MOVE:
				servoCount = UartRxBuffer[4];
				time = UartRxBuffer[5] + (UartRxBuffer[6]<<8);
				for(i = 0; i < servoCount; i++)
				{
					id =  UartRxBuffer[7 + i * 3];
					pos = UartRxBuffer[8 + i * 3] + (UartRxBuffer[9 + i * 3]<<8);
					ServoSetPluseAndTime(id,pos,time);
	
				}
 				break;
			
			case CMD_FULL_ACTION_RUN:
				fullActNum = UartRxBuffer[4];//动作组编号
				times = UartRxBuffer[5] + (UartRxBuffer[6]<<8);//运行次数				
				FullActRun(fullActNum,times);
				break;
				
			case CMD_FULL_ACTION_STOP:
				FullActStop();
				break;
				
			case CMD_FULL_ACTION_ERASE:
				FlashEraseAll();
				McuToPCSendData(CMD_FULL_ACTION_ERASE,0,0);
				break;

			case CMD_ACTION_DOWNLOAD:
				SaveAct(UartRxBuffer[4],UartRxBuffer[5],UartRxBuffer[6],UartRxBuffer + 7);
				McuToPCSendData(CMD_ACTION_DOWNLOAD,0,0);
				break;
 		}
	}
}
void SaveAct(uint8 fullActNum,uint8 frameIndexSum,uint8 frameIndex,uint8* pBuffer)
{
	uint8 i;
	
	
	if(frameIndex == 0)//下载之前先把这个动作组擦除
	{//一个动作组占16k大小，擦除一个扇区是4k，所以要擦4次
	
		for(i = 0;i < 4;i++)//ACT_SUB_FRAME_SIZE/4096 = 4
		{
			FlashEraseSector((MEM_ACT_FULL_BASE) + (fullActNum * ACT_FULL_SIZE) + (i * 4096));
		}
	}

	FlashWrite((MEM_ACT_FULL_BASE) + (fullActNum * ACT_FULL_SIZE) + (frameIndex * ACT_SUB_FRAME_SIZE)
		,ACT_SUB_FRAME_SIZE,pBuffer);
	
	if((frameIndex + 1) ==  frameIndexSum)
	{
		FlashRead(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
		frameIndexSumSum[fullActNum] = frameIndexSum;
		FlashEraseSector(MEM_FRAME_INDEX_SUM_BASE);
		FlashWrite(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
	}
}


void FlashEraseAll()
{
	uint16 i;
	
	for(i = 0;i <= 255;i++)
	{
		frameIndexSumSum[i] = 0;
	}
	FlashEraseSector(MEM_FRAME_INDEX_SUM_BASE);
	FlashWrite(MEM_FRAME_INDEX_SUM_BASE,256,frameIndexSumSum);
}

void InitMemory(void)
{
	uint8 i;
	uint8 logo[] = "LOBOT";
	uint8 datatemp[8];

	uint8 tt[4] = {0,1,2,3};
	uint8 ttt[4] = {5,5,5,5};
	
	FlashRead(MEM_LOBOT_LOGO_BASE,5,datatemp);
	for(i = 0; i < 5; i++)
	{
		if(logo[i] != datatemp[i])
		{
			FlashEraseSector(MEM_LOBOT_LOGO_BASE);
			FlashWrite(MEM_LOBOT_LOGO_BASE,5,logo);
			FlashEraseAll();
			break;
		}
	}
}





