#include "include.h"





static bool fUartRxComplete = FALSE;
static uint8 UartRxBuffer[260];
uint8 Uart1RxBuffer[260];

// static bool UartBusy = FALSE;

uint8  frameIndexSumSum[256];


void InitUart1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = 9600;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断

	USART_Cmd(USART1, ENABLE);                    //使能串口
	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
}

void Uart1SendData(BYTE dat)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
	USART1->DR = (u8) dat;
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
}

void UART1SendDataPacket(uint8 dat[],uint8 count)
{
	uint32 i;
	for(i = 0; i < count; i++)
	{
//		USART1_TransmitData(tx[i]);
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
		USART1->DR = dat[i];
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
	}
}


void USART1_IRQHandler(void)
{
	uint8 i;
	uint8 rxBuf;

	static uint8 startCodeSum = 0;
	static bool fFrameStart = FALSE;
	static uint8 messageLength = 0;
	static uint8 messageLengthSum = 2;
	

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {

        rxBuf = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
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
void FlashEraseAll(void);
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
		LED = !LED;
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
					BusServoCtrl(id,SERVO_MOVE_TIME_WRITE,pos,time);
				}				
 				break;
			
			case CMD_FULL_ACTION_RUN:
				fullActNum = UartRxBuffer[4];//动作组编号
				times = UartRxBuffer[5] + (UartRxBuffer[6]<<8);//运行次数
				McuToPCSendData(CMD_FULL_ACTION_RUN, 0, 0);
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


void FlashEraseAll(void)
{//将所有255个动作组的动作数设置为0，即代表将所有动作组擦除
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

	FlashRead(MEM_LOBOT_LOGO_BASE,5,datatemp);
	for(i = 0; i < 5; i++)
	{
		if(logo[i] != datatemp[i])
		{
		LED = LED_ON;
			//如果发现不相等的，则说明是新FLASH，需要初始化
			FlashEraseSector(MEM_LOBOT_LOGO_BASE);
			FlashWrite(MEM_LOBOT_LOGO_BASE,5,logo);
			FlashEraseAll();
			break;
		}
	}
	
}



