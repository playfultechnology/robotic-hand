#include "include.h"
//作者:深圳市乐幻索尔科技有限公司
//我们的店铺:lobot-zone.taobao.com



#define ADC_BAT		13		//电池电压的AD检测通道
// static bool UartBusy = FALSE;

u32 gSystemTickCount = 0;	//系统从启动到现在的毫秒数

uint8 BuzzerState = 0;
uint16 Ps2TimeCount = 0;

uint16 BatteryVoltage;

static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数
//初始化延迟函数
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void InitDelay(u8 SYSCLK)
{
//	SysTick->CTRL&=0xfffffffb;//bit2清空,选择外部时钟  HCLK/8
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SYSCLK/8;
	fac_ms=(u16)fac_us*1000;
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864
void DelayMs(u16 nms)
{
	u32 temp;
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL=0x01 ;          //开始倒数
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器
}
//延时nus
//nus为要延时的us数.
void DelayUs(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; //时间加载
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL=0x01 ;      //开始倒数
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达
	SysTick->CTRL=0x00;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器
}





void InitLED(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PA端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PC.2 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void InitKey(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void InitBuzzer(void)
{

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //使能端口时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

}

void InitTimer2(void)		//100us
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = (10 - 1); //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =(720-1); //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	TIM_ITConfig(  //使能或者失能指定的TIM中断
	    TIM2, //TIM2
	    TIM_IT_Update  |  //TIM 中断源
	    TIM_IT_Trigger,   //TIM 触发中断源
	    ENABLE  //使能
	);

	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

void InitADC(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE);	   //使能ADC1通道时钟

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //72M/6=12,ADC最大时间不能超过14M
	//PA0/1/2/3 作为模拟通道输入引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	ADC_DeInit(ADC1);  //将外设 ADC1 的全部寄存器重设为缺省值

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器



	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1

	ADC_ResetCalibration(ADC1);	//重置指定的ADC1的校准寄存器

	while(ADC_GetResetCalibrationStatus(ADC1));	//获取ADC1重置校准寄存器的状态,设置状态则等待

	ADC_StartCalibration(ADC1);		//开始指定ADC1的校准状态

	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
}


uint16 GetADCResult(BYTE ch)
{
	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5);	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)); //等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


void CheckBatteryVoltage(void)
{
	uint8 i;
	uint32 v = 0;
	for(i = 0;i < 8;i++)
	{
		v += GetADCResult(ADC_BAT);
	}
	v >>= 3;
	
	v = v * 2475 / 1024;//adc / 4096 * 3300 * 3(3代表放大3倍，因为采集电压时电阻分压了)
	BatteryVoltage = v;

}

uint16 GetBatteryVoltage(void)
{//电压毫伏
	return BatteryVoltage;
}

void Buzzer(void)
{//放到100us的定时中断里面
	static bool fBuzzer = FALSE;
	static uint32 t1 = 0;
	static uint32 t2 = 0;
	if(fBuzzer)
	{
		if(++t1 >= 2)
		{
			t1 = 0;
			BUZZER = !BUZZER;//2.5KHz
		}
	}
	else
	{
		BUZZER = 0;
	}

	
	if(BuzzerState == 0)
	{
		fBuzzer = FALSE;
		t2 = 0;
	}
	else if(BuzzerState == 1)
	{
		t2++;
		if(t2 < 5000)
		{
			fBuzzer = TRUE;
		}
		else if(t2 < 10000)
		{
			fBuzzer = FALSE;
		}
		else
		{
			t2 = 0;
		}
	}
}

BOOL manual = FALSE;
void TIM2_IRQHandler(void)   //TIM2中断
{//定时器2中断  100us
	static uint32 time = 0;
	static uint16 timeBattery = 0;
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查指定的TIM中断发生与否:TIM 中断源
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);    //清除TIMx的中断待处理位:TIM 中断源
		
		
		Buzzer();
		if(++time >= 10)
		{
			time = 0;
			gSystemTickCount++;
			Ps2TimeCount++;
			if(GetBatteryVoltage() < 5500)//小于5.5V报警
			{
				timeBattery++;
				if(timeBattery > 5000)//持续5秒
				{
					BuzzerState = 1;
				}
			}
			else
			{
				timeBattery = 0;
				if(manual == FALSE)
				{
					BuzzerState = 0;
				}
			}
		}
	}
}

void TaskTimeHandle(void)
{
	static uint32 time = 10;
	static uint32 times = 0;
	if(gSystemTickCount > time)
	{
		time += 10;
		times++;
		if(times % 2 == 0)//20ms
		{
			ServoPwmDutyCompare();
		}
		if(times % 50 == 0)//500ms
		{
			CheckBatteryVoltage();
		}
	}
	
}

int16 BusServoPwmDutySet[8] = {500,500,500,500,500,500,500,500};
uint8 i;
void TaskRun(void)
{
	static bool Ps2State = FALSE;
	static uint8 mode = 0;
	uint16 ly, rx,ry;
	uint8 PS2KeyValue;
	static uint8 keycount = 0;
	TaskTimeHandle();
	
	
	TaskPCMsgHandle();
	TaskBLEMsgHandle();
	TaskRobotRun();

	if(KEY == 0)
	{
		DelayMs(60);
		{
			if(KEY == 0)
			{
				keycount++;
			}
			else
			{
				if (keycount > 20)
				{
					keycount = 0;
					FullActRun(100,0);
					return;
				}
				else
				{
					keycount = 0;
					LED = ~LED;
					FullActRun(100,1);	
				}
			}
		}
	}
	
	if(Ps2TimeCount > 45)
	{
		Ps2TimeCount = 0;
		PS2KeyValue = PS2_DataKey();
		if(mode == 1)
		{
			if( PS2_Button( PSB_SELECT ) & PS2_ButtonPressed( PSB_START ) )
			{
				mode = 0;
				Ps2State = 0;
				manual = TRUE;
				BuzzerState = 1;
				LED=~LED;
				DelayMs(80);
				manual = FALSE;
				DelayMs(50);
				manual = TRUE;
				BuzzerState = 1;
				DelayMs(80);
				manual = FALSE;
				LED=~LED;
			}
			else
			{
				if(PS2KeyValue && !PS2_Button(PSB_SELECT))
			{
				LED=~LED;
			}
						
				switch( PS2KeyValue )
				{
					//根据按下的按键，控制舵机转动
					case PSB_PAD_LEFT:
						ServoSetPluseAndTime( 6, ServoPwmDutySet[6] + 20, 50 );
						BusServoPwmDutySet[6] = BusServoPwmDutySet[6] + 10;
						if (BusServoPwmDutySet[6] > 2500)
							BusServoPwmDutySet[6] = 2500;
						BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
						break;
					case PSB_PAD_RIGHT:
						ServoSetPluseAndTime( 6, ServoPwmDutySet[6] - 20, 50 );
						BusServoPwmDutySet[6] = BusServoPwmDutySet[6] - 10;
						if (BusServoPwmDutySet[6] < 500)
							BusServoPwmDutySet[6] = 500;
						BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
						break;
					case PSB_PAD_UP:
						ServoSetPluseAndTime( 5, ServoPwmDutySet[5] - 20, 50 );
						BusServoPwmDutySet[5] = BusServoPwmDutySet[5] - 10;
						if (BusServoPwmDutySet[5] < 900)
							BusServoPwmDutySet[5] = 900;
						BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);
						break;
					case PSB_PAD_DOWN:
						ServoSetPluseAndTime( 5, ServoPwmDutySet[5] + 20, 50 );
						BusServoPwmDutySet[5] = BusServoPwmDutySet[5] + 10;
						if (BusServoPwmDutySet[5] > 2200)
							BusServoPwmDutySet[5] = 2200;
						BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);
						break;
					case PSB_L1:
						ServoSetPluseAndTime( 2, ServoPwmDutySet[2] + 20, 50 );
						BusServoPwmDutySet[2] = BusServoPwmDutySet[2] + 10;
						if (BusServoPwmDutySet[2] > 2200)
							BusServoPwmDutySet[2] = 2200;
						BusServoCtrl(2,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[2],50);
						break;
					case PSB_L2:
						ServoSetPluseAndTime( 2, ServoPwmDutySet[2] - 20, 50 );
						BusServoPwmDutySet[2] = BusServoPwmDutySet[2] - 10;
						if (BusServoPwmDutySet[2] < 900)
							BusServoPwmDutySet[2] = 900;
						BusServoCtrl(2,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[2],50);
						break;
					case PSB_TRIANGLE:
						ServoSetPluseAndTime( 4, ServoPwmDutySet[4] + 20, 50 );
						BusServoPwmDutySet[4] = BusServoPwmDutySet[4] + 10;
						if (BusServoPwmDutySet[4] > 2200)
							BusServoPwmDutySet[4] = 2200;
						BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
						break;
					case PSB_CROSS:
						ServoSetPluseAndTime( 4, ServoPwmDutySet[4] - 20, 50 );
						BusServoPwmDutySet[4] = BusServoPwmDutySet[4] - 10;
						if (BusServoPwmDutySet[4] < 900)
							BusServoPwmDutySet[4] = 900;
						BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
						break;
					case PSB_CIRCLE:
						ServoSetPluseAndTime( 3, ServoPwmDutySet[3] + 20, 50 );
						BusServoPwmDutySet[3] = BusServoPwmDutySet[3] + 10;
						if (BusServoPwmDutySet[3] > 2200)
							BusServoPwmDutySet[3] = 2200;
						BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
						break;
					case PSB_SQUARE:
						ServoSetPluseAndTime( 3, ServoPwmDutySet[3] - 20, 50 );
						BusServoPwmDutySet[3] = BusServoPwmDutySet[3] - 10;
						if (BusServoPwmDutySet[3] < 900)
							BusServoPwmDutySet[3] = 900;
						BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
						break;
					case PSB_R1:
						ServoSetPluseAndTime( 1, ServoPwmDutySet[1] + 20, 50 );
						BusServoPwmDutySet[1] = BusServoPwmDutySet[1] + 10;
						if (BusServoPwmDutySet[1] > 2200)
							BusServoPwmDutySet[1] = 2200;
						BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[1],50);
						break;
					case PSB_R2:
						ServoSetPluseAndTime( 1, ServoPwmDutySet[1] - 20, 50 );
						BusServoPwmDutySet[1] = BusServoPwmDutySet[1] - 10;
						if (BusServoPwmDutySet[1] < 900)
							BusServoPwmDutySet[1] = 900;
						BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[1],50);
						break;
					case PSB_START:
						ServoSetPluseAndTime( 1, 1500, 1000 );
						ServoSetPluseAndTime( 2, 1500, 1000 );
						ServoSetPluseAndTime( 3, 1500, 1000 );
						ServoSetPluseAndTime( 4, 1500, 1000 );
						ServoSetPluseAndTime( 5, 1500, 1000 );
						ServoSetPluseAndTime( 6, 1500, 1000 );
						for (i = 1; i < 7; i++)
						{
							BusServoPwmDutySet[i] = 500;
							BusServoCtrl(i,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[i],1000);
						}
						break;
								//摇杆控制
//					default:
//						if (PS2_AnologData(PSS_LX) == 255)
//						{
//							ServoSetPluseAndTime( 3, ServoPwmDutySet[3] + 30, 60 );
//							BusServoPwmDutySet[3] = BusServoPwmDutySet[3] + 10;
//							if (BusServoPwmDutySet[3] > 1000)
//								BusServoPwmDutySet[3] = 1000;
//							BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
//						}
//						if (PS2_AnologData(PSS_LX) == 0)
//						{
//							ServoSetPluseAndTime( 3, ServoPwmDutySet[3] - 30, 60 );
//							BusServoPwmDutySet[3] = BusServoPwmDutySet[3] - 10;
//							if (BusServoPwmDutySet[3] < 0)
//								BusServoPwmDutySet[3] = 0;
//							BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[3],50);
//						}
//						if (PS2_AnologData(PSS_RY) == 0)
//						{
//							ServoSetPluseAndTime( 4, ServoPwmDutySet[4] + 30, 60 );
//							BusServoPwmDutySet[4] = BusServoPwmDutySet[4] + 10;
//							if (BusServoPwmDutySet[4] > 1000)
//								BusServoPwmDutySet[4] = 1000;
//							BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
//						}
//						if (PS2_AnologData(PSS_RY) == 255)
//						{
//							ServoSetPluseAndTime( 4, ServoPwmDutySet[4] - 30, 60 );
//							BusServoPwmDutySet[4] = BusServoPwmDutySet[4] - 10;
//							if (BusServoPwmDutySet[4] < 0)
//								BusServoPwmDutySet[4] = 0;
//							BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[4],50);
//						}
//						if (PS2_AnologData(PSS_LY) == 0)
//						{
//							ServoSetPluseAndTime( 5, ServoPwmDutySet[5] - 30, 60 );
//							BusServoPwmDutySet[5] = BusServoPwmDutySet[5] - 10;
//							if (BusServoPwmDutySet[5] < 0)
//								BusServoPwmDutySet[5] = 0;
//							BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);
//						}
//						if (PS2_AnologData(PSS_LY) == 255)
//						{
//							ServoSetPluseAndTime( 5, ServoPwmDutySet[5] + 30, 60 );
//							BusServoPwmDutySet[5] = BusServoPwmDutySet[5] + 10;
//							if (BusServoPwmDutySet[5] > 1000)
//								BusServoPwmDutySet[5] = 1000;
//							BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[5],50);	
//						}
//						if (PS2_AnologData(PSS_RX) == 0)
//						{
//							ServoSetPluseAndTime( 6, ServoPwmDutySet[6] + 30, 60 );
//							BusServoPwmDutySet[6] = BusServoPwmDutySet[6] + 10;
//							if (BusServoPwmDutySet[6] > 1000)
//								BusServoPwmDutySet[6] = 1000;
//							BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
//						}
//						if (PS2_AnologData(PSS_RX) == 255)
//						{
//							ServoSetPluseAndTime( 6, ServoPwmDutySet[6] - 30, 60 );
//							BusServoPwmDutySet[6] = BusServoPwmDutySet[6] - 10;
//							if (BusServoPwmDutySet[6] < 0)
//								BusServoPwmDutySet[6] = 0;
//							BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,BusServoPwmDutySet[6],50);
//						}
				}
				PS2KeyValue = 0;
			}
		}
		else
		{	
			 if( PS2_Button( PSB_SELECT ) && PS2_ButtonPressed( PSB_START ) )  //检查是不是 SELECT按钮被按住，然后按下START按钮， 是的话，切换模式
        {
          mode = 1; //将模式变为1， 就单舵机模式
          FullActStop();  //停止动作组运行
          Ps2State = 0;  //清除动作组模式用到的标志。
          ServoSetPluseAndTime( 1, 1500, 1000 );  //将机械臂的舵机都转到1500的位置
          ServoSetPluseAndTime( 2, 1500, 1000 );
          ServoSetPluseAndTime( 3, 1500, 1000 );
          ServoSetPluseAndTime( 4, 1500, 1000 );
          ServoSetPluseAndTime( 5, 1500, 1000 );
          ServoSetPluseAndTime( 6, 1500, 1000 );
					manual = TRUE;
				BuzzerState = 1;
				LED=~LED;
				DelayMs(80);
				manual = FALSE;
				DelayMs(50);
				manual = TRUE;
				BuzzerState = 1;
				DelayMs(80);
				manual = FALSE;
				LED=~LED;
        }
				else
				{
			if(PS2KeyValue && !Ps2State && !PS2_Button(PSB_SELECT))
			{
				LED=~LED;
			}

			switch(PS2KeyValue)
			{
				case 0:
					if(Ps2State)
					{
						Ps2State = FALSE;
					}
					break;
				
				case PSB_START:
					if(!Ps2State)
					{
						FullActRun(0,1);
					}
					Ps2State = TRUE;
					break;
				
				case PSB_PAD_UP:
					if(!Ps2State)
					{
						FullActRun(1,1);
					}
					Ps2State = TRUE;
					break;
				
				case PSB_PAD_DOWN:
					if(!Ps2State)
					{
						FullActRun(2,1);
					}
					Ps2State = TRUE;
					break;
				
				case PSB_PAD_LEFT:
					if(!Ps2State)
					{
					FullActRun(3,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_PAD_RIGHT:
				if(!Ps2State)
				{
					FullActRun(4,1);
				}
				Ps2State = TRUE;
				break;

			case PSB_TRIANGLE:
				if(!Ps2State)
				{
					FullActRun(5,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_CROSS:
				if(!Ps2State)
				{
					FullActRun(6,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_SQUARE:
				if(!Ps2State)
				{
					FullActRun(11,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_CIRCLE:
				if(!Ps2State)
				{
					FullActRun(12,1);
				}
				Ps2State = TRUE;
				break;

			case PSB_L1:
				if(!Ps2State)
				{
					FullActRun(13,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_R1:
				if(!Ps2State)
				{
					FullActRun(14,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_L2:
				if(!Ps2State)
				{
					FullActRun(15,1);
				}
				Ps2State = TRUE;
				break;
				
			case PSB_R2:
				if(!Ps2State)
				{
					FullActRun(16,1);
				}
				Ps2State = TRUE;
				break;
		}
	  PS2KeyValue = 0;
	}
	}
}
}
