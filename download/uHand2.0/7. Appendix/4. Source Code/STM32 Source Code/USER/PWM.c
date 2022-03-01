#include "include.h"


			
uint16 ServoPwmDuty[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
uint16 ServoPwmDutySet[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
float  ServoPwmDutyInc[8];		//为了速度控制，当PWM脉宽发生变化时，每2.5ms或20ms递增的PWM脉宽

bool ServoPwmDutyHaveChange = FALSE;	//脉宽有变化标志位

uint16 ServoTime = 2000;			//舵机从当前角度运动到指定角度的时间，也就是控制速度


void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time)
{
	if(id >= 0 && id <= 7 && p >= 500 && p <= 2500)
	{
		if(time < 20)
			time = 20;
		if(time > 30000)
			time = 30000;
		if(id == 6)
		{
			if(p > 2500)
				p = 2500;
			else if(p < 500)
				p = 500;
			
		}
		else
		{
			if(p > 2200)
				p = 2200;
			else if(p < 900)
				p = 900;
		}
		ServoPwmDutySet[id] = p;
		ServoTime = time;
		ServoPwmDutyHaveChange = TRUE;
	}
}


void ServoPwmDutyCompare(void)//脉宽变化比较及速度控制
{
	uint8 i;
	
	static uint16 ServoPwmDutyIncTimes;	//需要递增的次数
	static bool ServoRunning = FALSE;	//舵机正在以指定速度运动到指定的脉宽对应的位置
	if(ServoPwmDutyHaveChange)//停止运动并且脉宽发生变化时才进行计算      ServoRunning == FALSE && 
	{
		ServoPwmDutyHaveChange = FALSE;
		ServoPwmDutyIncTimes = ServoTime/20;	//当每20ms调用一次ServoPwmDutyCompare()函数时用此句
		for(i=0;i<8;i++)
		{
			//if(ServoPwmDuty[i] != ServoPwmDutySet[i])
			{
				if(ServoPwmDutySet[i] > ServoPwmDuty[i])
				{
					ServoPwmDutyInc[i] = ServoPwmDutySet[i] - ServoPwmDuty[i];
					ServoPwmDutyInc[i] = -ServoPwmDutyInc[i];
				}
				else
				{
					ServoPwmDutyInc[i] = ServoPwmDuty[i] - ServoPwmDutySet[i];
					
				}
				ServoPwmDutyInc[i] /= ServoPwmDutyIncTimes;//每次递增的脉宽
			}
		}
		ServoRunning = TRUE;	//舵机开始动作
	}
	if(ServoRunning)
	{
		ServoPwmDutyIncTimes--;
		for(i=0;i<8;i++)
		{
			if(ServoPwmDutyIncTimes == 0)
			{		//最后一次递增就直接将设定值赋给当前值

				ServoPwmDuty[i] = ServoPwmDutySet[i];

				ServoRunning = FALSE;	//到达设定位置，舵机停止运动
			}
			else
			{

				ServoPwmDuty[i] = ServoPwmDutySet[i] + 
					(signed short int)(ServoPwmDutyInc[i] * ServoPwmDutyIncTimes);

			}
		}
		
	}
}



void InitTimer3(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC->APB1ENR|=1<<1;//TIM3时钟使能
 	TIM3->ARR=10000 - 1;  //设定计数器自动重装值//刚好1ms    
	TIM3->PSC=72 - 1;  //预分频器72,得到1Mhz的计数时钟
	//这两个东东要同时设置才可以使用中断
	TIM3->DIER|=1<<0;   //允许更新中断				
//	TIM3->DIER|=1<<6;   //允许触发中断	   
	TIM3->CR1|=0x01;    //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}


void InitPWM(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	InitTimer3();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

//将PWM脉宽转化成自动装载寄存器的值
void Timer3ARRValue(uint16 pwm)	
{
	TIM3->ARR = pwm + 1;
}


//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 		
	static uint16 i = 1;
	
	if(TIM3->SR&0X0001)//溢出中断
	{
		switch(i)
		{
			case 1:
// 				SERVO0 = 1;	//PWM控制脚高电平
				//给定时器0赋值，计数Pwm0Duty个脉冲后产生中断，下次中断会进入下一个case语句
				Timer3ARRValue(ServoPwmDuty[0]);
				break;
			case 2:
// 				SERVO0 = 0;	//PWM控制脚低电平
				//此计数器赋值产生的中断表示下一个单元要进行任务的开始
				Timer3ARRValue(2500-ServoPwmDuty[0]);	
				break;
			case 3:
				SERVO1 = 1;	
				Timer3ARRValue(ServoPwmDuty[1]);
				break;
			case 4:
				SERVO1 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[1]);	
				break;
			case 5:
				SERVO2 = 1;	
				Timer3ARRValue(ServoPwmDuty[2]);
				break;
			case 6:
				SERVO2 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[2]);	
				break;	
			case 7:
				SERVO3 = 1;	
				Timer3ARRValue(ServoPwmDuty[3]);
				break;
			case 8:
				SERVO3 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[3]);	
				break;	
			case 9:
				SERVO4 = 1;	
				Timer3ARRValue(ServoPwmDuty[4]);
				break;
			case 10:
				SERVO4 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[4]);	
				break;	
			case 11:
				SERVO5 = 1;	
				Timer3ARRValue(ServoPwmDuty[5]);
				break;
			case 12:
				SERVO5 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[5]);	
				break;
			case 13:
				SERVO6 = 1;	
				Timer3ARRValue(ServoPwmDuty[6]);
				break;
			case 14:
				SERVO6 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[6]);	
				break;
			case 15:
// 				SERVO7 = 1;	
				Timer3ARRValue(ServoPwmDuty[7]);
				break;
			case 16:
// 				SERVO7 = 0;	//PWM控制脚低电平
				Timer3ARRValue(2500-ServoPwmDuty[7]);
				i = 0;	
				break;				 
		}
		i++;
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}

