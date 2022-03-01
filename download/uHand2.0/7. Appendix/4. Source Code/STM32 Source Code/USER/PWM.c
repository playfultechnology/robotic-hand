#include "include.h"


			
uint16 ServoPwmDuty[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM������
uint16 ServoPwmDutySet[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM������
float  ServoPwmDutyInc[8];		//Ϊ���ٶȿ��ƣ���PWM�������仯ʱ��ÿ2.5ms��20ms������PWM����

bool ServoPwmDutyHaveChange = FALSE;	//�����б仯��־λ

uint16 ServoTime = 2000;			//����ӵ�ǰ�Ƕ��˶���ָ���Ƕȵ�ʱ�䣬Ҳ���ǿ����ٶ�


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


void ServoPwmDutyCompare(void)//����仯�Ƚϼ��ٶȿ���
{
	uint8 i;
	
	static uint16 ServoPwmDutyIncTimes;	//��Ҫ�����Ĵ���
	static bool ServoRunning = FALSE;	//���������ָ���ٶ��˶���ָ���������Ӧ��λ��
	if(ServoPwmDutyHaveChange)//ֹͣ�˶������������仯ʱ�Ž��м���      ServoRunning == FALSE && 
	{
		ServoPwmDutyHaveChange = FALSE;
		ServoPwmDutyIncTimes = ServoTime/20;	//��ÿ20ms����һ��ServoPwmDutyCompare()����ʱ�ô˾�
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
				ServoPwmDutyInc[i] /= ServoPwmDutyIncTimes;//ÿ�ε���������
			}
		}
		ServoRunning = TRUE;	//�����ʼ����
	}
	if(ServoRunning)
	{
		ServoPwmDutyIncTimes--;
		for(i=0;i<8;i++)
		{
			if(ServoPwmDutyIncTimes == 0)
			{		//���һ�ε�����ֱ�ӽ��趨ֵ������ǰֵ

				ServoPwmDuty[i] = ServoPwmDutySet[i];

				ServoRunning = FALSE;	//�����趨λ�ã����ֹͣ�˶�
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
	
	RCC->APB1ENR|=1<<1;//TIM3ʱ��ʹ��
 	TIM3->ARR=10000 - 1;  //�趨�������Զ���װֵ//�պ�1ms    
	TIM3->PSC=72 - 1;  //Ԥ��Ƶ��72,�õ�1Mhz�ļ���ʱ��
	//����������Ҫͬʱ���òſ���ʹ���ж�
	TIM3->DIER|=1<<0;   //��������ж�				
//	TIM3->DIER|=1<<6;   //�������ж�	   
	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	//�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}


void InitPWM(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	InitTimer3();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

//��PWM����ת�����Զ�װ�ؼĴ�����ֵ
void Timer3ARRValue(uint16 pwm)	
{
	TIM3->ARR = pwm + 1;
}


//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 		
	static uint16 i = 1;
	
	if(TIM3->SR&0X0001)//����ж�
	{
		switch(i)
		{
			case 1:
// 				SERVO0 = 1;	//PWM���ƽŸߵ�ƽ
				//����ʱ��0��ֵ������Pwm0Duty�����������жϣ��´��жϻ������һ��case���
				Timer3ARRValue(ServoPwmDuty[0]);
				break;
			case 2:
// 				SERVO0 = 0;	//PWM���ƽŵ͵�ƽ
				//�˼�������ֵ�������жϱ�ʾ��һ����ԪҪ��������Ŀ�ʼ
				Timer3ARRValue(2500-ServoPwmDuty[0]);	
				break;
			case 3:
				SERVO1 = 1;	
				Timer3ARRValue(ServoPwmDuty[1]);
				break;
			case 4:
				SERVO1 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[1]);	
				break;
			case 5:
				SERVO2 = 1;	
				Timer3ARRValue(ServoPwmDuty[2]);
				break;
			case 6:
				SERVO2 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[2]);	
				break;	
			case 7:
				SERVO3 = 1;	
				Timer3ARRValue(ServoPwmDuty[3]);
				break;
			case 8:
				SERVO3 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[3]);	
				break;	
			case 9:
				SERVO4 = 1;	
				Timer3ARRValue(ServoPwmDuty[4]);
				break;
			case 10:
				SERVO4 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[4]);	
				break;	
			case 11:
				SERVO5 = 1;	
				Timer3ARRValue(ServoPwmDuty[5]);
				break;
			case 12:
				SERVO5 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[5]);	
				break;
			case 13:
				SERVO6 = 1;	
				Timer3ARRValue(ServoPwmDuty[6]);
				break;
			case 14:
				SERVO6 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[6]);	
				break;
			case 15:
// 				SERVO7 = 1;	
				Timer3ARRValue(ServoPwmDuty[7]);
				break;
			case 16:
// 				SERVO7 = 0;	//PWM���ƽŵ͵�ƽ
				Timer3ARRValue(2500-ServoPwmDuty[7]);
				i = 0;	
				break;				 
		}
		i++;
	}				   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}

