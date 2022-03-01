#include "include.h"
#include <Servo.h>


			
uint16 ServoPwmDuty[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
uint16 ServoPwmDutySet[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
float ServoPwmDutyInc[8];		//为了速度控制，当PWM脉宽发生变化时，每2.5ms或20ms递增的PWM脉宽

bool ServoPwmDutyHaveChange = FALSE;	//脉宽有变化标志位

uint16 ServoTime = 2000;			//舵机从当前角度运动到指定角度的时间，也就是控制速度

Servo myservo[6];  // create servo object to control a servo

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
			if((i >= 0) && (i <= 6))
			{
				myservo[i - 1].writeMicroseconds(ServoPwmDuty[i]);
			}
			
		}
		
	}
}




void InitPWM(void)
{
	myservo[0].attach(2,500,2500);  // attaches the servo on pin 9 to the servo object
	myservo[1].attach(3,500,2500);  // attaches the servo on pin 9 to the servo object
	myservo[2].attach(4,500,2500);
	myservo[3].attach(5,500,2500);
	myservo[4].attach(6,500,2500);
	myservo[5].attach(7,500,2500);
}





