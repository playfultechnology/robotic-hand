
#include "include.h"



int main(void)
{
	SystemInit(); 			 //ϵͳʱ�ӳ�ʼ��Ϊ72M	  SYSCLK_FREQ_72MHz
	InitDelay(72);	     //��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	InitPWM();
	InitTimer2();//���ڲ���100us�Ķ�ʱ�ж�
	InitUart1();//������PC�˽���ͨ��
	InitUart3();//���ģ��Ĵ���
	InitADC();
	InitLED();
	InitKey();
	InitBuzzer();
	InitPS2();//PS2��Ϸ�ֱ���������ʼ��
	InitFlash();
	InitMemory();
	InitBusServoCtrl();
	LED = LED_ON;
	BusServoCtrl(1,SERVO_MOVE_TIME_WRITE,500,1000);
	BusServoCtrl(2,SERVO_MOVE_TIME_WRITE,500,1000);
	BusServoCtrl(3,SERVO_MOVE_TIME_WRITE,500,1000);
	BusServoCtrl(4,SERVO_MOVE_TIME_WRITE,500,1000);
	BusServoCtrl(5,SERVO_MOVE_TIME_WRITE,500,1000);
	BusServoCtrl(6,SERVO_MOVE_TIME_WRITE,500,1000);
	while(1)
	{
		TaskRun();
	}
}

