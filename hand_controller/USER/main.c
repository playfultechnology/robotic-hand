
#include "include.h"

int main(void) {
	SystemInit();	//系统时钟初始化为72M	  SYSCLK_FREQ_72MHz
	InitDelay(72);	//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	InitPWM();
	InitTimer2();// Used to generate 100us timing interrupt
	InitUart1(); // Used for PC communication
	InitUart3(); //外接模块的串口
	InitADC();
	InitLED();
	InitKey();
	InitBuzzer();
	InitPS2();// PS2 Gamepad initialisation
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
	
	while(1) {
		TaskRun();
	}
}