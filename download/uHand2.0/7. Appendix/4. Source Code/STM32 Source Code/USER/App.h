#ifndef _APP_H_
#define _APP_H_
#include "include.h"

#define LED 		PBout(9)
#define BUZZER 		PCout(13)
#define KEY 		PCin(0)

#define LED_ON		0
#define LED_OFF		1


extern u32 gSystemTickCount;	//系统从启动到现在的毫秒数

void InitDelay(u8 SYSCLK);
void DelayMs(u16 nms);
void DelayUs(u32 nus);

void InitADC(void);
void InitTimer2(void);
void InitUart2(void);
void InitLED(void);
void InitKey(void);
void InitBuzzer(void);
void Uart1SendData(BYTE dat);
uint16 GetBatteryVoltage(void);


void TaskRun(void);

#endif
