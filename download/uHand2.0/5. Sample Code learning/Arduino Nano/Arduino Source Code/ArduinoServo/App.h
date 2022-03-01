#ifndef _APP_H_
#define _APP_H_

#define LED 		8
#define BUZZER 		9
#define KEY 		A7

#define ADC_BAT		A6		//电池电压的AD检测通道



extern u32 gSystemTickCount;	//系统从启动到现在的毫秒数
extern uint8 BuzzerState;

//void DelayMs(uint16 ms);
void InitPS2();
void InitADC(void);
void InitTimer2(void);
void InitUart2(void);
void Uart1SendData(BYTE dat);
uint16 GetBatteryVoltage(void);
void LedFlip();

void TaskRun(void);

#endif

