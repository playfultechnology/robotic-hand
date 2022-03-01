#ifndef _PWM_H_
#define _PWM_H_



//extern uint16 ServoPwmDutySet[];

//extern bool ServoPwmDutyHaveChange;

void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time);
void ServoPwmDutyCompare(void);//脉宽变化比较及速度控制
void InitPWM(void);

#endif


