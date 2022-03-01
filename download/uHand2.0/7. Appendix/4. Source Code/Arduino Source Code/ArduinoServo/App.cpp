#include "include.h"
//作者:深圳市乐幻索尔科技有限公司
//我们的店铺:lobot-zone.taobao.com



static bool UartBusy = FALSE;

u32 gSystemTickCount = 0;	//系统从启动到现在的毫秒数
uint16 ServoPwmDutyset[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //PWM脉冲宽度
uint8 BuzzerState = 0;
//uint16 Ps2TimeCount = 0;
uint8_t Mode = 0;
uint16 BatteryVoltage;

PS2X ps2X;                                     //实例化手柄类
void InitPS2()
{
	ps2X.config_gamepad(A2, A4, A3, A5);  //设置PS2接口 A2号IO为clock,A4号IO为command,A3号IO为attention,A5号IO为data
}
void InitTimer2(void)		//100us@12.000MHz
{
	TCCR2A=0;
	TCCR2B=_BV(CS21)|_BV(CS20);
	TIMSK2=_BV(TOIE2);
	TCNT2=206;//(256-206)/500000=100us
	sei();
}

void LedFlip()
{
	if(digitalRead(LED) == HIGH)
	{
		digitalWrite(LED, LOW);
	}
	else
	{
		digitalWrite(LED, HIGH);
	}
}

uint16 GetADCResult(void)
{
	return analogRead(ADC_BAT);
}

void CheckBatteryVoltage(void)
{
	uint8 i;
	uint32 v = 0;
	for(i = 0;i < 8;i++)
	{
		v += GetADCResult();
	}
	v >>= 3;
	
	v = v * 1875 / 128;//adc / 1024 * 5000 * 3(3代表放大3倍，因为采集电压时电阻分压了)
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
		t1++;
		if(t1 <= 2)
		{
			digitalWrite(BUZZER, LOW);//2.5KHz
		}
		else if(t1 <= 4)
		{
			digitalWrite(BUZZER, HIGH);//2.5KHz
		}
		if(t1 == 4)
		{
			t1 = 0;
		}
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

bool manual = FALSE;
ISR(TIMER2_OVF_vect)
{
	 static uint16 mytime = 0;
	static uint16 time = 0;
	static uint16 timeBattery = 0;
TCNT2=206; //定时器3中断  100us
	Buzzer();
	if(++time >= 10)
	{
		time = 0;
		gSystemTickCount++;
//		Ps2TimeCount++;
		 if (GetBatteryVoltage() < 5500) //小于5.5V报警
    {
      timeBattery++;
      if (timeBattery > 5000) //持续5秒
      {
        BuzzerState = 1;
      }
    }
    else
    {
      timeBattery = 0;
      if (manual == TRUE)
      {
        BuzzerState  = 1;
        mytime++;
        if (mytime > 80 && mytime < 130)
        {
          manual = FALSE;
        }
        if (mytime >= 210)
        {
          mytime = 0;
          manual = FALSE;
        }
      } else
      {
        if (mytime != 0)
        {
          mytime++;
          if (mytime >= 130 && mytime < 210)
          {
            manual = TRUE;
            BuzzerState   = 1;
            return;
          }
        }
        BuzzerState = 0;
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
	}
	
}

void ps2Handle() {                      //PS2 手柄 处理   
  static uint32_t Timer;                //定义静态变量Timer， 用于计时          
  if (Timer > millis())                 //Timer 大于 millis（）（运行的总毫秒数）时返回，//Timer 小于 运行总毫秒数时继续运行下面的语句
    return;
    
  ps2X.read_gamepad();                  //读取PS手柄按键数据

  if ( Mode == 0 )
  {
    if ( ps2X.Button( PSB_SELECT )   )
    {
      if ( ps2X.ButtonPressed( PSB_START ))
      {
        Mode = 1;
        manual = TRUE;
        FullActRun(0, 1);
        LedFlip();
        for (int i = 0 ; i < 8 ; i++)
        {
          ServoPwmDutyset[i] = 1500;
        }
        ServoSetPluseAndTime( 1, 1500, 1000 );
        ServoSetPluseAndTime( 2, 1500, 1000 );
        ServoSetPluseAndTime( 3, 1500, 1000 );
        ServoSetPluseAndTime( 4, 1500, 1000 );
        ServoSetPluseAndTime( 5, 1500, 1000 );
        ServoSetPluseAndTime( 6, 1500, 1000 );
        return;
      }
    }
    else {
      if (ps2X.ButtonPressed(PSB_START)) { //如果左侧向上按钮被按下�
        LedFlip();
        FullActRun(0, 1);
        Timer = millis() + 50;               //Timer 在 运行总毫秒数上加 50ms，50ms 后再次运行�
        return;       //返回，退出此函数
      }
      if (ps2X.ButtonPressed(PSB_PAD_UP)) { //如果左侧向上按钮被按下�
        LedFlip();
        FullActRun(1, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_PAD_DOWN)) {  //如果左侧向下按钮被按下
        LedFlip();
        FullActRun(2, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_PAD_LEFT)) {  //如果左侧向左按钮被按下�
        LedFlip();
        FullActRun(3, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_PAD_RIGHT)) { //如果左侧向右按钮被按下�
        LedFlip();
        FullActRun(4, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_GREEN)) {     //如果右侧绿色按钮（即右侧三角形按钮）被按下�
        LedFlip();
        FullActRun(5, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_BLUE)) {       //如果右侧蓝色按钮（即右侧交叉按钮）被按下
        LedFlip();
        FullActRun(6, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_PINK)) {       //如果右侧粉红色按钮（即右侧正方形按钮）被按下
        LedFlip();
        FullActRun(11, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_RED)) {       //如果右侧粉红色按钮（即右侧正方形按钮）被按下
        LedFlip();
        FullActRun(12, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_L1)) {        //如果左侧L1按钮被按下
        LedFlip();
        FullActRun(13, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_R1)) {        //如果左侧L2按钮被按下�
        LedFlip();
        FullActRun(14, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_L2)) {       //如果右侧R1按钮被按下
        LedFlip();
        FullActRun(15, 1);
        Timer = millis() + 50;
        return;
      }
      if (ps2X.ButtonPressed(PSB_R2)) {          //如果右侧R2按钮被按下
        LedFlip();
        FullActRun(16, 1);
        Timer = millis() + 50;
        return;
      }
      Timer = millis() + 50;
    }
  }

  if (Mode == 1)
  {
    if ( ps2X.Button( PSB_SELECT )   )
    {
      if ( ps2X.ButtonPressed( PSB_START ))
      {
        Mode = 0;
        manual = TRUE;
        LedFlip();
        return;
      }
    }
    else
    {
      if (ps2X.Button(PSB_PAD_LEFT)) { //如果左侧向上按钮被按下�
        ServoPwmDutyset[6] += 30;
        if (ServoPwmDutyset[6] > 2500)
          ServoPwmDutyset[6] = 2500;
        ServoSetPluseAndTime( 6, ServoPwmDutyset[6], 60 );
      }
      if (ps2X.Button(PSB_PAD_RIGHT)) { //如果左侧向上按钮被按下�
        ServoPwmDutyset[6] -= 30;
        if (ServoPwmDutyset[6] < 500)
          ServoPwmDutyset[6] = 500;
        ServoSetPluseAndTime( 6, ServoPwmDutyset[6], 60 );
      }
      if (ps2X.Button(PSB_PAD_UP)) { //如果左侧向上按钮被按下�
        ServoPwmDutyset[5] += 30;
        if (ServoPwmDutyset[5] > 2200)
          ServoPwmDutyset[5] = 2200;
        ServoSetPluseAndTime( 5, ServoPwmDutyset[5], 60 );
      }
      if (ps2X.Button(PSB_PAD_DOWN)) { //如果左侧向下按钮被按下�
        ServoPwmDutyset[5] -= 30;
        if (ServoPwmDutyset[5] < 900)
          ServoPwmDutyset[5] = 900;
        ServoSetPluseAndTime( 5, ServoPwmDutyset[5], 60 );
      }
      if (ps2X.Button(PSB_GREEN)) { //如果右侧向上按钮被按下�
        ServoPwmDutyset[4] -= 30;
        if (ServoPwmDutyset[4] > 2200)
          ServoPwmDutyset[4] = 2200;
        ServoSetPluseAndTime( 4, ServoPwmDutyset[4], 60 );
      }
      if (ps2X.Button(PSB_BLUE)) { //如果右侧向上按钮被按下�
        ServoPwmDutyset[4] += 30;
        if (ServoPwmDutyset[4] < 900)
          ServoPwmDutyset[4] = 900;
        ServoSetPluseAndTime( 4, ServoPwmDutyset[4], 60 );
      }
      if (ps2X.Button(PSB_PINK)) { //如果右侧向左按钮被按下�
        ServoPwmDutyset[3] -= 30;
        if (ServoPwmDutyset[3] > 2200)
          ServoPwmDutyset[3] = 2200;
        ServoSetPluseAndTime( 3, ServoPwmDutyset[3], 60 );
      }
      if (ps2X.Button(PSB_RED)) { //如果右侧向右按钮被按下�
        ServoPwmDutyset[3] += 30;
        if (ServoPwmDutyset[3] < 900)
          ServoPwmDutyset[3] = 900;
        ServoSetPluseAndTime( 3, ServoPwmDutyset[3], 60 );
      }
      if (ps2X.Button(PSB_L1)) { //如果左侧L1按钮被按下�
        ServoPwmDutyset[2] += 30;
        if (ServoPwmDutyset[2] > 2200)
          ServoPwmDutyset[2] = 2200;
        ServoSetPluseAndTime( 2, ServoPwmDutyset[2] , 60 );
      }
      if (ps2X.Button(PSB_L2)) { //如果右侧R1按钮被按下�
        ServoPwmDutyset[2] -= 30;
        if (ServoPwmDutyset[2] < 900)
          ServoPwmDutyset[2] = 900;
        ServoSetPluseAndTime( 2, ServoPwmDutyset[2], 60 );
      }
      if (ps2X.Button(PSB_R1)) { //如果左侧L2按钮被按下�
        ServoPwmDutyset[1] += 30;
        if (ServoPwmDutyset[1] > 2200)
          ServoPwmDutyset[1] = 2200;
        ServoSetPluseAndTime( 1, ServoPwmDutyset[1], 60 );
      }
      if (ps2X.Button(PSB_R2)) { //如果右侧R2按钮被按下�
        ServoPwmDutyset[1] -= 30;
        if (ServoPwmDutyset[1] < 900)
          ServoPwmDutyset[1] = 900;
        ServoSetPluseAndTime( 1, ServoPwmDutyset[1], 60 );
      }
      //摇杆控制
//      if (ps2X.Analog(PSS_RX) > 200) //摇杆的数值大于128则执行
//      {
//        ServoPwmDutyset[6] += 30;
//        if (ServoPwmDutyset[6] > 2500)
//          ServoPwmDutyset[6] = 2500;
//        ServoSetPluseAndTime( 6, ServoPwmDutyset[6], 60 );
//      }
//      if (ps2X.Analog(PSS_RX) < 50)
//      {
//        ServoPwmDutyset[6] -= 30;
//        if (ServoPwmDutyset[6] < 500)
//          ServoPwmDutyset[6] = 500;
//        ServoSetPluseAndTime( 6, ServoPwmDutyset[6], 60 );
//      }
//      if (ps2X.Analog(PSS_LY) > 200)
//      {
//        ServoPwmDutyset[5] += 30;
//        if (ServoPwmDutyset[5] > 2500)
//          ServoPwmDutyset[5] = 2500;
//        ServoSetPluseAndTime( 5, ServoPwmDutyset[5], 60 );
//      }
//      if (ps2X.Analog(PSS_LY) < 50)
//      {
//        ServoPwmDutyset[5] -= 30;
//        if (ServoPwmDutyset[5] < 500)
//          ServoPwmDutyset[5] = 500;
//        ServoSetPluseAndTime( 5, ServoPwmDutyset[5], 60 );
//      }
      if (ps2X.ButtonPressed(PSB_START)) { //如果START按钮被按下�
        LedFlip();
        for (int i = 0 ; i < 8 ; i++)
        {
          ServoPwmDutyset[i] = 1500;
        }
        ServoSetPluseAndTime( 1, 1500, 1000 );
        ServoSetPluseAndTime( 2, 1500, 1000 );
        ServoSetPluseAndTime( 3, 1500, 1000 );
        ServoSetPluseAndTime( 4, 1500, 1000 );
        ServoSetPluseAndTime( 5, 1500, 1000 );
        ServoSetPluseAndTime( 6, 1500, 1000 );
        return;
      }
    }
  }
  Timer = millis() + 50;
}

void TaskRun(void)
{
	static bool Ps2State = FALSE;
	uint8 PS2KeyValue;
  static uint16 keycount = 0;
	TaskTimeHandle();
	CheckBatteryVoltage();

	TaskPCMsgHandle();
	TaskRobotRun();

	if(analogRead(KEY) == 0)
	{
    keycount++;
	}
 else{
    if (keycount > 3000)
    {
      keycount = 0;
      FullActRun(100,0);
      return;
    }
    else if (keycount > 100)
    {
      keycount = 0;
      FullActRun(100,1);  
    }
	}

	ps2Handle();
	
}



