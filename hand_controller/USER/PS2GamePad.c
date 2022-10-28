/*****************************************************************************
 ** File			: ApiPS2GamePad.c
 ** Author			: Zheng23
 ** Date			: 			
 ** Function		: PS2��Ϸ�ֱ�����������
*****************************************************************************/
#include "include.h"


u16 Handkey;
u8 Comd[2]={0x01,0x42};	//��ʼ�����������
u8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
u16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_TRIANGLE,
    PSB_CIRCLE,
    PSB_CROSS,
    PSB_SQUARE
	};	//����ֵ�밴����



static void Delay(unsigned int time)
{
// 	time *=6;
// 	while(--time);
	DelayUs(time);
}

void InitPS2(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	PS2_SetInit();		 //�����ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�									  
}

//���ֱ���������
void PS2_Cmd(u8 CMD)
{
	volatile u16 ref=0x01;
	Data[1] = 0;
	for(ref=0x01;ref<0x0100;ref<<=1)
	{
		if(ref&CMD)
		{
			DO_H;                   //�����Ϊ����λ
		}
		else DO_L;

		Delay(10);
		CLK_L;
		Delay(40);
		CLK_H;
		if(DI)
			Data[1] = ref|Data[1];
		Delay(10);
	}
}
//�ж��Ƿ�Ϊ���ģʽ
//����ֵ��0�����ģʽ
//		  ����������ģʽ
u8 PS2_RedLight(void)
{
	CS_L;
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	CS_H;
	if( Data[1] == 0X73)   return 0 ;
	else return 1;

}
//��ȡ�ֱ�����
void PS2_ReadData(void)
{
	volatile u8 byte;
	volatile u16 ref;

	CS_L;
	Delay(10);
	PS2_Cmd(Comd[0]);  //��ʼ����
	PS2_Cmd(Comd[1]);  //��������
	for(byte=2;byte<9;byte++)          //��ʼ��������
	{
		for(ref=0x01;ref<0x100;ref<<=1)
		{

			CLK_L;
				Delay(50);
			CLK_H;
		      if(DI)
		      {
				Data[byte] = ref|Data[byte];
			  }
				Delay(20);
		     
		}
			Delay(40);
	}
	CS_H;
}

//�Զ�������PS2�����ݽ��д���      ֻ�����˰�������         Ĭ�������Ǻ��ģʽ  ֻ��һ����������ʱ
//����Ϊ0�� δ����Ϊ1

u16 LastHandkey = 0xFFFF;
u8 PS2_DataKey()
{
	u8 index;

	PS2_ClearData();
	PS2_ReadData();
	LastHandkey = Handkey;
	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
		return index+1;
	}
	return 0;          //û���κΰ�������
}

bool PS2_NewButtonState( u16 button )
{
  button = 0x0001u << ( button - 1 );  //�����button��ֵ�� �ð���������������bit��ֵ+1�� ���� PSB_SELECT ���ֵ �� 1�� �������е�λ��0λ�� ������ƣ�
  return ( ( ( LastHandkey ^ Handkey ) & button ) > 0 );  //���ϴεİ������ݺ���εİ������ݽ���������㣬����������β�ͬ�Ĳ��ֻ���1���͵õ���״̬�����˱仯�İ���
	                                                    //Ȼ������������Ҫ���İ������������㣬���������������˱仯����ô�������1�� ����0�����Է��ؾ���true
}

bool PS2_Button( u16 button )
{
  button = 0x0001u << ( button - 1 );  //�����button��ֵ�� �ð���������������bit��ֵ+1�� ���� PSB_SELECT ���ֵ �� 1�� �������е�λ��0λ�� ������ƣ�
  return ( ( (~Handkey) & button ) > 0 );  //�����������ӦλΪ0��û����Ϊ1�� ����������ȡ��֮�󣬾ͱ���˰���Ϊ1��û����Ϊ0
	                                         //����������Ҫ���İ����������㣬��������������£���Ӧλ����1��û���¾���0��������0�ȽϵĽ����
}

bool PS2_ButtonPressed( u16 button )
{
  return (PS2_NewButtonState( button ) && PS2_Button( button ));  //��������ס����������ǰ�����һ���µ�״̬����ô���ǰ����ձ�����
}

bool PS2_ButtonReleased( u16 button )
{
  return ( PS2_NewButtonState( button ) && !PS2_Button( button )); //����û����ס����������ǰ�����һ���µ�״̬����ô���ǰ����ձ��ɿ�
}

//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
u8 PS2_AnologData(u8 button)
{
	return Data[button];
}

//������ݻ�����
void PS2_ClearData()
{
	u8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
/******************************************************
Function:    void PS2_Vibration(u8 motor1, u8 motor2)
Description: �ֱ��𶯺�����
Calls:		 void PS2_Cmd(u8 CMD);
Input: motor1:�Ҳ�С�𶯵�� 0x00�أ�������
	   motor2:�����𶯵�� 0x40~0xFF �������ֵԽ�� ��Խ��
******************************************************/
void PS2_Vibration(u8 motor1, u8 motor2)
{
	CS_L;
	Delay(50);
    PS2_Cmd(0x01);  //��ʼ����
	PS2_Cmd(0x42);  //��������
	PS2_Cmd(0X00);
	PS2_Cmd(motor1);
	PS2_Cmd(motor2);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay(50);  
}
//short poll
void PS2_ShortPoll(void)
{
	CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	CS_H;
	Delay(50);	
}
//��������
void PS2_EnterConfing(void)
{
    CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01);
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay(50);
}
//����ģʽ����
void PS2_TurnOnAnalogMode(void)
{
	CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); //analog=0x01;digital=0x00  ������÷���ģʽ
	PS2_Cmd(0x03); //Ox03�������ã�������ͨ��������MODE������ģʽ��
				   //0xEE������������ã���ͨ��������MODE������ģʽ��
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	CS_H;
	Delay(50);
}
//������
void PS2_VibrationMode(void)
{
	CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01);
	CS_H;
	Delay(50);	
}
//��ɲ���������
void PS2_ExitConfing(void)
{
    CS_L;
	Delay(50);
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	CS_H;
	Delay(50);
}
//�ֱ����ó�ʼ��
void PS2_SetInit(void)
{
	DelayMs(100);
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		//��������ģʽ
	PS2_TurnOnAnalogMode();	//�����̵ơ�����ģʽ����ѡ���Ƿ񱣴�
	PS2_VibrationMode();	//������ģʽ
	PS2_ExitConfing();		//��ɲ���������
}
