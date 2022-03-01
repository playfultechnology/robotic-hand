#include "include.h"

/************************************************
SPI��ʼ��
��ڲ���: ��
���ڲ���: ��
************************************************/
u8 SPIx_ReadWriteByte(u8 TxData);
SPI_InitTypeDef  SPI_InitStructure;
void InitSpi(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  //SPI CS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���

	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����

	SPIx_ReadWriteByte(0xff);//��������

}


void InitFlash(void)
{
	InitSpi();
	DelayMs(150);
}

// /************************************************
// ʹ��SPI��ʽ��Flash�������ݽ���
// ��ڲ���:
//     dat : ׼��д�������
// ���ڲ���:
//     ��Flash�ж���������
// ************************************************/
// BYTE SpiShift(BYTE dat)
// {
//     SPDAT = dat;                                //����SPI����
//     while (!(SPSTAT & SPIF));                   //�ȴ�SPI���ݴ������
//     SPSTAT = SPIF | WCOL;                       //���SPI״̬
//     
//     return SPDAT;
// }

u8 SPIx_ReadWriteByte(u8 TxData)
{
	u8 retry=0;
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)  //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������
	retry=0;

	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);  //���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
	{
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1); //����ͨ��SPIx������յ�����
}


// /************************************************
// ���Flash��æ״̬
// ��ڲ���: ��
// ���ڲ���:
//     0 : Flash���ڿ���״̬
//     1 : Flash����æ״̬
// ************************************************/
// BOOL IsFlashBusy()
// {
//     BYTE dat;
//     
//     SS = 0;
//     SpiShift(SFC_RDSR);                         //���Ͷ�ȡ״̬����
//     dat = SpiShift(0);                          //��ȡ״̬
//     SS = 1;
//     
//     return (dat & 0x01);                        //״ֵ̬��Bit0��Ϊæ��־
// }

u8 SPI_Flash_ReadSR(void)
{
	u8 byte=0;
	SPI_FLASH_CS=0;                            //ʹ������
	SPIx_ReadWriteByte(W25X_ReadStatusReg);    //���Ͷ�ȡ״̬�Ĵ�������
	byte=SPIx_ReadWriteByte(0Xff);             //��ȡһ���ֽ�
	SPI_FLASH_CS=1;                            //ȡ��Ƭѡ
	return byte;
}

//�ȴ�����
void SPI_Flash_Wait_Busy(void)
{
	while((SPI_Flash_ReadSR()&0x01)==0x01);    // �ȴ�BUSYλ���
}
// /************************************************
// ʹ��Flashд����
// ��ڲ���: ��
// ���ڲ���: ��
// ************************************************/
// void FlashWriteEnable()
// {
//     while (IsFlashBusy());                      //Flashæ���
//     SS = 0;
//     SpiShift(SFC_WREN);                         //����дʹ������
//     SS = 1;
// }

//SPI_FLASHдʹ��
//��WEL��λ
void SPI_FLASH_Write_Enable(void)
{
	SPI_FLASH_CS=0;                            //ʹ������
	SPIx_ReadWriteByte(W25X_WriteEnable);      //����дʹ��
	SPI_FLASH_CS=1;                            //ȡ��Ƭѡ
}

// /************************************************
// ������ƬFlash
// ��ڲ���: ��
// ���ڲ���: ��
// ************************************************/
// void FlashErase()
// {
//     FlashWriteEnable();                     //ʹ��Flashд����
//     SS = 0;
//     SpiShift(SFC_CHIPER);                   //����Ƭ��������
//     SS = 1;
// }

/************************************************
�������� ������С4096,Flash��С������С��������Ϊ��λ����
��ڲ���: 
		addr
���ڲ���: ��
************************************************/
void FlashEraseSector(DWORD addr)
{
	SPI_FLASH_Write_Enable();                  //SET WEL
	SPI_Flash_Wait_Busy();
	SPI_FLASH_CS=0;                            //ʹ������
	SPIx_ReadWriteByte(W25X_SectorErase);      //������������ָ��
	SPIx_ReadWriteByte((u8)((addr)>>16));  //����24bit��ַ
	SPIx_ReadWriteByte((u8)((addr)>>8));
	SPIx_ReadWriteByte((u8)addr);
	SPI_FLASH_CS=1;                            //ȡ��Ƭѡ
	SPI_Flash_Wait_Busy();   				   //�ȴ��������
}

/************************************************
��Flash�ж�ȡ����
��ڲ���:
    addr   : ��ַ����
    size   : ���ݿ��С
    buffer : �����Flash�ж�ȡ������
���ڲ���:
    ��
************************************************/
void FlashRead(DWORD addr, DWORD size, BYTE *buffer)
{
    u16 i;
	SPI_FLASH_CS=0;                            //ʹ������
	SPIx_ReadWriteByte(W25X_ReadData);         //���Ͷ�ȡ����
	SPIx_ReadWriteByte((u8)((addr)>>16));  //����24bit��ַ
	SPIx_ReadWriteByte((u8)((addr)>>8));
	SPIx_ReadWriteByte((u8)addr);
	for(i=0; i<size; i++)
	{
		buffer[i]=SPIx_ReadWriteByte(0XFF);   //ѭ������
	}
	SPI_FLASH_CS=1;                            //ȡ��Ƭѡ
}


/************************************************
д���ݵ�Flash��
��ڲ���:
    addr   : ��ַ����
    size   : ���ݿ��С
    buffer : ������Ҫд��Flash������
���ڲ���: ��
************************************************/
void FlashWrite(DWORD addr, DWORD size, BYTE *buffer)
{
    u16 i;
	SPI_FLASH_Write_Enable();                  //SET WEL
	SPI_FLASH_CS=0;                            //ʹ������
	SPIx_ReadWriteByte(W25X_PageProgram);      //����дҳ����
	SPIx_ReadWriteByte((u8)((addr)>>16)); //����24bit��ַ
	SPIx_ReadWriteByte((u8)((addr)>>8));
	SPIx_ReadWriteByte((u8)addr);
	for(i=0; i<size; i++)SPIx_ReadWriteByte(buffer[i]); //ѭ��д��
	SPI_FLASH_CS=1;                            //ȡ��Ƭѡ
	SPI_Flash_Wait_Busy();					   //�ȴ�д�����
}

