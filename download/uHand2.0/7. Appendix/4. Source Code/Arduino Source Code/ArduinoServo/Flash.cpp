#include "include.h"
#include <SPI.h>



/************************************************
SPI初始化
入口参数: 无
出口参数: 无
************************************************/
void InitSpi(void)
{
	pinMode(SS,OUTPUT);
	
	SPI.begin ();
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
}


void InitFlash(void)
{
	InitSpi();
}

void CheckBusy()
{
	digitalWrite(SS, HIGH);
	digitalWrite(SS, LOW);
	SPI.transfer(SFC_RDSR);
	while(SPI.transfer(0) & 0x01); 
	digitalWrite(SS, HIGH);
}


void FlashRead(DWORD addr, DWORD size, BYTE *buffer) 
{
    digitalWrite(SS, HIGH);
    digitalWrite(SS, LOW);
    SPI.transfer(SFC_READ);
	SPI.transfer((BYTE)(addr>>16));
	SPI.transfer((BYTE)(addr>>8));
	SPI.transfer((BYTE)addr);
    for(int i = 0; i < size; i++)
    {
        buffer[i] = SPI.transfer(0);
    }
    digitalWrite(SS, HIGH);
    CheckBusy();
}

void FlashWrite(DWORD addr, DWORD size, BYTE *buffer) 
{
    digitalWrite(SS, HIGH);
    digitalWrite(SS, LOW);  
    SPI.transfer(SFC_WREN);
    digitalWrite(SS, HIGH);
    digitalWrite(SS, LOW);  
    SPI.transfer(SFC_PAGEPROG);
	SPI.transfer((BYTE)(addr>>16));
	SPI.transfer((BYTE)(addr>>8));
	SPI.transfer((BYTE)addr);
    for(int i = 0; i < size; i++)
    {
        SPI.transfer(byte(buffer[i]));
    }
    digitalWrite(SS, HIGH);
    CheckBusy();
}


void FlashEraseSector(DWORD addr)
{
    digitalWrite(SS, HIGH);
    digitalWrite(SS, LOW);  
    SPI.transfer(SFC_WREN);
    digitalWrite(SS, HIGH);
    digitalWrite(SS, LOW);  
    SPI.transfer(SFC_SECTOR_ERASE);
	SPI.transfer((BYTE)(addr>>16));
	SPI.transfer((BYTE)(addr>>8));
	SPI.transfer((BYTE)addr);
    digitalWrite(SS, HIGH);
    CheckBusy();
}



