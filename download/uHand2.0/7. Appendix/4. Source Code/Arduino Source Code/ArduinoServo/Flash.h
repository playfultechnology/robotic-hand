#ifndef _FLASH_H_
#define _FLASH_H_

#define  SS				10                   //SPI的SS脚,连接到Flash的CE


#define SFC_WREN        		0x06                    //串行Flash命令集
#define SFC_WRDI        		0x04
#define SFC_RDSR        		0x05
#define SFC_WRSR        		0x01
#define SFC_READ        		0x03
#define SFC_FASTREAD    		0x0B
#define SFC_RDID        		0xAB
#define SFC_PAGEPROG    		0x02
#define SFC_RDCR        		0xA1
#define SFC_WRCR        		0xF1
#define SFC_SECTORER    		0xD7
#define SFC_BLOCKER     		0xD8
#define SFC_SECTOR_ERASE		0x20
#define SFC_CHIPER      		0xC7


#define BUFFER_SIZE     1024                    //缓冲区大小
#define TEST_ADDR       0                       //Flash测试地址



void InitFlash(void);
void InitSpi(void);

void FlashEraseSector(DWORD addr);

void FlashRead(DWORD addr, DWORD size, BYTE *buffer);
void FlashWrite(DWORD addr, DWORD size, BYTE *buffer);



#endif


