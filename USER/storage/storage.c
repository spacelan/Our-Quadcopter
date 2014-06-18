#include "storage.h"
#include "flash.h"

#define MY_STORAGE_DATA_NUM 7
#define MY_STORAGE_BLOCK_LENGTH 32 //字节
#define MY_STORAGE_BLOCK_NUM 32 //1024字节 划分成32个块，每块32字节
#define MY_STORAGE_ADDR_OFFSET 0x0800FC00
#define MY_STORAGE_HEAD_FLAG 0xABCD

uint8_t dataBuf[MY_STORAGE_BLOCK_LENGTH] = {0};
uint8_t StorageDataLength[32];
uint8_t currentBlock;
//将24字节的数据循环存放在1024字节的块中
//初始化时要找到数据在块中的哪个位置。。。。
//若找不到。。。擦出快，数据从0开始
void MyStorage_Init()
{
	for(currentBlock=0;currentBlock<MY_STORAGE_BLOCK_NUM;currentBlock++)
	{
		if(MyFlash_ReadHalfWord(MY_STORAGE_ADDR_OFFSET + (currentBlock * MY_STORAGE_BLOCK_LENGTH)) == MY_STORAGE_HEAD_FLAG)
			break;
	}
	if(currentBlock >= MY_STORAGE_BLOCK_NUM)
	{
		*(uint16_t*)dataBuf = MY_STORAGE_HEAD_FLAG;
		MyStorage_Program();
	}
	else
	{
		MyFlash_Read(MY_STORAGE_ADDR_OFFSET + (currentBlock * MY_STORAGE_BLOCK_LENGTH),(uint16_t*)dataBuf,MY_STORAGE_BLOCK_LENGTH / 2);
	}
	
	StorageDataLength[STORAGE_DATA_TYPE_HEAD] = 2;
	StorageDataLength[STORAGE_DATA_TYPE_ACCEL_BIAS] = 6;
	StorageDataLength[STORAGE_DATA_TYPE_GYRO_BIAS] = 6;
	StorageDataLength[STORAGE_DATA_TYPE_MOTOR_BASE] = 4;
	StorageDataLength[STORAGE_DATA_TYPE_P] = 2;
	StorageDataLength[STORAGE_DATA_TYPE_I] = 2;
	StorageDataLength[STORAGE_DATA_TYPE_D] = 2;
}

//将数据写入RAM
void MyStorage_Write(void *data,uint8_t type)
{
	uint32_t addr;
	for(addr=0;addr<StorageDataLength[type];addr+=2)
		*(uint16_t*)((uint8_t*)dataBuf + type + addr) = *(uint16_t*)((uint8_t*)data + addr);
}
//从RAM读取数据
void MyStorage_Read(void *data,uint8_t type)
{
	uint32_t addr;
	for(addr=0;addr<StorageDataLength[type];addr+=2)
		*(uint16_t*)((uint8_t*)data + addr) = *(uint16_t*)((uint8_t*)dataBuf + type + addr);
}
//将RAM中数据写入flash，在多个区块中循环写入，减少flash刷写次数，增加了的复杂度。。。。
void MyStorage_Program()
{
	FLASH_Unlock();
	if(currentBlock >= MY_STORAGE_BLOCK_NUM - 1)//最后一个块的地址
	{
		FLASH_ErasePage(MY_STORAGE_ADDR_OFFSET);
		currentBlock = 0;
	}
	else
	{
		FLASH_ProgramHalfWord(MY_STORAGE_ADDR_OFFSET + (currentBlock * MY_STORAGE_BLOCK_LENGTH),0);
		currentBlock++;
	}
	MyFlash_Write(MY_STORAGE_ADDR_OFFSET + (currentBlock * MY_STORAGE_BLOCK_LENGTH),(uint16_t*)dataBuf,MY_STORAGE_BLOCK_LENGTH / 2);
	FLASH_Lock();
}
