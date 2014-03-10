#include "flash.h"

#define STM32_FLASH_BASE 0x08000000 //stm32从0偏移到了0x08000000
#define STM32_FLASH_SIZE 65536 //c8t6是65536Byte=64KB=0x10000
#define STM32_SECTOR_SIZE 1024 //c8t6是1024Byte=0x400

uint32_t MyFlash_ReadWord(uint32_t addr)
{
	return *(uint32_t*)addr;
}

uint16_t MyFlash_ReadHalfWord(uint32_t addr)
{
	return *(uint16_t*)addr; 
}

uint8_t MyFlash_ReadByte(uint32_t addr)
{
	return *(uint8_t*)addr;
}

void MyFlash_Read(uint32_t addr,uint16_t *data,uint16_t length)
{
	uint16_t i;
	for(i=0;i<length;i++)
	{
		data[i] = MyFlash_ReadHalfWord(addr);//读取2个字节.
		addr += 2; //偏移2个字节.	
	}
}



//不检查的写入
//addr:起始地址
//data:数据指针
//length:半字(16位)数   
void MyFlash_Write_NoCheck(uint32_t addr,uint16_t *data,uint16_t length)   
{ 			 		 
	uint16_t i;
	for(i=0;i<length;i++)
	{
		FLASH_ProgramHalfWord(addr,data[i]);
	    addr += 2; //地址增加2.
	}  
} 

//检查擦出，先读取整个扇区，若有没擦出的块，先备份不需写入的块，再擦出整个扇区，最后一期写入
//addr:起始地址
//data:数据指针
//length:半字(16位)数 
void MyFlash_Write(uint32_t addr,uint16_t *data,uint16_t length)
{
	uint32_t secPos;	   //扇区地址
	uint16_t secOff;	   //扇区内偏移地址(16位字计算)
	uint16_t secRemain; //扇区内剩余地址(16位字计算)	   
	uint32_t offAddr;   //去掉0X08000000后的地址
	uint16_t buf[STM32_SECTOR_SIZE / 2]; //储存一个扇区的数据
 	uint16_t i;    
	
	if(addr < STM32_FLASH_BASE || (addr >= (STM32_FLASH_BASE + STM32_FLASH_SIZE))) return;//非法地址
	
	offAddr = addr - STM32_FLASH_BASE;		//实际偏移地址.
	secPos = offAddr / STM32_SECTOR_SIZE;			//扇区地址  0~65 for STM32F103C8T6
	secOff = (offAddr % STM32_SECTOR_SIZE) / 2;		//在扇区内的偏移(2个字节为基本单位.)，半字
	secRemain = STM32_SECTOR_SIZE / 2 - secOff;		//扇区剩余空间大小，半字
	if(length <= secRemain) secRemain = length;		//不大于该扇区范围
	
	FLASH_Unlock();						//解锁
	while(1) 
	{	
		MyFlash_Read(secPos * STM32_SECTOR_SIZE + STM32_FLASH_BASE,buf,STM32_SECTOR_SIZE / 2);//读出整个扇区的内容
		for(i=0;i<secRemain;i++)//校验数据
			if(buf[secOff + i] != 0XFFFF) break;//需要擦除  
		
		if(i<secRemain)//需要擦除
		{
			FLASH_ErasePage(secPos * STM32_SECTOR_SIZE + STM32_FLASH_BASE);//擦除这个扇区
			for(i=0;i<secRemain;i++)
				buf[secOff + i] = data[i];	//复制  
			MyFlash_Write_NoCheck(secPos * STM32_SECTOR_SIZE + STM32_FLASH_BASE,buf,STM32_SECTOR_SIZE / 2);//写入整个扇区  
		}
		else MyFlash_Write_NoCheck(addr,data,secRemain);//写已经擦除了的,直接写入扇区剩余区间. 
		
		if(length == secRemain) break;//写入结束了
		else//写入未结束
		{
			secPos++;				//扇区地址增1
			secOff = 0;				//偏移位置为0 	 
		   	data += secRemain;  	//指针偏移
			addr += secRemain;	//写地址偏移	   
		   	length -= secRemain;	//字节(16位)数递减
			if(length > (STM32_SECTOR_SIZE / 2)) secRemain = STM32_SECTOR_SIZE / 2;//下一个扇区还是写不完
			else secRemain = length;//下一个扇区可以写完了
		}	 
	}
	FLASH_Lock();//上锁
}
