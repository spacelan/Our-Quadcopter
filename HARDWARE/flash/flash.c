#include "flash.h"

#define STM32_FLASH_BASE 0x08000000 //stm32��0ƫ�Ƶ���0x08000000
#define STM32_FLASH_SIZE 65536 //c8t6��65536Byte=64KB=0x10000
#define STM32_SECTOR_SIZE 1024 //c8t6��1024Byte=0x400

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
		data[i] = MyFlash_ReadHalfWord(addr);//��ȡ2���ֽ�.
		addr += 2; //ƫ��2���ֽ�.	
	}
}



//������д��
//addr:��ʼ��ַ
//data:����ָ��
//length:����(16λ)��   
void MyFlash_Write_NoCheck(uint32_t addr,uint16_t *data,uint16_t length)   
{ 			 		 
	uint16_t i;
	for(i=0;i<length;i++)
	{
		FLASH_ProgramHalfWord(addr,data[i]);
	    addr += 2; //��ַ����2.
	}  
} 

//���������ȶ�ȡ��������������û�����Ŀ飬�ȱ��ݲ���д��Ŀ飬�ٲ����������������һ��д��
//addr:��ʼ��ַ
//data:����ָ��
//length:����(16λ)�� 
void MyFlash_Write(uint32_t addr,uint16_t *data,uint16_t length)
{
	uint32_t secPos;	   //������ַ
	uint16_t secOff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
	uint16_t secRemain; //������ʣ���ַ(16λ�ּ���)	   
	uint32_t offAddr;   //ȥ��0X08000000��ĵ�ַ
	uint16_t buf[STM32_SECTOR_SIZE / 2]; //����һ������������
 	uint16_t i;    
	
	if(addr < STM32_FLASH_BASE || (addr >= (STM32_FLASH_BASE + STM32_FLASH_SIZE))) return;//�Ƿ���ַ
	
	offAddr = addr - STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
	secPos = offAddr / STM32_SECTOR_SIZE;			//������ַ  0~65 for STM32F103C8T6
	secOff = (offAddr % STM32_SECTOR_SIZE) / 2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)������
	secRemain = STM32_SECTOR_SIZE / 2 - secOff;		//����ʣ��ռ��С������
	if(length <= secRemain) secRemain = length;		//�����ڸ�������Χ
	
	FLASH_Unlock();						//����
	while(1) 
	{	
		MyFlash_Read(secPos * STM32_SECTOR_SIZE + STM32_FLASH_BASE,buf,STM32_SECTOR_SIZE / 2);//������������������
		for(i=0;i<secRemain;i++)//У������
			if(buf[secOff + i] != 0XFFFF) break;//��Ҫ����  
		
		if(i<secRemain)//��Ҫ����
		{
			FLASH_ErasePage(secPos * STM32_SECTOR_SIZE + STM32_FLASH_BASE);//�����������
			for(i=0;i<secRemain;i++)
				buf[secOff + i] = data[i];	//����  
			MyFlash_Write_NoCheck(secPos * STM32_SECTOR_SIZE + STM32_FLASH_BASE,buf,STM32_SECTOR_SIZE / 2);//д����������  
		}
		else MyFlash_Write_NoCheck(addr,data,secRemain);//д�Ѿ������˵�,ֱ��д������ʣ������. 
		
		if(length == secRemain) break;//д�������
		else//д��δ����
		{
			secPos++;				//������ַ��1
			secOff = 0;				//ƫ��λ��Ϊ0 	 
		   	data += secRemain;  	//ָ��ƫ��
			addr += secRemain;	//д��ַƫ��	   
		   	length -= secRemain;	//�ֽ�(16λ)���ݼ�
			if(length > (STM32_SECTOR_SIZE / 2)) secRemain = STM32_SECTOR_SIZE / 2;//��һ����������д����
			else secRemain = length;//��һ����������д����
		}	 
	}
	FLASH_Lock();//����
}
