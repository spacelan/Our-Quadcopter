#include "communication.h"
#include "usart.h"

enum GET_DATA_STATE
{
	NEED_AA = 0,
	NEED_55,
	NEED_TYPE,
	NEED_DATA
};

//DATA_TYPE中各种数据的长度，单位字节
uint8_t dataLength[] = {0,16,6,6};

/*********************************************************************
*Function: MyCOM_SendData
*Description: 发送数据，以数据包方式
*Description: 数据包格式 0xaa 0x55 datatype *data
*Description: 根据数据储存方式，直接传递指针发送，放弃移位构建数据buf的方式
*Input: const void *data 待发送的数据
*Input: uint8_t dataType 待发送数据的类型
*Output:
*Return:
*Others:
*Author: Spacelan
*Date: 2014-2-25
*********************************************************************/
void MyCOM_SendData(const void *data,uint8_t dataType)
{
	static uint8_t bufHead[] = {0xaa,0x55,0x00};
	uint8_t length;
	bufHead[2] = dataType;
	length = dataLength[dataType];
	MyUSART_Transmit(bufHead,3);
	MyUSART_Transmit(data,length);
}

//获取数据，传进来的数据指针应该指向一个连续的16字节空间
/*********************************************************************
*Function: MyCOM_GetData
*Description: 将数据打包发送
*Description: 数据包格式 0xaa 0x55 datatype *data
*Input: void *data
*Input: uint8_t *dataType
*Output:
*Return: 
*Others:
*Author: Spacelan
*Date: 2014-2-25
*********************************************************************/
void MyCOM_GetData(void *data,uint8_t *dataType)
{
	static uint8_t state = NEED_AA,byte = 0,length = 0;
	*dataType = DATA_TYPE_NONE;
	if(state == NEED_AA)
	{
		if(MyUSART_GetRxBufSize() < 1)
			return;
		MyUSART_Receive(&byte,1);
		if(byte != 0xaa)
			return;
		state = NEED_55;
	}
	if(state == NEED_55)
	{
		if(MyUSART_GetRxBufSize() < 1)
			return;
		MyUSART_Receive(&byte,1);
		if(byte != 0x55)
		{
			state = NEED_AA;
			return;
		}
		state = NEED_TYPE;		
	}
	if(state == NEED_TYPE)
	{
		if(MyUSART_GetRxBufSize() < 1)
			return;
		MyUSART_Receive(&byte,1);
		if(byte > 5 || byte == 0)
		{
			state = NEED_AA;
			return;
		}
		length = dataLength[byte];
		state = NEED_DATA;		
	}
	if(state == NEED_DATA)
	{
		if(MyUSART_GetRxBufSize() < length)
			return;
		MyUSART_Receive(data,length);
		*dataType = byte;
		state = NEED_AA;
	}
}
