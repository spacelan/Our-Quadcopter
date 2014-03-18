#include "communication.h"
#include "usart.h"

enum GET_DATA_STATE
{
	NEED_AA = 0,
	NEED_55,
	NEED_TYPE,
	NEED_DATA
};

//DATA_TYPE�и������ݵĳ��ȣ���λ�ֽ�
uint8_t dataLength[129];
uint8_t cmd;
uint8_t isDataReady;

uint16_t math_crc16(uint16_t crc,const void * data,uint16_t len);

uint8_t check(const uint8_t *data,uint8_t length);

void MyCOM_Init()
{
	dataLength[DATA_TYPE_NONE] = 0;
	dataLength[DATA_TYPE_QUAT] = 16;
	dataLength[DATA_TYPE_ACCEL] = 6;
	dataLength[DATA_TYPE_GYRO] = 6;
	dataLength[DATA_TYPE_COMMAND] = 1;
	
	cmd = COMMAND_TYPE_SEND_QUAT;
	isDataReady = 0;
}

/*********************************************************************
*Function: MyCOM_SendData
*Description: �������ݣ������ݰ���ʽ
*Description: ���ݰ���ʽ 0xaa 0x55 datatype *data
*Description: �������ݴ��淽ʽ��ֱ�Ӵ���ָ�뷢�ͣ�������λ��������buf�ķ�ʽ
*Input: const void *data �����͵�����
*Input: uint8_t dataType ���������ݵ�����
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
	uint16_t crc;
	bufHead[2] = dataType;
	length = dataLength[dataType];
	crc = math_crc16(0,data,length);
	MyUSART_Transmit(bufHead,3);
	MyUSART_Transmit(data,length);
	MyUSART_Transmit((uint8_t*)&crc,2);
}

//��ȡ���ݣ�������������ָ��Ӧ��ָ��һ��������16�ֽڿռ�
/*********************************************************************
*Function: MyCOM_GetData
*Description: �����ݴ������
*Description: ���ݰ���ʽ 0xaa 0x55 datatype *data
*Input: void *data
*Input: uint8_t *dataType
*Output:
*Return: 
*Others:
*Author: Spacelan
*Date: 2014-2-25
*********************************************************************/
void MyCOM_GetData()
{
	static uint8_t state = NEED_AA,byte = 0,length = 0;
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
			if(byte == 0xaa) return;
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
		if(byte == 0)
		{
			if(byte == 0xaa)
				state = NEED_55;
			else
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
		switch(byte)
		{
		case DATA_TYPE_COMMAND:
			MyUSART_Receive(&cmd,1);
		}
		isDataReady |= (enum DATA_TYPE)byte;
		state = NEED_AA;
	}
}

bool MyCOM_GetCOMMAND(uint8_t *needCmd)
{
	*needCmd = cmd;
	if((isDataReady & DATA_TYPE_COMMAND) == 0) return false;
	isDataReady &= ~((uint8_t)DATA_TYPE_COMMAND);
	return true;
}

uint8_t check(const uint8_t *data,uint8_t length)
{
	uint8_t i,bcc = 0;
	for(i=0;i<length;i++)
		bcc ^= data[i];
	return bcc;
}

// ����CRC-16-CCITT��
// http://www.dzjs.net/html/qianrushixitong/2007/0530/2162.html
// crcΪ�ϴεĽ������ʼʱ��Ϊ0�����ڷֶμ��㡣
uint16_t math_crc16(uint16_t crc,const void * data,uint16_t len)
{
    const static uint16_t crc_tab[16] =
    {
        0x0000 , 0x1021 , 0x2042 , 0x3063 , 0x4084 , 0x50A5 , 0x60C6 , 0x70E7 ,
        0x8108 , 0x9129 , 0xA14A , 0xB16B , 0xC18C , 0xD1AD , 0xE1CE , 0xF1EF
    };
    uint8_t h_crc;
    const uint8_t * ptr = (const uint8_t *)data;
    //
    while(len --)
    {
        h_crc = (uint8_t)(crc >> 12);
        crc <<= 4;
        crc ^= crc_tab[h_crc ^ ((*ptr) >> 4)];
        //
        h_crc = crc >> 12;
        crc <<= 4;
        crc ^= crc_tab[h_crc ^ ((*ptr) & 0x0F)];
        //
        ptr ++;
    }
    //
    return crc;
}
