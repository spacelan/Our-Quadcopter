#include "usart.h"
 
#define MY_USART_BUF_SIZE 256
u8 myUSARTRxBuf[MY_USART_BUF_SIZE];     //接收缓冲
u8 myUSARTTxBuf[MY_USART_BUF_SIZE];	 //发送缓冲
u8 rxBufHead = 0,rxBufTail = 0;
u8 txBufHead = 0,txBufTail = 0;


void MyUSART_Init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO, ENABLE);
	//USART1_TX   PA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART1_RX	  PA.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//

	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1

	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	//开启中断
	USART_ClearITPendingBit(USART1,USART_IT_TXE);
//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	USART_Cmd(USART1, ENABLE);//使能串口 
}

//发送队列头部
void MyUSART_SendByte()
{
	USART_SendData(USART1,myUSARTTxBuf[txBufHead]);
	if(txBufHead == MY_USART_BUF_SIZE - 1)
		txBufHead = 0;
	else
		txBufHead++;
}

//压入发送队列尾部
void MyUSART_Transmit(const u8 *data,u8 length)
{	
	//队列中有数据，开启发送中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	while(length--)
	{
		myUSARTTxBuf[txBufTail] = *data;
		data++;
		if(txBufTail == MY_USART_BUF_SIZE - 1)
			txBufTail = 0;
		else
			txBufTail++;
	}
	//队列中有数据，开启发送中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

//串口接收一个字节，压入接收队列尾部
void MyUSART_GetByte()
{
	myUSARTRxBuf[rxBufTail] = USART_ReceiveData(USART1);
	if(rxBufTail == MY_USART_BUF_SIZE - 1)
		rxBufTail = 0;
	else
		rxBufTail++;
}

//读取接收队列头部
void MyUSART_Receive(u8 *data,u8 length)
{
	while(length--)
	{
		*data = myUSARTRxBuf[rxBufHead];
		data++;
		if(rxBufHead == MY_USART_BUF_SIZE - 1)
			rxBufHead = 0;
		else
			rxBufHead++;
	}
}

u8 MyUSART_GetRxBufSize()
{
	if(rxBufHead <= rxBufTail)
		return rxBufTail - rxBufHead;
	else 
		return MY_USART_BUF_SIZE - rxBufHead + rxBufTail;
}

u8 MyUSART_GetTxBufSzie()
{
	if(txBufHead <= txBufTail)
		return txBufTail - txBufHead;
	else 
		return MY_USART_BUF_SIZE - txBufHead + txBufTail;
}

//串口1中断服务程序
void USART1_IRQHandler(void)                	
{
	if(USART_GetITStatus(USART1,USART_IT_TXE) == SET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
		if(MyUSART_GetTxBufSzie() >= 1)
			MyUSART_SendByte();
		else
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//队列中没有数据，关闭中断，开启中断在函数MyUSART_Transmit
	}
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		MyUSART_GetByte();
	}
} 
