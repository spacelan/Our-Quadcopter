#ifndef USART_H
#define USART_H
#include "sys.h"

void MyUSART_Init(u32 bound);
void MyUSART_Transmit(const u8 *data,u8 length);
void MyUSART_Receive(u8 *data,u8 length);
u8 MyUSART_GetRxBufSize(void);
u8 MyUSART_GetTxBufSzie(void);

#endif
