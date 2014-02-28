#ifndef I2C_H
#define I2C_H

#include "sys.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

//0表示写
#define	I2C_Direction_Transmitter   0
//１表示读
#define	I2C_Direction_Receiver      1	 
/*====================================================================================================*/
/*====================================================================================================*/
//bool i2cWriteWords(uint8_t addr, uint8_t reg, uint8_t len, uint16_t* data);
//bool i2cWriteWord(uint8_t addr, uint8_t reg, uint16_t data);
bool MyI2C_WriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t const *buf);
//bool i2cWriteBits(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t len, uint8_t data);
//bool i2cWriteBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t data);
bool MyI2C_Write(uint8_t addr, uint8_t reg, uint8_t const *data);

bool MyI2C_ReadBuffer(uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf);
//bool i2cReadBits(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t len, uint8_t *data, uint16_t *timeout);
//bool i2cReadBit(uint8_t addr, uint8_t reg, uint8_t bitNum, uint8_t *data, uint16_t *timeout);
bool MyI2C_Read(uint8_t addr, uint8_t reg, uint8_t *data);

void MyI2C_Init(void);

bool I2c_write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t const *buf);
bool I2c_read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
/*====================================================================================================*/
/*====================================================================================================*/
#endif

