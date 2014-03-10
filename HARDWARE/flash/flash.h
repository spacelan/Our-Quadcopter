#ifndef FLASH_H
#define FLASH_H

#include "sys.h"

uint32_t MyFlash_ReadWord(uint32_t addr);
uint16_t MyFlash_ReadHalfWord(uint32_t addr);
uint8_t MyFlash_ReadByte(uint32_t addr);
void MyFlash_Read(uint32_t addr,uint16_t *data,uint16_t length);
void MyFlash_Write(uint32_t addr,uint16_t *data,uint16_t length);

#endif
