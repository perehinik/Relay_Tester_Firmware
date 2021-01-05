#ifndef I2C_INI_H
#define I2C_INI_H

#include "stm32f1xx_hal.h"

void OutByteToI2C1(uint8_t Adres,uint8_t outByte);
void LCDInit(uint8_t LCDAdres);
void SecString(void);
void FirstString(void);
void LCDPrint(char s[]);
void LCDClear(void);
void LCDCursorPozition(uint8_t X,uint8_t Y);

#endif
