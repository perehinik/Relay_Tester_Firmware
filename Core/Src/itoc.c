#include "itoc.h"
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
/*******************************************************************/
void OutByteToI2C1(uint8_t Adres, uint8_t outByte)
{
	/*
	Sending 1 Byte of Data to devise with Adress on I2C bus I2C1	
	*/
	HAL_I2C_Master_Transmit(&hi2c1, Adres, &outByte, 1, 0x100);
	/* for SPL lib
   while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
   I2C_GenerateSTART(I2C1, ENABLE);
 
   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
   I2C_Send7bitAddress(I2C1, Adres, I2C_Direction_Transmitter);
 
   while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
   I2C_SendData(I2C1, outByte);
	
   while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
   I2C_GenerateSTOP(I2C1, ENABLE);
	*/
}

void LCDInit(uint8_t LCDAdres)
{
	/*
	Initialisating 4-bit mode and settings of LCD
	LCDAdres - variable with adress of LCD on I2C bus
	tacting on Input E of LCD - falling front
	*/
	HAL_Delay(50);
	//uint16_t InitComands[14]= {0x3C,0x3C,0x3C,0x2C,0x2C,0x8C,0x0C,0xCC,0x0C,0x1C,0x0C,0x6C,0x0C,0x2C}; //for 8574 module
	uint16_t InitComands[14]= {0x13,0x13,0x13,0x12,0x12,0x18,0x10,0x1C,0x10,0x11,0x10,0x16,0x10,0x12};
	for (uint16_t i=0;i<=11;i++)
	{
	OutByteToI2C1(LCDAdres,InitComands[i]);
	OutByteToI2C1(LCDAdres,0xEF&InitComands[i]);  // 1 takt impuls on input E of LCD
		
		if (i==0 ||  i==1 ||  i==9)
			{ 
				HAL_Delay(10);
			}
	} 
	HAL_Delay(20); 
}

void SecString(void){
	/*
	moving cursor to 2 string of LCD
	0x40 - adress LCD on i2C bus
	*/
	OutByteToI2C1(0x40,0x1A);
	OutByteToI2C1(0x40,0x0A);
	OutByteToI2C1(0x40,0x18);
	OutByteToI2C1(0x40,0x08);
	HAL_Delay(5);
}

void FirstString(void){
	/*
	moving cursor to 2 string of LCD
	0x40 - adress LCD on i2C bus
	*/
	OutByteToI2C1(0x40,0x18);
	OutByteToI2C1(0x40,0x08);
	OutByteToI2C1(0x40,0x10);
	OutByteToI2C1(0x40,0x00);
	HAL_Delay(5);
}

void LCDCursorPozition(uint8_t Y,uint8_t X){
	/*
	moving cursor to poxition of LCD writed in PozAdres
	0x40 - adress LCD on i2C bus
	for LCD 1601: 0x00 - first simbol of 1 string
	              0xC0 - first simbol of 2 string
	
  PozAdress shisted for interpretation to 4-bit mode LCD
	*/
	uint8_t PozAdres = 0x10;
	if (Y==2) PozAdres = 0xA8;
	PozAdres = PozAdres+X;
	
	OutByteToI2C1(0x40,(PozAdres>>4)|0x10);
	OutByteToI2C1(0x40,(PozAdres>>4)|0x00);
	OutByteToI2C1(0x40,(PozAdres & 0x0F)|0x10);
	OutByteToI2C1(0x40,(PozAdres & 0x0F)|0x00);

	HAL_Delay(5);
}

void LCDClear(void){
	/*
	clearing LCD
	moving cursor to start position 0x00
	0x4E - adress LCD on i2C bus
	*/
	OutByteToI2C1(0x40,0x10);
	OutByteToI2C1(0x40,0x00);
	OutByteToI2C1(0x40,0x11);
	OutByteToI2C1(0x40,0x01);
	HAL_Delay(5);
}

void LCDPrint(char s[])
	/*
	printing string array char s[]
	for printing other types you need to use function sprintf
				
				#include <stdio.h>
				char str[16];
				uint32_t t = 23;
				sprintf(str, "%i%s",t," Hello World!");
				LCDPrint(str);
			  (part ofprogram like this will write 23 Hello World!)

	0x4E - adress LCD on i2C bus
	*/
	{
		uint16_t SymbolCode[95]={0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,
		0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x41,0x42,0x43,0x44,0x45,
		0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x21,0x23,
		0x24,0x25,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,0x40,0x5B,0x5D,0x5E,0x5F,
		0x6B,0x6C,0x6D,0xDF};
		
		char Symbol[95]={"0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!#$%&'()*+,-./:;<=>?@[]^_{|}°"};
		
		uint8_t letrnum=strlen(s);
		uint16_t Letter;
		
		for(uint16_t i=0;i<letrnum;i++)
		{
			if (s[i]==' ')
				{
				Letter = 0x20; //can be 0x80 fo another LCD controller
				}
			for (uint16_t p=0;p<80;p++)
				{
					if (s[i]==Symbol[p])
					{
					Letter = SymbolCode[p];
					}
				}
			
			OutByteToI2C1(0x40,(Letter>>4)|0x50);
			OutByteToI2C1(0x40,(Letter>>4)|0x40);
			OutByteToI2C1(0x40,(Letter & 0x0F)|0x50);
			OutByteToI2C1(0x40,(Letter & 0x0F)|0x40);
		}
	}	
