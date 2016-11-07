#ifndef __MOO930_H
#define __MOO930_H

#include <stdint.h>
#include <stdbool.h>


#define OLED_IIC_ADDR 0x79
#define OLED_CMD  0x00
#define OLED_DAT  0x40


void OLEDInit();
void OledPwrEnable(uint8_t enable);

bool isOLEDOn();
bool isOLEDOff();

//¹Ø±ÕLED
void OLEDOff(void);
void OLEDON(void);

int8_t OLED_Write_Command(unsigned char command);
int8_t OLED_Write_CommandNBytes(unsigned char *pCommand,unsigned char n);
int8_t OLED_Write_Data(unsigned char date);
int8_t OLED_Write_DataNBytes(unsigned char *pData,unsigned char n);

void OLED_Contrast_Con(unsigned char bright);

#endif
