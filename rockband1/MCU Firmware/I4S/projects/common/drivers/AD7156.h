#ifndef  AD7156_H
#define  AD7156_H

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "stdint.h"

//addr
#define AD7156_IIC_ADDR               0x90 // refer AD7156.pdf p23
//write-read bit
#define AD7156_WRT_FLG                0x00
#define AD7156_RD_FLG                 0x01
//#define I2C_SUBA_ONEBYTE             0x01//要写几个字节

#define AD7156_PART_ID                0x88  //

#define INDOOR  0
#define OUTDOOR  1


#define AD7156_I2C_cmuClock_I2C       cmuClock_I2C0
#define AD7156_I2C_gpioPort           gpioPortD
#define AD7156_I2C_SCL_PIN            15
#define AD7156_I2C_SDA_PIN            14
#define AD7156_I2C                    I2C0
#define AD7156_I2C_LOC                I2C_ROUTE_LOCATION_LOC3
#define AD7156_I2C_IRQn               I2C0_IRQn


// I2C Registers
#define AD7156_STATUS       0x00
#define AD7156_CH1_DATA_H   0x01
#define AD7156_CH1_DATA_L   0x02
#define AD7156_CH2_DATA_H   0x03
#define AD7156_CH2_DATA_L   0x04
#define AD7156_CH1_AVG_H    0x05
#define AD7156_CH1_AVG_L    0x06
#define AD7156_CH2_AVG_H    0x07
#define AD7156_CH2_AVG_L    0x08
#define AD7156_CH1_SENS     0x09
#define AD7156_CH1_THR_H    0x09
#define AD7156_CH1_TIMEOUT  0x0A
#define AD7156_CH1_THR_L    0x0A
#define AD7156_CH1_SETUP    0x0B
#define AD7156_CH2_SENS     0x0C
#define AD7156_CH2_THR_H    0x0C
#define AD7156_CH2_TIMEOUT  0x0D
#define AD7156_CH2_THR_L    0x0D
#define AD7156_CH2_SETUP    0x0E
#define AD7156_CONFIG       0x0F
#define AD7156_PWDN_TIMER   0x10
#define AD7156_CH1_CAPDAC   0x11
#define AD7156_CH2_CAPDAC   0x12
#define AD7156_SN3          0x13
#define AD7156_SN2          0x14
#define AD7156_SN1          0x15
#define AD7156_SN0          0x16
#define AD7156_CHIP_ID      0x17    //AD7156_PART_ID 0x88

#define AD7156_RESET        0xBF    //reset all config to default, 2ms

/* AD7156_STATUS bit definition */
#define AD7156_STATUS_PWR_DWN   (1<<7)
#define AD7156_STATUS_DAC_STEP2 (1<<6)
#define AD7156_STATUS_OUT2      (1<<5)
#define AD7156_STATUS_DAC_STEP1 (1<<4)
#define AD7156_STATUS_OUT1      (1<<3)
#define AD7156_STATUS_C1_C2     (1<<2)
#define AD7156_STATUS_RDY2      (1<<1)
#define AD7156_STATUS_RDY1      (1<<0)

/* AD6156_CH1/2_SETUP bit definition */
#define AD7156_CH1_SETUP_RANGE(x)   (((x) & 0x3) << 6)
#define AD7156_CH1_SETUP_HYST1      (1 << 4)
#define AD7156_CH1_SETUP_THR1(x)    ((x) & 0xF)

#define AD7156_CH2_SETUP_RANGE(x)   (((x) & 0x3) << 6)
#define AD7156_CH2_SETUP_HYST2      (1 << 4)
#define AD7156_CH2_SETUP_THR2(x)    ((x) & 0xF)

/* AD7156_CH1_SETUP_RANGE(x) and AD7156_CH2_SETUP_RANGE(x) options */
#define AD7156_CDC_RANGE_2_PF       0
#define AD7156_CDC_RANGE_0_5_PF     1
#define AD7156_CDC_RANGE_1_PF       2
#define AD7156_CDC_RANGE_4_PF       3

/* AD7156_CFG definition */
#define AD7156_CONFIG_THR_FIXED        (1 << 7)
#define AD7156_CONFIG_THR_MD(x)        (((x) & 0x3) << 5)
#define AD7156_CONFIG_EN_CH1           (1 << 4)
#define AD7156_CONFIG_EN_CH2           (1 << 3)
#define AD7156_CONFIG_MD(x)            ((x) & 0x7)

/* AD7156_CONFIG_THR_FIXED options */
#define AD7156_ADAPTIVE_THRESHOLD       0
#define AD7156_FIXED_THRESHOLD          1

/* AD7156_CONFIG_THR_MD(x) options */
#define AD7156_THR_MODE_NEGATIVE        0
#define AD7156_THR_MODE_POSITIVE        1
#define AD7156_THR_MODE_IN_WINDOW       2
#define AD7156_THR_MODE_OU_WINDOW       3

/* AD7156_CONFIG_MD(x) options */
#define AD7156_CONV_MODE_IDLE            0
#define AD7156_CONV_MODE_CONT_CONV       1
#define AD7156_CONV_MODE_SINGLE_CONV     2
#define AD7156_CONV_MODE_PWR_DWN         3

/* AD7156_REG_PWR_DWN_TMR definition */
#define AD7156_PWR_DWN_TMR_TIMEOUT(x)   (((x) & 0x3F) | (1 << 6))

/* AD7156_REG_CH1_CAPDAC */
#define AD7156_CH1_CAPDAC_DAC_EN1       (1 << 7)
#define AD7156_CH1_CAPDAC_DAC_AUTO1     (1 << 6)
#define AD7156_CH1_CAPDAC_DAC_VAL1(x)   ((x) & 0x3F)

/* AD7156_REG_CH2_CAPDAC */
#define AD7156_CH2_CAPDAC_DAC_EN2       (1 << 7)
#define AD7156_CH2_CAPDAC_DAC_AUTO2     (1 << 6)
#define AD7156_CH2_CAPDAC_DAC_VAL2(x)   ((x) & 0x3F)

/* AD7156 channels */
#define AD7156_CHANNEL1                 1
#define AD7156_CHANNEL2                 2


/* =============function declare===================== */
void AD7156_IIC_Init(void);
void AD7156_Init(void);
void AD7156_Reset(void);
int AD7156_ReadReg(uint8_t Reg, uint8_t* data);
int AD7156_BlockRead(uint8_t address, uint8_t length, uint8_t* values);
int AD7156_WriteReg(uint8_t WriteAddr, uint8_t Data);
int AD7156_BlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values);
void AD7156_SetPowerMode(uint8_t pwrMode);
void AD7156_ChannelState(uint8_t ch, uint8_t enableConv);
void AD7156_SetRange(uint8_t ch, uint8_t range);
float AD7156_GetRange(uint8_t ch);
void AD7156_SetThrMode(uint8_t thrMode, uint8_t thrFixed);
void AD7156_SetThreshold(unsigned char channel, float pFthr);
void AD7156_SetSensitivity(unsigned char channel, float pFsensitivity);

uint16_t AD7156_ReadChData(uint8_t ch);
uint16_t GetAD7156_Ch1(void);
uint16_t GetAD7156_Ch2(void);
unsigned short AD7156_WaitReadChData(unsigned char channel);
float AD7156_ReadChCapacitance(unsigned char channel);
float AD7156_WaitReadChCapacitance(unsigned char channel);

#endif
