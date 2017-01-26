#ifndef  LPS35HW_H
#define  LPS35HW_H

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "stdint.h"

// [BG037] add BAROMETER/Pressure - LPS35HW
//#if (BAROMETER_SUPPORT==2)   //[BG037]
#define LPS35HW_I2C_cmuClock_I2C       cmuClock_I2C0
#define LPS35HW_I2C_gpioPort           gpioPortD
#define LPS35HW_I2C_SCL_PIN            15
#define LPS35HW_I2C_SDA_PIN            14
#define LPS35HW_I2C                    I2C0
#define LPS35HW_I2C_LOC                I2C_ROUTE_LOCATION_LOC3
#define LPS35HW_I2C_IRQn               I2C0_IRQn

#define LPS35HW_INT_PORT gpioPortD
#define LPS35HW_INT_PIN  6
//#endif

//addr
#define LPS35HW_IIC_ADDR               0xBA// SA0 1(0xBA) 0(0xB8)
#define LPS35HW_GLOBAL_RST_CMD         0x06 
//write-read bit
#define LPS35HW_WRT_FLG                0x00
#define LPS35HW_RD_FLG                 0x01
//#define I2C_SUBA_ONEBYTE             0x01//要写几个字节

#define INDOOR  0
#define OUTDOOR  1



// I2C Registers
#define LPS35HW_INTERRUPT_CFG 0x0B
#define LPS35HW_THS_P_L 0x0C
#define LPS35HW_THS_P_H 0x0D

#define LPS35HW_WHO_AM_I 0x0F	// 0xB1 10110001
#define LPS35HW_CTRL_REG1 0x10
#define LPS35HW_CTRL_REG2 0x11
#define LPS35HW_CTRL_REG3 0x12

#define LPS35HW_FIFO_CTRL 0x14
#define LPS35HW_REF_P_XL 0x15
#define LPS35HW_REF_P_L 0x16
#define LPS35HW_REF_P_H 0x17
#define LPS35HW_RPDS_L 0x18
#define LPS35HW_RPDS_H 0x19
#define LPS35HW_RES_CONF 0x1A

#define LPS35HW_INT_SOURCE 0x25
#define LPS35HW_FIFO_STATUS 0x26
#define LPS35HW_STATUS 0x27
#define LPS35HW_PRESS_OUT_XL 0x28
#define LPS35HW_PRESS_OUT_L 0x29
#define LPS35HW_PRESS_OUT_H 0x2A
#define LPS35HW_TEMP_OUT_L 0x2B
#define LPS35HW_TEMP_OUT_H 0x2C

#define LPS35HW_LPFP_RES 0x33

// Parameter


#define Psens	4096 // LSB/hPa
#define Tsens	100 // LSB/C

// CTRL_REG2 (11h)
#define ONE_SHOT	0x01

	
// RES_CONF_1Ah
#define LC_EN	0x01


// STATUS_27h
#define T_OR	0x20
#define P_OR	0x10
#define T_DA	0x02
#define P_DA	0x01



void LPS35HW_Init(void);

void LPS35HW_start_conversion(void);
void LPS35HW_Read_converter(void);

uint16_t GetLPS35HW_Pressure(void);
uint8_t GeLPS35HW_Temperature(void);

#endif
