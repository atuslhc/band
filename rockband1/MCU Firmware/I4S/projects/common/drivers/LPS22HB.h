#ifndef  LPS22HB_H
#define  LPS22HB_H

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "stdint.h"

//addr
#define LPS22HB_IIC_ADDR               0xBA // refer LPS22HB_Barometer.pdf p26, SA0 1(0xBA) 0(0xB8)
#define LPS22HB_GLOBAL_RST_CMD         0x06 
//write-read bit
#define LPS22HB_WRT_FLG                0x00
#define LPS22HB_RD_FLG                 0x01
//#define I2C_SUBA_ONEBYTE             0x01//要写几个字节

#define LPS22HB_PART_ID            0xB1  //



#define LPS22HB_I2C_cmuClock_I2C       cmuClock_I2C0
#define LPS22HB_I2C_gpioPort           gpioPortD
#define LPS22HB_I2C_SCL_PIN            15
#define LPS22HB_I2C_SDA_PIN            14
#define LPS22HB_I2C                    I2C0
#define LPS22HB_I2C_LOC                I2C_ROUTE_LOCATION_LOC3
#define LPS22HB_I2C_IRQn               I2C0_IRQn


// I2C Registers
#define LPS22HB_INTERRUPT_CFG 0x0B
#define LPS22HB_THS_P_L 0x0C
#define LPS22HB_THS_P_H 0x0D

#define LPS22HB_WHO_AM_I 0x0F	// 0xB1 10110001
#define LPS22HB_CTRL_REG1 0x10
#define LPS22HB_CTRL_REG2 0x11
#define LPS22HB_CTRL_REG3 0x12

#define LPS22HB_FIFO_CTRL 0x14
#define LPS22HB_REF_P_XL 0x15
#define LPS22HB_REF_P_L 0x16
#define LPS22HB_REF_P_H 0x17
#define LPS22HB_RPDS_L 0x18
#define LPS22HB_RPDS_H 0x19
#define LPS22HB_RES_CONF 0x1A

#define LPS22HB_INT_SOURCE 0x25
#define LPS22HB_FIFO_STATUS 0x26
#define LPS22HB_STATUS 0x27
#define LPS22HB_PRESS_OUT_XL 0x28
#define LPS22HB_PRESS_OUT_L 0x29
#define LPS22HB_PRESS_OUT_H 0x2A
#define LPS22HB_TEMP_OUT_L 0x2B
#define LPS22HB_TEMP_OUT_H 0x2C

#define LPS22HB_LPFP_RES 0x33

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



void LPS22HB_Init(void);

void LPS22HB_start_conversion(void);
void LPS22HB_Read_converter(void);

uint16_t GetLPS22HB_Pressure(void);
uint8_t GeLPS22HB_Temperature(void);

#endif
