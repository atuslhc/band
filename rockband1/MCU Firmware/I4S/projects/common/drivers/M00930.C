#include "freertos.h"
#include "task.h"

#include "m00930.h"

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "common_vars.h"
#include "main.h"
#include "Si14x.h"


uint8_t oledRamBuf[256];

#if (OLED_SUPPORT)
//void OledPwrEnable(uint8_t enable)
//{
//	if(true == enable)
//	{
//		OLED_PWR_OFF();
//	}
//	else
//	{
//		OLED_PWR_ON();
//	}
//}

void OledI2CInit(void)
{
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	uint8_t i;
	
	if(I2C0_used==false)
	{
		I2C0_used=true;
		
		CMU_ClockEnable(OLED_I2C_cmuClock_I2C, true);
		
		/* Output value must be set to 1 to not drive lines low... We set */
		/* SCL first, to ensure it is high before changing SDA. */
		GPIO_PinModeSet(OLED_I2C_gpioPort, OLED_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		GPIO_PinModeSet(OLED_I2C_gpioPort, OLED_I2C_SDA_PIN, gpioModeWiredAnd, 1);
		
		/* In some situations (after a reset during an I2C transfer), the slave */
		/* device may be left in an unknown state. Send 9 clock pulses just in case. */
		for (i = 0; i < 9; i++)
		{
			/*
			* TBD: Seems to be clocking at appr 80kHz-120kHz depending on compiler
			* optimization when running at 14MHz. A bit high for standard mode devices,
			* but DVK only has fast mode devices. Need however to add some time
			* measurement in order to not be dependable on frequency and code executed.
			*/
			GPIO_PinModeSet(OLED_I2C_gpioPort, OLED_I2C_SCL_PIN, gpioModeWiredAnd, 0);
			GPIO_PinModeSet(OLED_I2C_gpioPort, OLED_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		}
		
		/* Enable pins at location 2 */
		OLED_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | OLED_I2C_LOC;
		I2C_Init(OLED_I2C, &i2cInit);
		
		/* Clear and enable interrupt from I2C module */
		NVIC_ClearPendingIRQ(OLED_I2C_IRQn);
		NVIC_EnableIRQ(OLED_I2C_IRQn);
	}
}



//static void Delay_Ms(unsigned int j)
//{
//	unsigned int i;
//
//	for(; j > 0; j--)
//		for(i = 0xfff; i > 0; i--);
//}

/**********************************************
//
//写命令函数

**********************************************/


void OLED_RST(void)
{
	OLED_RST_L();
	vTaskDelay(2);
	OLED_RST_H();
}


bool blOLEDPowerOn = false;

bool isOLEDOn()
{
	return blOLEDPowerOn;
}

bool isOLEDOff()
{
	return !blOLEDPowerOn;
}

//关闭LED
void OLEDOff(void)
{
   
	blOLEDPowerOn = false;
	OLED_Write_Command(0xAE);     //Set Display Off 关闭显示
	//OLED_Write_Command(0x10);		//disable charge pump	
	
}

void OLEDON(void)
{
	Check_UV_Sensor(INDOOR);//通过UV sensor获取环境关系，调整对比度。
	if(AmbientLight<1000)
		OLED_Contrast_Con(AmbientLight/10);
	else		  
		OLED_Contrast_Con(0xff);	
	
	blOLEDPowerOn = true;
	OLED_Write_Command(0xAF);     //Set Display On
}


// OLED turn on and initialize.
void OLEDInit(void)
{
	if (blOLEDPowerOn)
		return;
	
	blOLEDPowerOn = true;
	
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(OLED_RST_PORT, OLED_RST_PIN, gpioModePushPull, 0);
	
	OledI2CInit();
	
	OLED_RST();
	
	OLED_Write_Command(0xAE);     //Set Display Off 关闭显示
	
	OLED_Write_Command(0xd5);     //display divide ratio/osc. freq. mode设置时钟分频因子，与震荡频率
	OLED_Write_Command(0xd1);     //105Hz	  内部晶振470 KHz      //[3:0]分频因子，[7:4]震荡频率
	
	OLED_Write_Command(0xA8);     //multiplex ration mode:31设置驱动路数
	OLED_Write_Command(0x1F);		//32
	
	OLED_Write_Command(0xD3);     //Set Display Offset设置显示偏移
	OLED_Write_Command(0x00);     //默认为0
	
	OLED_Write_Command(0x40);     //Set Display Start Line设置显示开始行[5:0]，行数
	
	OLED_Write_Command(0x8D);     //Set Display Offset 电荷泵设置
	OLED_Write_Command(0x95);//0X14  20140514  9V 	//for enabling charge pump，bit2，开启/关闭
	//OLED_Write_Command(0x14);

	
	OLED_Write_Command(0x20);//
	OLED_Write_Command(0x02);//
	
	OLED_Write_Command(0xa1);     //Segment Remap
	OLED_Write_Command(0xC8);     //Sst COM Output Scan Direction
	
	//设置COM硬件引脚 配置
	OLED_Write_Command(0xDA);     //common pads hardware: alternative
	OLED_Write_Command(0x12);
	
	//对比度设置
	OLED_Write_Command(0x81);     //contrast control
	OLED_Write_Command(0xFF);//1-255，默认是7F、亮度设置，越大越亮
	
	//设置预充电周期
	OLED_Write_Command(0xD9);	    //set pre-charge period
	OLED_Write_Command(0xF1);
	
	//设置VCOMH 电压倍率
	OLED_Write_Command(0xDB);     //VCOM deselect level mode
	OLED_Write_Command(0x40);	    //set Vvcomh=0.83*Vcc
	
	//全局显示开启;bit0:1开启，0,关闭;(白屏/黑屏)
	OLED_Write_Command(0xA4);     //Set Entire Display On/Off
	
	OLED_Write_Command(0xA6);     //Set Normal Display 设置显示方式;Bit0:1,反相显示，0：正常显示
	OLED_Write_Command(0xAF);     //Set Display On
}


void OLED_Contrast_Con(unsigned char bright)
{
	if(bright==0xff)
	{
		OLED_Write_Command(0x95);
		OLED_Write_Command(0x81);     //contrast control
		OLED_Write_Command(0xFF);//1-255，默认是7F、亮度设置，越大越亮
	}
	else
	{
		OLED_Write_Command(0x14);
		OLED_Write_Command(0x81);     //contrast control
		OLED_Write_Command(bright);//1-255，默认是7F、亮度设置，越大越亮
	}
	
}


int8_t OLED_Write_Command(unsigned char command)
{
//	// =============================================================
//	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return i2cTransferSwFault; // 获取信号量失败
//	// =============================================================
	
	
	unsigned char ret;
	ret = I2CWriteNByte(OLED_I2C,OLED_IIC_ADDR,I2C_SUBA_ONEBYTE,OLED_CMD,&command,1);
	
	
//	// =============================================================
//	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);
//	// =============================================================


	if (ret == i2cTransferDone) {
		return 0;
	} else {
		//while(1);
		return 0xff;
	}
	
}


int8_t OLED_Write_CommandNBytes(unsigned char *pCommand,unsigned char n)
{
//	// =============================================================
//	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return i2cTransferSwFault; // 获取信号量失败
//	// =============================================================
	
	unsigned char ret;
	ret = I2CWriteNByte(OLED_I2C,OLED_IIC_ADDR,I2C_SUBA_ONEBYTE,OLED_CMD,pCommand,n);
	
	
//	// =============================================================
//	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);
//	// =============================================================
	
	if (ret == i2cTransferDone) {
		return 0;
	} else {
		//while(1);
		return 0xff;
	}
}


/**********************************************
//
//写数据函数
//
**********************************************/
//int8_t OLED_Write_Data(unsigned char date)
//{
//	// =============================================================
//	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return i2cTransferSwFault; // 获取信号量失败
//	// =============================================================
//	
//
//	unsigned char ret;
//	ret = I2CWriteNByte(OLED_I2C,OLED_IIC_ADDR,I2C_SUBA_ONEBYTE,OLED_DAT,&date,1);
//	
//	
//	// =============================================================
//	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);
//	// =============================================================
//	
//	if (ret == i2cTransferDone) {
//		return 0;
//	} else {
//		//while(1);
//		return 0xff;
//	}
//}

int8_t OLED_Write_DataNBytes(unsigned char *pData,uint8_t n)
{
//	// =============================================================
//	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, 20) != 1)
//		return i2cTransferSwFault; // 获取信号量失败
//	// =============================================================
	

	unsigned char ret;
	ret = I2CWriteNByte(OLED_I2C,OLED_IIC_ADDR,I2C_SUBA_ONEBYTE,OLED_DAT,pData,n);
	
	
//	// =============================================================
//	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);
//	// =============================================================
	
	if (ret == i2cTransferDone) {
		return 0;
	} else {
		//while(1);
		return 0xff;
	}
}
#endif
