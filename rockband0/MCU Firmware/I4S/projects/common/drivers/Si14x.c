#include "FreeRTOS.h"
#include "Si14x.h"
#include "I2c_driver.h"
#include "em_gpio.h"
#include "task.h"
#include "GlobalData.h"
#include "common_vars.h"
#include "main.h"


/********有关校验的一些宏定义***************************************/
//                                                         msb  lsb  align
//                                                         i2c  i2c  ment
//                                                         addr addr

#define  ALSIR_ADCHI_IRSENSE                    (collect(buf,0x23, 0x22, 0))
#define  ALSIR_ADCLO_IRSENSE                    (collect(buf,0x22, 0x25, 1))
#define  ALSIR_ADCLO_WHSENSE                    (collect(buf,0x24, 0x26, 0))
#define  ALSVIS_ADCHI_WHSENSE                   (collect(buf,0x26, 0x27, 1))
#define  ALSVIS_ADCLO_WHSENSE                   (collect(buf,0x28, 0x29, 0))
#define  PROX3_ADCHI_IRSENSE                    (collect(buf,0x29, 0x2a, 1))
#define  LED_DRV65                              (collect(buf,0x2b, 0x2c, 0))
#define  ALSIR_ADCHI_IRSENSE_REF                 4.021290
#define  ALSIR_ADCLO_IRSENSE_REF                 57.528500
#define  ALSIR_ADCLO_WHSENSE_REF                 2.690010
#define  ALSVIS_ADCHI_WHSENSE_REF                0.042903
#define  ALSVIS_ADCLO_WHSENSE_REF                0.633435
#define  PROX3_ADCHI_IRSENSE_REF                 23.902900
#define  LED_DRV65_REF                           56.88930


//extern volatile I2C_TransferReturn_TypeDef I2C0_Status;
extern uint32_t I2C0_error_count, I2C1_error_count;

//#define Si14X_I2C_Status                    I2C0_Status

//bool systemStatus.blUVSensorOnline = false;


/********函数声明***************************************/

static int WaitUntilSleep(uint8_t nonused);

int SendCMD(uint8_t cmd);

//int SI14x_I2CReadNByte_GetRegData(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);
int SI14x_WriteReg(uint8_t WriteAddr, uint8_t Data);
//int SI14x_ReadReg(BYTE Reg, BYTE* nonused);
int SI14x_ReadReg(uint8_t Reg, uint8_t* data);

int Si114xBlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values);
int Si114xBlockRead(uint8_t address, uint8_t length, uint8_t* values);

int Si114xParamSet(uint8_t address, uint8_t value);
//int Si114xParamRead(BYTE address);
int Si114xParamRead(uint8_t address, uint8_t* data);

int si114x_set_ucoef(uint8_t ref_ucoef[], SI114X_CAL_S* si114x_cal );
int SI14xNOP(void);
int SI14x_PauseAll(void);
int SI14xPsForce(void);
float collect(uint8_t buf[], uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment);


void SI14x_IIC_Init(void)
{
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	uint8_t i = 0;

	if(I2C0_used == false)
	{
		I2C0_used = true;

		CMU_ClockEnable(cmuClock_GPIO, true);
		CMU_ClockEnable(SI14x_I2C_cmuClock_I2C, true);
		GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModePushPull, 1);
		GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SDA_PIN, gpioModePushPull, 1); //

		/*产生几个脉冲使IIC总线稳定*/
		for (i = 0; i < 9; i++)
		{
			GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModeWiredAnd, 0);
			GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		}

		/*确保IIC的两个引脚都输出高*/
		GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModeWiredAnd, 1);//配置成线与(也就是开漏) 数据是1
		GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SDA_PIN, gpioModeWiredAnd, 1);

		/* Enable pins at location 3*/
		SI14x_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | SI14x_I2C_LOC;
		I2C_Init(SI14x_I2C, &i2cInit);//后一个参数是指针，所以这里把变量取地址传入。

		/* Clear and enable interrupt from I2C module */
		NVIC_ClearPendingIRQ(SI14x_I2C_IRQn);
		NVIC_EnableIRQ(SI14x_I2C_IRQn);
	}
}


static int WaitUntilSleep(uint8_t nonused)
{
	//    BYTE  si114x_handle = nonused;
	INT8 retval;

	// This loops until the Si114x is known to be in its sleep state
	// or if an i2c error occurs
	while (1)
	{
		SI14x_ReadReg(REG_CHIP_STAT, (uint8_t*)&retval);

		if (retval == 1) break;
		else  return retval;
	}

	return 0;
}

/*最终获取的数据在data中存储*/
int SI14x_ReadReg(uint8_t Reg, uint8_t* data)
{
	uint8_t ret;


	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return i2cTransferSwFault;

	// =============================================================
	ret = I2CReadNByte(SI14x_I2C, SIl4x_IIC_ADDR, I2C_SUBA_ONEBYTE, Reg, data, 1);


	// =============================================================
	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);

	xSemaphoreGive(hI2CSemaphore);


	// =============================================================
	//
	if(ret == i2cTransferDone)
	{
		return i2cTransferDone; //0;
	}
	else
	{
		return 0xff;
	}
}

/*写成功返回0，失败返回0xff*/
int SI14x_WriteReg(uint8_t WriteAddr, uint8_t Data)
{
	int ret;


	// =============================================================
	// 获得i2c信号量
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // 获取信号量失败

	if (xSemaphoreTake(hI2CSemaphore, 20) != pdTRUE)
		return i2cTransferSwFault;


	// =============================================================
	ret = I2CWriteNByte(SI14x_I2C, SIl4x_IIC_ADDR, I2C_SUBA_ONEBYTE, WriteAddr, &Data, 1);


	// =============================================================
	// 释放I2C信号量
//	osSemaphoreRelease(hI2CSemaphore);
	xSemaphoreGive(hI2CSemaphore);

	// =========================================
	if (ret == i2cTransferDone)
	{
		return 0;
	}
	else
	{
		return 0xff;
	}

}

//块写入，用于配置参数
int Si114xBlockWrite(uint8_t  address, uint8_t  length, uint8_t  const* values)
{
	uint8_t  retval, counter;

	for ( counter = 0; counter < length; counter++)
	{
		retval = SI14x_WriteReg(address + counter, values[counter]);
	}

	return retval;
}


int Si114xBlockRead(uint8_t address, uint8_t length, uint8_t* values)
{
	uint8_t counter;
	//   BYTE nonused = 0;

	for ( counter = 0; counter < length; counter++)
	{
		//这里把数据放在value数组中。
		SI14x_ReadReg(address + counter, values);
		values++;
	}

	return 0;
}

//向RAM参数中address地址上写入value值
int Si114xParamSet(uint8_t address, uint8_t value)
{
	INT8  retval;
	uint8_t cmd, response;
	//   BYTE nonused = 0;

	// Wait for any prior command to complete
	if((retval = WaitUntilSleep(SIl4x_IIC_ADDR)) != 0 ) return retval;

	// Read RESPONSE register
	//这句完全可以用读响应寄存器

	SI14x_ReadReg(REG_RESPONSE, &response); //读到的数据放在response中
	// Parameter[address] = value
	cmd = 0xA0 + (address & 0x1F);

	//下面首先对RAM参数写寄存器中写入值value，然后往命令寄存器中写入命令cmd。
	// Fill PARAM_WR register
	if((retval = SI14x_WriteReg(REG_PARAM_WR, value)) != 0 )  return retval ;

	// Issue COMMAND register
	if((retval = SI14x_WriteReg(REG_COMMAND, cmd)) != 0 )  return retval ;

	// Wait for command to finish
	SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&retval);
	//   while(retval == response);

	//写入命令后，响应寄存器会变化的，与第一次读到的数据不一样，所以这里距退出了，表明命令写入正确。
	if (retval < 0 )
		return retval;
	else
		return 0;
}


// Sends the Parameter Read command to the SI114x to read and return Parameter values
// The Si114x incremenets the RESPONSE register upon the completion of a COMMAND
//  so the code reads the RESPONSE register, issues the COMMAND, then waits
//  for the RESPONSE register to complete before proceeding.

//该函数获取参数区address上的值，放在data中
int Si114xParamRead(uint8_t address, uint8_t* data)
{
	//  BYTE nonused = 0;
	uint8_t retval;
	uint8_t response ;

	// returns Parameter[address]
	uint8_t cmd = 0x80 + (address & 0x1F);

	// Read RESPONSE register
	SI14x_ReadReg(REG_RESPONSE, &response);
	SI14x_WriteReg(REG_COMMAND, cmd);    // Issue COMMAND register

	// Wait for command to finish
	SI14x_ReadReg(REG_RESPONSE, &retval);
	//   while(response == retval);

	return SI14x_ReadReg(REG_PARAM_RD, data); // Return PARAM_RD register value
}

/*该函数是直接向sensor寄存器中写入默认的校正值，真正的校正值，先获取校正数据，在计算，最后获取一个新的校正数据，这里先不这样做*/
int si114x_default_ucoef(void)
{
	int response;
	uint8_t  ucoef[4] = { 0x29, 0x89, 0x02, 0x00 } ;

	// This will write 4 bytes starting with I2C address 0x13
	response = Si114xBlockWrite( REG_UCOEF0, 4, &ucoef[0] );
	return response;
}


/***************************************关于UVsensor的一些命令*******************************************/

//static int SendCmd(BYTE command)
//{
//	BYTE si114x_handle = 0;
//	INT8  response;
//	INT8  retval;
//	INT8  temp;
//	BYTE  count = 0;
//
//	// Get the response register contents
//	SI14x_ReadReg(REG_RESPONSE, (BYTE*) &response);
//	if (response<0)
//		return response;
//
//	// Double-check the response register is consistent
//	while(1)
//	{
//		if((retval=WaitUntilSleep(si114x_handle)) != 0) return retval;
//
//		if(command==0) break; // Skip if the command is NOP
//
//		SI14x_ReadReg(REG_RESPONSE,(BYTE*) &retval);
//
//		if(retval==response) break;
//		else if(retval<0) return retval;
////		else response = retval;
//	    else //如果响应和最初的不一样，进行超时等待
//		{
//		  while((retval != response) && count < 3)
//			{
//			  	response = retval;
//				count++;
//				SI14x_ReadReg(REG_RESPONSE,(BYTE*) &retval);//再次读
//			}
//		  break;//退出while
//		}
//	}
//
//	// Send the Command
//	if ((retval=SI14x_WriteReg(REG_COMMAND, command)) !=0)
//		return retval;
//
//	 count = 3;
//	// Expect a change in the response register
//	while(1)
//	{
//		if(command==0)
//		  break; // Skip if the command is NOP
//		SI14x_ReadReg(REG_RESPONSE,(BYTE*)&temp);
//		if (temp != response)
//		  break;
//		else if(temp<0)
//		  return temp;
//		else if(temp == response)//如果发送完命令后读到的响应值与刚开始的一样，说明有问题，下面进行超时
//		{
//			while((temp == response) && (count--) )
//			{
//				response = temp;
//				SI14x_ReadReg(REG_RESPONSE,(BYTE*)&temp);
//				SysCtlDelay(800);
//			}
//			break;
//		}
//	}
//	return 0;
//}



static int SendCmd(uint8_t command)
{
	uint8_t si114x_handle = 0;
	INT16  response;
	INT16  retval;
	INT8  temp;
	uint8_t  count = 0;

	// Get the response register contents
	SI14x_ReadReg(REG_RESPONSE, (uint8_t*) &response);//在ALS溢出时RESPONSE寄存器中是0x8c,
//以前response的类型是int8，这里检测肯定为负数，就退出了，不执行命令函数了,所以溢出后显示-1回不去了

	if (response < 0)
		return response;

	// Double-check the response register is consistent


	if((retval = WaitUntilSleep(si114x_handle)) != 0) return retval;

	if(command != 0) // Skip if the command is NOP
	{
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*) &retval);

		if(retval == response) //如果读到的状态和最初一样，说明设备空闲可以直接发送命令
		{
			if ((retval = SI14x_WriteReg(REG_COMMAND, command)) != 0)
				return retval;
		}
		else if(retval < 0) //如果读到的状态是负数，出错直接退出
			return retval;
		else //如果响应和最初的不一样，进行超时处理，在退出超时时还是发送一次命令
		{
			while((retval != response) && count < 3)
			{
				response = retval;
				count++;
				SI14x_ReadReg(REG_RESPONSE, (uint8_t*) &retval); //再次读
			}

			if ((retval = SI14x_WriteReg(REG_COMMAND, command)) != 0)
				return retval;
		}
	}
	else//如果命令是0   Send the Command
	{
		if ((retval = SI14x_WriteReg(REG_COMMAND, command)) != 0)
			return retval;
	}

	count = 0;

// Expect a change in the response register

	if(command != 0) 		  // Skip if the command is NOP
	{
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&temp);

		if (temp != response)
			return 0;
		else if(temp < 0)
			return temp;
		else if(temp == response)//如果发送完命令后读到的响应值与刚开始的一样，说明有问题，下面再次获取状态
		{
			while((temp == response) && (count < 3) )
			{
				count++;
				response = temp;
				SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&temp);
			}

			if(count >= 3)
			{
				return temp;
			}
		}
	}
	else
	{
		return 0;
	}

	return 0;
}


int Si114xNop(void)
{
	return SendCmd(0x00);
}


static int PsAlsPause(void)
{
	return SendCmd(0x0B);
}


int Si114xAlsForce(void)
{

	if(systemSetting.blUVSensorEnabled == false)
		return 0;

	if(systemStatus.blUVSensorOnline == false)
		return 0;

	return SendCmd(0x06);
}

/*
  参数flg = 1，表示在户外；flg= 0；表示在室内
  注意：在室内时读到uv的数据要除以100得到真正的uv指数，然后再用于与1大小的比较和显示。
		这样做的原因是：在室内灵敏度和增益设置的较高，这样容易是读到的uv指数大于1，出现误动作。
*/
void InOutdoorChange(uint8_t flg)
{
	uint8_t visrange = 0;
	uint8_t visgain = 0;


	if(flg)//配置成大信号测量
	{
		visrange = 1;
		visgain = 0;

	}
	else//配置成小信号测量，注重灵敏度
	{
		visrange = 0;
		visgain = 2;
	}

	Si114xParamSet(PARAM_ALSVIS_ADC_GAIN,  visgain);
	Si114xParamSet(PARAM_ALSVIS_ADC_COUNTER, RECCNT_511);//上面2个值要互成补码，这里设置的没错
	Si114xParamSet(PARAM_ALSVIS_ADC_MISC, RANGE_EN * visrange); //当range范围是0时表示小信号，即在室内。

	Si114xParamSet(PARAM_ALS_ENCODING , 0x10); //取低16位ADC

	Si114xParamSet(PARAM_ADC_OFFSET, 255);//为ADC设置偏移
}



//int Si114xPauseAll(void )
//{
//	BYTE  nonused = 0;
//	while (1)
//	{
//		// Keep sending nops until the response is zero
//		while (1)
//		{
//			SI14x_ReadReg(REG_RESPONSE,&nonused);
//			if(nonused == 0)
//				break;
//			else
//				Si114xNop();
//		}
//		// Pause the device
//		PsAlsPause();
//
//		// Wait for response
//		while(1)
//		{
//			SI14x_ReadReg(REG_RESPONSE,&nonused);
//			if(nonused != 0)
//				break;
//		}
//
//		// When the PsAlsPause() response is good, we expect it to be a '1'.
//		SI14x_ReadReg(REG_RESPONSE,&nonused);
//		if(nonused == 1)
//			break;  // otherwise, start over.
//	}
//	return 0;
//}

//*****************************************************

int Si114xPauseAll(void )
{
	uint8_t  nonused = 0;
	uint8_t count = 0;

	// Keep sending nops until the response is zero
	SI14x_ReadReg(REG_RESPONSE, &nonused);

	if(nonused == 0)
	{
		PsAlsPause();	// Pause the device
	}
	else
	{
		Si114xNop();

		while((nonused != 0) && count < 3) //超时检测
		{
			count++;
			Si114xNop();
			SI14x_ReadReg(REG_RESPONSE, &nonused);
		}

		PsAlsPause();
	}

// Wait for response

	count = 0;

	SI14x_ReadReg(REG_RESPONSE, &nonused);

	while((nonused == 0) && (count < 3) ) //这里期望的响应值nonused不是0，因为上面发送了命令
	{
		count++;
		SI14x_ReadReg(REG_RESPONSE, &nonused);
	}

	// When the PsAlsPause() response is good, we expect it to be a '1'.

	SI14x_ReadReg(REG_RESPONSE, &nonused);

	if(nonused == 1)
		return 0 ;
	else// otherwise, start over.
	{
		count = 0;

		while((nonused != 1) && (count < 3) )
		{
			count++;
			SI14x_ReadReg(REG_RESPONSE, &nonused);
		}
	}

	return 0;
}


/***************************************关于UVsensor的一些命令*******************************************/




/***************************************关于UV校正的一些函数********************************************/

float decode(unsigned int input)
{
	unsigned int exponent, exponent_basis9, fraction;

	union
	{
		float  f;
		unsigned int i;
	} val;

	val.i = input;

	if(input == 0)
		return 0;

	exponent_basis9 = (val.i & 0x0f00) >> 8;
	exponent = exponent_basis9 - 9 + 0x76;
	exponent <<= 23;

	fraction = val.i & 0x00ff;
	fraction <<= 15;

	val.i =  exponent + fraction;
	return val.f;
}

float vispd_correction(uint8_t buf[])
{
	float val =  ALSVIS_ADCLO_WHSENSE_REF / ALSVIS_ADCLO_WHSENSE;

	if(val < 0.0 )
		return 1.0;
	else
		return val;
}

float irpd_correction(uint8_t buf[])
{
	float val = ALSIR_ADCLO_IRSENSE_REF / ALSIR_ADCLO_IRSENSE;

	if(val < 0.0)
	{
		return 1.0;
	}
	else
		return val;
}

float adcrange_ratio(uint8_t buf[])
{
	float valOne = ALSIR_ADCLO_IRSENSE;
	float valTwo = ALSIR_ADCHI_IRSENSE;

	if((valOne < 0.0) || (valTwo < 0.00))
		return PROX3_ADCHI_IRSENSE_REF / ALSIR_ADCHI_IRSENSE_REF;
	else
		return valOne / valTwo;
}

float irsize_ratio(uint8_t buf[])
{
	float valOne = PROX3_ADCHI_IRSENSE;
	float valTwo = ALSIR_ADCHI_IRSENSE;

	if((valOne < 0.0) || (valTwo < 0.00))
		return PROX3_ADCHI_IRSENSE_REF / ALSIR_ADCHI_IRSENSE_REF;
	else
		return valOne / valTwo;
}

float ledi_ratio(uint8_t buf[])
{
	float valOne = LED_DRV65_REF;
	float valTwo = LED_DRV65;

	if(valTwo < 0.0)
		return 1.0;
	else
		return valOne / valTwo;
}

float collect(uint8_t buf[], uint8_t msb_addr, uint8_t lsb_addr, uint8_t alignment)
{
	unsigned int val;
	uint8_t msb_ind = msb_addr - 0x22;
	uint8_t lsb_ind = lsb_addr - 0x22;

	if(alignment == 0)
	{
		val = buf[msb_ind] << 4;
		val += buf[lsb_ind] >> 4;
	}
	else
	{
		val = buf[msb_ind] << 8;
		val += buf[lsb_ind];
		val &= 0x0fff;
	}

	if((val == 0x0fff) || (val == 0x0000))
		return -1.0;
	else
		return decode(val);
}

/*获取校验数据*/
int SI14x_GetCAL(SI114X_CAL_S* si14x_cal, char secrity)
{
	uint8_t buf[12];
	INT8 retval;
	uint8_t response, i;
	//  BYTE nonused;

	/*当secrity为1时，对其接口寄存器进行判断，如果接口寄存器为0表明没有开启自治测试模式*/
	if(secrity == 1)
	{
		retval = Si114xBlockRead(REG_ALS_VIS_DATA0, 12, buf);

		if(retval != 0)//测试结果retval=0，buf[0:3]是0x10,0x01,0x41,0x01,后面的是0
			return -2;

		for(i = 0; i < 12; i++)
		{
			if(buf[i] != 0)
			{
				retval = -1;
				goto error_exit;
			}
		}
	}

	/*检测器件是否可以接受数据*/
	do
	{
		retval = Si114xNop();
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&response);
	}
	while(response != 0); //测试该返回值是0

	/*获取校验码,这句这是发生要校验数据的命令*/
	retval = SI14x_WriteReg(REG_COMMAND, 0x12);//测试，这里retval是0

	if(retval != 0)
	{
		retval = -2;
		goto error_exit;
	}

	/*等待响应寄存器增加.响应寄存器的内容
	0x80:非法命令
	0x8c:测试VIS时ADC溢出
	0x8d:测试IR时ADC溢出
	0x8e:测试AUX时ADC溢出
	*/
	do
	{
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&response);

		if(response == 0x80)//测试时这里读到的是1
		{
			Si114xNop();
			goto error_exit;
		}
		else if(response & 0xfff0)
		{
			retval = -3;
			goto error_exit;
		}
	}
	while(response != 1);

	/*从接口寄存器中获取12字节*/
	retval = Si114xBlockRead(REG_ALS_VIS_DATA0, 12, buf);

	/*
	在有覆盖物时读到的校验数据，buf中的值是：
	0x3e,0xac,0xa1,0x96,0x24,0x0b,0x7f,0x4d,0x60,0xeb,0xaf,0xff
	*/
	if(retval != 0)
	{
		retval = -2;
		goto error_exit;
	}

	si14x_cal->vispd_correction = vispd_correction(buf);
	si14x_cal->irpd_correction  = irpd_correction(buf);
	si14x_cal->adcrange_ratio   = adcrange_ratio(buf);
	si14x_cal->irsize_ratio     = irsize_ratio(buf);
	si14x_cal->ledi_ratio       = ledi_ratio(buf);
	return 0;
error_exit:
	si14x_cal->vispd_correction = 1.0;
	si14x_cal->irpd_correction  = 1.0;
	si14x_cal->adcrange_ratio   = 1.0 * ALSIR_ADCLO_IRSENSE_REF / ALSIR_ADCHI_IRSENSE_REF;
	si14x_cal->irsize_ratio     = 1.0 * PROX3_ADCHI_IRSENSE_REF / ALSIR_ADCHI_IRSENSE_REF;
	si14x_cal->ledi_ratio       = 1.0;
	return retval;

}


/*设置校验数据*/
int SI14x_SetUCOEF(uint8_t ref_ucoef[], SI114X_CAL_S* si14x_cal)
{
	uint8_t response ;
	int temp;
	//    uint8_t nonused;
	uint8_t ucoef[4] = {0x29, 0x89, 0x02, 0x00};
	float vc = 1.0, ic = 1.0;

	if(ref_ucoef != 0 )
	{
		for(temp = 0; temp < 4; temp++)
		{
			ucoef[temp] = ref_ucoef[temp];//通过传进来的数据来改写默认值
		}
	}

	SI14x_ReadReg(REG_PART_ID, &response);

	switch(response)
	{
		case 0x32:
		case 0x45:
		case 0x46:
		case 0x47:
			temp = 1;
			break;

		default:
			temp = 0;
	}

	if(!temp)
	{
		return -1;
	}

	if(si14x_cal != 0)//
	{
		if(si14x_cal->vispd_correction > 0.0)
			vc = si14x_cal->vispd_correction;

		if(si14x_cal->irpd_correction > 0.0)
			ic = si14x_cal->irpd_correction;
	}

	temp = (unsigned int)(((float)((unsigned long )ucoef[0] + ((unsigned long)ucoef[1] << 8)) * vc));
	ucoef[0] = (temp & 0x00ff);
	ucoef[1] = (temp & 0xff00) >> 8;

	temp = (unsigned int)(((float)((unsigned long )ucoef[2] + ((unsigned long)ucoef[3] << 8)) * ic));
	ucoef[2] = (temp & 0x00ff);
	ucoef[3] = (temp & 0xff00) >> 8;
	//测试发现这里ucoef[]都是0，错误了。
	response = Si114xBlockWrite(REG_UCOEF0, 4, ucoef);

	return response;
}


/***************************************关于UV校正的一些函数********************************************/



/*UV的复位*/
void UV_RST(void)
{

	SysCtlDelay(1000);
	SI14x_WriteReg(REG_MEAS_RATE, 0x00);//测试频率的设置
	Si114xPauseAll();//关掉所有功能

	SI14x_WriteReg(REG_MEAS_RATE, 0x00);
	SI14x_WriteReg(REG_IRQ_ENABLE, 0x00);
	SI14x_WriteReg(REG_INT_CFG, 0x00);
	SI14x_WriteReg(REG_IRQ_STATUS, 0xff);

	SI14x_WriteReg(REG_COMMAND, 1);//执行复位
	SysCtlDelay(1000);
	SI14x_WriteReg(REG_HW_KEY, HW_KEY_VAL0);//写入硬件key

}


void UV_Init(void)
{
	uint8_t nonused;

	SI14x_IIC_Init();//先完成Sensor和mcu通信的初始化

	SI14x_ReadReg(REG_PART_ID, &nonused); //获取ID

	systemStatus.blUVSensorOnline = false;

	if(nonused != 0x32)//经过测试这里正确的读到了该芯片的ID
		return;

	systemStatus.blUVSensorOnline = true;

	// Turn off RTC
	SI14x_WriteReg(REG_MEAS_RATE, 0);
	SI14x_WriteReg(REG_ALS_RATE, 0);

	UV_RST();//软件复位

	si114x_default_ucoef();//设置默认的UV校正系数

	Si114xParamSet(PARAM_CH_LIST, UV_TASK | ALS_VIS_TASK); //这里选择测试UV,ALS

	//InOutdoorChange(0); //  参数flg = 1，表示在户外；flg= 0；表示在室内
	InOutdoorChange(1); //  参数flg = 1，表示在户外；flg= 0；表示在室内

}


uint8_t GetUVindex(void)
{
	uint8_t t8_l=0, t8_h=0; //[BG023-2] add initialized
	int ret = 0; //[BG023-2] add for check return status.

	if(systemStatus.blUVSensorOnline == false)
		return 0;

	ret |= SI14x_ReadReg(REG_AUX_DATA0, &t8_l); /*读UV指数*/
	ret |= SI14x_ReadReg(REG_AUX_DATA1, &t8_h);
	//Atus: [BG023-2] should check read fail condition.
	if (ret != i2cTransferDone) //[BG023-2] add.
	{
	 	return bUltraViolet; //return last value.
	}

	float uvIndex = (1.0 * (t8_h * 256 + t8_l)) / 100.0;

	if(systemSetting.deviceColor == DEVICE_COLOR_BLACK)
		uvIndex = SLOPE_B * uvIndex + INTERCEPT_B ;
	else
		//uvIndex = SLOPE_R * uvIndex + INTERCEPT_R ; //[BG023-2] only BLACK formula. Need be fixed.
		uvIndex = SLOPE_B * uvIndex + INTERCEPT_B ;

	return((uint8_t)(uvIndex*10 + 5)/10);
}

uint16_t GetAmbLight(void)
{
	uint8_t t8_l, t8_h;

	if(systemStatus.blUVSensorOnline == false)
		return 0;

	SI14x_ReadReg(REG_ALS_VIS_DATA0 , &t8_l);/*读ALS指数*/
	SI14x_ReadReg(REG_ALS_VIS_DATA1 , &t8_h);

	return(t8_h * 256 + t8_l);

}


