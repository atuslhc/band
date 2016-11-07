#include "FreeRTOS.h"
#include "Si14x.h"
#include "I2c_driver.h"
#include "em_gpio.h"
#include "task.h"
#include "GlobalData.h"
#include "common_vars.h"
#include "main.h"


/********�й�У���һЩ�궨��***************************************/
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


/********��������***************************************/

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

		/*������������ʹIIC�����ȶ�*/
		for (i = 0; i < 9; i++)
		{
			GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModeWiredAnd, 0);
			GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModeWiredAnd, 1);
		}

		/*ȷ��IIC���������Ŷ������*/
		GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SCL_PIN, gpioModeWiredAnd, 1);//���ó�����(Ҳ���ǿ�©) ������1
		GPIO_PinModeSet(SI14x_I2C_gpioPort, SI14x_I2C_SDA_PIN, gpioModeWiredAnd, 1);

		/* Enable pins at location 3*/
		SI14x_I2C->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | SI14x_I2C_LOC;
		I2C_Init(SI14x_I2C, &i2cInit);//��һ��������ָ�룬��������ѱ���ȡ��ַ���롣

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

/*���ջ�ȡ��������data�д洢*/
int SI14x_ReadReg(uint8_t Reg, uint8_t* data)
{
	uint8_t ret;


	// =============================================================
	// ���i2c�ź���
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // ��ȡ�ź���ʧ��

	if (xSemaphoreTake(hI2CSemaphore, 20) != 1)
		return i2cTransferSwFault;

	// =============================================================
	ret = I2CReadNByte(SI14x_I2C, SIl4x_IIC_ADDR, I2C_SUBA_ONEBYTE, Reg, data, 1);


	// =============================================================
	// �ͷ�I2C�ź���
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

/*д�ɹ�����0��ʧ�ܷ���0xff*/
int SI14x_WriteReg(uint8_t WriteAddr, uint8_t Data)
{
	int ret;


	// =============================================================
	// ���i2c�ź���
//	if (osSemaphoreWait(hI2CSemaphore, osWaitForever) != 1)
//		return i2cTransferSwFault; // ��ȡ�ź���ʧ��

	if (xSemaphoreTake(hI2CSemaphore, 20) != pdTRUE)
		return i2cTransferSwFault;


	// =============================================================
	ret = I2CWriteNByte(SI14x_I2C, SIl4x_IIC_ADDR, I2C_SUBA_ONEBYTE, WriteAddr, &Data, 1);


	// =============================================================
	// �ͷ�I2C�ź���
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

//��д�룬�������ò���
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
		//��������ݷ���value�����С�
		SI14x_ReadReg(address + counter, values);
		values++;
	}

	return 0;
}

//��RAM������address��ַ��д��valueֵ
int Si114xParamSet(uint8_t address, uint8_t value)
{
	INT8  retval;
	uint8_t cmd, response;
	//   BYTE nonused = 0;

	// Wait for any prior command to complete
	if((retval = WaitUntilSleep(SIl4x_IIC_ADDR)) != 0 ) return retval;

	// Read RESPONSE register
	//�����ȫ�����ö���Ӧ�Ĵ���

	SI14x_ReadReg(REG_RESPONSE, &response); //���������ݷ���response��
	// Parameter[address] = value
	cmd = 0xA0 + (address & 0x1F);

	//�������ȶ�RAM����д�Ĵ�����д��ֵvalue��Ȼ��������Ĵ�����д������cmd��
	// Fill PARAM_WR register
	if((retval = SI14x_WriteReg(REG_PARAM_WR, value)) != 0 )  return retval ;

	// Issue COMMAND register
	if((retval = SI14x_WriteReg(REG_COMMAND, cmd)) != 0 )  return retval ;

	// Wait for command to finish
	SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&retval);
	//   while(retval == response);

	//д���������Ӧ�Ĵ�����仯�ģ����һ�ζ��������ݲ�һ��������������˳��ˣ���������д����ȷ��
	if (retval < 0 )
		return retval;
	else
		return 0;
}


// Sends the Parameter Read command to the SI114x to read and return Parameter values
// The Si114x incremenets the RESPONSE register upon the completion of a COMMAND
//  so the code reads the RESPONSE register, issues the COMMAND, then waits
//  for the RESPONSE register to complete before proceeding.

//�ú�����ȡ������address�ϵ�ֵ������data��
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

/*�ú�����ֱ����sensor�Ĵ�����д��Ĭ�ϵ�У��ֵ��������У��ֵ���Ȼ�ȡУ�����ݣ��ڼ��㣬����ȡһ���µ�У�����ݣ������Ȳ�������*/
int si114x_default_ucoef(void)
{
	int response;
	uint8_t  ucoef[4] = { 0x29, 0x89, 0x02, 0x00 } ;

	// This will write 4 bytes starting with I2C address 0x13
	response = Si114xBlockWrite( REG_UCOEF0, 4, &ucoef[0] );
	return response;
}


/***************************************����UVsensor��һЩ����*******************************************/

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
//	    else //�����Ӧ������Ĳ�һ�������г�ʱ�ȴ�
//		{
//		  while((retval != response) && count < 3)
//			{
//			  	response = retval;
//				count++;
//				SI14x_ReadReg(REG_RESPONSE,(BYTE*) &retval);//�ٴζ�
//			}
//		  break;//�˳�while
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
//		else if(temp == response)//���������������������Ӧֵ��տ�ʼ��һ����˵�������⣬������г�ʱ
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
	SI14x_ReadReg(REG_RESPONSE, (uint8_t*) &response);//��ALS���ʱRESPONSE�Ĵ�������0x8c,
//��ǰresponse��������int8��������϶�Ϊ���������˳��ˣ���ִ���������,�����������ʾ-1�ز�ȥ��

	if (response < 0)
		return response;

	// Double-check the response register is consistent


	if((retval = WaitUntilSleep(si114x_handle)) != 0) return retval;

	if(command != 0) // Skip if the command is NOP
	{
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*) &retval);

		if(retval == response) //���������״̬�����һ����˵���豸���п���ֱ�ӷ�������
		{
			if ((retval = SI14x_WriteReg(REG_COMMAND, command)) != 0)
				return retval;
		}
		else if(retval < 0) //���������״̬�Ǹ���������ֱ���˳�
			return retval;
		else //�����Ӧ������Ĳ�һ�������г�ʱ�������˳���ʱʱ���Ƿ���һ������
		{
			while((retval != response) && count < 3)
			{
				response = retval;
				count++;
				SI14x_ReadReg(REG_RESPONSE, (uint8_t*) &retval); //�ٴζ�
			}

			if ((retval = SI14x_WriteReg(REG_COMMAND, command)) != 0)
				return retval;
		}
	}
	else//���������0   Send the Command
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
		else if(temp == response)//���������������������Ӧֵ��տ�ʼ��һ����˵�������⣬�����ٴλ�ȡ״̬
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
  ����flg = 1����ʾ�ڻ��⣻flg= 0����ʾ������
  ע�⣺������ʱ����uv������Ҫ����100�õ�������uvָ����Ȼ����������1��С�ıȽϺ���ʾ��
		��������ԭ���ǣ������������Ⱥ��������õĽϸߣ����������Ƕ�����uvָ������1������������
*/
void InOutdoorChange(uint8_t flg)
{
	uint8_t visrange = 0;
	uint8_t visgain = 0;


	if(flg)//���óɴ��źŲ���
	{
		visrange = 1;
		visgain = 0;

	}
	else//���ó�С�źŲ�����ע��������
	{
		visrange = 0;
		visgain = 2;
	}

	Si114xParamSet(PARAM_ALSVIS_ADC_GAIN,  visgain);
	Si114xParamSet(PARAM_ALSVIS_ADC_COUNTER, RECCNT_511);//����2��ֵҪ���ɲ��룬�������õ�û��
	Si114xParamSet(PARAM_ALSVIS_ADC_MISC, RANGE_EN * visrange); //��range��Χ��0ʱ��ʾС�źţ��������ڡ�

	Si114xParamSet(PARAM_ALS_ENCODING , 0x10); //ȡ��16λADC

	Si114xParamSet(PARAM_ADC_OFFSET, 255);//ΪADC����ƫ��
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

		while((nonused != 0) && count < 3) //��ʱ���
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

	while((nonused == 0) && (count < 3) ) //������������Ӧֵnonused����0����Ϊ���淢��������
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


/***************************************����UVsensor��һЩ����*******************************************/




/***************************************����UVУ����һЩ����********************************************/

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

/*��ȡУ������*/
int SI14x_GetCAL(SI114X_CAL_S* si14x_cal, char secrity)
{
	uint8_t buf[12];
	INT8 retval;
	uint8_t response, i;
	//  BYTE nonused;

	/*��secrityΪ1ʱ������ӿڼĴ��������жϣ�����ӿڼĴ���Ϊ0����û�п������β���ģʽ*/
	if(secrity == 1)
	{
		retval = Si114xBlockRead(REG_ALS_VIS_DATA0, 12, buf);

		if(retval != 0)//���Խ��retval=0��buf[0:3]��0x10,0x01,0x41,0x01,�������0
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

	/*��������Ƿ���Խ�������*/
	do
	{
		retval = Si114xNop();
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&response);
	}
	while(response != 0); //���Ը÷���ֵ��0

	/*��ȡУ����,������Ƿ���ҪУ�����ݵ�����*/
	retval = SI14x_WriteReg(REG_COMMAND, 0x12);//���ԣ�����retval��0

	if(retval != 0)
	{
		retval = -2;
		goto error_exit;
	}

	/*�ȴ���Ӧ�Ĵ�������.��Ӧ�Ĵ���������
	0x80:�Ƿ�����
	0x8c:����VISʱADC���
	0x8d:����IRʱADC���
	0x8e:����AUXʱADC���
	*/
	do
	{
		SI14x_ReadReg(REG_RESPONSE, (uint8_t*)&response);

		if(response == 0x80)//����ʱ�����������1
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

	/*�ӽӿڼĴ����л�ȡ12�ֽ�*/
	retval = Si114xBlockRead(REG_ALS_VIS_DATA0, 12, buf);

	/*
	���и�����ʱ������У�����ݣ�buf�е�ֵ�ǣ�
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


/*����У������*/
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
			ucoef[temp] = ref_ucoef[temp];//ͨ������������������дĬ��ֵ
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
	//���Է�������ucoef[]����0�������ˡ�
	response = Si114xBlockWrite(REG_UCOEF0, 4, ucoef);

	return response;
}


/***************************************����UVУ����һЩ����********************************************/



/*UV�ĸ�λ*/
void UV_RST(void)
{

	SysCtlDelay(1000);
	SI14x_WriteReg(REG_MEAS_RATE, 0x00);//����Ƶ�ʵ�����
	Si114xPauseAll();//�ص����й���

	SI14x_WriteReg(REG_MEAS_RATE, 0x00);
	SI14x_WriteReg(REG_IRQ_ENABLE, 0x00);
	SI14x_WriteReg(REG_INT_CFG, 0x00);
	SI14x_WriteReg(REG_IRQ_STATUS, 0xff);

	SI14x_WriteReg(REG_COMMAND, 1);//ִ�и�λ
	SysCtlDelay(1000);
	SI14x_WriteReg(REG_HW_KEY, HW_KEY_VAL0);//д��Ӳ��key

}


void UV_Init(void)
{
	uint8_t nonused;

	SI14x_IIC_Init();//�����Sensor��mcuͨ�ŵĳ�ʼ��

	SI14x_ReadReg(REG_PART_ID, &nonused); //��ȡID

	systemStatus.blUVSensorOnline = false;

	if(nonused != 0x32)//��������������ȷ�Ķ����˸�оƬ��ID
		return;

	systemStatus.blUVSensorOnline = true;

	// Turn off RTC
	SI14x_WriteReg(REG_MEAS_RATE, 0);
	SI14x_WriteReg(REG_ALS_RATE, 0);

	UV_RST();//�����λ

	si114x_default_ucoef();//����Ĭ�ϵ�UVУ��ϵ��

	Si114xParamSet(PARAM_CH_LIST, UV_TASK | ALS_VIS_TASK); //����ѡ�����UV,ALS

	//InOutdoorChange(0); //  ����flg = 1����ʾ�ڻ��⣻flg= 0����ʾ������
	InOutdoorChange(1); //  ����flg = 1����ʾ�ڻ��⣻flg= 0����ʾ������

}


uint8_t GetUVindex(void)
{
	uint8_t t8_l=0, t8_h=0; //[BG023-2] add initialized
	int ret = 0; //[BG023-2] add for check return status.

	if(systemStatus.blUVSensorOnline == false)
		return 0;

	ret |= SI14x_ReadReg(REG_AUX_DATA0, &t8_l); /*��UVָ��*/
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

	SI14x_ReadReg(REG_ALS_VIS_DATA0 , &t8_l);/*��ALSָ��*/
	SI14x_ReadReg(REG_ALS_VIS_DATA1 , &t8_h);

	return(t8_h * 256 + t8_l);

}


