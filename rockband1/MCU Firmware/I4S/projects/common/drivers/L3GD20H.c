#include <stdlib.h>

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_int.h"

#include "common_vars.h"
#include "main.h"
#include "mems.h"

#include "device_task.h"
#include "mems_tracking.h" 

#include "L3GD20H.h"
///#include "mems.h"

#if (GYRO_SUPPORT==1)

//bool L3GD20H_ONLINE=false;
bool L3GD20H_Monitor_Model = false;

int16_t L3GD20H_BUFF[3][L3GD20H_BUFF_SIZE];

#define L3GD20H_FIFO_FREQ 50

//=========================
#define L3GD20H_TIME_WINDOW        L3GD20H_FIFO_FREQ 
#define L3GD20H_NOISE_TH           10
#define L3GD20H_ACT_THRES          6

uint8_t L3GD20H_FRQ;
int  L3GD20H_XYZ_DC[3];
uint8_t L3GD20H_TEMP;
//============================
gyrostatus_t L3GD20H_ReadReg(u8_t Reg, u8_t* Data);
gyrostatus_t L3GD20H_WriteReg(u8_t WriteAddr, u8_t Data);

void ReadL3GD20HRawData(uint8_t* p);

void L3GD20H_Par_Init(uint8_t freq);
void SleepWork(void);


void L3GD20H_Par_Init(uint8_t freq)
{
/*
	val_threshold = mems_P2P_threshold;
	Variance_Val_threshold = 12;//170; // v1.5-3=100
	L3GD20H_FRQ = freq;
	//pos_max_interval = (int16_t)(L3GD20H_FRQ * 1.2);
	pos_max_interval = (int16_t) L3GD20H_FRQ;
	pos_min_interval = (int16_t)(L3GD20H_FRQ >> 3);
	which_ax_near_zero = 0;

	Mems_TimeCount = 0;
	active_level_bak = active_level;
#if BG009
  if (active_level_max < active_level)
      active_level_max = active_level;
  if (active_level_min > active_level)
      active_level_min = active_level;
#endif
*/
}


void L3GD20H_FIFO_INIT(void)
{
	uint8_t iswhere;


	L3GD20H_SetOD(L3GD20H_CTRL1_DR_800_HZ);
	//L3GD20H_SetOD(L3GD20H_CTRL1_DR_12_5_HZ);
	L3GD20H_SetFullScale(L3GD20H_CTRL4_FS_245DPS); 

	L3GD20H_WriteReg(L3GD20H_O_CTRL2, L3GD20H_CTRL2_EXTREN_DIS);

	L3GD20H_WriteReg(L3GD20H_O_CTRL3, 0x00);
	L3GD20H_SetInt2Pin(L3GD20H_CTRL3_INT2_ORUN_EN);



	
	L3GD20H_FIFOModeEnable(L3GD20H_FIFO_Bypass2FIFO_mode); 
	L3GD20H_WriteReg(L3GD20H_O_CTRL5, L3GD20H_CTRL5_FIFOCTL_EN);
	
	///L3GD20H_WriteReg(L3GD20H_O_LOW_ODR, 0x08);

	GPIO_IntClear(1 << L3GD20H_INT1_PIN);
	GPIO_IntClear(1 << L3GD20H_INT2_PIN);

	L3GD20H_WriteReg(L3GD20H_O_LOW_ODR,
		L3GD20H_LOW_ODR_DRDY_HL_ACTIVE_HIGH |
		L3GD20H_LOW_ODR_DATARATE_LOW |
		L3GD20H_LOW_ODR_I2C_DISABLE_SPI_ONLY);

	L3GD20H_SetODR(ODR_50);

	L3GD20H_SetPOWERMode(L3GD20H_POWER_NORMAL);

	/// ReadL3GD20HRawData((uint8_t*)&L3GD20H_BUFF[0][0]);  // clear the old buff

#if	0
	L3GD20H_ReadReg(L3GD20H_O_IG_SRC, &iswhere);

	L3GD20H_Monitor_Model = false;
	L3GD20H_Par_Init(L3GD20H_FIFO_FREQ);
	XYZ_FIR_INIT(L3GD20H_FIFO_FREQ);
#endif

#if FALL_DETECT_SUPPORT		

	int i;

	for (i = 0 ; i < AXIS_FILTER_BUFF_SIZE ; i++)
	{
		axis_filter[i] = 0;
	}


	axis_filter_index = 0;
#endif
}

void L3GD20H_Disabled(void)
{
    if (systemStatus.blGyroSensorOnline==0x01) //&& 0
    {
        L3GD20H_SetPOWERMode(L3GD20H_POWER_DOWN);
    }
    systemStatus.blGyroSensorOnline = false;

	GPIO_IntConfig(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, false, false, false);
	GPIO_IntClear(1 << L3GD20H_INT1_PIN); 	
    GPIO_PinModeSet(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, gpioModeDisabled, 1);

	GPIO_IntConfig(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, false, false, false);	
	GPIO_IntClear(1 << L3GD20H_DRDY_PIN); 	
	GPIO_PinModeSet(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, gpioModeDisabled, 1);

	GPIO_PinModeSet(GYRO_CS_PORT, GYRO_CS_PIN, gpioModeDisabled, 1);
}

int L3GD20H_Init(uint8_t mode)
{

	uint8_t buf;
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(L3GD20H_DEN_PORT, L3GD20H_DEN_PIN, gpioModePushPull, 1);
	GPIO_PinOutClear(L3GD20H_DEN_PORT,L3GD20H_DEN_PIN );    //connect gnd if not used.
	GPIO_PinModeSet(GYRO_CS_PORT, GYRO_CS_PIN, gpioModePushPull, 1);
#if 0
    //config INT1 interrupt if need
	GPIO_PinModeSet(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, gpioModeInput, 1);
	GPIO_IntConfig(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, false, true, true);
	GPIO_IntClear(1 << L3GD20H_INT1_PIN); 	
	NVIC_SetPriority(GPIO_ODD_IRQn,GPIO_ODD_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	//config DRDY/INT2 interrupt if need
	GPIO_PinModeSet(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, gpioModeInput, 1);
	GPIO_IntConfig(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, false, true, true);	
	GPIO_IntClear(1 << L3GD20H_DRDY_PIN); 	
	NVIC_SetPriority(GPIO_EVEN_IRQn,GPIO_EVEN_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif
	L3GD20H_GetWHO_AM_I(&buf);
    if (buf != L3GD20H_ID)
    {
		systemStatus.blGyroSensorOnline = false;
        return DEVICE_NOTEXIST;
    }
    
    /* the sensor L3GD20H validated, software reset it. */
    //systemStatus.blGyroSensorOnline = 1;
		
    L3GD20H_WriteReg(L3GD20H_O_LOW_ODR, L3GD20H_LOW_ODR_SWRESET_RESET);
    SysCtlDelay(4000);
	/// L3GD20H_ReadReg(L3GD20H_O_FIFO_SRC,&buf);
	L3GD20H_SetPOWERMode(L3GD20H_POWER_DOWN);

    if (mode==0x00)
    {
        L3GD20H_Disabled();
        return DEVICE_SUCCESS;
    }
    
    //config INT1 interrupt if need
	GPIO_PinModeSet(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, gpioModeInput, 1);
	GPIO_IntConfig(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, false, true, true);
	GPIO_IntClear(1 << L3GD20H_INT1_PIN); 	
	NVIC_SetPriority(GPIO_ODD_IRQn,GPIO_ODD_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	//config DRDY/INT2 interrupt if need
	GPIO_PinModeSet(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, gpioModeInput, 1);
	GPIO_IntConfig(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, false, true, true);	
	GPIO_IntClear(1 << L3GD20H_DRDY_PIN); 	
	NVIC_SetPriority(GPIO_EVEN_IRQn,GPIO_EVEN_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    
	L3GD20H_FIFO_INIT();
	/// L3GD20H_ReadReg(L3GD20H_O_FIFO_SRC,&buf);
    systemStatus.blGyroSensorOnline = 1;
    
    return DEVICE_SUCCESS;
}

#if 1
void L3GD20H_MOTION_MONITOR(uint8_t which_ax)
{
	uint8_t iswhere;
	L3GD20H_SetOD(L3GD20H_CTRL1_DR_800_HZ);
	L3GD20H_SetPOWERMode(L3GD20H_POWER_DOWN);
	L3GD20H_SetFullScale(L3GD20H_CTRL4_FS_245DPS);
	L3GD20H_SetAxis(L3GD20H_X_ENABLE | L3GD20H_Y_ENABLE | L3GD20H_Z_ENABLE);
	
	///L3GD20H_WriteReg(L3GD20H_CTRL_REG6, L3GD20H_INT_ACTIVE_LOW + L3GD20H_I2_INT1_ON_PIN_INT2_ENABLE); //fallingEdge

	uint8_t t_8 = abs(L3GD20H_XYZ_DC[which_ax]) >> 6;

	if(t_8 < 6)
		t_8 = 6;

	///L3GD20H_WriteReg(L3GD20H_INT1_THS, t_8); // Threshold = 250 mg

	switch(which_ax)
	{
		case 0:
			///L3GD20H_WriteReg(L3GD20H_INT1_CFG, 0x02);
			break;

		case 1:
			///L3GD20H_WriteReg(L3GD20H_INT1_CFG, 0x08);
			break;

		case 2:
			///L3GD20H_WriteReg(L3GD20H_INT1_CFG, 0x20);
			break;
	}
	
	L3GD20H_FIFOModeEnable(L3GD20H_FIFO_Bypass_mode);
	
	L3GD20H_SetInt2Pin(L3GD20H_CTRL3_INT2_ORUN_DIS);
	L3GD20H_WriteReg(L3GD20H_O_CTRL3, 0x80);

// L3GD20H interrupt
	GPIO_IntConfig(L3GD20H_DRDY_PORT, L3GD20H_DRDY_PIN, false, true , false); //fallingEdge
	GPIO_IntClear(1 << L3GD20H_INT1_PORT);
	GPIO_IntConfig(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, false, true , true); //fallingEdge
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	
	L3GD20H_ReadReg(L3GD20H_O_IG_SRC, &iswhere);
	L3GD20H_Monitor_Model = true;
}
#else
void L3GD20H_MOTION_MONITOR(void)
{
	L3GD20H_WriteReg(L3GD20H_INT1_CFG, 0x3f);
	L3GD20H_WriteReg(L3GD20H_INT1_CFG, 0);
	GPIO_IntConfig(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, false, true , false); //fallingEdge
	GPIO_IntClear(1 << L3GD20H_INT2_PIN);
	GPIO_IntConfig(L3GD20H_INT2_PORT, L3GD20H_INT2_PIN, false, true , true); //fallingEdge
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	L3GD20H_Monitor_Model = true;
}
#endif

void L3GD20H_NO_FIFO_INIT(void)
{
	uint8_t iswhere;
	
	/* Disable interrupts */
	INT_Disable();
// L3GD20H interrupt
	GPIO_IntConfig( L3GD20H_DRDY_PORT,  L3GD20H_DRDY_PIN, false, true , false); //fallingEdge
	GPIO_IntConfig(L3GD20H_INT1_PORT, L3GD20H_INT1_PIN, false, true , false); //fallingEdge
	GPIO_IntClear(1 <<  L3GD20H_DRDY_PIN);
	GPIO_IntClear(1 << L3GD20H_INT1_PIN);
	/* Initialization done, enable interrupts globally. */
	INT_Enable();
	//===================================
	L3GD20H_SetOD(L3GD20H_CTRL1_DR_800_HZ);
	L3GD20H_SetPOWERMode(L3GD20H_POWER_NORMAL);
	L3GD20H_SetFullScale(L3GD20H_CTRL4_FS_245DPS);
	L3GD20H_FIFOModeEnable(L3GD20H_FIFO_Bypass_mode);
	L3GD20H_WriteReg(L3GD20H_O_CTRL3, 0x00);//without  filter
	L3GD20H_SetAxis(L3GD20H_X_ENABLE | L3GD20H_Y_ENABLE | L3GD20H_Z_ENABLE);
	L3GD20H_ReadReg(L3GD20H_O_FIFO_SRC, &iswhere);
	L3GD20H_Monitor_Model = true;
	L3GD20H_Par_Init(PPG_SAM_FREQ);
	active_level_delta = 0;
}

void L3GD20H_INT1_CALLBACK(void)
{
	//// MESSAGE_TYPES
	int32_t msg = MESSAGE_GYRO_INT;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
}

void L3GD20H_INT2_CALLBACK(void)
{
	int32_t msg = MESSAGE_GYRO_DRDY;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
}

void L3GD20H_CLOSE(void)
{
	L3GD20H_SetPOWERMode(L3GD20H_POWER_DOWN);
}


void L3GD20H_OPEN(void)
{
	L3GD20H_SetPOWERMode(L3GD20H_POWER_NORMAL);
}

void ReadL3GD20HTEMPData(void)
{
	L3GD20H_ReadReg(L3GD20H_O_OUT_TEMP, &L3GD20H_TEMP);
//The value is expressed as two¡¦s complement.
//-40 +85 ¢XC
}

void ReadL3GD20HRawData(uint8_t* p)
{
	unsigned char i;
	L3GD20H_CS_L();
	USART_Tx(L3GD20H_SPI, 0x80 + 0x40 + L3GD20H_O_OUT_X_LSB);
	USART_Rx(L3GD20H_SPI);

	for(i = 0; i < 6; i++)
	{
		USART_Tx(L3GD20H_SPI, 0);
		p[i] = USART_Rx(L3GD20H_SPI);
	}

	L3GD20H_CS_H();
}

gyrostatus_t L3GD20H_ReadReg(u8_t Reg, u8_t* Data)
{
	L3GD20H_CS_L();
	USART_Tx(L3GD20H_SPI, 0x80 + Reg);
	uint8_t temp = USART_Rx(L3GD20H_SPI);
	USART_Tx(L3GD20H_SPI, 0);
	*Data = USART_Rx(L3GD20H_SPI);
	L3GD20H_CS_H();
	return L3GD20H_SUCCESS;
}

void ReadL3GD20HFIFO(uint8_t* px, uint8_t* py, uint8_t* pz, uint8_t len)
{
	unsigned char i;

    if (systemStatus.blGyroSensorOnline==false)
        return;
    
	L3GD20H_ReadReg(L3GD20H_O_FIFO_SRC,&i);
	
	L3GD20H_CS_L();
	USART_Tx(L3GD20H_SPI, 0x80 + 0x40 + L3GD20H_O_OUT_X_LSB);
	USART_Rx(L3GD20H_SPI);

	for(i = 0; i < len; i++)
	{
		USART_Tx(L3GD20H_SPI, 0);
		*(px++) = USART_Rx(L3GD20H_SPI);
		USART_Tx(L3GD20H_SPI, 0);
		*(px++) = USART_Rx(L3GD20H_SPI);
		USART_Tx(L3GD20H_SPI, 0);
		*(py++) = USART_Rx(L3GD20H_SPI);
		USART_Tx(L3GD20H_SPI, 0);
		*(py++) = USART_Rx(L3GD20H_SPI);
		USART_Tx(L3GD20H_SPI, 0);
		*(pz++) = USART_Rx(L3GD20H_SPI);
		USART_Tx(L3GD20H_SPI, 0);
		*(pz++) = USART_Rx(L3GD20H_SPI);
	}

	L3GD20H_CS_H();
	L3GD20H_ReadReg(L3GD20H_O_FIFO_SRC,&i);
}


gyrostatus_t L3GD20H_WriteReg(u8_t WriteAddr, u8_t Data)
{
	L3GD20H_CS_L();
	USART_Tx(L3GD20H_SPI, WriteAddr);
	USART_Rx(L3GD20H_SPI);
	USART_Tx(L3GD20H_SPI, Data);
	USART_Rx(L3GD20H_SPI);
	L3GD20H_CS_H();
	return L3GD20H_SUCCESS;
}

gyrostatus_t L3GD20H_GetWHO_AM_I(u8_t* val)
{
	return(L3GD20H_ReadReg(L3GD20H_O_WHOAMI, val));
}

gyrostatus_t L3GD20H_SetOD(u8_t ov)
{
		u8_t value;
	
		if(!L3GD20H_ReadReg(L3GD20H_O_CTRL1, &value))
			return L3GD20H_ERROR;
	
		value &= (~L3GD20H_CTRL1_DR_M);
		///value |= (ov *0x40);
		value |= ov;
	
		if(!L3GD20H_WriteReg(L3GD20H_O_CTRL1, value))
			return L3GD20H_ERROR;
		
		return L3GD20H_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_SetODR
* Description    : Sets L3GD20H Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_SetODR(u8_t ov)
{
	u8_t value,Low_ODR,OD_value;

	switch(ov)
	{
		case ODR_12_5:
			Low_ODR = 1;
			OD_value = L3GD20H_CTRL1_DR_12_5_HZ;;
			break;
		case ODR_25:
			Low_ODR = 1;
			OD_value = L3GD20H_CTRL1_DR_25_HZ;
			break;
		case ODR_50:
			Low_ODR = 1;
			OD_value = L3GD20H_CTRL1_DR_50_HZ;
			break;
		case ODR_100:
			Low_ODR = 0;
			OD_value = L3GD20H_CTRL1_DR_12_5_HZ;
			break;
		case ODR_200:
			Low_ODR = 0;
			OD_value = L3GD20H_CTRL1_DR_25_HZ;
			break;
		case ODR_400:
			Low_ODR = 0;
			OD_value = L3GD20H_CTRL1_DR_50_HZ;
			break;
		case ODR_800:
			Low_ODR = 0;
			OD_value = L3GD20H_CTRL1_DR_800_HZ;
			break;
		default:
			return L3GD20H_ERROR;
	}

	if(!L3GD20H_ReadReg(L3GD20H_O_LOW_ODR, &value))
		return L3GD20H_ERROR;

	value &= (~L3GD20H_LOW_ODR_DATARATE_M);
	value |= Low_ODR;

	if(!L3GD20H_WriteReg(L3GD20H_O_LOW_ODR, value))
		return L3GD20H_ERROR;
	
	if(!L3GD20H_ReadReg(L3GD20H_O_CTRL1, &value))
		return L3GD20H_ERROR;

	value &= (~L3GD20H_CTRL1_DR_M);
	value |= OD_value;

	if(!L3GD20H_WriteReg(L3GD20H_O_CTRL1, value))
		return L3GD20H_ERROR;
#if	0

	u8_t value;

	if(!L3GD20H_ReadReg(L3GD20H_O_CTRL1, &value))
		return L3GD20H_ERROR;

	value &= (~L3GD20H_CTRL1_DR_M);
	///value |= (ov *0x40);
	value |= ov;

	if(!L3GD20H_WriteReg(L3GD20H_O_CTRL1, value))
		return L3GD20H_ERROR;
#endif
	return L3GD20H_SUCCESS;
}

/*******************************************************************************
* Function Name  : L3GD20H_SetPOWERMode
* Description    : Sets L3GD20H Operating Mode
* Input          : Modality (L3GD20H_NORMAL, L3GD20H_LOW_POWER, L3GD20H_POWER_DOWN)
* Output         : None
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_SetPOWERMode(L3GD20H_Mode_t md)
{
	u8_t value;
	//u8_t ODR_old_value;

	if(!L3GD20H_ReadReg(L3GD20H_O_CTRL1, &value))
		return L3GD20H_ERROR;


	value = value& (~L3GD20H_CTRL1_POWER_M); 

	switch(md)
	{
		case L3GD20H_POWER_DOWN:
			//ODR_old_value = value;
			value |= (L3GD20H_CTRL1_POWER_LOWPOW);
			break;

		case L3GD20H_POWER_NORMAL:
			value |= (L3GD20H_CTRL1_POWER_NORMAL);
			break;

		case L3GD20H_POWER_Sleep:
			value |= (L3GD20H_CTRL1_POWER_LOWPOW);
			break;

		default:
			return L3GD20H_ERROR;
	}

	if(!L3GD20H_WriteReg(L3GD20H_O_CTRL1, value))
		return L3GD20H_ERROR;

	return L3GD20H_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_SetFullScale
* Description    : Sets the L3GD20H FullScale
* Input          : L3GD20H_FULLSCALE_2/L3GD20H_FULLSCALE_4/L3GD20H_FULLSCALE_8/L3GD20H_FULLSCALE_16
* Output         : None
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_SetFullScale(uint8_t fs)
{
	u8_t value;

	if(!L3GD20H_ReadReg(L3GD20H_O_CTRL4, &value))
		return L3GD20H_ERROR;

	value &= (~L3GD20H_CTRL4_FS_M);
	value |= fs;//L3GD20H_CTRL4_FS_245DPS;

	if(!L3GD20H_WriteReg(L3GD20H_O_CTRL4, value))
		return L3GD20H_ERROR;

	return L3GD20H_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_SetAxis
* Description    : Enable/Disable L3GD20H Axis
* Input          : L3GD20H_X_ENABLE/DISABLE | L3GD20H_Y_ENABLE/DISABLE | L3GD20H_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_SetAxis(L3GD20H_Axis_t axis)
{
	u8_t value;

	if(!L3GD20H_ReadReg(L3GD20H_O_CTRL1, &value))
		return L3GD20H_ERROR;

	value &= 0xF8;
	value |= (0x07 & axis);

	if(!L3GD20H_WriteReg(L3GD20H_O_CTRL1, value))
		return L3GD20H_ERROR;

	return L3GD20H_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_SetInt2Pin
* Description    : Set Interrupt1 pin Function
* Input          :  L3GD20H_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | L3GD20H_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |
L3GD20H_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | L3GD20H_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |
L3GD20H_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | L3GD20H_WTM_ON_INT1_ENABLE/DISABLE         |
L3GD20H_INT1_OVERRUN_ENABLE/DISABLE
* example        : SetInt1Pin(L3GD20H_CLICK_ON_PIN_INT1_ENABLE | L3GD20H_I1_INT1_ON_PIN_INT1_ENABLE |
L3GD20H_I1_INT2_ON_PIN_INT1_DISABLE | L3GD20H_I1_DRDY1_ON_INT1_ENABLE | L3GD20H_I1_DRDY2_ON_INT1_ENABLE |
L3GD20H_WTM_ON_INT1_DISABLE | L3GD20H_INT1_OVERRUN_DISABLE   )
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_SetInt2Pin(L3GD20H_IntPinConf_t pinConf)
{
	u8_t value=0;

	if(!L3GD20H_ReadReg(L3GD20H_O_CTRL3, &value))
		return L3GD20H_ERROR;
	
	if(L3GD20H_CTRL3_INT2_ORUN_EN == pinConf)
		{
	value = L3GD20H_CTRL3_INT2_ORUN_EN|
		///L3GD20H_CTRL3_INT2_DRDY_EN|   //over run mode, not data ready mode(no fifo).
		L3GD20H_CTRL3_H_LACTIVE_HI|
		L3GD20H_CTRL3_DRIVE_TYPE_OD;
		}

	if(!L3GD20H_WriteReg(L3GD20H_O_CTRL3, value))
		return L3GD20H_ERROR;

	return L3GD20H_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : L3GD20H_FIFO_DISABLE, L3GD20H_FIFO_BYPASS_MODE, L3GD20H_FIFO_MODE,
L3GD20H_FIFO_STREAM_MODE, L3GD20H_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_FIFOModeEnable(L3GD20H_FifoMode_t fm)
{
	u8_t value;



	if( !L3GD20H_ReadReg(L3GD20H_O_FIFO_CTRL, &value) )
			return L3GD20H_ERROR;
	
		value &= 0x1f;
		value |= (fm*0x20); 
		/*
	if(fm == L3GD20H_FIFO_Bypass_mode)
	{
		value |= (L3GD20H_FIFO_Bypass_mode*0x20);                   //fifo mode configuration
	}

	if(fm == L3GD20H_FIFO_mode)
	{
		value |= (L3GD20H_FIFO_mode*0x20);                    //fifo mode configuration
	}

	if(fm == L3GD20H_FIFO_Stream_mode)
	{
		value |= (L3GD20H_FIFO_Stream_mode*0x20);                    //fifo mode configuration
	}
*/
	
	if(!L3GD20H_WriteReg(L3GD20H_O_FIFO_CTRL, value))
			return L3GD20H_ERROR;

	return L3GD20H_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without L3GD20H_6D_INT)
* Input          : L3GD20H_INT1_AND/OR | L3GD20H_INT1_ZHIE_ENABLE/DISABLE | L3GD20H_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [L3GD20H_ERROR, L3GD20H_SUCCESS]
*******************************************************************************/
gyrostatus_t L3GD20H_SetIntConfiguration(L3GD20H_Int1Conf_t ic)
{
	u8_t value;

	if(!L3GD20H_ReadReg(L3GD20H_O_IG_CFG, &value))
		return L3GD20H_ERROR;

	value &= 0x40;
	value |= ic;

	if(!L3GD20H_WriteReg(L3GD20H_O_IG_CFG, value))
		return L3GD20H_ERROR;

	return L3GD20H_SUCCESS;
}
/*******************************************************************************
* Function Name  : L3GD20H_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
gyrostatus_t L3GD20H_GetFifoSourceFSS(u8_t* val)
{
	u8_t value;

	if(!L3GD20H_ReadReg(L3GD20H_O_FIFO_SRC, &value))
		return L3GD20H_ERROR;

	value &= 0x1F;
	*val = value;
	return L3GD20H_SUCCESS;
}

#endif