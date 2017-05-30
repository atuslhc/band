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
#include "mems_tracking.h"      //[BG008] add for initialize vars

uint8_t MotionModel = 0;

//bool MEMS_ONLINE=false;
bool MEMS_Monitor_Model = false;

int16_t MEMS_BUFF[3][MEMS_BUFF_SIZE];

#define MEMS_FIFO_FREQ 50

//=========================
#define MEMS_TIME_WINDOW        MEMS_FIFO_FREQ //(2000*MEMS_FIFO_SIZE/MEMS_FRQ) //  64 second
#define MEMS_NOISE_TH           10
#define MEMS_ACT_THRES          6

//#define mems_P2P_threshold      1600 //800 // 600  peak to peak
#define mems_P2P_threshold      3000 //1600 //800 // 600  peak to peak

uint8_t Variance_Val_threshold;
uint8_t MEMS_FRQ;
int16_t pos_max_interval, pos_min_interval;
int16_t val_threshold;
uint8_t which_ax_near_zero;
int  Window_ACT_LEVEL[3];
int  MEMS_XYZ_DC[3];
//============================
status_t LIS3DH_ReadReg(u8_t Reg, u8_t* Data);
status_t LIS3DH_WriteReg(u8_t WriteAddr, u8_t Data);

void ReadMemsRawData(uint8_t* p);
uint8_t mems_test_val;

void MEMS_Par_Init(uint8_t freq);
void SleepWork(void);

int Mems_TimeCount = 0;
uint32_t  active_level_bak = 0;

void MEMS_Par_Init(uint8_t freq)
{
	val_threshold = mems_P2P_threshold;
	Variance_Val_threshold = 12;//170; // v1.5-3=100
	MEMS_FRQ = freq;
	//pos_max_interval = (int16_t)(MEMS_FRQ * 1.2);
	pos_max_interval = (int16_t) MEMS_FRQ;
	pos_min_interval = (int16_t)(MEMS_FRQ >> 3);
	which_ax_near_zero = 0;

	Mems_TimeCount = 0;
	active_level_bak = active_level;
#if BG009
  if (active_level_max < active_level)
      active_level_max = active_level;
  if (active_level_min > active_level)
      active_level_min = active_level;
#endif
}

void MEMS_Disabled(void)
{
    if (systemStatus.blAccelSensorOnline==0x01) //&& 0
    {
        LIS3DH_SetMode(LIS3DH_POWER_DOWN);
    }
    systemStatus.blAccelSensorOnline = false;

	INT_Disable();

	GPIO_IntConfig(MEMS_INT1_PORT, MEMS_INT1_PIN, false, false , false); //fallingEdge
	GPIO_IntClear(1 << MEMS_INT1_PIN);
	GPIO_PinModeSet(MEMS_INT1_PORT, MEMS_INT1_PIN, gpioModeDisabled, 1);
    
	GPIO_IntConfig(MEMS_INT2_PORT, MEMS_INT2_PIN, false, false , false); //fallingEdge
	GPIO_IntClear(1 << MEMS_INT2_PIN);
	GPIO_PinModeSet(MEMS_INT2_PORT, MEMS_INT2_PIN, gpioModeDisabled, 1);

	INT_Enable();

	GPIO_PinModeSet(MEMS_CS_PORT, MEMS_CS_PIN, gpioModeDisabled, 1);
}

int MEMS_Init(uint8_t mode)
{
	USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(MEMS_CS_PORT, MEMS_CS_PIN, gpioModePushPull, 1);

	/* Setup clocks */
	CMU_ClockEnable(MEMS_SPI_CMUCLOCK, true);

	/* Setup GPIO's */
	GPIO_PinModeSet(MEMS_SPI_GPIOPORT, MEMS_SPI_CLKPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(MEMS_SPI_GPIOPORT, MEMS_SPI_MOSIPIN, gpioModePushPull, 1);
	GPIO_PinModeSet(MEMS_SPI_GPIOPORT, MEMS_SPI_MISOPIN, gpioModeInputPull, 1);

	GPIO_PinModeSet(MEMS_INT1_PORT, MEMS_INT1_PIN, gpioModeInputPull, 1);
	GPIO_PinModeSet(MEMS_INT2_PORT, MEMS_INT2_PIN, gpioModeInputPull, 1);

	/* Setup USART */
	usartInit.baudrate = 8000000;
	usartInit.databits = usartDatabits8;
	usartInit.msbf = true;
	usartInit.clockMode = usartClockMode3;

	USART_InitSync(MEMS_SPI, &usartInit );
	MEMS_SPI->ROUTE = (USART_ROUTE_CLKPEN | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | MEMS_SPI_LOCATION);

	LIS3DH_GetWHO_AM_I(&mems_test_val);

	if(mems_test_val != LIS3DH_PART_ID) //0x33
    {
		systemStatus.blAccelSensorOnline = false;
        return DEVICE_NOTEXIST;
    }
    
    if (mode==0)
    {
        MEMS_Disabled();
		systemStatus.blAccelSensorOnline = false;
    }
	else
	{
		systemStatus.blAccelSensorOnline = 1;
		LIS3DH_SetMode(LIS3DH_POWER_DOWN);
	}
    return DEVICE_SUCCESS;
}

#if 1
void MEMS_MOTION_MONITOR(uint8_t which_ax)
{
	uint8_t iswhere;
	LIS3DH_SetODR(LIS3DH_ODR_10Hz);
	LIS3DH_SetMode(LIS3DH_LOW_POWER);
	LIS3DH_SetFullScale(LIS3DH_FULLSCALE_2);
	LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
	LIS3DH_WriteReg(LIS3DH_CTRL_REG6, LIS3DH_INT_ACTIVE_LOW + LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE); //fallingEdge

	uint8_t t_8 = abs(MEMS_XYZ_DC[which_ax]) >> 6;

	if(t_8 < 6)
		t_8 = 6;

	LIS3DH_WriteReg(LIS3DH_INT1_THS, t_8); // Threshold = 250 mg

	switch(which_ax)
	{
		case 0:
			LIS3DH_WriteReg(LIS3DH_INT1_CFG, 0x02);
			break;

		case 1:
			LIS3DH_WriteReg(LIS3DH_INT1_CFG, 0x08);
			break;

		case 2:
			LIS3DH_WriteReg(LIS3DH_INT1_CFG, 0x20);
			break;
	}

	LIS3DH_WriteReg(LIS3DH_INT1_DURATION, 0x00);
	LIS3DH_SetInt1Pin(LIS3DH_INT1_OVERRUN_DISABLE);
	LIS3DH_WriteReg(LIS3DH_CTRL_REG5, 0x00); //  not  Interrupt latched
	LIS3DH_FIFOModeEnable(LIS3DH_FIFO_DISABLE);
	GPIO_IntConfig(MEMS_INT1_PORT, MEMS_INT1_PIN, false, true , false); //fallingEdge
	GPIO_IntClear(1 << MEMS_INT2_PIN);
	GPIO_IntConfig(MEMS_INT2_PORT, MEMS_INT2_PIN, false, true , true); //fallingEdge
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	LIS3DH_ReadReg(LIS3DH_INT1_SRC, &iswhere);
	MEMS_Monitor_Model = true;
}
#else
void MEMS_MOTION_MONITOR(void)
{
	LIS3DH_WriteReg(LIS3DH_INT1_CFG, 0x3f);
	LIS3DH_WriteReg(LIS3DH_INT1_CFG, 0);
	GPIO_IntConfig(MEMS_INT1_PORT, MEMS_INT1_PIN, false, true , false); //fallingEdge
	GPIO_IntClear(1 << MEMS_INT2_PIN);
	GPIO_IntConfig(MEMS_INT2_PORT, MEMS_INT2_PIN, false, true , true); //fallingEdge
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	MEMS_Monitor_Model = true;
}
#endif

void MEMS_NO_FIFO_INIT(void)
{
	uint8_t iswhere;
	//===========================20140730
	/* Disable interrupts */
	INT_Disable();
	GPIO_IntConfig(MEMS_INT2_PORT, MEMS_INT2_PIN, false, true , false); //fallingEdge
	GPIO_IntConfig(MEMS_INT1_PORT, MEMS_INT1_PIN, false, true , false); //fallingEdge
	GPIO_IntClear(1 << MEMS_INT2_PIN);
	GPIO_IntClear(1 << MEMS_INT1_PIN);
	/* Initialization done, enable interrupts globally. */
	INT_Enable();
	//===================================
	LIS3DH_SetODR(LIS3DH_ODR_100Hz);
	LIS3DH_SetMode(LIS3DH_NORMAL);//LIS3DH_LOW_POWER
	LIS3DH_SetFullScale(LIS3DH_FULLSCALE_4);
	LIS3DH_FIFOModeEnable(LIS3DH_FIFO_BYPASS_MODE);
	LIS3DH_WriteReg(LIS3DH_CTRL_REG2, 0x00);//without  filter
	LIS3DH_SetAxis(LIS3DH_X_ENABLE | LIS3DH_Y_ENABLE | LIS3DH_Z_ENABLE);
	LIS3DH_ReadReg(LIS3DH_INT1_SRC, &iswhere);
	MEMS_Monitor_Model = true;
	MEMS_Par_Init(PPG_SAM_FREQ);
	active_level_delta = 0;
}

void MEMS_FIFO_INIT(void)
{
	uint8_t iswhere;

	LIS3DH_SetODR(LIS3DH_ODR_50Hz);
	LIS3DH_SetMode(LIS3DH_NORMAL);
	LIS3DH_SetFullScale(LIS3DH_FULLSCALE_4);

	//LIS3DH_WriteReg(LIS3DH_CTRL_REG2, 0x08);//cut off 0.5hz  filter

	LIS3DH_WriteReg(LIS3DH_CTRL_REG5, 0x00); //   not Interrupt latched
	LIS3DH_WriteReg(LIS3DH_CTRL_REG6, LIS3DH_INT_ACTIVE_LOW); //fallingEdge

	LIS3DH_SetInt1Pin(LIS3DH_INT1_OVERRUN_ENABLE);//LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE
	//LIS3DH_SetInt1Pin(LIS3DH_WTM_ON_INT1_ENABLE);

	LIS3DH_WriteReg(LIS3DH_INT1_CFG, 0x00);

	//LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, 16);
	LIS3DH_FIFOModeEnable(LIS3DH_FIFO_BYPASS_MODE);
	LIS3DH_FIFOModeEnable(LIS3DH_FIFO_STREAM_MODE);
	//LIS3DH_FIFOModeEnable(LIS3DH_FIFO_MODE);

	GPIO_IntConfig(MEMS_INT2_PORT, MEMS_INT2_PIN, false, true , false); //fallingEdge

	GPIO_IntClear(1 << MEMS_INT1_PIN);
	GPIO_IntConfig(MEMS_INT1_PORT, MEMS_INT1_PIN, false, true , true); //fallingEdge
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	ReadMemsRawData((uint8_t*)&MEMS_BUFF[0][0]);  // clear the old buff

	LIS3DH_ReadReg(LIS3DH_INT1_SRC, &iswhere);

	MEMS_Monitor_Model = false;
	MEMS_Par_Init(MEMS_FIFO_FREQ);
	XYZ_FIR_INIT(MEMS_FIFO_FREQ);
#if FALL_DETECT_SUPPORT		//[BG012] add compiler flag.
	/* [BG008] init vars for safe. */
	int i;

	for (i = 0 ; i < AXIS_FILTER_BUFF_SIZE ; i++)
	{
		axis_filter[i] = 0;
	}

	//FD_result = 0; //[BG008-4] remark, should not reset or will think as a new trigger if it not zero.
	//oldFallStatus = 0;
	axis_filter_index = 0;
#endif
}

uint32_t MemesEventCount = 0;
//uint8_t iswhere=0;
uint32_t save_activity_delta;
void Mems_Proc(void)
{
    if (systemStatus.blAccelSensorOnline==0x0) //if not online, action should be filter.
      return;
#if (BGXXX==10)
    test2.typeuint32++;
#endif
	ReadMemsFIFO((uint8_t*)&MEMS_BUFF[0][0],
	             (uint8_t*)&MEMS_BUFF[1][0],
	             (uint8_t*)&MEMS_BUFF[2][0],
	             MEMS_FIFO_SIZE); // 900us to read

	if(systemSetting.SystemMode == SYSTEM_MODE_ACTIVATED)
		active_level_delta = XYZFilter_TRACK();  //call to mems_tracking

	MemesEventCount++;
	//MemesEventCount%=100;
	Mems_TimeCount++;

	if(Mems_TimeCount > MEMS_TIME_WINDOW) // 32/50*50=32second
	{
		Mems_TimeCount = 0;
		save_activity_delta = active_level - active_level_bak;

		if(save_activity_delta < MEMS_NOISE_TH)
		{
			SleepWork();
		}

		active_level_bak = active_level;
	}
}

void Mems_WakeUp(void)
{
    if (systemStatus.blAccelSensorOnline==0x0)
      return;
    
	if(systemStatus.blHRSensorOn == false)
	{
		MEMS_FIFO_INIT();
	}

	isMemsSleeping = false;
}


void SleepWork(void)
{
	//LED_ON();
	MEMS_MOTION_MONITOR(which_ax_near_zero);
	isMemsSleeping = true;
}

uint32_t MemesErrorCount = 0; //record how many times detect the accelerometer miss event.
void isMemsError(void)
{
	static char twosecondinterval = 0;
	static uint32_t MemesEventCountBak = 5; //why not 0?

	if( MEMS_Monitor_Model == true)
		return;

	twosecondinterval++;

	if(twosecondinterval > 3)
	{
		if(MemesEventCountBak == MemesEventCount)
		{
			MEMS_Init(systemSetting.blAccelSensorEnabled);
			MEMS_FIFO_INIT();
			MemesErrorCount++;
		}

		MemesEventCountBak = MemesEventCount;
		twosecondinterval = 0;
	}
}

void MEMS_INT1_CALLBACK(void)
{
	int32_t msg = MEMS_INT1_Message;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
	//osMessagePut(hMsgInterrupt,MEMS_INT1_Message, 0);
}

void MEMS_INT2_CALLBACK(void)
{
	//osMessagePut(hMsgInterrupt,MEMS_INT2_Message, 0);
	int32_t msg = MEMS_INT2_Message;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
}

void MEMS_CLOSE(void)
{
	LIS3DH_SetMode(LIS3DH_POWER_DOWN);
}

// 2015/6/16 16:49:02 added, to operate under save power modes
void MEMS_OPEN(void)
{
	LIS3DH_SetMode(LIS3DH_NORMAL);
}

void ReadMemsRawData(uint8_t* p)
{
	unsigned char i;
	MEMS_CS_L();
	USART_Tx(MEMS_SPI, 0x80 + 0x40 + LIS3DH_OUT_X_L);
	USART_Rx(MEMS_SPI);

	for(i = 0; i < 6; i++)
	{
		USART_Tx(MEMS_SPI, 0);
		p[i] = USART_Rx(MEMS_SPI);
	}

	MEMS_CS_H();
}

void ReadMemsFIFO(uint8_t* px, uint8_t* py, uint8_t* pz, uint8_t len)
{
	unsigned char i;
    
    //Add the Online check to avoid external access while disable or fail.
    if (systemStatus.blAccelSensorOnline==0)
      return;
	MEMS_CS_L();
	USART_Tx(MEMS_SPI, 0x80 + 0x40 + LIS3DH_OUT_X_L);
	USART_Rx(MEMS_SPI);

	for(i = 0; i < len; i++)
	{
		USART_Tx(MEMS_SPI, 0);
		*(px++) = USART_Rx(MEMS_SPI);
		USART_Tx(MEMS_SPI, 0);
		*(px++) = USART_Rx(MEMS_SPI);
		USART_Tx(MEMS_SPI, 0);
		*(py++) = USART_Rx(MEMS_SPI);
		USART_Tx(MEMS_SPI, 0);
		*(py++) = USART_Rx(MEMS_SPI);
		USART_Tx(MEMS_SPI, 0);
		*(pz++) = USART_Rx(MEMS_SPI);
		USART_Tx(MEMS_SPI, 0);
		*(pz++) = USART_Rx(MEMS_SPI);
	}

	MEMS_CS_H();
}

status_t LIS3DH_ReadReg(u8_t Reg, u8_t* Data)
{
	MEMS_CS_L();
	USART_Tx(MEMS_SPI, 0x80 + Reg);
	uint8_t temp = USART_Rx(MEMS_SPI);
	USART_Tx(MEMS_SPI, 0);
	*Data = USART_Rx(MEMS_SPI);
	MEMS_CS_H();
	return MEMS_SUCCESS;
}


status_t LIS3DH_WriteReg(u8_t WriteAddr, u8_t Data)
{
	MEMS_CS_L();
	USART_Tx(MEMS_SPI, WriteAddr);
	USART_Rx(MEMS_SPI);
	USART_Tx(MEMS_SPI, Data);
	USART_Rx(MEMS_SPI);
	MEMS_CS_H();
	return MEMS_SUCCESS;
}

status_t LIS3DH_GetWHO_AM_I(u8_t* val)
{
	return(LIS3DH_ReadReg(LIS3DH_WHO_AM_I, val));
}

/*******************************************************************************
* Function Name  : LIS3DH_SetODR
* Description    : Sets LIS3DH Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetODR(LIS3DH_ODR_t ov)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value))
		return MEMS_ERROR;

	value &= 0x0f;
	value |= ov << LIS3DH_ODR_BIT;

	if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value))
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}

status_t LIS3DH_GetODR(u8_t * ov)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value))
		return MEMS_ERROR;

	value &= 0xf0;
	*ov = value >> LIS3DH_ODR_BIT;

 /*
 ODR3 ODR2 ODR1 ODR0 Power mode selection
0 0 0 0 Power down mode
0 0 0 1 Normal / low power mode (1 Hz)
0 0 1 0 Normal / low power mode (10 Hz)
0 0 1 1 Normal / low power mode (25 Hz)
0 1 0 0 Normal / low power mode (50 Hz)
0 1 0 1 Normal / low power mode (100 Hz)
0 1 1 0 Normal / low power mode (200 Hz)
0 1 1 1 Normal / low power mode (400 Hz)
1 0 0 0 Low power mode (1.6 KHz)
1 0 0 1 Normal (1.25 kHz) / low power mode (5 KHz)
 *
 */
	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_SetMode
* Description    : Sets LIS3DH Operating Mode
* Input          : Modality (LIS3DH_NORMAL, LIS3DH_LOW_POWER, LIS3DH_POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetMode(LIS3DH_Mode_t md)
{
	u8_t value;
	u8_t value2;
	static u8_t ODR_old_value;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value))
		return MEMS_ERROR;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value2))
		return MEMS_ERROR;

	if((value & 0xF0) == 0)
		value = value | (ODR_old_value & 0xF0); //if it comes from POWERDOWN

	switch(md)
	{
		case LIS3DH_POWER_DOWN:
			ODR_old_value = value;
			value &= 0x0F;
			break;

		case LIS3DH_NORMAL:
			value &= 0xF7;
			value |= (MEMS_RESET << LIS3DH_LPEN);
			value2 &= 0xF7;
			value2 |= (MEMS_SET << LIS3DH_HR); //set HighResolution_BIT
			break;

		case LIS3DH_LOW_POWER:
			value &= 0xF7;
			value |=  (MEMS_SET << LIS3DH_LPEN);
			value2 &= 0xF7;
			value2 |= (MEMS_RESET << LIS3DH_HR); //reset HighResolution_BIT
			break;

		default:
			return MEMS_ERROR;
	}

	if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value))
		return MEMS_ERROR;

	if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value2))
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_SetFullScale
* Description    : Sets the LIS3DH FullScale
* Input          : LIS3DH_FULLSCALE_2/LIS3DH_FULLSCALE_4/LIS3DH_FULLSCALE_8/LIS3DH_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetFullScale(LIS3DH_Fullscale_t fs)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value))
		return MEMS_ERROR;

	value &= 0xCF;
	value |= (fs << LIS3DH_FS);

	if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value))
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_SetAxis
* Description    : Enable/Disable LIS3DH Axis
* Input          : LIS3DH_X_ENABLE/DISABLE | LIS3DH_Y_ENABLE/DISABLE | LIS3DH_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetAxis(LIS3DH_Axis_t axis)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value))
		return MEMS_ERROR;

	value &= 0xF8;
	value |= (0x07 & axis);

	if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value))
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS3DH_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |
LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS3DH_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |
LIS3DH_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS3DH_WTM_ON_INT1_ENABLE/DISABLE         |
LIS3DH_INT1_OVERRUN_ENABLE/DISABLE
* example        : SetInt1Pin(LIS3DH_CLICK_ON_PIN_INT1_ENABLE | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE |
LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE | LIS3DH_I1_DRDY1_ON_INT1_ENABLE | LIS3DH_I1_DRDY2_ON_INT1_ENABLE |
LIS3DH_WTM_ON_INT1_DISABLE | LIS3DH_INT1_OVERRUN_DISABLE   )
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Pin(LIS3DH_IntPinConf_t pinConf)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG3, &value))
		return MEMS_ERROR;

	value &= 0x00;
	value |= pinConf;

	if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG3, value))
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS3DH_FIFO_DISABLE, LIS3DH_FIFO_BYPASS_MODE, LIS3DH_FIFO_MODE,
LIS3DH_FIFO_STREAM_MODE, LIS3DH_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_FIFOModeEnable(LIS3DH_FifoMode_t fm)
{
	u8_t value;

	if(fm == LIS3DH_FIFO_DISABLE)
	{
		if(!LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value))
			return MEMS_ERROR;

		value &= 0x1F;
		value |= (LIS3DH_FIFO_BYPASS_MODE << LIS3DH_FM);

		if(!LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value)) //fifo mode bypass
			return MEMS_ERROR;

		if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
			return MEMS_ERROR;

		value &= 0xBF;

		if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo disable
			return MEMS_ERROR;
	}

	if(fm == LIS3DH_FIFO_BYPASS_MODE)
	{
		if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value))
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET << LIS3DH_FIFO_EN;

		if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
			return MEMS_ERROR;

		if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm << LIS3DH_FM);                   //fifo mode configuration

		if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
			return MEMS_ERROR;
	}

	if(fm == LIS3DH_FIFO_MODE)
	{
		if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value))
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET << LIS3DH_FIFO_EN;

		if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
			return MEMS_ERROR;

		if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm << LIS3DH_FM);                    //fifo mode configuration

		if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
			return MEMS_ERROR;
	}

	if(fm == LIS3DH_FIFO_STREAM_MODE)
	{
		if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value))
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET << LIS3DH_FIFO_EN;

		if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value))               //fifo enable
			return MEMS_ERROR;

		if(!LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value))
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm << LIS3DH_FM);                    //fifo mode configuration

		if(!LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value))
			return MEMS_ERROR;
	}

	if(fm == LIS3DH_FIFO_TRIGGER_MODE)
	{
		if(!LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value))
			return MEMS_ERROR;

		value &= 0xBF;
		value |= MEMS_SET << LIS3DH_FIFO_EN;

		if(!LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value))       //fifo enable
			return MEMS_ERROR;

		if(!LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value))
			return MEMS_ERROR;

		value &= 0x1f;
		value |= (fm << LIS3DH_FM); //fifo mode configuration

		if(!LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value))
			return MEMS_ERROR;
	}

	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without LIS3DH_6D_INT)
* Input          : LIS3DH_INT1_AND/OR | LIS3DH_INT1_ZHIE_ENABLE/DISABLE | LIS3DH_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetIntConfiguration(LIS3DH_Int1Conf_t ic)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value))
		return MEMS_ERROR;

	value &= 0x40;
	value |= ic;

	if(!LIS3DH_WriteReg(LIS3DH_INT1_CFG, value))
		return MEMS_ERROR;

	return MEMS_SUCCESS;
}
/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceFSS(u8_t* val)
{
	u8_t value;

	if(!LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, &value))
		return MEMS_ERROR;

	value &= 0x1F;
	*val = value;
	return MEMS_SUCCESS;
}