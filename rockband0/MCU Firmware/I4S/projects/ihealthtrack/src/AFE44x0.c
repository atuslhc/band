#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "AFE44x0.h"
#include "main.h"
#include "em_adc.h"
#include "freertos.h"
#include "task.h"

//#include "Signal_proc.h"

#include "common_vars.h"
#include "device_task.h"
#include "em_int.h"

extern int itest; //BG013_2
extern uint32_t utest; //BG013_2

//bool afe_online=false;

uint8_t LED_INTENSITY = LED_Intensity_Init_val; //refer AFE4400.pdf p65 0-255.
uint8_t AMB_uA = 0;     //refer AFE4400.pdf p64 0-10.

//int16_t LED_Noise_Count = 0;

uint16_t  afe_sample_counter;

uint32_t ambient_light_low_counter = 0;
uint32_t ambient_light_high_counter = 0;

uint16_t light_locked_Counter = 0;

int32_t read_IR_reg_22bit, output_ppg_22bit;
int32_t read_AB_reg_22bit;

#if (BG013_LED_ADJ_METHOD==0)
int16_t read_IR_reg;
int16_t read_AB_reg; //, ambient_light_val_at_this_cancellation = 0; //Atus: remove ambient_light_val_at_this_cancellation, no used.
int16_t DBG_IR_reg_min=REG_VAL_MAX, DBG_IR_reg_max=REG_VAL_MIN;
int16_t DBG_AB_reg_min=REG_VAL_MAX, DBG_AB_reg_max=REG_VAL_MIN; //-32768;
#elif (BG013_LED_ADJ_METHOD==1)
int16_t read_IR_reg;
int16_t read_AB_reg;
int16_t DBG_IR_reg_min=REG_VAL_MAX, DBG_IR_reg_max=REG_VAL_MIN;
int16_t DBG_AB_reg_min=REG_VAL_MAX, DBG_AB_reg_max=REG_VAL_MIN; //-32768;
#elif (BG013_LED_ADJ_METHOD==2)
int32_t read_IR_reg;
int32_t IR_reg_min=REG_VAL_MAX, IR_reg_max=REG_VAL_MIN;
int32_t DBG_IR_reg_min=REG_VAL_MAX, DBG_IR_reg_max=REG_VAL_MIN;
int32_t read_AB_reg;
int32_t AB_reg_min=REG_VAL_MAX, AB_reg_max=REG_VAL_MIN;
int32_t DBG_AB_reg_min=REG_VAL_MAX, DBG_AB_reg_max=REG_VAL_MIN;
#define AFE_ADJ_SKIP_COUNT    3       //skip the data after adj
#endif
//int16_t pri_read_IR_reg;
int16_t AxieOUT[3];
int16_t axie[3];
uint8_t Ref_F = 0;


/*====  use  LED1   ADC2 , ADC3 ========*/
#if 0  //50pF-50K--10uA-64HZ-0.1%-Stage2
static const uint32_t AFE44xx_Default_Register_Settings[49] =
{

	/*AFE4490_CONTROL0*/	0x000000,
	/*AFE4490_LED2STC*/     0x000000,
	/*AFE4490_LED2ENDC*/	0x000000,
	/*AFE4490_LED2LEDSTC*/	0x000000,
	/*AFE4490_LED2LEDENDC */ 0x000000,
	/*AFE4490_ALED2STC*/	0x000000,
	/*AFE4490_ALED2ENDC*/	0x000000,
	/*AFE4490_LED1STC */    0x003D09,
	/*AFE4490_LED1ENDC*/	 0x0041E9,// 2% 0x003F78,
	/*AFE4490_LED1LEDSTC*/	0x003D09,
	/*AFE4490_LED1LEDENDC */0x0041EA,//2%  0x003F79,
	/*AFE4490_ALED1STC*/	0x007A12,
	/*AFE4490_ALED1ENDC	*/0x007EF2,// 0x007C81,
	/*AFE4490_LED2CONVST*/	0x000000,
	/*AFE4490_LED2CONVEND*/ 0x000000,
	/*AFE4490_ALED2CONVST */0x003D0B,
	/*AFE4490_ALED2CONVEND*/	0x007A11,
	/*AFE4490_LED1CONVST*/	0x007A14,
	/*AFE4490_LED1CONVEND*/ 0x00B71A,
	/*AFE4490_ALED1CONVST*/ 0x00B71D,
	/*AFE4490_ALED1CONVEND*/	0x00F423,
	/*AFE4490_ADCRSTSTCT0*/ 0x000000,
	/*AFE4490_ADCRSTENDCT0*/	0x000000,
	/*AFE4490_ADCRSTSTCT1*/ 0x003D09,
	/*AFE4490_ADCRSTENDCT1*/	0x003D09,
	/*AFE4490_ADCRSTSTCT2*/ 0x007A12,
	/*AFE4490_ADCRSTENDCT2*/	0x007A12,
	/*AFE4490_ADCRSTSTCT3*/ 0x00B71B,
	/*AFE4490_ADCRSTENDCT3*/	0x00B71B,
	/*AFE4490_PRPCOUNT*/	0x00F423,
	/*AFE4490_CONTROL1*/	0x000101,
	/*AFE4490_SPARE1*/	0x000000,
	/*AFE4490_TIAGAIN*/ 0x000000,
	/*AFE4490_TIA_AMB_GAIN*/	0x05403B,  // 5uA  50fpF =0x05003B     10uA 50fpF =0x0A003B
	/*AFE4490_LEDCNTRL*/	0x000000,
	/*AFE4490_CONTROL2*/	0x040000,
	/*AFE4400_SPARE2	*/0x000000,
	/*AFE4400_SPARE3	*/0x000000,
	/*AFE4400_SPARE4	*/0x000000,
	/*AFE4400_SPARE5	*/0x000000,
	/*AFE4400_SPARE6	*/0x000000,
	/*AFE4400_ALARM */0x000000,
	/*AFE4400_LED2VAL	*/0x000000,
	/*AFE4400_ALED2VAL	*/0x000000,
	/*AFE4400_LED1VAL	*/0x000000,
	/*AFE4400_ALED1VAL	*/0x000000,
	/*AFE4400_LED2-ALED2VAL */0x000000,
	/*AFE4400_LED1-ALED1VAL */0x000000,
	/*AFE4400_DIAGNOSTICS	*/0x000000

};

#else
#if 1
// 50pF-50K--10uA-64HZ-0.5%-Stage2
static unsigned long 	AFE44xx_Default_Register_Settings[49] =
{

	/*AFE4490_CONTROL0*/	0x000000,       //0x000000 >> 0x000001
	/*AFE4490_LED2STC*/     0x000000,
	/*AFE4490_LED2ENDC*/	0x000000,
	/*AFE4490_LED2LEDSTC*/	0x000000,
	/*AFE4490_LED2LEDENDC*/ 0x000000,
	/*AFE4490_ALED2STC*/	0x000000,
	/*AFE4490_ALED2ENDC*/	0x000000,
	/*AFE4490_LED1STC */    0x003D89,// dealy to sample
	/*AFE4490_LED1ENDC*/	0x003E3f,//0.5%
	/*AFE4490_LED1LEDSTC*/	0x003D09,
	/*AFE4490_LED1LEDENDC*/ 0x003E40,///0.5%
	/*AFE4490_ALED1STC*/	0x007A12,
	/*AFE4490_ALED1ENDC*/   0x007B48,///0.5%
	/*AFE4490_LED2CONVST*/	0x000000,
	/*AFE4490_LED2CONVEND*/ 0x000000,
	/*AFE4490_ALED2CONVST*/ 0x003D0B,
	/*AFE4490_ALED2CONVEND*/ 0x007A11,
	/*AFE4490_LED1CONVST*/	0x007A14,
	/*AFE4490_LED1CONVEND*/ 0x00B71A,
	/*AFE4490_ALED1CONVST*/ 0x00B71D,
	/*AFE4490_ALED1CONVEND*/ 0x00F423,
	/*AFE4490_ADCRSTSTCT0*/ 0x000000,
	/*AFE4490_ADCRSTENDCT0*/ 0x000000,
	/*AFE4490_ADCRSTSTCT1*/ 0x003D09,
	/*AFE4490_ADCRSTENDCT1*/ 0x003D09,
	/*AFE4490_ADCRSTSTCT2*/ 0x007A12,
	/*AFE4490_ADCRSTENDCT2*/ 0x007A12,
	/*AFE4490_ADCRSTSTCT3*/ 0x00B71B,
	/*AFE4490_ADCRSTENDCT3*/ 0x00B71B,
	/*AFE4490_PRPCOUNT*/	0x00F423,
	/*AFE4490_CONTROL1*/	0x000102,//0x000101 >> 0x000102
	/*AFE4490_SPARE1*/	0x000000,
	/*AFE4490_TIAGAIN*/     0x000000,
	/*AFE4490_TIA_AMB_GAIN*/ 0x00003B,  // 5uA  50fpF =0x05003B     10uA 50fpF =0x0A003B
	/*AFE4490_LEDCNTRL*/	0x000000,
	/*AFE4490_CONTROL2*/	0x020100,       //Atus: 0x040000 >> 0x020100 p66
	/*AFE4400_SPARE2*/      0x000000,
	/*AFE4400_SPARE3*/      0x000000,
	/*AFE4400_SPARE4*/      0x000000,
	/*AFE4400_SPARE5*/      0x000000,
	/*AFE4400_SPARE6*/      0x000000,
	/*AFE4400_ALARM*/       0x000000,
	/*AFE4400_LED2VAL*/     0x000000,
	/*AFE4400_ALED2VAL*/    0x000000,
	/*AFE4400_LED1VAL*/     0x000000,
	/*AFE4400_ALED1VAL*/    0x000000,
	/*AFE4400_LED2-ALED2VAL*/ 0x000000,
	/*AFE4400_LED1-ALED1VAL*/ 0x000000,
	/*AFE4400_DIAGNOSTICS*/ 0x000000

};
#endif
#if 0
// 100pF-50K--64HZ-2%-Stage2
static unsigned long	AFE44xx_Default_Register_Settings[49] =
{

	/*AFE4490_CONTROL0*/	0x000000,
	/*AFE4490_LED2STC*/ 	0x00B71B,
	/*AFE4490_LED2ENDC*/	0x00F422,

	/*AFE4490_LED2LEDSTC*/	0x00B71B,
	/*AFE4490_LED2LEDENDC */ 0x000000,
	/*AFE4490_ALED2STC*/	0x00F423,


	/*AFE4490_ALED2ENDC*/	0x003D07,


	/*AFE4490_LED1STC */	0x003D09,


	/*AFE4490_LED1ENDC*/	 0x007A10,//25%


	/*AFE4490_LED1LEDSTC*/	0x003D09,

	/*AFE4490_LED1LEDENDC */0x007A11,///25%

	/*AFE4490_ALED1STC*/	0x007A12,
	/*AFE4490_ALED1ENDC */0x00B719,///2%
	/*AFE4490_LED2CONVST*/	0x000002,
	/*AFE4490_LED2CONVEND*/ 0x003D08,
	/*AFE4490_ALED2CONVST */0x003D0B,

	/*AFE4490_ALED2CONVEND*/	0x007A11,
	/*AFE4490_LED1CONVST*/	0x007A14,

	/*AFE4490_LED1CONVEND*/ 0x00B71A,
	/*AFE4490_ALED1CONVST*/ 0x00B71D,

	/*AFE4490_ALED1CONVEND*/	0x00F423,
	/*AFE4490_ADCRSTSTCT0*/ 0x000000,
	/*AFE4490_ADCRSTENDCT0*/	0x000000,
	/*AFE4490_ADCRSTSTCT1*/ 0x003D09,
	/*AFE4490_ADCRSTENDCT1*/	0x003D09,
	/*AFE4490_ADCRSTSTCT2*/ 0x007A12,
	/*AFE4490_ADCRSTENDCT2*/	0x007A12,
	/*AFE4490_ADCRSTSTCT3*/ 0x00B71B,
	/*AFE4490_ADCRSTENDCT3*/	0x00B71B,
	/*AFE4490_PRPCOUNT*/	0x00F423,
	/*AFE4490_CONTROL1*/	0x000101,
	/*AFE4490_SPARE1*/	0x000000,
	/*AFE4490_TIAGAIN*/ 0x000000,
	/*AFE4490_TIA_AMB_GAIN*/	0x000003,  // 100fpF
	/*AFE4490_LEDCNTRL*/	0x011A00,
	/*AFE4490_CONTROL2*/	0x020000,
	/*AFE4400_SPARE2	*/0x000000,
	/*AFE4400_SPARE3	*/0x000000,
	/*AFE4400_SPARE4	*/0x000000,
	/*AFE4400_SPARE5	*/0x000000,
	/*AFE4400_SPARE6	*/0x000000,
	/*AFE4400_ALARM */0x000000,
	/*AFE4400_LED2VAL	*/0x000000,
	/*AFE4400_ALED2VAL	*/0x000000,
	/*AFE4400_LED1VAL	*/0x000000,
	/*AFE4400_ALED1VAL	*/0x000000,
	/*AFE4400_LED2-ALED2VAL */0x000000,
	/*AFE4400_LED1-ALED1VAL */0x000000,
	/*AFE4400_DIAGNOSTICS	*/0x000000

};

#endif
#if 0
// 100pF-50K--64HZ-2%-Stage2
static unsigned long 	AFE44xx_Default_Register_Settings[49] =
{

	/*AFE4490_CONTROL0*/	0x000000,
	/*AFE4490_LED2STC*/     0x000000,
	/*AFE4490_LED2ENDC*/	0x000000,
	/*AFE4490_LED2LEDSTC*/	0x000000,
	/*AFE4490_LED2LEDENDC */ 0x000000,
	/*AFE4490_ALED2STC*/	0x000000,
	/*AFE4490_ALED2ENDC*/	0x000000,
	/*AFE4490_LED1STC */    0x003D09,
	/*AFE4490_LED1ENDC*/	 0x0041E9,//2%
	/*AFE4490_LED1LEDSTC*/	0x003D09,
	/*AFE4490_LED1LEDENDC */0x0041EA,///2%
	/*AFE4490_ALED1STC*/	0x007A12,
	/*AFE4490_ALED1ENDC	*/0x007EF2,///2%
	/*AFE4490_LED2CONVST*/	0x000000,
	/*AFE4490_LED2CONVEND*/ 0x000000,
	/*AFE4490_ALED2CONVST */0x003D0B,
	/*AFE4490_ALED2CONVEND*/	0x007A11,
	/*AFE4490_LED1CONVST*/	0x007A14,
	/*AFE4490_LED1CONVEND*/ 0x00B71A,
	/*AFE4490_ALED1CONVST*/ 0x00B71D,
	/*AFE4490_ALED1CONVEND*/	0x00F423,
	/*AFE4490_ADCRSTSTCT0*/ 0x000000,
	/*AFE4490_ADCRSTENDCT0*/	0x000000,
	/*AFE4490_ADCRSTSTCT1*/ 0x003D09,
	/*AFE4490_ADCRSTENDCT1*/	0x003D09,
	/*AFE4490_ADCRSTSTCT2*/ 0x007A12,
	/*AFE4490_ADCRSTENDCT2*/	0x007A12,
	/*AFE4490_ADCRSTSTCT3*/ 0x00B71B,
	/*AFE4490_ADCRSTENDCT3*/	0x00B71B,
	/*AFE4490_PRPCOUNT*/	0x00F423,
	/*AFE4490_CONTROL1*/	0x000101,
	/*AFE4490_SPARE1*/	0x000000,
	/*AFE4490_TIAGAIN*/ 0x000000,
	/*AFE4490_TIA_AMB_GAIN*/	0x00007B,  // 100fpF
	/*AFE4490_LEDCNTRL*/	0x000000,
	/*AFE4490_CONTROL2*/	0x040000,
	/*AFE4400_SPARE2	*/0x000000,
	/*AFE4400_SPARE3	*/0x000000,
	/*AFE4400_SPARE4	*/0x000000,
	/*AFE4400_SPARE5	*/0x000000,
	/*AFE4400_SPARE6	*/0x000000,
	/*AFE4400_ALARM */0x000000,
	/*AFE4400_LED2VAL	*/0x000000,
	/*AFE4400_ALED2VAL	*/0x000000,
	/*AFE4400_LED1VAL	*/0x000000,
	/*AFE4400_ALED1VAL	*/0x000000,
	/*AFE4400_LED2-ALED2VAL */0x000000,
	/*AFE4400_LED1-ALED1VAL */0x000000,
	/*AFE4400_DIAGNOSTICS	*/0x000000

};
#endif

#endif
/**********************************************************************************************************
* Init_AFE44xx_DRDY_Interrupt					                                          *
**********************************************************************************************************/
void Init_AFE44xx_DRDY_Interrupt (void)
{
	GPIO_PinModeSet(AFE_ADC_DRDY_PORT, AFE_ADC_DRDY_PIN, gpioModeInputPull, 0);
	GPIO_IntConfig(AFE_ADC_DRDY_PORT, AFE_ADC_DRDY_PIN, true, false, false);

}

/**********************************************************************************************************
* Enable_AFE44xx_DRDY_Interrupt	        			                                          *
**********************************************************************************************************/
void Enable_AFE44xx_DRDY_Interrupt (void)
{
	GPIO_IntConfig(AFE_ADC_DRDY_PORT, AFE_ADC_DRDY_PIN, true, false, true);

	NVIC_SetPriority(AFE_INT_LOC, AFE_INT_LEVEL);

	NVIC_ClearPendingIRQ((AFE_INT_LOC));
	NVIC_EnableIRQ(AFE_INT_LOC);
}


/**********************************************************************************************************
* Init_AFE44xx_Resource						                                          *
**********************************************************************************************************/

void Init_AFE44xx_Resource(void)
{
	USART_InitSync_TypeDef usartInit = USART_INITSYNC_DEFAULT;

	/* Setup clocks */
	CMU_ClockEnable( cmuClock_GPIO, true );
	CMU_ClockEnable( AFE_SPI_CMUCLOCK, true );

	/* Setup GPIO's */
	GPIO_PinModeSet( AFE_SPI_GPIOPORT, AFE_SPI_CLKPIN, gpioModePushPull, 0 );
	GPIO_PinModeSet( AFE_SPI_GPIOPORT, AFE_SPI_MOSIPIN, gpioModePushPull, 0 );
	GPIO_PinModeSet( AFE_SPI_GPIOPORT, AFE_SPI_MISOPIN, gpioModePushPull, 0 );
	GPIO_PinModeSet( AFE_CS_PORT, AFE_CS_PIN, gpioModePushPull, 0 );


	/* Setup USART */
	usartInit.baudrate = 7000000;
	usartInit.databits = usartDatabits8;
	usartInit.msbf = true;
	usartInit.clockMode = usartClockMode0;

	USART_InitSync( AFE_SPI, &usartInit );
	AFE_SPI->ROUTE = (USART_ROUTE_CLKPEN | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | AFE_SPI_LOCATION);

}



/*********************************************************************************************************
* AFE44xx_Reg_Write									                 *
**********************************************************************************************************/
void AFE44xx_SW_CMD (unsigned char ch)
{
	// as use read command at ISR , avoid to conflict ,maks int
	INT_Disable();
	AFE_CS_L();
	USART_Tx(AFE_SPI, 0);
	USART_Rx(AFE_SPI);
	USART_Tx(AFE_SPI, 0);
	USART_Rx(AFE_SPI);
	USART_Tx(AFE_SPI, 0);
	USART_Rx(AFE_SPI);
	USART_Tx(AFE_SPI, ch);
	USART_Rx(AFE_SPI);
	AFE_CS_H();
	INT_Enable();
}

void AFE44xx_Reg_Write (unsigned char reg_address, uint32_t data)
{
	// as use read command at ISR , avoid to conflict ,mark int
	INT_Disable();
	//Write to register - byte wise transfer, 8-Bit transfers
	//  SEN LOW FOR TRANSMISSION.
	AFE_CS_L();
	USART_Tx(AFE_SPI, reg_address);
	USART_Rx(AFE_SPI);
	USART_Tx(AFE_SPI, (unsigned char)(data >> 16));
	USART_Rx(AFE_SPI);
	USART_Tx(AFE_SPI, (unsigned char)(((data & 0x00FFFF) >> 8)));
	USART_Rx(AFE_SPI);
	USART_Tx(AFE_SPI, (unsigned char)(((data & 0x0000FF))));
	USART_Rx(AFE_SPI);
	AFE_CS_H();
	INT_Enable();
}

/*********************************************************************************************************
* AFE44xx_Reg_Read									                 *
**********************************************************************************************************/
uint32_t AFE44xx_Reg_Read(unsigned char Reg_address)
{
//	unsigned char dummy_rx;
	uint32_t  SPI_Rx_buf[4], retVal;

	retVal = 0;

	//Read from register - byte wise transfer, 8-Bit transfers
	AFE_CS_L();

	USART_Tx(AFE_SPI, Reg_address);
	SPI_Rx_buf[0] = USART_Rx(AFE_SPI);

	USART_Tx(AFE_SPI, 0);
	SPI_Rx_buf[1] = USART_Rx(AFE_SPI);

	USART_Tx(AFE_SPI, 0);
	SPI_Rx_buf[2] = USART_Rx(AFE_SPI);

	USART_Tx(AFE_SPI, 0);
	SPI_Rx_buf[3] = USART_Rx(AFE_SPI);
	AFE_CS_H();

	retVal = (SPI_Rx_buf[1] << 16) | (SPI_Rx_buf[2] << 8) | (SPI_Rx_buf[3]);
	return 	retVal;
}



/**********************************************************************************************************
*	        AFE44xx default Initialization          				                  					  *
**********************************************************************************************************/

void AFE44xx_Default_Reg_Init(void)
{
	unsigned char i;

	AFE44xx_SW_CMD(AFE_REG_WRITE);

//	for ( i = 0; i < sizeof(AFE44xx_Default_Register_Settings); i++)
	for ( i = 0; i < sizeof(AFE44xx_Default_Register_Settings)/sizeof(AFE44xx_Default_Register_Settings[0]); i++)
	{
		AFE44xx_Reg_Write(i, AFE44xx_Default_Register_Settings[i]);
	}

	AFE44xx_SW_CMD(AFE_REG_READ); //2014.07.08
        //itest ++; //BG013_2, i
}

//这里amb_uA是什么意思？
void LED_Val_AMB_Cancellation(int32_t led_val, int32_t amb_uA)
{

	if(systemStatus.blHRSensorOn == false)
		return;

	AFE44xx_SW_CMD(AFE_REG_WRITE);
	//AFE44xx_Reg_Write(AFE_LEDCNTRL, 0x00000 | (led_val << 8)); //orignal
	AFE44xx_Reg_Write(AFE_LEDCNTRL, 0x10000 | ((led_val & 0x000000FF) << 8)); //[BG013] atus 0x00000 >> 0x10000

	if(amb_uA > 0)
        {
                if (amb_uA>10)
                  amb_uA = 10; //refer AFE4400.pdf p64 0-10
		//AFE44xx_Reg_Write(AFE_TIA_AMB_GAIN, 0x00403B | (amb_uA << 16));
		AFE44xx_Reg_Write(AFE_TIA_AMB_GAIN, 0x00403B | ((amb_uA & 0x0000000F)<< 16));
        }
	else
		AFE44xx_Reg_Write(AFE_TIA_AMB_GAIN, 0x00003B);

	AFE44xx_SW_CMD(AFE_REG_READ); // set command back to read
}


/**********************************************************************************************************
*	        AFE44xx_Read_All_Regs          				                  					  *
**********************************************************************************************************/

void AFE44xx_Read_All_Regs(uint32_t AFE44xxeg_buf[])
{
	unsigned char Regs_i;

	for ( Regs_i = 0; Regs_i < 50; Regs_i++)
	{
		AFE44xxeg_buf[Regs_i] = AFE44xx_Reg_Read(Regs_i);
	}
}

void AFE44xx_Shutoff(void)
{

//	if((systemSetting.blHRSensorEnabled == false) && (systemStatus.blHRSensorEnabled == false))
//		return;

	if(systemStatus.blHRSensorOn == false)
		return;

	//===========================20140730
	/* Disable interrupts */
	INT_Disable();

	systemStatus.blHRSensorOn = false;

	GPIO_IntClear(1 << AFE_ADC_DRDY_PIN);
	GPIO_IntConfig(AFE_ADC_DRDY_PORT, AFE_ADC_DRDY_PIN, true, false, false);
	/* Initialization done, enable interrupts globally. */
	INT_Enable();

	//===================================



	//===========================
	// 通知 sensor 关闭
	MESSAGE msg;
	msg.params.type = (UINT16) MESSAGE_SENSOR_DEACTIVATED;
	msg.params.param = (UINT16) SENSOR_TYPE_PPG;
	xQueueSend(hEvtQueueDevice, &msg.id, 0);

	CMU_ClockEnable( AFE_SPI_CMUCLOCK, false );//opening  USART clock involves about 1uA
	AFE_RESETZ_L();
	BOTTOM_POWER_OFF();

	GPIO_PinModeSet( AFE_SPI_GPIOPORT, AFE_SPI_CLKPIN, gpioModeDisabled, 0 );
	GPIO_PinModeSet( AFE_SPI_GPIOPORT, AFE_SPI_MOSIPIN, gpioModeDisabled, 0 );
	GPIO_PinModeSet( AFE_SPI_GPIOPORT, AFE_SPI_MISOPIN, gpioModeDisabled, 0 );
	GPIO_PinModeSet( AFE_CS_PORT, AFE_CS_PIN, gpioModeDisabled, 0 );

	iHeartRate.data = 0;
#if (MODEL_TYPE==1) //HEALTHCARE_TYPE
	systemStatus.blHeartBeatLock = false;
#elif (MODEL_TYPE==2) //CONSUMER_TYPE
	//systemStatus.blHeartBeatLock = false;
#endif
	Mems_WakeUp();
	//MEMS_FIFO_INIT();

}

void AFE44xx_PowerOn_Init()
{
	if((systemSetting.blHRSensorEnabled == false) || (systemStatus.blHRSensorTempEnabled == false))
		return;

	if(systemStatus.blHRSensorOn == true)
		return;

	systemStatus.blHRSensorOn = true;


	//===========================
	// 通知 sensor 打开
	MESSAGE msg;
	msg.params.type = (UINT16) MESSAGE_SENSOR_ACTIVATED;
	msg.params.param = (UINT16) SENSOR_TYPE_PPG;
	xQueueSend(hEvtQueueDevice, &msg.id, 0);

	BOTTOM_POWER_ON();
	/* Setup clocks */
	CMU_ClockEnable( cmuClock_GPIO, true );
	//GPIO_PinModeSet(AFE_POWER_CON_PORT,AFE_POWER_CON_PIN,gpioModePushPull,1);
	GPIO_PinModeSet(AFE_RESETZ_PORT, AFE_RESETZ_PIN, gpioModePushPull, 1);
	AFE_RESETZ_L();
//	osDelay(20);
	vTaskDelay(20);
	AFE_RESETZ_H();
//	osDelay(2);
	vTaskDelay(2);
	Init_AFE44xx_Resource();
	Init_AFE44xx_DRDY_Interrupt();

	AFE44xx_SW_CMD(AFE_REG_WRITE);
	AFE44xx_Reg_Write(1, 0x55); // next set read back to save time
	AFE44xx_SW_CMD(AFE_REG_READ);

	systemStatus.blHRSensorOnline = 0;
        
	if(AFE44xx_Reg_Read(1) == 0x55)
	{
		systemStatus.blHRSensorOnline = 1;
		AFE44xx_Default_Reg_Init();
#if BG013MS
                //LED_INTENSITY and AMB_uA already set by LoadSystemSettings()
#else
		AMB_uA = AMB_UA_MS_MIN; //[BG013-2] 0 >> AMB_UA_MS_MIN
		LED_INTENSITY = LED_Intensity_Init_val; //[BG013] 0 >> LED_Intensity_Init_val
#endif
		LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);

		Init_AFE44xx_DRDY_Interrupt();
		Enable_AFE44xx_DRDY_Interrupt();
		ambient_light_low_counter = 0;
		PPG_INIT();
		MEMS_NO_FIFO_INIT();

		afe_sample_counter = 0;
	}

}



#define Low_Level 1
#define Mid_Level 2
#define Hig_Level 3


/* BG013MS add */
#if 0
#define LED_INTENSITY_MS_MAX    200
#define LED_INTENSITY_MS_MIN    15
#define AMB_UA_MS_MAX   5 //10 keep as original assigned
#define AMB_UA_MS_MIN   0
#endif

#define LED_INTENSITY_MAX  LED_INTENSITY_MS_MAX  //[BG013] 200 >> 100 >> LED_INTENSITY_MS_MAX
#define AMB_uA_MAX AMB_UA_MS_MAX //[BG013] 5 >> AMB_UA_MS_MAX

uint8_t AFE_AN_Voltage = 0;
//static int8_t my_memsPeriod = 0;

void AFE_DATA_PROC(void)
{
	//static uint8_t delay_count = 0;
	int i;

#if BG013
        if (read_IR_reg>DBG_IR_reg_max)
            DBG_IR_reg_max = read_IR_reg;
        if (read_IR_reg<DBG_IR_reg_min)
            DBG_IR_reg_min = read_IR_reg;
        if (read_AB_reg>DBG_AB_reg_max)
            DBG_AB_reg_max = read_AB_reg;
        if (read_AB_reg<DBG_AB_reg_min)
            DBG_AB_reg_min = read_AB_reg;        
#endif
	// if AFE4400 is power off now , fail to read spi port, system will be reset
	if(systemStatus.blHRSensorOn == false)
		return;

	//=======================================
#if BG013MS
        AFE_AN_Voltage = Mid_Level;  //force to adjust ready.
#else
#if (BG013_LED_ADJ_METHOD==0)
	if(read_IR_reg < WORKING_L_LEVEL)
	{
		AFE_AN_Voltage = Low_Level;
	}
	else if(read_IR_reg > WORKING_H_LEVEL)
	{
		AFE_AN_Voltage = Hig_Level;
	}

	if(AFE_AN_Voltage == Low_Level)
	{
		if(read_IR_reg < WORKING_M_LEVEL)
		{
			LED_INTENSITY += 2;

			if(LED_INTENSITY > LED_INTENSITY_MAX)   //[BG013-1] '>' >> '>='
			{
				LED_INTENSITY = LED_INTENSITY_MAX;
				AFE_AN_Voltage = Mid_Level;
			}
			else LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
		}
		else if(read_IR_reg > WORKING_H_LEVEL)
		{
			if((LED_INTENSITY > LED_INTENSITY_MAX) && (AMB_uA < AMB_uA_MAX)) //[BG013-1] '>' >> '>='
			{
				AMB_uA++;
				LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
			}

			//else  AFE_AN_Voltage=Mid_Level;
		}
		else
		{
			AFE_AN_Voltage = Mid_Level;
		}
	}
	else if(AFE_AN_Voltage == Hig_Level)
	{
		if(read_IR_reg > WORKING_M_LEVEL)
		{
			if(read_AB_reg > (-WORKING_M_LEVEL))
			{
				AMB_uA++;

				if(AMB_uA > AMB_uA_MAX)
				{
					AMB_uA = AMB_uA_MAX;

					if(LED_INTENSITY) LED_INTENSITY--;
				}
			}
			else
			{
				if(LED_INTENSITY)LED_INTENSITY--;
			}

			LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
		}
		else
		{
			AFE_AN_Voltage = Mid_Level;
		}

	}
#elif (BG013_LED_ADJ_METHOD==1)
#if 0
        if (abs(read_IR_reg)>WORKING_H_LEVEL) {
          if (LED_INTENSITY > LED_INTENSITY_MS_MIN) {
                LED_INTENSITY -= 2;
                light_locked_Counter = 0;
          }
        }
#endif
	if(read_IR_reg < WORKING_L_LEVEL)
	{
		AFE_AN_Voltage = Low_Level;
	}
	else if(read_IR_reg > WORKING_H_LEVEL)
	{
		AFE_AN_Voltage = Hig_Level;
	}

	if(AFE_AN_Voltage == Low_Level)
	{
		if(read_IR_reg < WORKING_M_LEVEL)
		{
			LED_INTENSITY += 2;

			if(LED_INTENSITY > LED_INTENSITY_MAX)   //[BG013-1] '>' >> '>='
			{
				LED_INTENSITY = LED_INTENSITY_MAX;
				AFE_AN_Voltage = Mid_Level;
			}
			else LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
		}
		else if(read_IR_reg > WORKING_H_LEVEL)
		{
			if((LED_INTENSITY > LED_INTENSITY_MAX) && (AMB_uA < AMB_uA_MAX)) //[BG013-1] '>' >> '>='
			{
				AMB_uA++;
				LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
			}

			//else  AFE_AN_Voltage=Mid_Level;
		}
		else
		{
			AFE_AN_Voltage = Mid_Level;
		}
	}
	else if(AFE_AN_Voltage == Hig_Level)
	{
		if(read_IR_reg > WORKING_M_LEVEL)
		{
			if(read_AB_reg > (-WORKING_M_LEVEL))
			{
				AMB_uA++;

				if(AMB_uA > AMB_uA_MAX)
				{
					AMB_uA = AMB_uA_MAX;

					if(LED_INTENSITY) LED_INTENSITY--;
				}
			}
			else
			{
				if(LED_INTENSITY)LED_INTENSITY--;
			}

			LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
		}
		else
		{
			AFE_AN_Voltage = Mid_Level;
		}
	}
#else
#error not specify adjust algorithm!
#endif
#endif
//====================================
	ReadMemsFIFO((uint8_t*)&axie[0], (uint8_t*)&axie[1], (uint8_t*)&axie[2], 1);
	extern uint16_t PPG_IN_1S_WP;

	for(i = 0; i < 3; i++)
	{
		MEMS_BUFF[i][PPG_IN_1S_WP] = axie[i];
		AxieOUT[i] = axie[i];
	}

	PPG_IN_1S_WP++;

	if(PPG_IN_1S_WP == 32) //PPG_1S_LEN)
	{
		PPG_IN_1S_WP = 0;
		MEMS_TRACKING(&MEMS_BUFF[0][0],
		              &MEMS_BUFF[1][0],
		              &MEMS_BUFF[2][0], MEMS_FIFO_SIZE);
	}
#if BG013MS   //BG013
	if(1 || AFE_AN_Voltage == Mid_Level) //debug force enter PPG_PROCESSING even not ready.
#else
        if(AFE_AN_Voltage == Mid_Level) //stable level enter PPG_PROCESSING.
#endif
        {
//		ReadMemsFIFO((uint8_t*)&axie[0], (uint8_t*)&axie[1], (uint8_t*)&axie[2], 1); //
		//TEST_H();
		PPG_PROCESSING();

#ifdef  PPG2Dongle  // 
		output_ppg_22bit = read_IR_reg_22bit;

		for(int i = 0; i < 3; i++)
			AxieOUT[i] = axie[i];

#endif
	}
#if 0 //BG013
        else {
          PPG_OUT_DATA = DSP_PROCESSING(read_IR_reg_22bit, read_AB_reg_22bit);
        }
#endif        

	//TEST_L();
	extern uint8_t RealDataOnPhone;

	if(RealDataOnPhone)
	{

		SendRealDataOverBLE();
	}

//======================================
}



//================================
//================================
// afe4400  Vref=+- 1.2V

// 22bit adc , max=0x1fffff ,  Vx=(x/0x200000)*(1.2V)
// convert to long ,   if(X&0x200000) , longX = X|0xffe00000
//================================
//================================


void AFE_READ(void)     //No one call it?
{

	//read_IR_reg= AFE44xx_Reg_Read(44);  //read IR Data
	//read_IR_ambient_reg= AFE44xx_Reg_Read(45);  //read Ambient Data
	//AFE44xx_SPO2_Data_buf[0] = AFE44xx_Reg_Read(42);  //read RED Data
	//AFE44xx_SPO2_Data_buf[1] = AFE44xx_Reg_Read(43);  //read Ambient data
	//AFE44xx_SPO2_Data_buf[4] = AFE44xx_Reg_Read(46);  //read RED - Ambient Data
	//AFE44xx_SPO2_Data_buf[5] = AFE44xx_Reg_Read(47);  //read IR - Ambient Data

	// as set read back after all write command ,mask this line
	//AFE44xx_SW_CMD(AFE_REG_READ);

	int temp = AFE44xx_Reg_Read(44);

	if(temp & 0x00200000) // if value is minus
		temp |= 0xffe00000;

	read_IR_reg_22bit = temp;
	read_IR_reg = (int16_t)(read_IR_reg_22bit >> 6);

	temp = AFE44xx_Reg_Read(45);

	if(temp & 0x00200000) // if value is minus
		temp |= 0xffe00000;

	read_AB_reg = (int16_t)(temp >> 6);



}



void AFE_INT_CALLBACK(void)
{
	// if AFE4400 is power off now , fail to read spi port, system will be reset
	if(systemStatus.blHRSensorOn == false)
		return;

	static bool to32HZ = false;  //control average 2 point to one point.(64Hz to 32Hz)
	static int32_t ppg_bak = 0, amb_bak = 0;
	int32_t ppg_temp = AFE44xx_Reg_Read(44);

	if(ppg_temp & 0x00200000) // if value is minus
		ppg_temp |= 0xffe00000;

	int32_t amb_temp = AFE44xx_Reg_Read(45);

	if(amb_temp & 0x00200000) // if value is minus
		amb_temp |= 0xffe00000;

	if(to32HZ == false)
	{
		ppg_bak = ppg_temp;
		amb_bak = amb_temp;

		to32HZ = true;
		return;
	}

	to32HZ = false;

	read_IR_reg_22bit = (ppg_temp + ppg_bak) / 2;
	read_IR_reg = (int16_t)(read_IR_reg_22bit >> 6);

        read_AB_reg_22bit = (amb_temp + amb_bak) / 2;
	read_AB_reg = (int16_t)((amb_temp + amb_bak) >> 7); //  >>6 + >>1

	afe_sample_counter++;

        //utest = AFE44xx_Reg_Read(AFE_CONTROL2); //35
        //utest = read_IR_reg_22bit;

	//
//	osMessagePut(hMsgInterrupt, AFE_Message, 0);  //
	uint32_t msg = AFE_Message;
	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);
}

