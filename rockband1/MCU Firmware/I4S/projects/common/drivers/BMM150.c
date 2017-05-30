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

#include "BMM150.h"

#if (MAGNETIC_SUPPORT==1)

#define Magnetic_BUFF_SIZE 32

uint8_t Magnetic_BUFF[Magnetic_BUFF_SIZE][BMM150_DATA_FRAME_SIZE];
uint8_t Magnetic_BUFF_Count=0;

static struct BMM150_t *p_BMM150;

BMM150_RETURN_FUNCTION_TYPE BMM150_read_register(u8 v_addr_u8,
u8 *v_data_u8)
{

	BMM150_CS_L();
	USART_Tx(BMM150_SPI, 0x80 + v_addr_u8);
	uint8_t temp = USART_Rx(BMM150_SPI);
	USART_Tx(BMM150_SPI, 0);
	*v_data_u8 = USART_Rx(BMM150_SPI);
	BMM150_CS_H();
	
	return 1;
}

BMM150_RETURN_FUNCTION_TYPE BMM150_write_register(u8 v_addr_u8,
u8 v_data_u8)
{

	BMM150_CS_L();
	USART_Tx(BMM150_SPI, v_addr_u8);
	USART_Rx(BMM150_SPI);
	USART_Tx(BMM150_SPI, v_data_u8);
	USART_Rx(BMM150_SPI);
	BMM150_CS_H();
	
	return 1;
}
/*!
 *	@brief This API used to set the power control bit
 *	in the register 0x4B bit 0
 *
 *
 *
 *  @param v_power_mode_u8 : The value of power control bit enable
 *   value     |  status
 *  -----------|------------
 *      0      | Disable the power control bit
 *      1      | Enable the power control bit
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_set_power_mode(u8 v_power_mode_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;
	
	{
		
		com_rslt = BMM150_read_register(BMM150_POWER_CONTROL_POWER_CONTROL_BIT__REG,&v_data_u8);

		v_data_u8 &= (~BMM150_POWER_CONTROL_POWER_CONTROL_BIT__MSK);
		v_data_u8 |= v_power_mode_u8;
		
		
		com_rslt = BMM150_write_register(BMM150_POWER_CONTROL_POWER_CONTROL_BIT__REG,v_data_u8);
	}
	return com_rslt;
}
/*!
 *	@brief This API used to set the preset modes
 *
 *	@note The preset mode setting is
 *	depend on Data Rate, XY and Z repetitions
 *
 *
 *
 *  @param v_presetmode_u8: The value of selected preset mode
 *  value    | preset_mode
 * ----------|-----------------
 *    1      | BMM150_PRESETMODE_LOWPOWER
 *    2      | BMM150_PRESETMODE_REGULAR
 *    3      | BMM150_PRESETMODE_HIGHACCURACY
 *    4      | BMM150_PRESETMODE_ENHANCED
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_set_presetmode(u8 v_presetmode_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;


	switch (v_presetmode_u8) {
	case BMM150_PRESETMODE_LOWPOWER:
		/* Set the data rate for Low Power mode */
		com_rslt = BMM150_set_data_rate(BMM150_LOWPOWER_DR);
		/* Set the XY-repetitions number for Low Power mode */
		com_rslt += BMM150_set_rep_XY(BMM150_LOWPOWER_REPXY);
		/* Set the Z-repetitions number  for Low Power mode */
		com_rslt += BMM150_set_rep_Z(BMM150_LOWPOWER_REPZ);
		break;
	case BMM150_PRESETMODE_REGULAR:
		/* Set the data rate for Regular mode */
		com_rslt = BMM150_set_data_rate(BMM150_REGULAR_DR);
		/* Set the XY-repetitions number for Regular mode */
		com_rslt += BMM150_set_rep_XY(BMM150_REGULAR_REPXY);
		/* Set the Z-repetitions number  for Regular mode */
		com_rslt += BMM150_set_rep_Z(BMM150_REGULAR_REPZ);
		break;
	case BMM150_PRESETMODE_HIGHACCURACY:
		/* Set the data rate for High Accuracy mode */
		com_rslt = BMM150_set_data_rate(BMM150_HIGHACCURACY_DR);
		/* Set the XY-repetitions number for High Accuracy mode */
		com_rslt += BMM150_set_rep_XY(BMM150_HIGHACCURACY_REPXY);
		/* Set the Z-repetitions number  for High Accuracy mode */
		com_rslt += BMM150_set_rep_Z(BMM150_HIGHACCURACY_REPZ);
		break;
	case BMM150_PRESETMODE_ENHANCED:
		/* Set the data rate for Enhanced Accuracy mode */
		com_rslt = BMM150_set_data_rate(BMM150_ENHANCED_DR);
		/* Set the XY-repetitions number for High Enhanced mode */
		com_rslt += BMM150_set_rep_XY(BMM150_ENHANCED_REPXY);
		/* Set the Z-repetitions number  for High Enhanced mode */
		com_rslt += BMM150_set_rep_Z(BMM150_ENHANCED_REPZ);
		break;
	default:
		com_rslt = E_BMM150_OUT_OF_RANGE;
		break;
	}
	return com_rslt;
}

void BMM150_debug(void)
{
    uint8_t ui=0x4B, u8_data;

    BMM150_read_register(0x4B, &u8_data);
    if ((u8_data&0x01)==0x00)
    {
        BMM150_write_register(BMM150_POWER_CONTROL,0x01);
        SysCtlDelay(3000);  // 2ms at least.
    }   
    for (ui=0x40 ; ui<0x52 ; ui++)
    {
        BMM150_read_register(ui, &u8_data);
        u8_data = 0x00;
    }
}

void BMM150_Disabled(void)
{
  if (systemStatus.blGeoMSensorOnline==0x01) //&& 0
  {
    //reset chip to sleep.
	BMM150_write_register(BMM150_POWER_CONTROL,0x82);
    SysCtlDelay(4000);  // 2ms at least.
    BMM150_set_power_mode(BMM150_OFF);
  }
  systemStatus.blGeoMSensorOnline = false;
#if 0
  GPIO_IntConfig(BMM150_INT_PORT, BMM150_INT_PIN, false, true, false);
  GPIO_IntClear(1 << BMM150_INT_PIN); 	
  GPIO_PinModeSet(BMM150_INT_PORT, BMM150_INT_PIN, gpioModeDisabled, 1);

  GPIO_IntConfig(BMM150_INT_PORT, BMM150_DRDY_PIN, false, true, false);	
  GPIO_IntClear(1 << BMM150_DRDY_PIN); 	
  GPIO_PinModeSet(BMM150_INT_PORT, BMM150_DRDY_PIN, gpioModeDisabled, 1);

  GPIO_PinModeSet(BMM150_CS_PORT, BMM150_CS_PIN, gpioModeDisabled, 1);
#endif
}

/*!
 *	@brief This function is used for initialize
 *	bus read and bus write functions
 *	assign the chip id and device address
 *	chip id is read in the register 0x40 bit from 0 to 7
 *
 *	@note While changing the parameter of the BMM150
 *	consider the following point:
 *	@note Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_Init(uint8_t mode)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	/*Array holding the mag chip id
	v_data_u8[0] - chip id
	*/
	///u8 v_data_u8[BMM150_INIT_DATA_SIZE] = {BMM150_INIT_VALUE, BMM150_INIT_VALUE};


	///p_BMM150 = BMM150;

	uint8_t p_BMM150_id ;
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(BMM150_CS_PORT, BMM150_CS_PIN, gpioModePushPull, 1);
#if 0
	/*Read CHIP_ID and REv. info */
	com_rslt = 	BMM150_read_register(BMM150_ID_Reg,&p_BMM150_id);
	if(p_BMM150_id == BMM150_ID)
	{
		systemStatus.blGeoMSensorOnline = 1;
		////SetPOWERMode(POWER_DOWN);
	}
	else
    {
		systemStatus.blGeoMSensorOnline = 0;
        return BMM150_NOTEXIST;
    }
    if (mode==0)
    {
      BMM150_Disabled();
      return BMM150_SUCCESS;
    }
#endif    
#if 1 /// int tset
	GPIO_PinModeSet(BMM150_INT_PORT, BMM150_INT_PIN, gpioModeInput, 1);
	GPIO_IntConfig(BMM150_INT_PORT, BMM150_INT_PIN, false, true, true);
	GPIO_IntClear(1 << BMM150_INT_PIN); 	
	NVIC_SetPriority(GPIO_ODD_IRQn,GPIO_ODD_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	
	GPIO_PinModeSet(BMM150_INT_PORT, BMM150_DRDY_PIN, gpioModeInput, 1);
	GPIO_IntConfig(BMM150_INT_PORT, BMM150_DRDY_PIN, false, true, true);	
	GPIO_IntClear(1 << BMM150_DRDY_PIN); 	
	NVIC_SetPriority(GPIO_EVEN_IRQn,GPIO_EVEN_INT_LEVEL);
	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
#endif

    BMM150_write_register(BMM150_POWER_CONTROL, 0x00); //into suspend
    BMM150_write_register(BMM150_POWER_CONTROL, 0x82); //reset
    SysCtlDelay(6000);
    BMM150_write_register(BMM150_CONTROL, BMM150_SLEEP_MODE);
    
	/* set device from suspend into sleep mode */
	////com_rslt = BMM150_set_power_mode(BMM150_ON);
	BMM150_write_register(BMM150_POWER_CONTROL,0x01);
    SysCtlDelay(7000);  // 2ms at least.
    //BMM150_debug();
    //
	BMM150_write_register(BMM150_CONTROL,0x00);
    com_rslt = BMM150_set_functional_state(BMM150_NORMAL_MODE);


	/* wait two millisecond for bmc to settle */
	////p_BMM150->delay_msec(BMM150_DELAY_SETTLING_TIME);

#if 1
	/*Read CHIP_ID and REv. info */
	com_rslt = 	BMM150_read_register(BMM150_ID_Reg,&p_BMM150_id);
	if(p_BMM150_id == BMM150_ID)
	{
		systemStatus.blGeoMSensorOnline = 1;
		////SetPOWERMode(POWER_DOWN);
	}
	else
    {
		systemStatus.blGeoMSensorOnline = 0;
        return BMM150_NOTEXIST;
    }

    if (mode==0)
    {
      BMM150_Disabled();
    }
#endif

	/* Function to initialise trim values */
	///com_rslt += BMM150_init_trim_registers();
	/* set the preset mode as regular*/
	com_rslt += BMM150_set_presetmode(BMM150_PRESETMODE_REGULAR);
	/// BMM150_write_register(BMM150_INT_CONTROL,0x80);	/// int tset
	BMM150_set_data_rate(BMM150_DR_15HZ);  //BMM150_DR_30HZ

	BMM150_write_register(BMM150_SENS_CONTROL,0x83);
	return com_rslt;
}
/*!
 *	@brief This API used to set the functional state
 *	in the register 0x4C and 0x4B
 *	@note 0x4C bit 1 and 2
 *	@note 0x4B bit 0
 *
 *
 *  @param  v_functional_state_u8: The value of functional mode
 *  value     |   functional state
 * -----------|-------------------
 *   0x00     | BMM150_NORMAL_MODE
 *   0x01     | BMM150_SUSPEND_MODE
 *   0x02     | BMM150_FORCED_MODE
 *   0x03     | BMM150_SLEEP_MODE
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_set_functional_state(
u8 v_functional_state_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;

	{
		/* select the functional state*/
		switch (v_functional_state_u8) {
		/* write the functional state*/
		case BMM150_NORMAL_MODE:
			com_rslt = BMM150_get_power_mode(&v_data_u8);
			if (v_data_u8 == BMM150_OFF) {
				com_rslt += BMM150_set_power_mode(BMM150_ON);
				///p_BMM150->delay_msec(BMM150_DELAY_SUSPEND_SLEEP);
			}

			BMM150_read_register(BMM150_CONTROL_OPERATION_MODE__REG,&v_data_u8);
			v_data_u8 &= (~BMM150_CONTROL_OPERATION_MODE__MSK);
			v_data_u8 |= BMM150_NORMAL_MODE;			
			BMM150_write_register(BMM150_CONTROL_OPERATION_MODE__REG,v_data_u8);			
			break;
			
		case BMM150_SUSPEND_MODE:
			com_rslt = BMM150_set_power_mode(BMM150_OFF);
			break;
		case BMM150_FORCED_MODE:
			com_rslt = BMM150_get_power_mode(&v_data_u8);
			if (v_data_u8 == BMM150_OFF) {
				com_rslt += BMM150_set_power_mode(BMM150_ON);
				///p_BMM150->delay_msec(BMM150_DELAY_SUSPEND_SLEEP);
			}
			
			BMM150_read_register(BMM150_CONTROL_OPERATION_MODE__REG,&v_data_u8);
			v_data_u8 &= (~BMM150_CONTROL_OPERATION_MODE__MSK);
			v_data_u8 |= BMM150_FORCED_MODE;			
			BMM150_write_register(BMM150_CONTROL_OPERATION_MODE__REG,v_data_u8);
			break;
		case BMM150_SLEEP_MODE:
			com_rslt = BMM150_get_power_mode(&v_data_u8);
			if (v_data_u8 == BMM150_OFF) {
				com_rslt += BMM150_set_power_mode(BMM150_ON);
				///p_BMM150->delay_msec(BMM150_DELAY_SUSPEND_SLEEP);
			}
			
			BMM150_read_register(BMM150_CONTROL_OPERATION_MODE__REG,&v_data_u8);
			v_data_u8 &= (~BMM150_CONTROL_OPERATION_MODE__MSK);
			v_data_u8 |= BMM150_SLEEP_MODE;			
			BMM150_write_register(BMM150_CONTROL_OPERATION_MODE__REG,v_data_u8);
			break;
		default:
			com_rslt = E_BMM150_OUT_OF_RANGE;
			break;
		}
	}
	return com_rslt;
}
/*!
 * @brief This API reads compensated Magnetometer
 * data of X,Y,Z values
 * from location 0x42 to 0x49
 *
 *
 *
 *
 *  @param  mag_data : The data of mag compensated XYZ data
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
///struct BMM150_mag_data_s16_t BMM150_mag_data;

BMM150_RETURN_FUNCTION_TYPE BMM150_read_mag_data_XYZ(void)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
    
    if (systemStatus.blGeoMSensorOnline==false)
    {
      return BMM150_NOTONLINE;
    }
	/* Array holding the mag XYZ and R data
	v_data_u8[0] - X LSB
	v_data_u8[1] - X MSB
	v_data_u8[2] - Y LSB
	v_data_u8[3] - Y MSB
	v_data_u8[4] - Z LSB
	v_data_u8[5] - Z MSB
	v_data_u8[6] - R LSB
	v_data_u8[7] - R MSB
	*/
	u8 v_data_u8[BMM150_DATA_FRAME_SIZE] = {
	BMM150_INIT_VALUE, BMM150_INIT_VALUE,
	BMM150_INIT_VALUE, BMM150_INIT_VALUE,
	BMM150_INIT_VALUE, BMM150_INIT_VALUE,
	BMM150_INIT_VALUE, BMM150_INIT_VALUE};
	

	uint8_t loop,DRDYbit;

	{
		
        {
		
          for(loop=0;loop<8;loop++)
            ///BMM150_read_register((BMM150_DATA_X_LSB+loop),&v_data_u8[loop]);
            BMM150_read_register((BMM150_DATA_X_LSB+loop),&Magnetic_BUFF[Magnetic_BUFF_Count][loop]);
		  
			Magnetic_BUFF_Count++;
			
			if(Magnetic_BUFF_SIZE == Magnetic_BUFF_Count)
				{
				/// Magnetic_BUFF full
				Magnetic_BUFF_Count=0;
				}

			return BMM150_SUCCESS;
        }
	
	}
	return com_rslt;
}
/*!
 *	@brief This API used to set the data rate of the sensor
 *	in the register 0x4C bit 3 to 5
 *
 *
 *
 *  @param  v_data_rate_u8 : The value of data rate
 *  value     |       Description
 * -----------|-----------------------
 *   0x00     |  BMM150_DATA_RATE_10HZ
 *   0x01     |  BMM150_DATA_RATE_02HZ
 *   0x02     |  BMM150_DATA_RATE_06HZ
 *   0x03     |  BMM150_DATA_RATE_08HZ
 *   0x04     |  BMM150_DATA_RATE_15HZ
 *   0x05     |  BMM150_DATA_RATE_20HZ
 *   0x06     |  BMM150_DATA_RATE_25HZ
 *   0x07     |  BMM150_DATA_RATE_30HZ
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_set_data_rate(u8 v_data_rate_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;
	/* check the p_BMM150 pointer is NULL*/
	///if (p_BMM150 == BMM150_NULL) {
	///	return  E_BMM150_NULL_PTR;
	///	} else 
		{
	
		/* set the data rate */

		com_rslt = BMM150_read_register(
			BMM150_CONTROL_DATA_RATE__REG,&v_data_u8);
		
		v_data_u8 &= (~BMM150_CONTROL_DATA_RATE__MSK);		
		v_data_u8 |= v_data_rate_u8;
		
		com_rslt = BMM150_write_register(
			BMM150_CONTROL_DATA_RATE__REG,v_data_u8);

	}
	return com_rslt;
}

BMM150_RETURN_FUNCTION_TYPE BMM150_Get_data_rate(u8 *v_data_rate_u8)
{
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;
		com_rslt = BMM150_read_register(
			BMM150_CONTROL_DATA_RATE__REG,&v_data_u8);
		
		v_data_u8 &= (BMM150_CONTROL_DATA_RATE__MSK);		
		*v_data_rate_u8 = v_data_u8 >> 3;
		
 /*	in the register 0x4C bit 3 to 5
 *
 *  @param  v_data_rate_u8 : The value of data rate
 *  value     |       Description
 * -----------|-----------------------
 *   0x00     |  BMM150_DATA_RATE_10HZ
 *   0x01     |  BMM150_DATA_RATE_02HZ
 *   0x02     |  BMM150_DATA_RATE_06HZ
 *   0x03     |  BMM150_DATA_RATE_08HZ
 *   0x04     |  BMM150_DATA_RATE_15HZ
 *   0x05     |  BMM150_DATA_RATE_20HZ
 *   0x06     |  BMM150_DATA_RATE_25HZ
 *   0x07     |  BMM150_DATA_RATE_30HZ
 */
	return com_rslt;
}
/*!
 *	@brief This API used to get the power control bit
 *	in the register 0x4B bit 0
 *
 *
 *
 *  @param v_power_mode_u8 : The value of power control bit enable
 *   value     |  status
 *  -----------|------------
 *      0      | Disable the power control bit
 *      1      | Enable the power control bit
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_get_power_mode(u8 *v_power_mode_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;
	{
		/* read power control bit */
		com_rslt = BMM150_read_register(
		BMM150_POWER_CONTROL_POWER_CONTROL_BIT__REG,&v_data_u8);
		if (v_data_u8&BMM150_POWER_CONTROL_POWER_CONTROL_BIT__MSK)
			*v_power_mode_u8 = BMM150_ON;
			else
			*v_power_mode_u8 = BMM150_OFF;
			
	}
	return com_rslt;
}
/*!
 *	@brief This API used to set the x and y
 *	repetition in the register 0x51 bit 0 to 7
 *
 *
 *
 *  @param v_rep_xy_u8 : The value of x and y repetitions
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_set_rep_XY(
u8 v_rep_xy_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;
	/* check the p_BMM150 pointer is NULL*/
	if (p_BMM150 == BMM150_NULL) {
		return  E_BMM150_NULL_PTR;
		} else {
		/* write XY repetitions*/
		v_data_u8 = v_rep_xy_u8;
		BMM150_write_register(BMM150_REP_XY,v_data_u8);
	}
	return com_rslt;
}
/*!
 *	@brief This API used to set the z repetition in the
 *	register 0x52 bit 0 to 7
 *
 *
 *
 *  @param v_rep_z_u8 : The value of z repetitions
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
*/
BMM150_RETURN_FUNCTION_TYPE BMM150_set_rep_Z(
u8 v_rep_z_u8)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
	u8 v_data_u8 = BMM150_INIT_VALUE;
	/* check the p_BMM150 pointer is NULL*/
	if (p_BMM150 == BMM150_NULL) {
		return  E_BMM150_NULL_PTR;
		} else {
		/* write Z repetitions*/
		v_data_u8 = v_rep_z_u8;
		BMM150_write_register(BMM150_REP_Z,v_data_u8);
	}
	return com_rslt;
}
#endif
