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
BMM150_RETURN_FUNCTION_TYPE BMM150_init(void)
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
	///GPIO_PinModeSet(L3GD20H_DEN_PORT, L3GD20H_DEN_PIN, gpioModePushPull, 1);
	///GPIO_PinOutClear(L3GD20H_DEN_PORT,L3GD20H_DEN_PIN );
	GPIO_PinModeSet(BMM150_CS_PORT, BMM150_CS_PIN, gpioModePushPull, 1);


	/* set device from suspend into sleep mode */
	com_rslt = BMM150_set_power_mode(BMM150_ON);


	/* wait two millisecond for bmc to settle */
	////p_BMM150->delay_msec(BMM150_DELAY_SETTLING_TIME);


	/*Read CHIP_ID and REv. info */
	com_rslt = 	BMM150_read_register(BMM150_ID_Reg,&p_BMM150_id);
	

	if(p_BMM150_id == BMM150_ID)
	{
		systemStatus.blGeoMSensorOnline = 1;
		////SetPOWERMode(POWER_DOWN);
	}


	/* Function to initialise trim values */
	///com_rslt += BMM150_init_trim_registers();
	/* set the preset mode as regular*/
	com_rslt += BMM150_set_presetmode(BMM150_PRESETMODE_REGULAR);
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
struct BMM150_mag_data_s16_t BMM150_mag_data;

BMM150_RETURN_FUNCTION_TYPE BMM150_read_mag_data_XYZ(void)
{
	/* variable used to return the bus communication result*/
	BMM150_RETURN_FUNCTION_TYPE com_rslt = BMM150_ERROR;
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
	

	uint8_t loop;

	{
		for(loop=0;loop<8;loop++)
		BMM150_read_register((BMM150_DATA_X_LSB+loop),&v_data_u8[loop]);

		BMM150_mag_data.datax = 
		((v_data_u8[BMM150_XLSB_DATA]+v_data_u8[BMM150_XMSB_DATA]*0x100)>>3);
		
		BMM150_mag_data.datay = 
		((v_data_u8[BMM150_YLSB_DATA]+v_data_u8[BMM150_YMSB_DATA]*0x100)>>3);
		
		BMM150_mag_data.dataz = 
		((v_data_u8[BMM150_ZLSB_DATA]+v_data_u8[BMM150_ZMSB_DATA]*0x100)>>1);

		// two��s complement
		BMM150_mag_data.resistance = 
		((v_data_u8[BMM150_RLSB_DATA]+v_data_u8[BMM150_RMSB_DATA]*0x100)>>2);
		
		/* read the mag xyz and r data*/
		com_rslt = p_BMM150->BMM150_BUS_READ_FUNC(p_BMM150->dev_addr,
		BMM150_DATA_X_LSB, v_data_u8, BMM150_ALL_DATA_FRAME_LENGTH);

#if 0
		if (!com_rslt) {
			/* Reading data for X axis */
			v_data_u8[BMM150_XLSB_DATA] =
			BMM150_GET_BITSLICE(v_data_u8[BMM150_XLSB_DATA],
			BMM150_DATA_X_LSB_BIT);
			raw_data_xyz_t.raw_data_x = (s16)((((s32)
			((s8)v_data_u8[BMM150_XMSB_DATA])) <<
			BMM150_SHIFT_BIT_POSITION_BY_05_BITS)
			| v_data_u8[BMM150_XLSB_DATA]);


			/* Reading data for Y axis */
			v_data_u8[BMM150_YLSB_DATA] =
			BMM150_GET_BITSLICE(v_data_u8[BMM150_YLSB_DATA],
			BMM150_DATA_Y_LSB_BIT);
			raw_data_xyz_t.raw_data_y = (s16)((((s32)
			((s8)v_data_u8[BMM150_YMSB_DATA])) <<
			BMM150_SHIFT_BIT_POSITION_BY_05_BITS)
			| v_data_u8[BMM150_YLSB_DATA]);


			/* Reading data for Z axis */
			v_data_u8[BMM150_ZLSB_DATA] =
			BMM150_GET_BITSLICE(v_data_u8[BMM150_ZLSB_DATA],
			BMM150_DATA_Z_LSB_BIT);
			raw_data_xyz_t.raw_data_z = (s16)((((s32)
			((s8)v_data_u8[BMM150_ZMSB_DATA])) <<
			BMM150_SHIFT_BIT_POSITION_BY_07_BITS)
			| v_data_u8[BMM150_ZLSB_DATA]);




				/* read the data ready status*/
			mag_data->data_ready = BMM150_GET_BITSLICE(
			v_data_u8[BMM150_RLSB_DATA],
			BMM150_DATA_RDYSTAT);






			/* Reading data for Resistance*/
			v_data_u8[BMM150_RLSB_DATA] =
			BMM150_GET_BITSLICE(v_data_u8[BMM150_RLSB_DATA],
			BMM150_DATA_R_LSB_BIT);
			raw_data_xyz_t.raw_data_r = (u16)((((u32)
			v_data_u8[BMM150_RMSB_DATA]) <<
			BMM150_SHIFT_BIT_POSITION_BY_06_BITS)
			| v_data_u8[BMM150_RLSB_DATA]);






			/* Compensation for X axis */
			mag_data->datax = BMM150_compensate_X(
			raw_data_xyz_t.raw_data_x,
			raw_data_xyz_t.raw_data_r);


			/* Compensation for Y axis */
			mag_data->datay = BMM150_compensate_Y(
			raw_data_xyz_t.raw_data_y,
			raw_data_xyz_t.raw_data_r);


			/* Compensation for Z axis */
			mag_data->dataz = BMM150_compensate_Z(
			raw_data_xyz_t.raw_data_z,
			raw_data_xyz_t.raw_data_r);


			/* Output raw resistance value */
			mag_data->resistance = raw_data_xyz_t.raw_data_r;
		}
#endif
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
	if (p_BMM150 == BMM150_NULL) {
		return  E_BMM150_NULL_PTR;
		} else {
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
