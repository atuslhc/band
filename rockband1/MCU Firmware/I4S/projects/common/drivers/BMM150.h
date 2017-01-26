
#ifndef __BMM150_H__
#define __BMM150_H__

/* singed integer type*/
typedef	int8_t s8;/**< used for signed 8bit */
typedef	int16_t s16;/**< used for signed 16bit */
typedef	int32_t s32;/**< used for signed 32bit */
typedef	int64_t s64;/**< used for signed 64bit */


typedef	uint8_t u8;/**< used for unsigned 8bit */
typedef	uint16_t u16;/**< used for unsigned 16bit */
typedef	uint32_t u32;/**< used for unsigned 32bit */
typedef	uint64_t u64;/**< used for unsigned 64bit */


/***************************************************************/
/**\name	BUS READ AND WRITE FUNCTION POINTERS        */
/***************************************************************/
/*!
	@brief Define the calling convention of YOUR bus communication routine.
	@note This includes types of parameters. This example shows the
	configuration for an SPI bus link.

    If your communication function looks like this:

    write_my_bus_xy(u8 device_addr, u8 register_addr,
    u8 * data, u8 length);

    The BMM150_WR_FUNC_PTR would equal:

    BMM150_WR_FUNC_PTR s8 (* bus_write)(u8,
    u8, u8 *, u8)

    Parameters can be mixed as needed refer to the
    refer BMM150_BUS_WRITE_FUNC  macro.


*/
#define BMM150_BUS_WR_RETURN_TYPE s8
#define BMM150_BUS_WR_PARAM_TYPES \
u8, u8, u8 *, u8
#define BMM150_BUS_WR_PARAM_ORDER \
(device_addr, register_addr, register_data, wr_len)
#define BMM150_BUS_WRITE_FUNC( \
device_addr, register_addr, register_data, wr_len) \
bus_write(device_addr, register_addr, register_data, wr_len)


/*!
	@brief  link macro between API function calls and bus read function
	@note The bus write function can change since this is a
	system dependant issue.

    @note If the bus_read parameter calling order is like: reg_addr,
    reg_data, wr_len it would be as it is here.

    @note If the parameters are differently ordered or your communication
    function like I2C need to know the device address,
    you can change this macro accordingly.


    @note BMM150_BUS_READ_FUNC(dev_addr, reg_addr, reg_data, wr_len)\
    bus_read(dev_addr, reg_addr, reg_data, wr_len)

    @note This macro lets all API functions call
	YOUR communication routine in a
    way that equals your definition in the
    refer BMM150_RD_FUNC_PTR definition.

    @note : this macro also includes the "MSB='1'
    for reading BMM150 addresses.

*/
#define BMM150_BUS_RD_RETURN_TYPE s8
#define BMM150_BUS_RD_PARAM_TYPES \
u8, u8, u8 *, u8
#define BMM150_BUS_RD_PARAM_ORDER (device_addr, register_addr, register_data)
#define BMM150_BUS_READ_FUNC(device_addr, register_addr, register_data, rd_len)\
bus_read(device_addr, register_addr, register_data, rd_len)
/***************************************************************/
/**\name	RETURN TYPE DEFINITIONS        */
/***************************************************************/
#define BMM150_DELAY_RETURN_TYPE void
#define BMM150_DELAY_FUNC(delay_in_msec) \
delay_func(delay_in_msec)
#define BMM150_RETURN_FUNCTION_TYPE        s8
/***************************************************************/
/**\name	I2C ADDRESS DEFINITIONS        */
/***************************************************************/
#define BMM150_I2C_ADDRESS                 (0x10)
/***************************************************************/
/**\name	REGISTER ADDRESS DEFINITION        */
/***************************************************************/
/********************************************/
/**\name	CHIP ID       */
/********************************************/
/* Fixed Data Registers */
#define BMM150_ID_Reg                     (0x40)
#define BMM150_ID				0x32
/********************************************/
/**\name	DATA REGISTERS       */
/********************************************/
/* Data Registers*/
#define BMM150_DATA_X_LSB                   (0x42)
#define BMM150_DATA_X_MSB                   (0x43)
#define BMM150_DATA_Y_LSB                   (0x44)
#define BMM150_DATA_Y_MSB                   (0x45)
#define BMM150_DATA_Z_LSB                   (0x46)
#define BMM150_DATA_Z_MSB                   (0x47)
#define BMM150_DATA_R_LSB                   (0x48)
#define BMM150_DATA_R_MSB                   (0x49)
/********************************************/
/**\name	REMAPPED DATA REGISTERS      */
/********************************************/
/* Data Registers for remapped axis(XandY)
 * this only applicable for BMX055 */
#define BMM150_REMAPPED_BMX055_DATA_Y_LSB      (0x42)
#define BMM150_REMAPPED_BMX055_DATA_Y_MSB      (0x43)
#define BMM150_REMAPPED_BMX055_DATA_X_LSB      (0x44)
#define BMM150_REMAPPED_BMX055_DATA_X_MSB      (0x45)
/********************************************/
/**\name	INTERRUPT STATUS      */
/********************************************/
/* Status Registers */
#define BMM150_INT_STAT_REG                    (0x4A)
/********************************************/
/**\name	POWER MODE DEFINITIONS      */
/********************************************/
/* Control Registers */
#define BMM150_POWER_CONTROL               (0x4B)
#define BMM150_CONTROL                     (0x4C)
#define BMM150_INT_CONTROL                 (0x4D)
#define BMM150_SENS_CONTROL                (0x4E)
#define BMM150_LOW_THRES                   (0x4F)
#define BMM150_HIGH_THRES                  (0x50)
/********************************************/
/**\name XY AND Z REPETITIONS DEFINITIONS  */
/********************************************/
#define BMM150_REP_XY                      (0x51)
#define BMM150_REP_Z                       (0x52)
/********************************************/
/**\name	TRIM REGISTERS      */
/********************************************/
/* Trim Extended Registers */
#define BMM150_DIG_X1                      (0x5D)
#define BMM150_DIG_Y1                      (0x5E)
#define BMM150_DIG_Z4_LSB                  (0x62)
#define BMM150_DIG_Z4_MSB                  (0x63)
#define BMM150_DIG_X2                      (0x64)
#define BMM150_DIG_Y2                      (0x65)
#define BMM150_DIG_Z2_LSB                  (0x68)
#define BMM150_DIG_Z2_MSB                  (0x69)
#define BMM150_DIG_Z1_LSB                  (0x6A)
#define BMM150_DIG_Z1_MSB                  (0x6B)
#define BMM150_DIG_XYZ1_LSB                (0x6C)
#define BMM150_DIG_XYZ1_MSB                (0x6D)
#define BMM150_DIG_Z3_LSB                  (0x6E)
#define BMM150_DIG_Z3_MSB                  (0x6F)
#define BMM150_DIG_XY2                     (0x70)
#define BMM150_DIG_XY1                     (0x71)


/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF DATA REGISTERS   */
/********************************************/
/* Data X LSB Register */
#define BMM150_DATA_X_LSB_BIT__POS        (3)
#define BMM150_DATA_X_LSB_BIT__LEN        (5)
#define BMM150_DATA_X_LSB_BIT__MSK        (0xF8)
#define BMM150_DATA_X_LSB_BIT__REG        (BMM150_DATA_X_LSB)


/* Data X SelfTest Register */
#define BMM150_DATA_X_LSB_TESTX__POS         (0)
#define BMM150_DATA_X_LSB_TESTX__LEN         (1)
#define BMM150_DATA_X_LSB_TESTX__MSK         (0x01)
#define BMM150_DATA_X_LSB_TESTX__REG         (BMM150_DATA_X_LSB)


/* Data Y LSB Register */
#define BMM150_DATA_Y_LSB_BIT__POS        (3)
#define BMM150_DATA_Y_LSB_BIT__LEN        (5)
#define BMM150_DATA_Y_LSB_BIT__MSK        (0xF8)
#define BMM150_DATA_Y_LSB_BIT__REG        (BMM150_DATA_Y_LSB)


/* Data Y SelfTest Register */
#define BMM150_DATA_Y_LSB_TESTY__POS         (0)
#define BMM150_DATA_Y_LSB_TESTY__LEN         (1)
#define BMM150_DATA_Y_LSB_TESTY__MSK         (0x01)
#define BMM150_DATA_Y_LSB_TESTY__REG         (BMM150_DATA_Y_LSB)


/* Data Z LSB Register */
#define BMM150_DATA_Z_LSB_BIT__POS        (1)
#define BMM150_DATA_Z_LSB_BIT__LEN        (7)
#define BMM150_DATA_Z_LSB_BIT__MSK        (0xFE)
#define BMM150_DATA_Z_LSB_BIT__REG        (BMM150_DATA_Z_LSB)


/* Data Z SelfTest Register */
#define BMM150_DATA_Z_LSB_TESTZ__POS         (0)
#define BMM150_DATA_Z_LSB_TESTZ__LEN         (1)
#define BMM150_DATA_Z_LSB_TESTZ__MSK         (0x01)
#define BMM150_DATA_Z_LSB_TESTZ__REG         (BMM150_DATA_Z_LSB)


/* Hall Resistance LSB Register */
#define BMM150_DATA_R_LSB_BIT__POS             (2)
#define BMM150_DATA_R_LSB_BIT__LEN             (6)
#define BMM150_DATA_R_LSB_BIT__MSK             (0xFC)
#define BMM150_DATA_R_LSB_BIT__REG             (BMM150_DATA_R_LSB)


#define BMM150_DATA_RDYSTAT__POS            (0)
#define BMM150_DATA_RDYSTAT__LEN            (1)
#define BMM150_DATA_RDYSTAT__MSK            (0x01)
#define BMM150_DATA_RDYSTAT__REG            (BMM150_DATA_R_LSB)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF REMAPPED DATA REGISTERS   */
/********************************************/
/* Data X LSB Remapped Register only applicable for BMX055 */
#define BMM150_REMAPPED_BMX055_DATA_X_LSB_BIT__POS        (3)
#define BMM150_REMAPPED_BMX055_DATA_X_LSB_BIT__LEN        (5)
#define BMM150_REMAPPED_BMX055_DATA_X_LSB_BIT__MSK        (0xF8)
#define BMM150_REMAPPED_BMX055_DATA_X_LSB_BIT__REG\
(BMM150_REMAPPED_BMX055_DATA_X_LSB)


/* Data Y LSB Remapped Register only applicable for BMX055  */
#define BMM150_REMAPPED_BMX055_DATA_Y_LSB_BIT__POS        (3)
#define BMM150_REMAPPED_BMX055_DATA_Y_LSB_BIT__LEN        (5)
#define BMM150_REMAPPED_BMX055_DATA_Y_LSB_BIT__MSK        (0xF8)
#define BMM150_REMAPPED_BMX055_DATA_Y_LSB_BIT__REG\
(BMM150_REMAPPED_BMX055_DATA_Y_LSB)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF INTERRUPT STATUS REGISTERS   */
/********************************************/
/* Interrupt Status Register */
#define BMM150_INT_STAT_DOR__POS            (7)
#define BMM150_INT_STAT_DOR__LEN            (1)
#define BMM150_INT_STAT_DOR__MSK            (0x80)
#define BMM150_INT_STAT_DOR__REG            (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_OVRFLOW__POS        (6)
#define BMM150_INT_STAT_OVRFLOW__LEN        (1)
#define BMM150_INT_STAT_OVRFLOW__MSK        (0x40)
#define BMM150_INT_STAT_OVRFLOW__REG        (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_HIGH_THZ__POS       (5)
#define BMM150_INT_STAT_HIGH_THZ__LEN       (1)
#define BMM150_INT_STAT_HIGH_THZ__MSK       (0x20)
#define BMM150_INT_STAT_HIGH_THZ__REG       (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_HIGH_THY__POS       (4)
#define BMM150_INT_STAT_HIGH_THY__LEN       (1)
#define BMM150_INT_STAT_HIGH_THY__MSK       (0x10)
#define BMM150_INT_STAT_HIGH_THY__REG       (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_HIGH_THX__POS       (3)
#define BMM150_INT_STAT_HIGH_THX__LEN       (1)
#define BMM150_INT_STAT_HIGH_THX__MSK       (0x08)
#define BMM150_INT_STAT_HIGH_THX__REG       (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_LOW_THZ__POS        (2)
#define BMM150_INT_STAT_LOW_THZ__LEN        (1)
#define BMM150_INT_STAT_LOW_THZ__MSK        (0x04)
#define BMM150_INT_STAT_LOW_THZ__REG        (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_LOW_THY__POS        (1)
#define BMM150_INT_STAT_LOW_THY__LEN        (1)
#define BMM150_INT_STAT_LOW_THY__MSK        (0x02)
#define BMM150_INT_STAT_LOW_THY__REG        (BMM150_INT_STAT_REG)


#define BMM150_INT_STAT_LOW_THX__POS        (0)
#define BMM150_INT_STAT_LOW_THX__LEN        (1)
#define BMM150_INT_STAT_LOW_THX__MSK        (0x01)
#define BMM150_INT_STAT_LOW_THX__REG        (BMM150_INT_STAT_REG)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF SOFT RESET REGISTERS   */
/********************************************/
/* Power Control Register */
#define BMM150_POWER_CONTROL_SOFT_RST_7__POS       (7)
#define BMM150_POWER_CONTROL_SOFT_RST_7__LEN       (1)
#define BMM150_POWER_CONTROL_SOFT_RST_7__MSK       (0x80)
#define BMM150_POWER_CONTROL_SOFT_RST_7__REG       (BMM150_POWER_CONTROL)


#define BMM150_POWER_CONTROL_SPI3_ENABLE__POS     (2)
#define BMM150_POWER_CONTROL_SPI3_ENABLE__LEN     (1)
#define BMM150_POWER_CONTROL_SPI3_ENABLE__MSK     (0x04)
#define BMM150_POWER_CONTROL_SPI3_ENABLE__REG     (BMM150_POWER_CONTROL)


#define BMM150_POWER_CONTROL_SOFT_RST_1__POS       (1)
#define BMM150_POWER_CONTROL_SOFT_RST_1__LEN       (1)
#define BMM150_POWER_CONTROL_SOFT_RST_1__MSK       (0x02)
#define BMM150_POWER_CONTROL_SOFT_RST_1__REG       (BMM150_POWER_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF POWER MODE REGISTERS   */
/********************************************/
#define BMM150_POWER_CONTROL_POWER_CONTROL_BIT__POS         (0)
#define BMM150_POWER_CONTROL_POWER_CONTROL_BIT__LEN         (1)
#define BMM150_POWER_CONTROL_POWER_CONTROL_BIT__MSK         (0x01)
#define BMM150_POWER_CONTROL_POWER_CONTROL_BIT__REG         \
(BMM150_POWER_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF SELF TEST REGISTERS   */
/********************************************/
/* Control Register */
#define BMM150_CONTROL_ADVANCED_SELFTEST__POS            (6)
#define BMM150_CONTROL_ADVANCED_SELFTEST__LEN            (2)
#define BMM150_CONTROL_ADVANCED_SELFTEST__MSK            (0xC0)
#define BMM150_CONTROL_ADVANCED_SELFTEST__REG            (BMM150_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF DATA RATE REGISTERS   */
/********************************************/
#define BMM150_CONTROL_DATA_RATE__POS                (3)
#define BMM150_CONTROL_DATA_RATE__LEN                (3)
#define BMM150_CONTROL_DATA_RATE__MSK                (0x38)
#define BMM150_CONTROL_DATA_RATE__REG                (BMM150_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF OPERATION MODE REGISTERS   */
/********************************************/
#define BMM150_CONTROL_OPERATION_MODE__POS            (1)
#define BMM150_CONTROL_OPERATION_MODE__LEN            (2)
#define BMM150_CONTROL_OPERATION_MODE__MSK            (0x06)
#define BMM150_CONTROL_OPERATION_MODE__REG            (BMM150_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF SELF TEST REGISTERS   */
/********************************************/
#define BMM150_CONTROL_SELFTEST__POS            (0)
#define BMM150_CONTROL_SELFTEST__LEN            (1)
#define BMM150_CONTROL_SELFTEST__MSK            (0x01)
#define BMM150_CONTROL_SELFTEST__REG            (BMM150_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF INTERRUPT CONTROL REGISTERS   */
/********************************************/
/* Interrupt Control Register */
#define BMM150_INT_CONTROL_DOR_EN__POS            (7)
#define BMM150_INT_CONTROL_DOR_EN__LEN            (1)
#define BMM150_INT_CONTROL_DOR_EN__MSK            (0x80)
#define BMM150_INT_CONTROL_DOR_EN__REG            (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_OVRFLOW_EN__POS        (6)
#define BMM150_INT_CONTROL_OVRFLOW_EN__LEN        (1)
#define BMM150_INT_CONTROL_OVRFLOW_EN__MSK        (0x40)
#define BMM150_INT_CONTROL_OVRFLOW_EN__REG        (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_HIGH_THZ_EN__POS       (5)
#define BMM150_INT_CONTROL_HIGH_THZ_EN__LEN       (1)
#define BMM150_INT_CONTROL_HIGH_THZ_EN__MSK       (0x20)
#define BMM150_INT_CONTROL_HIGH_THZ_EN__REG       (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_HIGH_THY_EN__POS       (4)
#define BMM150_INT_CONTROL_HIGH_THY_EN__LEN       (1)
#define BMM150_INT_CONTROL_HIGH_THY_EN__MSK       (0x10)
#define BMM150_INT_CONTROL_HIGH_THY_EN__REG       (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_HIGH_THX_EN__POS       (3)
#define BMM150_INT_CONTROL_HIGH_THX_EN__LEN       (1)
#define BMM150_INT_CONTROL_HIGH_THX_EN__MSK       (0x08)
#define BMM150_INT_CONTROL_HIGH_THX_EN__REG       (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_LOW_THZ_EN__POS        (2)
#define BMM150_INT_CONTROL_LOW_THZ_EN__LEN        (1)
#define BMM150_INT_CONTROL_LOW_THZ_EN__MSK        (0x04)
#define BMM150_INT_CONTROL_LOW_THZ_EN__REG        (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_LOW_THY_EN__POS        (1)
#define BMM150_INT_CONTROL_LOW_THY_EN__LEN        (1)
#define BMM150_INT_CONTROL_LOW_THY_EN__MSK        (0x02)
#define BMM150_INT_CONTROL_LOW_THY_EN__REG        (BMM150_INT_CONTROL)


#define BMM150_INT_CONTROL_LOW_THX_EN__POS        (0)
#define BMM150_INT_CONTROL_LOW_THX_EN__LEN        (1)
#define BMM150_INT_CONTROL_LOW_THX_EN__MSK        (0x01)
#define BMM150_INT_CONTROL_LOW_THX_EN__REG        (BMM150_INT_CONTROL)


/* Sensor Control Register */
#define BMM150_SENS_CONTROL_DRDY_EN__POS          (7)
#define BMM150_SENS_CONTROL_DRDY_EN__LEN          (1)
#define BMM150_SENS_CONTROL_DRDY_EN__MSK          (0x80)
#define BMM150_SENS_CONTROL_DRDY_EN__REG          (BMM150_SENS_CONTROL)


#define BMM150_SENS_CONTROL_IE__POS               (6)
#define BMM150_SENS_CONTROL_IE__LEN               (1)
#define BMM150_SENS_CONTROL_IE__MSK               (0x40)
#define BMM150_SENS_CONTROL_IE__REG               BMM150_SENS_CONTROL


#define BMM150_SENS_CONTROL_CHANNELZ__POS         (5)
#define BMM150_SENS_CONTROL_CHANNELZ__LEN         (1)
#define BMM150_SENS_CONTROL_CHANNELZ__MSK         (0x20)
#define BMM150_SENS_CONTROL_CHANNELZ__REG         (BMM150_SENS_CONTROL)


#define BMM150_SENS_CONTROL_CHANNELY__POS         (4)
#define BMM150_SENS_CONTROL_CHANNELY__LEN         (1)
#define BMM150_SENS_CONTROL_CHANNELY__MSK         (0x10)
#define BMM150_SENS_CONTROL_CHANNELY__REG         (BMM150_SENS_CONTROL)


#define BMM150_SENS_CONTROL_CHANNELX__POS         (3)
#define BMM150_SENS_CONTROL_CHANNELX__LEN         (1)
#define BMM150_SENS_CONTROL_CHANNELX__MSK         (0x08)
#define BMM150_SENS_CONTROL_CHANNELX__REG         (BMM150_SENS_CONTROL)


#define BMM150_SENS_CONTROL_DR_POLARITY__POS      (2)
#define BMM150_SENS_CONTROL_DR_POLARITY__LEN      (1)
#define BMM150_SENS_CONTROL_DR_POLARITY__MSK      (0x04)
#define BMM150_SENS_CONTROL_DR_POLARITY__REG      (BMM150_SENS_CONTROL)


#define BMM150_SENS_CONTROL_INTERRUPT_LATCH__POS    (1)
#define BMM150_SENS_CONTROL_INTERRUPT_LATCH__LEN    (1)
#define BMM150_SENS_CONTROL_INTERRUPT_LATCH__MSK    (0x02)
#define BMM150_SENS_CONTROL_INTERRUPT_LATCH__REG    (BMM150_SENS_CONTROL)


#define BMM150_SENS_CONTROL_INTERRUPT_POLARITY__POS         (0)
#define BMM150_SENS_CONTROL_INTERRUPT_POLARITY__LEN         (1)
#define BMM150_SENS_CONTROL_INTERRUPT_POLARITY__MSK         (0x01)
#define BMM150_SENS_CONTROL_INTERRUPT_POLARITY__REG         \
(BMM150_SENS_CONTROL)
/********************************************/
/**\name BIT MASK, LENGTH AND POSITION OF TRIM REGISTER   */
/********************************************/
/* Register 6D */
#define BMM150_DIG_XYZ1_MSB__POS         (0)
#define BMM150_DIG_XYZ1_MSB__LEN         (7)
#define BMM150_DIG_XYZ1_MSB__MSK         (0x7F)
#define BMM150_DIG_XYZ1_MSB__REG         (BMM150_DIG_XYZ1_MSB)
/*****************************************************************/
/********************************************/
/**\name CONSTANTS DEFINITIONS  */
/********************************************/
/********************************************/
/**\name ERROR CODE */
/********************************************/


/** Error code definitions**/
#define E_BMM150_NULL_PTR           ((s8)-127)
#define BMM150_ERROR				((u8)0)///((s8)-1)
#define E_BMM150_OUT_OF_RANGE       ((s8)-2)
#define BMM150_NULL                 ((u8)0)
#define E_BMM150_UNDEFINED_MODE     (0)


/********************************************/
/**\name RESET DEFINITIONS */
/********************************************/
/*General Info data's*/
#define BMM150_SOFT_RESET7_ON              (1)
#define BMM150_SOFT_RESET1_ON              (1)
#define BMM150_SOFT_RESET7_OFF             (0)
#define BMM150_SOFT_RESET1_OFF             (0)
#define BMM150_DELAY_SOFTRESET             (1)


/********************************************/
/**\name DELAY DEFINITIONS  */
/********************************************/
/* Constants */
#define BMM150_DELAY_POWEROFF_SUSPEND      (1)
#define BMM150_DELAY_SUSPEND_SLEEP         (3)
#define BMM150_DELAY_SLEEP_ACTIVE          (1)
#define BMM150_DELAY_ACTIVE_SLEEP          (1)
#define BMM150_DELAY_SLEEP_SUSPEND         (1)
#define BMM150_DELAY_ACTIVE_SUSPEND        (1)
#define BMM150_DELAY_SLEEP_POWEROFF        (1)
#define BMM150_DELAY_ACTIVE_POWEROFF       (1)
#define BMM150_DELAY_SETTLING_TIME         (3)
#define BMM150_MDELAY_DATA_TYPE			  u32
/********************************************/
/**\name XYZ AXIS DEFINITIONS  */
/********************************************/
#define BMM150_X_AXIS               (0)
#define BMM150_Y_AXIS               (1)
#define BMM150_Z_AXIS               (2)
#define BMM150_RESISTANCE           (3)
#define BMM150_X                    (1)
#define BMM150_Y                    (2)
#define BMM150_Z                    (4)
#define BMM150_XYZ                  (7)
/********************************************/
/**\name ENABLE/DISABLE DEFINITIONS  */
/********************************************/
#define BMM150_CHANNEL_DISABLE                  (1)
#define BMM150_CHANNEL_ENABLE                   (0)
#define BMM150_OFF                              (0)
#define BMM150_ON                               (1)
/********************************************/
/**\name POWER MODE DEFINITIONS  */
/********************************************/
#define BMM150_NORMAL_MODE                      (0)*0x02
#define BMM150_FORCED_MODE                      (1)*0x02
#define BMM150_SUSPEND_MODE                     (2)*0x02
#define BMM150_SLEEP_MODE                       (3)*0x02
/********************************************/
/**\name SELF TEST DEFINITIONS  */
/********************************************/
#define BMM150_ADVANCED_SELFTEST_OFF            (0)
#define BMM150_ADVANCED_SELFTEST_NEGATIVE       (2)
#define BMM150_ADVANCED_SELFTEST_POSITIVE       (3)


#define BMM150_NEGATIVE_SATURATION_Z            (-32767)
#define BMM150_POSITIVE_SATURATION_Z            (32767)
/********************************************/
/**\name SPI READ/WRITE DEFINITIONS  */
/********************************************/
#define BMM150_SPI_RD_MASK                      (0x80)
#define BMM150_READ_SET                         (0x01)
/********************************************/
/**\name READ AND WRITE FUNCTION POINTERS  */
/********************************************/
/* Bus read and bus write */
#define BMM150_WR_FUNC_PTR \
	s8 (*bus_write)(u8, u8, \
	u8 *, u8)


#define BMM150_RD_FUNC_PTR \
	s8 (*bus_read)(u8, u8, \
	u8 *, u8)


/********************************************/
/**\name NUMERIC DEFINITIONS  */
/********************************************/
#define  BMM150_GEN_READ_WRITE_DATA_LENGTH		((u8)1)
#define  BMM150_TRIM_DATA_LENGTH		((u8)2)
#define  BMM150_SELFTEST_DELAY			((u8)4)
#define  BMM150_SELFTEST_DATA_LENGTH	((u8)5)
#define  BMM150_ALL_DATA_FRAME_LENGTH	((u8)8)
/**< Frame length refers the
x,y,z and r values*/
#define BMM150_INIT_VALUE               (0)


/********************************************/
/**\name GET AND SET BITSLICE FUNCTIONS  */
/********************************************/
/* get bit slice  */
#define BMM150_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


/* Set bit slice */
#define BMM150_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))


/********************************************/
/**\name OVERFLOW DEFINITIONS  */
/********************************************/
/* compensated output value returned if sensor had overflow */
#define BMM150_OVERFLOW_OUTPUT			-32768
#define BMM150_OVERFLOW_OUTPUT_S32		((s32)(-2147483647-1))
#define BMM150_OVERFLOW_OUTPUT_FLOAT	0.0f
#define BMM150_FLIP_OVERFLOW_ADCVAL		-4096
#define BMM150_HALL_OVERFLOW_ADCVAL		-16384
/********************************************/
/**\name PRESET MODE DEFINITIONS  */
/********************************************/
#define BMM150_PRESETMODE_LOWPOWER                  (1)
#define BMM150_PRESETMODE_REGULAR                   (2)
#define BMM150_PRESETMODE_HIGHACCURACY              (3)
#define BMM150_PRESETMODE_ENHANCED                  (4)


/* PRESET MODES - DATA RATES */
#define BMM150_LOWPOWER_DR                       (BMM150_DR_10HZ)
#define BMM150_REGULAR_DR                        (BMM150_DR_10HZ)
#define BMM150_HIGHACCURACY_DR                   (BMM150_DR_20HZ)
#define BMM150_ENHANCED_DR                       (BMM150_DR_10HZ)


/* PRESET MODES - REPETITIONS-XY RATES */
#define BMM150_LOWPOWER_REPXY                     (1)
#define BMM150_REGULAR_REPXY                      (4)
#define BMM150_HIGHACCURACY_REPXY                (23)
#define BMM150_ENHANCED_REPXY                     (7)


/* PRESET MODES - REPETITIONS-Z RATES */
#define BMM150_LOWPOWER_REPZ                      (2)
#define BMM150_REGULAR_REPZ                      (14)
#define BMM150_HIGHACCURACY_REPZ                 (82)
#define BMM150_ENHANCED_REPZ                     (26)
/********************************************/
/**\name DATA RATE DEFINITIONS  */
/********************************************/
/* Data Rates */
#define BMM150_DR_10HZ                     (0)*0x08
#define BMM150_DR_02HZ                     (1)*0x08
#define BMM150_DR_06HZ                     (2)*0x08
#define BMM150_DR_08HZ                     (3)*0x08
#define BMM150_DR_15HZ                     (4)*0x08
#define BMM150_DR_20HZ                     (5)*0x08
#define BMM150_DR_25HZ                     (6)*0x08
#define BMM150_DR_30HZ                     (7)*0x08


#define BMM150_DATA_RATE_10HZ        (0x00)
#define BMM150_DATA_RATE_02HZ        (0x01)
#define BMM150_DATA_RATE_06HZ        (0x02)
#define BMM150_DATA_RATE_08HZ        (0x03)
#define BMM150_DATA_RATE_15HZ        (0x04)
#define BMM150_DATA_RATE_20HZ        (0x05)
#define BMM150_DATA_RATE_25HZ        (0x06)
#define BMM150_DATA_RATE_30HZ        (0x07)


/********************************************/
/**\name BIT SHIFTING DEFINITIONS  */
/********************************************/
/*Shifting Constants*/
#define BMM150_SHIFT_BIT_POSITION_BY_01_BIT     (1)
#define BMM150_SHIFT_BIT_POSITION_BY_02_BITS    (2)
#define BMM150_SHIFT_BIT_POSITION_BY_03_BITS    (3)
#define BMM150_SHIFT_BIT_POSITION_BY_05_BITS    (5)
#define BMM150_SHIFT_BIT_POSITION_BY_06_BITS    (6)
#define BMM150_SHIFT_BIT_POSITION_BY_07_BITS    (7)
#define BMM150_SHIFT_BIT_POSITION_BY_08_BITS    (8)
#define BMM150_SHIFT_BIT_POSITION_BY_09_BITS    (9)
#define BMM150_SHIFT_BIT_POSITION_BY_12_BITS    (12)
#define BMM150_SHIFT_BIT_POSITION_BY_13_BITS    (13)
#define BMM150_SHIFT_BIT_POSITION_BY_16_BITS    (16)
#define BMM150_SHIFT_BIT_POSITION_BY_14_BITS    (14)
#define BMM150_SHIFT_BIT_POSITION_BY_15_BITS    (15)
/****************************************************/
/**\name	ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define	BMM150_DATA_FRAME_SIZE		(8)
#define	BMM150_INIT_DATA_SIZE		(2)
#define	BMM150_CHIP_ID_DATA			(0)
#define	BMM150_SELFTEST_DATA_SIZE	(5)
#define	BMM150_TRIM_DATA_SIZE		(2)
#define	BMM150_XLSB_DATA			(0)
#define	BMM150_XMSB_DATA			(1)
#define	BMM150_YLSB_DATA			(2)
#define	BMM150_YMSB_DATA			(3)
#define	BMM150_ZLSB_DATA			(4)
#define	BMM150_ZMSB_DATA			(5)
#define	BMM150_RLSB_DATA			(6)
#define	BMM150_RMSB_DATA			(7)
#define	BMM150_TRIM_DIG_Z1_LSB_DATA	(0)
#define	BMM150_TRIM_DIG_Z1_MSB_DATA	(1)
#define	BMM150_TRIM_DIG_Z2_LSB_DATA	(0)
#define	BMM150_TRIM_DIG_Z2_MSB_DATA	(1)
#define	BMM150_TRIM_DIG_Z3_LSB_DATA	(0)
#define	BMM150_TRIM_DIG_Z3_MSB_DATA	(1)
#define	BMM150_TRIM_DIG_Z4_LSB_DATA	(0)
#define	BMM150_TRIM_DIG_Z4_MSB_DATA	(1)
#define	BMM150_TRIM_DIG_XYZ1_LSB_DATA	(0)
#define	BMM150_TRIM_DIG_XYZ1_MSB_DATA	(1)
#define	BMM150_REMAPPED_YLSB_DATA			(0)
#define	BMM150_REMAPPED_YMSB_DATA			(1)
#define	BMM150_REMAPPED_XLSB_DATA			(2)
#define	BMM150_REMAPPED_XMSB_DATA			(3)


/********************************************/
/**\name STRUCTURE DEFINITIONS  */
/********************************************/
/*!
 * @brief Structure containing mag xyz data
 * output of the data is s16
 */
struct BMM150_mag_data_s16_t {
	s16 datax;/**<mag compensated X  data*/
	s16 datay;/**<mag compensated Y  data*/
	s16 dataz;/**<mag compensated Z  data*/
	u16 resistance;/**<mag R  data*/
	u8 data_ready;/**<mag data ready status*/
};
/*!
 * @brief Structure containing mag xyz data
 * output of the data is s32
 */
struct BMM150_mag_s32_data_t {
	s32 datax;/**<mag compensated X  data*/
	s32 datay;/**<mag compensated Y  data*/
	s32 dataz;/**<mag compensated Z  data*/
	u16 resistance;/**<mag R  data*/
	u8 data_ready;/**<mag data ready status*/
};
/*!
 * @brief Structure containing mag xyz data
 * output of the data is float
 */
struct BMM150_mag_data_float_t {
	float datax;/**<mag compensated X  data*/
	float datay;/**<mag compensated Y  data*/
	float  dataz;/**<mag compensated Z  data*/
	u16 resistance;/**<mag R  data*/
	u8 data_ready;/**<mag data ready status*/
};
/*!
 * @brief Structure containing mag remapped xyz data
 * output of the data is s16
 * this only applicable for BMX055 sensor
 */
struct BMM150_remapped_mag_s16_data_t {
	s16 datax;/**<mag compensated remapped X  data*/
	s16 datay;/**<mag compensated remapped Y  data*/
	s16 dataz;/**<mag compensated Z  data*/
	u16 resistance;/**<mag R  data*/
	u8 data_ready;/**<mag data ready status*/
};
/*!
 * @brief Structure containing mag remapped xyz data
 * output of the data is s32
 * this only applicable for BMX055 sensor
 */
struct BMM150_remapped_mag_s32_data_t {
	s32 datax;/**<mag compensated remapped X  data*/
	s32 datay;/**<mag compensated remapped Y  data*/
	s32 dataz;/**<mag compensated Z  data*/
	u16 resistance;/**<mag R  data*/
	u8 data_ready;/**<mag data ready status*/
};
/*!
 * @brief Structure containing mag remapped xyz data
 * output of the data is float
 * this only applicable for BMX055 sensor
 */
struct BMM150_remapped_mag_data_float_t {
	float datax;/**<mag compensated remapped X  data*/
	float datay;/**<mag compensated remapped Y  data*/
	float  dataz;/**<mag compensated Z  data*/
	u16 resistance;/**<mag R  data*/
	u8 data_ready;/**<mag data ready status*/
};
/*!
 * @brief Structure containing mag initial parameters
 */
struct BMM150_t {
	u8 company_id;/**<mag chip id*/
	u8 dev_addr;/**<mag device address*/


	BMM150_WR_FUNC_PTR;/**< bus write function pointer*/
	BMM150_RD_FUNC_PTR;/**< bus read function pointer*/
	void (*delay_msec)(BMM150_MDELAY_DATA_TYPE);/**< delay function pointer*/


	s8 dig_x1;/**< trim x1 data */
	s8 dig_y1;/**< trim y1 data */


	s8 dig_x2;/**< trim x2 data */
	s8 dig_y2;/**< trim y2 data */


	u16 dig_z1;/**< trim z1 data */
	s16 dig_z2;/**< trim z2 data */
	s16 dig_z3;/**< trim z3 data */
	s16 dig_z4;/**< trim z4 data */


	u8 dig_xy1;/**< trim xy1 data */
	s8 dig_xy2;/**< trim xy2 data */


	u16 dig_xyz1;/**< trim xyz1 data */
};
/********************************************/
/**\name FUNCTION DECLARATIONS  */
/********************************************/
#define BMM150_SPI           (USART1)
#define BMM150_CS_PORT    (gpioPortD)
#define BMM150_CS_PIN    (10)

#define BMM150_CS_H()    GPIO_PinOutSet(BMM150_CS_PORT,BMM150_CS_PIN )
#define BMM150_CS_L()    GPIO_PinOutClear(BMM150_CS_PORT,BMM150_CS_PIN )

#define BMM150_INT_PORT    (gpioPortA)
#define BMM150_DRDY_PIN    (12)

#define BMM150_INT_PORT    (gpioPortA)
#define BMM150_INT_PIN    (13)

/********************************************/
/**\name INITIALIZATION  */
/********************************************/
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
BMM150_RETURN_FUNCTION_TYPE BMM150_Init(void);
/********************************************/
/**\name DATA READ FUNCTIONS  */
/********************************************/
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
BMM150_RETURN_FUNCTION_TYPE BMM150_read_mag_data_XYZ(void);

/********************************************/
/**\name COMMON READ AND WRITE FUNCTIONS  */
/********************************************/
/*!
 * @brief
 *	This API reads the data from
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMM150_RETURN_FUNCTION_TYPE BMM150_read_register(u8 v_addr_u8,
u8 *v_data_u8);
/*!
 * @brief
 *	This API write the data to
 *	the given register
 *
 *
 *	@param v_addr_u8 -> Address of the register
 *	@param v_data_u8 -> The data from the register
 *	@param v_len_u8 -> no of bytes to read
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
BMM150_RETURN_FUNCTION_TYPE BMM150_write_register(u8 v_addr_u8,
u8 v_data_u8);
/********************************************/
/**\name DATA RATE FUNCTIONS  */
/********************************************/
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
BMM150_RETURN_FUNCTION_TYPE BMM150_set_data_rate(u8 v_data_rate_u8);
/********************************************/
/**\name FUNCTIONAL STATE FUNCTION  */
/********************************************/
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
u8 v_functional_state_u8);
/********************************************/
/**\name POWER MODE   */
/********************************************/
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
BMM150_RETURN_FUNCTION_TYPE BMM150_get_power_mode(u8 *v_power_mode_u8);
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
BMM150_RETURN_FUNCTION_TYPE BMM150_set_power_mode(u8 v_power_mode_u8);
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
u8 v_rep_xy_u8);
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
u8 v_rep_z_u8);
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
BMM150_RETURN_FUNCTION_TYPE BMM150_set_presetmode(u8 v_presetmode_u8);


#endif

