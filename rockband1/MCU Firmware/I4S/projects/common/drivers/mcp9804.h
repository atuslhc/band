
#ifndef __MCP9804_H
#define __MCP9804_H

#include "em_device.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

/** I2C device address for temperature sensor on DVK */

#define AMB_TEMP_ADDR 0x30
#define SKIN_TEMP_ADDR 0x32

#define MCP9804_DEV_ID 0x0054

/*******************************************************************************
 ********************************   ENUMS   ************************************
 ******************************************************************************/

/** Available registers in MCP9804 sensor device */
typedef enum
{
  tempsensRegRFU =       0,   /**< reserved for future use  (read-only) */
  tempsensRegCONFIG =     1,   /**< Configuration register */
  tempsensRegTUPPER = 2,   /**< Alert Temperature Uppe r-Boundary Trip register*/
  tempsensRegLOWER = 3,   /**< Alert Temperature Lowe r-Boundary Trip register */ 
  tempsensRegTCRIT =   4,    /**< Critical Temperature Trip register  */
  tempsensRegTEMP =   5,    /**< Temperature register  */
  tempsensRegMFTID =   6,    /**< Manufacturer ID register  */
  tempsensRegDEVID =   7,    /**< Device ID/Revision register  */
  tempsensRegRES =   8    /**< Resolution register  */
} TEMPSENS_Register_TypeDef;


/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/


/*******************************************************************************
 *****************************   PROTOTYPES   **********************************
 ******************************************************************************/

void MCP9804_Init(void);
int TEMPSENS_RegisterGet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t *val);
int TEMPSENS_RegisterSet(I2C_TypeDef *i2c,
                         uint8_t addr,
                         TEMPSENS_Register_TypeDef reg,
                         uint16_t val);
int MCP9804_TemperatureGet(I2C_TypeDef *i2c,
                            uint8_t addr,
                            uint8_t*  temp);

void MCP9804_DOWN(I2C_TypeDef *i2c, uint8_t addr);
void Start_Cap_Temp(void);
void TempReadAndStop(void);
void SKIN_TEMP_INIT(void);
void Pre_INIT_I2C1(void);


extern bool AMB_TMP_ONLINE;
extern bool SKIN_TMP_ONLINE;

extern uint8_t AMB_TMP[],SKIN_TMP[];


#ifdef __cplusplus
}
#endif

#endif /* __MCP9804_H */
