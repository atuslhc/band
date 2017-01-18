/*
 * memlcd.h
 *
 *  Created on: May 3, 2012
 *      Author: d.bruyn
 */

#ifndef MEMLCD_H_
#define MEMLCD_H_

#include "em_gpio.h"
#include "em_cmu.h"

typedef uint32_t   EMSTATUS;


#define MEMLCD_SIZE_Y   168
#define MEMLCD_SIZE_X   144


#define MEMLCD_CMD_UPDATE                 0x01
#define MEMLCD_CMD_ALL_CLEAR              0x04


#define MEMLCD_OK 0x00000000


typedef struct _MEMLCD_Pin
{
  GPIO_Port_TypeDef port;
  unsigned int      pin;
} MEMLCD_Pin;

typedef enum _MEMLCD_Mode {
  memlcdMode_Extcom,
  memlcdMode_Serial
} MEMLCD_Mode;



typedef struct _RowLimits
{
  uint8_t min;
  uint8_t max;
} RowLimits;


void initLETIMER(void);

void MEMLCD_Init(void);
void MEMLCD_Enable( bool enable );
//void MEMLCD_ComInv( void );
void MEMLCD_Clear( void );
EMSTATUS MEMLCD_Update(uint16_t *data, int firstLine, int lastLine);


void MEMLCD_CreateTestFrameBufferDMA(void);
EMSTATUS MEMLCD_Update_DMA( uint8_t firstline, uint8_t lastline );

#endif /* MEMLCD_H_ */
