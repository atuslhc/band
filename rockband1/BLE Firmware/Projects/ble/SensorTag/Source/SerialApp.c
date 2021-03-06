#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "hal_uart.h"


#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
//#include "central.h"
#include "gapbondmgr.h"
#include "SensorTagUser.h"
//#include "simpleBLECentral.h"
#include "SerialApp.h"
#include "SensorTagUser.h"

extern uint8 sbpGattWriteString(uint8* pBuffer, uint16 length);

/*该函数将会在任务函数的初始化函数中调用*/
void SerialApp_Init( void )
{
	//调用uart初始化代码
	serialAppInitTransport();
}

/*uart初始化代码，配置串口的波特率、流控制等*/
void serialAppInitTransport( void )
{
	halUARTCfg_t uartConfig;

	// configure UART
	uartConfig.configured           = TRUE;
	uartConfig.baudRate             = SBP_UART_BR;
	uartConfig.flowControl          = SBP_UART_FC; //Flow Control
	uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD; //flow control threshold. it is validate while FC enable.
	uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE; //uart RX buffer size
	uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE; //uart TX buffer size.
	uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
	uartConfig.intEnable            = SBP_UART_INT_ENABLE; //enable the inerrupt service.
	uartConfig.callBackFunc         = sbpSerialAppCallback; //uart receive callback, read uart buffer in the function.

	// start UART
	// Note: Assumes no issue opening UART port.
	(void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

	return;
}

/*uart接收回调函数*/
/*-----------------------------------------------------------------------------
* 函数名称: sbpSerialAppCallback()
* 输入参数: uint8 port, uint8 event
* 输出参数: void
*
* 功能描述:  这是串口查询回调函数。
*
* 作    者:  何志辉
* 创建日期: 2014/6/19 星期四
*-----------------------------------------------------------------------------*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
	uint8 dataBuf[SBP_UART_RX_BUF_SIZE];//这个只是用来暂存串口数据
	uint16 numBytes;
	// unused input parameter; PC-Lint error 715.
	(void)event;

// HalLcdWriteString("Data form my UART:", HAL_LCD_LINE_4 );
	//返回可读的字节
	if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 )
	{
		//读取全部有效的数据，这里可以一个一个读取，以解析特定的命令
		(void)HalUARTRead (port, dataBuf, numBytes);
		//delaySleep = DELAYSLEEP_INIT;
//    rxDataRead(dataBuf,numBytes);
		getRxDataRead(dataBuf, numBytes);
	}
}
void sbpSerialAppWrite(uint8* pBuffer, uint16 length)
{
	HalUARTWrite (SBP_UART_PORT, pBuffer, length);
}

void Serial_PrintString(uint8* pBuffer)
{
	uint8 i = 0, *pData = pBuffer;

	while(*pData != '\0' && i < 100)
	{
		pData++;
		i++;

	}

	sbpSerialAppWrite(pBuffer, i);
}
void Serial_PrintHex(uint8 ucData)
{
	uint8 ucTemp[2];
	ucTemp[1] = ucData & 0x0f;
	ucTemp[0] = (ucData >> 4) & 0x0f;

	if(ucTemp[0] <= 9)ucTemp[0] += '0';
	else ucTemp[0] = ucTemp[0] + 'A' - 10;

	if(ucTemp[1] <= 9)ucTemp[1] += '0';
	else ucTemp[1] = ucTemp[1] + 'A' - 10;

	sbpSerialAppWrite(ucTemp, 2);
}
