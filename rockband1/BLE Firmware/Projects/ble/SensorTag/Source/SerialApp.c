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

/*�ú����������������ĳ�ʼ�������е���*/
void SerialApp_Init( void )
{
	//����uart��ʼ������
	serialAppInitTransport();
}

/*uart��ʼ�����룬���ô��ڵĲ����ʡ������Ƶ�*/
void serialAppInitTransport( void )
{
	halUARTCfg_t uartConfig;

	// configure UART
	uartConfig.configured           = TRUE;
	uartConfig.baudRate             = SBP_UART_BR;//������
	uartConfig.flowControl          = SBP_UART_FC;//������
	uartConfig.flowControlThreshold = SBP_UART_FC_THRESHOLD;//��������ֵ��������flowControlʱ����������Ч
	uartConfig.rx.maxBufSize        = SBP_UART_RX_BUF_SIZE;//uart���ջ�������С
	uartConfig.tx.maxBufSize        = SBP_UART_TX_BUF_SIZE;//uart���ͻ�������С
	uartConfig.idleTimeout          = SBP_UART_IDLE_TIMEOUT;
	uartConfig.intEnable            = SBP_UART_INT_ENABLE;//�Ƿ����ж�
	uartConfig.callBackFunc         = sbpSerialAppCallback;//uart���ջص��������ڸú����ж�ȡ����uart����

	// start UART
	// Note: Assumes no issue opening UART port.
	(void)HalUARTOpen( SBP_UART_PORT, &uartConfig );

	return;
}

/*uart���ջص�����*/
/*-----------------------------------------------------------------------------
* ��������: sbpSerialAppCallback()
* �������: uint8 port, uint8 event
* �������: void
*
* ��������:  ���Ǵ��ڲ�ѯ�ص�������
*
* ��    ��:  ��־��
* ��������: 2014/6/19 ������
*-----------------------------------------------------------------------------*/
void sbpSerialAppCallback(uint8 port, uint8 event)
{
	uint8 dataBuf[SBP_UART_RX_BUF_SIZE];//���ֻ�������ݴ洮������
	uint16 numBytes;
	// unused input parameter; PC-Lint error 715.
	(void)event;

// HalLcdWriteString("Data form my UART:", HAL_LCD_LINE_4 );
	//���ؿɶ����ֽ�
	if ( (numBytes = Hal_UART_RxBufLen(port)) > 0 )
	{
		//��ȡȫ����Ч�����ݣ��������һ��һ����ȡ���Խ����ض�������
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