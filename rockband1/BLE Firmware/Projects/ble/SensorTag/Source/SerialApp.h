#ifndef _SERIAL_APP_H_
#define _SERIAL_APP_H_

#ifdef __cplusplus
extern "C"
{
#endif
  


#if  (HAL_UART_DMA == 2)
#define SBP_UART_PORT                  HAL_UART_PORT_1
#else
#define SBP_UART_PORT                  HAL_UART_PORT_0
#endif

//#define SBP_UART_FC                    TRUE
#define SBP_UART_FC                    FALSE
#define SBP_UART_FC_THRESHOLD          48
#define SBP_UART_RX_BUF_SIZE           128
#define SBP_UART_TX_BUF_SIZE           128
#define SBP_UART_IDLE_TIMEOUT          6
#define SBP_UART_INT_ENABLE            FALSE//TRUE
#define SBP_UART_BR                    HAL_UART_BR_115200 


// Serial Port Related
extern void SerialApp_Init(void);
extern void sbpSerialAppCallback(uint8 port, uint8 event);
void serialAppInitTransport( void );
void sbpSerialAppWrite(uint8 *pBuffer, uint16 length);
void Serial_PrintString(uint8 * pBuffer);
void Serial_PrintHex(uint8 ucData);
#ifdef _DEBUG_PRINT//_DEBUG_PRINT
#define DEBUG_PRINT(x,y) Serial_PrintString(x)
#define DEBUG_PRINT_HEX(x) Serial_PrintHex(x)
#else

#define DEBUG_PRINT(x,y)
#define DEBUG_PRINT_HEX(x)
#endif
#ifdef __cplusplus
}
#endif

#endif
