#ifndef _BIM_MAIN_H
#define _BIM_MAIN_H

#define UART_DATA_START			0x3c//'<'
#define UART_DATA_STOP			0x3e//'>'

#define UART_ID_START			0x00+0x06
#define UART_ID_LEN				0x01+0x06
#define UART_ID_CMD				0x02+0x06
#define UART_ID_DATA			0x03+0x06

#define UART_CMD_INFOR			0x01
#define UART_CMD_UPDATA                 0x02
#define UART_CMD_DATA			0x03



#endif

