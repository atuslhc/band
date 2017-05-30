#ifndef _SENSORTAGUSER_H
#define _SENSORTAGUSER_H

#include "CC2540_bitdef.h"
#if defined(MODEL_TYPE)
#if (MODEL_TYPE==1)	//HEALTHCARE_TYPE
 #if !defined(VENDOR_TYPE)
 #define VENDOR_TYPE             2
 #endif
#if (VENDOR_TYPE==1)
#define Dev_FW_V1				1
#define Dev_FW_V2				87
#elif (VENDOR_TYPE==2)
#define Dev_FW_V1				0
#define Dev_FW_V2				19
//EXT_MAX_TX_POWER=3(4_DBM), 0(MINUS_23_DBM)
//RSSI_DIFF=0
#else
#error "Please specify VENDOR_TYPE!"
#endif
#elif (MODEL_TYPE==2)	//CONSUMER_TYPE
 #if !defined(VENDOR_TYPE)
 #define VENDOR_TYPE             2
 #endif
#if (VENDOR_TYPE==1)
#define Dev_FW_V1				1
#define Dev_FW_V2				80
#elif (VENDOR_TYPE==2)
#define Dev_FW_V1				0
#define Dev_FW_V2				10
#else
#error "Please specify VENDOR_TYPE!"
#endif
#else	//not specify the MODEL_TYPE
#error "Without specify MODEL_TYPE !!!"
#endif
#endif
extern uint8 irTempDataWrite[20];
extern uint8 irTempLenWrite;

extern uint8 gyroTempDataWrite[20];
extern uint8 gyroTempLenWrite;
extern uint8 SystemBuyDelay;
#define SystemBuyDelaySleep		10
#define DELAYSLEEP_INIT                 200

#define UART_DATA_START		0x3c	//'<'
#define UART_DATA_STOP		0x3e	//'>'



/*MCU->BLE*/
//串口通信偏移位置
#define UART_DATA_EXTRA_LEN             5 // for noti data, full msg minus START(<),LEN,CMD,CH,...STOP(>)
#define UART_ID_START_POS		0x00
#define UART_ID_LEN_POS			0x01
#define UART_ID_CMD_POS			0x02
#define UART_ID_DATA_POS		0x03   //这里是buffer位置

//ID = UART_CMD_DATA 数据。
#define UART_ID_DATA_CHANNEL            0X03
#define UART_ID_DATA_NOTI               0X04


#define UART_CMD_DATA			0x00    //as MCU UART_CMD_2HOST
#define UART_CMD_INFOR			0x01	//ble driver to mcu
#define UART_CMD_UPDATA                 0x02  //self upgrade cmd.(MCU bootloader send BLE upgrade to BLE.
#define UART_CMD_UPGRADE_DATA           0x03  //upgrade data(image).
#define UART_CMD_APPDATA                0x04  //是从APP端发到watch端BLE的数据。

#define UART_CMD_ALERT                  0x05 //这里在增加一个类型表示MCU端来的警报 2015年7月8日18:13:35

#define UART_CMD_REQUEST_INFO           0X07
#define UART_CMD_WRITE_ATT              0x08
#define UART_CMD_ANCS                   0X09

#define WRITE_ATT_FIND_ME               0x01

#define UART_DATA_COM_LEN		(20)
#define UART_DATA_IRTEMP		0x01
#define UART_DATA_ACCEL			0x02
#define UART_DATA_HEARTRATE		0x03
#define UART_DATA_BATT			0x04
#define UART_DATA_GYRO			0x05

#define UART_TYPE_ADVER			0x00
#define UART_TYPE_DEV			0x01
#define UART_TYPE_INFOR			0x02	//ble status information.
#define UART_TYPE_COM			0x03
#define UART_TYPE_CONN			0x04    //ble connect interval
#define UART_TYPE_ANCS          0x05
#define UART_TYPE_BROADCAST     0x06
#if 0
//配合I4B 检测外部32M晶振
#define UART_TYPE_SEND_DEV              0x07
#endif
#define UART_LEN_FOU			0x04
#define UART_CMD_ACK			0x00

#define UART_LEN_DEV			(UART_LEN_FOU+12)


/*BLE->MCU*/
#define UART_B2M_ID_CMD         0x02
#define UART_B2M_ID_DATA        0x03

#define UART_B2M_ID_STAT_CMD	0x03
#define UART_BWM_ID_STAT_DATA	0x04
//连接信息
#define UART_B2M_CMD_CONN	0X02
#define UART_B2M_CMD_CONN_IDEL	0X00
#define UART_B2M_CMD_CONN_ADVE	0X01
#define UART_B2M_CMD_CONN_CONN	0X02
#define UART_B2M_CMD_CONN_UCONN	0X03
//Notification 状态
#define UART_B2M_CMD_NOTI	0X03
#define UART_B2M_CMD_NOTI_HR	0X01

#define UART_B2M_CMD_NOTI_DIABLE 0X00
#define UART_B2M_CMD_NOTI_ENABLE 0X01

#define BOOTLOAD		0x00
#define BLE_APP			0x01

//#define UART_LEN_STARTED	 (UART_LEN_FOU+2)	//2: DATA_START(0x3c), DATA_STOP(0x3e)
#define UART_LEN_STARTED	 (UART_LEN_FOU+3)	//3: DATA_START(0x3c), DATA_STOP(0x3e), gapProfileState
#define UART_INFOR_IDLE          0x00
#define UART_INFOR_ADVERTISING   0x01
#define UART_INFOR_CONNECTED     0x02

#define BLE_STATE_IDLE              0x00
#define BLE_STATE_ADVERTISING       0x01
#define BLE_STATE_CONNECTED         0x02

#define ADVER_EN            0x01
#define ADVER_DIS           0x00

#define crcCheck_Page         123
#define ANCS_ENABLE_PAGE      122	//0x7A

#define STATE_CMD_INFOR		0x00
#define STATE_COM_DATA		0x01

#define ANCS_ENABLE_FLAG        0x55
#define ANCS_DISABLE_FLAG        0xaa
//Device Info
#define DEVICE_INFO_MODEL_1 "I4"
#define DEVICE_INFO_MODEL_2 "I4S"
#define DEVICE_INFO_MODEL_3 "I2"
#define DEVICE_INFO_MODEL_4 "I2S"
#define DEVICE_INFO_MODEL_5 "I3"

extern uint8 BLE_State;
void devDataInit(void);
void ReadMessage(void);
void UART_Analyze(void);
void SPI_SendInfo(void);
void SPI_SendApply(void);
void SPI_Send(uint8 len, uint8* data);
void Delay_uS(uint16 microSecs);

//#define IMBUSY P0 &= ~BIT5
//#define IMFREE P0 |= BIT5
extern uint16 delaySleep;
void rxDataRead(void);
void txDtatSend(uint8* data, uint8 len);
void rxDataSave(uint8* data, uint8 start);
void getRxDataRead(uint8* data, uint8 len);
void startEvt_UartRX(void);
void stopEvt_UartRX(void);
void setCONN_INTERVAL(uint8 conn_interval);

#endif /* _SENSORTAGUSER_H */
