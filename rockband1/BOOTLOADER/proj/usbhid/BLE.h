#ifndef _ble_H
#define _ble_H


#define BLE_FRAME_SIZE     128  //The MCU to BLE upgrade data frame size.
#define RX_BUF_SIZE        64 //UART接收数据BUF
#define TX_BUF_SIZE        64
#define ESC                0x33

//这是数据的内容，即开始、长度、命令类型、命令数据。 命令的格式< length,cmd,content >
#define UART_DATA_START			0x3c//'<'
#define UART_DATA_STOP			0x3e//'>'
#define UART_ID_START			0x00
#define UART_ID_LEN			    0x01
#define UART_ID_CMD			    0x02
#define UART_ID_DATA			0x03

#define UART_CMD_UPDATA                 0x02 //The upgrade command
#define UART_CMD_UPGRADE_DATA           0x03 //The upgrade data(image)

#define BLE_UART_Closing 0
#define BLE_UART_Opening 1


union _BLE_CHIP{
  struct _BLE_DEVICE{
  	uint8_t  MCUTYPE;  //  0x41//我读到的0x8D
  	uint8_t  MEMCAP;   // 0x08 = 256K//我读到的0x4c
  	uint8_t  FW_VER1; // 1
    uint8_t  FW_VER2; // 0//我读到的2
    uint32_t unique0;  // MAC address , the first 4bytes  //我读到的AF2B230D
    uint16_t unique1;  // the second 2 bytes , 6 bytes in total //我读到的0x9059          
	uint8_t  WORKSTA; // 0= ble bootload ,1 = ble app ////我读到的0x01
	uint8_t  Rev;////我读到的0x00
  }BLE_Device;
  uint8_t BLE_DeviceInfo[12];
};

extern uint8_t bleInfo[12];
extern uint8_t bleStatusFlag;
extern volatile bool  BLE_ONLINE;
//extern uint8_t BLE_STATE;
extern union _BLE_CHIP BLE_DevChip;
extern char CopyRxBuff[64];//在流程判断中会用到它里面的数据。
extern	char tempESC[RX_BUF_SIZE];
extern	char escCount;//转移字符计数

extern volatile bool BleLeUartSta;
extern volatile bool BLE_Responsed;
extern volatile bool crccheck;

void BLE_INIT(void);
void LeuartConfig (void);
void BLE_Update_Start(void);
void BLE_Update_End(uint16_t checksum);
void WriteCC254xFlash(uint8_t *p);
void AppToBoot(void);
void BLE_Update_End(uint16_t checksum);
void LEUARTSentByDma(uint8_t comm_type,uint8_t *p ,uint8_t len);
void MyLEUARTSentByDma(uint8_t comm_type,uint8_t *p ,uint8_t len);
void MyBLE_Update_Start(void);


#if 0
//2015年10月5日09:21:14 调整代码去掉的

#define BLE_ADVEN_1S  10
#define BLE_ADVEN_2S  20

#define BLE_ON  1
#define BLE_OFF 0

#define BLE_STATE_IDLE              0x00//空闲
#define BLE_STATE_ADVERTISING       0x01//这是广播模式？
#define BLE_STATE_CONNECTED         0x02//连接


//这里表明可以发送的命令，首先是要INFOR命令查看相关信息
#define UART_CMD_2HOST  		0x00 //
#define UART_CMD_INFOR			0x01 //器件信息类型
#define UART_CMD_DATA			0x04
#define UART_CMD_ACK                    0x05
#define UART_CMD_TEST                   0x06




#define UART_DATA_COM_LEN		(20+1)
#define UART_DATA_IRTEMP		0x01
#define UART_DATA_ACCEL			0x02
#define UART_DATA_HEARTRATE		0x03
#define UART_DATA_BATT			0x04
#define UART_DATA_GYRO			0x05

#define UART_TYPE_ADVER			0x00//这是广播类型
#define UART_TYPE_DEV			0x01//这是器件信息？
#define UART_TYPE_INFOR			0x02//这是器件状态信息。
#define UART_TYPE_COM			0x03//查询下行的数据


#define UART_LEN_FOU			0x04

#define UART_LEN_DEV			(UART_LEN_FOU+11)
#define Dev_FW_V1				0x00
#define Dev_FW_V2				0x0e

#define BOOTLOAD				0x00
#define BLE_APP					0x01

#define UART_LEN_STARTED		 (UART_LEN_FOU+2)
#define UART_INFOR_IDLE          0x00
#define UART_INFOR_ADVERTISING   0x01
#define UART_INFOR_CONNECTED     0x02


//使能或停止广播
#define ADVER_EN            0x01
#define ADVER_DIS           0x00

#define crcCheck_Page         123

#define STATE_CMD_INFOR		0x00
#define STATE_COM_DATA		0x01


#define BLE_CH1 1
#define BLE_CH2 2
#define BLE_CH3 3
#define BLE_CH4 4
#define BLE_CH5 5


// BLE HOST Command

#define getRealData  0x01
#define FindMe       0x02
#define ClockSynch   0x03
#define AlertSynch   0x04
#define UserProfileSynch   0x05
#define PhoneComing  0x06	// 作废
#define notifyFeature	0x10	// 短信、电话等通知：接收sms/calling通知，vibra/ring

//======================
#define FW_Update_COMM 0x14
#define FW_CRC_GET 0x15
#define FW_Update_DATA 0x16


//======================

#define ResetDevice  0xf0
#define RunOutBatQuick  0xf1
//下面关于延时的定义
#define LeUartUsingDelayTimer      (1 << 3)

void BLE_ADVEN_CON(uint8_t onoff, uint8_t inteval);
void SendRealDataOverBLE();
void SendXYZ2Dongle(void);
void SendPPGECG(void);

void ParseBleUartPak(void);
void BLE_Close(void);
void BLE_Open(void);
void Send_1HZ_PacketOverBLE(void);
void getBleDeviceInfo(void);


#endif

#endif


