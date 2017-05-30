#include "hal_uart.h"
#include "OSAL.h"
#include "npi.h"

/*******************************************************************************
 * MACROS
 */

#define MAX_PKT_SIZE    128
//#define RX_BUFF_SIZE    500
#define RX_BUFF_SIZE    128

#define UART_DATA_START		0x3c	//'<'
#define UART_DATA_STOP		0x3e	//'>'

#define UART_DATA_EXTRA_LEN     5 //START(<),LEN,CMD,CH,XX,..,00,00,STOP(>)
#define UART_LEN_FOU            4   //UART transfer head START(<),LEN,CMD,<params>,STOP(>)

#define UART_ID_START_POS		0x00
#define UART_ID_LEN_POS			0x01
#define UART_ID_CMD_POS			0x02
#define UART_ID_DATA_POS		0x03

//ID = UART_CMD_DATA
#define UART_ID_DATA_CHANNEL    0X03
#define UART_ID_DATA_NOTI       0X04

#define UART_CMD_DATA           0x00    //as MCU UART_CMD_2HOST
#define UART_CMD_INFOR          0x01	//ble driver to mcu
#define UART_CMD_UPDATA         0x02
#define UART_CMD_APPDATA        0x04

#define UART_CMD_ALERT          0x05

#define UART_CMD_REQUEST_INFO   0X07
#define UART_CMD_WRITE_ATT      0x08
#define UART_CMD_ANCS           0X09

#define DEVICE_INFO_CHANNEL     0X09

#define UART_TYPE_ADVER			0x00
#define UART_TYPE_DEV			0x01
#define UART_TYPE_INFOR			0x02	//ble status information.
#define UART_TYPE_COM			0x03
#define UART_TYPE_CONN			0x04
#define UART_TYPE_ANCS          0x05
#define UART_TYPE_BROADCAST     0x06
#if (P180F_PATCH==1)
#define UART_TYPE_BATTERY       0x07
#endif

#define ANCS_ENABLE_FLAG        0x55
#define ANCS_DISABLE_FLAG       0xaa

#define CRC_CHECK_PAGE          123 //The page number should be match your bootloader definition.
#define ANCS_ENABLE_PAGE        122

#define ADVER_ENABLE            0x01
#define ADVER_DISABLE           0x00

#define TGAP_LIM_DISC_ADV_INT_MIN      6  //!< Minimum advertising interval, when in limited discoverable mode (n * 0.625 mSec)
#define TGAP_LIM_DISC_ADV_INT_MAX      7  //!< Maximum advertising interval, when in limited discoverable mode (n * 0.625 mSec)
#define TGAP_GEN_DISC_ADV_INT_MIN      8  //!< Minimum advertising interval, when in General discoverable mode (n * 0.625 mSec)
#define TGAP_GEN_DISC_ADV_INT_MAX      9  //!< Maximum advertising interval, when in General discoverable mode (n * 0.625 mSec)

#define STATE_CMD_INFOR		    0x00
#define STATE_COM_DATA		    0x01

#define BLE_STATE_IDLE          0x00
#define BLE_STATE_ADVERTISING   0x01
#define BLE_STATE_CONNECTED     0x02

#define BLE_BOOTLOAD		    0x00
#define BLE_APPLICATION			0x01

#define UART_DATA_COM_LEN       20

/* define channel map */
#define HEART_RATE_CHANNEL      0x03

#define TEMPERATURE_CHANNEL     0X06
#define RUNNING_CHANNEL         0X07
#define READ_ATTRIBUTE_CHANNEL  0X08
#define DEVICE_INFO_CHANNEL     0X09
#define HEART_ALERT_CHANNEL     0x0A

/* define notification state */
#define UART_B2M_CMD_NOTI	0X03
#define UART_B2M_CMD_NOTI_HR	0X01

#define UART_B2M_CMD_NOTI_DIABLE 0X00
#define UART_B2M_CMD_NOTI_ENABLE 0X01

#define sendDataBufSiz         6
#define sendDataId              0
#define sendDataLen             1
#define sendDataData            2

//===================================================


/* States for CRC parser */
typedef enum {
  SERIAL_STATE_START = 0,
  SERIAL_STATE_TYPE,
  SERIAL_STATE_LEN,
  SERIAL_STATE_DATA,
  SERIAL_STATE_COMPLETE //received complete serial message
} serial_state_t;

typedef struct {
  bool heartRate;
  bool temperature;
  bool running;
  bool heartRateAlert;
}stNotiFlag_Type;

void cSerialPacketParser( uint8 port, uint8 events );
void parseCmd(void);
void sendSerialEvt(void);

#if (UART_RXOW_CHECK==1)
typedef enum {
  SERIAL_RX_NOBUF = 0, //bit0
  SERIAL_RX_FRAG,
  SERIAL_RX_OW,
  SERIAL_TX_NOBUF,
  SERIAL_TX_FRAG,
  SERIAL_TX_OW,
  SERIAL_TX_DATAERR,
}serial_err_t;
extern uint8 uart_errs;
#endif

//global

extern uint8 BLE_State;
extern uint16 serialBufferOffset;
extern uint8 serialBuffer[RX_BUFF_SIZE];
extern uint8 MessageType;
extern uint8 BLEBufferSlave[MAX_PKT_SIZE >> 1];
extern uint16 BLE_DELAY;
extern uint8 send_flage;
extern uint8 sendDataBuf[sendDataBufSiz][24];
/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void SerialInterface_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events );

extern uint8 sendAckMessage(uint8 bytes_sent);

extern uint8 sendDataToHost(uint8* data, uint8 len);  
  
extern uint16 circular_add(uint16 x, uint16 y);

extern uint16 circular_diff(uint16 offset, uint16 tail);

extern void myAlertCB(uint8 event); //[AlertNotification]

extern void txEncodeSend(uint8* data, uint8 len);

extern void rxDtatSave(uint8* data, uint8 len, uint8 id);