#include "freertos.h"
#include "task.h"
//#include "cmsis_os.h"
#include "display_task.h"

#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "em_int.h"
#include "em_timer.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "subMenu.h"
//#include "led_font.h"

#include "common_vars.h"
#include "sys_sharing_source.h"
#include "main.h"
#include "ble.h"

#include "dma.h"
#include "crc.h"

#include "device_task.h"
#include "flash_task.h"

#include "menu.h"
#include "notification.h"

#define DMA_CH_RX    1 //DMAͨ��0
#define DMA_CH_TX    2 //DMAͨ��1
DMA_CB_TypeDef TX_DMA_CALLBACK ;
//DMA_CB_TypeDef RX_DMA_CALLBACK ;


#define RX_BUF_SIZE        64 //UART��������BUF
#define TX_BUF_SIZE        128 //  128/20=6 ble pak
#define BLE_PAK_SIZE       32

#define LOOPSIZE 8

#define LeUartTxInterval (100) //2*50=100ms,  delaytime unit is 20ms ,decide how many 0x00 header data to active CC254x
#define LeUartRxAllowWaitTime (400/LeUartTxInterval) // delaytime unit is 50ms , 8*50=400


//extern osMessageQId hMsgInterrupt;



extern uint8_t AMB_uA;
extern uint16_t SkinTouchVal, touchValues;
//extern int16_t read_AB_reg, read_IR_reg;

//volatile bool systemStatus.blBleOnline=false,
volatile bool BLE_APP_Model = true;
uint8_t BLE_STATE = BLE_STATE_IDLE;
uint8_t Last_BLE_STATE = BLE_STATE_IDLE;

union _BLE_CHIP BLE_DevChip;
uint8_t RealDataOnPhone = 0;

bool En_HR_Read;

uint8_t insert_zero_number = 14;

uint8_t BLE_Should_Status;


uint8_t LEUARTRXBUFF[LOOPSIZE][RX_BUF_SIZE] __attribute__ ((aligned(4)));
uint8_t LeUartTxBuff[TX_BUF_SIZE] __attribute__ ((aligned(4))); //128+8

uint8_t WpIndex = 0, RpIndex = 0;

uint8_t Rec_Ble_Pak[BLE_PAK_SIZE];


uint8_t getConnectTime = 0;

//uint8_t lastNotifyId = 0;
//int notifyBufferPoint = 0;


uint16_t LeUartWorkTimeCount = 0, LeUartTxCount = 0; //
uint16_t BLE_BUSY_COUNT = 0;


volatile bool TxDone = true;

volatile bool BleLeUartSta = BLE_UART_Closing;
#if 1 //HEALTHCARE_TYPE	//set to 0 in I4_C because it used with other variables.
// The filed just a test case temporary, not function we used. remark in I4_C type.
typedef struct _RESEND_NOTIFICATION
{
	uint8_t watchWearStatusResend;
	uint8_t sosCounterStatusResend;
	uint8_t fallCounterStatusresend;
	uint8_t heartrateLevelResend;
	uint8_t chargeStatusResend;
	uint8_t lowBatteryStatusResend;
} RESEND_NOTIFICATION;

RESEND_NOTIFICATION resendTimes = {0,0,0,0,0,0}; //Atus: It is better initialize in code.
#endif

// ANCS��صĶ���
typedef union
{
	uint8_t ucData8[4];
	uint32_t uiData32;
} _UINION32_8_Type;

typedef enum
{
	PAIRING_IDL = 0,
	PAIRING_ENABLE,
	PAIRING_DISABLE
} pairing_Type;

typedef struct
{
	_UINION32_8_Type uuid;
	uint32_t         flag; //bit 1 title ���=1�� bit 4 message ��ɡ�
	uint32_t      crtEvent;
	uint16_t      msgLen;
	pairing_Type crtPairState;//��ǰ��״̬��
	pairing_Type setPairState; //Ҫ���õ�PAIR״̬
} s_Ancs_Type;

s_Ancs_Type  g_sAncs;

//���ڸ���sector index���������ݶ������һЩ����
SECTOR_NODE* sectorNode = NULL;
SECTOR_NODE* tempSectorNode = NULL;
int existIndexCounter = 0;

void SendData2Host(uint8_t* p, uint8_t len);
void LEUARTSentByDma(uint8_t comm_type, uint8_t* p , uint8_t len);
void Rdy2DmaRx(void);
void DisableLeUart(void);
void EnableLeUart(void);
void WaitLastTxDone(void);
void DoNeed_Change_CONNECT_INTERVAL(void);
void Change_BLE_CONNECT_INTERVAL(uint8_t val);
void ANCS_Parsing(unsigned char* p);
//void SendHeartAlert(void); //I4_C remark it, and not found body.
uint16_t GetTotalChunks(uint16_t currentScanSector, uint16_t currentUploadSectors);

#define FW_RX_BUFF_SIZE 152//128   �������Ĵ�С��152��ÿ��BLE���а�����������firmware������19byte,��ô��Ҫ����8�β����ⲿflash��д��

BYTE Data_Upload_Pak_Size = 80; //80  // 4 packages(20bytes) in 20mS

uint8_t FW_Update_sta = 0;
uint8_t FW_RX_BUFF[FW_RX_BUFF_SIZE];
uint8_t FW_RX_BUFF_WP = 0, FW_RX_BUFF_RP = 0;
int WR_SEC_LOC, WR_SEC_OFFSET, Left_SEC_DATA;
int WR_SEC_COUNT;
int searchStartIndex = 0, searchEndIndex = 0;
uint8_t  WR_SEC_STA;

extern uint8_t ucMenuRamBuf[4][192];

// -----------------------------------------------------------------
#define TIME_OUT_UPLOAD_DATA 	40
#define TIME_OUT_UPGRADE_FW 	30

#define BLE_TASK_TYPE_DATA_UPLOADING	1
#define BLE_TASK_TYPE_FW_UPGRADING		2

//bool blDataUploadingFlag = false;		// �����ϴ���־
//bool blFwUpgradingFlag = false;			// �̼����±�־
bool blTurnedOffPpgByBleTask = false;	// �������ϴ����̼�����ʱ���Ƿ���ʱ�ر���ppg

//uint8_t bleTaskTimeoutCounter = 0;		// �����ϴ����̼����� ��ʱ������


void onBleTaskTimeOut(void* data)
{
	int type = (int) data;

	if (type == BLE_TASK_TYPE_DATA_UPLOADING)
	{
		// data uploading

		// ͣ���Զ��������ݵĶ�ʱ��
		DisableDelayTimer(TIMER_FLAG_BLE);
	}
	else if (type == BLE_TASK_TYPE_FW_UPGRADING)
	{
		// fw upgrading
		if(blTurnedOffPpgByBleTask == true)
		{
			blTurnedOffPpgByBleTask = false;
//			systemSetting.blHRSensorEnabled = true;
			systemStatus.blHRSensorTempEnabled = true;
		}
	}
}


// -----------------------------------------------------------------
BYTE dataUploadMode = 0;

bool blDataDumpMode = false;
//uint8_t  BLE_FW_Update_Sta=0;

uint16_t Upload_Sector_Num = 0; // ��¼�ϴ�sector�����
uint32_t Upload_Sector_index = 0; // ��¼�ϴ�sector�Ĵ洢��ţ����ڼ�����ݵ��¾ɣ�
uint16_t startUploadSectorNum = 0;
uint16_t endUploadSectorNum = 0;
uint16_t totalUploadSector = 0;

uint16_t Upload_Count = 0, Upload_Sector_Crc, Upload_Left = 0;
int Upload_physical_add;

uint16_t EXT_FLASH_SECTOR_OFFSET;

uint16_t CRC_TEMP;

void EFM2BLE_SetANCS(bool enable);


// ɨ����յ������ݣ�ȷ���ַ���ֻ����ascii�ַ�
// ɨ�跽����
//     ���� �ַ� > 127 �ľ�����ascii�ַ�
//     �Ƚ���ascii�ַ��滻Ϊ �ո�0x20
//     Ȼ��ȥ��ͷβ�Ŀո�
void cleanUpAsciiString(char* buffer, int size)
{
	BYTE d = 0;
	int sp = -1;	// ��һ���� 0x20 ��ascii�ַ���λ��

	for (int i = 0; i < size; i++)
	{
		d = (uint8_t) buffer[i];

		if (d == 0)
			break;
		else if (d >= 0x80)
			buffer[i] = 0x20;
		else if (d != 0x20 && sp < 0)
			sp = i;
	}

	// �ҵ���һ���� 0x20 ���ַ�
	if (sp < 0)
	{
		// û�ҵ�
		memset(buffer, 0, size);
	}
	else if (sp > 0)
	{
		memcpy(buffer, buffer + sp, size - sp);
	}
}


void cleanUpUTF8String(char* buffer, int size)
{
	// ɨ����յ������ݣ�ȷ��utf8�ַ���������ȷ
	// ɨ�跽����
	// �ҵ���һ�� 0x00��Ȼ������ɨ��
	//7 	U+0000 	U+007F 	1 	0xxxxxxx
	//11 	U+0080 	U+07FF 	2 	110xxxxx 	10xxxxxx
	//16 	U+0800 	U+FFFF 	3 	1110xxxx 	10xxxxxx 	10xxxxxx
	//21 	U+10000 	U+1FFFFF 	4 	11110xxx 	10xxxxxx 	10xxxxxx 	10xxxxxx
	//26 	U+200000 	U+3FFFFFF 	5 	111110xx 	10xxxxxx 	10xxxxxx 	10xxxxxx 	10xxxxxx
	//31 	U+4000000 	U+7FFFFFFF 	6 	1111110x 	10xxxxxx 	10xxxxxx 	10xxxxxx 	10xxxxxx 	10xxxxxx

	if (buffer == NULL || size <= 0)
		return;

	int end = 0;
	BYTE d = 0;

	for (; end < size; end++)
	{
		d = buffer[end];

		if (d == 0)
			break;
	}

	d = buffer[end - 1];

	if (d <= 0x7F) // ���һ���ַ�Ϊ ascii�ַ���������
		return;

	for (int i = 1; i < 4; i++) // utf8���4���ֽ� http://en.wikipedia.org/wiki/UTF-8
	{
		end--;
		d = buffer[end];

		if ((d & 0xC0) == 0x80) // 10xxxxxx
			continue;
		else
		{
			int len = 2;

			if ((d & 0xE0) == 0xC0) // 110xxxxx��2bytes
				;
			else if ((d & 0xF0) == 0xE0) // 11110xxx, 3 bytes
				len = 3;
			else if ((d & 0xF8) == 0xF0) // 11110xxx, 4 bytes
				len = 4;

			// ����ַ����ݲ�ȫ�����ַ������滻Ϊ 0x20 �ո�
			if (i < len)
			{
				memset(buffer + end, ' ', i);
			}

			break;
		}
	}
}


void CHECK_BLE_STUFF(void)
{
	static uint8_t here_couter = 0;


	//===========================
	DoNeed_Change_CONNECT_INTERVAL();
	isBLEError();

	if(systemSetting.SystemMode == SYSTEM_MODE_MANUFACTORING)
		Send_1HZ_PacketOverBLE();
	else
	{
		here_couter++;
#ifndef  PPG2Dongle  // for phone

		if(here_couter >= CHECK_INTERVAL_AT_BLE)
#endif
		{
			here_couter = 0;
			Send_1HZ_PacketOverBLE();
		}
	}
}

uint8_t Current_Connect_Interval = BLE_CONNECT_300ms;
static uint8_t  Check_Connect_Counter = 0;
static uint8_t  Check_ADV_Counter = 0;

void DoNeed_Change_CONNECT_INTERVAL(void)
{
	char speed_up = 0;


	if(BLE_STATE == BLE_STATE_ADVERTISING)
	{
#if 1

		if(Check_ADV_Counter)
		{
			Check_ADV_Counter--;

			if(Check_ADV_Counter == 0)
				BLE_ADVEN_CON(BLE_ON, BLE_ADVEN_2S);
		}

#endif
	}
	else if(BLE_STATE == BLE_STATE_CONNECTED)
	{
		INT_Disable();

		if(BLE_BUSY_COUNT > 10) speed_up = 1;

		BLE_BUSY_COUNT = 0;
		INT_Enable();

		if(speed_up)
		{
			Check_Connect_Counter++;

			if(Check_Connect_Counter > 10)
			{

				Check_Connect_Counter = 0;
				Current_Connect_Interval = BLE_CONNECT_20ms;

				if(Current_Connect_Interval != getConnectTime)
				{
					BLE_SET_CONNECT_INTERVAL(BLE_CONNECT_20ms);
				}
			}
		}
		else
		{
			Check_Connect_Counter++;

			if(Check_Connect_Counter > BLE_Connected_SleepTime)
			{
				Check_Connect_Counter = 0;
				Current_Connect_Interval = BLE_CONNECT_1000ms;

				if(Current_Connect_Interval != getConnectTime)
				{
					BLE_SET_CONNECT_INTERVAL(BLE_CONNECT_1000ms);

					if(FW_Update_sta)
						FW_Update_sta = 0; // if here, means  fw update failed
				}
			}
		}
	}
}

//���Ӽ������2�������¼�֮���ʱ�䣬�����¼���ָ��ʹ��û������ͨѶ������£�cc2540
//Ҳ�ᷢ����ϡ���ݸ����������Ҵ���������һЩ���ݣ�����������֮������ӡ�
void Change_BLE_CONNECT_INTERVAL(uint8_t val)
{
	Check_Connect_Counter = 0;
	Current_Connect_Interval = val;
	BLE_SET_CONNECT_INTERVAL(val);
}

__STATIC_INLINE void ReChargeTimeCount(void)
{
	INT_Disable();
	LeUartWorkTimeCount = LeUartRxAllowWaitTime;
	BLE_BUSY_COUNT++;
	INT_Enable();
}

void LeuartConfig (void)
{
	LEUART_Init_TypeDef tLeuartInit =
	{
		.enable   = leuartEnable,
		.refFreq  = 0,
		.baudrate = 115200,
		.databits = leuartDatabits8,
		.parity   = leuartNoParity,
		.stopbits = leuartStopbits1
	};

// CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);//���ں�ʱ�ӵ�һ��

	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LEUART0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	LEUART_Reset(LEUART0);
	LEUART_Init(LEUART0, &tLeuartInit);


	LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
	                 LEUART_ROUTE_RXPEN |
	                 LEUART_ROUTE_LOCATION_LOC4;

	GPIO_PinModeSet(BLE_TX_PORT, BLE_TX_PIN, gpioModePushPull,  1);
	GPIO_PinModeSet(BLE_RX_PORT, BLE_RX_PIN, gpioModeInputPull, 1);

	LEUART0->STARTFRAME = UART_DATA_START;
	LEUART0->SIGFRAME = UART_DATA_STOP;

	//LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF|LEUART_IEN_STARTF);
	LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF); // only the stop char int

	NVIC_SetPriority(LEUART0_IRQn, LEUART0_INT_LEVEL);

	NVIC_EnableIRQ(LEUART0_IRQn);
}

/**************************************************************************//**
 * @brief  Setup Low Energy UART with DMA operation
 *
 * The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
 * is initialized. Then DMA starts to wait for and receive data, and the LEUART1
 * is set up to generate an interrupt when it receives the defined signal frame.
 * The signal frame is set to '\r', which is the "carriage return" symbol.
 *
 *****************************************************************************/
/*----------------------------------------------------------------------------
 * DMA1����˵Ļص�����
-----------------------------------------------------------------------------*/
void BleTxDMADone(unsigned int channel, bool primary, void* user)
{
	TxDone = true;
	LeUartTxCount = LeUartTxInterval;
}

/*------------------------------------------------------------------------------
 *LEuart MDA��ʼ��
 *�õ�����Դ��DMA0->RX DMA1->TX USART0
--------------------------------------------------------------------------------*/
void SetupLeuartDma(void)
{

	DMA_Init_TypeDef dmaInit;

	/* Setting up channel */
	DMA_CfgChannel_TypeDef chnl0Cfg =
	{
		.highPri   = false,                     /* Normal priority */
		.enableInt = false,                     /* No interupt enabled for callback functions */
		.select    = DMAREQ_LEUART0_RXDATAV,    /* Set LEUART1 RX data avalible as source of DMA signals */
		.cb        = NULL,                      /* No callback funtion */
	};


	/* Setting up channel descriptor */
	DMA_CfgDescr_TypeDef descr0Cfg =
	{
		.dstInc  = dmaDataInc1,       /* Increment destination address by one byte */
		.srcInc  = dmaDataIncNone,    /* Do no increment source address  */
		.size    = dmaDataSize1,      /* Data size is one byte */
		.arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved*/
		.hprot   = 0,                 /* No read/write source protection */
	};

	/* Setting up channel */
	DMA_CfgChannel_TypeDef chnl1Cfg =
	{
		.highPri   = false,                     /* Normal priority */
		.enableInt = true,                     /* No interupt enabled for callback functions */
		.select    = DMAREQ_LEUART0_TXBL,    /* Set LEUART1 RX data avalible as source of DMA signals */
		.cb        = &TX_DMA_CALLBACK,                      /* No callback funtion */
	};


	/* Setting up channel descriptor */
	DMA_CfgDescr_TypeDef descr1Cfg =
	{
		.dstInc  = dmaDataIncNone,       /* Increment destination address by one byte */
		.srcInc  = dmaDataInc1,    /* Do no increment source address  */
		.size    = dmaDataSize1,      /* Data size is one byte */
		.arbRate = dmaArbitrate1,     /* Rearbitrate for each byte recieved*/
		.hprot   = 0,                 /* No read/write source protection */
	};

	CMU_ClockEnable(cmuClock_DMA, true);        /* Enable DMA clock */

	dmaInit.hprot = 0;
	dmaInit.controlBlock = dmaControlBlock;
	DMA_Init(&dmaInit);

	DMA_CfgChannel(DMA_CH_RX, &chnl0Cfg);
	DMA_CfgDescr(DMA_CH_RX, true, &descr0Cfg);
	DMA_CfgChannel(DMA_CH_TX, &chnl1Cfg);
	DMA_CfgDescr(DMA_CH_TX, true, &descr1Cfg);

	TX_DMA_CALLBACK.cbFunc  = BleTxDMADone;
	TX_DMA_CALLBACK.userPtr = NULL;

	Rdy2DmaRx();


}


void Rdy2DmaRx(void)
{
	/* Starting the transfer. Using Basic Mode */
	DMA_ActivateBasic(DMA_CH_RX,                /* Activate channel selected */
	                  true,                       /* Use primary descriptor */
	                  false,                      /* No DMA burst */
	                  (void*) &LEUARTRXBUFF[WpIndex][0],             /* Destination address */
	                  (void*) &LEUART0->RXDATA,   /* Source address*/
	                  RX_BUF_SIZE - 1);               /* Size of buffer minus1 */
}


void LEUART0_IRQHandler(void)
{
	uint32_t GucLeuartIF;
	GucLeuartIF = LEUART_IntGet(LEUART0);     /* �õ��жϱ�־λ             */
	LEUART_IntClear(LEUART0, GucLeuartIF);  /* ����жϱ�־λ               */

	ReChargeTimeCount();
	WpIndex++;
	WpIndex %= LOOPSIZE;
	//Atus: Should clean the buffer before write to avoid last remain data.

	Rdy2DmaRx();

//	WpIndex++;	//Atus: move from above to here, should increment after write.
//	WpIndex %= LOOPSIZE;

//	osMessagePut(hMsgInterrupt, BLE_RX_MSG, 0);

	MESSAGE  msg;
	msg.params.type = BLE_RX_MSG;

	xQueueSendFromISR(hEvtQueueDevice, &msg, 0);

}


// 100ms at most to wait
void WaitLastTxDone(void)
{
	while(TxDone == false)
	{
		EMU_EnterEM1();//DMA Not Done
		INT_Disable();

		if(LeUartWorkTimeCount == 0)
		{
			INT_Enable();
			break;
		}

		INT_Enable();
	};

}

void LEUARTSentByDma(uint8_t comm_type, uint8_t* p , uint8_t len)
{
	if(BleLeUartSta == BLE_UART_Closing)
	{
		EnableLeUart();
	}

	WaitLastTxDone();

	//if(BLE_APP_Model==true)
	{
		//if(LeUartTxCount==0)insert_zero_number=16;// make sure CC254x to wake up
		if(LeUartTxCount == 0)
			insert_zero_number = 18; // 2014.08.15
		else
			insert_zero_number = 6;
	}
	//else  insert_zero_number=6;

	memset(LeUartTxBuff, 0, insert_zero_number);
//	for(int i=0; i<insert_zero_number; i++)
//		LeUartTxBuff[i]=0;

	LeUartTxBuff[insert_zero_number + 0] = UART_DATA_START;
	LeUartTxBuff[insert_zero_number + 1] = len + 3 + 2 + 1; //3: prefix from '<',len+6,comm. 2:tail 0x00,0x00, 1:'>'
	LeUartTxBuff[insert_zero_number + 2] = comm_type;
	memcpy(&LeUartTxBuff[insert_zero_number + 3], p, len);

	LeUartTxBuff[insert_zero_number + 3 + len + 0] = 0;
	LeUartTxBuff[insert_zero_number + 3 + len + 1] = 0;
	LeUartTxBuff[insert_zero_number + 3 + len + 2] = UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone = false;
	DMA_ActivateBasic(DMA_CH_TX,            /*DMAͨ��1������       */
	                  true,
	                  false,
	                  (void*)(uint32_t) & LEUART0->TXDATA,
	                  (void*)(uint32_t) LeUartTxBuff,
	                  len - 1 + insert_zero_number + 2 + 4); // 2= 2 0x00 at end ,  4 = < len ,type ,>
}


void LEUARTCallback(void* data)
{
	if(LeUartTxCount)
		LeUartTxCount = 0;

	if(LeUartWorkTimeCount) // 10*20ms =200ms
	{
		LeUartWorkTimeCount--;

		if(LeUartWorkTimeCount == 0) // uart port is idle
		{
			DisableLeUart();
		}
	}
}

EFM32_PACK_START( 1 )

union _FW_INFO
{
	struct _INFO
	{
		uint16_t fw_type;
		uint16_t ver_num;
		uint32_t fw_length;
		uint16_t fw_crc;
		uint32_t Rev1;
		uint16_t Head_CRC;
	} INFO;
	uint8_t INFO_BUF[16];
};
union _FW_INFO FW_INFO;
EFM32_PACK_END()

//
#define ACTION_GET	(0)
#define ACTION_SET	(1)

#define ACTION_PASSWORD_SIZE	2
static const uint8_t ACTION_PASSWORD[ACTION_PASSWORD_SIZE] = { 0x20, 0x13 }; //[BG025] char >> uint8_t

__STATIC_INLINE bool checkActionPassword(const uint8_t* const pInput) //[BG025] char >> uint8_t
{
	for (int i = 0; i < ACTION_PASSWORD_SIZE; i++)
	{
		if (*(pInput + i) != ACTION_PASSWORD[i])
		{
			return false;
		}
	}

	return true;
}

//
float allSector = 1.0; //[BG033] 0.0 >> 1.0 to avoid divide by 0 condition.
//bool firstCall = true;
//time_t oldTime = 0;
static bool first = true;
#if 0
//���������һ������������Ŀ���������ȥ����2015��10��6��08:06:32
static time_t buffer[4] = {0};
#endif

uint16_t currentChunkNo = 0;

void ParseHostData(uint8_t* pr)//�������������������Ȼ�������Ӧ��ͨ��BLE_CH2������������ȥ��
{
//	static char set_done = 0;

	MESSAGE msg;
	uint8_t u8buff[21]; //Response command buffer.
	int sector_Start_num, sector_start_addr; 
	//int sector_end_num, sector_len; //[BG025] remark sector_end_num.sector_len;
	int i;
	static bool once = false;

	uint8_t comm = pr[0];
#if BGXXX==8
	test1.typeuint8[0] = comm;
#endif
	
	SetFlashUsingTime(30); //Atus: Why? Assume it will use flash, set flash counter.


#ifdef DEBUG_MODE
	//========================
	//����ble������������
//	USER_EVENT* bleEvent = (USER_EVENT*) osMailCAlloc(hDispEventQueue, 0);
//	bleEvent->type = EVT_TYPE_BLUETOOTH;
//	bleEvent->data.v = comm;
//	osMailPut(hDispEventQueue, bleEvent);


	USER_EVENT bleEvent;
	bleEvent.type = EVT_TYPE_BLUETOOTH;	//Atus: the relative MENU_TYPE_Ble_Testing remark now.
	bleEvent.data.v = comm;

	xQueueSend(hEvtQueueDisplay, &bleEvent, 0);
#endif

	/* For debug can blink for every call, set at the beginning. */
	startFlashLed(true, 1, 4, 1, 1, 4, 1);

	// Ԥ�Ȳ���׼���÷��ͻ������������ݰ����׸��ֽ��ǲ��õ�ͨ���������������BLE����˭�������ڶ����ֽ��������
	memset(u8buff, 0x00, sizeof(u8buff)); //clean buffer.
	u8buff[0] = BLE_CH2;
	u8buff[1] = comm;
	/* process ble command */
	switch(comm)
	{
		case get_FactoryTime:
		{
			u8buff[2] = (uint8_t)systemSetting.FactoryTime;
			u8buff[3] = (uint8_t)(systemSetting.FactoryTime >> 8);
			u8buff[4] = (uint8_t)(systemSetting.FactoryTime >> 16);
			u8buff[5] = (uint8_t)(systemSetting.FactoryTime >> 24);

			SendData2Host(u8buff, 6);  //[BG031] 10 >> 6
			break;
		}

#if 0

		//�豸�ͺ�д�ɹ̶��ģ����ﲻ��Ҫ�ˡ�
		case DeviceModel:
		{
			u8buff[2] = pr[1];

			if(pr[1] == 0) // get
			{
				u8buff[3] = systemSetting.bDeviceModel;
			}
			else if (pr[1] == 1)	// set
			{
				systemSetting.bDeviceModel = pr[2];
				SaveSystemSettings();

				//
				u8buff[3] = 0;
			}

			//
			SendData2Host(u8buff, 4);

			break;
		}

#endif

		/*********  Flash data upload   *********************************/
		case  Flash_Upload_COMM:
		{
			if (systemStatus.bBatteryLevel == OUT_OF_BATTERY)
			{
				memset(u8buff + 2, 0xff, 15);
				u8buff[8] = 0xf0;
			}
			else
			{
				vTaskDelay(20);
				Change_BLE_CONNECT_INTERVAL(BLE_CONNECT_20ms);//�������Ӽ���Ѿ�����̵�
				vTaskDelay(20);

				SetFlashBusyBit(FLASH_BUSY_BIT_BLE);  //�����flash����æ������ֹͣ����flash�Ĳ���
				WakeFlashUp();

				// parameters   ���ʱ������ϴ�ͬ������ʱ��ʱ���������ͬ���Ը�ʱ���Ϊ������һֱ�����е�ʱ�����Ҫô�ҵ��հ׵�sector��Ҫô�ҵ������ʱ�����С��ʱ���Ϊֹ��
				time_t searchTimestamp = (uint32_t)(pr[4] << 24) +
				                         (uint32_t)(pr[3] << 16) +
				                         (uint32_t)(pr[2] << 8) +
				                         (uint32_t)(pr[1]);

				dataUploadMode = pr[5];

				if (dataUploadMode == 1)
					Data_Upload_Pak_Size = 20;
				else
					Data_Upload_Pak_Size = 80;

				blDataDumpMode = pr[6];

				//
				uint16_t totalSector = 0;

				if (blDataDumpMode)
				{
					// dump ģʽ��ֱ�Ӵ���ʼ������ʼ����
					Upload_Sector_Num = INDEX_DATA_START_SECTOR;
					totalSector = INDEX_DATA_END_SECTOR - INDEX_DATA_START_SECTOR + 1; //��totalsector��ʾ�����õģ�������ʵ��������ռ�õ�sector��
				}
				else
				{
					Upload_Sector_Num = SearchSectorForTimestamp(&searchTimestamp);//���ݸ�ʱ�����ȷ��sector
#if BGXXX==6
					test1.typeuint16[0] = Upload_Sector_Num;
#endif
				}

				Upload_Sector_index = 0; // reset Upload_Sector_index for SEC_Upload_START requestSector=0xFFFF can identify first sector read.
//������һ���ϴ����ݵ���ʼ�ط��������ϴ������Ҫ���㿪ʼ��

				scanWholeFlash = true;//ÿ���ϴ�����ʱ����������λ��
				first = true;

				u8buff[2] = (uint8_t)Upload_Sector_Num;  //�����ֵ��FF��FFʱ��APP�Ͳ�ͬ�������ˡ�
				u8buff[3] = (uint8_t)(Upload_Sector_Num >> 8);

				u8buff[4] = (uint8_t)searchTimestamp;
				u8buff[5] = (uint8_t)(searchTimestamp >> 8);
				u8buff[6] = (uint8_t)(searchTimestamp >> 16);
				u8buff[7] = (uint8_t)(searchTimestamp >> 24);

				u8buff[8] = 0;

				u8buff[9] = (uint8_t) totalSector;
				u8buff[10] = (uint8_t) (totalSector >> 8);

//������һ��ʱ������ҵ��ϴ�ͬ����sector��ţ�����ͬʱ�����sector�������͸�APP��
				//
				if (searchTimestamp != DEFAULT_TIMESTAMP)
				{
					// ������ʱ��
					EnableLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA, false,
					                TIME_OUT_UPLOAD_DATA, onBleTaskTimeOut,
					                (void*)BLE_TASK_TYPE_DATA_UPLOADING);
				}
			}

			//
			SendData2Host(u8buff, 11); //�������ֻ�е������ϲŻ�����ݷ��ͳ�ȥ��


			break;
		}

		case  SEC_Upload_START:
		{
			//LED_TOGGLE();
			uint8_t result = 0x01;
			static uint16_t total_chunks = 0;
			time_t tsStart = 0, tsEnd = 0;

			// ��������sector
			uint16_t requestSector = (uint16_t)(pr[2] << 8) + (uint16_t)(pr[1]); //[BG021] uint32_t >> uint16_t

			if (requestSector == 0xFFFF)
			{
				// �����ж��Է�ֹ������һ��
				if (Upload_Sector_index != 0)
				{
					// process to next sector
					uint16_t tempSector = getNextIndexDataSector(Upload_Sector_Num);

					if (blDataDumpMode)
					{
						if (tempSector <= Upload_Sector_Num)
						{
							// dump �Ѿ���ɣ���Ϊ���ص���ǰ���sector
							result = 0xf0;
							goto FINISH_UPLOAD;
						}
						else
							Upload_Sector_Num = tempSector;
					}
					else
						Upload_Sector_Num = tempSector;
				}
			}
			else if (requestSector == 0xFFFE)
			{
				// resend current sector
			}
			else
			{
				// process specificed sector
				Upload_Sector_Num = requestSector; //��¼�ϴ�sector�����
				//Atus: [BG030] maybe should add Upload_Sector_index = 0; to force specify sector download.
			}


			// ��鴫�������Ч�� �� �Ƿ��п��ϴ�������
			//Upload_Sector_Num is physical flash secotr number.
			//if(Upload_Sector_Num < INDEX_DATA_START_SECTOR || Upload_Sector_Num > INDEX_DATA_END_SECTOR)
			if(Upload_Sector_Num > INDEX_DATA_END_SECTOR) //[BG032] move INDEX_DATA_START_SECTOR to below.
			{
				result = 0x03; // sector��������
			}
			else if (requestSector>=0xFFFE && Upload_Sector_Num < INDEX_DATA_START_SECTOR) //[BG032] add.
			{ //we accept the 0~INDEX_DATA_START_SECTOR if force specify.
				result = 0x03; // sector��������
			}
			else
			{
				// ��ȡҪ�����sector��ͷ��������sector�Ƿ�������
				// ���ķ���Ϊ��
				//		sector��tag��Ч����˵����sector��Ч�����п���Ϊδ��ʼ��sector��
				// 		sector��indexС���ϴδ����index��˵����sectorΪ�����ݣ�upload����
				//		sector��startTimestamp��endTimestampΪ DEFAULT_TIMESTAMP��0xffffffff����˵����sector���ڲɼ��У�������һ��upload��˵
				//      sector��startTimestamp��endTimestampΪ 0��˵����sectorΪ��sector��������һ��
//
				WakeFlashUp();

				//
				if (Upload_Sector_Num >= INDEX_DATA_START_SECTOR) //[BG032] add to support download not index data.
				{ //index data, need check tag and header.
					FLASH_SECTOR_HEAD fsh;

					while (true)
					{
						long addr = Upload_Sector_Num * pFlashInfo->sectorSize; //��¼�ϴ�sector����� ����sectorsize����ȡ�����ݵĵ�ַ
						ReturnType rt = FlashRead(addr, (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD));//��ȡsectorͷ��Ϣ

						if (rt != Flash_Success)
							result = 0x04;	// flash read failed
						else if (blDataDumpMode == false)
						{
							if (fsh.tag != FLASH_SECTOR_TAG)
								result = 0x02;	// sector��tag��Ч
							else if (fsh.index < Upload_Sector_index)
								result = 0x05;	//sector��indexС���ϴδ����index����sector�Ǿ����ݲ��ô���
							else if (fsh.startTimestamp == DEFAULT_TIMESTAMP || fsh.endTimestamp == DEFAULT_TIMESTAMP)
								result = 0x06;//��sector���ڲɼ��У����β��ϴ���
							else if (fsh.startTimestamp == 0 || fsh.endTimestamp == 0)
							{
								// ��sectorΪ��sector������
								Upload_Sector_Num = getNextIndexDataSector(Upload_Sector_Num);
//���������Upload_Sector_Num��ζ�Ÿ����˶�����ʼ��ַ��ȥ����һ��sector��
								continue;
							}
							else
								Upload_Sector_index = fsh.index; // ����  �ϴ�sector����ţ����ڱ��ؽ������߼���ţ�
						}
						else
							Upload_Sector_index = fsh.index; // ����

						break;
					}

					tsStart = fsh.startTimestamp;
					tsEnd = fsh.endTimestamp;
				}
			}

			if(scanWholeFlash && (result == 1))
			{
				//��һ�ν�����ɨ�������е�sector
				scanWholeFlash = false;
				total_chunks = GetTotalChunks(Upload_Sector_Num, Upload_Sector_index);
			}

			// ��Ӧ   ����������ᵽ��������ֻ������⵽�Ѿ�����󣬾���������������
FINISH_UPLOAD:
			u8buff[2] = result; //������6ʱ������ʾ��ǰsector���ڲɼ��У�û����Ч����Ҫ�ϴ��ˡ�

			u8buff[3] = (uint8_t)Upload_Sector_Num;
			u8buff[4] = (uint8_t)(Upload_Sector_Num >> 8);

			u8buff[5] = (uint8_t)tsStart;
			u8buff[6] = (uint8_t)(tsStart >> 8);
			u8buff[7] = (uint8_t)(tsStart >> 16);
			u8buff[8] = (uint8_t)(tsStart >> 24);

			u8buff[9] = (uint8_t)tsEnd;
			u8buff[10] = (uint8_t)(tsEnd >> 8);
			u8buff[11] = (uint8_t)(tsEnd >> 16);
			u8buff[12] = (uint8_t)(tsEnd >> 24);

			u8buff[13] = (uint8_t)Upload_Sector_index;
			u8buff[14] = (uint8_t)(Upload_Sector_index >> 8);
			u8buff[15] = (uint8_t)(Upload_Sector_index >> 16);
			u8buff[16] = (uint8_t)(Upload_Sector_index >> 24);

			u8buff[17] = (uint8_t)currentChunkNo;
			u8buff[18] = (uint8_t)(currentChunkNo >> 8);
			u8buff[19] = (uint8_t)total_chunks;
			u8buff[20] = (uint8_t)(total_chunks >> 8);


			SendData2Host(u8buff, 21);

			currentChunkNo++;//��ǰsector����0��ʼ���㡣

//����ֻ�ǻ�ȡ��Ҫ�ϴ���sector��ʱ������ϴ���������������һ����ʱ�����ڶ�ʱ����
			//
			if (result == 0x01)
			{
				// begin to send data
				ResetLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA);
//				blDataUploadingFlag=true;
//				bleTaskTimeoutCounter=TIME_OUT_UPLOAD_DATA;
//����һ��sector�����յ�ǰ���Ĵ�СҪ���ݶ��ٴΣ��Լ�����һ�������Ƕ��ٸ��ֽڡ��Լ�������ȡ�������ַ
				Upload_Count = pFlashInfo->sectorSize / Data_Upload_Pak_Size;
				Upload_Left = pFlashInfo->sectorSize % Data_Upload_Pak_Size;
				Upload_physical_add = Upload_Sector_Num * pFlashInfo->sectorSize;
				Upload_Sector_Crc = 0;

				//CRC_TEMP=FlashCRC(Upload_physical_add,4096);//systemStatus.flashInfo.sectorSize);

				EnableDelayTimer(TIMER_FLAG_BLE, true, 20, NULL, NULL);
			}

			break;
		}

		//��һ���ϴ����ݵķ���,������start/end sector index; ����ʼɨ������81���ҽ���ɨ������82ʱ��������ȷ�Ĳ������ſ������ء�
		case Flash_Upload_ByIndex:
		{
			BYTE searchRangeResult = 0x00;
			BYTE searchStartResult, searchEndResult;

			if (systemStatus.bBatteryLevel == OUT_OF_BATTERY)
			{
				memset(u8buff + 2, 0xff, 1);
				searchRangeResult = 0xf0;
				//u8buff[2] = result;
			}
			else
			{
				vTaskDelay(20);
				Change_BLE_CONNECT_INTERVAL(BLE_CONNECT_20ms);//�������Ӽ���Ѿ�����̵�
				vTaskDelay(20);

				SetFlashBusyBit(FLASH_BUSY_BIT_BLE);
				WakeFlashUp(); //�������flash�����Ĺر�ʱ�Զ��ģ�ʱ�䵽���Զ����flash�رա�

				searchStartIndex = (uint32_t)(pr[4] << 24) +
				                   (uint32_t)(pr[3] << 16) +
				                   (uint32_t)(pr[2] << 8) +
				                   (uint32_t)(pr[1]);

				searchEndIndex = (uint32_t)(pr[8] << 24) +
				                 (uint32_t)(pr[7] << 16) +
				                 (uint32_t)(pr[6] << 8) +
				                 (uint32_t)(pr[5]);

				if((searchEndIndex - searchStartIndex) > (INDEX_DATA_END_SECTOR - INDEX_DATA_START_SECTOR))
					searchRangeResult = 0x40;//���ҵķ�Χ������ǰ����ķ�Χ��
				else if(searchEndIndex <= searchStartIndex)
					searchRangeResult = 0x60; //����indexС����ʼindex
				else
					searchRangeResult = 0x80;

				searchStartResult = searchEndResult = searchRangeResult;

				searchStartResult |= SearchSectorIndex(searchStartIndex, &startUploadSectorNum, true);
				searchEndResult |= SearchSectorIndex(searchEndIndex, &endUploadSectorNum, false);
			}

			if((searchStartResult == 0x81) && (searchEndResult == 0x83))
			{
				//
				int bytes = (searchEndIndex - searchStartIndex + 1) * sizeof(SECTOR_NODE);
				sectorNode = (SECTOR_NODE*)malloc(bytes);
				tempSectorNode = sectorNode;

				//�����ɨ���ƥ��ܷ�ʱ��
				if(sectorNode != NULL)
					CombinIndexNumber(tempSectorNode, searchStartIndex, startUploadSectorNum,(searchEndIndex - searchStartIndex), &existIndexCounter);
				else
					searchRangeResult = 0x70;
			}

			u8buff[2] = searchRangeResult;
			u8buff[3] = searchStartResult;
			u8buff[4] = searchEndResult;
			SendData2Host(u8buff, 5);
			break;
		}

		case Flash_Upload_NextSector:
		{
//�����ϴ�sector����

			bool blFinish = false;
			int tempExistIndexCounter = existIndexCounter;

			uint16_t requestSector = (uint32_t)(pr[2] << 8) +
			                         (uint32_t)(pr[1]);

			Upload_Count = pFlashInfo->sectorSize / Data_Upload_Pak_Size;
			Upload_Left = pFlashInfo->sectorSize % Data_Upload_Pak_Size;
			Upload_physical_add = (tempSectorNode->sectorNumber) * pFlashInfo->sectorSize;
			Upload_Sector_Crc = 0;//ÿ���ϴ�һ��sectorʱ����crc��������

			if(0xFFFF == requestSector)
				totalUploadSector++;//ֻ�е�������һ��ʱ����ֵ�ű��ӡ�

			if(existIndexCounter > 0)
			{
				//�����м���ĳЩindex�Ҳ�����
				if(tempExistIndexCounter < searchEndIndex)
				{
					u8buff[5] = (BYTE)tempExistIndexCounter;
					u8buff[6] = (BYTE)(tempExistIndexCounter >> 8);
					u8buff[7] = (BYTE)(tempExistIndexCounter >> 16);
					u8buff[8] = (BYTE)(tempExistIndexCounter >> 24);
				}


				if(totalUploadSector <= existIndexCounter - searchStartIndex)
				{
					//sector��û�д���
					if(0xFFFF == requestSector)
					{
						//�ϴ���һ��sector������
						tempSectorNode++;//����һ����
					}
					else if(0xFFEF == requestSector )
					{
						//�����ϴ���ǰ��

					}

					u8buff[2] = blFinish;
				}
				else
				{
					blFinish = true;
					u8buff[2] = blFinish;
					u8buff[3] = (BYTE)totalUploadSector;
					u8buff[4] = (BYTE)(totalUploadSector >> 8);//��2����Ҫ���ڵ�����Ϣ
					SendData2Host(u8buff, 9);

					//
					if(sectorNode != NULL)
					{
						free(sectorNode);
						sectorNode = NULL;
					}

					break;//�����е�sector�����ϴ����ˡ�
				}

				//
				u8buff[3] = (BYTE)totalUploadSector;
				u8buff[4] = (BYTE)(totalUploadSector >> 8);//��2����Ҫ���ڵ�����Ϣ

			}
			else
			{
				//������������ĳ��index�Ҳ�����
				u8buff[3] = (BYTE)totalUploadSector;
				u8buff[4] = (BYTE)(totalUploadSector >> 8);//��2����Ҫ���ڵ�����Ϣ

				u8buff[5] = 0;
				u8buff[6] = 0;
				u8buff[7] = 0;
				u8buff[8] = 0;

				if(totalUploadSector <= (searchEndIndex - searchStartIndex) + 1)
				{
					//sector��û�д���
					if(0xFFFF == requestSector)
					{
						//�ϴ���һ��sector������
						tempSectorNode++;//����һ����
					}
					else if(0xFFEF == requestSector )
					{
						//�����ϴ���ǰ��

					}

					u8buff[2] = blFinish;

				}
				else
				{
					blFinish = true;
					u8buff[2] = blFinish;
					SendData2Host(u8buff, 9);
					break;//�����е�sector�����ϴ����ˡ�
				}
			}


			SendData2Host(u8buff, 9);
			EnableDelayTimer(TIMER_FLAG_BLE, true, 20, NULL, NULL);

			break;
		}

		case Get_Max_Index:
		{ //[Pe1072] add.
			int maxIndex = 0;
			uint16_t secnum = 0;
			SetFlashBusyBit(FLASH_BUSY_BIT_BLE);
			WakeFlashUp(); //�������flash�����Ĺر�ʱ�Զ��ģ�ʱ�䵽���Զ����flash�رա�

			maxIndex = SearchMaxSectorIndex(&secnum);
			u8buff[2] = (uint8_t)maxIndex;
			u8buff[3] = (uint8_t)(maxIndex >> 8);
			u8buff[4] = (uint8_t)(maxIndex >> 16);
			u8buff[5] = (uint8_t)(maxIndex >> 24);
			u8buff[6] = (uint8_t)secnum;	//[BG030] add.
			u8buff[7] = (uint8_t)(secnum >> 8); //[BG030] add.
			SendData2Host(u8buff, 8); //6 > 8
			
			break;
		}
		
		case  Flash_Upload_END:
#if 0  //I4_C remark, maybe it terminated not by this command.
                       //if download by index, we should free the sectorNode allocated by earlier.
//			if(sectorNode != NULL)
//			{
//				free(sectorNode);
//				sectorNode = NULL;
//			}
#endif             
			if(totalUploadSector != 0)
				totalUploadSector = 0; //��λ�����´����ء�

			if(existIndexCounter != 0)
				existIndexCounter = 0;

			//
			DisableLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA);

			ClearFlashBusyBit(FLASH_BUSY_BIT_BLE);
			PutFlashToSleep();

			u8buff[2] = 0;
			u8buff[3] = 0;
			SendData2Host(u8buff, 6);
			currentChunkNo = 0;//���ͽ���ʱ���Ѹ�ֵ���㡣

			break;

		case EraseDataGather:
		{
			if (checkActionPassword(pr + 1) == false)
			{
				u8buff[2] = 1;
				SendData2Host(u8buff, 6);

				return;
			}

//			FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//			fcmd->cmd = FLASH_CMD_RESET_DATA_GATHER;
//			osMailPut(hFlashCommandQueue, fcmd);

			FLASH_COMMAND fcmd;
			fcmd.cmd = FLASH_CMD_RESET_DATA_GATHER;

			if (pr[3] == 'E' && pr[4] == 'A')
				fcmd.data.v = 1;

			xQueueSend(hEvtQueueFlash, &fcmd, 0);

			// ��Ӧ
			u8buff[2] = 0;
			SendData2Host(u8buff, 6);

			break;
		}

		case ResetAccumulativeData:
		{

			iCalories = 0;
			iCalories_lastSaving = 0;
			iSteps = 0;
			iSteps_lastSaving = 0;
			iDistance = 0;
			iDistance_lastSaving = 0;
			active_level = 0;
			active_level_lastSaving = 0;
			active_level_delta = 0;

			// ��Ӧ
			u8buff[2] = 0;
			SendData2Host(u8buff, 3);
			break;
		}



		/*********  Firmware update   *********************************/
		//����firmware upgrading.doc�ĵ�����ȡЭ��
		case  GET_DEV_INFO:
			u8buff[2] = (uint8_t)systemStatus.flashInfo.sectorSize;
			u8buff[3] = (uint8_t)(systemStatus.flashInfo.sectorSize >> 8);
			u8buff[4] = (uint8_t)systemStatus.flashInfo.sectors;
			u8buff[5] = (uint8_t)(systemStatus.flashInfo.sectors >> 8);
			u8buff[6] = APP_FW_VER_M;
			u8buff[7] = APP_FW_VER_S;

			u8buff[8] = BLE_DevChip.BLE_Device.FW_VER1;
			u8buff[9] = BLE_DevChip.BLE_Device.FW_VER2;
			memcpy(&u8buff[10], &BLE_DevChip.BLE_DeviceInfo[4], 6); //mac address

			u8buff[16] = *(uint8_t*) BOOT_FW_VER_M_Add; //BOOT_FW_VER_M;
			u8buff[17] = *(uint8_t*) BOOT_FW_VER_S_Add; //BBOOT_FW_VER_S;

			u8buff[18] = systemSetting.bDeviceModel; // i4
			u8buff[19] = (uint8_t)systemSetting.SystemRstTimes;
			u8buff[20] = (uint8_t)systemSetting.SystemRstTimes >> 8;

			SendData2Host(u8buff, 21);
			break;

		case  ERASE_SECTOR_COMM:
		{
		  	ReturnType ret = Flash_SectorNrInvalid; //[BG031-1] add.
			sector_Start_num = pr[1] + (uint16_t)(pr[2] << 8);

			if(sector_Start_num < systemStatus.flashInfo.sectors) //[BG031-1] sectorSize >> sectors
			{
				ret = FlashSectorErase(EXT_FLASH_SECTOR_OFFSET + sector_Start_num);
			}

			//
			u8buff[2] = (uint8_t)sector_Start_num;
			u8buff[3] = (uint8_t)(sector_Start_num >> 8);
			if (ret == Flash_Success) //[BG031-1] add
				u8buff[4] = 0x00;
			else
			  	u8buff[4] = 0x01;

			//u8buff[5] = 0; //[BG031] removed.

			SendData2Host(u8buff, 5); //[BG031] 10 >> 5

			break;
		}

		case FW_Update_COMM:
		{
			uint8_t update_sta = 0x01; // 0 >> 0x01 indicate fail status.

			FW_Update_sta = pr[1];

			if(FW_Update_sta == 1) //fw update begin
			{
				if (systemStatus.bBatteryLevel == OUT_OF_BATTERY)
				{
					update_sta = 0xf0; // out of battery
				}
				else
				{
#if (AFE44x0_SUPPORT==1)
					AFE44xx_Shutoff();
#endif

					if((systemSetting.blHRSensorEnabled == true) && (systemStatus.blHRSensorTempEnabled == true))
						blTurnedOffPpgByBleTask = true;

//					systemSetting.blHRSensorEnabled = false;
//					systemStatus.blHRSensorEnabled  = false;

					// ������ʱ��
//					blFwUpgradingFlag=true;
					EnableLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA, false,
					                TIME_OUT_UPGRADE_FW, onBleTaskTimeOut,
					                (void*)BLE_TASK_TYPE_FW_UPGRADING);

					//
					SetFlashBusyBit(FLASH_BUSY_BIT_BLE);
					WakeFlashUp();
					update_sta = 2;//2015��6��11��10:56:59 ��������Э�飬�Ÿĵġ�������2��ʾ�ɹ���

					if(pr[2] == 0x55)//�ĵ���˵������������ֽ���0x55,��ô��������2���ֽڰ��������ݴ�ŵ���ʼsector��ƫ����
						EXT_FLASH_SECTOR_OFFSET = pr[3] + (uint16_t)(pr[4] << 8);
					else
						EXT_FLASH_SECTOR_OFFSET = 0;
				}
			}
			else  if(FW_Update_sta == 2) //fw update end
			{
				//
				DisableLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA);

				if(blTurnedOffPpgByBleTask == true)
				{
					blTurnedOffPpgByBleTask = false;
//					systemSetting.blHRSensorEnabled = true;
					systemStatus.blHRSensorTempEnabled = true;
				}

				//
				update_sta = 1; //  if update_sta==1 ,update failed,  ==2 ,success
				FlashRead(EXT_FLASH_SECTOR_OFFSET * 4096, FW_INFO.INFO_BUF, 16);

				if(FW_INFO.INFO.Head_CRC == CRC_calc(&FW_INFO.INFO_BUF[0], &FW_INFO.INFO_BUF[13]))
				{
					CRC_TEMP = FlashCRC(EXT_FLASH_SECTOR_OFFSET * 4096 + 16, FW_INFO.INFO.fw_length);

					if(FW_INFO.INFO.fw_crc == CRC_TEMP) //download is correct
					{
						//���ؽ����ر���Ļ�����������ؽ���
						blDuringDownload = false;
						once = false;
#if (OLDE_SUPPORT==1)
						clearScreen(false);
						OLEDOff();
#endif
						update_sta = 2; //  if update_sta==1 ,update failed,  ==2 ,success
						u8buff[2] = FW_Update_sta;
						u8buff[3] = update_sta; //  if update_sta==1 ,update failed,  ==2 ,success
						SendData2Host(u8buff, 4); // [BG031] 6 >> 4 remove dummy[2]
						vTaskDelay(100);
						RESET_MCU();

						while(1);
					}
				}

				once = false; //[BG030] should reset ready for next time even fail.
				LED_OFF();
				ClearFlashBusyBit(FLASH_BUSY_BIT_BLE);
				PutFlashToSleep();
			}

			u8buff[2] = FW_Update_sta;
			u8buff[3] = update_sta; //  if update_sta==1 ,update failed,  ==2 ,success
			SendData2Host(u8buff, 4); // [BG031] 6 >> 4 remove dummy[2]

			break;
		}

//���Է��ֻ�ȡCRCʱ��������ʼ��ַ�ͽ�����ַ��һ���ġ�
		case FW_CRC_GET:
		{
#if 1 //BG033_FIX
		  	bool flashstate = CheckFlashBusyStatus(); //[BG033] add, to check do wakeup or not.
			
			if (flashstate==false) //[BG033] add block
			{
				SetFlashBusyBit(FLASH_BUSY_BIT_BLE);
				WakeFlashUp();
			}
#endif
			sector_Start_num = pr[1] + (uint16_t)(pr[2] << 8);
			sector_start_addr = sector_Start_num * systemStatus.flashInfo.sectorSize;
#if 0	//[BG031] Atus: Somebody disable multi-sectors calculate, I patch it for reference.  
			sector_end_num = pr[3] + (uint16_t)(pr[4] << 8); //[BG025] not used, remark.
			if (sector_Start_num<=sector_end_num) 
			{
			  	sector_len = (sector_end_num - sector_Start_num + 1)*systemStatus.flashInfo.sectorSize;
			}
			else
			{
			  	sector_len = systemStatus.flashInfo.sectorSize;
			}
			CRC_TEMP = FlashCRC(EXT_FLASH_SECTOR_OFFSET * systemStatus.flashInfo.sectorSize + sector_start_addr, sectorLlen);
#endif
			/* In the test, we find sector_Start_num, is same as sector_end_num. It means one sector request. */
			CRC_TEMP = FlashCRC(EXT_FLASH_SECTOR_OFFSET * 4096 + sector_start_addr, 4096); //systemStatus.flashInfo.sectorSize);

#if BGXXX==8
			test3.typeuint16[1] = EXT_FLASH_SECTOR_OFFSET;
			test2.typeuint16[0] = (uint16_t)sector_Start_num + EXT_FLASH_SECTOR_OFFSET;
			test2.typeuint16[1] = CRC_TEMP;
#endif
#if 1 //BG033_FIX
			if (flashstate==false) //[BG033] add block
			{
				ClearFlashBusyBit(FLASH_BUSY_BIT_BLE);
				PutFlashToSleep();
			}
#endif
			u8buff[2] = pr[1];
			u8buff[3] = pr[2];
			u8buff[4] = pr[3];
			u8buff[5] = pr[4];

			u8buff[6] = (uint8_t)CRC_TEMP;
			u8buff[7] = (uint8_t)(CRC_TEMP >> 8);

			SendData2Host(u8buff, 8); //[BG031] 10 >> 8


			//------------------------------------------------------------------
			downloadPercentage = (int)(((float)sector_Start_num / allSector  ) * 100);
			USER_EVENT event;
			event.type = EVT_TYPE_DOWNLOAD;
//			event.data.p = (void*)percentage;    //���Ҫ�Ķ���������ȥ�����ϱ仯
#if BGXXX>0
			//xQueueSend(hEvtQueueDisplay, &event, 0);
#else
			xQueueSend(hEvtQueueDisplay, &event, 0);
#endif

			blDuringDownload = true;//�ñ�������ȷ����������������ز˵�������ʾ��

			//
			break;
		}

		case WR_SECTOR_COMM:

			WR_SEC_STA = pr[3];

			if(WR_SEC_STA == 1) //begin
			{
				WR_SEC_LOC = pr[1] + (uint16_t)(pr[2] << 8);

				//if(WR_SEC_LOC>63)WR_SEC_LOC=63;// avoid to beyond the board

				WR_SEC_LOC = WR_SEC_LOC * systemStatus.flashInfo.sectorSize;
				WR_SEC_OFFSET = 0;
				FW_RX_BUFF_WP = 0;
				Left_SEC_DATA = systemStatus.flashInfo.sectorSize;
				WR_SEC_COUNT = 0;
			}
			else if(WR_SEC_STA == 2) //end �����ʾ1��sector�Ѿ�д����
			{
				WR_SEC_OFFSET = 0;
			}

			u8buff[2] = pr[1]; //(uint8_t)WR_SEC_LOC;
			u8buff[3] = pr[2]; //(uint8_t)(WR_SEC_LOC >> 8);
			u8buff[4] = WR_SEC_STA;
			u8buff[5] = (uint8_t)WR_SEC_COUNT;
			u8buff[6] = (uint8_t)(WR_SEC_COUNT >> 8);
			SendData2Host(u8buff, 7); //[BG031] 10 >> 7
			break;

//ÿ����λ������1��sector�����ݣ��ֳ����ɸ�BLE����ÿ��ֻ��д19��byte�����͵��豸���豸ֱ��д��Ӧ��
		case FW_Update_DATA:
		{
		  	ReturnType rett;
		  	static char entrcount = 0; //[BG030] add to protect the semaphone destroy while reentrance.
		  
		  	if (entrcount!=0)
		  	{ //reentrance occurs, will overrun.
#if BGXXX==8
				test1.typeuint16[1] = WR_SEC_COUNT;
				test3.typechar[0] ++; //check reentrance flag+1
#endif
				entrcount = 0;
				break;
		  	}
			entrcount ++; //check reentrance flag+1
		    
//	        bleTaskTimeoutCounter=TIME_OUT_UPGRADE_FW;
			ResetLongTimer(LONG_TIMER_FLAG_UPLOAD_DATA);

			WR_SEC_COUNT++; //packet count increment.

			//Atus: It should check Left_SEC_DATA>0 can write data.
			for(i = 0; i < 19; i++)
			{
				FW_RX_BUFF[FW_RX_BUFF_WP++] = pr[1 + i];
			}

			if(Left_SEC_DATA >= FW_RX_BUFF_SIZE)
			{
				if(FW_RX_BUFF_WP >= FW_RX_BUFF_SIZE)
				{
					Left_SEC_DATA -= FW_RX_BUFF_SIZE;
					FW_RX_BUFF_WP = 0;
					rett = FlashProgram(EXT_FLASH_SECTOR_OFFSET * 4096 + WR_SEC_LOC + WR_SEC_OFFSET, FW_RX_BUFF, FW_RX_BUFF_SIZE);
					if (rett != Flash_Success)
					{
#if BGXXX==8
	  					test3.typechar[1] ++;
#endif
					}
					WR_SEC_OFFSET += FW_RX_BUFF_SIZE;

					if(once == false)
					{
						//firmwareͷ����ֻ��16���ֽڣ� ��һ��д���϶���д��ȥ�ˣ������������������Ҫ�����ȡfirmware���ܳ��ȡ�
						rett = FlashRead(EXT_FLASH_SECTOR_OFFSET * 4096, FW_INFO.INFO_BUF, 16);
						if (rett != Flash_Success)
						{
#if BGXXX==8
		  					test3.typechar[1] ++;
#endif
						}
						allSector = FW_INFO.INFO.fw_length / 4096.0;
						if (allSector==0.0)
						  allSector = 1.0; //[BG032] avoid divide by zero.
						once = true;
					}

					LED_TOGGLE();
				}
			}
			else
			{
				if(FW_RX_BUFF_WP >= Left_SEC_DATA)
				{
					rett = FlashProgram(EXT_FLASH_SECTOR_OFFSET * 4096 + WR_SEC_LOC + WR_SEC_OFFSET, FW_RX_BUFF, Left_SEC_DATA);
					if (rett != Flash_Success)
					{
#if BGXXX==8
	  					test3.typechar[1] ++;
#endif
					}

					Left_SEC_DATA = 0;
				}
			}
			entrcount --; //check reentrance flag-1
			if (entrcount!=0)
			{ //reentrance occurs.
#if BGXXX==8
			test3.typechar[0] ++; //check reentrance flag-1
#endif
			}
			break;
		}

		/********* ***************************************************/

		case getRealData:
			//if(BLE_STATE != BLE_STATE_CONNECTED)
			//break;
			RealDataOnPhone = pr[1];

			//�ȷ�����
			u8buff[2] = pr[1]; //0; [BG022-1] 0>>pr[1]
			//vTaskDelay(2);
			SendData2Host(u8buff, 3);  //[BG022-1] 4>>3

			//
#if PPG_WORK_MODE_SET
			if((RealDataOnPhone == 1) && (ppgWorkSpanCount < 61)) //�ڶ���������ʾ�������ppg��ĳ��ģʽ�£������ڹر�ppg��״̬ʱ����֧�ֲ鿴ʵʱ���Ρ�
#else
			if(RealDataOnPhone == 1)
#endif
			{
				vTaskDelay(20);
				Change_BLE_CONNECT_INTERVAL(BLE_CONNECT_20ms);
				vTaskDelay(20);

#if BATTERY_LIFE_OPTIMIZATION
				systemStatus.blHRSensorTempEnabled = false;
#endif
#if (AFE44x0_SUPPORT==1)
				AFE44xx_PowerOn_Init();
#endif
				SensorOffDelay = default_Ble_SensorOffDelay * current_touchsensor_feq;
			}

			//
			break;

		case ClockSynch:
		{
			// ����ʱ��
			systemSetting.timezoneOffset = (int16_t)(pr[8]) + (int16_t)(pr[9] << 8);
			SaveSystemSettings();

			// ���ñ���ʱ��
			struct tm myCalendar;
			int iyear;
			iyear = pr[2];
			iyear = iyear * 256;
			iyear += pr[1];

			myCalendar.tm_year = iyear - 1900;
			myCalendar.tm_mon = pr[3] - 1;
			myCalendar.tm_mday = pr[4];
			myCalendar.tm_hour = pr[5];
			myCalendar.tm_min = pr[6];
			myCalendar.tm_sec = pr[7];
			myCalendar.tm_isdst = 0;

			/* Setup BURTC */
			//burtcSetup();

			/* Backup initial calendar (also to initialize retention registers) */
			//clockAppBackup();

			clockAppInit(myCalendar, true);

			//SetRTCCalendar(&myCalendar);

			if(blTimeReset == 0x01)
				blTimeReset = 0xAA; //�Ѿ�ͬ����ʱ�䣬�Ѹñ�����Ϊ��

			//
			//startFlashLed(true, 1, 4, 1, 1, 4, 1);

			// ����ʱ��ͬ������Ϣ�������ڸ���low power watch֮���ˢ��Ƶ�ʵĽ���
//			osMessagePut(hMsgInterrupt, MESSAGE_CLOCK_SYNC, 0);

			int32_t msg = MESSAGE_CLOCK_SYNC;
			xQueueSend(hEvtQueueDevice, &msg, 0);

			u8buff[2] = 0;
			SendData2Host(u8buff, 3); //[BG022-1] 4 >> 3

			break;
		}

		case AlarmSynch:
		{
			if(pr[1] == ACTION_GET) // read
			{
				u8buff[2] = 0;
				u8buff[3] = pAlarmSetting->enabled;
				u8buff[4] = pAlarmSetting->alarmTime.Hour;
				u8buff[5] = pAlarmSetting->alarmTime.Minute;

				u8buff[6] = 0;

				for (int i = 0; i < 7; i++)
					u8buff[6] |= pAlarmSetting->weekday[i] << i;

				SendData2Host(u8buff, 10);
			}
			else  //setting
			{
				pAlarmSetting->mode = ALARM_MODE_WEEKDAY;
				pAlarmSetting->enabled = pr[2] > 0;
				pAlarmSetting->alarmTime.Hour = pr[3];
				pAlarmSetting->alarmTime.Minute = pr[4];

				for (int i = 0; i < 7; i++)
				{
					if ((pr[5] >> i) & 0x01)
						pAlarmSetting->weekday[i] = true;
					else
						pAlarmSetting->weekday[i] = false;
				}

				//
				SaveSystemSettings();

				//
				//startFlashLed(true, 1, 4, 1, 1, 4, 1);

				// ������Ϣ����ʾ�û��������
//				osMessagePut(hMsgInterrupt, MESSAGE_ALARM_SYNC, 0);

				int32_t msg = MESSAGE_ALARM_SYNC;
				xQueueSend(hEvtQueueDevice, &msg, 0);

				//
				u8buff[2] = pr[1];
				u8buff[3] = 0;
				SendData2Host(u8buff, 4);
			}

			break;
		}

		case UserProfileSynch:
		{
			if(pr[1] == ACTION_GET) // read
			{
				u8buff[2] = 0; // reading the setting
				u8buff[3] = pUserProfile->unit;// 0; //unit(1-Metric ,0---English)
				u8buff[4] = pUserProfile->gender;//0; //gender ( 1-male ,0---femal)
				u8buff[5] = pUserProfile->height;//170; //CM height
				u8buff[6] = pUserProfile->weight;//65; //KG	 weight

				u8buff[7] = (BYTE) pUserProfile->birthYear;
				u8buff[8] = (BYTE) (pUserProfile->birthYear >> 8);

				SendData2Host(u8buff, 10);
			}
			else  //setting
			{
				pUserProfile->unit = (ENUM_UNIT_SYSTEM)pr[2];// 0; //unit(1-Metric ,0---English). [BG025] add (ENUM_UNIT_SYSTEM)
				pUserProfile->gender = (ENUM_GENDER)pr[3];//0; //gender ( 1-male ,0---femal). [BG025] add (ENUM_GENDER)
				pUserProfile->height = pr[4];//170; //CM height
				pUserProfile->weight = pr[5];//65; //KG	 weight

				pUserProfile->birthYear = (UINT16) pr[6] + ((UINT16) pr[7] << 8);

				//
				SaveSystemSettings();
				UpdateBaseLine();

				//
				//startFlashLed(true, 1, 4, 1, 1, 4, 1);

				//
				u8buff[2] = pr[1];
				u8buff[3] = 0;
				SendData2Host(u8buff, 4);
			}

			break;
		}

		case ActivityGoals:
		{
			if(pr[1] == ACTION_GET) // read
			{
				u8buff[2] = pr[1];

				u8buff[3] = (BYTE) systemSetting.userGoals[GOAL_SLEEP];

				u8buff[4] = (BYTE) systemSetting.userGoals[GOAL_STEPS];
				u8buff[5] = (BYTE) (systemSetting.userGoals[GOAL_STEPS] >> 8);

				u8buff[6] = (BYTE) systemSetting.userGoals[GOAL_DISTANCE];
				u8buff[7] = (BYTE) (systemSetting.userGoals[GOAL_DISTANCE] >> 8);

				u8buff[8] = (BYTE) systemSetting.userGoals[GOAL_CALORIES];
				u8buff[9] = (BYTE) (systemSetting.userGoals[GOAL_CALORIES] >> 8);
                                //[BG023-1] add 
				//u8buff[10] = (BYTE) systemSetting.userGoals[GOAL_UVexpT];
				//u8buff[11] = (BYTE) (systemSetting.userGoals[GOAL_UVexpT] >> 8);

				SendData2Host(u8buff, 10); //[BG023-1] 10 >> 12
			}
			else if(pr[1] == ACTION_SET)  //setting
			{
				systemSetting.userGoals[GOAL_SLEEP] = pr[2];
				systemSetting.userGoals[GOAL_STEPS] =  (UINT16) pr[3] + ((UINT16) pr[4] << 8);
				systemSetting.userGoals[GOAL_DISTANCE] =  (UINT16) pr[5] + ((UINT16) pr[6] << 8);
				systemSetting.userGoals[GOAL_CALORIES] =  (UINT16) pr[7] + ((UINT16) pr[8] << 8);
				//systemSetting.userGoals[GOAL_UVexpT] =  (UINT16) pr[9] + ((UINT16) pr[10] << 8); //[BG023-1] add.

				//
				SaveSystemSettings();

				//
				//startFlashLed(true, 1, 4, 1, 1, 4, 1);

				//
				u8buff[2] = pr[1];
				u8buff[3] = 0;
				SendData2Host(u8buff, 4);

//����û��ڴﵽĿ���������������Ŀ�꣬��ôĿ��ֵ����������Ҫ����
				if(blStepAccomplish == true)
					blStepAccomplish = false;

				if(blDistanceAccomplish == true)
					blDistanceAccomplish = false;

				if(blCalorieAccomplish == true)
					blCalorieAccomplish = false;

			}

			break;
		}

		case NotifySettings:
		{
			if(pr[1] == ACTION_GET) // get
			{
				u8buff[2] = 0;

				if ((systemSetting.notificationMode & 0x02) == 0x02)
					u8buff[3] = 0x01;
				else
					u8buff[3] = 0x00;

				if ((systemSetting.notificationMode & 0x01) == 0x01)
					u8buff[4] = 0x01;
				else
					u8buff[4] = 0x00;

				u8buff[5] = (BYTE) (systemSetting.notifiedServices & 0xFF);
				u8buff[6] = (BYTE) (systemSetting.notifiedServices >> 8);

				u8buff[7] = (BYTE) (g_sAncs.crtPairState);


				//
				SendData2Host(u8buff, 8);
			}
			else if(pr[1] == ACTION_SET) // set
			{
				if (pr[2] == 0x01)
					systemSetting.notificationMode |= 0x02;
				else
					systemSetting.notificationMode &= ~0x02;

				if (pr[3] == 0x01)
					systemSetting.notificationMode |= 0x01;
				else
					systemSetting.notificationMode &= ~0x01;

				systemSetting.notifiedServices = (UINT16) pr[4] + ((UINT16)pr[5] << 8);

				int8_t appType = pr[6]; // 0=ios;1=android

				//
				SaveSystemSettings();

				//
				//startFlashLed(true, 1, 4, 1, 1, 4, 1);

				//
				u8buff[2] = pr[1];
				u8buff[3] = 0;
				SendData2Host(u8buff, 4);

				//////////////////////////
				vTaskDelay(2);

				if(systemSetting.notifiedServices != 0)
				{
					if (appType != 1)
						EFM2BLE_SetANCS(1);
				}
				else
					EFM2BLE_SetANCS(0);

				/////////////////////////
			}

			break;
		}

		case FindMeSettings:
		{
			if(pr[1] == ACTION_GET) // get
			{
				u8buff[2] = pr[1];

				u8buff[3] = (systemSetting.findMeMode & 1) > 0 ? 1 : 0;
				u8buff[4] = (systemSetting.findMeMode & 2) > 0 ? 1 : 0;
				u8buff[5] = systemSetting.findMeDuration;

				SendData2Host(u8buff, 6);
			}
			else if (pr[1] == ACTION_SET)	// set
			{
				//
				systemSetting.findMeMode = (pr[2] == 1 ? 1 : 0) + (pr[3] == 1 ? 2 : 0);
				systemSetting.findMeDuration = pr[4];

				SaveSystemSettings();

				//
				//startFlashLed(true, 1, 4, 1, 1, 4, 1);

				// response
				u8buff[2] = pr[1];

				SendData2Host(u8buff, 3);
			}

			break;
		}

		case PhoneComing:
		{
			int8_t resp = 0;

			if ((systemSetting.notifiedServices & (1 << NOTIFY_SERVICE_IncomingCall)) == 0)
			{
				// �� �������� δ���ã���ֱ�ӷ���
				resp = 1;
			}
			else
			{
				if(pr[1] == 0)
				{
					//
					systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 0;
//					osMessagePut(hMsgInterrupt, NOTIFICATION + (NOTIFY_SERVICE_IncomingCall << 16), 0);

#if !BATTERY_LIFE_OPTIMIZATION2
					msg.params.type = NOTIFICATION;
					msg.params.param = NOTIFY_SERVICE_IncomingCall;
					xQueueSend(hEvtQueueDevice, &msg.id, 0);
#endif
				}

				if(pr[1] == 1)
				{
					//
//					systemStatus.incomingCallNumber[0] = 0;
					memset(systemStatus.incomingCallNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
					memcpy(systemStatus.incomingCallNumber, pr + 2, 18);

					systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 1;
//					osMessagePut(hMsgInterrupt, NOTIFICATION + (NOTIFY_SERVICE_IncomingCall << 16), 0);
#if 0
//ȥ��ԭ��notification�Ĳ���

					/**-----------------------------------------------------------------------------
					˼·��
					1) ��������buffer[4],��ʼΪ0;
					2) ÿ����һ���绰ʱ����¼ʱ�䣬��ɨ��buffer[],�ҵ���һ��Ϊ0��Ԫ��ʱ���ͰѸ�ʱ�丳ֵ��buffer�ĸ�Ԫ��
					3) buffer�е�����Ԫ�ض�����0ʱ������һ������currentTime��¼��ǰ��ʱ�䣬��ִ�����µĲ���
					3.1)ɨ������buffer�����뵱ǰʱ������1Сʱ������Ԫ��������Ϊ0
					3.2)���buffer�Ƿ�ȫ������
					3.3�����ȫ�����㣬��˵������1��Сʱ���Ѿ���5���绰
					3.4������в���Ԫ����0����ѵ�ǰֵ��䵽��һ��0��Ԫ�����

					-----------------------------------------------------------------------------**/
//					static time_t buffer[4] = {0};
					uint8_t i = 0, k;
					time_t currentTime = 0;

					//
					for( i = 0; i < 4; i++)
					{
						if(buffer[i] == 0)
							break;
					}

					if(i < 4)
						buffer[i] = time(NULL);
					else
					{
						//˵��4�����鶼�����ʱ����
						currentTime = time(NULL);

						//�����ڶ���ɨ��
						volatile	uint8_t k = 0;

						for( k = 0; k < 4; k++)
						{
							//�������бȵ�ǰʱ�����1Сʱ������Ԫ����Ϊ0
							if(currentTime - buffer[k] > ONE_HOUR_SECONDS)
							{
								buffer[k] = 0;
							}
						}



						volatile uint8_t m = 0;

						for( m = 0; m < 4; m++)
						{
							if(buffer[m] == 0)
								break;
						}

						if(m == 4)
						{
							//˵��������ȫ����0���������1��Сʱ������5���绰,���ѵ�ǰʱ�䱣����buffer�����
							notificationAlertParameters.callsCounterStatus = 0x01;

							buffer[0] = buffer[1];
							buffer[1] = buffer[2];
							buffer[2] = buffer[3];
							buffer[3] = currentTime;
						}
						else
						{
							//�ٴ�ɨ�裬�ѵ�ǰʱ��ŵ��ʺϵ�λ��
							volatile uint8_t n = 0;

							for( n = 0; n < 4; n++ )
							{
								if(buffer[n] == 0)
								{
									buffer[n] = currentTime;
									break;
								}
							}
						}
					}

#endif
#if !BATTERY_LIFE_OPTIMIZATION2
					msg.params.type = NOTIFICATION;
					msg.params.param = NOTIFY_SERVICE_IncomingCall;
					xQueueSend(hEvtQueueDevice, &msg.id, 0);
#endif

				}
				else if(pr[1] == 2) // missed call
				{
					//
					systemStatus.incomingCallNumber[0] = 0;
					memcpy(systemStatus.incomingCallNumber, pr + 2, 18);

					systemStatus.notifyEvents[NOTIFY_SERVICE_MissedCall] = 1;
//					osMessagePut(hMsgInterrupt, NOTIFICATION + (NOTIFY_SERVICE_MissedCall << 16), 0);

#if !BATTERY_LIFE_OPTIMIZATION2
					msg.params.type = NOTIFICATION;
					msg.params.param = NOTIFY_SERVICE_MissedCall;
					xQueueSend(hEvtQueueDevice, &msg.id, 0);
#endif
				}

				//			RaiseNotification(NOTIFY_SERVICE_IncomingCall);
			}

			u8buff[2] = resp;
			SendData2Host(u8buff, 4);

			break;
		}

		case notifyFeature:
		{
//			0x01-sms;
//			0x02-calendar;
//			0x03-alarm
			int8_t resp = 0;

			NOTIFY_SERVICE ns = NOTIFY_SERVICE_Other;

			if(pr[1] == NOTIFY_SERVICE_Email)
			{
				if ((systemSetting.notifiedServices & (1 << NOTIFY_SERVICE_Email)) == 0)
				{
					// �� �ʼ����������� δ���ã���ֱ�ӷ���
					resp = 1;
				}
				else
				{
					ns = NOTIFY_SERVICE_Email;
					uint8_t type = pr[2] & 0x0f;
					//uint8_t id = pr[2] >> 4; //[BG025] not used remark it.

					// �� i4�������� sender��Ϣ������ body ��Ϣ
					if (type == 1) // msg sender
					{
						memset(systemStatus.latestSmsNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
						memcpy(systemStatus.latestSmsNumber, pr + 3, 16);
					}
					else if (type == 2) // msg body
					{
//						uint8_t remain = pr[3];
//
//						if (remain == 0)
//						{
//							u8buff[2] = 0;
//							SendData2Host(u8buff, 4);
//						}
						ns = NOTIFY_SERVICE_Other;//�����ܵ����ŵ�����ʱ�����ڷ���notification��Ϣ����I4�в���Ҫ��Ϣ���ݡ�
					}
				}
			}
			else if(pr[1] == NOTIFY_SERVICE_Schedule)
			{
				ns = NOTIFY_SERVICE_Schedule;
			}

//			else if(pr[1] == 0x03)
//			{
//				systemStatus.notifyEvents[NOTIFY_SERVICE_Email]++;
//			}

			if (ns != NOTIFY_SERVICE_Other)
			{
//				osMessagePut(hMsgInterrupt, NOTIFICATION + (ns << 16), 0);
#if !BATTERY_LIFE_OPTIMIZATION2
				msg.params.type = NOTIFICATION;
				msg.params.param = ns;
				xQueueSend(hEvtQueueDevice, &msg.id, 0);
#else
				if(ns != NOTIFY_SERVICE_Email)
				{
					msg.params.type = NOTIFICATION;
					msg.params.param = ns;
					xQueueSend(hEvtQueueDevice, &msg.id, 0);
				}
#endif
			}


			// response
			u8buff[2] = resp;
			SendData2Host(u8buff, 4);

			break;
		}

		case Change_Connect_Interval:
		{
			u8buff[2] = 0x00;
			u8buff[3] = 0xff;

			if(pr[1] == ACTION_SET) // setting
			{
				uint8_t val = pr[2];

				if(val > BLE_CONNECT_1000ms)
					val = BLE_CONNECT_1000ms;

				Change_BLE_CONNECT_INTERVAL(val);
				vTaskDelay(20);
			}
			else if(pr[1] == ACTION_GET) //get
			{
				u8buff[3] = getConnectTime;
			}

			// response
			u8buff[2] = pr[1];
			SendData2Host(u8buff, 4);

			//
			break;
		}

		case Set_SystemMode:
		{
			if (checkActionPassword(pr + 1))
			{
				uint8_t oldMode = systemSetting.SystemMode;

//����֮����д3����Ϊ�˼�����������
				if((pr[4] == 0x00) || (pr[4] == 0x12) || (pr[4] == 0x56)) //set mode
				{
					if (oldMode != SYSTEM_MODE_RELEASED
					        && oldMode != SYSTEM_MODE_ACTIVATED)
						oldMode = SYSTEM_MODE_MANUFACTORING;

					uint8_t newMode = pr[3];

					if (newMode != SYSTEM_MODE_RELEASED
					        && newMode != SYSTEM_MODE_ACTIVATED)
						newMode = SYSTEM_MODE_MANUFACTORING;

					if (oldMode != newMode)
					{
						systemSetting.SystemMode = newMode;

						if (newMode == SYSTEM_MODE_RELEASED)
							systemSetting.FactoryTime = time(NULL);
						else if (newMode == SYSTEM_MODE_ACTIVATED)
							systemSetting.ActivateTime = time(NULL);

						//
						msg.params.type = MESSAGE_SYSTEM_MODE_CHANGED;
						msg.params.param = oldMode + ((newMode) << 8);

						xQueueSend(hEvtQueueDevice, &msg.id, 0);
					}

					u8buff[3] = 0x01;//

				}
				else if(pr[4] == 0xAA) //get mode
				{
					u8buff[3] = oldMode;//���ص�ǰ״̬
				}

				u8buff[2] = 0;
			}
			else
			{
				u8buff[2] = 1;
			}

			SendData2Host(u8buff, 4);

			//
			break;
		}

		case CalibrateDevice:
		{
			if (checkActionPassword(pr + 1))
			{
				AdjustCapVal();
			}

			//
			//startFlashLed(true, 1, 4, 1, 1, 4, 1);

			//
			u8buff[2] = 0;
			SendData2Host(u8buff, 4);

			break;
		}

		//��ӵ�ص�����Ϣ 2015��6��24��17:59:13
		case GET_BATTERY_LEVEL:
		{
			u8buff[2] = systemStatus.bBatteryRemaining;
			SendData2Host(u8buff, 3);

			break;
		}

		case SensorSettings:
		{
			if (pr[1] == ACTION_GET)
			{
				// get
				u8buff[2] = pr[1];

				u8buff[3] = (BYTE) (systemSetting.blHRSensorEnabled ? (systemStatus.blHRSensorOn ? 0x11 : 0x10) : 0x00);
				u8buff[4] = (BYTE) (systemSetting.blUVSensorEnabled ? 0x11 : 0x00);
				u8buff[5] = (BYTE) (systemSetting.blAccelSensorEnabled ? 0x11 : 0x00);
				u8buff[6] = (BYTE) (systemSetting.blTouchSensorEnabled ? 0x11 : 0x00);

				SendData2Host(u8buff, 10);
			}
			else if (pr[1] == ACTION_SET)	// set
			{
				// skin touch
				if ((pr[5] & 0xF0) == 0x20) // enable
				{
					systemSetting.blTouchSensorEnabled = true;
				}
				else if ((pr[5] & 0xF0) == 0x10) // disable
				{
					systemSetting.blTouchSensorEnabled = false;//������Ҫͨ���ֶ�����sensor��
				}

//˵����pc tools ������������ǵ��turn on ���� turn off ����ʼ��pr[2] = 0x22,�ο�ble�ĵ�.
				// ppg
				if ((pr[2] & 0xF0) == 0x20) // enable
				{
					//���´������ڽ���ֱ��Ѿ����ʱ������ppg�޷������򿪣���Ҫժ���ڴ���
					if ((systemSetting.blHRSensorEnabled == false) || (systemStatus.blHRSensorTempEnabled == false))
					{
						SensorOffDelay = 0;
						systemStatus.blSkinTouched = false;
					}

					//���touch on��Ҳ�ᵽ������ 2015��6��17��9:22:03
					systemSetting.blHRSensorEnabled = true; //20141209

					if (systemSetting.blTouchSensorEnabled == false)
					{
						// �� �Զ�����ppg�Ĺ��ܱ��رղ���ֱ�ӿ���ppg����
						if ((pr[2] & 0x0F) == 0x02) // on
						{
#if BATTERY_LIFE_OPTIMIZATION
							systemStatus.blHRSensorTempEnabled = false;
#endif
#if (AFE44x0_SUPPORT==1)
							AFE44xx_PowerOn_Init();
#endif
							SensorOffDelay = default_Key_SensorOffDelay * current_touchsensor_feq;
						}
						else if ((pr[2] & 0x0F) == 0x01) // off
						{
#if (AFE44x0_SUPPORT==1)
							AFE44xx_Shutoff();
#endif
						}
					}
				}
				else if ((pr[2] & 0xF0) == 0x10) // disable
				{
					systemSetting.blHRSensorEnabled = false;
#if (AFE44x0_SUPPORT==1)
					AFE44xx_Shutoff();
#endif
				}

				// uv
				if ((pr[3] & 0xF0) == 0x20) // enable
				{
					systemSetting.blUVSensorEnabled = true;
				}
				else if ((pr[3] & 0xF0) == 0x10) // disable
				{
					systemSetting.blUVSensorEnabled = false;
				}

				// accelerometer
				if ((pr[4] & 0xF0) == 0x20) // enable
				{
					systemSetting.blAccelSensorEnabled = true;
				}
				else if ((pr[4] & 0xF0) == 0x10) // disable
				{
					systemSetting.blAccelSensorEnabled = false;
				}

				//
				SaveSystemSettings();

				//
				u8buff[2] = pr[1];
				u8buff[3] = 0;
				SendData2Host(u8buff, 4);
			}

			break;
		}

		case ResetDevice:  // 0xf0 , 0x20  0x13
		{
			if (checkActionPassword(pr + 1))
				u8buff[2] = 0;
			else
				u8buff[2] = 1;

			SendData2Host(u8buff, 4);

			//
			vTaskDelay(10);
			RESET_MCU(); // reset the device

			while(1);

			//
			break;
		}

		case SetDeviceColor:
		{
			//0xf4,0x20,0x13,color
			if(checkActionPassword(pr + 1))
			{
				systemSetting.deviceColor = pr[3];
				SaveSystemSettings();

				u8buff[2] = 0x01;
				u8buff[3] = pr[3];
			}
			else
			{
				u8buff[2] = 0xFF;
				u8buff[3] = 0;
			}

			SendData2Host(u8buff, 4);
			break;
		}

		case ResetSettings:  // 0xfe , 0x20  0x13
		{
			if (checkActionPassword(pr + 1))
			{
				u8buff[3] = 0;

				//
				ResetParaSettings();
				SaveSystemSettings();
			}
			else
				u8buff[3] = 1;

			u8buff[2] = pr[1];
			SendData2Host(u8buff, 4);

			//
			break;
		}

		case ResetRebootCount:  // 0xfd , 0x20  0x13
		{
			if (checkActionPassword(pr + 1))
			{
				u8buff[3] = 0;

				//
				systemSetting.SystemRstTimes = 0;
				SaveSystemSettings();
			}
			else
				u8buff[3] = 1;

			u8buff[2] = pr[1];
			SendData2Host(u8buff, 4);

			//
			break;
		}

		case RunOutBatQuick: // 0xf1 , 0x20  0x13
		{
			if(checkActionPassword(pr + 1))
			{
//				if(pr[3] == 1)
//				{
//					//SavingBattery();
//				}
//				else if(pr[3] == 2)
//				{
//					//QuickRunOutBattery();
//				}
				StartBatteryDrain();
			}

			//
			u8buff[2] = 0;
			SendData2Host(u8buff, 4);

			break;
		}

		case Get_Menus_Info:
		{
			// ����豸�˵��б�
			// �����˵�id���Ƿ��ѡ�Լ��Ƿ�ʹ�ܡ��ܹ�3���ֽڣ�ÿ���˵��
			BYTE* mi = (BYTE*) malloc(3 * GetMenuCount());

			if (mi == 0)
				break;

			int mcount = GetMenuInfo(&mi);
//PC �����ÿ����ʾ5���˵����������Ҫ��ʾ���С�
			int pc = mcount / 5 + ((mcount % 5 == 0) ? 0 : 1);  //5 item per block

			//	���͵�app��
			BYTE* buff = (BYTE*) malloc(pc * 20 + 1);//per block 20 bytes. ʵ������Ҫ15���ֽڼ��ɴ洢��������ʹ��20���ֽڡ���������ܹ�Ҫʹ�ö��������洢���еĲ˵���Ϣ��

			if (buff == 0)
			{
				free(mi);
				break;
			}

			buff[0] = BLE_CH2;

			for (int i = 0; i < pc; i++)
			{
				//����һ��һ�еİѲ˵���Ϣ�������������͸�APP��
				(buff + 1)[i * 20] = Get_Menus_Info;  //data tag.

				if (i < pc - 1)
				{
					(buff + 1)[1 + i * 20] = 5;
					(buff + 1)[2 + i * 20] = mcount - (i + 1) * 5;

					memcpy((buff + 4 + i * 20), mi + (i * 5 * 3), 5 * 3);
				}
				else
				{
					(buff + 1)[1 + i * 20] = mcount - i * 5;
					(buff + 1)[2 + i * 20] = 0;

					memcpy((buff + 4 + i * 20), mi + (i * 5 * 3), (mcount - (i * 5)) * 3);
				}
			}
//#if (MODEL_TYPE==2) //CONSUMER_TYPE
//			LEUARTSentByDma(UART_CMD_2HOST, buff, pc*21);
//#endif
			LEUARTSentByDma(UART_CMD_2HOST, buff, pc*20+1); //Atus: pc*21 >> pc*20+1

			free(mi);
			free(buff);

			break;
		}

		case Enable_Menus:
		{
			// ����/ͣ�ò˵�
			// ��������ͣ�ñ�ѡ�Ĳ˵�
			MENU_TYPE menu;
			bool enabled;

			for (int i = 0; i < pr[1]; i++)
			{
				menu = (MENU_TYPE) pr[2 + i * 2];
				enabled = (pr[2 + i * 2 + 1] == 1);

				EnableMenu(menu, enabled);

				if (enabled)
					systemSetting.iMenuEnableFlags |= 1 << (BYTE) menu;
				else
					systemSetting.iMenuEnableFlags &= ~(1 << (BYTE) menu);
			}

			//
			SaveSystemSettings();

			//
			u8buff[2] = 0;
			SendData2Host(u8buff, 4);
			break;
		}

		case StatusReport:
		{
			// ����ϵͳ״̬�����ڵ���
			uint8_t idx = 2;

			u8buff[idx++] = (BYTE) systemSetting.blHRSensorEnabled;
			u8buff[idx++] = (BYTE) systemStatus.blHRSensorOn;
			u8buff[idx++] = (BYTE) systemSetting.blTouchSensorEnabled;

			u8buff[idx++] = (BYTE) systemStatus.blBatteryCharging;//(BYTE) GetCurrentMenuIndex();
			u8buff[idx++] = 0xFF;//(BYTE) GetCurrentMenuType();

			u8buff[idx++] = (BYTE) systemStatus.blEnableDataGathering;
			u8buff[idx++] = (BYTE) pFlashStorageIndicator->sector;
			u8buff[idx++] = (BYTE) ((pFlashStorageIndicator->sector) >> 8);
//			u8buff[idx++] = (BYTE) pFlashStorageIndicator->index;
//			u8buff[idx++] = (BYTE) ((pFlashStorageIndicator->index) >> 8);
//			u8buff[idx++] = (BYTE) pFlashStorageIndicator->capacity;
//			u8buff[idx++] = (BYTE) pFlashStorageIndicator->nextSectorIsPrepared;
			u8buff[idx++] = (uint8_t)(systemSetting.SKIN_CAPVAL);
			u8buff[idx++] = (uint8_t)(systemSetting.SKIN_CAPVAL >> 8);
			u8buff[idx++] = (uint8_t)(systemSetting.TOUCH_CAPVAL);
			u8buff[idx++] = (uint8_t)(systemSetting.TOUCH_CAPVAL >> 8);

//			u8buff[idx++] = (BYTE) pFlashStorageIndicator->startTimestamp;
//			u8buff[idx++] = (BYTE) ((pFlashStorageIndicator->startTimestamp) >> 8);
//			u8buff[idx++] = (BYTE) ((pFlashStorageIndicator->startTimestamp) >> 16);
//			u8buff[idx++] = (BYTE) ((pFlashStorageIndicator->startTimestamp) >> 24);
			u8buff[idx++] = (uint8_t)(SkinTouchVal);
			u8buff[idx++] = (uint8_t)(SkinTouchVal >> 8);
			u8buff[idx++] = (uint8_t)(touchValues);
			u8buff[idx++] = (uint8_t)(touchValues >> 8);

			u8buff[idx++] = (BYTE) (systemSetting.SystemMode);
			u8buff[idx++] = (BYTE) (systemStatus.blSystemSettingsCrcOk);

			SendData2Host(u8buff, 21);

			break;
		}
#if (MODEL_TYPE==2) //CONSUMER_TYPE
		case PPG_MODE:
		{
			if(pr[1] == ACTION_GET) // read
			{
				u8buff[2] = ACTION_GET;
				u8buff[3] = systemSetting.ppgRunMode;
				SendData2Host(u8buff, 4);
			}
			else
			{
				systemSetting.ppgRunMode = pr[2];
				SaveSystemSettings();
				u8buff[2] = ACTION_SET;
				u8buff[3] = pr[2];
				SendData2Host(u8buff, 4);

				if(systemSetting.ppgRunMode == 1 && ppgWorkSpanCount < 60)
					ppgWorkTimespan = 0;
				else if(systemSetting.ppgRunMode == 1 && ppgWorkSpanCount >= 61)
				{
#if (AFE44x0_SUPPORT==1)
					AFE44xx_PowerOn_Init();//����Ӧ���û����ڴ��ϱ��ʱ�򣬵�ǰ���Ѿ����ڹر�PPG��״̬�£��û��л���ȫ��ģʽ�£�PPG�����������⡣
#endif
					ppgWorkTimespan = 0;
					ppgWorkSpanCount = 0;
				}
				else if(systemSetting.ppgRunMode == 2)
					ppgWorkTimespan = 300;//5 minutes
				else if(systemSetting.ppgRunMode == 3)
					ppgWorkTimespan = 600;//10 minutes
				else if(systemSetting.ppgRunMode == 4)
					ppgWorkTimespan = 900;
				else if(systemSetting.ppgRunMode == 5)
					ppgWorkTimespan = 1800;
				else if(systemSetting.ppgRunMode == 6)
					ppgWorkTimespan = 3600;
			}

			break;
		}
#endif
		case READ_SERIAL_NUMBER:
			break;

		case WRITE_SERIAL_NUMBER:
			break;

		default:
#if BGXXX==8
		  test1.typeuint8[1] = test1.typeuint8[1]+1; //count error command.
#endif
			break;
	}
}


uint32_t BLE_Updata_Count;

void BLE_DATA_UPLOADING_PROC()
{
	uint8_t u8buff[21];

	static uint8_t index = 0;

	SetFlashUsingTime(30); //  20140529

	if(Current_Connect_Interval != getConnectTime)
    {
        /* Before do upload, will Change_BLE_CONNECT_INTERVAL to BLE_CONNECT_20ms,and set to Current_Connect_Interval,
         * the getConnectTime is update by ble(client), so if not match, it means the change interval are fail.
         * But why can not do it? 
         */
		return;
    }

	index++;

	if (dataUploadMode == 1) // slow speed mode
	{
		// ����1��������ͣ2������һ����20�ֽ�
		if(index < 3)
		{
			return;
		}
		else
			index = 0;
	}
	else
	{
		// ����1��������ͣ1������һ����80�ֽ�
		if(index >= 2)
		{
			index = 0;

			return;
		}
	}

	if(TxDone == false)
		return;

//	if(blDataUploadingFlag==true)
	{
		if(Upload_Count)
		{
			Upload_Count--;
			FW_RX_BUFF[0] = BLE_CH5;//����ʹ��ͨ����ͨ�;�����BLE�����Ǹ���������ݷ��ͳ�ȥ��
			FlashRead(Upload_physical_add, &FW_RX_BUFF[1], Data_Upload_Pak_Size);
			LEUARTSentByDma(UART_CMD_2HOST, FW_RX_BUFF, Data_Upload_Pak_Size + 1);
			CRC_Tracking(&FW_RX_BUFF[1], &FW_RX_BUFF[Data_Upload_Pak_Size], &Upload_Sector_Crc);//ÿ�η������ݶ�����CRC�����ǲ�û�з��ͳ�ȥ������
			Upload_physical_add += Data_Upload_Pak_Size;//����ַ����
		}
		else if(Upload_Left)
		{
			//���sector�����ˣ�ֻʣ�����һ������1����������
			FW_RX_BUFF[0] = BLE_CH5;
			FlashRead(Upload_physical_add, &FW_RX_BUFF[1], Upload_Left);
			LEUARTSentByDma(UART_CMD_2HOST, FW_RX_BUFF, Upload_Left + 1);
			CRC_Tracking(&FW_RX_BUFF[1], &FW_RX_BUFF[Upload_Left], &Upload_Sector_Crc);
			Upload_Left = 0;
		}
		else
		{
			//���sector������������������

//			blDataUploadingFlag=false;
			DisableDelayTimer(TIMER_FLAG_BLE);

			u8buff[0] = BLE_CH2;
			u8buff[1] = SEC_Upload_END;//�����ϴ����(������һ��sector���)������0x52����APP��
			u8buff[2] = (uint8_t)Upload_Sector_Num;
			u8buff[3] = (uint8_t)(Upload_Sector_Num >> 8);
			u8buff[4] = (uint8_t)Upload_Sector_Crc;
			u8buff[5] = (uint8_t)(Upload_Sector_Crc >> 8);
			SendData2Host(u8buff, 6); //[BG031] 10>>6
		}
	}
}

#if 1

//�����ݿ��ġ�����ANCS 0x09 �����͹�������Ҫ��
#ifdef __ANCS_BODY
#define ANCS_FLAG_MASK 0x0000000A
#else
#define ANCS_FLAG_MASK 0x00000002
#endif

NOTIFY_SERVICE g_ns = NOTIFY_SERVICE_Other;
void EFM2BLE_OpenANCS(bool enable);
void EFM2BLE_IsANCS(void);

/***
˵����
����qq��Ϣʱ��pData�е������ǣ� 02 00 06 00 00 00 01 02 00 51 51 03 0c 00 e4
b8 80 e6 b0 b4 3a 68 6a 6e 6d 62

˵����������ǰ����09����ʾANCS������.
�������������֪�� �����˽�ANCS����Ϣ�����ͨ�����ڷ��͹�����Э����ucDataArry[13]={UART_DATA_START,sizeof(ucDataArry),UART_CMD_ANCS,1,\
    noti->EventID,noti->EventFlag,noti->CategoryID,\
      noti->CategoryCount,noti->NotificationUID[0],\
        noti->NotificationUID[1],noti->NotificationUID[2],\
          noti->NotificationUID[3],UART_DATA_STOP};

// ANCS Notification info
typedef struct
{
  uint8 EventID;
  uint8 EventFlag;
  uint8 CategoryID;
  uint8 CategoryCount;
  uint8 NotificationUID[4];
}Ancs_Noti_t;
����EventID�Ͷ�Ӧ�����pData[1]��0��ʾ����¼���1��ʾ�ı��¼���2��ʾ�Ƴ��¼���

����pData[3]��ӦCategoryID����ANCS�����Ǳ���ȷ�����ˣ���Ӧ��ߵ�NOTIFY_SERVICE_xxx
***/
void BLE2EFM32_ANCSHandle(uint8_t* pData, uint16_t nLen)
{
	MESSAGE msg;

	switch (pData[0])
	{
		case 0://״̬���ء�

			break;

		case 1://֪ͨ������ݡ�
			if(pData[1] == 0) //add
			{
				g_sAncs.uuid.ucData8[0] = pData[5];
				g_sAncs.uuid.ucData8[1] = pData[6];
				g_sAncs.uuid.ucData8[2] = pData[7];
				g_sAncs.uuid.ucData8[3] = pData[8];
				g_sAncs.flag = 0;
				g_sAncs.msgLen = 0;
				g_ns = (NOTIFY_SERVICE) pData[3]; //��QQ��Ϣ��ʱ��������ʾ04����ANCS�ж������social ,����ȡ���͡�����Ҳ��04��Ҳ������social��
//��������Ϣ��ʱ������ʾ����09��NOTIFY_SERVICE_BusinessAndFinance
			}

			if (pData[1] == 2) // remove
			{
				if (pData[3] == NOTIFY_SERVICE_IncomingCall)
				{
					RemoveIncomingCallNotify();
//					osMessagePut(hMsgInterrupt, NOTIFICATION + (NOTIFY_SERVICE_IncomingCall << 16), 0);
					msg.params.type = NOTIFICATION;
					msg.params.param = NOTIFY_SERVICE_IncomingCall;
					xQueueSend(hEvtQueueDevice, &msg.id, 0);
				}

				return;
			}

			break;

		case 2://�׸����������ˡ�

			if(pData[1] == 0) //commandID =0
			{
				// if( g_sAncs.uuid.ucData8[0]== pData[2]&& g_sAncs.uuid.ucData8[1]== pData[3]&& \
				// g_sAncs.uuid.ucData8[2]== pData[4]&& g_sAncs.uuid.ucData8[3]== pData[5])
				{
					uint8_t* p = pData + 6;
					nLen -= 6; //�õ������ݳ�����
					uint16_t unLen;

					for(uint8_t ucCount = 0; nLen > 3 && ucCount < 8; ucCount++)
					{
						if(*p == 1 ) //Title
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);

							if(unLen > NOTIFY_SENDER_BUFFER_SIZE)
								unLen = NOTIFY_SENDER_BUFFER_SIZE;


							if((g_ns == NOTIFY_SERVICE_IncomingCall) || (g_ns == NOTIFY_SERVICE_MissedCall))
							{
								memset(systemStatus.incomingCallNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
								memcpy(systemStatus.incomingCallNumber, p + 3, unLen);
							}
							else if(g_ns == NOTIFY_SERVICE_Social)
							{
								//����Ķ��ź�QQ��Ϣ����ANCS�����04
								memset(systemStatus.latestSmsNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
								memcpy(systemStatus.latestSmsNumber, p + 3, unLen);

								if(systemStatus.latestSmsNumber[0] == 'Q' && systemStatus.latestSmsNumber[1] == 'Q')
									g_ns = NOTIFY_SERVICE_Other; //���ﲻ��ʾQQ��Ϣ

							}

							p = p + 3 + unLen;
							nLen -= (3 + unLen);
							g_sAncs.flag |= 2;//title ��ɡ�

						}// end if Title
						else if(*p == 0 )
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);
							p = p + 3 + unLen;
							nLen -= (3 + unLen);
						}// end else if 0
						else if(*p == 3 ) //Message  //������Ϣ������
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);
#ifdef __ANCS_BODY
							memset(notifyBodyBuffer, 0, NOTIFY_BODY_BUFFER_SIZE);
							memcpy(notifyBodyBuffer, p + 3, unLen);
#endif
							p = p + 3 + unLen;
							nLen -= (3 + unLen);
							g_sAncs.flag |= 8;//title ��ɡ�
						}// end else if 0
						else
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);
							p = p + 3 + unLen;

						}// end else if 0

						if((g_sAncs.flag & ANCS_FLAG_MASK ) == ANCS_FLAG_MASK) //�յ��������ġ�
						{

							break;
						}
					}//end for
				}// end if UUID
			}//if commandID =0

			break;

		case 3://�׸������滹�����ݡ�

			if(pData[1] == 0)
			{
				if( g_sAncs.uuid.ucData8[0] == pData[2] && g_sAncs.uuid.ucData8[1] == pData[3] && \
				        g_sAncs.uuid.ucData8[2] == pData[4] && g_sAncs.uuid.ucData8[3] == pData[5])
				{
					//�Ѿ���case 1�н����˸�ֵ�����Կ��ԱȽ��ˡ�
					uint8_t* p = pData + 6;
					nLen -= 6; //�õ������ݳ�����
					uint16_t unLen;

					for(uint8_t ucCount = 0; nLen > 3 && ucCount < 8; ucCount++)
					{
						if(*p == 1 ) //Title
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);

							if(unLen > NOTIFY_SENDER_BUFFER_SIZE)
								unLen = NOTIFY_SENDER_BUFFER_SIZE;

							if((g_ns == NOTIFY_SERVICE_IncomingCall) || (g_ns == NOTIFY_SERVICE_MissedCall))
							{
								memset(systemStatus.incomingCallNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
								memcpy(systemStatus.incomingCallNumber, p + 3, unLen);
							}
							else if((g_ns == NOTIFY_SERVICE_Social))
							{
								//����Ķ��ź�QQ��Ϣ����ANCS�����04
								memset(systemStatus.latestSmsNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
								memcpy(systemStatus.latestSmsNumber, p + 3, unLen);

								if(systemStatus.latestSmsNumber[0] == 'Q' && systemStatus.latestSmsNumber[1] == 'Q')
									g_ns = NOTIFY_SERVICE_Other; //���ﲻ��ʾQQ��Ϣ

							}

							p = p + 3 + unLen;
							nLen -= (3 + unLen);
							g_sAncs.flag |= 2;//title ��ɡ�

						}// end if Title
						else if(*p == 0 )
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);
							p = p + 3 + unLen;
							nLen -= (3 + unLen);
						}// end else if 0
						else if(*p == 3 ) //Message
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);
							g_sAncs.msgLen = unLen;
#ifdef __ANCS_BODY
							memset(notifyBodyBuffer, 0, NOTIFY_BODY_BUFFER_SIZE);
							memcpy(notifyBodyBuffer, p + 3, unLen);
#endif
							p = p + 3 + unLen;

							if(nLen < (3 + unLen))
							{
								nLen = 0;
								g_sAncs.crtEvent = 3;
								break;
							}
							else
							{
								nLen -= (3 + unLen);
								g_sAncs.flag |= 8;//title ��ɡ�
							}


						}// end else if 0
						else
						{
							unLen = p[1];
							unLen |= (((uint16_t)p[2]) << 8);
							p = p + 3 + unLen;
						}// end else if 0

						if((g_sAncs.flag & ANCS_FLAG_MASK ) == ANCS_FLAG_MASK) //�յ��������ġ�
						{

							break;
						}
					}//end for
				}// end if UUID
			}//if commandID =0

			break;
#ifdef __ANCS_BODY

		case 4://�����׸������������滹�����ݡ�

			if(g_sAncs.crtEvent == 3)//msg
			{
				if((g_sAncs.msgLen + nLen) > NOTIFY_BODY_BUFFER_SIZE)
				{
					nLen = NOTIFY_BODY_BUFFER_SIZE - g_sAncs.msgLen;
					g_sAncs.crtEvent = 0xff;
					g_sAncs.flag |= 0x08;
				}

				memcpy(notifyBodyBuffer + g_sAncs.msgLen, pData + 1, nLen);
				g_sAncs.msgLen += nLen;
			}

			break;

		case 5://�������ݡ�
			if(g_sAncs.crtEvent == 3)//msg
			{
				if((g_sAncs.msgLen + nLen) > NOTIFY_BODY_BUFFER_SIZE)
				{
					nLen = NOTIFY_BODY_BUFFER_SIZE - g_sAncs.msgLen;
					g_sAncs.crtEvent = 0xff;
				}

				g_sAncs.flag |= 0x08;
				memcpy(notifyBodyBuffer + g_sAncs.msgLen, pData + 1, nLen);
			}

			break;
#endif

		case 0x11://��ѯ����ص�ǰ״̬��0 û�д򿪣�1�򿪡�
			if(pData[1] == 0)
			{
				g_sAncs.crtPairState = PAIRING_DISABLE;
			}
			else
			{
				g_sAncs.crtPairState = PAIRING_ENABLE;
			}

			if(g_sAncs.crtPairState == g_sAncs.setPairState)//һ�����Ͳ������á�
			{
				//�������óɹ�����Ϣ��
			}
			else
			{
				if(g_sAncs.setPairState == PAIRING_ENABLE)
					EFM2BLE_OpenANCS(1);
				else
					EFM2BLE_OpenANCS(0);
			}

			break;

		case 0x12://�������������״̬��0���򿪣�1�򿪡�
			//�������óɹ�����Ϣ��
			BLE_RESET();
			break;

		default:
			break;
	}

	if((g_sAncs.flag & ANCS_FLAG_MASK ) == ANCS_FLAG_MASK)
	{
		cleanUpAsciiString(systemStatus.incomingCallNumber, NOTIFY_SENDER_BUFFER_SIZE);
		//     cleanUpUTF8String(systemStatus.incomingCallNumber, NOTIFY_SENDER_BUFFER_SIZE);
		//cleanUpUTF8String(notifyBodyBuffer, NOTIFY_BODY_BUFFER_SIZE);

		if (g_ns == NOTIFY_SERVICE_IncomingCall)
			systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 1;
		else if (g_ns == NOTIFY_SERVICE_MissedCall)
			systemStatus.notifyEvents[NOTIFY_SERVICE_MissedCall] = 1;
		else if(g_ns == NOTIFY_SERVICE_Social)
			systemStatus.notifyEvents[NOTIFY_SERVICE_Social] = 1;

		//����Ӧ�ü����жϣ�ֻ���յ��绰�����ţ�QQ��΢�ŵ�notification���ŷ����������Ϣ��
//		osMessagePut(hMsgInterrupt, NOTIFICATION + (g_ns << 16), 0);

		if((g_ns == NOTIFY_SERVICE_IncomingCall) || (g_ns == NOTIFY_SERVICE_Social)
		        || (g_ns == NOTIFY_SERVICE_MissedCall))
		{
#if (MODEL_TYPE==2) //CONSUMER_TYPE	//healthcare�汾������msm��incoming call��
			msg.params.type = NOTIFICATION;
			msg.params.param = g_ns;
			xQueueSend(hEvtQueueDevice, &msg.id, 0);
#endif
		}
	}
}

//����ANCS
void EFM2BLE_SetANCS(bool enable)
{
	if(enable == 1)
		g_sAncs.setPairState = PAIRING_ENABLE;
	else
		g_sAncs.setPairState = PAIRING_DISABLE;

	EFM2BLE_IsANCS();
}
/*
 *��pair��ʹ�÷�����(�ر�Ҳ��һ����)
 *1.��ANCS״̬�����Ϊ�򿪣��Ǿ���ɣ�����Ϊ�����ߡ�
 *2.��ANCS
 *3.�������������
 *4.��λBLE.
*/

/* check ANCS enable or not */
void EFM2BLE_IsANCS(void)
{
	uint8_t u8buff[8];
	u8buff[0] = SBLE_ANCS_CONTROL;//0x05 ANCS message.
	u8buff[1] = 1; //Query
	LEUARTSentByDma(UART_CMD_INFOR, u8buff, 2);
	g_sAncs.crtPairState = PAIRING_IDL;
}

//����ANCS �����յ�״̬���غ�Ҫ��BLE��λ
void EFM2BLE_OpenANCS(bool enable)
{
	uint8_t u8buff[8];
	u8buff[0] = SBLE_ANCS_CONTROL; //0x05 ANCS message
	u8buff[1] = 2; //check

	u8buff[2] = enable; //state
	LEUARTSentByDma(UART_CMD_INFOR, u8buff, 3);
}
#endif

// The task has to pick up the ble data in 2ms ,otherwise the next ble will be lost
void ParseBleUartPak(void)
{
	uint8_t TempChar, LastCharWasEsc = 0, len = 0, start = 0, index;
	uint8_t u8buff[21];

	index = RpIndex;
	RpIndex++;
	RpIndex %= LOOPSIZE;

	for(int i = 0; i < RX_BUF_SIZE; i++)
	{
		TempChar = LEUARTRXBUFF[index][i];

		if(start == 0)
		{
			if(TempChar == UART_DATA_START)  //locate the start delimeter.
			{
				start = 1;
				i++; // skip the length position
#if BGXXX==7
				if (LEUARTRXBUFF[index][i+1]==UART_CMD_INFOR && LEUARTRXBUFF[index][i+2]==SBLE_STATE_UPDATE)
				{
					test1.typeuint8[0] = LEUARTRXBUFF[index][i];  //temporary store in [0]len
					test1.typeuint8[1] = LEUARTRXBUFF[index][i+1];  //temporary store in [1]cmd
					test1.typeuint8[2] = LEUARTRXBUFF[index][i+2];  //temporary store in [2]subcmd
				}
#endif
				len = 0;
			}

			continue;
		}

		if(TempChar == UART_DATA_STOP)
			break;

		/* Atus: As decoding schema, decode process before leading code check to avoid miss data is same as leading code. */
		if (TempChar == SOH_ESCAPE_CHAR)
		{
			LastCharWasEsc = 1;
			continue;
		}
		else  if (LastCharWasEsc)
		{
			TempChar ^= SOH_ESCAPE_CHAR_MASK;
			LastCharWasEsc = 0;
		}

		Rec_Ble_Pak[len++] = TempChar; //Rec_Ble_Pak store the data without UART_DATA_START/STOP and length.
	}

	//
	systemStatus.iBleHeartBeatCounter = 0;

    /* BLE command validation. length(len>0) and complete command pak(found STOP delimiter. */
    if (len<=0 || TempChar!=UART_DATA_STOP)  //[BG038] 
    {
      return;
    }
    
	switch(Rec_Ble_Pak[0])
	{
		case  UART_CMD_WRITE:
			if(Rec_Ble_Pak[1] == 1) //  find me
			{
				MESSAGE msg;
				msg.params.type = MESSAGE_FIND_ME;
				msg.params.param = Rec_Ble_Pak[2];
//				osMessagePut(hMsgInterrupt, msg.id, 0);
				xQueueSend(hEvtQueueDevice, &msg.id, 0);
			}

			break;

		case UART_CMD_ASK_DevInfo: //

			u8buff[0] = BLE_CH9;
			u8buff[1] = Dev_Name; // i4,i4S
			u8buff[2] = APP_FW_VER_M;
			u8buff[3] = APP_FW_VER_S;

			LEUARTSentByDma(UART_CMD_2HOST, u8buff, 4);

			break;

		case UART_CMD_UPDATA: // update the firmware

			break;

		case UART_CMD_ANCS: //��������������ANCS���е����ݡ�
			BLE2EFM32_ANCSHandle(&Rec_Ble_Pak[1], len);

			break;

		case UART_CMD_ACK:

			break;

		case UART_CMD_DATA_FROM_HOST: // Host data  �����ble�����ĵ�����ν�Ĵ�BLe-��MCu ��͸�����ݡ�

			ParseHostData(&Rec_Ble_Pak[1]);

			break;

		case UART_CMD_TEST: // send back
			break;

		case UART_CMD_INFOR: // ble info to mcu

			switch(Rec_Ble_Pak[1])
			{

				case SBLE_CONNECT_INTERVAL:
					getConnectTime = Rec_Ble_Pak[2];
					break;

				case SBLE_DEVINFO_QUERY:
					systemStatus.blBleOnline = true;
					memcpy(BLE_DevChip.BLE_DeviceInfo, &Rec_Ble_Pak[2], sizeof(BLE_DevChip.BLE_DeviceInfo));

					if(BLE_DevChip.BLE_Device.WORKSTA == BLE_APP)
					{
						BLE_APP_Model = true;
					}
					else
					{
						BLE_APP_Model = false;
					}

					break;

				case SBLE_HR_NOTIFY_SET:
					if(Rec_Ble_Pak[2] == 1) //i = HR���������BLE��enable heartrate notify �������������
					{
						if(Rec_Ble_Pak[3] == 1)
							En_HR_Read = true;
						else
							En_HR_Read = false;
					}

					break;

				case SBLE_STATE_UPDATE:
					systemStatus.blBleOnline = true;
					BLE_STATE = Rec_Ble_Pak[2];

					//UnLockScreen(true);

					if(BLE_STATE == BLE_STATE_CONNECTED)
					{
						systemStatus.blBluetoothConnected = true;
						BLE_Should_Status = BLE_STATE_CONNECTED;

						startFlashLed(true, 3, 2, 1, 1);
					}
					else
					{
						systemStatus.blBluetoothConnected = false;
						RealDataOnPhone = 0;

						if(Last_BLE_STATE == BLE_STATE_CONNECTED)
						{
							Check_ADV_Counter = FAST_ADV_TIME;
							BLE_ADVEN_CON(BLE_ON, BLE_ADVEN_100mS);
							BLE_Should_Status = BLE_STATE_ADVERTISING;
						}

					}

					Last_BLE_STATE = BLE_STATE;
					break;

				default:
					break;
			}

			break;

		default:
			break;
	}

}


//�ú���û�б����á�
void ANCS_Parsing(unsigned char* p)
{
	MESSAGE msg;

	BYTE eventId = p[0];
	NOTIFY_SERVICE ns = (NOTIFY_SERVICE) p[2];

	if(eventId == 0) // add
	{
		switch(ns)
		{
			case NOTIFY_SERVICE_IncomingCall:  // in call
			case NOTIFY_SERVICE_MissedCall: // miss call
			{
//				systemStatus.incomingCallNumber[0] = 0;
				memset(systemStatus.incomingCallNumber, 0, NOTIFY_SENDER_BUFFER_SIZE);
				memcpy(systemStatus.incomingCallNumber, p + 2, NOTIFY_SENDER_BUFFER_SIZE - 2);

				systemStatus.notifyEvents[ns] = 1;
//				osMessagePut(hMsgInterrupt, NOTIFICATION + (ns << 16), 0);

				msg.params.type = NOTIFICATION;
				msg.params.param = ns;
				xQueueSend(hEvtQueueDevice, &msg.id, 0);
				break;
			}

//				break;

			case NOTIFY_SERVICE_Voicemail: // voicemail

//				break;

			case NOTIFY_SERVICE_Social: // social

//				break;
			case NOTIFY_SERVICE_Email:
			case NOTIFY_SERVICE_News:
			{
				memset(systemStatus.latestSmsNumber, 0, 20);
				memcpy(systemStatus.latestSmsNumber, p + 3, 16);

//				osMessagePut(hMsgInterrupt, NOTIFICATION + (ns << 16), 0);
				msg.params.type = NOTIFICATION;
				msg.params.param = ns;
				xQueueSend(hEvtQueueDevice, &msg.id, 0);

				break;
			}

			default:
				break;
		}
	}
	else if (eventId == 2) //EventIDNotificationRemoved
	{
		if (ns == NOTIFY_SERVICE_IncomingCall)
		{
			systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] = 0;
//			osMessagePut(hMsgInterrupt, NOTIFICATION + (NOTIFY_SERVICE_IncomingCall << 16), 0);
			msg.params.type = NOTIFICATION;
			msg.params.param = NOTIFY_SERVICE_IncomingCall;
			xQueueSend(hEvtQueueDevice, &msg.id, 0);
		}
	}
}

void BLE_RESET(void)
{
#if (BOARD_TYPE==2)  //FIXLME: if not debug use, please remove it.
    if (GetDAUIO_1()==0x1)
    {
      GPIO_PinModeSet(BLE_RST_PORT, BLE_RST_PIN, gpioModeDisabled, 1); /* BLE_RESET set to Z-impedance */
      return;
    }
#endif
	GPIO_PinModeSet(BLE_RST_PORT, BLE_RST_PIN, gpioModePushPull, 1); /* BLE_RESET as output */
	BLE_RST_L();
	SysCtlDelay(8000 * SYSCLOCK); //
	BLE_RST_H();
}


void BLE_INIT(void)
{
	CMU_ClockEnable( cmuClock_GPIO, true );

	// will eat 2uA current
	GPIO_PinModeSet(BLE_32K_PORT, BLE_32K_PIN, gpioModePushPull, 0); /* BLE 32KHZ CLOCK */
	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_CLKOUTSEL1_MASK) | CMU_CTRL_CLKOUTSEL1_LFXO ;
	CMU->ROUTE = CMU_ROUTE_CLKOUT1PEN | CMU_ROUTE_LOCATION_LOC0;

	GPIO_PinModeSet(BLE_RST_PORT, BLE_RST_PIN, gpioModePushPull, 1); /* BLE_ RESET */
	BLE_RESET();

	LeuartConfig();
	SetupLeuartDma();
	DisableLeUart();
	systemStatus.blBleOnline = false;
	systemStatus.iBleHeartBeatCounter = 0;
	BLE_Should_Status = BLE_STATE_IDLE;
}

void DisableLeUart(void)
{
	if(BleLeUartSta == BLE_UART_Opening)
	{
		BleLeUartSta = BLE_UART_Closing;
		CMU_ClockEnable(cmuClock_LEUART0, false);
		GPIO_IntClear(1 << BLE_INT_PIN);
		GPIO_IntConfig(BLE_INT_PORT, BLE_INT_PIN, false, true, true);
		/* Enabling Interrupt from GPIO_EVEN */

		NVIC_SetPriority(GPIO_EVEN_IRQn, GPIO_EVEN_INT_LEVEL);
		NVIC_EnableIRQ(GPIO_EVEN_IRQn);
		DisableDelayTimer(TIMER_FLAG_LeUart);
	}

}


void EnableLeUart(void)
{
	if(BleLeUartSta == BLE_UART_Closing)
	{
		BleLeUartSta = BLE_UART_Opening;
		GPIO_IntConfig(BLE_INT_PORT, BLE_INT_PIN, false, true, false); // close int
		CMU_ClockEnable(cmuClock_LEUART0, true);
		Rdy2DmaRx();
		ReChargeTimeCount();
		EnableDelayTimer(TIMER_FLAG_LeUart, true, LeUartTxInterval, LEUARTCallback, NULL);
	}
}

/* send a query command to ble get the ble device information while ble on first time. */
void getBleDeviceInfo(void)
{
	uint8_t u8buff[2];
	u8buff[0] = SBLE_DEVINFO_QUERY;
	LEUARTSentByDma(UART_CMD_INFOR, u8buff, 1);
}

//�ú���˵����ֻ�������ϲŷ�����Щ���ݡ�
void SendData2Host(uint8_t* p, uint8_t len)
{
	if(BLE_STATE != BLE_STATE_CONNECTED)  // 2014.08.14
		return;

	LEUARTSentByDma(UART_CMD_2HOST, p, len);

}

void BLE_Update_Start(void)
{
	uint8_t u8buff[3];
	u8buff[0] = 0;
	u8buff[1] = 0;
	u8buff[2] = 0;
	LEUARTSentByDma(UART_CMD_UPDATA, u8buff, 3);
}


void  BLE_Update_End(uint16_t checksum)
{
	uint8_t u8buff[3];
	u8buff[0] = 0x01;
	u8buff[1] = checksum & 0xff;;
	u8buff[2] = checksum >> 8;
	LEUARTSentByDma(UART_CMD_UPDATA, u8buff, 3);

}


void BLE_ADVEN_CON(uint8_t onoff, uint8_t inteval)
{
	uint8_t u8buff[3];
	u8buff[0] = SBLE_ADVER_SET; //0x00 set advertise on/off and interval time.
	u8buff[1] = onoff;
	u8buff[2] = inteval;
	LEUARTSentByDma(UART_CMD_INFOR, u8buff, 3);
}


void BLE_SET_CONNECT_INTERVAL(uint8_t inteval)
{
	uint8_t u8buff[3];
	u8buff[0] = SBLE_CONNECT_INTERVAL; //0x04 set connect_interval.
	u8buff[1] = inteval;
	LEUARTSentByDma(UART_CMD_INFOR, u8buff, 2);
}


void BLE_BROADCAST_UPDATE(void)
{
#if (FALL_DETECT_SUPPORT || SOS_HIT_SUPPORT)
	static uint8_t data[6] = {0x0, 0xFF, 0xAA, 0xFF, 0xFF, 0xFF};

	data[0] = SBLE_BROADCAST_UPDATE;

    //FD tag
	if(SendAlertNotification == 1)
		data[1] = 0x01;
	else if(SendAlertNotification == 0xff)
		data[1] = 0xFF;
    //charging tag
	if(systemStatus.blBatteryCharging == true)
		data[2] = 0x55;
	else
		data[2] = 0xAA;
    //low battery tag
	if(lowBatteryLevelAlert == 0x01)
		data[3] = 0x01;
	else
		data[3] = 0xFF;
    //time reset
	if(blTimeReset == 0x01)
		data[4] = 0x01;
	else if(blTimeReset == 0xAA)
		data[4] = 0xFF;
    //SOS tag
	if(sosNotification == 1)
		data[5] = 0x01;
	else if(sosNotification == 0xff)
		data[5] = 0xFF;

	LEUARTSentByDma(UART_CMD_INFOR, data, 6);
#else
	static uint8_t data[4] = {0x0, 0xAA, 0xFF, 0xFF};

	data[0] = SBLE_BROADCAST_UPDATE;

	if(systemStatus.blBatteryCharging == true)
		data[1] = 0x55;
	else
		data[1] = 0xAA;

	if(lowBatteryLevelAlert == 0x01)
		data[2] = 0x01;
	else
		data[2] = 0xFF;

	if(blTimeReset == 0x01)
		data[3] = 0x01;
	else if(blTimeReset == 0xAA)
		data[3] = 0xFF;

	LEUARTSentByDma(UART_CMD_INFOR, data, 4);
#endif
}

void PermitUpdateBroadcast(void)
{
	static uint8_t resetTimeSendCount = 0;
#if (FALL_DETECT_SUPPORT || SOS_HIT_SUPPORT)

//��֧�ֹ㲥һ��ʱ��Ȼ���������ڰѵ����¼����ͳ�ȥ��
	if(((SendAlertNotification == 1) || (SendAlertNotification == 0xFF) || (isChargeStatusChange == 0x01)
	        || (lowBatteryLevelAlert == 0x01 || lowBatteryLevelAlert == 0xAA)
	        || (blTimeReset == 0x01 ||  blTimeReset == 0xAA)
	        || (sosNotification == 0x01) || (sosNotification == 0xFF))
	        && (BLE_STATE != BLE_STATE_CONNECTED) )
	{
		BLE_BROADCAST_UPDATE();

		//��������Ŀ����ֻ����������ʱ����һ�ι㲥���ݡ�
		if((SendAlertNotification == 1) || (SendAlertNotification == 0xFF))
			SendAlertNotification = 0;
		else if (isChargeStatusChange == 0x01)
			isChargeStatusChange = 0xFF;
		else if (lowBatteryLevelAlert == 0xAA)
			lowBatteryLevelAlert = 0xFF;
		else if((sosNotification == 1) || (sosNotification == 0xFF))
			sosNotification = 0;
		else if(blTimeReset == 0xAA)
		{
			resetTimeSendCount++;

			if(resetTimeSendCount > 2)
			{
				resetTimeSendCount = 0;
				blTimeReset = 0xFF;//��ֹ����
			}
		}
	}
#else

//��֧�ֹ㲥һ��ʱ��Ȼ���������ڰѵ����¼����ͳ�ȥ��
	if(((isChargeStatusChange == 0x01) || (lowBatteryLevelAlert == 0x01 || lowBatteryLevelAlert == 0xAA)
	        || (blTimeReset == 0x01 ||  blTimeReset == 0xAA))
	        && (BLE_STATE != BLE_STATE_CONNECTED) )
	{
		BLE_BROADCAST_UPDATE();

		//��������Ŀ����ֻ����������ʱ����һ�ι㲥���ݡ�
		if (isChargeStatusChange == 0x01)
			isChargeStatusChange = 0xFF;
		else if (lowBatteryLevelAlert == 0xAA)
			lowBatteryLevelAlert = 0xFF;
		else if(blTimeReset == 0xAA)
		{
			resetTimeSendCount++;

			if(resetTimeSendCount > 2)
			{
				resetTimeSendCount = 0;
				blTimeReset = 0xFF;//��ֹ����
			}
		}
	}
#endif
}

void BLE_Close(void)
{
	BLE_ADVEN_CON(BLE_OFF, 0);
	BLE_Should_Status = BLE_STATE_IDLE;
}

void BLE_Open(void)
{

	BLE_ADVEN_CON(BLE_ON, BLE_ADVEN_2S);
	vTaskDelay(50);

	BLE_Should_Status = BLE_STATE_ADVERTISING;
}

void isBLEError(void)
{

	static uint8_t BLEErrorCounte = 0;
	BLEErrorCounte++;

	if(BLE_Should_Status == BLE_STATE)
	{
		BLEErrorCounte = 0;
	}
	else if(BLEErrorCounte == 5) // 5seconds
	{
		BLE_Open();
	}
	else if(BLEErrorCounte >= 10) // 10seconds
	{
		BLE_INIT();
		BLEErrorCounte = 0;
	}

}



extern uint16_t  afe_sample_counter;    //Atus: uint32_t >> uint16_t
#ifndef  PPG2Dongle  // for phone
void SendRealDataOverBLE()
{
	uint8_t TEMP[21];
	static  uint8_t  index = 0;
	static  int16_t ppg_buff[2];
	static  uint16_t ecg_buff[2];
	static  int16_t xyz_buff[2][3];
	static  uint8_t Wait20msCONNECTED = 0;


	if(BLE_STATE != BLE_STATE_CONNECTED)
		return;

	if(BLE_CONNECT_20ms != getConnectTime)
	{
		Wait20msCONNECTED++;

		if(Wait20msCONNECTED < 16)
			return;
	}

	Wait20msCONNECTED = 0;
	extern int16_t PPG_OUT_DATA;
	ppg_buff[index] = PPG_OUT_DATA;
	ecg_buff[index] = (uint16_t)afe_sample_counter;

	extern int16_t AxieOUT[3];
	xyz_buff[index][0] = AxieOUT[0];
	xyz_buff[index][1] = AxieOUT[1];
	xyz_buff[index][2] = AxieOUT[2];

	index++;

	if(index >= 2)
	{
		index = 0;
		TEMP[0] = BLE_CH1;
		TEMP[1] = (uint8_t)ppg_buff[0];
		TEMP[2] = (uint8_t)(ppg_buff[0] >> 8);

		TEMP[3] = (uint8_t)ppg_buff[1];
		TEMP[4] = (uint8_t)(ppg_buff[1] >> 8);

		TEMP[5] = (uint8_t)ecg_buff[0];
		TEMP[6] = (uint8_t)(ecg_buff[0] >> 8);

		TEMP[7] = (uint8_t)ecg_buff[1];
		TEMP[8] = (uint8_t)(ecg_buff[1] >> 8);

		TEMP[9] = (uint8_t)xyz_buff[0][0];
		TEMP[10] = (uint8_t)(xyz_buff[0][0] >> 8); //x1

		TEMP[11] = (uint8_t)xyz_buff[1][0];
		TEMP[12] = (uint8_t)(xyz_buff[1][0] >> 8); //x2

		TEMP[13] = (uint8_t)xyz_buff[0][1];
		TEMP[14] = (uint8_t)(xyz_buff[0][1] >> 8); //y1

		TEMP[15] = (uint8_t)xyz_buff[1][1];
		TEMP[16] = (uint8_t)(xyz_buff[1][1] >> 8); //y2

		TEMP[17] = (uint8_t)xyz_buff[0][2];
		TEMP[18] = (uint8_t)(xyz_buff[0][2] >> 8); //z1

		TEMP[19] = (uint8_t)xyz_buff[1][2];
		TEMP[20] = (uint8_t)(xyz_buff[1][2] >> 8); //z2

		SendData2Host(TEMP, 21);

	}


	//TEST_H();
}

#else // for algorithm

void SendRealDataOverBLE()
{
	uint8_t TEMP[21];
	static  uint8_t  index = 0;
	static  int32_t ppg_buff[2];
	static  int16_t xyz_buff[2][3];



	if(BLE_STATE != BLE_STATE_CONNECTED)
		return;


	if(Current_Connect_Interval != getConnectTime)
		return;

	extern int32_t output_ppg_22bit;//��ֵ��AFE�ļ���
	ppg_buff[index] = output_ppg_22bit;

	extern int16_t AxieOUT[3];
	xyz_buff[index][0] = AxieOUT[0];
	xyz_buff[index][1] = AxieOUT[1];
	xyz_buff[index][2] = AxieOUT[2];

	index++;

	if(index >= 2)
	{
		index = 0;
		TEMP[0] = BLE_CH1;
		TEMP[1] = (uint8_t)ppg_buff[0];
		TEMP[2] = (uint8_t)(ppg_buff[0] >> 8);
		TEMP[3] = (uint8_t)(ppg_buff[0] >> 16);

		TEMP[4] = (uint8_t)ppg_buff[1];
		TEMP[5] = (uint8_t)(ppg_buff[1] >> 8);
		TEMP[6] = (uint8_t)(ppg_buff[1] >> 16);

		TEMP[7] = (uint8_t)afe_sample_counter;
		TEMP[8] = (uint8_t)(afe_sample_counter >> 8);

		TEMP[9] = (uint8_t)xyz_buff[0][0];
		TEMP[10] = (uint8_t)(xyz_buff[0][0] >> 8); //x1

		TEMP[11] = (uint8_t)xyz_buff[1][0];
		TEMP[12] = (uint8_t)(xyz_buff[1][0] >> 8); //x2

		TEMP[13] = (uint8_t)xyz_buff[0][1];
		TEMP[14] = (uint8_t)(xyz_buff[0][1] >> 8); //y1

		TEMP[15] = (uint8_t)xyz_buff[1][1];
		TEMP[16] = (uint8_t)(xyz_buff[1][1] >> 8); //y2

		TEMP[17] = (uint8_t)xyz_buff[0][2];
		TEMP[18] = (uint8_t)(xyz_buff[0][2] >> 8); //z1

		TEMP[19] = (uint8_t)xyz_buff[1][2];
		TEMP[20] = (uint8_t)(xyz_buff[1][2] >> 8); //z2

		SendData2Host(TEMP, 21);

	}
	else
	{
		TEMP[0] = BLE_CH1;
		TEMP[1] = 0x55;
		TEMP[2] = 0x66;
		TEMP[3] = 0x77;
		TEMP[4] = iHeartRate.component.heart;
		TEMP[5] = 0; // & 0xFF;

		extern uint8_t debug_info[];
		TEMP[6] = debug_info[0];
		TEMP[7] = debug_info[1];
		TEMP[8] = debug_info[2];
		TEMP[9] = debug_info[3];
		TEMP[10] = debug_info[4];

		SendData2Host(TEMP, 11);
	}

}


#endif

//������ݶ�ӦAPP���ϰ벿�ֵ����ݣ��¶ȣ��Ʋ�����·����룬UV�����ʡ� 2015��8��5��15:53:31
//����APP�����ʷ���ʱ��������Ҳ����������ݡ�
extern uint8_t heart_display;
void Send_1HZ_PacketOverBLE(void)
{
	extern uint8_t AMB_TMP[], SKIN_TMP[];

	uint8_t TEMP[21];

//	static uint8_t index = 0;

	if(BLE_STATE != BLE_STATE_CONNECTED)
		return;

	if(En_HR_Read == false)
		return;

	TEMP[0] = BLE_CH3;

	// �µ����ʸ�ʽ������ ble hr service ��׼

	// ���������ֽ�Ϊ��׼characteristic: Heart Rate Measurement
	// https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml
	TEMP[1] = 0; // 1byte heart rate

	if(systemStatus.blHeartBeatLock == false)
		TEMP[2] = 0xff;
	else
		//TEMP[2] = iHeartRate.component.heart;
		TEMP[2] = heart_display;

	// �����ֽ�Ϊ�Զ�����չ����
	TEMP[3] = AMB_TMP[0]; //ambient
	TEMP[4] = AMB_TMP[1]; //ambient

	TEMP[5] =  SKIN_TMP[0]; // skin
	TEMP[6] =  SKIN_TMP[1]; // skin

//	if(systemSetting.SystemMode == SYSTEM_MODE_MANUFACTORING)
//	{
//		if(systemStatus.blSkinTouched)
//		{
//			TEMP[7] = (uint8_t)LED_INTENSITY;
//			TEMP[8] = 0;
//			TEMP[9] = (uint8_t)AMB_uA * 10; //
//			TEMP[10] = 0;//
//		}
//		else
//		{
//			TEMP[7] = 0;
//			TEMP[8] = 0;
//			TEMP[9] = 0;//
//			TEMP[10] = 0;//
//		}
//	}
//	else
	{
		TEMP[7] = (BYTE) iSteps;// & 0xFF;
		TEMP[8] = (BYTE) (iSteps >> 8);// & 0xFF;

		TEMP[9] = (BYTE) iDistance;
		TEMP[10] = (BYTE) (iDistance >> 8);
	}

	TEMP[11] = bUltraViolet;

	TEMP[12] = systemStatus.bBatteryRemaining;

#if 0
	TEMP[13] = (uint8_t)((active_level - active_level_lastSaving) >> 8);
	TEMP[14] = (uint8_t)(active_level - active_level_lastSaving);

#else

	TEMP[13] = (BYTE) iCalories;
	TEMP[14] = (BYTE) (iCalories >> 8);

	//TEMP[15] = (uint8_t)(SkinTouchVal>>8);
	//TEMP[16] = (uint8_t)(SkinTouchVal);

	TEMP[15] = (uint8_t)(systemSetting.TOUCH_CAPVAL >> 8);
	TEMP[16] = (uint8_t)(systemSetting.TOUCH_CAPVAL);

	TEMP[17] = (uint8_t)(touchValues >> 8);
	TEMP[18] = (uint8_t)(touchValues);

	extern uint8_t AMB_uA;
	//TEMP[19] =  (uint8_t)(read_IR_reg>>8);
	//TEMP[20] =  (uint8_t)(read_IR_reg);

	TEMP[19] =  (uint8_t)AMB_uA;//0x55;
	TEMP[20] =  0x55;

	//TEMP[19] =  (uint8_t)LED_INTENSITY;//0x55;
	//TEMP[20] =  0x55;

	//if(GPIO_PinInGet(MEMS_INT1_PORT,MEMS_INT1_PIN)==0)
	//TEMP[16] = 0;

#endif

	SendData2Host(TEMP, 21);

}

//��ԭ����notification�Ĳ���ȥ��

#define SOS_ALERT_GOT         0x00
#define FALL_ALERT_GOT        0x01
#define STEP_ALERT_GOT        0x02
#define WEAR_ALERT_GOT        0x03
#define CHARGE_STATUS         0x04
#define LOW_POWER_STATUS      0x05

uint8_t perimitSend = 0;
uint8_t resendCountLowbattery = 0;
uint8_t resendCountCharging = 0;

//static volatile UINT steps = 0;

void SendNotificationAlert(void)
{
	uint8_t TEMP[21] = {0x0, 0xFF, 0xAA, 0xFF,0xFF};
	uint8_t index = 0;

	if(BLE_STATE != BLE_STATE_CONNECTED)
		return;

	//ͨ��
	TEMP[index++] = BLE_CHA;

#if 0
	static uint8_t oldlevel = 0;

	// 1byte heart rate alert level
	if((iHeartRate.component.heart > 100) && (iHeartRate.component.heart <= 110))
		notificationAlertParameters.heartRateLevel = HEART_ALERT_LEVEL0;

	if((iHeartRate.component.heart > 110) && (iHeartRate.component.heart <= 120))
		notificationAlertParameters.heartRateLevel = HEART_ALERT_LEVEL1;

	if((iHeartRate.component.heart > 120) && (iHeartRate.component.heart <= 130))
		notificationAlertParameters.heartRateLevel = HEART_ALERT_LEVEL2;

	if((iHeartRate.component.heart > 130) && (iHeartRate.component.heart <= 140))
		notificationAlertParameters.heartRateLevel = HEART_ALERT_LEVEL3;

	if((iHeartRate.component.heart > 140) && (iHeartRate.component.heart <= 150))
		notificationAlertParameters.heartRateLevel = HEART_ALERT_LEVEL4;

	if(iHeartRate.component.heart > 150 )
		notificationAlertParameters.heartRateLevel = HEART_ALERT_LEVEL5;

	index = 1;

	if(notificationAlertParameters.heartRateLevel != oldlevel)
	{
		//�����֣���ʵ������
		TEMP[index] = notificationAlertParameters.heartRateLevel;
		resendTimes.heartrateLevelResend++;
		perimitSend |= 0x01 << HEART_ALERT_GOT;

		if(resendTimes.heartrateLevelResend > 2)
		{
			oldlevel = notificationAlertParameters.heartRateLevel;
			resendTimes.heartrateLevelResend = 0;
			perimitSend &= ~(0x01 << HEART_ALERT_GOT);

		}
	}
	else
	{
		TEMP[index] = 0xFF;
	}
	index++;

	if(notificationAlertParameters.stepsCounterStatus == 0x01)
	{
		TEMP[index] = notificationAlertParameters.stepsCounterStatus;
		resendTimes.stepCounterStatusResend++;
		perimitSend |= 0x01 << STEP_ALERT_GOT;

		if(resendTimes.stepCounterStatusResend > 3)
		{
			notificationAlertParameters.stepsCounterStatus = 0;
			resendTimes.stepCounterStatusResend = 0;
			perimitSend &= ~(0x01 << STEP_ALERT_GOT);
		}
	}
	else
	{
		TEMP[index] = 0xFF;

	}
	index++;

	if(notificationAlertParameters.wearStatus == 0x01)
	{
		TEMP[index] = notificationAlertParameters.wearStatus;
		resendTimes.watchWearStatusResend++;
		perimitSend |= 0x01 << WEAR_ALERT_GOT;

		if(resendTimes.watchWearStatusResend > 2)
		{
			notificationAlertParameters.wearStatus = 0;
			resendTimes.watchWearStatusResend = 0;
			perimitSend &= ~(0x01 << WEAR_ALERT_GOT);
		}
	}
	else
	{
		TEMP[index] = 0xFF;
	}
	index++;
	
#endif


	//��call counter���ɵ���״̬
//	if(notificationAlertParameters.callsCounterStatus == 0x01)
//	{
//		TEMP[index] = notificationAlertParameters.callsCounterStatus;
#if (MODEL_TYPE==1) //HEALTHCARE_TYPE
	if(SendAlertNotification == 1)
	{
		TEMP[index] = 0x01;//����

//		resendTimes.callsCounterStatusresend++; //��������Ҫ��������5����,������ʱ��������Ҫ���Ͷ��ٴ�
		perimitSend |= 0x01 << FALL_ALERT_GOT;

//		if(resendTimes.callsCounterStatusresend > 2)
//		if(isSendAlertNotification == false)
//		{
		//	notificationAlertParameters.callsCounterStatus = 0;
		//	resendTimes.callsCounterStatusresend = 0;

//		}
	}
	else if (SendAlertNotification == 0xff)
	{
		TEMP[index] = 0xFF;
		resendTimes.fallCounterStatusresend++; //����⵽5���ӽ����ˣ��ڷ���һ�ε����������������ֵ����0xFF

		if(resendTimes.fallCounterStatusresend > 1)
		{
			resendTimes.fallCounterStatusresend = 0;
			perimitSend &= ~(0x01 << FALL_ALERT_GOT);
		}
	}
	index++;

//�����ڳ��״̬�����仯�Ƿ������״̬notification������3�Ρ�
	if(isChargeStatusChange == 0x01)
	{
		if(systemStatus.blBatteryCharging)
			TEMP[index] = 0x55; //�ڳ�硣
		else
			TEMP[index] = 0xAA; //û��硣

		resendTimes.chargeStatusResend++;
		perimitSend |= 0x01 << CHARGE_STATUS;

		if(resendTimes.chargeStatusResend > 2)
		{
			resendTimes.chargeStatusResend = 0;
			perimitSend &= ~(0x01 << CHARGE_STATUS);
			isChargeStatusChange = 0;
		}
	}
	index++;

	if(lowBatteryLevelAlert == 0x01)
	{
		TEMP[index] = 0x01;
		resendTimes.lowBatteryStatusResend++;
		perimitSend |= 0x01 << LOW_POWER_STATUS;

		if(resendTimes.lowBatteryStatusResend > 2)
		{
			lowBatteryLevelAlert = 0;
			resendTimes.lowBatteryStatusResend = 0;
			perimitSend &= ~(0x01 << LOW_POWER_STATUS);
		}
	}
	else
		TEMP[index] = 0xFF;
	index++;

	if(sosNotification == 0x01)
	{
		TEMP[index] = 0x01;//SOS
		perimitSend |= 0x01 << SOS_ALERT_GOT;
	}
	else if(sosNotification == 0xFF)
	{
		TEMP[index] = 0xFF;
		resendTimes.sosCounterStatusResend++;

		if(resendTimes.sosCounterStatusResend > 1)
		{
			resendTimes.sosCounterStatusResend = 0;
			perimitSend &= ~(0x01 << SOS_ALERT_GOT);
		}

	}
	index++;
#elif (MODEL_TYPE==2) //CONSUMER_TYPE
	if(lowBatteryLevelAlert == 0x01)
	{
		TEMP[index] = 0x01;
		resendTimes.lowBatteryStatusResend++;
		perimitSend |= 0x01 << LOW_POWER_STATUS;

		if(resendTimes.lowBatteryStatusResend > 3)
		{
			lowBatteryLevelAlert = 0;
			resendTimes.lowBatteryStatusResend = 0;
			perimitSend &= ~(0x01 << LOW_POWER_STATUS);
		}
	}
	else
		TEMP[index] = 0xFF;
	index++;

	if(isChargeStatusChange == 0x01)
	{
		if(systemStatus.blBatteryCharging)
			TEMP[index] = 0x55; //�ڳ�硣
		else
			TEMP[index] = 0xAA; //û��硣

		resendTimes.chargeStatusResend++;
		perimitSend |= 0x01 << CHARGE_STATUS;

		if(resendTimes.chargeStatusResend > 3)
		{
			resendTimes.chargeStatusResend = 0;
			perimitSend &= ~(0x01 << CHARGE_STATUS);
			isChargeStatusChange = 0;
		}
	}
	index++;


#endif
	
	
	TEMP[index++] = 0xAA;
	TEMP[index++] = 0xAA;

	if((perimitSend & 0xFF) != 0)//ֻҪ�յ����֪ͨ��������
		SendData2Host(TEMP, index);

}

#if 0
void StepsDuringOneHour()
{
	static volatile bool firstCounter = true;
	static time_t oldTime = 0;
	time_t currenttime = 0;

	if(firstCounter == true)
	{
		steps = iSteps;
		oldTime = time(NULL);
		firstCounter = false;
	}

	currenttime = time(NULL);

	if((currenttime - oldTime) >= ONE_HOUR_SECONDS)
	{
		if(iSteps - steps <= 50)
		{
			notificationAlertParameters.stepsCounterStatus = 0x01;
//			checkOk = 0xAF;
		}

		steps = iSteps;
		oldTime = currenttime;
	}
}

#endif

#if (FALL_DETECT_SUPPORT==1)
// Check FallDetect flag, then set 0x01 value to update the broadcast and notify.
// After duration, set 0xFF and update broadcast and notify again.
void CheckFall(void)
{
	extern uint8_t FD_result; //initialize 0
	static bool fallDetectedTimeCount = true;
	static bool fallFlashOledTimeCount = true;
	static time_t fdStartTime = 0;
	static time_t startTimeFlashOled = 0;
	time_t fdCurrTime = 0;
	time_t flashOledanyTime = 0;
	extern uint8_t oldFallStatus; //[BG033] static change to global and move to mems_track.h
	uint8_t currentFallStatus = FD_result;

#ifdef DEBUG0
	//simulate a falling
	static uint8_t fallCount = 0;
	fallCount++;

	if(fallCount == 30)
	{
		currentFallStatus = 1;
		fallCount = 0;
	}

#endif

	if(oldFallStatus != currentFallStatus)
	{ // get a falling
		oldFallStatus = currentFallStatus;
		isDetectedFall = true;
		SendAlertNotification = 1;

#if BGXXX==0 //not debug
		/* set display flash duration 10 seconds */
		blAlertMenu = true;
		UnLockScreen(false);
#if (OLED_SUPPORT==1)
		OLEDON(); // turn on oled display
#endif
		NoTOUCHPressCount = 10;//keep 10 seconds flash screeen w/o button press.

		ForceShowMenu(1, MENU_TYPE_FALL_ALERT);
#endif

		if(fallFlashOledTimeCount)
		{
			fallFlashOledTimeCount = false; //lock
			startTimeFlashOled = time(NULL); //get the start timestamp.
		}

		if(fallDetectedTimeCount)
		{
			fallDetectedTimeCount = false;
			fdStartTime = time(NULL);
#if (BOARD_TYPE==2)
            LEDB_ON();
#endif
		}
	}

	if(isDetectedFall == true) //get the fall flag, start counting time.
	{
		fdCurrTime = time(NULL);
		flashOledanyTime = time(NULL);
	}

	if(flashOledanyTime - startTimeFlashOled >= 12)
	{
		fallFlashOledTimeCount = true; //clean flag for next trigger.
		blAlertMenu = false;

#if BGXXX==0
		//force jump to Time menu
		if(IsForcedShowMenu())
		{
			UnforceShowMenu();

			JumpToMenu(MENU_TYPE_Time);
//			fireDisplayEvent(EVT_TYPE_KEY_UNLOCKED, 0);

		}
#endif
	}

	if(fdCurrTime - fdStartTime >= EVENT_FD_DURATION)
	{ //over duration, reset to non fd envent.
		fallDetectedTimeCount = true; //clean timeflag for next.
		isDetectedFall = false; //clean while 5 minutes, terminate��
		SendAlertNotification = 0xFF;
#if (BOARD_TYPE==2)
        LEDB_OFF();
#endif
	}
}
#endif

#if SOS_HIT_SUPPORT
void CheckSOS(void)
{
	extern uint8_t SOS_result;
	static uint8_t oldSOSStatus = 0x00;//δ������
	uint8_t currentSOSStatus = SOS_result;
	static bool sosDetectedTimeCount = true;
	static time_t sosStartTime = 0;
	time_t sosCurrTime = 0;

#ifdef DEBUG0
	//ģ��һ��sos
	static uint8_t tempCount = 0;
	tempCount++;

	if(tempCount == 30)
	{
		currentSOSStatus = 1;
		tempCount = 0;
	}

#endif

	if(oldSOSStatus != currentSOSStatus)
	{
		oldSOSStatus = currentSOSStatus;
		isSOSDetected = true;

		if(sosDetectedTimeCount)
		{
			sosNotification = 0x01;
			sosDetectedTimeCount = false;
			sosStartTime = time(NULL);
#if (BOARD_TYPE==2)
            LEDR_ON();
#endif
		}
	}

	if(isSOSDetected)
	{
		sosCurrTime = time(NULL);
	}

	if(sosCurrTime - sosStartTime > EVENT_SOS_DURATION)
	{
		sosDetectedTimeCount = true;
		isSOSDetected = false;
		sosNotification = 0xFF;
#if (BOARD_TYPE==2)
        LEDR_OFF();
#endif
	}
}
#endif

uint16_t GetTotalChunks(uint16_t currentScanSector, uint16_t currentUploadSectors)
{
	uint16_t tempScanSector = 0;
	uint16_t tempCount = 1; //����1��ʾ�ú����ĵ��ñ�ʾ�Ѿ��ҵ�һ����Ч��sector������Ҫ�����ܵ���Чsector�Ļ�Ҫ��1��ʼɨ�衣
	bool upScan = true;
	FLASH_SECTOR_HEAD fsh;
	ReturnType rt;
	tempScanSector = currentScanSector; //The physical sector we scan.
	long addr = 0;

	while(true)
	{
		if(upScan)
		{
			//
			tempScanSector++;

			if(tempScanSector > pFlashInfo->sectors - 1 )
			{
				upScan = false; //The forward scan finish, will scan from first sector.
			}
		}

		if(upScan == false)
		{
		  	//scan from first sector.
			if(first)
			{
				first = false;
				tempScanSector = INDEX_DATA_START_SECTOR - 1; //�����мӣ����������1.���ܷŹ���һ�顣
			}

			tempScanSector++;

			if( tempScanSector == currentScanSector - 1)
			{
				return tempCount;//������flashɨ�����˿���ֱ���˳���
			}
		}

		addr = tempScanSector * pFlashInfo->sectorSize;
		rt = FlashRead(addr, (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD));

		if(fsh.tag != FLASH_SECTOR_TAG)//sector û�б���ʼ��������һ��sector
			continue;
		else if(rt != Flash_Success)
			continue;
		else if(fsh.startTimestamp == 0 || fsh.endTimestamp == 0)
		{
			tempScanSector++;//����ǿտ飬������һ���顣
			continue;
		}
		else
		{
			if((fsh.index < currentUploadSectors) || (fsh.startTimestamp == DEFAULT_TIMESTAMP || fsh.endTimestamp == DEFAULT_TIMESTAMP))
				return tempCount;
			else
			{
				//����sector������Ч���ݣ��ֻ��˵�chunk������ߵ�sector.
				tempCount++;
			}
		}
	}
}
