#include "em_gpio.h"
#include "em_cmu.h"
#include "em_usart.h"
#include "BLE.h"
#include "main.h"
#include "em_timer.h"
#include "em_emu.h"
#include "common_vars.h"
#include "em_int.h"
#include "dma.h"
#include "crc.h"
#include "em_leuart.h"
#include "delayusingtimerint.h"


#define DMA_CH_RX    1 //DMAͨ��0
#define DMA_CH_TX    2 //DMAͨ��1
//frame format
#define FRAME_START_POSITION 0
#define FRAME_LEN_POSITION 1
#define FRAME_CMD_POSITION 2


//#define SYSCLOCK           14   //�����ݱ�ʾϵͳʱ��14M
//uint8_t BLE_STATE = BLE_STATE_IDLE;

volatile bool BLE_ONLINE = false, BLE_APP_Model = true;
union _BLE_CHIP BLE_DevChip;//������Ϣ
uint8_t insert_zero_number = 16;

DMA_CB_TypeDef TX_DAM_CALLBACK ;//�ص��ṹ�嶨��
//DMA_CB_TypeDef RX_DAM_CALLBACK ;

char LeUartRxBuff[RX_BUF_SIZE];
char CopyRxBuff[RX_BUF_SIZE];

char LeUartTxBuff[RX_BUF_SIZE * 2 + 32]; //128+8
//ע�����������Ҫ4�ֽڶ��롣
volatile bool TxDone = true;


volatile bool BleLeUartSta = BLE_UART_Closing;

volatile bool BLE_Responsed = false;
//volatile bool crccheck = false;

void SendData2Host(uint8_t* p, uint8_t len);
void Rdy2DmaRx(void);
//void getBleDeviceInfo(void);
void DisableLeUart(void);
void EnableLeUart(void);
void WaitLastTxDone(void);


uint16_t LeUartWorkTimeCount = 0, LeUartTxCount = 0; //

#define LeUartRxAllowWaitTime 10 // delaytime unit is 20ms , 10*20=200
#define LeUartTxInterval 4 //4*20=80ms,  delaytime unit is 20ms ,decide how many 0x00 header data to active CC254x


__STATIC_INLINE void ReChargeTimeCount(void)
{
	INT_Disable();
	LeUartWorkTimeCount = LeUartRxAllowWaitTime;
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
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_CORELEDIV2);//���ں�ʱ�ӵ�һ�롣ʹ���ں�ʱ�ӣ����õ�ȷ���Ĳ����ʵ�������
	//ʹ��LFXOʱ��ʱ���õĲ�һ����

	CMU_ClockEnable(cmuClock_CORELE, true);
	CMU_ClockEnable(cmuClock_LEUART0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	LEUART_Reset(LEUART0);
	LEUART_Init(LEUART0, &tLeuartInit);//�����ʼ��LEUART


	LEUART0->ROUTE = LEUART_ROUTE_TXPEN |
	                 LEUART_ROUTE_RXPEN |
	                 LEUART_ROUTE_LOCATION_LOC4;

	GPIO_PinModeSet(BLE_TX_PORT, BLE_TX_PIN, gpioModePushPull,  1);
	GPIO_PinModeSet(BLE_RX_PORT, BLE_RX_PIN, gpioModeInputPull, 1);

	LEUART0->STARTFRAME = UART_DATA_START;//��ʼ֡�Ĵ�����0x3c  '<'
	LEUART0->SIGFRAME = UART_DATA_STOP;//�ź�֡�Ĵ�����0x3e   ��>��
	/*
	��leuart���յ��붨�����ʼ֡ƥ���֡��STARTF�жϱ�־λ����λ�����SFUBRXλ����λ����ôRXBLOCK
	���ᱻ�����ͬʱ��ʼ֡�����ص�RXbuffer
	��leuart���յ��붨���SIGFRAMEһ����֡��SIGF�жϱ�־λ����λ��
	*/
	//LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF|LEUART_IEN_STARTF);
	LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF); // only the stop char int���ź�֡�ж�ʹ��

	NVIC_SetPriority(LEUART0_IRQn, LEUART0_INT_LEVEL);

	NVIC_EnableIRQ(LEUART0_IRQn);
	//������жϱ�־λ��Ϣ��efm32wg_leuart.h���ж��塣��Ӧ�ļĴ�����LEUART_IEN.
}

/**************************************************************************//**
* @brief  Setup Low Energy UART with DMA operation
*
* The LEUART/DMA interaction is defined, and the DMA, channel and descriptor
* is initialized. Then DMA starts to wait for and receive data, and the LEUART1
* is set up to generate an interrupt when it receives the defined signal frame.
�����յ�������ź�֡�󣬲����жϡ�
* The signal frame is set to '\r', which is the "carriage return" symbol.
*
*****************************************************************************/
/*----------------------------------------------------------------------------
* DMA1����˵Ļص����������ͣ�
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
	//����������2��ͨ����2����������

	/* Setting up channel */
	DMA_CfgChannel_TypeDef chnl0Cfg =
	{
		.highPri   = false,                     /* Normal priority */
		.enableInt = false,                     /* No interupt enabled for callback functions */
		.select    = DMAREQ_LEUART0_RXDATAV,    /* Set LEUART1 RX data avalible as source of DMA signals ��LEUART1���յ����ݿ���DMA��Դ*/
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
		.select    = DMAREQ_LEUART0_TXBL,    /* Set LEUART1 TX data avalible as source of DMA signals,�����TX�е�������ΪDMA���ź�Դ */
		.cb        = &TX_DAM_CALLBACK,                      /* No callback funtion */
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

	//���������˽��պͷ���ͨ��
	DMA_CfgChannel(DMA_CH_RX, &chnl0Cfg);
	DMA_CfgDescr(DMA_CH_RX, true, &descr0Cfg);
	DMA_CfgChannel(DMA_CH_TX, &chnl1Cfg);
	DMA_CfgDescr(DMA_CH_TX, true, &descr1Cfg);


	TX_DAM_CALLBACK.cbFunc  = BleTxDMADone;//���ûص�������
	TX_DAM_CALLBACK.userPtr = NULL;

	Rdy2DmaRx();//��LEUART���յ������ݰ��Ƶ�leUartRXBuff[].��
}
//��LEUART0�н��յ����ݰ��Ƶ�RxBuff���൱���ڴ棩��LEUART0�е������������͸�efm32�ġ�
void Rdy2DmaRx(void)
{
	/* Starting the transfer. Using Basic Mode */
	DMA_ActivateBasic(DMA_CH_RX,                /* Activate channel selected */
	                  true,                       /* Use primary descriptor */
	                  false,                      /* No DMA burst */
	                  (void*) &LeUartRxBuff,            /* Destination address */ //Ŀ�ĵ�ַ
	                  (void*) &LEUART0->RXDATA,  /* Source address*/ //Դ��ַ
	                  RX_BUF_SIZE - 1);               /* Size of buffer minus1 */
}





uint8_t bleStatusFlag = 0x55;
char tempESC[RX_BUF_SIZE] = {0};
uint8_t g_ucCurrentAddr = 0; //the pointer is just point to location of current operation.
uint8_t g_ucUartFrame[1024];//the global buffer is used for containing one unbroken frame.
uint8_t  start_add = 0;

void LEUART0_IRQHandler(void)
{
	uint32_t GucLeuartIF;
	uint8_t i = 0;
	uint8_t k = 0;
	uint8_t j = 0;
	uint8_t  end_add = 0;


	GucLeuartIF = LEUART_IntGet(LEUART0);
	LEUART_IntClear(LEUART0, GucLeuartIF);
	ReChargeTimeCount();

	if(GucLeuartIF & LEUART_IF_SIGF) //get a frame now.
	{
		BLE_ONLINE = true;

		//  NewFrame: extract it into g_ucUartFrame[]
		if(0 == g_ucCurrentAddr )
		{
			for(i = 0; i < RX_BUF_SIZE - 4; i++)
			{
				if(LeUartRxBuff[i] == UART_DATA_START)
				{
					start_add = i;

					for(k = start_add + 1; k < RX_BUF_SIZE - 4; k++)
					{
						if(LeUartRxBuff[k] == UART_DATA_STOP)
						{
							end_add = k;
							break;
						}
					}//find 0x3e"

					memcpy(g_ucUartFrame, &LeUartRxBuff[start_add], (k - start_add + 1));
					g_ucCurrentAddr = (k - start_add + 1);
					break;//�ҵ���һ��3cӦ�þ���֡ͷ���������治Ҫ����������֡ͷ��ֱ��������
				}
			}
		}
		else//the frame contains 0x3e, was divided two interrupt dealing with. the remaining of the frame was dealed with here
		{
			uint32_t uiLen = 0;

			for(k = start_add + g_ucCurrentAddr, uiLen = 0; k < RX_BUF_SIZE - 4; k++, uiLen++)
			{
				if(LeUartRxBuff[k] == UART_DATA_STOP)
				{
					memcpy(&g_ucUartFrame[g_ucCurrentAddr], &LeUartRxBuff[start_add + g_ucCurrentAddr], (uiLen + 1));
					g_ucCurrentAddr += (uiLen + 1);
					break;
				}
			}//find 0x3e"
		}

		// frame deal with
		{
			if(g_ucUartFrame[1] <= g_ucCurrentAddr) //one frame is received �������յ������ݳ��ȹ��ˣ��п��ܲ�������֡
			{
				if(0x3c == g_ucUartFrame[FRAME_START_POSITION] && (0x3e == (g_ucUartFrame[g_ucUartFrame[FRAME_LEN_POSITION] - 1]))) //�յ�����֡������
				{
					if((g_ucUartFrame[FRAME_CMD_POSITION] == 0x01) && (g_ucUartFrame[FRAME_CMD_POSITION + 1] == 0x01)) // the Ble is locate at app and boot state
					{
						if((g_ucUartFrame[g_ucUartFrame[FRAME_LEN_POSITION] - 2] == 0x00) && (g_ucUartFrame[g_ucUartFrame[FRAME_LEN_POSITION] - 3] == 0x00) &&  (g_ucUartFrame[g_ucUartFrame[FRAME_LEN_POSITION] - 4] == 0x00))
						{
							//boot   when there are three 0 byte in bottom of the frame before the last byte.e,g:"00 00 00 3E"
							memcpy(BLE_DevChip.BLE_DeviceInfo, &g_ucUartFrame[UART_ID_DATA + 1], sizeof(BLE_DevChip.BLE_DeviceInfo));
							bleStatusFlag = 2;
						}
						else if(g_ucUartFrame[g_ucUartFrame[FRAME_LEN_POSITION] - 2] == 0x01)
						{
							//app
							k = 0;

							for(j = 2; j < g_ucUartFrame[FRAME_LEN_POSITION] - 1; j++)
							{
								tempESC[k] = g_ucUartFrame[j];

								if((g_ucUartFrame[j] == 0x1B))
								{
									tempESC[k] = g_ucUartFrame[j + 1] ^ ESC;
									j += 1;
								}

								k++;
							}

							memcpy(BLE_DevChip.BLE_DeviceInfo, &tempESC[UART_ID_DATA - 1], sizeof(BLE_DevChip.BLE_DeviceInfo));
							bleStatusFlag = 1;
						}
					}
					else if(g_ucUartFrame[FRAME_CMD_POSITION] == 0x05)
					{
						BLE_Responsed = true;  //most of send BLE command will check this response flag.
					}

					memset(g_ucUartFrame, 0, g_ucCurrentAddr);
					g_ucCurrentAddr = 0; //��2����Ҫ�������ϵĻ���������ʽ��֡�����ݲ�����3E������һ��֮���´ξ���Ҳ�����˸��ж��������յĳ���
				}//end if ����֡
				else //��������֡����֡������
				{
					memset(g_ucUartFrame, 0, g_ucCurrentAddr);
					g_ucCurrentAddr = 0;
				}

				memset(LeUartRxBuff, 0, sizeof(LeUartRxBuff));
				DMA_ActivateBasic(DMA_CH_RX,				/* Activate channel selected */
				                  true,					   /* Use primary descriptor */
				                  false,					   /* No DMA burst */
				                  (void*) &LeUartRxBuff,			   /* Destination address */
				                  (void*) &LEUART0->RXDATA,  /* Source address*/ //�����LEUART0�����ݰ��Ƶ��ڴ�RxBuff��
				                  RX_BUF_SIZE - 1);
			}//end if �������յ������ݳ��ȹ��ˣ��п��ܲ�������֡
		}
	}
}



// 100ms at most to wait
void WaitLastTxDone(void)
{
	while(TxDone == false) //txDone=1,
	{
		EMU_EnterEM1();//DMA Not Done

		//	INT_Disable();
		if(LeUartWorkTimeCount == 0) //ReChargeTimeCount()������LeUartWorkTimeCount���и�ֵ��
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

    /* The process insert_zero_number=16, because BLE in App mode, need wake up by prefix 16 0x00, then send cmd. */

	for(int i = 0; i < insert_zero_number; i++)
	{
		LeUartTxBuff[i] = 0;   //���ͻ������Ѿ��洢��16��0
	}

	LeUartTxBuff[insert_zero_number + 0] = UART_DATA_START; //���ͻ���ǰ���16��0�����ŷ�����ʼ�Լ���������
	LeUartTxBuff[insert_zero_number + 1] = len + 3 + 2 + 1;
	LeUartTxBuff[insert_zero_number + 2] = comm_type;
	memcpy(&LeUartTxBuff[insert_zero_number + 3], p, len); //��p�ڴ濪ʼ��λ���𿽱�len���ֽڣ���LeUartTxBuff[insert_zero_number+3]ָ��ĺ���

	LeUartTxBuff[insert_zero_number + 3 + len + 0] = 0; //����LeUartTxBuff��������ԭ���Ļ���������len�ֽڣ������2��0�ͽ�����־
	LeUartTxBuff[insert_zero_number + 3 + len + 1] = 0;
	LeUartTxBuff[insert_zero_number + 3 + len + 2] = UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone = false; //����ʹ����DMA�Ļ���ģʽ��
	DMA_ActivateBasic(DMA_CH_TX,                                         /*DMAͨ��1������       */
	                  true,
	                  false,
	                  (void*)(uint32_t) & LEUART0->TXDATA,
	                  (void*)(uint32_t) LeUartTxBuff,
	                  len - 1 + insert_zero_number + 2 + 4); // 2= 2 0x00 at end ,  4 = < len ,type ,>
}


void MyLEUARTSentByDma(uint8_t comm_type, uint8_t* p , uint8_t len)
{

	if(BleLeUartSta == BLE_UART_Closing)
	{
		EnableLeUart();
	}

	WaitLastTxDone();


	for(int i = 0; i < 6; i++)
	{
		LeUartTxBuff[i] = 0;
	}

	LeUartTxBuff[6 + 0] = UART_DATA_START;
	LeUartTxBuff[6 + 1] = len + 3 + 2 + 1;
	LeUartTxBuff[6 + 2] = comm_type;
	memcpy(&LeUartTxBuff[6 + 3], p, len);

	LeUartTxBuff[6 + 3 + len + 0] = 0;
	LeUartTxBuff[6 + 3 + len + 1] = 0;
	LeUartTxBuff[6 + 3 + len + 2] = UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone = false;
	DMA_ActivateBasic(DMA_CH_TX,                                         /*DMAͨ��1������       */
	                  true,
	                  false,
	                  (void*)(uint32_t) & LEUART0->TXDATA,
	                  (void*)(uint32_t) LeUartTxBuff,
	                  len - 1 + 6 + 2 + 4); // 2= 2 0x00 at end ,  4 = < len ,type ,>

}

void BLE_INIT(void)
{
	CMU_ClockEnable( cmuClock_GPIO, true );

    /* turn on MCU-BLE CLK_32K clock output. */
	// will eat 2uA current
	GPIO_PinModeSet(BLE_32K_PORT, BLE_32K_PIN, gpioModePushPull, 0); //set PA1 iomode, the cmd could be skip.
	CMU->CTRL = (CMU->CTRL & ~_CMU_CTRL_CLKOUTSEL1_MASK) | CMU_CTRL_CLKOUTSEL1_LFXO; /* BLE 32KHZ CLOCK   PA1/CMU_CLK1 */
	CMU->ROUTE = CMU_ROUTE_CLKOUT1PEN | CMU_ROUTE_LOCATION_LOC0;

    /* reset BLE chip */
	GPIO_PinModeSet(BLE_RST_PORT, BLE_RST_PIN, gpioModePushPull, 1); /* BLE_RESET PF7*/
	BLE_RST_L();
	SysCtlDelay(8000 * SYSCLOCK);
	BLE_RST_H();

    /* config MCU-BLE interface Leuart and dma channel */
	LeuartConfig();
	SetupLeuartDma(); //setup DMA for LEUART
	EnableLeUart(); //enable LeUart
	BLE_ONLINE = false;  //just init interface not ble ready, it will set in LEUART0_IRQHandler() get frame.

}


void EnableLeUart(void)
{
	GPIO_IntConfig(BLE_INT_PORT, BLE_INT_PIN, false, true, false); // close int
	CMU_ClockEnable(cmuClock_LEUART0, true);
	Rdy2DmaRx();//ʹ�û�����ģʽ��ʼ����
	BleLeUartSta = BLE_UART_Opening; //��BLE

}

//set cmd to make device(BLE) change state from App to boot. The command prefix 16 0x00.
void AppToBoot(void)
{
	uint8_t COMM_DEV_INFO[3];
	COMM_DEV_INFO[0] = 0;
	COMM_DEV_INFO[1] = 0;
	COMM_DEV_INFO[2] = 0;
	LEUARTSentByDma(UART_CMD_UPDATA, COMM_DEV_INFO, 3);
}


//Add 6 0x00 before upgrade data.
void MyBLE_Update_Start(void)
{
	uint8_t COMM_DEV_INFO[3];
	COMM_DEV_INFO[0] = 0;
	COMM_DEV_INFO[1] = 0;
	COMM_DEV_INFO[2] = 0;
	MyLEUARTSentByDma(UART_CMD_UPDATA, COMM_DEV_INFO, 3);

}

//send the CRC checksum at the tail of upgrade process.
void  BLE_Update_End(uint16_t checksum)
{
	uint8_t COMM_DEV_INFO[3];

	COMM_DEV_INFO[0] = 0x01;
	COMM_DEV_INFO[1] = checksum & 0xff;
	COMM_DEV_INFO[2] = checksum >> 8;
	MyLEUARTSentByDma(UART_CMD_UPDATA, COMM_DEV_INFO, 3);

}



void WriteCC254xFlash(uint8_t* p)
{
	if(BleLeUartSta == BLE_UART_Closing) //Should be keep BLE_UART_Opening
	{
		EnableLeUart();
	}

	WaitLastTxDone();

	while(TxDone == false) //
	{
		EMU_EnterEM1();//DMA Not Done
	};


	for(int i = 0; i < 6; i++)
		LeUartTxBuff[i] = 0;

	LeUartTxBuff[6 + 0] = UART_DATA_START;
	LeUartTxBuff[6 + 1] = 128 + 3 + 2 + 1; //len
	LeUartTxBuff[6 + 2] = UART_CMD_UPGRADE_DATA;
	memcpy(&LeUartTxBuff[6 + 3], p, 128);

	LeUartTxBuff[6 + 3 + 128 + 0] = 0;
	LeUartTxBuff[6 + 3 + 128 + 1] = 0;
	LeUartTxBuff[6 + 3 + 128 + 2] = UART_DATA_STOP;

	ReChargeTimeCount();

	TxDone = false;
	DMA_ActivateBasic(DMA_CH_TX,           /*DMAͨ��1������       */
	                  true,
	                  false,
	                  (void*)(uint32_t) & LEUART0->TXDATA,
	                  (void*)(uint32_t) LeUartTxBuff,
	                  128 - 1 + 6 + 2 + 4); // 6=ǰ���0�ĸ�����2= 2 0x00 at end ,  4 = < len ,type ,>������ʼ�����ȣ����ͣ�������4���ֽ�
	//-1˵����ע�ⷲ���漰������ʱ������ĩβ��ַ������Ԫ�ظ�����1.
}




#if 0
//2015��10��5��09:17:02 ��������ȥ����

//UART_CMD_INFOR������Ҫ�鿴��������Ϣ��
void getBleDeviceInfo(void)
{
	uint8_t COMM_DEV_INFO[2];
	COMM_DEV_INFO[0] = UART_TYPE_DEV; //������Ըĳ�DEV��INFO
	MyLEUARTSentByDma(UART_CMD_INFOR, COMM_DEV_INFO, 1); //@p1,ͨ�����ͣ�@p2,���ݣ�@p3,����
}

#endif