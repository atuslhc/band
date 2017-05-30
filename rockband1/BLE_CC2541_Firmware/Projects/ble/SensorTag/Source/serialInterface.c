#include "hal_uart.h"
#include "serialInterface.h"
//#include "BLE_Bridge.h"
#include "bcomdef.h"
#include "hal_flash.h"
#include "gap.h"
#include "peripheral.h"
#include "string.h"
#include "sensorTag.h"
#include "battservice.h"
#include "att.h"
#include "gatt.h"
#include "myalertnotification.h"

__no_init XDATA uint8 CHIP_INFO0@0x624A;
__no_init XDATA uint8 CHIP_INFO1@0x6276;
extern __no_init XDATA uint8 mac_buf[6];
extern gaprole_States_t gapProfileState;
extern uint16 gapConnHandle;
//local function
static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void S2S_NotificationStat(uint8 ucCmd, uint8 enbale);

uint8 serialInterface_TaskID;   // Task ID for internal task/event processing

#define SERIAL_MSG_START_ID        0xAB //indicate the start of serial message
#define SERIAL_ACK                 0xA5 //inidicate an ACK
#define SERIAL_DATA                0xAD //indiciate data from central device

uint8 serialBuffer[RX_BUFF_SIZE];   //use as a circular ring.
uint16 serialBufferOffset = 0;

//static serial_state_t  serialRxState = SERIAL_STATE_LEN;
uint8 packet_length = 0;

//static uint8 temp_buf[48];
uint8 DeviceInfo[16];

#if (NOTI_PATCH==1)
#define sendDataBufSiz         6
#define sendDataId              0
#define sendDataLen             1
#define sendDataData            2
uint8 sendDataBuf[sendDataBufSiz][24]; //[i][0]:ID, [i][1]:len, [i][2]...[21] data.
#endif

stNotiFlag_Type g_stNotiFlag =
{
	.heartRate = FALSE,
	.temperature = FALSE,
	.running = FALSE,
	.heartRateAlert = TRUE, //FALSE, //new add for alert include SOS
};

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;
  NPI_InitTransport(cSerialPacketParser);
  DeviceInfo[UART_ID_START_POS] = UART_DATA_START;
  DeviceInfo[UART_ID_LEN_POS] = 16;
  DeviceInfo[UART_ID_CMD_POS] = UART_CMD_INFOR;
  DeviceInfo[UART_ID_DATA_POS + 0] = UART_TYPE_DEV;
  DeviceInfo[UART_ID_DATA_POS + 1] = CHIP_INFO0;
  DeviceInfo[UART_ID_DATA_POS + 2] = CHIP_INFO1;
  DeviceInfo[UART_ID_DATA_POS + 3] = ((FIRMWARE_VERSION[0] - '0') << 4) + (FIRMWARE_VERSION[1] - '0');
  DeviceInfo[UART_ID_DATA_POS + 4] = ((FIRMWARE_VERSION[3] - '0') << 4) + (FIRMWARE_VERSION[4] - '0');  //skip .
/*
    for (i=0, val=0; i< strlen(FIRMWARE_VERSION) ; i++)
    {
       if (FIRMWARE_VERSION[i]!='.')
          val = val*10+FIRMWARE_VERSION[i]-'0';
       else
          val
    }  
*/  
  memcpy(&DeviceInfo[UART_ID_DATA_POS + 5], mac_buf, 6);
  DeviceInfo[UART_ID_DATA_POS + 11] = BLE_APPLICATION;
  DeviceInfo[UART_ID_DATA_POS + 12] = UART_DATA_STOP;
}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
    {
      SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  // Discard unknown events
  return 0;
}

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}

uint8 g_ucConnInterval = 0;
uint16 BLE_DELAY = 0;
void setCONN_INTERVAL(uint8 conn_interval)
{
  if(conn_interval > 7)
    conn_interval = 7;
  g_ucConnInterval = conn_interval;
  uint16 intervalMin_tab[10] = {(8 * 1), (8 * 2), (8 * 5), (8 * 10), (8 * 15), (8 * 20), (8 * 25), (8 * 50), (8 * 75), (8 * 100)};
  uint16 intervalMax_tab[10] = {(8 * 2), (8 * 5), (8 * 10), (8 * 20), (8 * 30), (8 * 40), (8 * 50), (8 * 100), (8 * 150), (8 * 200)};
  uint16 desired_interval;
  uint16 conn_timeout = 0;
  if (conn_interval < 10)
  {
    desired_interval = intervalMax_tab[conn_interval];
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_interval);
    BLE_DELAY = desired_interval;
    BLE_DELAY = BLE_DELAY / 8;
    BLE_DELAY = BLE_DELAY * 10;
    conn_timeout = desired_interval / 8;
    conn_timeout = conn_timeout * 6;
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &conn_timeout);
    desired_interval = intervalMin_tab[conn_interval];
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_interval);
    uint8 update_request = TRUE; // And send update request to make iPad accept the conn intervals defined in the device
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &update_request);
  }
}

static uint8 S2S_ReadAttributeHdl(uint8* pData, uint8 Len)
{
  Batt_Update(pData[0]);
  //BattPeriodicityFill(pData[0]);
  return 0;
}

#define BAT_LOCAL       11
static uint8 S2S_HeartRateHdl(uint8* pData, uint8 ucLen)
{
  Batt_Update(pData[BAT_LOCAL]);
  /*
  BattPeriodicityFill(pData[BAT_LOCAL]);
  if(g_stNotiFlag.heartRate == TRUE)
  {
    attHandleValueNoti_t noti;
    if(ucLen > HR_LEN)
      noti.len = HR_LEN;
    else
      noti.len = ucLen;
    for(uint8 i = 0; i < noti.len; i++)
      noti.value[i] = pData[i];
    HeartRate_MeasNotify(gapConnHandle, &noti);
  }
  if(g_stNotiFlag.temperature == TRUE)
  {
    attHandleValueInd_t indcation;
    float fTemp = 0;
    uint8 ucTemp = pData[SKIN_TEMP_LOCAL];
    if(ucTemp & 0x80)
    {
      fTemp = (float)pData[SKIN_TEMP_LOCAL + 1];
      fTemp /= 4.0;
      ucTemp &= ~(0x80);
      fTemp += (float)ucTemp;
      fTemp = -fTemp;
    }
    else
    {
      fTemp = (float)pData[SKIN_TEMP_LOCAL + 1];
      fTemp /= 4.0;
      fTemp += (float)ucTemp;
    }
    uint8* pTemp = (uint8*)(&fTemp);
    indcation.len = 5;
    indcation.value[0] = 0;
    indcation.value[1] = pTemp[0];
    indcation.value[2] = pTemp[1];
    indcation.value[3] = pTemp[2];
    indcation.value[4] = pTemp[3];
    Thermometer_TempIndicate( gapConnHandle, &indcation, sensorTag_TaskID);
  }
  */
  return 0;
}
#if (AA60_PATCH==1)
void myAlertCB(uint8 event)
{

	if(event == MYALERT_MEAS_NOTI_ENABLED )
	{
		S2S_NotificationStat(HEART_ALERT_CHANNEL, TRUE);
		g_stNotiFlag.heartRateAlert = TRUE;
	}
	else if(event == MYALERT_MEAS_NOTI_DISABLED )
	{
		S2S_NotificationStat(HEART_ALERT_CHANNEL, FALSE);
		g_stNotiFlag.heartRateAlert = FALSE;
	}
}

static uint8 S2S_HeartAlertHdl(uint8* pData, uint8 ucLen)
{
  bStatus_t status;
	if(g_stNotiFlag.heartRateAlert == TRUE)
	{
		attHandleValueNoti_t noti;

		noti.len = ucLen;
        //copy data to noti, CC2540 is array, CC2541 is ptr 
		//for(uint8 i = 0; i < noti.len; i++)
		//	noti.value[i] = pData[i]; //把???的?据放到服?商准??射
#if (NOTI2_PATCH==1)
            uint16 len;
            noti.pValue = (uint8 *)GATT_bm_alloc( 0, ATT_HANDLE_VALUE_NOTI, GATT_MAX_MTU, &len );  //The NOTI GATT_MAX_MTU len=20.
            if (noti.pValue!=NULL)
              osal_memcpy(noti.pValue, pData, ucLen);
#if (UART_RXOW_CHECK==1)
            else
              uart_errs |= (0x01<<SERIAL_TX_NOBUF);
            if (pData[4]!=0xAA || pData[5]!=0xAA)
              uart_errs |= (0x01<<SERIAL_TX_DATAERR);
#endif
#else
        noti.pValue = pData;
#endif
//		acc_HeartAlertNotify( gapConnHandle, &noti );
		status = MyAlert_MeasNotify(gapConnHandle, &noti);
#if (NOTI2_PATCH==1)
        if (status != SUCCESS)
        {
               GATT_bm_free( (gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI );
        }
#endif
        return (status);
	}

	return 0;
}
#endif

static uint8 S2S_NullHdl(uint8* pData, uint8 Len)
{
  return 0xFE;
}

void String2Arry(uint8* sting, uint8 arry[])
{
  uint8 i = 0, *pData = sting;
  while(*pData != '\0' && i < 20)
  {
    arry[i] = *pData++;
    i++;
  }
  *pData = '\0';
}

extern uint8 devInfoSoftwareRev[];
static uint8 S2S_DeviceInfo(uint8* pData, uint8 ucLen)
{
  /*
  if(pData[1] < 10)
  {
    devInfoSoftwareRev[0] = ' ';
    devInfoSoftwareRev[1] = pData[1] + '0';
  }
  else
  {
    devInfoSoftwareRev[0] = pData[1] / 10 + '0';
    devInfoSoftwareRev[1] = pData[1] % 10 + '0';
  }
  if(pData[2] < 10)
  {
    devInfoSoftwareRev[3] = ' ';
    devInfoSoftwareRev[4] = pData[2] + '0';
  }
  else
  {
    devInfoSoftwareRev[3] = pData[2] / 10 + '0';
    devInfoSoftwareRev[4] = pData[2] % 10 + '0';
  }
  */
  return 0;
}

typedef uint8 (*pFnSerial2Service)(uint8 *pData,uint8 len);
/* The array table define and mapping all of the channel service(notification) function */
const pFnSerial2Service fnSerial2Service[] =
{
	S2S_NullHdl,    //CH0
	S2S_NullHdl,
	S2S_NullHdl,
	S2S_HeartRateHdl,
	S2S_NullHdl,
	S2S_NullHdl,
	S2S_NullHdl,//S2S_TemperatureHdl,
	S2S_NullHdl,//S2S_RunningHdl,
	S2S_ReadAttributeHdl,
	S2S_DeviceInfo,
#if (AA60_PATCH==1)
	S2S_HeartAlertHdl,  //CHA
#else
    S2S_NullHdl,
#endif
};

const uint8 S2S_FN_MAX = (sizeof(fnSerial2Service) / sizeof(fnSerial2Service[0]));
/*-----------------------------------------------------------------------------
* desc: S2S_NotificationStat(uint8 ucCmd,uint8 enbale)
*       BLE and MCU send notification state
* param: uint8 ucCmd,uint8 enbale
* output: void
*-----------------------------------------------------------------------------*/
static void S2S_NotificationStat(uint8 ucCmd, uint8 enbale)
{
	uint8 ucChnl, ucDataArry[7] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_INFOR, UART_B2M_CMD_NOTI, 0, (enbale == TRUE ? 1 : 0), UART_DATA_STOP};

	switch(ucCmd)
	{
		case HEART_RATE_CHANNEL:
			ucChnl = 1;
			break;

		case TEMPERATURE_CHANNEL:
			ucChnl = 2;
			break;

		case RUNNING_CHANNEL:
			ucChnl = 3;
			break;

		case HEART_ALERT_CHANNEL:
			ucChnl = 4;
			break;

		default:
			break;
	}

	ucDataArry[4] = ucChnl;
	txEncodeSend(ucDataArry, sizeof(ucDataArry)); //sendDataToHost(ucDataArry, sizeof(ucDataArry));//send to MCU.
}

uint8 rxBufferUart[MAX_PKT_SIZE >> 1];
uint8 rxIndexUart = 0;
uint8 rxPackCommandUart = false;
uint8 UartDataACK[UART_LEN_FOU] = {UART_DATA_START, UART_LEN_FOU, UART_CMD_DATA, UART_DATA_STOP};
uint8 g_ucANCSFlag = 0;
bool advConStatus = true;
uint8 MessageType = STATE_CMD_INFOR;
uint8 BLEStatus[6];
uint8 BLEBufferSlave[MAX_PKT_SIZE >> 1];  //ATUSDBG: why set half of RX_BUFF_SIZE?
uint8 BLEBufferLen = 0;
extern uint8 advertData[];
uint8 BLE_State = BLE_STATE_IDLE;
uint8 send_flage = 0;
#if (UART_RXOW_CHECK==1)
char uart_rx_buf_flag = 0;
uint8 uart_errs = 0;
#endif
void cSerialPacketParser( uint8 port, uint8 events )
{
  //unused input parameters
  (void)port;
  (void)events;
  uint8 numBytes;
  uint8 UartANCSDataACK[6] = {UART_DATA_START, 6, UART_CMD_ANCS, 0x11, 0x0, UART_DATA_STOP};
  uint8 new_adv_enabled_status;
  uint16 davInt2;
  uint8 group = 0;
  uint8 rem = 0;
  uint8 add = 0;
  uint8 i = 0;
  uint8 id = 0;
  uint8 temp = 0;
  uint8 ucCmd; //can be faster without same array reference fetch.
  uint8 ucLen;
      
  // get the number of available bytes to process
  numBytes = NPI_RxBufLen();
  if(numBytes != 0)
  {
    uint8 *temp_buf;
    uint8 *msg_buf;
    temp_buf = osal_mem_alloc(numBytes);
    /* store dma buffer into temp_buf */
#if (UART_RXOW_CHECK==1)
    if (temp_buf==NULL)
    {
      uart_errs|=(0x1<<SERIAL_RX_NOBUF);
      return;
    }
#endif
    (void)NPI_ReadTransport(temp_buf, numBytes);
    //HalUARTWrite(HAL_UART_PORT_0, temp_buf, numBytes);
    if(rxIndexUart == 0) //new msg.
    {
      /* addressing the start delimiter to skip leading pad */
      for(i = 0; i < numBytes; i++)
      {
        if(temp_buf[i] == UART_DATA_START)
        {
#if (UART_RXOW_CHECK==1)
          if (uart_rx_buf_flag!=0) //the last buf not free yet.
          {
            uart_errs |= (0x01<<SERIAL_RX_OW);
          }
          uart_rx_buf_flag++; //set flag, should be 1
#endif          
          break;
        }
      }
    }
    else
      i = 0;
    /* addressing stop delimiter */
    for(; i < numBytes; i++)
    {
      rxBufferUart[rxIndexUart++] = temp_buf[i];
      if(temp_buf[i] == UART_DATA_STOP)
      {
        rxPackCommandUart = true; //get the complete msg now.
#if (UART_RXOW_CHECK==1)
        uart_rx_buf_flag++; //set flag, should be 2
#endif
        break;
      }
    }
    osal_mem_free(temp_buf);
    /* process msg when we got the complete msg */
    if(rxPackCommandUart == true)
    {
      /* fixed the pad and length before apply */
      rxBufferUart[UART_ID_LEN_POS] -= 0x02; //the MCU pad 2 zero bytes.
      rxBufferUart[rxBufferUart[UART_ID_LEN_POS]] = UART_DATA_STOP; //the mark the end to remove the pad 2 zero bytes.
      
      //HalUARTWrite(HAL_UART_PORT_0, rxBufferUart, rxIndexUart);
      rxPackCommandUart = false;
      rxIndexUart = 0;
      if (rxBufferUart[UART_ID_CMD_POS] == UART_CMD_DATA)
      {
        ucCmd = rxBufferUart[UART_ID_DATA_POS];
        ucLen = rxBufferUart[UART_ID_LEN_POS] - UART_DATA_EXTRA_LEN;

		UartDataACK[UART_ID_CMD_POS] = UART_CMD_DATA;
        if(ucCmd == DEVICE_INFO_CHANNEL)
        {
          devInfoSoftwareRev[0] = (rxBufferUart[UART_ID_DATA_POS + 2] / 10) + '0'; //MCU: APP_FW_VER_M
          devInfoSoftwareRev[1] = (rxBufferUart[UART_ID_DATA_POS + 2] % 10) + '0';
          devInfoSoftwareRev[3] = (rxBufferUart[UART_ID_DATA_POS + 3] / 10) + '0'; //MCU: APP_FW_VER_S
          devInfoSoftwareRev[4] = (rxBufferUart[UART_ID_DATA_POS + 3] % 10) + '0';
        }
#if 0  //change version string from 2 digits(ab.cd) to 3 digits(ab.cd.ef) reference Uma.
        {
          devInfoSoftwareRev[0] = (rxBufferUart[UART_ID_DATA_POS + 2] / 10) + '0';
          devInfoSoftwareRev[1] = (rxBufferUart[UART_ID_DATA_POS + 2] % 10) + '0';
          devInfoSoftwareRev[3] = (rxBufferUart[UART_ID_DATA_POS + 3] / 10) + '0';
          devInfoSoftwareRev[4] = (rxBufferUart[UART_ID_DATA_POS + 3] % 10) + '0';
          devInfoSoftwareRev[6] = (rxBufferUart[UART_ID_DATA_POS + 4] / 10) + '0';
          devInfoSoftwareRev[7] = (rxBufferUart[UART_ID_DATA_POS + 4] % 10) + '0';
        }
        {
          int idx;
          idx=0;
          if (rxBufferUart[UART_ID_DATA_POS + 2]>9)
              devInfoSoftwareRev[idx++] = (rxBufferUart[UART_ID_DATA_POS + 2] / 10) + '0';
          devInfoSoftwareRev[idx++] = (rxBufferUart[UART_ID_DATA_POS + 2] % 10) + '0';
          devInfoSoftwareRev[idx++] = '.';
          if (rxBufferUart[UART_ID_DATA_POS + 3]>9)
              devInfoSoftwareRev[idx++] = (rxBufferUart[UART_ID_DATA_POS + 3] / 10) + '0';
          devInfoSoftwareRev[idx++] = (rxBufferUart[UART_ID_DATA_POS + 3] % 10) + '0';
          devInfoSoftwareRev[idx++] = '.';
          if (rxBufferUart[UART_ID_DATA_POS + 4]>9)
              devInfoSoftwareRev[idx++] = (rxBufferUart[UART_ID_DATA_POS + 4] / 10) + '0';
          devInfoSoftwareRev[idx++] = (rxBufferUart[UART_ID_DATA_POS + 4] % 10) + '0';
          for ( ; idx < sizeof(devInfoSoftwareRev) ; idx++)
               devInfoSoftwareRev[idx] = ' ';
        }
#endif        
		else if(gapProfileState == GAPROLE_CONNECTED || gapProfileState == GAPROLE_CONNECTED_ADV) //else if(BLE_State == BLE_STATE_CONNECTED)

		{
          if(ucCmd <= S2S_FN_MAX)
          {
            temp = fnSerial2Service[ucCmd](&rxBufferUart[UART_ID_DATA_NOTI], ucLen); //rxIndexUart
          }
          if(temp == 0xfe)  //It should be S2S_NullHdl
          {
            id = rxBufferUart[UART_ID_DATA_POS];
            group = rxBufferUart[UART_ID_LEN_POS];
            group = group - UART_ID_DATA_NOTI; //0x04: <,len,comm,ch
            if (group > 0)
              group--;
            else //length shorter than 4, just a need a ack to MCU.
            {
              txEncodeSend(UartDataACK, UART_LEN_FOU); //HalUARTWrite(HAL_UART_PORT_0, UartDataACK, 4); //0x3C 04 00 3E feedback ack to UART
#if (UART_RXOW_CHECK==1)
              uart_rx_buf_flag = 0; //clean flag, should be 0
#endif    
              return;
            }
            rem = group % UART_DATA_COM_LEN;
            group = group / UART_DATA_COM_LEN;
            for (i = 0; i < group; i++)
            {
              if (i == 0)
                add =  0x04;
              else
              {
                add = i;
                add = add * UART_DATA_COM_LEN;
                add +=  0x04;
              }
#if (NOTI_PATCH==1)
              rxDtatSave(&rxBufferUart[add], UART_DATA_COM_LEN, id);  //rxBufferSlave
#endif
            }
            if (rem > 0)
            {
              add = i;
              add = add * UART_DATA_COM_LEN;
              add +=  0x04;
#if (NOTI_PATCH==1)
              rxDtatSave(&rxBufferUart[add], rem, id); //rxBufferSlave
#endif
            }
            if (send_flage == 0x00)
            {
              send_flage = 0x01;
#if (NOTI_PATCH==1)
              osal_start_timerEx( sensorTag_TaskID, ST_SPI_BLE_EVT, SBP_BURST_EVT_PERIOD );
#endif
            }
          }
		}
        txEncodeSend(UartDataACK, UART_LEN_FOU); //0x3C 04 00 3E //txEncodeSend(HAL_UART_PORT_0, UartDataACK, UART_LEN_FOU);
      }
      else if(rxBufferUart[UART_ID_CMD_POS] == UART_CMD_INFOR)
      {
        UartDataACK[UART_ID_CMD_POS] = UART_CMD_INFOR;
		switch (rxBufferUart[UART_ID_DATA_POS])
		{
        case UART_TYPE_ANCS:
          if(rxBufferUart[UART_ID_DATA_POS + 1] == 0x01)
          {
            UartANCSDataACK[3] = 0x11;
            if(g_ucANCSFlag == ANCS_DISABLE_FLAG)
              UartANCSDataACK[4] = 0;
            else
              UartANCSDataACK[4] = 1;
          }
          else if(rxBufferUart[UART_ID_DATA_POS + 1] == 0x02)
          {
            UartANCSDataACK[3] = 0x12;
            uint8 temp[2] = {ANCS_DISABLE_FLAG, ANCS_DISABLE_FLAG};
			HalFlashErase(ANCS_ENABLE_PAGE);
			while (FCTL & 0x80);
            if(rxBufferUart[UART_ID_DATA_POS + 2] == 0)
            {
              UartANCSDataACK[4] = 0;
              HalFlashWrite((uint16)(ANCS_ENABLE_PAGE << 9), temp, 2);
            }
            else
            {
              UartANCSDataACK[4] = 1;
              temp[0] = temp[1] = ANCS_ENABLE_FLAG;
              HalFlashWrite((uint16)(ANCS_ENABLE_PAGE << 9), temp, 2);
            }
          }
          txEncodeSend(UartANCSDataACK, 6); //0x3C 06 09 11|12 00|01 3E //HalUARTWrite(HAL_UART_PORT_0, UartANCSDataACK, 6);
          break;
        case UART_TYPE_ADVER:
          if(gapProfileState != GAPROLE_CONNECTED)
          {
            if (rxBufferUart[UART_ID_DATA_POS + 1] == ADVER_ENABLE)
            {
              davInt2 = rxBufferUart[UART_ID_DATA_POS + 2];
              new_adv_enabled_status = false;
              GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status);
              advConStatus = true;
              davInt2 *= 160;
              GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, davInt2);
              GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, davInt2);
              GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, davInt2);
              GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, davInt2);
              new_adv_enabled_status = true;
              GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status);
            }
            else
            {
              GAPRole_GetParameter(GAPROLE_ADVERT_ENABLED, &new_adv_enabled_status);
              if (new_adv_enabled_status != FALSE)
              {
                new_adv_enabled_status = false;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status);
              }
            }
          }
          txEncodeSend(UartDataACK, UART_LEN_FOU); //0x3C 04 01 3E //HalUARTWrite(HAL_UART_PORT_0, UartDataACK, 4);
          break;
		case UART_TYPE_DEV:
          txEncodeSend(DeviceInfo, DeviceInfo[UART_ID_LEN_POS]); //0x3C 10 01 01 41 ...
          break;
		case UART_TYPE_INFOR:
		case UART_TYPE_COM:
          if (MessageType == STATE_CMD_INFOR)
            txEncodeSend(BLEStatus, BLEStatus[UART_ID_LEN_POS]); //HalUARTWrite(HAL_UART_PORT_0, BLEStatus, BLEStatus[UART_ID_LEN_POS]);
          else
          {
            BLEBufferSlave[UART_ID_LEN_POS] = BLEBufferLen + UART_LEN_FOU; //4, ATUS: trace contents
            //BLEBufferSlave[BLEBufferSlave[UART_ID_LEN]-1] = UART_DATA_STOP;
            txEncodeSend(BLEBufferSlave, BLEBufferLen); //HalUARTWrite(HAL_UART_PORT_0, BLEBufferSlave, BLEBufferLen);
            BLEBufferLen = 0;
          }
          break;
		case UART_TYPE_CONN:
          setCONN_INTERVAL(rxBufferUart[UART_ID_DATA_POS + 1]); //set the BLE connecgtion interval
          break;
		case UART_TYPE_BROADCAST:
          advertData[15] = rxBufferUart[UART_ID_DATA_POS + 1]; //FD alert
          advertData[16] = rxBufferUart[UART_ID_DATA_POS + 2];
          advertData[17] = rxBufferUart[UART_ID_DATA_POS + 3];
          advertData[18] = rxBufferUart[UART_ID_DATA_POS + 4];
          advertData[19] = rxBufferUart[UART_ID_DATA_POS + 5]; //SOS
          GAPRole_SetParameter(GAPROLE_ADVERT_DATA, 20, advertData);
          break;
#if (P180F_PATCH==1)
        case UART_TYPE_BATTERY: //0x07
          ucLen = rxBufferUart[UART_ID_LEN_POS] - UART_DATA_EXTRA_LEN;
          if (ucLen >=1)
            Batt_Update(rxBufferUart[UART_ID_DATA_POS + 1]);
          if (ucLen>=3)
          {
            int16 volt;
            volt = rxBufferUart[UART_ID_DATA_POS + 2] + rxBufferUart[UART_ID_DATA_POS + 3]<<8;
            BattVolt_Update(volt);
          }
          break;
#endif
        }
      }
      else if(rxBufferUart[UART_ID_CMD_POS] == UART_CMD_UPDATA)
      {
        UartDataACK[UART_ID_CMD_POS] = UART_CMD_UPDATA;
        if (rxBufferUart[UART_ID_DATA_POS] == 0x00)
        {
          txEncodeSend(UartDataACK, UART_LEN_FOU); //0x3C 04 02 3E //HalUARTWrite(HAL_UART_PORT_0, UartDataACK, UART_LEN_FOU);
          HalFlashErase(CRC_CHECK_PAGE);
          while (FCTL & 0x80); //wait until erase complete, then reset CC2540.(imply switch to bootloader mode)
          HAL_SYSTEM_RESET(); 
		}
      }
      /*
      else if(rxBufferUart[UART_ID_CMD_POS] == UART_CMD_REQUEST_INFO)
      {
        UartDataACK[UART_ID_CMD_POS] = UART_CMD_REQUEST_INFO;
        txEncodeSend(UartDataACK, UART_LEN_FOU); //HalUARTWrite(HAL_UART_PORT_0, UartDataACK, 4);
      }
      */
#if (UART_RXOW_CHECK==1)
        uart_rx_buf_flag = 0; //clean flag, should be 0
#endif    
    } //rxPackCommandUart==true
  }
}

#if 0 //use in circular_diff(), circular_add()
uint8 sendAckMessage(uint8 bytes_sent)
{
  uint8 data[3] = {0};
  
  data[0]= SERIAL_MSG_START_ID;
  data[1]= SERIAL_ACK;
  data[2]= bytes_sent;
  uint8 success_len = HalUARTWrite(NPI_UART_PORT, (uint8*)data, 3);
  if (success_len == 3)
  {
    return SUCCESS;
  }
  else
  {
    return 1;   //ack wasn't sent over UAR
  }
}
#endif
#if 0 //use in S2S_NotificationStat(), but it should be txEncodeSend().
uint8 sendDataToHost(uint8* data, uint8 len)
{
  uint8* buf = osal_mem_alloc((2+len)*sizeof(uint8));
  if (buf)  //if allocated
  {
    buf[0] = SERIAL_MSG_START_ID;
    buf[1]= SERIAL_DATA;
    osal_memcpy(&buf[2], data, len);
    uint8 bytes_sent = HalUARTWrite(NPI_UART_PORT, (uint8*)buf, len+2);
    osal_mem_free(buf);
    if (bytes_sent == len + 2)
    {
      return SUCCESS;
    }
    else
    {
      return 1;  //data not sent over UART
    }    
  }
  else
  {
    return bleMemAllocError;
  }
}
#endif

#if 0 //ATUS: remark it, not used.
uint16 circular_diff(uint16 offset, uint16 tail)
{
  if (offset > tail)
  {
    return (offset - tail);
  }
  else
  {
    return (RX_BUFF_SIZE - tail) + offset;
  }    
}

uint16 circular_add(uint16 x, uint16 y)
{
  uint16 sum = x + y;
  if (sum != RX_BUFF_SIZE)
  {
    sum = sum % RX_BUFF_SIZE;
  }
  else
  {
    sum = 0;
  }
  return sum;
}
#endif

#define Head_S       UART_DATA_START   // Start of packet byte
#define Head_E       UART_DATA_STOP    // End of packet byte

#define SOH_ESCAPE_CHAR          0x1b
#define SOH_ESCAPE_CHAR_MASK     0x33

#define ADD_ZERO_NUMBER  8

/*******************************************************
***	Desc: Serial data encode before send.
***	params: 
***	  data: the pointer of data buffer.
***   len: the length of data buffer.
***	output: encode into TxBufferSlave[] and send with TxBufferSlave[] 
*******************************************************/
void txEncodeSend(uint8* data, uint8 len)
{
	uint8 TxBufferSlave[128];
	uint8 len_temp, i, char_temp, index;

	len_temp = len - 3; //skip [0]START,[1] LEN, [LAST]STOP. All the remain will encode.

	TxBufferSlave[0] = 0;
	TxBufferSlave[1] = 0;
	TxBufferSlave[2] = 0;
	TxBufferSlave[3] = 0;
	TxBufferSlave[4] = 0;
	TxBufferSlave[5] = 0;
	TxBufferSlave[6] = 0;
	TxBufferSlave[7] = 0;
	TxBufferSlave[ADD_ZERO_NUMBER] = UART_DATA_START; //'<';
	index = ADD_ZERO_NUMBER + 2;

	for(i = 0; i < len_temp; i++)
	{
		char_temp = data[2 + i];

		switch(char_temp)
		{
			case Head_S:
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR;
				TxBufferSlave[index++] = Head_S ^ SOH_ESCAPE_CHAR_MASK;
				break;

			case Head_E:
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR;
				TxBufferSlave[index++] = Head_E ^ SOH_ESCAPE_CHAR_MASK;
				break;

			case SOH_ESCAPE_CHAR:
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR;
				TxBufferSlave[index++] = SOH_ESCAPE_CHAR ^ SOH_ESCAPE_CHAR_MASK;
				break;

			default:
				TxBufferSlave[index++] = char_temp;
				break;

		}

	}

	TxBufferSlave[index++] = UART_DATA_STOP; //'>'
	TxBufferSlave[(UART_ID_LEN_POS + ADD_ZERO_NUMBER)] = index - ADD_ZERO_NUMBER; //overwrite len after encode.

	NPI_WriteTransport(TxBufferSlave, index); //send the encode buffer via UART to MCU. sbpSerialAppWrite

}

#if (NOTI_PATCH==1)
/*******************************************************
***	desc: save data to sendDataBuf[][]
***	params:
***   data: the address(pointer) of data.
***   len: the data length
***	  id: the channel tag
*******************************************************/
void rxDtatSave(uint8* data, uint8 len, uint8 id)
{
	uint8 i = 0;

	for (i = 0; i < sendDataBufSiz; i++)
	{
		if (sendDataBuf[i][sendDataLen] == 0)
		{
			sendDataBuf[i][sendDataId] = id;
			memcpy(&sendDataBuf[i][sendDataData], data, len);
			sendDataBuf[i][sendDataLen] = len;
			return;
		}
	}
}
#endif