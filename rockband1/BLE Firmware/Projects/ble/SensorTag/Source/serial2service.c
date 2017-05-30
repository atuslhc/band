#include "serial2Service.h"
#include "runningservice.h"
#include "runningservice.h"
#include "accelerometerservice.h"
#include "string.h"
#include "testAlertNotification.h"
#include "battservice.h"


static void S2S_NotificationStat(uint8 ucCmd, uint8 enbale);

extern uint16 gapConnHandle;

extern uint8 devInfoModelNumber[];
extern uint8 devInfoSoftwareRev[];

extern uint8 sensorTag_TaskID;

//�ñ���������Щ�����Ҫ����notification��
stNotiFlag_Type g_stNotiFlag =
{
	.heartRate = FALSE,
	.temperature = FALSE,
	.running = FALSE,
	.heartRateAlert = FALSE, //�����
};


//�ú���δ�����á�
bStatus_t Running_CBs(uint8 event)
{
	if(event == RSC_MEAS_NOTI_ENABLED )
	{
		S2S_NotificationStat(RUNNING_CHANNEL, TRUE);
		g_stNotiFlag.running = TRUE;
	}
	else if(event == RSC_MEAS_NOTI_DISABLED )
	{
		S2S_NotificationStat(RUNNING_CHANNEL, FALSE);
		g_stNotiFlag.running = FALSE;
	}

	return SUCCESS;
}





void Thermometer_CBs(uint8 event)
{
	if(event == THERMOMETER_TEMP_IND_ENABLED )
	{
		S2S_NotificationStat(TEMPERATURE_CHANNEL, TRUE);
		g_stNotiFlag.temperature = TRUE;
	}
	else if(event == THERMOMETER_TEMP_IND_DISABLED )
	{
		S2S_NotificationStat(TEMPERATURE_CHANNEL, FALSE);
		g_stNotiFlag.temperature = FALSE;
	}
	else
	{

	}
}

//�ں��������ʷ���ע��ʱ�����óɻص������������ʷ��ͱ仯ʱ���á�
void heartRateCB(uint8 event)
{
	if (event == HEARTRATE_MEAS_NOTI_ENABLED)
	{
		S2S_NotificationStat(HEART_RATE_CHANNEL, TRUE);
		g_stNotiFlag.heartRate = TRUE;
	}
	else if (event == HEARTRATE_MEAS_NOTI_DISABLED)
	{
		S2S_NotificationStat(HEART_RATE_CHANNEL, FALSE);
		g_stNotiFlag.heartRate = FALSE;
	}
	else
	{

	}
}

//ʹ��lightblue���е���ʱ����lightblue�е��listen for notification�ͻᵽ�ú����С�
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

#if 0
//�ú����Ѿ������ˣ���myAlertCB���  2015��9��2��15:41:38
//Ϊ�˲�������� 2015��8��12��11:43:18
//-----------------�¼ӵġ�  �����Ҫenable����disable������Ƶ����MCU������
void HeartRateAlert_CBs(uint8 event)
{
	if(event == HEARTRATE_ALERT_NOTI_ENABLED )
	{
		S2S_NotificationStat(HEART_ALERT_CHANNEL, TRUE);
		g_stNotiFlag.heartRateAlert = TRUE;
	}
	else if(event == HEARTRATE_ALERT_NOTI_DISABLED )
	{
		S2S_NotificationStat(HEART_ALERT_CHANNEL, FALSE);
		g_stNotiFlag.heartRateAlert = FALSE;
	}

}
#endif

/*-----------------------------------------------------------------------------
* ��������: S2S_NullHdl(uint8 *pData,uint8 Len)
* �������: uint8 *pData,uint8 Len
* �������: static uint8
*
* ��������: ���ڵ������ϵĿպ���
*
* ��    ��:  ��־��
* ��������: 2014/6/19 ������
*-----------------------------------------------------------------------------*/
static uint8 S2S_NullHdl(uint8* pData, uint8 Len)
{

	return 0xFE;
}
/*-----------------------------------------------------------------------------
* ��������: S2S_HeartRateHdl(uint8 *pData,uint8 Len)
* �������: uint8 *pData,uint8 Len
* �������: static uint8
*
* ��������: ���ڵ������ϵ����ʴ�����
*
* ��    ��:  ��־��
* ��������: 2014/6/19 ������
*flag HeartRate AmTemp skinTemp Steps Distance UV
* 1       1        2       2     2        2     1  =11
*-----------------------------------------------------------------------------*/
#define SKIN_TEMP_LOCAL 4
#define HR_LOCAL        0
#define BAT_LOCAL       11
#define RUN_LOCAL       12

#define HR_LEN          (20)
#define BAT_LEN          (RUN_LOCAL-BAT_LOCAL)

//��������ʺ��¶ȷ��͵�APP
static uint8 S2S_HeartRateHdl(uint8* pData, uint8 ucLen)
{
	BattPeriodicityFill(pData[BAT_LOCAL]);

	if(g_stNotiFlag.heartRate == TRUE) //��ֵ�����ʻص���������Ϊtrue.
	{
		attHandleValueNoti_t noti;

		if(ucLen > HR_LEN)noti.len = HR_LEN;
		else
			noti.len = ucLen;

		for(uint8 i = 0; i < noti.len; i++)
			noti.value[i] = pData[i]; //�Ѵ����������ݷŵ�������׼������

		HeartRate_MeasNotify(gapConnHandle, &noti);
	}

	if(g_stNotiFlag.temperature == TRUE)
	{
		attHandleValueInd_t indcation;
		float fTemp = 0;
		uint8 ucTemp = pData[SKIN_TEMP_LOCAL];

		if(ucTemp & 0x80) //����
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
		Thermometer_TempIndicate( gapConnHandle, &indcation, sensorTag_TaskID);//����Ͱ��¶����ݷ��ͳ�ȥ���Է������ʽ���͸��ֻ��ģ��У�
	}

//  if(g_stNotiFlag.running == TRUE){
//
//    attHandleValueNoti_t noti;
//    noti.len=ucLen-RUN_LOCAL;
//    for(uint8 i=0;i<ucLen;i++)
//      noti.value[i]=pData[i+RUN_LOCAL];
//   Running_MeasNotify(gapConnHandle,&noti);
//  }
	return 0;
}


/*-----------------------------------------------------------------------------
* ��������: S2S_ReadAttributeHdl(uint8 *pData,uint8 Len)
* �������: uint8 *pData,uint8 Len
* �������: static uint8
*
* ��������:  ���ڵ������ϵĶ����Դ�����
*
* ��    ��:  ��־��
* ��������: 2014/6/19 ������
*-----------------------------------------------------------------------------*/
static uint8 S2S_ReadAttributeHdl(uint8* pData, uint8 Len)
{

	BattPeriodicityFill(pData[0]);
	return 0;
}


/*-----------------------------------------------------------------------------
* ��������: static uint8 S2S_TemperatureHdl(uint8* pData,uint8 ucLen)
* �������: uint8* pData,uint8 ucLen
* �������: static uint8
*
* ��������:  ���ڵ������ϵ��¶ȴ�����
*
* ��    ��:  ��־��
* ��������: 2014/6/20 ������
*-----------------------------------------------------------------------------*/

static uint8 S2S_TemperatureHdl(uint8* pData, uint8 ucLen)
{

	attHandleValueInd_t indcation;
	indcation.len = ucLen;

	for(uint8 i = 0; i < ucLen; i++)
		indcation.value[i] = pData[i];

	return Thermometer_TempIndicate( gapConnHandle, &indcation, sensorTag_TaskID);

}


/*-----------------------------------------------------------------------------
* ��������: static uint8 S2S_RunningHdl(uint8 *pData , uint8 ucLen)
* �������: uint8 *pData , uint8 ucLen
* �������: static uint8
*
* ��������:  ���ڵ������ϵļƲ��;��봦����
*
* ��    ��:  ��־��
* ��������: 2014/6/20 ������
*-----------------------------------------------------------------------------*/
static uint8 S2S_RunningHdl(uint8* pData , uint8 ucLen)
{
//     attHandleValueNoti_t noti;
//    noti.len=ucLen;
//    for(uint8 i=0;i<ucLen;i++)
//      noti.value[i]=pData[i];
//   return Running_MeasNotify(gapConnHandle,&noti);
	return 0;
}


//----------�¼ӵ� 2015��7��9��12:16:30 ��APP�������ʱ�����notify
static uint8 S2S_HeartAlertHdl(uint8* pData, uint8 ucLen)
{
	if(g_stNotiFlag.heartRateAlert == TRUE)
	{
		attHandleValueNoti_t noti;

		noti.len = ucLen;

		for(uint8 i = 0; i < noti.len; i++)
			noti.value[i] = pData[i]; //�Ѵ����������ݷŵ�������׼������

//		acc_HeartAlertNotify( gapConnHandle, &noti );
		MyAlert_MeasNotify(gapConnHandle, &noti);
	}

	return 0;
}

/*-----------------------------------------------------------------------------
* ��������: S2S_DeviceInfo(uint8 *pData,uint8 Len)
* �������: uint8 *pData,uint8 Len
* �������: static uint8
*
* ��������: ���ڵ������ϵ��豸��Ϣ������
*
* ��    ��:  ��־��
* ��������: 2014/6/19 ������
*-----------------------------------------------------------------------------*/
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


static uint8 S2S_DeviceInfo(uint8* pData, uint8 ucLen)
{
#define DEVICE_INFO_LEN_V1      4

	//if(ucLen==DEVICE_INFO_LEN_V1)
	{
		switch(pData[0])//Model
		{
			case 0x01://i4
				String2Arry(DEVICE_INFO_MODEL_1, devInfoModelNumber);
				break;

			case 0x02://i4s
				String2Arry(DEVICE_INFO_MODEL_2, devInfoModelNumber);
				break;

			case 0x03://i2
				String2Arry(DEVICE_INFO_MODEL_3, devInfoModelNumber);
				break;

			case 0x04://i2s
				String2Arry(DEVICE_INFO_MODEL_4, devInfoModelNumber);
				break;

			case 0x05://i3
				String2Arry(DEVICE_INFO_MODEL_5, devInfoModelNumber);
				break;
		}

		if(pData[1] < 10)
		{
			devInfoSoftwareRev[0] = ' ';
			devInfoSoftwareRev[1] = pData[1] + '0';
		}
		else{
			devInfoSoftwareRev[0] = pData[1] / 10 + '0';
			devInfoSoftwareRev[1] = pData[1] % 10 + '0';
		}

		if(pData[2] < 10)
		{
			devInfoSoftwareRev[3] = ' ';
			devInfoSoftwareRev[4] = pData[2] + '0';
		}
		else{
			devInfoSoftwareRev[3] = pData[2] / 10 + '0';
			devInfoSoftwareRev[4] = pData[2] % 10 + '0';
		}
		return 0;
	}
}



/* The array table define and mapping all of the channel service(notification) function */
//���������Ŷ�Ӧ����ʼ����ͨѶЭ���е�ͨ���š�
const pFnSerial2Service fnSerial2Service[] =
{
	S2S_NullHdl,
	S2S_NullHdl,
	S2S_NullHdl,
	S2S_HeartRateHdl,
	S2S_NullHdl,
	S2S_NullHdl,
	S2S_TemperatureHdl,
	S2S_RunningHdl,
	S2S_ReadAttributeHdl,
	S2S_DeviceInfo,//����Ӧ�����ݷŵ���Ӧ�Ļ����С�
	S2S_HeartAlertHdl,//�¼� 2015��7��9��12:10:58���ڸú����������APP�������ʱ���notify.
};
const uint8 S2S_FN_MAX = (sizeof(fnSerial2Service) / sizeof(fnSerial2Service[0]));



/*-----------------------------------------------------------------------------
* ��������: void S2S_NotificationStat(uint8 ucCmd,uint8 enbale)
* �������: uint8 ucCmd,uint8 enbale
* �������: void
*
* ��������: BLE����MCU�Ϸ���notification��״̬��
*
* ��    ��:  ��־��
* ��������: 2014/6/19 ������
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

//�����ӵġ�������
		case HEART_ALERT_CHANNEL:
			ucChnl = 4;
			break;

		default:
			break;
	}

	ucDataArry[4] = ucChnl;
	txDtatSend(ucDataArry, sizeof(ucDataArry));//������ݷ��͵���MCU��
}

/*-----------------------------------------------------------------------------
* ��������: void ImmediateAlertCB(uint8 ucValue)
* �������: uint8 ucValue 0�� 1��AlertCB 2��AlertCB
* �������: void
*
* ��������: findMe�Ļص����������ֻ��˷������ݣ��ͻᴫ��ucValue��BLE����ת���ⲿMCU
*
* ��    ��:  ��־��
* ��������: 2014/6/20 ������
*-----------------------------------------------------------------------------*/
void ImmediateAlertCB(uint8 ucValue)
{
	if(ucValue > 2)ucValue = 2;

	uint8 ucDataArry[6] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_WRITE_ATT, WRITE_ATT_FIND_ME, ucValue, UART_DATA_STOP};
	txDtatSend(ucDataArry, sizeof(ucDataArry));
}


/*-----------------------------------------------------------------------------
* ��������: static void S2S_RequestDevInfo(uint8 ucData )
* �������: uint8 ucData
* �������: static void
*
* ��������: �����豸��Ϣ
*
* ��    ��:  ��־��
* ��������: 2014/6/20 ������
*-----------------------------------------------------------------------------*/
static void S2S_RequestDevInfo(uint8 ucData )
{
	uint8 ucDataArry[5] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_REQUEST_INFO, ucData, UART_DATA_STOP};
	txDtatSend(ucDataArry, sizeof(ucDataArry));
}


/*-----------------------------------------------------------------------------
* ��������: void DevInfoRequestCB(void)
* �������: void
* �������: void
*
* ��������: �����豸��Ϣ�ص�����
*
* ��    ��:  ��־��
* ��������: 2014/6/20 ������
*-----------------------------------------------------------------------------*/
void DevInfoRequestCB(void)
{
	S2S_RequestDevInfo(1);
}
