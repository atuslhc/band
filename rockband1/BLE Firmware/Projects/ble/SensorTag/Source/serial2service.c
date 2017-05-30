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

//该变量决定哪些情况下要发送notification。
stNotiFlag_Type g_stNotiFlag =
{
	.heartRate = FALSE,
	.temperature = FALSE,
	.running = FALSE,
	.heartRateAlert = FALSE, //新添加
};


//该函数未被调用。
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

//在函数在心率服务注册时被设置成回调函数，当心率发送变化时调用。
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

//使用lightblue进行调试时，在lightblue中点击listen for notification就会到该函数中。
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
//该函数已经不用了，被myAlertCB替代  2015年9月2日15:41:38
//为了查错，先屏蔽 2015年8月12日11:43:18
//-----------------新加的。  这个不要enable或者disable。发送频率有MCU决定。
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
* 函数名称: S2S_NullHdl(uint8 *pData,uint8 Len)
* 输入参数: uint8 *pData,uint8 Len
* 输出参数: static uint8
*
* 功能描述: 串口到服务上的空函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/19 星期四
*-----------------------------------------------------------------------------*/
static uint8 S2S_NullHdl(uint8* pData, uint8 Len)
{

	return 0xFE;
}
/*-----------------------------------------------------------------------------
* 函数名称: S2S_HeartRateHdl(uint8 *pData,uint8 Len)
* 输入参数: uint8 *pData,uint8 Len
* 输出参数: static uint8
*
* 功能描述: 串口到服务上的心率处理函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/19 星期四
*flag HeartRate AmTemp skinTemp Steps Distance UV
* 1       1        2       2     2        2     1  =11
*-----------------------------------------------------------------------------*/
#define SKIN_TEMP_LOCAL 4
#define HR_LOCAL        0
#define BAT_LOCAL       11
#define RUN_LOCAL       12

#define HR_LEN          (20)
#define BAT_LEN          (RUN_LOCAL-BAT_LOCAL)

//这里把心率和温度发送到APP
static uint8 S2S_HeartRateHdl(uint8* pData, uint8 ucLen)
{
	BattPeriodicityFill(pData[BAT_LOCAL]);

	if(g_stNotiFlag.heartRate == TRUE) //该值在心率回调函数中置为true.
	{
		attHandleValueNoti_t noti;

		if(ucLen > HR_LEN)noti.len = HR_LEN;
		else
			noti.len = ucLen;

		for(uint8 i = 0; i < noti.len; i++)
			noti.value[i] = pData[i]; //把传进来的数据放到服务商准备发射

		HeartRate_MeasNotify(gapConnHandle, &noti);
	}

	if(g_stNotiFlag.temperature == TRUE)
	{
		attHandleValueInd_t indcation;
		float fTemp = 0;
		uint8 ucTemp = pData[SKIN_TEMP_LOCAL];

		if(ucTemp & 0x80) //负数
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
		Thermometer_TempIndicate( gapConnHandle, &indcation, sensorTag_TaskID);//这里就把温度数据发送出去，以服务的形式发送给手机的ＡＰＰ
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
* 函数名称: S2S_ReadAttributeHdl(uint8 *pData,uint8 Len)
* 输入参数: uint8 *pData,uint8 Len
* 输出参数: static uint8
*
* 功能描述:  串口到服务上的读属性处理函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/19 星期四
*-----------------------------------------------------------------------------*/
static uint8 S2S_ReadAttributeHdl(uint8* pData, uint8 Len)
{

	BattPeriodicityFill(pData[0]);
	return 0;
}


/*-----------------------------------------------------------------------------
* 函数名称: static uint8 S2S_TemperatureHdl(uint8* pData,uint8 ucLen)
* 输入参数: uint8* pData,uint8 ucLen
* 输出参数: static uint8
*
* 功能描述:  串口到服务上的温度处理函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/20 星期五
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
* 函数名称: static uint8 S2S_RunningHdl(uint8 *pData , uint8 ucLen)
* 输入参数: uint8 *pData , uint8 ucLen
* 输出参数: static uint8
*
* 功能描述:  串口到服务上的计步和距离处理函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/20 星期五
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


//----------新加的 2015年7月9日12:16:30 向APP发送心率报警的notify
static uint8 S2S_HeartAlertHdl(uint8* pData, uint8 ucLen)
{
	if(g_stNotiFlag.heartRateAlert == TRUE)
	{
		attHandleValueNoti_t noti;

		noti.len = ucLen;

		for(uint8 i = 0; i < noti.len; i++)
			noti.value[i] = pData[i]; //把传进来的数据放到服务商准备发射

//		acc_HeartAlertNotify( gapConnHandle, &noti );
		MyAlert_MeasNotify(gapConnHandle, &noti);
	}

	return 0;
}

/*-----------------------------------------------------------------------------
* 函数名称: S2S_DeviceInfo(uint8 *pData,uint8 Len)
* 输入参数: uint8 *pData,uint8 Len
* 输出参数: static uint8
*
* 功能描述: 串口到服务上的设备信息处理函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/19 星期四
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
//这里面的序号对应的起始就是通讯协议中的通道号。
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
	S2S_DeviceInfo,//把相应的数据放到相应的缓存中。
	S2S_HeartAlertHdl,//新加 2015年7月9日12:10:58，在该函数中添加向APP发送心率报警notify.
};
const uint8 S2S_FN_MAX = (sizeof(fnSerial2Service) / sizeof(fnSerial2Service[0]));



/*-----------------------------------------------------------------------------
* 函数名称: void S2S_NotificationStat(uint8 ucCmd,uint8 enbale)
* 输入参数: uint8 ucCmd,uint8 enbale
* 输出参数: void
*
* 功能描述: BLE与主MCU上发送notification的状态。
*
* 作    者:  何志辉
* 创建日期: 2014/6/19 星期四
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

//新增加的。。。。
		case HEART_ALERT_CHANNEL:
			ucChnl = 4;
			break;

		default:
			break;
	}

	ucDataArry[4] = ucChnl;
	txDtatSend(ucDataArry, sizeof(ucDataArry));//这个数据发送到主MCU。
}

/*-----------------------------------------------------------------------------
* 函数名称: void ImmediateAlertCB(uint8 ucValue)
* 输入参数: uint8 ucValue 0关 1轻AlertCB 2重AlertCB
* 输出参数: void
*
* 功能描述: findMe的回调函数，当手机端发送数据，就会传给ucValue，BLE负责转给外部MCU
*
* 作    者:  何志辉
* 创建日期: 2014/6/20 星期五
*-----------------------------------------------------------------------------*/
void ImmediateAlertCB(uint8 ucValue)
{
	if(ucValue > 2)ucValue = 2;

	uint8 ucDataArry[6] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_WRITE_ATT, WRITE_ATT_FIND_ME, ucValue, UART_DATA_STOP};
	txDtatSend(ucDataArry, sizeof(ucDataArry));
}


/*-----------------------------------------------------------------------------
* 函数名称: static void S2S_RequestDevInfo(uint8 ucData )
* 输入参数: uint8 ucData
* 输出参数: static void
*
* 功能描述: 请求设备信息
*
* 作    者:  何志辉
* 创建日期: 2014/6/20 星期五
*-----------------------------------------------------------------------------*/
static void S2S_RequestDevInfo(uint8 ucData )
{
	uint8 ucDataArry[5] = {UART_DATA_START, sizeof(ucDataArry), UART_CMD_REQUEST_INFO, ucData, UART_DATA_STOP};
	txDtatSend(ucDataArry, sizeof(ucDataArry));
}


/*-----------------------------------------------------------------------------
* 函数名称: void DevInfoRequestCB(void)
* 输入参数: void
* 输出参数: void
*
* 功能描述: 请求设备信息回调函数
*
* 作    者:  何志辉
* 创建日期: 2014/6/20 星期五
*-----------------------------------------------------------------------------*/
void DevInfoRequestCB(void)
{
	S2S_RequestDevInfo(1);
}
