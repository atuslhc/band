#include "m00930.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "common_vars.h"
#include "device_task.h"
#include "led_font.h"
#include "menu.h"
#include "subMenu.h"
#include "notification.h"
#include "drv2605.h"
#include "BLE.h"

#include "main.h"
#include "data_gather.h"
#include "task.h"
#include "AFE44x0.h"

void OledDrawGuage(uint8_t guage, bool isShowNumber);
void OledDisplayUV(BYTE uvIndex, BYTE* icon);

//static const char* MONTHS[12] = {"January", "February", "March", "April", "May",
//				"June", "July", "August", "September", "October", "November", "December"};
//static const char* ABBR_MONTHS[12] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN",
//								"JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};
//static const char* ABBR_MONTHS_CAMEL[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
//								"Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
//
//static const char* WEEKDAYS[7] = {"Sunday", "Monday", "Tuesday", "Wednesday",
//									"Thurday", "Friday", "Saturday"};
//static const char* ABBR_WEEKDAYS[7] = { "SUN","MON", "TUE", "WED", "THU", "FRI", "SAT"};
//static const char* ABBR_WEEKDAYS_CAMEL[7] = { "Sun","Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
//static const char* WEEKDAYS_INITIAL[7] = { "S", "M", "T", "W", "T", "F", "S"};
//static const char* WEEKDAYS_INITIAL_LOWCASE[7] = { "s", "m", "t", "w", "t", "f", "s"};


extern tMemuInfoTypeDef g_tMenuInfo;
extern uint8_t ucMenuRamBuf[4][192];

void clearScreen(bool fillWithWhite)
{
	BYTE data = fillWithWhite ? 0xFF : 0x00;

	// clear screen
	for(int j = 0; j < 4; j++)
	{
		for(int i = 0; i < SCREEN_WIDTH; i++)
		{
			ucMenuRamBuf[j][i + g_tMenuInfo.mucUpDataArea] = data;
		}
	}
}


/*
在指定位置显示一个字符串
支持 \n 换行
*/
void OledShowString(short page, const FONT_INFO* const font, int interval, short x, short y, char* str)
{
	if (font == 0)
		return;

	int FONT_WIDTH = font->width + interval;
	int FONT_HEIGHT = font->height;

	short cx = x, cy = y;
	char c;

	while ((c = *(str++)) != 0)
	{
		if (c == ' ')
		{
			cx += FONT_WIDTH;
			continue;
		}

		if (c == '\r')
			continue;

		if (c == '\n')
		{
			cx = 0;
			cy += FONT_HEIGHT;
			continue;
		}

		oled_drawPic(page, cx, cy,
		             (BYTE*)(font->data + ((c - font->base) * font->bytes)),
		             font->width, font->bHeight, DRAW_MODE_TRANSPARENT);
		cx += FONT_WIDTH;
	}
}

/*------------------------------------------------------------------------------
 *函数：void OledDisplayDecimals(const uint8_t (* pucDat)[16], int16_t ulTemp)
 *行参：1.pucDat 图标指针 32X16像素
 *              pucDat[4][16];
 *              ulTemp 显示的温度，注：温度要乘以10+0.5
 *      2.ulTemp
 *              如：36.56度，那就是ulTemp=36.56*10+0.5=366
 *描述：显示温度
 *----------------------------------------------------------------------------*/
void OledDisplayDecimals(BYTE* icon, int16_t ulTemp , bool flag)
{
	char ucTemp[4]; //[BG025] uint8_t >> char
	uint8_t length = 0;
//	int16_t tempSave = 0;

	INT16 page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	sprintf(ucTemp, "%d", ulTemp);
	length = strlen(ucTemp);

	for(uint8_t n = 0; n < length; n++)
		ucTemp[n] -= 0x30;

	clearScreen(false);

	if(flag != true)
	{
		//false表示正温度

		switch(length)
		{
			case 1:
				//仅有1位时，需要在小数点前面加上0
				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[0],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				break;

			case 2:
				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				break;

			case 3:
				oled_drawPic(page, SCREEN_WIDTH - 52, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[2]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				break;

			case 4:
				oled_drawPic(page, 0, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 52, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[2]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[3]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				break;

			default:
				break;
		}
	}
	else
	{
		//负温度不显示小数
//		ucTemp[0] = (ulTemp % 1000 / 100);
//		ucTemp[1] = (ulTemp % 100 / 10);

//		if(ucTemp[0] == 0)
//		{
//			oled_drawHortLine(page, 24, 28, 36, 0);
//			oled_drawHortLine(page, 25, 28, 36, 0);//显示负号
//
//			oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
//			             (BYTE*)iconBigNumbers_12_16[1],
//			             12, 16, DRAW_MODE_TRANSPARENT);
//		}
//		else
//		{
//			oled_drawHortLine(page, 24, SCREEN_WIDTH - 48, SCREEN_WIDTH - 40, 0);
//			oled_drawHortLine(page, 25, SCREEN_WIDTH - 48, SCREEN_WIDTH - 40, 0); //显示负号
//
//			oled_drawPic(page, SCREEN_WIDTH - 36, SCREEN_HEIGHT - 15,
//			             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
//			             12, 16, DRAW_MODE_TRANSPARENT);
//			oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
//			             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
//			             12, 16, DRAW_MODE_TRANSPARENT);
//		}

		switch(length)
		{
			case 1:
				oled_drawHortLine(page, 24, 12, 20, 0);
				oled_drawHortLine(page, 25, 12, 20, 0);//显示负号

				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[0],
				             12, 16, DRAW_MODE_TRANSPARENT);
				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			case 2:
				oled_drawHortLine(page, 24, 12, 20, 0);
				oled_drawHortLine(page, 25, 12, 20, 0);//显示负号

				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			case 3:
			case 4:

				oled_drawHortLine(page, 24, 0, 8, 0);
				oled_drawHortLine(page, 25, 0, 8, 0);//显示负号

				oled_drawPic(page, SCREEN_WIDTH - 52, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				oled_drawPic(page, SCREEN_WIDTH - 40, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				oled_drawPic(page, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[2]],
				             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			default:
				break;

		}
	}

	//小数点
	oled_drawPic(page, SCREEN_WIDTH - 28, SCREEN_HEIGHT - 15,
	             (BYTE*)iconDecimalPoint_4_16,
	             4, 16, DRAW_MODE_TRANSPARENT);


	//图标

	oled_drawPic(page, SCREEN_WIDTH - 15, 0,
	             icon,
	             15, 16, DRAW_MODE_TRANSPARENT);
}

/*------------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
//void OledDisplayNumber(const uint8_t (* pucDat)[16], int intPart, int decPart, int decLen)
//stepOrDistance = 1,表示step; 0 表示distance
void OledDisplayNumber(float value, int decLen, bool hasUnit)
{
	uint8_t i, length;
	char ucTemp[5]; //[BG025] uint8_t >> char
	int t = 0;

	INT16 page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	if(hasUnit)
	{
		//仅仅距离有单位，并且保留1位小数 最大显示99.9KM 或者99.9mi
		if(decLen == 1)
		{
			sprintf(ucTemp, "%.1f", value + 0.05);

			if((ucTemp[0] == '0') && (ucTemp[1] == '.') && (ucTemp[2] == '1'))
			{
				ucTemp[2] = '0';
				length = 3;
			}
			else
				length = strlen(ucTemp);


			for(uint8_t n = 0; n < length ; n++)
			{
				if(ucTemp[n] == '.')
					continue;
				else
					ucTemp[n] -= 0x30;
			}


			switch(length)
			{
				//显示小数点前面的
				case 3:
					oled_drawPic(page, SCREEN_WIDTH - 43, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
					             12, 16, DRAW_MODE_TRANSPARENT);
					//最后一位
					oled_drawPic(page, SCREEN_WIDTH - 27, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[2]],
					             12, 16, DRAW_MODE_TRANSPARENT);

					break;

				case 4:
					oled_drawPic(page, SCREEN_WIDTH - 55, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
					             12, 16, DRAW_MODE_TRANSPARENT);
					oled_drawPic(page, SCREEN_WIDTH - 43, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[1]],
					             12, 16, DRAW_MODE_TRANSPARENT);
					//最后一位
					oled_drawPic(page, SCREEN_WIDTH - 27, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[3]],
					             12, 16, DRAW_MODE_TRANSPARENT);

					break;

				default:
					break;
			}
		}

		//显示小数点
		oled_drawPic(page, SCREEN_WIDTH - 31, SCREEN_HEIGHT - 15,
		             (BYTE*)iconDecimalPoint_4_16,
		             4, 16, DRAW_MODE_TRANSPARENT);
	}
	else
	{
		//没有单位的数据显示,并且没有小数点
		t = (int) value;
		sprintf(ucTemp, "%d", t);
		length = strlen(ucTemp);

		for(int k = 0; k < length; k++)
			ucTemp[k] -= 0x30 ;

		switch(length)
		{
			case 1:
				oled_drawPic(page, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 15,
				             (BYTE*)iconBigNumbers_12_16[ucTemp[0]],
				             12, 16, DRAW_MODE_TRANSPARENT);
				break;

			case 2:
				for(i = 0 ; i < 2; i++)
					oled_drawPic(page, 40 + (i * 12), SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[i]],
					             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			case 3:
				for(i = 0 ; i < 3; i++)
					oled_drawPic(page, 28 + i * 12, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[i]],
					             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			case 4:
				for(i = 0 ; i < 4; i++)
					oled_drawPic(page, 16 + i * 12, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[i]],
					             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			case 5:
				for(i = 0 ; i < 5; i++)
					oled_drawPic(page, 2 + i * 12, SCREEN_HEIGHT - 15,
					             (BYTE*)iconBigNumbers_12_16[ucTemp[i]],
					             12, 16, DRAW_MODE_TRANSPARENT);

				break;

			default:
				break;
		}

	}
}



void OledDisplayTimeAndDate(void)
{

	bool isAfternoon = false;
	clearScreen(false);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	uint8_t hoursTemp = 0;
	uint8_t minutesTemp[2] = {0};


	if(pSystemTime->tm_hour > 11)
	{
		isAfternoon = true;
	}

	if(pSystemTime->tm_hour > 12)
	{
		hoursTemp = pSystemTime->tm_hour - 12;
	}
	else if (pSystemTime->tm_hour == 0)
	{
	  hoursTemp = 12; //mid-night 12
	}
	else
		hoursTemp = pSystemTime->tm_hour;


	minutesTemp[0] = pSystemTime->tm_min / 10;
	minutesTemp[1] = pSystemTime->tm_min % 10;


	oled_drawPic(page, 0, 0,
	             (BYTE*)iconAlarmHourNumber_20_24[hoursTemp - 1],//该数组中的首个元素是“1”，hour值从1到12.所以要减1.
	             20, 24, DRAW_MODE_NORMAL);



	BYTE* iconColon = (BYTE*) iconAlarmColon;
	oled_drawPic(page, 20, 0,
	             iconColon,
	             6, 24, DRAW_MODE_NORMAL);


	oled_drawPic(page, 26, 0,
	             (BYTE*)iconAlarmMintuesNumbers_14_24[minutesTemp[0]],
	             14, 24, DRAW_MODE_NORMAL);
	oled_drawPic(page, 40, 0,
	             (BYTE*)iconAlarmMintuesNumbers_14_24[minutesTemp[1]],
	             14, 24, DRAW_MODE_NORMAL);


	BYTE* iconDistinguishAmPm = NULL;

	if(isAfternoon)
	{
		//pm
		iconDistinguishAmPm = (BYTE*) iconAlarm_10_8[2];
	}
	else
	{
		//am
		iconDistinguishAmPm = (BYTE*) iconAlarm_10_8[1];
	}

	oled_drawPic(page, SCREEN_WIDTH - 10, 9,
	             iconDistinguishAmPm,
	             10, 8, DRAW_MODE_NORMAL);

	// display date

	oled_drawPic(page, 12, SCREEN_HEIGHT - 12,
	             (BYTE*)iconSmallNumbers_8_13[(pSystemTime->tm_mon + 1) / 10 + 1],
	             8, 16, DRAW_MODE_NORMAL);
	oled_drawPic(page, 20, SCREEN_HEIGHT - 12,
	             (BYTE*)iconSmallNumbers_8_13[(pSystemTime->tm_mon + 1) % 10 + 1],
	             8, 16, DRAW_MODE_NORMAL);

	oled_drawPic(page, 28, SCREEN_HEIGHT - 12,
	             (BYTE*)iconSmallNumbers_8_13[11],
	             8, 16, DRAW_MODE_NORMAL);

	oled_drawPic(page, 36, SCREEN_HEIGHT - 12,
	             (BYTE*)iconSmallNumbers_8_13[pSystemTime->tm_mday / 10 + 1],
	             8, 16, DRAW_MODE_NORMAL);
	oled_drawPic(page, 44, SCREEN_HEIGHT - 12,
	             (BYTE*)iconSmallNumbers_8_13[pSystemTime->tm_mday % 10 + 1],
	             8, 16, DRAW_MODE_NORMAL);

}

//这个函数仅仅用于显示heart rate时的操作。。。。。 心率的数字格式与calorie，steps,distance的都不一样。后三种重新写函数
void OledDisplaySingle(BYTE* const pucDat, uint32_t ulTemp, bool blReplaceZeroWithHyphen)
{
	uint8_t ucTemp[4];
	uint8_t k = 0;

	//
	if (ulTemp == 0)
	{
		ucTemp[0] = ucTemp[1] = ucTemp[2] = 0;
	}
	else
	{
		ucTemp[0] = (ulTemp % 1000 / 100) + 2;
		ucTemp[1] = (ulTemp % 100 / 10) + 2;
		ucTemp[2] = (ulTemp % 10) + 2; //这里加2原因是：数组iconHeartNumbers_14_32前2个元素表示“-”“=”从第三个位置开始才是数字

	}

	int page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	if (pucDat > 0)
	{
//		for(j = 0; j < 4; j++) //图标
//		{
//			for(i = 0; i < 16; i++)
//			{
//				ucMenuRamBuf[j][i + g_tMenuInfo.mucUpDataArea] = pucDat[j][i];
//			}
//		}
		oled_drawPic(page, 0, 0,
		             pucDat,
		             22, 32, DRAW_MODE_TRANSPARENT);//这里写的是heart rate的坐标位置。函数OledDisplaySingle
		//在既要显示图片，又要显示数字的情况仅仅在heart
	}


	if(ulTemp == 0)
	{
		if(systemSetting.blTouchSensorEnabled == true)
		{
			//图标 ---
			for( k = 0; k < 3; k++)
				oled_drawPic(page, 22 + k * 14, 0,
				             (BYTE*)iconHeartNumbers_14_32[0],
				             14, 32, DRAW_MODE_TRANSPARENT);
		}
		else
			OledShowString(page, &FONT_ASCII_12x24, 0, 24, 4, "OFF");
	}
	else if(ulTemp > 99)
	{
		for( k = 0; k < 3; k++)
			oled_drawPic(page, 22 + k * 14, 0,
			             (BYTE*)iconHeartNumbers_14_32[ucTemp[k]],
			             14, 32, DRAW_MODE_TRANSPARENT);
	}
	else if((ulTemp > 9) && (ulTemp < 100))
	{
		for( k = 1; k < 3; k++)
			oled_drawPic(page, 22 + k * 14, 0,
			             (BYTE*)iconHeartNumbers_14_32[ucTemp[k]],
			             14, 32, DRAW_MODE_TRANSPARENT);

	}
	else if(ulTemp < 10)
	{
		oled_drawPic(page, 22 + 2 * 14, 0,
		             (BYTE*)iconHeartNumbers_14_32[ucTemp[2]],
		             14, 32, DRAW_MODE_TRANSPARENT);
	}

}


void CallBackSubMenuActivate(void)
{
	//
	clearScreen(false);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, "Hold to");
	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, "Activate");
}



void CallBackSubMenuTime(void)
{
	//
	clearScreen(false);
#if ENABLE_PAUSE_STOPWATCH
	int t = 0;

	if (systemStatus.blStopwatchEnabled)
	{
		BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

		time_t now = time(NULL);
		int s = now - lTimeBase + lTimeAccumulated;

		if (systemStatus.blStopwatchPaused)
			s = lTimeAccumulated;

		t = s % (1000 * 60);
//		if (s >= 1000 * 60)
//			t = 0;

		char sm[4], ss[3];
		sprintf(sm, "%03d", (t / 60));// + 123);
		sprintf(ss, "%02d", t % 60);

		OledShowString(page, &FONT_ASCII_12x24, 2, 2, 6, sm);

		//
		if (systemStatus.blStopwatchPaused)
		{
			if (pSystemTime->tm_sec % 2 == 0)
				return;
		}

		OledShowString(page, &FONT_DIGITAL_GULIMCHE_8x16, 0, 46, 2, ss);

		oled_drawPic(page, 51, 20,
		             (BYTE*) ICON_STOPWATCH,
		             7, 16, DRAW_MODE_TRANSPARENT);
	}
	else
#endif
	{
//		t = pSystemTime->tm_hour * 100 + pSystemTime->tm_min;

		OledDisplayTimeAndDate();
	}
}

void CallBackSubMenuAlarm(void)
{

	char alarmBuff[3] = {0};
	//
	clearScreen(false);
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;
	BYTE hour = pAlarmSetting->alarmTime.Hour;
	BYTE minute = pAlarmSetting->alarmTime.Minute;


	if(checkNotification(NOTIFY_SERVICE_Alarm) > 0)
	{
		// 闹钟被触发，仅显示一个闹钟图标
		oled_drawPic(page, 0, 0,
		             (BYTE*) iconAlarmTimeNow,
		             64, 32, DRAW_MODE_TRANSPARENT);
	}
	else
	{
		oled_drawPic(page, SCREEN_WIDTH - 10, 0,
		             (BYTE*) iconAlarm_10_8[0],
		             10, 8, DRAW_MODE_TRANSPARENT);

		//显示am/pm
		if(pAlarmSetting->alarmTime.Hour >= 12)
			oled_drawPic(page, SCREEN_WIDTH - 10,  9,
			             (BYTE*) iconAlarm_10_8[2],
			             10, 8, DRAW_MODE_TRANSPARENT);
		else
			oled_drawPic(page, SCREEN_WIDTH - 10,  9,
			             (BYTE*) iconAlarm_10_8[1],
			             10, 8, DRAW_MODE_TRANSPARENT);


		if(hour > 12)
			hour -= 12;
		else if(hour == 0)
			hour = 12;//应对午夜12点

		oled_drawPic(page, 0, 0,
		             (BYTE*) iconAlarmHourNumber_20_24[hour - 1],
		             20, 24, DRAW_MODE_TRANSPARENT);

		oled_drawPic(page, 20, 0,
		             (BYTE*) iconAlarmColon,
		             6, 24, DRAW_MODE_TRANSPARENT);

		sprintf(alarmBuff, "%02d", minute);

		oled_drawPic(page, 26, 0,
		             (BYTE*) iconAlarmMintuesNumbers_14_24[alarmBuff[0] - '0'],
		             14, 24, DRAW_MODE_TRANSPARENT);
		oled_drawPic(page, 40, 0,
		             (BYTE*) iconAlarmMintuesNumbers_14_24[alarmBuff[1] - '0'],
		             14, 24, DRAW_MODE_TRANSPARENT);

		for(int i = 0; i < 7; i++)
		{
			if(systemSetting.alarmSetting.enabled == false)
			{
				//如果没有使能闹钟，则把所有都设为禁止
				for(uint8_t m = 0; m < 7; m++)
					oled_drawPic(page, 0 + 9 * m, SCREEN_HEIGHT - 14,
					             (BYTE*) iconAlarmOnOff_9_16[m],
					             9, 16, DRAW_MODE_TRANSPARENT);

				return;
			}

			if(pAlarmSetting->weekday[i])
			{
				//on
				oled_drawPic(page, 0 + 9 * i, SCREEN_HEIGHT - 14,
				             (BYTE*) iconAlarmOnOff_9_16[7 + i],
				             9, 16, DRAW_MODE_TRANSPARENT);
			}
			else
			{
				//off
				oled_drawPic(page, 0 + 9 * i, SCREEN_HEIGHT - 14,
				             (BYTE*) iconAlarmOnOff_9_16[i],
				             9, 16, DRAW_MODE_TRANSPARENT);
			}
		}
	}
}

void CallBackSubMenuIncomingCall(void)
{
	unsigned char numberBuff[NOTIFY_SENDER_BUFFER_SIZE] = {0};
	uint8_t isCharacter = 0;//0表示数字，1表示可显示的ASCII，2表示不能显示的字符
	//
	clearScreen(false);

	//	oled_drawPic(g_tMenuInfo.mucUpDataArea / DISPLAY_X, 16, 0,
	//				 (BYTE*) ICON_INCOMING_CALL,
	//				 32, 32, DRAW_MODE_TRANSPARENT);

	// 显示号码时，第二行显示最后8个字符，第一行显示其他字符
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	int len = strlen(systemStatus.incomingCallNumber);

	if (len == 0)
	{
		if (systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] > 0)
			oled_drawPic(page, SCREEN_WIDTH - 15, 0,
			             (BYTE*) iconMessageAndCall[1],
			             15, 16, DRAW_MODE_NORMAL);
		else
			oled_drawPic(page, SCREEN_WIDTH - 15, 0,
			             (BYTE*) iconMessageAndCall[1],
			             15, 16, DRAW_MODE_NORMAL);
	}
	else
	{
		if (systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall] > 0)
			oled_drawPic(page, SCREEN_WIDTH - 15, 0,
			             (BYTE*) iconMessageAndCall[1],
			             15, 16, DRAW_MODE_NORMAL);
		else
			oled_drawPic(page, SCREEN_WIDTH - 15, 0,
			             (BYTE*) iconMessageAndCall[1],
			             15, 16, DRAW_MODE_NORMAL);

		//不可显示ascii
		for(uint8_t i = 0; i < len; i++)
		{
			if(systemStatus.incomingCallNumber[i] > 127)
			{
				isCharacter = 2;
				break;
			}
		}

		if(isCharacter == 2)
		{
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, "???");
			return;
		}

		if(systemStatus.incomingCallNumber[0] > 0x39 || systemStatus.incomingCallNumber[0] < 0x30)
			isCharacter = 1;//非数字



		if(isCharacter == 0)
		{
			for(uint8_t m = 0; m < len; m++)
			{
				if(systemStatus.incomingCallNumber[m] == '-')
				{
					numberBuff[m] = 16;
					continue;
				}

				numberBuff[m] = systemStatus.incomingCallNumber[m] - '0';
			}
		}

		if (len <= 8)
		{
			//右对齐
			if(isCharacter == 0)
			{
				//显示数字
				for(int8_t n = len - 1, k = 1; n >= 0; n--, k++)
					oled_drawPic(page, SCREEN_WIDTH - k * 8, SCREEN_HEIGHT - 16,
					             (BYTE*) macAddrdata[numberBuff[n]],
					             8, 16, DRAW_MODE_NORMAL);
			}
			else
				//显示ascii字符
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, systemStatus.incomingCallNumber);
		}
		else
		{
//			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, systemStatus.incomingCallNumber + (len - 8));
//
//			char firstLine[9] = {0};
//			memcpy(firstLine, systemStatus.incomingCallNumber, len - 8);
//			OledShowString(page, &FONT_ARIAL_8x16, 0, 8 * (16 - len), 0, firstLine);


			char firstLine[6] = {0};
			uint8_t firstLineNumber = len - 8;

			if(firstLineNumber > 6)
				firstLineNumber = 6;

			memcpy(firstLine, systemStatus.incomingCallNumber, firstLineNumber);

			if(isCharacter == 0)
			{
				//显示数字
//显示首行
				uint8_t firstLineLength = strlen(firstLine);

				for(uint8_t k = 0; k < firstLineLength; k++)
				{
					if(firstLine[k] == '-')
						firstLine[k] = 16;
					else
						firstLine[k] -= '0';

					oled_drawPic(page, 0 + k * 8, 0,
					             (BYTE*) macAddrdata[firstLine[k]],
					             8, 16, DRAW_MODE_NORMAL);
				}

//显示第二行
				for(uint8_t j = 0; j < 8; j++)
					oled_drawPic(page, 0 + j * 8, SCREEN_HEIGHT - 16,
					             (BYTE*) macAddrdata[numberBuff[len - 8 + j]],
					             8, 16, DRAW_MODE_NORMAL);
			}
			else
			{
				//显示ascii字符
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, firstLine);
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, systemStatus.incomingCallNumber + (len - 8));
			}

		}
	}
}


void CallBackSubMenuBattery(void)
{
#ifdef DEBUG
//	systemStatus.bBatteryRemaining = 5;
#endif

	static BYTE batteryChargingIndicator = 0;
	BYTE ci = 0;

	//
	clearScreen(false);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;


	/* convert remain battery to cpapcity index for draw.*/

	if(systemStatus.bBatteryRemaining < 11)
		ci = 0;
	else if((systemStatus.bBatteryRemaining > 10) && (systemStatus.bBatteryRemaining < 26))
		ci = 1;
	else if((systemStatus.bBatteryRemaining > 25) && (systemStatus.bBatteryRemaining < 51))
		ci = 2;
	else if((systemStatus.bBatteryRemaining > 50) && (systemStatus.bBatteryRemaining < 76))
		ci = 3;
	else
		ci = 4;

	if(systemStatus.blBatteryCharging != true)
	{
		//显示剩余电量
		switch(ci)
		{
			case 0:
				oled_drawPic(page, 0, 0,
				             (BYTE*) iconBattery64_32[0],
				             64, 32, DRAW_MODE_TRANSPARENT);
				break;

			case 1:
				oled_drawPic(page, 0, 0,
				             (BYTE*) iconBattery64_32[1],
				             64, 32, DRAW_MODE_TRANSPARENT);
				break;

			case 2:
				oled_drawPic(page, 0, 0,
				             (BYTE*) iconBattery64_32[2],
				             64, 32, DRAW_MODE_TRANSPARENT);
				break;

			case 3:
				oled_drawPic(page, 0, 0,
				             (BYTE*) iconBattery64_32[3],
				             64, 32, DRAW_MODE_TRANSPARENT);
				break;

			case 4:
				oled_drawPic(page, 0, 0,
				             (BYTE*) iconBattery64_32[4],
				             64, 32, DRAW_MODE_TRANSPARENT);
				break;

			default:
				break;

		}
	}
	else //[BG028] not necessary in else to check //if (systemStatus.blBatteryCharging)
	{
		//显示充电画面

		if(startCharging)
		{
			//首次进来获取剩余电量。
			batteryChargingIndicator = ci;
			startCharging = false;
		}

		oled_drawPic(page, 0, 0,
		             (BYTE*) iconBattery64_32[5 + batteryChargingIndicator],
		             64, 32, DRAW_MODE_TRANSPARENT);

		batteryChargingIndicator++;

		if(batteryChargingIndicator > 4)
			batteryChargingIndicator = ci;

	}
}

void CallBackSubMenuBleMAC(void)
{
	uint8_t saveBleMac[6] = {0};
	uint8_t modifyMac[12] = {0};
	uint8_t m = 0;

	memcpy(&saveBleMac, &BLE_DevChip.BLE_DeviceInfo[4], MAC_SIZE); //mac address 6

	for(int8_t k = 5; k >= 0; k--)
	{
		modifyMac[m] = (saveBleMac[k] & 0xf0) >> 4;
		m++;
		modifyMac[m] = (saveBleMac[k] & 0x0f);
		m++;
	}

	//
	clearScreen(false);
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	//这里想显示版本信息
	if(lastLongPressedisBLE > 0)
	{
		ShowVersion();
		lastLongPressedisBLE--;
		return;
	}
	
	if(BLE_STATE == BLE_STATE_CONNECTED)
	{
	// icon
	oled_drawPic(page, SCREEN_WIDTH - 24, 0,
	             (BYTE*) iconBLEConnected,
	             24, 16, DRAW_MODE_NORMAL);
	}
	else
	{
	// icon
	oled_drawPic(page, SCREEN_WIDTH - 15, 0,
	             (BYTE*) iconBLEDisconnected,
	             15, 16, DRAW_MODE_NORMAL);
	
	}

	for(uint8_t k = 0; k < 12; k++)
	{
		if(modifyMac[k] > 9)
		{
			//是字符
			if(k < 4)
				oled_drawPic(page, 0 + k * 8, 0, (BYTE*)macAddrdata[modifyMac[k]], 8, 16, DRAW_MODE_NORMAL);//第1行
			else
				oled_drawPic(page, 0 + (k - 4) * 8, 16, (BYTE*)macAddrdata[modifyMac[k] ], 8, 16, DRAW_MODE_NORMAL); //第2行
		}
		else
		{
			//数字
			if(k < 4)
				oled_drawPic(page, 0 + k * 8, 0, (BYTE*)macAddrdata[modifyMac[k]], 8, 16, DRAW_MODE_NORMAL);
			else
				oled_drawPic(page, 0 + (k - 4) * 8, 16, (BYTE*)macAddrdata[modifyMac[k]], 8, 16, DRAW_MODE_NORMAL);
		}
	}
}

void CallBackSubMenuDownload(void)
{
	if(blDuringDownload == false)
		return;

	clearScreen(false);
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	oled_drawPic(page, 0, 0,
	             (BYTE*)downloadProgress,
	             64, 32, DRAW_MODE_TRANSPARENT);
	OledDrawGuage(downloadPercentage, false);

//由于刷屏的速率由rtc时基决定，但是在一个时基内百分比变化大于1，所以下载看不到100%,所以只要达到96%就认为是100%
	if(downloadPercentage < 97)
	{
		oled_drawPic(page, 64 - 16 - 2, 32 - 14, (BYTE*)macAddrdata[downloadPercentage % 10], 8, 16, DRAW_MODE_NORMAL);
		oled_drawPic(page, 64 - 24 - 2, 32 - 14, (BYTE*)macAddrdata[downloadPercentage / 10], 8, 16, DRAW_MODE_NORMAL);
	}
	else
	{
		//100%
		oled_drawPic(page, 64 - 16 - 2, 32 - 14, (BYTE*)macAddrdata[0], 8, 16, DRAW_MODE_NORMAL);
		oled_drawPic(page, 64 - 24 - 2, 32 - 14, (BYTE*)macAddrdata[0], 8, 16, DRAW_MODE_NORMAL);
		oled_drawPic(page, 64 - 32 - 2, 32 - 14, (BYTE*)macAddrdata[1], 8, 16, DRAW_MODE_NORMAL);
	}
}

void ShowVersion(void)
{
	uint8_t MCUVersionBuffer[8] = {0};
	uint8_t BLEVersionBuffer[8] = {0};

	//
	clearScreen(false);
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	//MCU version data
	MCUVersionBuffer[0] = 19; //M
	MCUVersionBuffer[1] = 12; //C
	MCUVersionBuffer[2] = 20; //U
	MCUVersionBuffer[3] = 21; //:

	if(APP_FW_VER_M > 9)
	{
		MCUVersionBuffer[4] = APP_FW_VER_M / 10; //
		MCUVersionBuffer[5] = APP_FW_VER_M % 10; //
		MCUVersionBuffer[6] = 17;//.
		MCUVersionBuffer[7] = APP_FW_VER_S > 10 ? APP_FW_VER_S / 10 : APP_FW_VER_S;
	}
	else
	{
		MCUVersionBuffer[4] = APP_FW_VER_M; //
		MCUVersionBuffer[5] = 17; //.
		MCUVersionBuffer[6] = APP_FW_VER_S / 10;
		MCUVersionBuffer[7] = APP_FW_VER_S % 10 ;
	}

	//BLE version data
	BLEVersionBuffer[0] = 11; //B
	BLEVersionBuffer[1] = 18; //L
	BLEVersionBuffer[2] = 14; //E
	BLEVersionBuffer[3] = 21; //:

	if(BLE_DevChip.BLE_Device.FW_VER1 > 9)
	{
		BLEVersionBuffer[4] = BLE_DevChip.BLE_Device.FW_VER1 / 10; //
		BLEVersionBuffer[5] = BLE_DevChip.BLE_Device.FW_VER1 % 10; //
		BLEVersionBuffer[6] = 17;//.
		BLEVersionBuffer[7] = BLE_DevChip.BLE_Device.FW_VER2 > 10 ? BLE_DevChip.BLE_Device.FW_VER2 / 10 : BLE_DevChip.BLE_Device.FW_VER2;
	}
	else
	{
		BLEVersionBuffer[4] = BLE_DevChip.BLE_Device.FW_VER1; //
		BLEVersionBuffer[5] = 17; //.
		BLEVersionBuffer[6] = BLE_DevChip.BLE_Device.FW_VER2 / 10;
		BLEVersionBuffer[7] = BLE_DevChip.BLE_Device.FW_VER2 % 10 ;
	}

	for(uint8_t k = 0; k < 8; k++)
	{
		oled_drawPic(page, 0 + k * 8, 0, (BYTE*)macAddrdata[MCUVersionBuffer[k]], 8, 16, DRAW_MODE_NORMAL);
		oled_drawPic(page, 0 + k * 8, 16, (BYTE*)macAddrdata[BLEVersionBuffer[k]], 8, 16, DRAW_MODE_NORMAL);
	}
}
//void CallBackSubMenuIntenseUV(void)
//{
//	//
//	clearScreen(false);
//
//	oled_drawPic(g_tMenuInfo.mucUpDataArea / DISPLAY_X, 11, 4,
//				 (BYTE*) ICON_BATTERY_EMPTY,
//				 40, 24, DRAW_MODE_TRANSPARENT);
//
//}

const char timerChar[4] = { '-', '\\', '|', '/' };
extern uint8_t heart_display;
void CallBackSubMenuHeartRate(void)
{
#if 0
	//
	clearScreen(false);

	int page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	if (systemStatus.iAutoHRWarning > 0)
	{
		// 提示用户当前是 在自动开关ppg的设置下
		oled_drawPic(page, 0, 0,
		             (BYTE*) oled_HeartRate_Image,
		             16, 32, DRAW_MODE_TRANSPARENT);

		OledShowString(page, &FONT_ASCII_12x24, 0, 16, 4, "AUTO");

		//
		systemStatus.iAutoHRWarning --;
		return;
	}

	//
	if (systemStatus.blHRSensorOn)
	{
		//if (iHeartRate.component.heart > 0)
		if(systemStatus.blHeartBeatLock == true)
			OledDisplaySingle((BYTE*) oled_HeartRate_Image, heart_display, true);
		else
		{
			static BYTE timerWaitForHeartRate = 0;

			//
			oled_drawPic(page, 0, 0,
			             (BYTE*) oled_HeartRate_Image,
			             16, 32, DRAW_MODE_TRANSPARENT);

			//
			char buff[4] = "---";
			buff[timerWaitForHeartRate] = '=';
//			BYTE tt = timerWaitForHeartRate;
//			buff[0] = timerChar[tt];
//			tt++;
//			tt %= 4;
//			buff[1] = timerChar[tt];
//			tt++;
//			tt %= 4;
//			buff[2] = timerChar[tt];
//			buff[3] = 0;
			timerWaitForHeartRate ++;
			timerWaitForHeartRate %= 3;

			OledShowString(page, &FONT_ASCII_12x24, 2, 20, 4, buff);
		}
	}
	else
		OledDisplaySingle((BYTE*) oled_HeartRate_Image, 0, true);

#endif
	clearScreen(false);

	int page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	if (systemStatus.iAutoHRWarning > 0)
	{
		// 提示用户当前是 在自动开关ppg的设置下
		oled_drawPic(page, 0, 0,
		             (BYTE*) iconHeart_22_32,
		             22, 32, DRAW_MODE_TRANSPARENT);

		OledShowString(page, &FONT_ASCII_12x24, 0, 16, 4, "AUTO");

		//
		systemStatus.iAutoHRWarning --; //它主要是显示“AUTO”保持一段时间。
		return;
	}


	if (systemStatus.blHRSensorOn)
	{
//		iHeartRate.component.heart = 34;//测试用

		//if (iHeartRate.component.heart > 0)
		if(systemStatus.blHeartBeatLock == true)
		{
			OledDisplaySingle((BYTE*) iconHeart_22_32, heart_display, true);//iHeartRate.component.heart
			waitValidHeartrate = false;
		}
		else
		{
			//心率等于0
			static BYTE timerWaitForHeartRate = 0;

			//
			oled_drawPic(page, 0, 0,
			             (BYTE*) iconHeart_22_32,
			             22, 32, DRAW_MODE_TRANSPARENT);

			//
			uint8_t buff[4] = {0, 0, 0};
			buff[timerWaitForHeartRate] = 1;

			timerWaitForHeartRate ++;
			timerWaitForHeartRate %= 3;

			for(uint8_t m = 0; m < 3; m++)
				oled_drawPic(page, 22 + m * 14, 0,
				             (BYTE*) iconHeartNumbers_14_32[buff[m]],
				             14, 32, DRAW_MODE_TRANSPARENT);

		}
	}
	else
		OledDisplaySingle((BYTE*) iconHeart_22_32, 0, true); //心率没打开就显示为0



}

void CallBackSubMenuCalories(void)
{


#ifdef DEBUG
	//just for test 2015年5月19日11:32:42
//	iCalories = 9;
#endif

	if(checkNotification(NOTIFY_SERVICE_Calorie_Accomplish) > 0)
	{
		blAccomplishGoalShine = true;
	}

	//
	clearScreen(false);

	UINT value = iCalories;
	BYTE* icon = (BYTE*) iconCalories_32_13;

#if ENABLE_PAUSE_STOPWATCH

	if (systemStatus.blStopwatchEnabled)
	{
		if (systemStatus.blStopwatchPaused)
			value = iCaloriesAccumulated;
		else
			value = iCalories - iCaloriesBase + iCaloriesAccumulated;

		icon = (BYTE*) ICON_CALORIES_STOPWATCH;
	}

#endif

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;


	// -------------------------------------------------------------
	// icon
	oled_drawPic(page, 32, 0,
	             icon,
	             32, 16, DRAW_MODE_TRANSPARENT);


	// -------------------------------------------------------------
	// value
	if (systemStatus.blStopwatchEnabled && systemStatus.blStopwatchPaused)
	{
		if (pSystemTime->tm_sec % 2 == 0)
			return;
	}

	// 设置上限
	if(value > 100000)
		value = 100000 - 1;

	OledDisplayNumber(value, 0, false);

	//
//	if (systemStatus.blStopwatchEnabled)
//	{
//		oled_drawPic(page, 1, 21,
//			 (BYTE*)ICON_STOPWATCH,
//			 7, 18, DRAW_MODE_TRANSPARENT);
//	}

	// -------------------------------------------------------------
	// process
	BYTE maxProcess	= 100;

	int w = (int)((float)value / (float)systemSetting.userGoals[GOAL_CALORIES] * (float)maxProcess);

	if (w < 0)
		w = 0;
	else if (w >= 100)
		w = 100;

	OledDrawGuage(w, true);
}

void CallBackSubMenuStep(void)
{
#ifdef DEBUG
	//just for test 2015年5月19日11:32:42
	//iSteps = 6;
#endif

	if(checkNotification(NOTIFY_SERVICE_Step_Accomplish) > 0)
	{
		blAccomplishGoalShine = true;
	}

	//
	clearScreen(false);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	UINT value = iSteps;
#if BG009   //[BG009] debug show active_level_max;
        value = active_level_max;
#else

	BYTE* icon = (BYTE*) iconStep;

#if ENABLE_PAUSE_STOPWATCH

	if (systemStatus.blStopwatchEnabled)
	{
		if (systemStatus.blStopwatchPaused)
			value = iStepsAccumulated;
		else
			value = iSteps - iStepsBase + iStepsAccumulated;

		icon = (BYTE*) ICON_STEPS_STOPWATCH;
	}

#endif
#endif
	// -------------------------------------------------------------
	// icon

	oled_drawPic(page, 31, 0,
	             icon,
	             32, 16, DRAW_MODE_NORMAL);

	// -------------------------------------------------------------
	// value
	if (systemStatus.blStopwatchEnabled && systemStatus.blStopwatchPaused)
	{
		if (pSystemTime->tm_sec % 2 == 0)
			return;
	}

	// 设置上限
	if(value > 100000)
		value = 100000 - 1;

	OledDisplayNumber(value, 0, false); //显示步数。

	//
//	if (systemStatus.blStopwatchEnabled)
//	{
//		oled_drawPic(page, 0, 0,
//			 (BYTE*)ICON_STEPS_STOPWATCH,
//			 16, 32, DRAW_MODE_TRANSPARENT);
//
////		oled_drawPic(page, 1, 21,
////			 (BYTE*)ICON_STOPWATCH,
////			 7, 18, DRAW_MODE_TRANSPARENT);
//	}
//	else
//	{
//		oled_drawPic(page, 0, 0,
//			 (BYTE*)oled_Step_Image,
//			 16, 32, DRAW_MODE_TRANSPARENT);
//	}

	// -------------------------------------------------------------
	// process
	BYTE maxProcess	= 100;

	int w = (int)((float)value / (float)systemSetting.userGoals[GOAL_STEPS] * (float)maxProcess);

	if (w < 0)
		w = 0;
	else if (w >= 100)
		w = 100;

//	oled_drawHortLine(page, 30, 0, w, 0);
	OledDrawGuage(w, true);
}

void CallBackSubMenuDistance(void)
{

	if(checkNotification(NOTIFY_SERVICE_Distance_Accomplish) > 0)
	{
		blAccomplishGoalShine = true;
	}

	//
	clearScreen(false);
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;


#ifdef DEBUG
//	pUserProfile->unit = 0;
//	iDistance = 6000;
//	systemSetting.userGoals[GOAL_DISTANCE] = 1200;//DEFAULT_GOAL_DISTANCE
#endif

	UINT value = iDistance;
	BYTE* icon = (BYTE*) iconDistance;

#if ENABLE_PAUSE_STOPWATCH

	if (systemStatus.blStopwatchEnabled)
	{
		if (systemStatus.blStopwatchPaused)
			value = iDistanceAccumulated;
		else
			value = iDistance - iDistanceBase + iDistanceAccumulated;

		icon = (BYTE*) ICON_DISTANCE_STOPWATCH;
	}

	// value
	if (systemStatus.blStopwatchEnabled && systemStatus.blStopwatchPaused)
	{
		if (pSystemTime->tm_sec % 2 == 0)
			return;
	}

#endif



	float d = value;
//	if (pUserProfile->unit == UNIT_ENGLISH)
//		 d /= 1.0936; // yard


	// 设置上限,这个上限仅仅由屏的显示位数决定，最大显示99.9km
	if (d >= 100000)
		d = 100000 - 1;

	if(pUserProfile->unit == UNIT_METRIC)
	{
		OledDisplayNumber((float)(d / 1000.0), 1, true);
		//km
		oled_drawPic(page, SCREEN_WIDTH - 15, SCREEN_HEIGHT - 15,
		             (BYTE*)iconKmMeter[0],
		             15, 18, DRAW_MODE_TRANSPARENT);
	}
	else //if(pUserProfile->unit == UNIT_ENGLISH)
	{

		d = (d / 1000.0) * 0.621371;  //把km转换成英里
		OledDisplayNumber(d, 1, true);

		//mi
		oled_drawPic(page, SCREEN_WIDTH - 15, SCREEN_HEIGHT - 15,
		             (BYTE*)iconKmMeter[1],
		             15, 18, DRAW_MODE_TRANSPARENT);

	}

	// icon
	oled_drawPic(page, 32, 0,
	             icon,
	             32, 16, DRAW_MODE_TRANSPARENT);

	// -------------------------------------------------------------
	// process
	BYTE maxProcess	= 100;

	int w = (int)((float)value / (float)systemSetting.userGoals[GOAL_DISTANCE] * (float)maxProcess);

	if (w < 0)
		w = 0;
	else if (w >= 100)
		w = 100;


	OledDrawGuage(w, true);

}

void CallBackSubMenuUltraViolet(void)
{

	//
	clearScreen(false);

//	if (checkNotification(NOTIFY_SERVICE_Intense_UV) > 0)
//	{
//		oled_drawPic(g_tMenuInfo.mucUpDataArea / DISPLAY_X, SCREEN_WIDTH - 24, 0,
//		             (BYTE*) ICON_INTENSE_UV,
//		             24, 32, DRAW_MODE_TRANSPARENT);

//		OledDisplaySingle(NULL, bUltraViolet, false);
//		OledDisplayUV(bUltraViolet, (BYTE*)iconUVAlert);
//	}
//	else
	{
//		OledDisplaySingle((BYTE*) oled_UltraViolet_Image, bUltraViolet, false);
#if 0   //FALL_DETECT_SUPPORT && BGXXX  //move the display to CallBackSubMenuTesting()
                //[BG008] for debug display result by UV submenu
                extern uint8_t FD_result;
                uint8_t display_u8;
                display_u8=FD_result%100;
		OledDisplayUV(display_u8, (BYTE*)iconUV);
#else
		OledDisplayUV(bUltraViolet, (BYTE*)iconUV);
#endif
	}
}

void CallBackSubMenuBodyTemper(void)
{
	//
//	clearScreen(false);

#ifdef DEBUG
//	fSkinTemperature = -0.4;
	//	t = 135.7;//for test;
#endif
	float t = fSkinTemperature;


	if (pUserProfile->unit == UNIT_ENGLISH)
		t = t * 9 / 5 + 32; //这个就是把摄氏度转换成华氏度 。 F = C*9/5+32;


	OledDisplayDecimals((BYTE*)iconTemperature_15_16[0], (int16_t)( t * 10), skinTempFlag);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	//无论正负，温度单位都存在
	if (pUserProfile->unit == UNIT_ENGLISH)
	{
		//华氏温度
		oled_drawPic(page, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 15,
		             (BYTE*)iconTemperatureUnit_12_16[0],
		             12, 16, DRAW_MODE_TRANSPARENT);
	}
	else
	{
		oled_drawPic(page, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 15,
		             (BYTE*)iconTemperatureUnit_12_16[1],
		             12, 16, DRAW_MODE_TRANSPARENT);
	}

//	else
//	{
//		//不考虑零下的温度。
//		if (pUserProfile->unit == UNIT_ENGLISH)
//		{
//			oled_drawPic(page, 52, 4,
//			             (BYTE*)ICON_UNIT_SMALL_FAHRENHEIT,
//			             10, 16, DRAW_MODE_TRANSPARENT);
//		}
//		else
//		{
//			oled_drawPic(page, 52, 4,
//			             (BYTE*)ICON_UNIT_SMALL_CELSIUS,
//			             10, 16, DRAW_MODE_TRANSPARENT);
//		}
//	}
}

void CallBackSubMenuAmbientTemper(void)
{
	//
//	clearScreen(false);

#ifdef DEBUG
//	fAmbientTemperature = -20.4;
#endif

	float t = fAmbientTemperature;


	if (pUserProfile->unit == UNIT_ENGLISH)
		t = t * 9 / 5 + 32;

	OledDisplayDecimals((BYTE*)iconTemperature_15_16[1], (int16_t)( t * 10), ambTempFlag);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;


	//无论正负，温度单位都存在
	if (pUserProfile->unit == UNIT_ENGLISH)
	{
		//华氏温度
		oled_drawPic(page, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 15,
		             (BYTE*)iconTemperatureUnit_12_16[0],
		             12, 16, DRAW_MODE_TRANSPARENT);
	}
	else
	{
		oled_drawPic(page, SCREEN_WIDTH - 12, SCREEN_HEIGHT - 15,
		             (BYTE*)iconTemperatureUnit_12_16[1],
		             12, 16, DRAW_MODE_TRANSPARENT);
	}

//	else
//	{
//		if (pUserProfile->unit == UNIT_ENGLISH)
//		{
//			oled_drawPic(page, 52, 4,
//			             (BYTE*)ICON_UNIT_SMALL_FAHRENHEIT,
//			             10, 16, DRAW_MODE_TRANSPARENT);
//		}
//		else
//		{
//			oled_drawPic(page, 52, 4,
//			             (BYTE*)ICON_UNIT_SMALL_CELSIUS,
//			             10, 16, DRAW_MODE_TRANSPARENT);
//		}
//	}
}


void ShowNotification(BYTE page, short eventIndex, short margin, short interval, short count, BYTE* icon)
{
	oled_drawPic(page, margin + (interval - 16) / 2 + interval * eventIndex, 2, icon, 16, 16, 0);

	if (count > 9)
	{
		oled_drawPic(page, margin + (interval - 14) / 2 + interval * eventIndex, 18,
		             (BYTE*)FONT_DIGITAL_ARIAL_7x14[9],
		             7, 16, DRAW_MODE_TRANSPARENT);
		oled_drawPic(page, margin + (interval) / 2 + interval * eventIndex, 18,
		             (BYTE*)FONT_DIGITAL_ARIAL_7x14[10],
		             7, 16, DRAW_MODE_TRANSPARENT);
	}
	else
		oled_drawPic(page, margin + (interval - 7) / 2 + interval * eventIndex, 18,
		             (BYTE*)FONT_DIGITAL_ARIAL_7x14[count],
		             7, 16, DRAW_MODE_TRANSPARENT);
}

// 显示所有事件信息，不包括 incoming call, battery，这两个有单独的菜单
void CallBackSubMenuNotifications(void)
{
	//
	clearScreen(false);

//	bool blLowBatteryFlag;
//	bool blBatteryCharging;
//	bool blIncomingCall;	// 当前是否有正在呼叫的电话
//	BYTE bMissingCalls;		// 未接电话数量
//	BYTE bUnreadMessages;	// 未读信息数量

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

//	if (systemStatus.notifyEvents[NOTIFY_SERVICE_IncomingCall])
//	{
//		// 如果有电话，则唯一显示此提醒
//		oled_drawPic(page, 16, 0, (BYTE*)ICON_INCOMING_CALL, 32, 32, 0);
//	}
//	else
	{
#define MAX_SHOWED_NOTIFICATIONS	3

		// 其他提醒一起显示，最多同时显示 3个事件 + 其他事件汇总
		// 其中 电量，电话，消息 为三个主要的事件，优先
		BYTE types = getNotifications(true);

		BYTE displayedTypes = types;

		if (types > 3)
			displayedTypes = MAX_SHOWED_NOTIFICATIONS;

		short interval = (SCREEN_WIDTH - 4) / displayedTypes;
		short margin = (SCREEN_WIDTH - interval * displayedTypes) / 2;
		short eventIndex = 0;
		short count = 0;

//		if (systemStatus.blLowBatteryFlag && !systemStatus.blBatteryCharging)
//		{
//			// 电量不足且未在充电
//			oled_drawPic(page, margin + interval * eventIndex, 3, (BYTE*)ICON_LOW_BATTERY, 16, 16, 0);
//			eventIndex++;
//		}

		if (types > MAX_SHOWED_NOTIFICATIONS)
		{
			// 显示多余的通知
			short pages = types / MAX_SHOWED_NOTIFICATIONS + (types % MAX_SHOWED_NOTIFICATIONS == 0 ? 0 : 1);

			for (int p = 0; p < pages; p++)
				Oled_DrawPixel(page, 63 - 2 * p, 31, 0);
		}

		count = systemStatus.notifyEvents[NOTIFY_SERVICE_MissedCall];

		if (count > 0)
		{
			ShowNotification(page, eventIndex, margin, interval, count, (BYTE*) ICON_MISSED_CALL);
//			oled_drawPic(page, margin + (interval - 16) / 2 + interval * eventIndex, 3, (BYTE*)ICON_MISSED_CALL, 16, 16, DRAW_MODE_TRANSPARENT);
//
//			if (count > 9)
//			{
//				oled_drawPic(page, margin + (interval - 14) / 2 + interval * eventIndex, 19,
//						 (BYTE*)FONT_DIGITAL_ARIAL_7x14[9],
//						 7, 16, DRAW_MODE_TRANSPARENT);
//				oled_drawPic(page, margin + (interval) / 2 + interval * eventIndex, 19,
//						 (BYTE*)FONT_DIGITAL_ARIAL_7x14[10],
//						 7, 16, DRAW_MODE_TRANSPARENT);
//			}
//			else
//				oled_drawPic(page, margin + (interval - 7) / 2 + interval * eventIndex, 19,
//						 (BYTE*)FONT_DIGITAL_ARIAL_7x14[count],
//						 7, 16, DRAW_MODE_TRANSPARENT);

			if ((++eventIndex) >= MAX_SHOWED_NOTIFICATIONS)
				return;
		}

		count = systemStatus.notifyEvents[NOTIFY_SERVICE_Email];

		if (count > 0)
		{
			ShowNotification(page, eventIndex, margin, interval, count, (BYTE*) ICON_MESSAGES);
//			oled_drawPic(page, margin + (interval - 16) / 2 + interval * eventIndex, 3, (BYTE*)ICON_MESSAGES, 16, 16, 0);
//
//			if (count > 9)
//			{
//				oled_drawPic(page, margin + (interval - 14) / 2 + interval * eventIndex, 19,
//						 (BYTE*)FONT_DIGITAL_ARIAL_7x14[9],
//						 7, 16, DRAW_MODE_TRANSPARENT);
//				oled_drawPic(page, margin + (interval) / 2 + interval * eventIndex, 19,
//						 (BYTE*)FONT_DIGITAL_ARIAL_7x14[10],
//						 7, 16, DRAW_MODE_TRANSPARENT);
//			}
//			else
//				oled_drawPic(page, margin + (interval - 7) / 2 + interval * eventIndex, 19,
//						 (BYTE*)FONT_DIGITAL_ARIAL_7x14[count],
//						 7, 16, DRAW_MODE_TRANSPARENT);

			if ((++eventIndex) >= MAX_SHOWED_NOTIFICATIONS)
				return;
		}

		count = systemStatus.notifyEvents[NOTIFY_SERVICE_Schedule];

		if (count > 0)
		{
			ShowNotification(page, eventIndex, margin, interval, count, (BYTE*) ICON_CALENDAR);

			if ((++eventIndex) >= MAX_SHOWED_NOTIFICATIONS)
				return;
		}

		count = systemStatus.notifyEvents[NOTIFY_SERVICE_News];

		if (count > 0)
		{
			ShowNotification(page, eventIndex, margin, interval, count, (BYTE*) ICON_NEWS);

			if ((++eventIndex) >= MAX_SHOWED_NOTIFICATIONS)
				return;
		}
	}
}

void CallBackSubMenuSMS(void)
{
	uint8_t numberBuff[NOTIFY_SENDER_BUFFER_SIZE] = {0};
	uint8_t isCharacter  = 0;//0表示数字，1，表示ASCII字符，2表示非ASCII.

	clearScreen(false);

//	oled_drawPic(g_tMenuInfo.mucUpDataArea / DISPLAY_X, 16, 0,
//				 (BYTE*) ICON_INCOMING_CALL,
//				 32, 32, DRAW_MODE_TRANSPARENT);

	// 显示号码时，第二行显示最后8个字符，第一行显示其他字符
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	// icon
	oled_drawPic(page, SCREEN_WIDTH - 15, 0,
	             (BYTE*) iconMessageAndCall[0],
	             15, 16, DRAW_MODE_NORMAL);


	int len = strlen(systemStatus.latestSmsNumber);

	for(uint8_t i = 0; i < len; i++)
	{
		if(systemStatus.latestSmsNumber[i] > 127)
		{
			isCharacter = 2;//判断一下有没有非ascii字符
			break;
		}
	}

	if(isCharacter == 2)
	{
		//非ASCII，显示成？,并退出
		OledShowString(page, &FONT_ARIAL_8x16, 0, 0, SCREEN_HEIGHT - 16, "???");
		return;
	}



	if(systemStatus.latestSmsNumber[0] > 0x39 || systemStatus.latestSmsNumber[0] < 0x30)
	{
		//说明过来的是字符，非电话号码
		isCharacter = 1;
	}

	if(isCharacter == 0)
	{
		for(uint8_t i = 0; i < len; i++)
		{
			if(systemStatus.latestSmsNumber[i] == '-')
			{
				numberBuff[i] = 16;
				continue;
			}

			numberBuff[i] = systemStatus.latestSmsNumber[i] - '0';
		}
	}

	//这里用来测试显示短信者的姓名。
//	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, SCREEN_HEIGHT-16, "abcdefg");

//	unsigned char bufCharactertest[8] = {0};
//	sprintf(bufCharactertest, "%s", "ABCD");
//	uint8_t length = strlen(bufCharactertest);
//	for(uint8_t i = 0; i < length; i++)
//	{
//		bufCharactertest[i] -= 65;
//		oled_drawPic(page, 0+8*i, SCREEN_HEIGHT - 16,
//		             (BYTE*) charGulim_8_16[bufCharactertest[i]],
//		             8, 16, DRAW_MODE_NORMAL);
//	}



	if (len <= 8)
	{
//		if (len == 0)
//			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, "unknown");
//		else
//			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, systemStatus.latestSmsNumber);

		//右对齐
		if(isCharacter == 0)
		{
			if(len != 0)
				for(int8_t n = len - 1, k = 1; n >= 0; n--, k++)
					oled_drawPic(page, SCREEN_WIDTH - k * 8, SCREEN_HEIGHT - 16,
					             (BYTE*) macAddrdata[numberBuff[n]],
					             8, 16, DRAW_MODE_NORMAL);
		}
		else if(isCharacter == 1)
		{
			if(len != 0)
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, SCREEN_HEIGHT - 16, systemStatus.latestSmsNumber);
		}

	}
	else
	{
//		OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, systemStatus.latestSmsNumber + (len - 8));
//
//		char firstLine[9] = {0};
//		memcpy(firstLine, systemStatus.latestSmsNumber, len - 8);
//		OledShowString(page, &FONT_ARIAL_8x16, 0, 8 * (16 - len), 0, firstLine);

		char firstLine[6] = {0};
		uint8_t firstLineNumber = len - 8;

		if(firstLineNumber > 6)
			firstLineNumber = 6;

		memcpy(firstLine, systemStatus.latestSmsNumber, firstLineNumber);

		if(isCharacter == false)
		{
//显示首行
			uint8_t firstLinelength = strlen(firstLine);

			for(uint8_t k = 0; k < firstLinelength; k++)
			{
				if(firstLine[k] == '-')
					firstLine[k] = 16;
				else
					firstLine[k] -= '0';

				oled_drawPic(page, 0 + k * 8, 0,
				             (BYTE*) macAddrdata[firstLine[k]],
				             8, 16, DRAW_MODE_NORMAL);
			}

//显示第二行
			for(uint8_t j = 0; j < 8; j++)
				oled_drawPic(page, 0 + j * 8, SCREEN_HEIGHT - 16,
				             (BYTE*) macAddrdata[numberBuff[len - 8 + j]],
				             8, 16, DRAW_MODE_NORMAL);
		}
		else if(isCharacter == 1)
		{
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, firstLine);//显示首行
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, SCREEN_HEIGHT - 16, systemStatus.latestSmsNumber + (len - 8));//显示第二行
		}
	}
}

#if 0
void CallBackSubMenuTesting(void)
{
	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;
	char buff[12];
	static char loopcount = 0, insideCount = 0;

	switch(loopcount)
	{

		case 0:
			clearScreen(false);
			extern union _BLE_CHIP BLE_DevChip;
			sprintf(buff, "MAC:%04x", (uint16_t)BLE_DevChip.BLE_Device.unique0);

			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, buff);
			insideCount++;

			if(insideCount > 3)
			{
				insideCount = 0;
				loopcount = 1;
			}

			break;

		case 1:
			clearScreen(true);
			insideCount++;

			if(insideCount > 3)
			{
				insideCount = 0;
				loopcount = 2;
			}

			break;

		case 2:
			clearScreen(false);
			char error = 0, error_buff[7];

			for(int i = 0; i < 7; i++)
				error_buff[i] = ' ';


//			extern bool AMB_TMP_ONLINE;
//			extern bool SKIN_TMP_ONLINE;
//			extern bool UV_SENSOR_ONLINE;
//			extern bool afe_online;
//			extern bool BLE_ONLINE;
//			extern bool MEMS_ONLINE;


			if(AMB_TMP_ONLINE == false)
			{
				error_buff[error++] = 'A';
			}

			if(SKIN_TMP_ONLINE == false)
			{
				error_buff[error++] = 'S';
			}

			if(UV_SENSOR_ONLINE == false)
			{
				error_buff[error++] = 'U';
			}

#if (AFE44x0_SUPPORT==1)
			AFE44xx_PowerOn_Init();
			if(afe_online == false)
			{
				error_buff[error++] = 'P';
			}
#endif

			if(BLE_ONLINE == false)
			{
				error_buff[error++] = 'B';
			}

			if(MEMS_ONLINE == false)
			{
				error_buff[error++] = 'M';
			}

			if(systemStatus.blFlashOnline == false)
			{
				error_buff[error++] = 'F';
			}

			if(error == 0)
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, "ALL OK");
			else
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, error_buff);

			insideCount++;

			if(insideCount > 1)
			{
				insideCount = 0;
				loopcount = 3;
			}

			break;

		case 3:

			clearScreen(false);
			extern int16_t read_IR_reg;
                        extern uint8_t AMB_uA;
			sprintf(buff, "PPG %d\n%d", AMB_uA, read_IR_reg);
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, buff);

			insideCount++;

			if(insideCount > 10)
			{
				insideCount = 0;
				loopcount = 3;
#if (AFE44x0_SUPPORT==1)
				AFE44xx_Shutoff();
#endif
			}

			if(systemStatus.blHRSensorOn == false)
			{
				insideCount = 0;
				loopcount = 4;
			}

			break;

		case 4:
			clearScreen(false);
			extern uint16_t SkinTouchVal, touchValues;
			sprintf(buff, "S %d\nT %d", SkinTouchVal, touchValues);
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, buff);

			insideCount++;

			if(insideCount > 10)
			{
				insideCount = 0;
				loopcount = 5;
			}

			break;

		case 5:
			clearScreen(false);
			extern volatile float fSkinTemperature, fAmbientTemperature;
			sprintf(buff, "B%2.1f\nA%2.1f", fSkinTemperature, fAmbientTemperature);
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, buff);

			insideCount++;

			if(insideCount > 1)
			{
				insideCount = 0;
				loopcount = 6;
			}

			break;

		case 6:

			clearScreen(false);
#if (VIBRATION_SUPPORT==1)
			VibrateCon(BuzzAlert1000ms, 1, 1);
#endif
			sprintf(buff, "Motor");
			OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, buff);
			loopcount = 7;
			insideCount = 0;
			break;

		case 7:
			insideCount++;

			if(insideCount > 1)
			{
				insideCount = 0;
				loopcount = 0;
			}

			break;

		default:
			break;
	}

}

#else

char test_menu_loopcount = 0;

//void CallBackSubMenuTesting(void)
//{
//	static char insideCount = 0;
//
//	switch(test_menu_loopcount)
//	{
//
//		case 0:
//			clearScreen(true);
//			insideCount++;
//
//			if(insideCount > 5)
//			{
//				insideCount = 0;
//				test_menu_loopcount = 1;
//			}
//
//			LED_ON();
//			break;
//
//		case 1:
//			OLEDOff();
//			LockScreen();
//#if (VIBRATION_SUPPORT==1)
//			VibrateCon(BuzzAlert1000ms, 1, 1);
//#endif
//test_menu_loopcount = 2;
//
//			break;
//
//
//		default:
//			//test_menu_loopcount++;
//			//if(test_menu_loopcount>5)
//                              test_menu_loopcount = 0;
//			break;
//	}
//
//}

#if BGXXX>0
int itest=0;
uint32_t utest=0;
float ftest=0.0;
#endif

void CallBackSubMenuTesting(void)
{
//	static char insideCount = 0;
        char error_buff[9*2];   //[BGXXX]
#if BGXXX>0
        switch(0)
#else
	switch(test_menu_loopcount)
#endif
	{
		case 0:
		{
			char idx = 0, error = 0; //, error_buff[9]; //[BGXXX] move declare to begin.

			for(int i = 0; i < 8; i++)
				error_buff[i] = ' ';

//			systemStatus.blAmbTempSensorOnline = false;
//			systemStatus.blUVSensorOnline = false;
//			systemStatus.blHRSensorOnline = false;
//			systemStatus.blBleOnline = false;
//			systemStatus.blAccelSensorOnline = false;
//			systemStatus.blFlashOnline = false;
			if(systemStatus.blAmbTempSensorOnline == false)
			{
				error_buff[idx] = 'T';
				error++;
			}

			idx++;

//			if(systemStatus.blSkinTempSensorOnline == false)
//			{
//				error_buff[idx] = 'S';
//				error++;
//			}
//			idx++;

			if(systemStatus.blUVSensorOnline == false)
			{
				error_buff[idx] = 'U';
				error++;
			}

			idx++;

#if BATTERY_LIFE_OPTIMIZATION
			systemStatus.blHRSensorTempEnabled = false;
#endif
#if BGXXX
#if 0   //skip the AFE44xx LED force on, avoid the LED always on, just skip it.
            if(systemStatus.blHRSensorOn == false && test_menu_loopcount<5) //force turn on for inspection temporary
            {
                          
#if (AFE44x0_SUPPORT==1)
                                AFE44xx_PowerOn_Init();
                                //SysCtlDelay(9000);
#endif
            }
#if (AFE44x0_SUPPORT==1)    //should not check while without AFE44x0
			if(systemStatus.blHRSensorOnline == false)
			{
				error_buff[idx] = 'P';
				error++;
			}
#endif

            if(systemStatus.blSkinTouched == false && test_menu_loopcount >=5 && systemStatus.blHRSensorOn == true) //force turn off after inspection with no SkinTouch
            {
                //vTaskDelay(200); //
                //SysCtlDelay(50000);
#if (AFE44x0_SUPPORT==1)
                AFE44xx_Shutoff();
#endif
            }
#endif
#else
#if (AFE44x0_SUPPORT==1)
			AFE44xx_PowerOn_Init();
			if(systemStatus.blHRSensorOnline == false)
			{
				error_buff[idx] = 'P';
				error++;
			}
#endif
#endif

			idx++;
#if 0
			if(systemStatus.blBleOnline == false)
			{
				error_buff[idx] = 'B';
				error++;
			}
#endif
			idx++;

			if(systemStatus.blAccelSensorOnline == false)
			{
				error_buff[idx] = 'A';
				error++;
			}

			idx++;

			if(systemStatus.blFlashOnline == false)
			{
				error_buff[idx] = 'F';
				error++;
			}

			idx++;

			if(error == 0)
			{
#if BGXXX>0
//extern int8_t BLE_STATE;
//extern uint8_t FD_result;
				clearScreen(false);
				BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

extern int8_t lastLongPressedisTest; //defined in Device_task.c
                                if (lastLongPressedisTest>0)  //longpress
                                {
                                    test_menu_loopcount = 0;
                                    lastLongPressedisTest = 0;
#if BG013  //reset the DBG_IR_reg_min, DBG_IR_reg_max value.defined in AFE44x0.c
                                    DBG_IR_reg_min = REG_VAL_MAX;
                                    DBG_IR_reg_max = REG_VAL_MIN;
                                    DBG_AB_reg_min = REG_VAL_MAX;
                                    DBG_AB_reg_max = REG_VAL_MIN;
    #if 0       //[BG013-1]
    extern uint8_t LED_INTENSITY,AMB_uA;
                                    LED_INTENSITY =75;
                                    AMB_uA ++;//= 0;
                                    LED_Val_AMB_Cancellation(LED_INTENSITY, AMB_uA);
    #endif
#endif
#if BGXXX==6	//[BG021-2]
					//reset the debug variables.
					test1.typeint = 0;
					test2.typeint = 0;
					test3.typeint = 0;
#elif BGXXX==7
					test1.typeint = 0;
					test2.typeint = 0;
					test3.typeint = 0;
#elif BGXXX==8
					test1.typeint = 0;
					test2.typeint = 0;
					test3.typeint = 0;
#elif BGXXX==9
					test1.typeint = 0;
					test2.typeint = 0;
					test3.typeint = 0;
#endif
#if (SOS_HIT_SUPPORT)
extern uint8_t SOS_result;
                                    SOS_result=0; //++;  //[BG014-x] add
extern int8_t SOS_hit_count;
                                    SOS_hit_count=0;
#endif
                                    //return;
                                }
                                //sprintf(error_buff, "%d", test_menu_loopcount);
                                //sprintf(error_buff, "%X\n%d", test_menu_loopcount, systemSetting.SystemRstTimes);
                                //snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n%d F:%d", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes, systemStatus.bBatteryRemaining, FD_result);
                                /* show first line debug: %X:test_menu_loopcount, %d:BLE_STATE, %d SystemRstTimes */
#if BGXXX==1
                                //snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
    #if BG013
    extern unsigned long 	AFE44xx_Default_Register_Settings[49];
                                //snprintf(error_buff, sizeof(error_buff)-1, "%X %02x%02x\n", test_menu_loopcount, LED_INTENSITY, AMB_uA);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_IR_reg_min);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_IR_reg_max);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_AB_reg_min);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_AB_reg_max);
                                if ((test_menu_loopcount&0x04)==0) {
                                    snprintf(error_buff, sizeof(error_buff)-1, "%X %02x\n", test_menu_loopcount, LED_INTENSITY);
                                    snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_IR_reg_min);
                                    snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_IR_reg_max);
                                } else {
                                    snprintf(error_buff, sizeof(error_buff)-1, "%X   %02x\n", test_menu_loopcount, AMB_uA);
                                    snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_AB_reg_min);
                                    snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04hx", (int16_t)DBG_AB_reg_max);
                                }
    #endif
#elif BGXXX==2
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
    #if GESTURE_DISP_SUPPORT
    extern uint32_t gesturecount;
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %u", gesturecount);
    #endif
#elif BGXXX==3
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
    #if SOS_HIT_SUPPORT
    extern uint8_t SOS_result;
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %d", SOS_result);                                
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %d", SOS_hit_count);                                
    #endif
#elif BGXXX==4
       extern volatile UINT iUVexp;
       extern volatile BYTE bUltraVioletGather;
                                snprintf(error_buff, sizeof(error_buff)-1, "%02d %04d\n", test_menu_loopcount%60, (UINT)iUVexp);
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%02d %04d", UVexp_Threshold, bUltraVioletGather);
				//snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %u", GetMenuCount());
#elif BGXXX==5	//for BLE debug.
	extern uint8_t WpIndex,RpIndex;
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
                                //snprintf(error_buff, sizeof(error_buff)-1, "%X %04d\n", test_menu_loopcount, (int16_t)itest);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%4.3f", ftest);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04x", (uint16_t)utest);                                
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%d %d", WpIndex, RpIndex);
#elif BGXXX==6	//[BG021-2] for Download debug
	extern uint16_t Upload_Sector_Num;
                                //snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %04X\n", test_menu_loopcount, test1.typeuint16[0]);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04X%04X", test1.typeuint16[0], test2.typeuint16[0]);                                
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04X%02X", pFlashStorageIndicator->sector, pFlashStorageIndicator->capacity);
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04X%04X", pFlashStorageIndicator->sector, test1.typeuint16[1]);
				
#elif BGXXX==7	//[BG021-3]
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%02X %02X %02X", test1.typeuint8[0], test1.typeuint8[1], test1.typeuint8[2]);
extern uint8_t RpIndex;
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%02X %02X %02X", test1.typeuint8[0], test1.typeuint8[1], RpIndex);                                
#elif BGXXX==8	//BLE upgrade stop
extern int WR_SEC_COUNT;
                                //snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %02X\n", test_menu_loopcount, systemSetting.SystemRstTimes, test3.typechar[1]);
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%02X %02X %02X", test1.typeuint8[0], test1.typeuint8[1], test1.typeuint8[2]);
				//XX[last comm in ParseHostData(),XX[err comm count in ParseHostData[], XX[WR_SEC_COUNT]
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%02X %02X %02X", test1.typeuint8[0], test1.typeuint8[1], (uint8_t)WR_SEC_COUNT); //test1.typeuint16[1]

				//FW_CRC_GET XXXX[sector num],XXXX[ CRC16 of the sector]
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%02X%02X%04X", test3.typechar[0],(uint8_t)test2.typeuint16[0], test2.typeuint16[1]);
				//FW_CRC_GET XXXX[WR_SEC_COUNT backup],XXXX[ CRC16 of the sector]
                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%04X%04X", test1.typeuint16[1], test2.typeuint16[1]);
#elif BGXXX==9
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %02X\n", test_menu_loopcount, systemSetting.SystemRstTimes, systemStatus.blBatteryCharging);
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%d %d", systemStatus.bBatteryLevel, systemStatus.bBatteryRemaining);
#else
                                snprintf(error_buff, sizeof(error_buff)-1, "%X %d %d\n", test_menu_loopcount, BLE_STATE, systemSetting.SystemRstTimes);
                                /* show second cascade, %d:bBatteryRemaining */
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, "%d", systemStatus.bBatteryRemaining);
    #if FALL_DETECT_SUPPORT
    extern uint8_t FD_result;
                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %d", FD_result);
    #endif
//    #if GESTURE_DISP_SUPPORT
//    extern uint32_t gesturecount;
//                                snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %u", gesturecount);
//    #endif
#endif

                                //snprintf(error_buff+strlen(error_buff), sizeof(error_buff)-strlen(error_buff)-1, " %d%d%d", systemStatus.blHRSensorOn, systemStatus.blHRSensorOnline,systemStatus.blSkinTouched );

                                error_buff[sizeof(error_buff)-1]='\0';  //make sure string is terminated.
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, error_buff);
//                                sprintf(error_buff, "%X", test_menu_loopcount);
//				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 1, error_buff);

				//
//				AFE44xx_Shutoff();
//				ForceShowMenu(1, MENU_TYPE_Testing); //BGXXX remark

				// 禁用自动锁屏，即lcd常亮
				systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_TESTING_MENU;
				test_menu_loopcount=test_menu_loopcount +1; //++;
#else   //normal, not BGXXX
                clearScreen(true);
#if (VIBRATION_SUPPORT==1)
				VibrateCon(BuzzAlert1000ms, 1, 2);
#endif
				test_menu_loopcount++;
#endif
			}
			else
			{
				clearScreen(false);
				BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;
				OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, error_buff);

				//
#if BGXXX
				JumpToMenu(MENU_TYPE_Testing);
#else
#if (AFE44x0_SUPPORT==1)                
				AFE44xx_Shutoff();
#endif
				ForceShowMenu(1, MENU_TYPE_Testing);
#endif

				// 禁用自动锁屏，即lcd常亮
				systemStatus.bDisableAutoLockFlags |= AUTOLOCK_FLAG_TESTING_MENU;
			}

//			insideCount++;
//
//			if(insideCount > 1)
//			{
//				insideCount = 0;
//				loopcount = 3;
//			}

			break;
		}

		case 1:
//			OLEDOff();
//			LockScreen();
			test_menu_loopcount++;

			break;

		case 2:
			test_menu_loopcount++;
#if (AFE44x0_SUPPORT==1)
			AFE44xx_Shutoff();
#endif
#if (OLED_SUPPORT==1)
			OLEDOff();
#endif
			LockScreen();

			break;

		case 3:
			test_menu_loopcount++;
#if (OLED_SUPPORT==1)
			OLEDOff();
#endif
			LockScreen();

			break;


		default:
			//test_menu_loopcount++;
			//if(test_menu_loopcount>5)
			//test_menu_loopcount=0;
			break;
	}

}
// 从原来的代码中找回这段测试代码       2015年6月25日12:07:21
//void CallBackSubMenuTesting(void)
//{
//	static char insideCount = 0;
//
//	switch(test_menu_loopcount)
//	{
//
//		case 0:
//			clearScreen(true);
//			insideCount++;
//
//			if(insideCount > 5)
//			{
//				insideCount = 0;
//				test_menu_loopcount = 1;
//			}
//
//			LED_ON();
//			break;
//
//		case 1:
//			OLEDOff();
//			LockScreen();
//#if (VIBRATION_SUPPORT==1)
//			VibrateCon(BuzzAlert1000ms, 1, 1);
//#endif
//			test_menu_loopcount = 2;
//
//			break;
//
//
//		default:
//			//test_menu_loopcount++;
//			//if(test_menu_loopcount>5)
//			//test_menu_loopcount=0;
//			break;
//	}
//
//}


#endif

#if FALL_DETECT_SUPPORT
//检测到跌落后，屏幕的显示
void CallBackSubMenuFallAlert(void)
{
	static uint8_t counter = 0;

	if(counter % 2 == 0)
	{
		clearScreen(true);
	}
	else
	{
		clearScreen(false);
	}

	counter++;
	counter %= 10;
}
#endif

#if defined(DEBUG) || defined(DEBUG_MODE)
uint8_t blecmd = 0;
void CallBackSubMenuBLE(void)
{
	clearScreen(false);

	char buff[10];
	sprintf(buff, "%d 0x%x", blecmd, blecmd);

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;
	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, "cmd:");
	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, buff);
}
#endif

#if defined(DEBUG) || defined(DEBUG_MODE)
extern DATA_GATHER_BUFFER_HEAD gatherBufferHead;
void CallBackSubMenuDataGather(void)
{
	clearScreen(false);

	char buff[10];

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	sprintf(buff, "%d,%d,%d", pFlashStorageIndicator->sector, pFlashStorageIndicator->capacity, pFlashStorageIndicator->nextSectorIsPrepared);
	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 0, buff);

	INDEX_DATA_BUFFER_INDICATOR* pbi = &(gatherBufferHead.bufferIndicator[0]);
	short p = pbi->bufferOffset * 16 / pbi->bufferSize;

	for (int i = 0; i < p; i++)
		oled_drawHortLine(page, 15 - i, SCREEN_WIDTH - 4, SCREEN_WIDTH, 0);

	sprintf(buff, "%d", pFlashStorageIndicator->startTimestamp);
	OledShowString(page, &FONT_ARIAL_8x16, 0, 0, 16, buff);
}
#endif

void OledDrawGuage(uint8_t guage, bool isShowNumber)
{

	uint8_t valueBit = 0;
	uint8_t tempValue = 0;
	uint8_t j = 0;
	char tempBuf[3] = {0}; //[BG025] int8_t >> char


	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

//show percentage line
	sprintf(tempBuf, "%d", guage);
	valueBit = strlen(tempBuf);

	for(uint8_t k = 0; k < valueBit; k++)
		tempBuf[k] -= 0x30;

	if(guage >= 100)
		tempValue = SCREEN_WIDTH - 3;
	else
		tempValue = (uint8_t)((float)(guage * (SCREEN_WIDTH - 3)) / 100.0); //[BG025] (int)>>(uint8_t)

	if(isShowNumber == false)
	{
		//说明是下载firmware，此时只要达到97%就认为是100%，因为用RTC时基来刷屏的，
		//一个时基内，百分比变化大于1，所以下载看不到100%
		if(tempValue >= 96)
		{
			tempValue = 100;
		}
	}

	oled_drawHortLine(page, 14, 2, tempValue, 0);
	oled_drawHortLine(page, 15, 2, tempValue, 0);
	oled_drawPic(page, 0, 13,
	             (BYTE*)iconCaloriesPercentage_64_4[0],
	             64, 8, DRAW_MODE_TRANSPARENT);


	if(isShowNumber == false)
		return;

// show percentage numbers
	switch(valueBit)
	{
		case 1:
			oled_drawPic(page, 16, 0,
			             (BYTE*)iconSmallNumbers_8_13[tempBuf[0] + 1],
			             8, 16, DRAW_MODE_TRANSPARENT);
			break;

		case 2:
			for(j = 1; j < 3; j++)
				oled_drawPic(page,  8 * j, 0,
				             (BYTE*)iconSmallNumbers_8_13[tempBuf[j - 1] + 1],
				             8, 16, DRAW_MODE_TRANSPARENT);

			break;

		case 3:
			for(j = 0; j < 3; j++)
				oled_drawPic(page,  8 * j, 0,
				             (BYTE*)iconSmallNumbers_8_13[tempBuf[j] + 1],
				             8, 16, DRAW_MODE_TRANSPARENT);

			break;

		default:
			break;
	}

// show %
	oled_drawPic(page, 24, 0,
	             (BYTE*)iconSmallNumbers_8_13[0],
	             8, 16, DRAW_MODE_TRANSPARENT);
}


void OledDisplayUV(BYTE uvIndex, BYTE* icon)
{

#ifdef DEBUG
// uvIndex = 11;
#endif

	BYTE page = g_tMenuInfo.mucUpDataArea / DISPLAY_X;

	if(uvIndex > 9)
	{
		if(uvIndex >= 11)
		{
			oled_drawPic(page, 0, 0, (BYTE*)maxUVIndex, 64, 32, DRAW_MODE_TRANSPARENT);
			return;
		}

		oled_drawPic(page, 0, 0,
		             (BYTE*)iconNumbers_22_32[uvIndex / 10],
		             22, 32, DRAW_MODE_TRANSPARENT);
		oled_drawPic(page, 22, 0,
		             (BYTE*)iconNumbers_22_32[uvIndex % 10],
		             22, 32, DRAW_MODE_TRANSPARENT);
	}
	else
	{
		oled_drawPic(page, 22, 0,
		             (BYTE*)iconNumbers_22_32[uvIndex],
		             22, 32, DRAW_MODE_TRANSPARENT);

	}

	if(icon > 0)
		oled_drawPic(page, 44, 0,
		             (BYTE*)icon,
		             20, 32, DRAW_MODE_TRANSPARENT);
}