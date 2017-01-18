#ifndef __LED_FRONT_H
#define __LED_FRONT_H

#include "typedefs.h"
#include "common_vars.h"

#if ENABLE_PAUSE_STOPWATCH //0
//屏蔽没有用到的 2015年10月6日14:32:12
extern const unsigned char ICON_BATTERY_EMPTY[];

extern const unsigned char ICON_BATTERY_GAUGE[];

extern const unsigned char ICON_STOPWATCH[];

// 16x32
extern const unsigned char oled_Step_Image[4][16];

extern const unsigned char oled_Distance_Image[4][16];

extern const unsigned char oled_calorie_Image[4][16];

extern const unsigned char oled_UltraViolet_Image[4][16];

extern const unsigned char oled_Out_Temper_Image[4][16];

extern const unsigned char ICON_ALARM[4][16];
extern const unsigned char ICON_ALARM_LARGE[4][27];

extern const unsigned char oled_HeartRate_Image[][16];

extern const unsigned char oled_Body_Temper_Image[4][16];


extern const unsigned char oled_Step_Image_T[64];//测试用 2015年3月25日11:41:57


//extern 
// 16x32
// 附加秒表图标
extern const unsigned char ICON_STOPWATCH[];
extern const unsigned char ICON_STEPS_STOPWATCH[];
extern const unsigned char ICON_DISTANCE_STOPWATCH[];
extern const unsigned char ICON_CALORIES_STOPWATCH[];

// icons for notifications menu
// 16x16
extern const unsigned char ICON_LOW_BATTERY[][16];
extern const unsigned char ICON_INCOMING_CALL_16x16[][16];
extern const unsigned char ICON_MISSED_CALL_16x16[][16];
extern const unsigned char ICON_MAIL[][16];

////14x9
//const unsigned char ICON_UNIT_KM[][14]={
//0x00,0x02,0xFA,0x22,0x52,0x8A,0x02,0xFA,0x0A,0x72,0x0A,0xFA,0x02,0x00,0x00,0x00,
//0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/*"D:\Works\qs\docs\imgs\lcd icons\i4\km.bmp",0*/
//};

// 15x12
extern const unsigned char ICON_UNIT_KM[][15];

// 15x12
extern const unsigned char ICON_UNIT_MI[][15];

// 15x12
extern const unsigned char ICON_UNIT_CELSIUS[][15];

// 15x12
extern const unsigned char ICON_UNIT_FAHRENHEIT[][15];

// 15x12
extern const unsigned char ICON_UNIT_KILOCAL[][15];

// 15x12
extern const unsigned char ICON_UNIT_KILO[][15];

// 10x10
extern const unsigned char ICON_UNIT_SMALL_CELSIUS[][10];

// 10x10
extern const unsigned char ICON_UNIT_SMALL_FAHRENHEIT[][10];

// 24x32
extern const unsigned char ICON_INTENSE_UV[][24];

// 32x32
extern const unsigned char ICON_INCOMING_CALL[][32];

// 用于显示时间
extern const unsigned char FONT_DS_DIGITAL_14x32[][4][14];

#endif

extern const unsigned char ICON_MISSED_CALL[][32];
extern const unsigned char ICON_CALENDAR[][16];
extern const unsigned char ICON_NEWS[][16];
extern const unsigned char ICON_MESSAGES[][16];

extern const unsigned char amPm_7_32[][28];
extern const unsigned char colon_9_32[];
extern const unsigned char hourNumbers_20_32[][80];
extern const unsigned char mintuesNumber_14_32[][56];
extern const unsigned char iconHeart_22_32[];
extern const unsigned char iconHeartNumbers_14_32[][56];
extern const unsigned char iconCalories_32_13[];
extern const unsigned char iconBigNumbers_12_16[][24];
extern const unsigned char iconSmallNumbers_8_13[][16];
extern const unsigned char iconCaloriesPercentage_64_4[][64];
extern const unsigned char iconStep[];
extern const unsigned char iconDistance[];
extern const unsigned char iconKmMeter[][30];
extern const unsigned char iconDecimalPoint_4_16[];
extern const unsigned char iconUV[];
extern const unsigned char iconUVAlert[];
extern const unsigned char iconNumbers_22_32[][88];
extern const unsigned char iconTemperatureUnit_12_16[][24];
extern const unsigned char iconTemperature_15_16[][30];
extern const unsigned char iconAlarm_10_8[][10];
extern const unsigned char iconAlarmHourNumber_20_24[][60];
extern const unsigned char iconAlarmMintuesNumbers_14_24[][42];
extern const unsigned char iconAlarmOnOff_9_16[][18];
extern const unsigned char iconAlarmColon[];
extern const unsigned char iconMessageAndCall[][30];
//extern const unsigned char iconCallsAndMSMNumber8_16[][16];
extern const unsigned char iconBattery64_32[][256];
extern const unsigned char iconAlarmTimeNow[];
extern const unsigned char charGulim_8_16[][16];
extern const unsigned char iconBLEDisconnected[];
extern const unsigned char iconBLEConnected[];
extern const unsigned char macAddrdata[][16];
extern const unsigned char maxUVIndex[];
extern const unsigned char downloadProgress[];


// =================================================================
typedef struct _FONT_INFO
{
	int width;	// 字体的宽度
	int height; // 字体的高度
	
//	int bytesOfWidth;	// 宽度的字节数（对齐字节）
//	int bytesOfHeight;	// 高度的字节数（对齐字节）
	
	int bWidth;		// 宽度（对齐字节）
	int bHeight;	// 高度（对齐字节）
	
	int bytes;	// 一个字符的字节数
	
	char base; 	// 起始字符。比如只有数字的字库，base = '0'；
	BYTE* data;
} FONT_INFO;

// 用于通知菜单显示事件的数量
extern const unsigned char FONT_DIGITAL_ARIAL_7x14[][1][14];

// 8x16
extern const unsigned char FONT_DATA_ARIAL_8x16[][2][8];
extern const FONT_INFO FONT_ARIAL_8x16;


// 用于显示正常菜单文字
// gulimche
extern const unsigned char FONT_DATA_ASCII_12x24[][3][12];
extern const FONT_INFO FONT_ASCII_12x24;

// 用于显示小数部分
// gulimche
extern const unsigned char FONT_DATA_DIGITAL_GULIMCHE_8x16[][2][8];
extern const FONT_INFO FONT_DIGITAL_GULIMCHE_8x16;

#endif