#ifndef __MENU_H
#define __MENU_H

#include <stdbool.h>
#include "typedefs.h"

#include "m00930.h"

#define SCREEN_WIDTH	64
#define SCREEN_HEIGHT	32

#define DISPLAY_X       SCREEN_WIDTH
#define DISPLAY_Y       (SCREEN_HEIGHT / 8)

//�˵�����
#define MENU_UPDATA_AREA_0      (0)                     //0
#define MENU_UPDATA_AREA_1      (SCREEN_WIDTH)          //64
#define MENU_UPDATA_AREA_2      (SCREEN_WIDTH * 2)      //128

#define UPDATA_AREA_FOREGROUND		MENU_UPDATA_AREA_0
#define UPDATA_AREA_BACKGROUND		MENU_UPDATA_AREA_1


//����˵�����
#define MENU_ACT_RIGHT_SLIDING  (-1)
#define MENU_ACT_CURRENT        (0)
#define MENU_ACT_LIFT_SLIDING   (1)


//�˵�����Ϣ
typedef struct {
  int8_t  		mtMenu;	//�˵����
  uint8_t       mucUpDataArea;
  uint8_t       mucAction;
} tMemuInfoTypeDef;

typedef enum
{
	MENU_TYPE_Current = 0, //��ǰ�˵�
	MENU_TYPE_Time,
	MENU_TYPE_HeartRate,
	MENU_TYPE_Calories,
	MENU_TYPE_Step,
	MENU_TYPE_Distance,
	MENU_TYPE_UltraViolet,
	MENU_TYPE_BodyTemper,
	MENU_TYPE_AmbientTemper,
	MENU_TYPE_Alarm,
	MENU_TYPE_IncomingCall,
	MENU_TYPE_Battery,
	MENU_TYPE_BleMAC,
	MENU_TYPE_DOWNLOAD,//�¼ӵ�
	MENU_TYPE_Notifications,
	MENU_TYPE_Testing,
	MENU_TYPE_Ble_Testing,
	MENU_TYPE_Data_gather_monitor,
	MENU_TYPE_Device_activate,
#if FALL_DETECT_SUPPORT
	MENU_TYPE_FALL_ALERT,
#endif
} MENU_TYPE;



extern tMemuInfoTypeDef g_tMenuInfo;

//�����õ�
void MENU_T_Display(void);


//��ʼ���˵�
void MENU_Init(void);

BYTE GetMenuCount();

// ���ص�ǰ�˵�������
BYTE GetCurrentMenuIndex();
MENU_TYPE GetCurrentMenuType();

// ��ò˵�����Ϣ�����ڷ��͵�app���Ʋ˵���Ŀɼ���
int GetMenuInfo(BYTE** pmi);

void EnableMenu(MENU_TYPE menu, bool enabled);
int GetEnabledMenus();

// ֱ����ת��ĳ���˵�
// ����˵������ã�enable=false || filter=false)�����޶���
void JumpToMenu(MENU_TYPE menu);

void ForceShowMenu(int num, ...);
void UnforceShowMenu();
bool IsForcedShowMenu();

//app����
void MENU_FadeOut();

void MENU_AppLeft(void);
void MENU_AppRight(void);
void MENU_AppUpdata(void);
void Menu_FlyIn();
void MENU_ReplyUpdata(int8_t cAct);
void MENU_AppPwrOn(uint8_t enable);
void MENU_UpdataRam(uint8_t x);

//void MENU_showEvents();

// ��ͼ�����ݣ��ֿ��ͼƬ��д���Դ�򻺴�
// page-�Դ�ҳ��0,1,2
// x,y-��ʾλ��
// pic-ͼ������
// w,h-ͼ��ĳߴ�
// mode-����ģʽ��0=normal/overwrite,1=or/transparent,2=xor,4=not(invert)
void oled_drawPic(BYTE page, short x, short y, BYTE* const pic, short w, short h, int mode);

void Oled_DrawPixel(BYTE page, BYTE x, BYTE y, int drawMode);

void oled_drawHortLine(BYTE page, BYTE y, BYTE xFrom, BYTE xTo, int mode);

void oled_drawVertLine(BYTE page, BYTE x, BYTE yFrom, BYTE yTo, int mode);

#endif
