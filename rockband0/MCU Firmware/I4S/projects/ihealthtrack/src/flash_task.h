#ifndef FLASH_TASK_H
#define FLASH_TASK_H

#include "freertos.h"
#include "queue.h"

#include "common_vars.h"
#include "data_gather.h"

// flash����ʼsector�����ڱ������ɸ�sector���ں�����չ
#define INDEX_DATA_START_SECTOR	(512)  //512*4K=2M
#ifdef DEBUG
#define INDEX_DATA_END_SECTOR	2047//(550) //���������Ĵ��  2015��12��29��9:14:13
#else
#define INDEX_DATA_END_SECTOR	(2047) //8M
#endif

// flash��С�������ӣ���ǰ����С�� 1/FLASH_MIN_CAPACITY_FACTORʱ��Ԥ����һ��sector
#define FLASH_MIN_CAPACITY_FACTOR	(4)

#define DEFAULT_TIMESTAMP	(0xFFFFFFFF)

typedef enum
{
	FLASH_CMD_PREPARE_CHIP = 1,	// ���ڲ�������flash��������ϵͳ��һ������
	FLASH_CMD_PREPARE_DATA_STORAGE,	// ���ڲ������ݴ洢����һ��ֻ����ǰ����sector
	FLASH_CMD_INIT,				// ����ϵͳ�ϵ�ʱ��ɨ������flash����ȷ����ǰ���õ�sector����洢״̬�����Ŀ�д���ַ��
	FLASH_CMD_WRITE,			// д���ݵ�flash
	FLASH_CMD_PREPARE_SECTOR,	// Ԥ��һ��sector����ָ��һ�㲻��Ӧ��ֱ�ӵ��ã�������flashtask�ڲ�ʹ�á�
								// ��ָ�����ָ����sector����д�ϴ洢�����š��ο� ָ�����ݲɼ���洢
	
	FLASH_CMD_UPLOAD,
//	FLASH_CMD_BLOCK_ERASE,
	
	FLASH_CMD_DATA_GATHER_TICK,
	
#ifdef DEBUG
	FLASH_CMD_PREPARE_SECTOR_FOR_TESTING,
#endif
	
	FLASH_CMD_RESET_DATA_GATHER,
	
} ENUM_FLASH_COMMANDS;


typedef struct _FLASH_COMMAND
{
	BYTE cmd;
	union {
		int sector;
		int block;
	} address;
	union {
		BYTE * p;
		INT32 v;
		struct {
			INT16 dataOffset;
			INT16 dataSize;
		} d;
	} data;
	
} FLASH_COMMAND;


#pragma pack(push, 1)

typedef struct _INDEX_DATA_HEAD
{
	BYTE type;		// ��������
	INT16 interval;	// ��������
	INT16 samples;	// ��������
	INT16 offset;	// ����ƫ�ƣ������chunk
					// chunk
					// 		INDEX_DATA_HEAD x n
					//		data x n
} INDEX_DATA_HEAD;

//�ýṹ��16�ֽ�
typedef struct _FLASH_DATA_CHUNK_HEAD
{
	time_t timestamp;	// gmt timestamp
	BYTE channels;
#ifdef FLASH_STORAGE_SCHEMA_V2
	int16_t timezoneOffset;	// tz offset���Է��Ӽ�
	BYTE reserved[9];
#else
	INT16 size; // �������ݵĴ�С������ͷ
#endif
//	
//	INDEX_DATA_HEAD firstChannel;
} FLASH_DATA_CHUNK_HEAD;


#define DATA_STORAGE_SCHEMA_VERSION 2

#define FLASH_SECTOR_TAG	(0xB38F0CB9)

/* the flash sector which stored the historical data header layout.32 bytes */
typedef struct _FLASH_SECTOR_HEAD
{
	INT32 tag;		// write FLASH_SECTOR_TAG
	INT32 index;		// increment sector index number.
	time_t startTimestamp;	// the sector stored data start timestamp. (eqv. first chunk timestamp)
	time_t endTimestamp;	// the sector stored data end timestamp. (eqv.the last chunk timestamp)
//	INT16 dataSize;		// sector����Ч���ݵĴ�С
	INT16 chunkSize;	// chunk�Ĵ�С
	BYTE schemaVersion;	// 
	BYTE reserved[13];	// ���������ֽ����ں�����չ��Ŀǰ�����ṹ����32���ֽ�
} FLASH_SECTOR_HEAD;

#pragma pack(pop)


// =================================================================
//extern osMailQId hFlashCommandQueue;

void initFlashTask(); //initialize task

// flash power management
void WakeFlashUp();
void PutFlashToSleep();

typedef enum
{
	FLASH_BUSY_BIT_DATA_GATHER = 1 << 0,	// data gathering write
	FLASH_BUSY_BIT_BLE = 1 << 1,		// ble cmd access��include firmware upgrading, index data download
} FLASH_BUSY_BIT;

void SetFlashBusyBit(FLASH_BUSY_BIT busyBit); //set flash busy flag
void ClearFlashBusyBit(FLASH_BUSY_BIT busyBit); //clean flash busy flag
bool CheckFlashBusyStatus(); //check flash busy status

void CheckFlashStatus(void);
void SetFlashUsingTime(uint16_t t);

void switchToNextSector(bool updateTimestamp);

void FlashOpeartionCallback(BYTE flashCMD);



BaseType_t createFlashTask();

// �����һ�����ݿ��
// ��Ҫ�Ĺ������Ǽ��sector����Ч��
int getNextIndexDataSector(int sector);


time_t searchMatchedTimestamp(time_t t);

// ��������sector���ҵ��� <= t �����ʱ�����������sector���
// ���û�з������������ݣ����� 0xFFFF
// ��Ӧ�� timestamp ͨ�� pt ����
uint16_t SearchSectorForTimestamp(time_t* pt);


// �˺������ɼ���������д��flash����data_gather.c����
// ���������������ݲɼ�Ϊ��������������
// ָ��洢�����pbh��
// buffer��������ֻ��ָ�����ݣ�
void saveIndexDataToFlash(DATA_GATHER_BUFFER_HEAD* pbh, BYTE* buffer);


void doFlashUpload();
BYTE SearchSectorIndex(uint32_t index, uint16_t* sectorNo,bool blStart );
void CombinIndexNumber(SECTOR_NODE* sectorNode, int startSectorIndex, int startSectorNumber, int totalNum, int* counter);
int SearchMaxSectorIndex(uint16_t *secnum);

//#define FLASH_BUFFER_SIZE	1024
//extern BYTE dataGatherBuffer[FLASH_BUFFER_SIZE];
//extern int dataBufferCapability;
//extern INT16 dataBufferHead;
//extern INT16 dataBufferTail;
//
//extern INT16 flashOperationHead;
//extern INT16 flashOperationTail;
//extern INT16 flashOperationPoint;
//
#define LEAST_FLASH_DATA_CHUNK_SIZE	11
//#define LEAST_FREE_FLASH_DATA_CHUNK_SIZE	600
//
//#define MAX_FLASH_WRITING_SIZE	(256)

// ====================================================================================
//extern FLASH_STORAGE_INDICATOR flashStorageIndicator;
//extern osMessageQId hMsgQFlashCommands;

//osMailQDef(FlashCommandQueue, 5, FLASH_COMMAND);
//extern osMailQId hFlashCommandQueue;

extern QueueHandle_t hEvtQueueFlash;



#endif  /* Avoid multiple inclusion */