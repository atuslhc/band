#ifndef FLASH_TASK_H
#define FLASH_TASK_H

#include "freertos.h"
#include "queue.h"

#include "common_vars.h"
#include "data_gather.h"

// flash的起始sector，用于保留若干个sector用于后续扩展
#define INDEX_DATA_START_SECTOR	(512)  //512*4K=2M
#ifdef DEBUG
#define INDEX_DATA_END_SECTOR	2047//(550) //后来把它改大的  2015年12月29日9:14:13
#else
#define INDEX_DATA_END_SECTOR	(2047) //8M
#endif

// flash最小容量因子，当前容量小于 1/FLASH_MIN_CAPACITY_FACTOR时，预备下一个sector
#define FLASH_MIN_CAPACITY_FACTOR	(4)

#define DEFAULT_TIMESTAMP	(0xFFFFFFFF)

typedef enum
{
	FLASH_CMD_PREPARE_CHIP = 1,	// 用于擦除整个flash，可用于系统第一次启动
	FLASH_CMD_PREPARE_DATA_STORAGE,	// 用于擦除数据存储区。一般只擦除前几个sector
	FLASH_CMD_INIT,				// 用于系统上电时，扫描整个flash，以确定当前可用的sector及其存储状态（最后的可写入地址）
	FLASH_CMD_WRITE,			// 写数据到flash
	FLASH_CMD_PREPARE_SECTOR,	// 预备一个sector，此指令一般不由应用直接调用，而是由flashtask内部使用。
								// 此指令将擦除指定的sector，并写上存储索引号。参看 指标数据采集与存储
	
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
	BYTE type;		// 数据类型
	INT16 interval;	// 采样周期
	INT16 samples;	// 样本数量
	INT16 offset;	// 数据偏移，相对于chunk
					// chunk
					// 		INDEX_DATA_HEAD x n
					//		data x n
} INDEX_DATA_HEAD;

//该结构体16字节
typedef struct _FLASH_DATA_CHUNK_HEAD
{
	time_t timestamp;	// gmt timestamp
	BYTE channels;
#ifdef FLASH_STORAGE_SCHEMA_V2
	int16_t timezoneOffset;	// tz offset，以分钟计
	BYTE reserved[9];
#else
	INT16 size; // 本块数据的大小，包括头
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
//	INT16 dataSize;		// sector中有效数据的大小
	INT16 chunkSize;	// chunk的大小
	BYTE schemaVersion;	// 
	BYTE reserved[13];	// 保留若干字节用于后续扩展，目前整个结构共计32个字节
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
	FLASH_BUSY_BIT_BLE = 1 << 1,		// ble cmd access，include firmware upgrading, index data download
} FLASH_BUSY_BIT;

void SetFlashBusyBit(FLASH_BUSY_BIT busyBit); //set flash busy flag
void ClearFlashBusyBit(FLASH_BUSY_BIT busyBit); //clean flash busy flag
bool CheckFlashBusyStatus(); //check flash busy status

void CheckFlashStatus(void);
void SetFlashUsingTime(uint16_t t);

void switchToNextSector(bool updateTimestamp);

void FlashOpeartionCallback(BYTE flashCMD);



BaseType_t createFlashTask();

// 获得下一个数据块号
// 主要的工作就是检查sector的有效性
int getNextIndexDataSector(int sector);


time_t searchMatchedTimestamp(time_t t);

// 搜索所有sector，找到最 <= t 的最大时间戳，返回其sector序号
// 如果没有符合条件的数据，返回 0xFFFF
// 对应的 timestamp 通过 pt 返回
uint16_t SearchSectorForTimestamp(time_t* pt);


// 此函数将采集到的数据写入flash，由data_gather.c调用
// ！！仅适用于数据采集为单缓冲的情况！！
// 指标存储情况在pbh中
// buffer缓冲区中只有指标数据，
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