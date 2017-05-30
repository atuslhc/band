/*
 * displayTask.c
 *
 *  Created on: 2013-6-24
 *      Author: Administrator
 */
#include <assert.h>
#include <stdlib.h>

#include "freertos.h"
#include "task.h"
//#include "cmsis_os.h"

#include "m25pxx.h"

#include "em_usbtypes.h"
#include "em_usbhal.h"
#include "em_usbd.h"

#include "descriptors.h"
#include "config.h"

#include "common_vars.h"
#include "main.h"
#include "crc.h"
#include "device_task.h"
#include "flash_task.h"

#include "debug.h"


uint16_t FlashUsingTimeCount = 0;


//osMailQDef(FlashCommandQueue, 5, FLASH_COMMAND);
//osMailQId hFlashCommandQueue = NULL;


#define FLASH_TASK_NAME			"Flash"
#define FLASH_TASK_STACK_DEPTH	(128)	// 堆栈级别
#define FLASH_TASK_PRIORITY		(0)

#define FLASH_TASK_QUEUE_LEN			5
#define FLASH_TASK_QUEUE_ITME_SIZE		sizeof(FLASH_COMMAND)

QueueHandle_t hEvtQueueFlash = 0;



//FLASH_STORAGE_INDICATOR dataStartSector;

// 用于保存数据上传的位置
// 有效参数有
//		sector:
//		offset: 读取数据位置，对当前sector起始地址
//		capacity: 当前sector数据总量
//		startTimestamp;
FLASH_STORAGE_INDICATOR dataUploadPointer;

void initFlashTask()
{
//	hFlashCommandQueue = osMailCreate(osMailQ(FlashCommandQueue), osThreadGetId());
//	osMailQAddToRegistry(hFlashCommandQueue, "FLASH");

	hEvtQueueFlash = xQueueCreate(FLASH_TASK_QUEUE_LEN, FLASH_TASK_QUEUE_ITME_SIZE);
	vQueueAddToRegistry(hEvtQueueFlash, "MsgQ_Flash");
}

// =================================================================
// flash power managment

volatile BYTE flashBusyIndicator = 0;

void WakeFlashUp()
{
	SetFlashUsingTime(30);//这里设置一个值30，当该值减到0时，就会把flash关掉。

	if (FLASH_POWER_BACK())
		vTaskDelay(1);
}

// 设置flash忙
void SetFlashBusyBit(FLASH_BUSY_BIT busyBit)
{
	taskENTER_CRITICAL();

	flashBusyIndicator |= busyBit;

	taskEXIT_CRITICAL();

}

void ClearFlashBusyBit(FLASH_BUSY_BIT busyBit)
{
	taskENTER_CRITICAL();

	flashBusyIndicator &= ~busyBit;

	taskEXIT_CRITICAL();

}

void SetFlashUsingTime(uint16_t t) //Atus: SetFalshUsingTime >> SetFlashUsingTime
{
	FlashUsingTimeCount = t;
}


void CheckFlashStatus(void)  //Atus: CheckFalshStatus >> CheckFlashStatus
{
	taskENTER_CRITICAL();

	if(FlashUsingTimeCount > 0)
	{
		FlashUsingTimeCount--;

		if(FlashUsingTimeCount == 0)
		{
			flashBusyIndicator = 0;
			PutFlashToSleep();
		}

	}

	taskEXIT_CRITICAL();
}


bool CheckFlashBusyStatus()
{
	taskENTER_CRITICAL();

	bool busy = false;

	if (flashBusyIndicator != 0)
		busy = true;

	taskEXIT_CRITICAL();

	return busy;
}

void PutFlashToSleep()
{
	if (CheckFlashBusyStatus())
		return;

	FLASH_POWER_DOWN();
}

// flash power managment
// =================================================================


// blNotify，是否发送通知事件
void WaitFlashOpeartionDone(BYTE cmd, bool blNotify)
{
//	while (IsFlashBusy() == 1)
//		osThreadYield();
	MESSAGE msg;

	if (blNotify)
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_FLASH_OPERATION_DONE + ((long)cmd << 16), 0);
	{
		msg.params.type = MESSAGE_FLASH_OPERATION_DONE;

		msg.params.param = cmd;
		xQueueSend(hEvtQueueDevice, &msg.id, 0);
	}
}

// 等待长时间操作完成
// 改成 vTaskDelay 进行延时，防止watchdog动作
void WaitLongTimeFlashOpeartionDone(BYTE cmd, bool blNotify)
{
	MESSAGE msg ;

	while (IsFlashBusy() == 1)
	{
		vTaskDelay(100);
	}

	if (blNotify)
//		osMessagePut(hMsgInterrupt, (long)MESSAGE_FLASH_OPERATION_DONE + ((long)cmd << 16), 0);
	{
		msg.params.type = MESSAGE_FLASH_OPERATION_DONE;
		msg.params.param = cmd;
		xQueueSend(hEvtQueueDevice, &msg.id, 0);
	}
}

int getNextIndexDataSector(int sector)
{
	sector++;

	if (sector > INDEX_DATA_END_SECTOR)
		sector = INDEX_DATA_START_SECTOR;
	else if(sector < INDEX_DATA_START_SECTOR)
		sector = INDEX_DATA_START_SECTOR;

	return sector;
}

int getPreIndexDataSector(int sector)
{
  	sector--;
	if (sector <INDEX_DATA_START_SECTOR)
	  	sector = INDEX_DATA_END_SECTOR;
	else if (sector > INDEX_DATA_END_SECTOR)
	  	sector = INDEX_DATA_END_SECTOR;
	return sector;
}

/*
每次数据存储都以时间戳开头(4字节)，后接一个字节的指标数(1字节)，
然后是第一个指标的存储头（类型、采样频率、samples，共5字节），以及至少1个字节的数据，
即每块数据至少有 11字节
*/

/*
扫描一个sector以确定其剩余容量
*/
int ScanSectorCapacity(FLASH_STORAGE_INDICATOR* fsi)
{
	if (fsi == 0 || fsi->sector < INDEX_DATA_START_SECTOR || fsi->sector > INDEX_DATA_END_SECTOR)
		return -1;

	CCTRACE("ScanSectorCapacity -- sector: %d\n", fsi->sector);

	WakeFlashUp();

	ReturnType rt;
	FLASH_DATA_CHUNK_HEAD dch;
//	INDEX_DATA_HEAD cdh;
//	int sampleSize = 0;
//	int chunkSize = 0;

	long sectorAddr = fsi->sector * pFlashInfo->sectorSize;
	long addr = sectorAddr;//首先获取本sector的实际地址，在加上sector头的偏移
	addr += sizeof(FLASH_SECTOR_HEAD); // 跳过FLASH_SECTOR_HEAD

#ifdef FLASH_STORAGE_SCHEMA_V2

	time_t minTS = DEFAULT_TIMESTAMP;

	fsi->capacity = 0;

	for (int c = 0; c < ChunksPerSector; c++)
	{
		//下面读的是chunk的时间戳
		rt = FlashRead(addr + c * INDEX_DATA_CHUNK_SIZE, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // 读取数据块头
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d, size: %d\n", addr, rt, dch.timestamp, dch.channels, dch.size); //[BG025] add rt
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success)
		{ }

		if (dch.timestamp <= 0 || dch.timestamp == DEFAULT_TIMESTAMP)
		{
			// 此时间戳无效，搜索结束
			// 当前地址为新数据写入地址
			fsi->capacity = ChunksPerSector - c; //fsi->capacity表示剩下的可用空间
			break;
		}
		else
		{
			if (dch.timestamp < minTS)
				minTS = dch.timestamp;//获取当前读到的时间，继续查找
		}
	}

	fsi->startTimestamp = minTS;//用上次扫描得到的时间戳来更新起始时间戳。

	return fsi->capacity; //从sector中减去已经扫描过的chunk，返回剩下的就可以用来存储新的数据。

#else
	long sts = 0, ets = 0; //[BG025] move upper scope into here.

	while (addr < sectorAddr + pFlashInfo->sectorSize - LEAST_FLASH_DATA_CHUNK_SIZE)
	{
		rt = FlashRead(addr, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // 读取数据块头
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d, size: %d\n", addr, rt, dch.timestamp, dch.channels, dch.size); //[BG025] add rt
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (dch.timestamp <= 0 || dch.timestamp == DEFAULT_TIMESTAMP || dch.size < 0)
		{
			// 此时间戳无效，搜索结束
			// 当前地址为新数据写入地址
			fsi->offset = addr - sectorAddr;
			fsi->capacity = pFlashInfo->sectorSize - fsi->offset;
			fsi->startTimestamp = sts;
			fsi->endTimestamp = ets;

			CCTRACE("\tcapacity: %d\n", fsi->capacity);
			return fsi->capacity;
		}
		else
		{
			// 有效时间戳
			if (sts <= 0)
				sts = dch.timestamp;

			ets = dch.timestamp;

			//
//#ifdef DEBUG
//			addr += sizeof(FLASH_DATA_CHUNK_HEAD); // 跳过数据块头
//
//			for (int i = 0; i < dch.channels; i++)
//			{
//				// 逐个读取数据头
//				rt = FlashRead(addr, (BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));
//				CCTRACE("\t\taddr: %d, type: %d, int: %d, samp: %d\n", addr, cdh.type, cdh.interval, cdh.samples);
//
//				sampleSize = INDEX_DATA_SAMPLE_SIZE[cdh.type];
//				chunkSize = sampleSize * cdh.samples;
//
//				// 跳到下一种数据
//				addr += chunkSize + sizeof(INDEX_DATA_HEAD);
//			}
//#else
			addr += dch.size;
//#endif
		}
	}

	// 未找到空闲空间，可用容量直接返回0
	return 0;

#endif
}

/*
初始化flash芯片
包括擦除flash芯片，更新系统状态，发送 FLASH_CMD_INIT 命令以初始化flash存储状态
*/
void onCmdInitChip(FLASH_COMMAND* fcmd)
{
	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;

	//
	systemStatus.flashStatus = FLASH_STATUS_INITIALIZING;

	// ----------------------------------------------------------------------------
	//
	WakeFlashUp();

	ReturnType rt = FlashBulkErase();

	WaitFlashOpeartionDone(cmd, false);

	// 更新系统状态
	systemStatus.blFlashInitialized = true;

	// ----------------------------------------------------------------------------
	// 发送命令初始化flash（重建索引）
//	FLASH_COMMAND* cmdInit = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmdInit->cmd = FLASH_CMD_INIT;
//
//	osMailPut(hFlashCommandQueue, cmdInit);


	FLASH_COMMAND cmdInit;
	cmdInit.cmd = FLASH_CMD_INIT;

	xQueueSend(hEvtQueueFlash, &cmdInit, 0);
}

/*
初始化数据存储区
包括擦除 用于数据存储的 flash sector，更新系统状态，发送 FLASH_CMD_INIT 命令以初始化flash存储状态
*/

//void onCmdPrepareDataStorage(FLASH_COMMAND* fcmd, bool blEarseAllData, bool blRestartGathering)
void onCmdPrepareDataStorage(FLASH_COMMAND* fcmd, bool blEarseAllData)
{
//	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;

	//
//	systemStatus.flashStatus = FLASH_STATUS_INITIALIZING;

	// ----------------------------------------------------------------------------
	// 擦除数据存储sector
	WakeFlashUp();

	ReturnType rt = Flash_Success;
//	ReturnType rt = FlashBulkErase();

	//仅擦除数据存储的sector
	int endSector = INDEX_DATA_START_SECTOR + 3;

	if (blEarseAllData)
		endSector = INDEX_DATA_END_SECTOR;

	for (int s = INDEX_DATA_START_SECTOR; s <= endSector; s++)
	{
		SetFlashUsingTime(5);
		rt = FlashSectorErase(s);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }
	}

	// 更新系统状态
	systemStatus.blFlashInitialized = true;

	// ----------------------------------------------------------------------------
	// 发送命令初始化flash（重建索引）
//	FLASH_COMMAND* cmdInit = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmdInit->cmd = FLASH_CMD_INIT;
//
//	osMailPut(hFlashCommandQueue, cmdInit);


	FLASH_COMMAND cmdInit;
	cmdInit.cmd = FLASH_CMD_INIT;

	xQueueSend(hEvtQueueFlash, &cmdInit, 0);
}


time_t searchTimestampInSector(UINT32 sector, time_t t)
{
	//
	ReturnType rt;

	long sectorAddr = sector * pFlashInfo->sectorSize;
	long addr = sectorAddr + sizeof(FLASH_SECTOR_HEAD); // 跳过FLASH_SECTOR_HEAD. Atus???: Why skip sector_head check? maybe it is not used.

	FLASH_DATA_CHUNK_HEAD dch;
	//INDEX_DATA_HEAD cdh; //[BG025] remark.

	long sts = 0; // 用于搜索时记录sector内的时间戳

//	int sampleSize = 0;
//	int chunkSize = 0;

	//
	dataUploadPointer.sector = sector;

#ifdef FLASH_STORAGE_SCHEMA_V2

	for (int c = 0; c < ChunksPerSector; c++)
	{
		rt = FlashRead(addr + c * INDEX_DATA_CHUNK_SIZE, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // 读取数据块头
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d, size: %d\n", addr, rt, dch.timestamp, dch.channels, dch.size);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (dch.timestamp == 0 || dch.timestamp == DEFAULT_TIMESTAMP)
		{
			// 此时间戳无效，搜索结束
			break;
		}
		else
		{
			// 找到完全匹配时间戳
			if (dch.timestamp == t)
			{
				dataUploadPointer.offset = addr - sectorAddr;

				return t;
			}

			// 找到一个较小的时间戳，先记录
			if (dch.timestamp < t)
			{
				sts = dch.timestamp;
				continue;
			}

			// 找到一个较大的时间戳，则从稍小时间戳开始上传
			if (dch.timestamp > t)
			{
                          dataUploadPointer.offset = addr - sectorAddr; //Atus???: it always point to first chunk, should be + (c-1)*INDEX_DATA_CHUNK_SIZE

				return sts;
			}
		}
	}

#else

	while (addr < sectorAddr + pFlashInfo->sectorSize - LEAST_FLASH_DATA_CHUNK_SIZE)
	{
		rt = FlashRead(addr, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // 读取数据块头
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d\n", addr, rt, dch.timestamp, dch.channels);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (dch.timestamp == 0 || dch.timestamp == DEFAULT_TIMESTAMP)
		{
			// 此时间戳无效，搜索结束
			break;
		}
		else
		{
			// 找到完全匹配时间戳
			if (dch.timestamp == t)
			{
				dataUploadPointer.offset = addr - sectorAddr;
//				dataUploadPointer.capacity = getSectorDataSize(dataUploadPointer.sector);
//				dataUploadPointer.startTimestamp = t;

				return t;
			}

			// 找到一个较小的时间戳，先记录
			if (dch.timestamp < t)
			{
				sts = dch.timestamp;
				goto findNextChunk;
			}

			// 找到一个较大的时间戳，则从稍小时间戳开始上传
			if (dch.timestamp > t)
			{
				dataUploadPointer.offset = addr - sectorAddr;
//				dataUploadPointer.capacity = getSectorDataSize(dataUploadPointer.sector);
//				dataUploadPointer.startTimestamp = sts;

				return sts;
			}

findNextChunk:
//			// -----------------------------------------
//			//
//			addr += sizeof(FLASH_DATA_CHUNK_HEAD); // 跳过数据块头
//
//			for (int i = 0; i < dch.channels; i++)
//			{
//				// 逐个读取数据头
//				rt = FlashRead(addr, (BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));
//				CCTRACE("\t\taddr: %d, type: %d, int: %d, samp: %d\n", addr, cdh.type, cdh.interval, cdh.samples);
//
//				sampleSize = INDEX_DATA_SAMPLE_SIZE[cdh.type];
//				chunkSize = sampleSize * cdh.samples;
//
//				// 跳到下一种数据
//				addr += chunkSize + sizeof(INDEX_DATA_HEAD);
//			}
			addr += dch.size;
		}
	}

#endif

	return 0;
}

/* search the ring buffer to locate the sector by timestamp
 * param: time_t *pt: the timestamp buffer which will be search.
 * output: the sector number value and overwrite the real data timestamp near(floor) the request to *pt.
 *  fail locate return the 0xFFFF and without overwrite *pt.
 */
uint16_t SearchSectorForTimestamp(time_t* pt)  //Atus: SearchSecotorForTimestamp >> SearchSectorForTimestamp
{
	ReturnType rt;
	long addr = 0;
	FLASH_SECTOR_HEAD fsh;
	time_t t = *pt;

	CCTRACE("scanDataFromTimestamp >>>>>>\n");
	memset(&fsh, 0xff, sizeof(fsh)); //[BG030] reset to 0xff protect read flash fail without verify.
	// ----------------------------------------------------------------------------
	WakeFlashUp();

	time_t minTS = DEFAULT_TIMESTAMP;
	UINT32 minSector = 0;

	long sts = 0; 		// store sector startTS in searching
	long ets = 0; 		// store sector endTS in searching
	UINT32 sector = 0;	// store sector in searching passing

	// read FLASH_SECTOR_HEAD by sector, skip the reserved sector(not FLASH_SECTOR_TAG)
	/* work cases:
		case1:timestamp between startTS, endTS of one sector. return the sector and update TS with startTS
		case2:timestamp between conseutive two sectors. pre-endTS, ts, startTS. return pre-sector and update TS with pre-startTS.
		case3:timestamp less than all sector(small than oldest startTS). return oldest sector and update TS with oldest startTS.
		case4:timestamp more than all sector(more than current endTS). return 0xFFFF without update TS.
	*/
	/*Atus: the start from INDEX_DATA_START_SECTOR will make a bug, in case3, if the buffer ring exist a lose battery
	        timestamp back 2012, it will locate to the sector, and lost the previous data. 
	        Generally, the ring buffer should be a head and tail, and scan from head until tail.It should be start from pFlashStorageIndicator->sector next.
	 */
#if 1 //BG030_FIX
	for (int e=pFlashStorageIndicator->sector, s=getNextIndexDataSector(e); s != e; s=getNextIndexDataSector(s))
	{
		addr = s * pFlashInfo->sectorSize;
#if (BGXXX==6)
		test1.typeuint16[1] = (uint16_t)s;
#endif

		rt = FlashRead(addr, (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD));

		CCTRACE("\tsector: %d, rt=%d, tag: %X, idx: %d, sts: %lu, ets: %lu\n", s, rt, fsh.tag, fsh.index, fsh.startTimestamp, fsh.endTimestamp);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		// 找到真实时间戳
		if (fsh.tag == FLASH_SECTOR_TAG)
		{
			if (fsh.startTimestamp == DEFAULT_TIMESTAMP || fsh.endTimestamp == DEFAULT_TIMESTAMP)
			{
				// The sector is initialized only, not have data.
				// 但不能结束查找，因为可能后面有 sector 数据有效
				continue; //it will skip the partial data.
			}
			else
			{
				// 找到有有效数据的sector(tag有效，时间戳有效）

				// 保存最小的时间戳，（存在时间戳为0的异常情况）  //仅仅更新最小时间戳和已经扫描过的sector。
			  if (minTS==DEFAULT_TIMESTAMP && fsh.startTimestamp < minTS && fsh.startTimestamp != 0) //Atus: add minTS==DEFAULT_TIMESTAMP foce first valid sector set only.
				{ //just write first time valid timestamp, all of timestamp should be increment
				  // even back to 2012/1/1, we should think as previous time/ignore it.
					minTS = fsh.startTimestamp; //keep smallest timestamp.
					minSector = s;
				}

				if (fsh.startTimestamp <= t && t <= fsh.endTimestamp)
				{ // case1: found the sector we want.
					*pt = fsh.startTimestamp; //searchTimestampInSector(s, t);
					return s;
				}
				else if (fsh.endTimestamp < t && fsh.endTimestamp != 0)
				{
					// 找到一个较小的时间戳，先记录（存在时间戳为0的异常情况）   //要找的时间戳大于该sector的结束时间戳，记录下该sector的起始和结束时间戳。
					sector = s;
					sts = fsh.startTimestamp;
					ets = fsh.endTimestamp;
				}
				else if (ets > 0 && t < fsh.startTimestamp)
				{ //case2: found sector was pre-sector.
					// found a larger startTS, back to pre-sector.
					*pt = sts; //searchTimestampInSector(sector, ets);
					return sector;
				}
				else if (t < fsh.startTimestamp)
				{
				  	break; //it is increment, not necessary scan to tail.
				}
			}
		}
		else //if (fsh.tag != FLASH_SECTOR_TAG)
		{
			// 发现了未用的sector
			// 查找结束
			break;
		}
	}

	// 如果传入时间戳小于所有时间戳，则从最早的数据开始上传
	if (t < minTS && minTS != DEFAULT_TIMESTAMP)
	{ //case3: less than all(minTS)
		*pt = minTS;
		return minSector;
	}

	return 0xFFFF; //case4
#else
	for (int s = INDEX_DATA_START_SECTOR; s <= INDEX_DATA_END_SECTOR; s++)
	{
		addr = s * pFlashInfo->sectorSize;
#if (BGXXX==6)
		test1.typeuint16[1] = (uint16_t)s;
#endif

		rt = FlashRead(addr, (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD));

		CCTRACE("\tsector: %d, rt=%d, tag: %X, idx: %d, sts: %lu, ets: %lu\n", s, rt, fsh.tag, fsh.index, fsh.startTimestamp, fsh.endTimestamp);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		// 找到真实时间戳
		if (fsh.tag == FLASH_SECTOR_TAG)
		{
			if (fsh.startTimestamp == DEFAULT_TIMESTAMP || fsh.endTimestamp == DEFAULT_TIMESTAMP)
			{
				// The sector is initialized only, not have data.
				// 但不能结束查找，因为可能后面有 sector 数据有效
				continue; //it will skip the partial data.
			}
			else
			{
				// 找到有有效数据的sector(tag有效，时间戳有效）

				// 保存最小的时间戳，（存在时间戳为0的异常情况）  //仅仅更新最小时间戳和已经扫描过的sector。
				if (fsh.startTimestamp < minTS && fsh.startTimestamp != 0)
				{
					minTS = fsh.startTimestamp; //keep smallest timestamp.
					minSector = s;
				}

				if (fsh.startTimestamp <= t && fsh.endTimestamp >= t)
				{ // case1: found the sector we want.
					*pt = fsh.startTimestamp; //searchTimestampInSector(s, t);
					return s;
				}
				else if (fsh.endTimestamp < t && fsh.endTimestamp != 0)
				{
					// 找到一个较小的时间戳，先记录（存在时间戳为0的异常情况）   //要找的时间戳大于该sector的结束时间戳，记录下该sector的起始和结束时间戳。
					sector = s;
					sts = fsh.startTimestamp;
					ets = fsh.endTimestamp;
				}
				else if (ets > 0 && fsh.startTimestamp > t)
				{ //case2: found sector was pre-sector.
					// found a larger startTS, back to pre-sector.
					*pt = sts; //searchTimestampInSector(sector, ets);
					return sector;
				}
			}
		}
		else //if (fsh.tag != FLASH_SECTOR_TAG)
		{
			// 发现了未用的sector
			// 查找结束
			break;
		}
	}

	// 如果传入时间戳小于所有时间戳，则从最早的数据开始上传
	if (t < minTS && minTS != DEFAULT_TIMESTAMP)
	{ //case3: less than all(minTS)
		*pt = minTS;
		return minSector;
	}

	return 0xFFFF; //case4
#endif
}


//每次系统启动时都要扫描整个flash，确定本次是否可以启动后，可以从什么地址写采集数据
void onCmdFlashInit(FLASH_COMMAND* fcmd)
{
	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;

	ReturnType rt;
	long addr = 0;


	CCTRACE("FLASH_CMD_INIT >>\n");
//	systemStatus.flashStatus = FLASH_STATUS_INITIALIZING;
	SetFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);

	// ----------------------------------------------------------------------------
	/*
	扫描整个flash，以确定当前可用的sector及其存储状态（最后的可写入地址）
	扫描方法参见 指标数据采集与存储
	读取每个sector起始32个字节，FLASH_SECTOR_HEAD，并判断：
	1）如果没有tag,则找到可用的sector
	2）如果有Tag,判断该sector的endTimeStamp是否是DEFAULT_TIMESTAMP。如果是，则#3；如果不是，则5
	3）检测startTimeStamp,如果他是DEFAULT_TIMESTAMP，则已找到一个的sector。它是预备sector。
	4）检测startTimeStamp,如果他不是DEFAULT_TIMESTAMP，那么表示该sector还没有用完，只需要找到当前的存储索引。
	5）
	*/

	WakeFlashUp();

	//
	UINT32 sector = 0; //INDEX_DATA_START_SECTOR; //[BG030] 0 >> INDEX_DATA_START_SECTOR
	UINT32 maxidx = 0;
	FLASH_SECTOR_HEAD fsh;

	//bool findDefaultTimestamp = false; //[BG025] remark.

	// 逐sector读取前8个字节，跳过保留的sector
	/* 需考虑如下情况：

	    1. Sn在使用时，未预备Sn+1，但Sn+1中为有效旧数据，且序号大于Sn序号
	       如果跳过Sn，检查Sn+1，可能会导致下载旧的数据

		2. Sn在使用时，预备了Sn+1，此时两个sector都有正确的tag,index，但时间戳都为0xFFFFFFFF
		此时系统重启，则会有两个sector都有正确的tag,index，
		解决方法是增加对时间戳的检查

	*/
	for (int s = INDEX_DATA_START_SECTOR; s <= INDEX_DATA_END_SECTOR; s++)
	{
		addr = s * pFlashInfo->sectorSize;

		rt = FlashRead(addr, (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD));
		CCTRACE("sector: %d, rt=%d, tag: %X, idx: %d\n\tsts: %lu, ets: %lu",//, size: %d\n",
		        s, rt, fsh.tag, fsh.index, fsh.startTimestamp, fsh.endTimestamp);//, fsh.dataSize);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (fsh.tag == FLASH_SECTOR_TAG)// && fsh.startTimestamp != DEFAULT_TIMESTAMP && fsh.endTimestamp != DEFAULT_TIMESTAMP)
		{
			//这里表明该sector已经被格式化了
#ifdef DEBUG0
			pFlashStorageIndicator->sector = s;
			ScanSectorCapacity((FLASH_STORAGE_INDICATOR*)pFlashStorageIndicator);
#endif

			// 发现缺省时间戳
//			if (fsh.startTimestamp == DEFAULT_TIMESTAMP)
//			if (fsh.endTimestamp == DEFAULT_TIMESTAMP)
//			{
//				sector = s;
//				maxidx = fsh.index;
//				break;
//			}

			if (fsh.endTimestamp == DEFAULT_TIMESTAMP)
			{
				// 有两个可能，一个是当前sector没有任何数据，仅是预备sector
				// 二是有数据，但未结束
				// 如何判断？
				if (fsh.startTimestamp != DEFAULT_TIMESTAMP)
				{ //startTimestamp yes, without endTimestamp, indicate latest sector found.
					sector = s;
					maxidx = fsh.index;
					break;
				}
				else
				{ //both DEFAULT_TIMESTAMP, indicate prepare sector
					//sector = s; //[BG030] add
					//maxidx = fsh.index; //[BG030] add, we should know it is validate.
					break;  //sector的时间戳为缺省的那么就表示该sector可以用作最新数据存储的sector。所以不用在找了。

				}
			}

			if (fsh.index > maxidx)
			{
				// 读到更大的序号，需要继续找，
				sector = s;
				maxidx = fsh.index;
			}
			else
			{
				// 读到小的序号，就直接使用，不用在找了？？？
				break;
			}
		}
		else
		{
			// 发现了一个未初始化sector
			break;
		}
	}

	pFlashStorageIndicator->index = maxidx;  //这里找到当前可以存储数据sector，并获取了存储序号。
	pFlashStorageIndicator->sector = sector;     //找到新数据存储的起始序号和起始sector。


//	if (t < minTS && minTS != DEFAULT_TIMESTAMP)
//	{
//		dataUploadPointer.sector = minSector;
//		dataUploadPointer.offset = addr - sectorAddr;
//		dataUploadPointer.startTimestamp = minTS;
//	}

	// -------------------------------------------------
	// 扫描当前sector获得其剩余容量
	// 数据存储布局参见 指标数据采集与存储
	if (pFlashStorageIndicator->index > 0)
	{
		ScanSectorCapacity((FLASH_STORAGE_INDICATOR*)pFlashStorageIndicator); //返回当前sector所剩的容量。但是并没有使用。
	}

	// -------------------------------------------------
	// 初始化结束
	WaitFlashOpeartionDone(cmd, true);
	systemStatus.flashStatus = FLASH_STATUS_IDLE;

//	CCTRACE("FLASH_CMD_INIT <<\n\n");
}


// ============================================================================
// 预备一个sector
// 包括擦除sector，写入特定标志和新的索引号
void onCmdPrepareSector(FLASH_COMMAND* fcmd)
{
	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;
	CCTRACE("FLASH_CMD_PREPARE_SECTOR, sector: %d, idx: %d >>\n", fcmd->address.sector, fcmd->data.v);

	// 擦除一个sector
	// 擦除动作一般是预擦除，即在使用当前sector时，预先擦除下一个sector
	//
	systemStatus.flashStatus = FLASH_STATUS_WRITING;

	// -------------------------------------------------
	// 擦除指定sector
	WakeFlashUp();

	ReturnType rt = FlashSectorErase(fcmd->address.sector);

//	WaitFlashOpeartionDone(cmd, false);

	// -------------------------------------------------
	// 写入标志和索引号
	FLASH_SECTOR_HEAD fsh; //这里的时间戳全变成默认时间戳了。
	memset(&fsh, 0xFF, sizeof(FLASH_SECTOR_HEAD));
	fsh.tag = FLASH_SECTOR_TAG;
	fsh.index = fcmd->data.v; //这里是新的sector index。
	fsh.chunkSize = INDEX_DATA_CHUNK_SIZE;
	fsh.schemaVersion = DATA_STORAGE_SCHEMA_VERSION;

	rt = FlashProgram(fcmd->address.sector * pFlashInfo->sectorSize,
	                  (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD)); //把格式化的tag写到这个sector的最开始。

	WaitFlashOpeartionDone(cmd, true);//预备的sector准备好了发送一个notify出去。

	//
	systemStatus.flashStatus = FLASH_STATUS_IDLE;

//				CCTRACE("FLASH_CMD_PREPARE_SECTOR <<\n\n");
}

/*
在当前sector空间即将不足时，发送指令准备下一个sector。当当前sector还剩下最后一个chunk时，就准备下一个sector。
*/
void prepareNextFlashSector()
{
	if (pFlashStorageIndicator->nextSectorIsPrepared)
		return;

	pFlashStorageIndicator->nextSectorIsPrepared = true;

	// 当前sector空间即将不足，准备下一个sector
	int idx = pFlashStorageIndicator->index + 1; //这里把sector的存储序号加1.
	int sector = getNextIndexDataSector(pFlashStorageIndicator->sector);

//	FLASH_COMMAND* cmdInit = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmdInit->cmd = FLASH_CMD_PREPARE_SECTOR;
//	cmdInit->address.sector = sector;
//	cmdInit->data.v = idx;
//
//	osMailPut(hFlashCommandQueue, cmdInit);


	FLASH_COMMAND cmdFlash;
	cmdFlash.cmd = FLASH_CMD_PREPARE_SECTOR;
	cmdFlash.address.sector = sector;
	cmdFlash.data.v = idx;

	xQueueSend(hEvtQueueFlash, &cmdFlash, 0);
}


/*
切换到下一个sector
更新sector起始ts，结束ts信息，便于后续检索
*/
void switchToNextSector(bool updateTimestamp)
{
	ReturnType rt;

//	if (updateTimestamp)
	if (pFlashStorageIndicator->index > 0 && updateTimestamp)
	{
		// 更新timestamp
		FLASH_SECTOR_HEAD fsh;
		memset(&fsh, 0xFF, sizeof(FLASH_SECTOR_HEAD));

//		fsh.tag = 0xFFFFFFFF;
//		fsh.index = 0xFFFFFFFF;
//		fsh.tag = FLASH_SECTOR_TAG;
//		fsh.index = pFlashStorageIndicator->index;
		fsh.startTimestamp = pFlashStorageIndicator->startTimestamp;
		fsh.endTimestamp = pFlashStorageIndicator->endTimestamp;
//		fsh.dataSize = pFlashStorageIndicator->offset;

		//
		WakeFlashUp();

		CCTRACE("** switchToNextSector** sector: %d, sts: %lu, ets: %lu\n",
		        pFlashStorageIndicator->sector,
		        pFlashStorageIndicator->startTimestamp,
		        pFlashStorageIndicator->endTimestamp);
		rt = FlashProgram(pFlashStorageIndicator->sector * pFlashInfo->sectorSize,
		                  (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD)); //写入sector头信息
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		//
//		waitFlashOperaDone();
	}

	// 切换sector
	pFlashStorageIndicator->index++; //存储索引增加
	pFlashStorageIndicator->sector = getNextIndexDataSector(pFlashStorageIndicator->sector);

	pFlashStorageIndicator->offset = sizeof(FLASH_SECTOR_HEAD);//存储数据的真实位置。
#ifdef FLASH_STORAGE_SCHEMA_V2
	pFlashStorageIndicator->capacity = ChunksPerSector; //每到一个新的sector，那么它就有8个可用的chunk块。
#else
	pFlashStorageIndicator->capacity = pFlashInfo->sectorSize - pFlashStorageIndicator->offset;
#endif

	//新的sector里，把存储时间都设置成0.
	pFlashStorageIndicator->startTimestamp = 0;
	pFlashStorageIndicator->endTimestamp = 0;

	pFlashStorageIndicator->nextSectorIsPrepared = false;
}


//BYTE flashWritingBuffer[5];

// 根据 pFlashStorageIndicator 执行flash写入动作，
// 并调整 pFlashStorageIndicator 相应状态
void writeToFlash(BYTE* buffer, UINT32 size)
{
	ReturnType rt;
	long addr = 0;

	addr = pFlashStorageIndicator->sector * pFlashInfo->sectorSize
	       + pFlashStorageIndicator->offset;//找到写入数据的位置。

	rt = FlashProgram(addr, buffer, size);
	//Atus: Why did not check rt with error code, Flash_AddressInvalid...
	if (rt!=Flash_Success) //[BG025] add dumy check.
	{ }

	pFlashStorageIndicator->offset += size;
//#ifdef FLASH_STORAGE_SCHEMA_V2
//	pFlashStorageIndicator->capacity --;
//#else
	pFlashStorageIndicator->capacity -= size; //这里减是什么意思？？
//#endif
}


// 此函数将采集到的数据写入flash
// ！！仅适用于数据采集为双缓冲的情况！！
// 命令参数为缓冲区指针，缓冲区中包含了指标存储的信息
void onCmdSaveDataToFlash(FLASH_COMMAND* fcmd)
{
	MESSAGE msg;

	CCASSERT(fcmd);

	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;


	/* 分析数据
	 每次存储都有如下额外数据
	extra = 块头大小 + 数据头大小 * n
		 = 7 + 5 * n
	*/
	int dataSize;//, remainDataSize, writingBytes;
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) fcmd->data.p;
	CCASSERT(pbh);

	dataSize = sizeof(FLASH_DATA_CHUNK_HEAD) + 5 * GATHERABLE_INDEX_DATA_COUNT;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		dataSize += pbh->bufferIndicator[i].bufferOffset;
	}


	// =============================================================
	// 当前sector空间不足，切换到下一个sector
#ifdef FLASH_STORAGE_SCHEMA_V2

	if (pFlashStorageIndicator->capacity < 1)
#else
	if (pFlashStorageIndicator->capacity < dataSize)
#endif
	{
		switchToNextSector(true);

		// 因为是新切换过来的，因此更新 startTimestamp
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;
	}
	else if (pFlashStorageIndicator->startTimestamp <= 0)
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;

	// 每次写入数据，都更新 endTimestamp
	pFlashStorageIndicator->endTimestamp = pbh->timestamp;

	CCTRACE("\nFLASH_CMD_WRITE, sector: %d, offset: %d, size: %d, sts: %lu, ets: %lu >>\n",
	        pFlashStorageIndicator->sector, pFlashStorageIndicator->offset, dataSize,
	        pFlashStorageIndicator->startTimestamp,
	        pFlashStorageIndicator->endTimestamp);
//	systemStatus.flashStatus = FLASH_STATUS_WRITING;

	//
	WakeFlashUp();

	// =============================================================
	// 写入头

	FLASH_DATA_CHUNK_HEAD dch;
	dch.timestamp = pbh->timestamp;
	dch.channels = GATHERABLE_INDEX_DATA_COUNT;
#ifdef FLASH_STORAGE_SCHEMA_V2
	dch.timezoneOffset = pbh->timezoneOffset;
#else
	dch.size = dataSize;
#endif

	writeToFlash((BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD));

	// =============================================================
	// 写入数据
	INDEX_DATA_DEF* pDef = NULL;
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];
		pInd = &(pbh->bufferIndicator[i]);

		// =========================================================
		// 写入指标头
		dataSize = pInd->bufferOffset;;
//		CCTRACE("    totalData: %d, flashOffset: %d\n", dataSize, pFlashStorageIndicator->offset);

		INDEX_DATA_HEAD cdh = { pInd->type, pDef->sampleInterval,  pInd->samples };

		writeToFlash((BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));


		// =========================================================
		// 写入指标数据，仅写入有数据的指标
		if (pInd->samples > 0)
		{
			BYTE* buffer = (BYTE*) pbh;
			buffer += pInd->bufferPos;
			writeToFlash(buffer, pInd->bufferOffset);
		}
	}

	// 发送数据写完通知
//	WaitFlashOpeartionDone(cmd, true);
//	osMessagePut(hMsgInterrupt, (long)MESSAGE_FLASH_OPERATION_DONE + ((long)cmd << 16), 0);

	msg.params.type = MESSAGE_FLASH_OPERATION_DONE;
	msg.params.param = cmd;
	xQueueSend(hEvtQueueDevice, &msg.id, 0);
	//
	systemStatus.flashStatus = FLASH_STATUS_IDLE;

//	CCTRACE("FLASH_CMD_WRITE <<\n
}


// 此函数将采集到的数据写入flash，由data_gather.c调用
// ！！仅适用于数据采集为单缓冲的情况！！
// 指标存储情况在pbh中
// buffer缓冲区中只有指标数据，
FLASH_SECTOR_HEAD flashSectorHead;
void saveIndexDataToFlash(DATA_GATHER_BUFFER_HEAD* pbh, BYTE* buffer)
{
	// =============================================================
	// flash存储布局v2，将flash分为8个508字节的chunk
	// 每次写入都占用一个chunk，因此不需要检查flash剩余可用空间，
	// 只需要检查当前sector是否还有可用chunk即可

	// =============================================================
	// 当前sector空间不足，切换到下一个sector
#ifdef FLASH_STORAGE_SCHEMA_V2
	if (pFlashStorageIndicator->capacity < 1)
#else
	if (pFlashStorageIndicator->capacity < dataSize)
#endif
	{
		switchToNextSector(true);

		// 因为是新切换过来的，因此更新 startTimestamp
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;
	}
	else if (pFlashStorageIndicator->startTimestamp <= 0)
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;


	// 每次写入数据，都更新 endTimestamp
	pFlashStorageIndicator->endTimestamp = pbh->timestamp;

	CCTRACE("\nFLASH_CMD_WRITE, sector: %d, offset: %d, size: %d, sts: %lu, ets: %lu >>\n",
	        pFlashStorageIndicator->sector, pFlashStorageIndicator->offset, dataSize,
	        pFlashStorageIndicator->startTimestamp,
	        pFlashStorageIndicator->endTimestamp);
//	systemStatus.flashStatus = FLASH_STATUS_WRITING;

	//
	WakeFlashUp();

	// =============================================================
	// 写入头
	FLASH_DATA_CHUNK_HEAD dch;
	memset(&dch, 0xFF, sizeof(FLASH_DATA_CHUNK_HEAD));
	dch.timestamp = pbh->timestamp;
	dch.timezoneOffset = pbh->timezoneOffset;
	dch.channels = GATHERABLE_INDEX_DATA_COUNT;
#ifdef FLASH_STORAGE_SCHEMA_V2
#else
	dch.size = dataSize;
#endif

#ifdef FLASH_STORAGE_SCHEMA_V2

	// sector head address
	long addr = pFlashStorageIndicator->sector * pFlashInfo->sectorSize;

	if (pFlashStorageIndicator->capacity >= ChunksPerSector) //表示什么？？？
	{
		// 第一个 chunk，写入 start timestamp?
//		FLASH_SECTOR_HEAD fsh;
		memset(&flashSectorHead, 0xFF, sizeof(FLASH_SECTOR_HEAD));
		flashSectorHead.startTimestamp = pFlashStorageIndicator->startTimestamp;

		FlashProgram(addr, (BYTE*) &flashSectorHead, sizeof(FLASH_SECTOR_HEAD));//写入sector头信息
	}

	// skip sector head
	addr += sizeof(FLASH_SECTOR_HEAD);

	// 跳到指定的chunk
	addr += INDEX_DATA_CHUNK_SIZE * (ChunksPerSector - pFlashStorageIndicator->capacity);

	FlashProgram(addr, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); //写Chunk头信息
	addr += sizeof(FLASH_DATA_CHUNK_HEAD);

	// =============================================================
	// 写入数据
	// v2布局是先写入所有指标头，然后才是所有数据
	INDEX_DATA_DEF* pDef = NULL;
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	short dataOffsetBase = sizeof(FLASH_DATA_CHUNK_HEAD) + sizeof(INDEX_DATA_HEAD) * GATHERABLE_INDEX_DATA_COUNT;
//chunk头大小，加上指标头大小乘以所有指标个数	，然后才会是真正数据。

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];
		pInd = &(pbh->bufferIndicator[i]);

		// =========================================================
		// 写入指标头
		INDEX_DATA_HEAD cdh = { pInd->type, pDef->sampleInterval,  pInd->samples, pInd->bufferPos + dataOffsetBase};

		FlashProgram(addr, (BYTE*) &cdh, sizeof(INDEX_DATA_HEAD)); //这里写入的是指标的头信息。
		addr += sizeof(INDEX_DATA_HEAD);//这里一个指标一个指标的写入。
	}

	// 写入数据
	// buffer缓冲区中只有指标数据，但有效大小为 buffersize - FLASH_DATA_CHUNK_HEAD size - all INDEX_DATA_HEAD size
	FlashProgram(addr, buffer, INDEX_DATA_CHUNK_SIZE - sizeof(FLASH_DATA_CHUNK_HEAD) - GATHERABLE_INDEX_DATA_COUNT * sizeof(INDEX_DATA_HEAD));

	//
	pFlashStorageIndicator->capacity--; //每个sector只能有8个chunk，每写一个就减少一个。

	// 数据写完，发送事件通知
//	osMessagePut(hMsgInterrupt, (long)MESSAGE_FLASH_OPERATION_DONE + ((long)FLASH_CMD_WRITE << 16), 0);
	MESSAGE msg;
	msg.params.type = MESSAGE_FLASH_OPERATION_DONE;
	msg.params.param = FLASH_CMD_WRITE;
	xQueueSend(hEvtQueueDevice, &msg.id, 0);
#else

	writeToFlash((BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD));

	// v1布局是：指标头a+数据a+指标头b+数据b...
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pInd = &(pbh->bufferIndicator[i]);

		// =========================================================
		// 写入指标头
		dataSize = pInd->bufferOffset;;
//		CCTRACE("    totalData: %d, flashOffset: %d\n", dataSize, pFlashStorageIndicator->offset);

		INDEX_DATA_HEAD cdh = { pInd->type, INDEX_DATA_SAMPLE_INTERVAL[pInd->type],  pInd->samples };

		writeToFlash((BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));


		// =========================================================
		// 写入指标数据，仅写入有数据的指标
		if (pInd->samples > 0)
		{
			BYTE* buffer = (BYTE*) pbh;
			buffer += pInd->bufferPos;
			writeToFlash(buffer, pInd->bufferOffset);
		}
	}

#endif
}


// ============================================================================
// 扫描flash数据存储完成
// 如果未使用任何一个sector，或最后一个sector空间不足，则准备下一个sector
void onPostScanFlashStorage()
{
	CCTRACE("flash init done.\n");


	// ============================================================================
	// 启动数据采集
	StartDataGathering();//准备工作，主要是把已经开始采集置位


	// ============================================================================
	// 视需要发送 FLASH_CMD_PREPARE_SECTOR 命令
	if (pFlashStorageIndicator->index == 0)
	{
		// 说明未使用任何一个sector（如系统初次启动）
		pFlashStorageIndicator->sector = 0;
		pFlashStorageIndicator->capacity = 0; // 设置sector的capacity为0，因此在下次存储时，会切换到新sector
//这里是不是要保留第一个sector用作其他用处啊？？？？？
		prepareNextFlashSector();
	}

#ifdef FLASH_STORAGE_SCHEMA_V2
	else if (pFlashStorageIndicator->capacity < 2)	// 还剩一个chunk
#else
	else if (pFlashStorageIndicator->capacity < pFlashInfo->sectorSize / FLASH_MIN_CAPACITY_FACTOR)
#endif
	{
		CCTRACE("\n** sector %d out of space, switch to next sector\n",
		        pFlashStorageIndicator->sector);

		// 当前sector空间即将不足，预备下一个sector
		prepareNextFlashSector();
	}
	else
	{
		// set flash to power off
		ClearFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);
		PutFlashToSleep();
	}
}

// ============================================================================
// sector预备完毕
void onPostPreparedSector()
{
	CCTRACE("preparing sector done.\n");

	// set flash to power off
	ClearFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);
	PutFlashToSleep();
}

// ============================================================================
// 数据写入flash完成
// 调整flash存储参数
// 如果sector空间即将不足，则准备下一个sector
void onPostWroteDataToFlash()
{
	CCTRACE("flash-writing done.\n");

	//
#ifdef FLASH_STORAGE_SCHEMA_V2

	if (pFlashStorageIndicator->capacity < 2)	// 还剩一个chunk
#else
	if (pFlashStorageIndicator->capacity < pFlashInfo->sectorSize / FLASH_MIN_CAPACITY_FACTOR)
#endif
	{
		CCTRACE("\n** sector %d running out of space, prepare next sector\n",
		        pFlashStorageIndicator->sector);

		// 当前sector空间即将不足，准备下一个sector
		prepareNextFlashSector();
	}
	else
	{
		// set flash to power off
		ClearFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);
		PutFlashToSleep();
	}
}


// 读取下一块数据用于上传，需在外部唤醒flash
// 读取位置由 dataUploadPointer 指定
// 读取后更新 dataUploadPointer
//
// 返回值：
// true 表示还有数据可用
// false 表示数据全部读完
bool readNextUploadChunk(BYTE* buffer, int size)
{
	ReturnType rt;
	int addr = dataUploadPointer.sector * pFlashInfo->sectorSize + dataUploadPointer.offset; //找到真实数据的起始位置

	if (dataUploadPointer.capacity <= 0) //这个capacity在写入时表示该sector还有多少空间能够用来存数据，在读出时表示还有多少数据没有读。
		return false;//问题：这里的capacity表示当前sector的数据总量还是有多少个chunk块？？？

	if (dataUploadPointer.offset + size <= dataUploadPointer.capacity)
	{
		// 当前sector的数据足以填满buffer
		rt = FlashRead(addr, buffer, size);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		dataUploadPointer.offset += size;
		return true;
	}
	else
	{
		bool bufferHasRemain = false;
		int bufferOffset = 0;

		// 当前sector的数据不足以填满buffer
		int sectorRemain = dataUploadPointer.capacity - dataUploadPointer.offset;

		if (sectorRemain > 0)
		{
			// 当前sector还有数据
			rt = FlashRead(addr, buffer + bufferOffset, sectorRemain);
			//Atus: Why did not check rt with error code, Flash_AddressInvalid...
			if (rt!=Flash_Success) //[BG025] add dumy check.
			{ }

			bufferOffset += sectorRemain;
			size -= sectorRemain;
			bufferHasRemain = true;
		}

		// 切换到下一个sector
		dataUploadPointer.sector = getNextIndexDataSector(dataUploadPointer.sector);
		dataUploadPointer.offset = sizeof(FLASH_SECTOR_HEAD); // 切换到新sector后，初始化偏移必然为 sizeof(FLASH_SECTOR_HEAD)
//		dataUploadPointer.capacity = getSectorDataSize(dataUploadPointer.sector);

		if (dataUploadPointer.capacity > 0)
		{
			// 新sector有数据
			return readNextUploadChunk(buffer + bufferOffset, size);//这里出现递归调用
		}
		else
			return bufferHasRemain;
	}

//	return false;
}

#ifdef DEBUG
// 峈聆彸袧掘sector
void onCmdPrepareSectorForTesting(FLASH_COMMAND* fcmd)
{
#if 0
	// 呴儂恁寁?補跺sectorㄛ笠壺甜迡?sector head
	int from = INDEX_DATA_START_SECTOR;
	int to = (rand() % (INDEX_DATA_END_SECTOR / 10)) + from + 10;

	//
	FLASH_SECTOR_HEAD fsh;
	ReturnType rt = 0;

	WakeFlashUp();
//	WakeFlashUp();

	time_t ts = getCurrentTimestamp() - 7 * 24 * 3600;

	for (int s = from; s <= to; s++)
	{
		rt = FlashSectorErase(s);
//		WaitFlashOpeartionDone(fcmd->cmd, false);

		// -------------------------------------------------
		// 迡?梓祩睿坰竘瘍
		memset(&fsh, 0xFF, sizeof(FLASH_SECTOR_HEAD));
		fsh.tag = FLASH_SECTOR_TAG;
		fsh.index = s;
		fsh.startTimestamp = ts;
		ts += 3600;
		fsh.endTimestamp = ts;
		ts += 3600;

		rt = FlashProgram(s * pFlashInfo->sectorSize,
		                  (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD));

//		WaitFlashOpeartionDone(cmd, false);
	}

	WaitFlashOpeartionDone(fcmd->cmd, true);

#else

	WakeFlashUp();

	for (int i = INDEX_DATA_START_SECTOR; i <= INDEX_DATA_START_SECTOR; i++)
		FlashSectorErase(i);

//	FlashSectorErase(INDEX_DATA_START_SECTOR + 1);

#endif
}
#endif


#define BATCH_UPLOAD_SIZE	(50)



void FlashOpeartionCallback(BYTE flashCMD)
{
	switch (flashCMD)
	{
		case FLASH_CMD_INIT:
		{
			onPostScanFlashStorage();

			break;
		}

		case FLASH_CMD_PREPARE_SECTOR:
		{
			// sector预备完成
			onPostPreparedSector();

			break;
		}

		case FLASH_CMD_WRITE:
		{
			// 数据写完
			onPostWroteDataToFlash();

			break;
		}

#ifdef DEBUG

		case FLASH_CMD_PREPARE_SECTOR_FOR_TESTING:
		{
			// 测试sector准备完毕
			break;
		}

#endif
	}
}



//==============================================================================

BYTE SearchSectorIndex(uint32_t indexSearch, uint16_t* sectorNo, bool blStart)
{
	/****
	算法描述：
	1）搜索范围是 sector number 512  到 sector number 2047，逐个读取每个sector的头信息
	2）查看sector tag是否有效；
	   2.1 如果有效，再读sector index，在进行比较
			 2.1.1 如果sector index和index 一直，退出整个循环；result=05并返回
			 2.1.2 如果不一致，继续读取下一个sector的头信息
	   2.2 如果无效，退出循环。
	***/

	BYTE result = 0;
	uint16_t i = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = INDEX_DATA_START_SECTOR; i <= INDEX_DATA_END_SECTOR; i++ )
	{
		addr = i * pFlashInfo->sectorSize;
		FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

		if(fsh.tag == FLASH_SECTOR_TAG)
		{
			if((fsh.index == indexSearch))
			{
				//说明找到了指定index
				if((fsh.endTimestamp !=  DEFAULT_TIMESTAMP))
				{
					if(blStart)
					{
						result = 0x01;
					}
					else
						result = 0x03;

					*sectorNo = i;//获取当前的sector number;
				}
				else
				{
					if(blStart)
					{
						result = 0x05;//指定的sector还么有采集满
					}
					else
						result = 0x06;
				}

				return result;
			}
			else
				continue;
		}
		else
		{
#if 0

			//一旦发现有sector的tag不正确，说明接下来的flash中没有存储采集数据，故不扫描了
			if(blStart)
				result = 0x02;
			else
				result = 0x04;

			return result;
			//有时候，一个sector的头信息中index=-1，时间戳都是FF，tag也都是FFFF,所以碰到坏tag的sector，不能停止搜寻。这样就会有一个问题，在一段区间中可能包含坏tag的sector
#endif
			continue;
		}
	}


	if(i == INDEX_DATA_END_SECTOR + 1)
	{
		result = 0x07;//当前要找的sector index不存在。
		return result;
	}

	return result;
}


#if 0
//下面就是几种查找sector index和sector number对的方法。
void CombinIndexNumberOne(SECTOR_NODE* sectorNode, int startSectorIndex, int totalNum)
{
	uint16_t i = 0;
	uint16_t j = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = 0; i <= totalNum; i++)
	{
		//顶层循环处理要找的所有sector

		for(j = INDEX_DATA_START_SECTOR; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//只要调用该函数意味着，起始和结束index都已经找到了，这里就不判读tag有效性了
			if(fsh.index == startSectorIndex)
			{
				//找到index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//查找下一个index,还是从头找，这里有区间跨flashsector尾的情况。可以分段找，效率高；
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}
	}
}

//这个写法很耗时，如果一个区间包含较多的不存在index的话。没遇到这样一个sector index，只有扫描了整个flash的sector后才能做决定。
void CombinIndexNumberTwo(SECTOR_NODE* sectorNode, int startSectorIndex, int totalNum, int* counter)
{
	uint16_t i = 0;
	uint16_t j = 0;
	int nonExistIndex = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = 0; i <= totalNum; i++)
	{
		//顶层循环处理要找的所有sector

		for(j = INDEX_DATA_START_SECTOR; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//只要调用该函数意味着，起始和结束index都已经找到了，这里就不判读tag有效性了
			if(fsh.index == startSectorIndex)
			{
				//找到index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//查找下一个index,还是从头找，这里有区间跨flashsector尾的情况。可以分段找，效率高；
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}

		if(j == INDEX_DATA_END_SECTOR + 1)
		{
			//如果把整个sector都搜索一遍后，还没有这个index的话，就去找下一个index，否则程序会卡死在这里一直找，其他的部分不能运行。
			startSectorIndex++;//这样的结果是sectorNode指针指向的区域中值保留了找的到的那些sector的索引和真实的sector编号。
			nonExistIndex++;
		}
	}

	* counter = nonExistIndex;
}


void CombinIndexNumber(SECTOR_NODE* sectorNode, int startSectorIndex, int totalNum, int* counter)
{
	uint16_t i = 0;
	uint16_t j = 0;
	int nonExistIndex = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = 0; i <= totalNum; i++)
	{
		//顶层循环处理要找的所有sector

		for(j = INDEX_DATA_START_SECTOR; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//只要调用该函数意味着，起始和结束index都已经找到了，这里就不判读tag有效性了
			if(fsh.index == startSectorIndex)
			{
				//找到index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//查找下一个index,还是从头找，这里有区间跨flashsector尾的情况。可以分段找，效率高；
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}

		if(j == INDEX_DATA_END_SECTOR + 1)
		{
			//表明某个sector index在整个flash中都没有。
			* counter = startSectorIndex;
			break;//就退出整个扫描过程，把前面所有的index的数据下载下来，后面的就不找了。
		}
	}
}

#endif

void CombinIndexNumber(SECTOR_NODE* sectorNode, int startSectorIndex, int startSectorNumber, int totalNum, int* counter)
{
	uint16_t i = 0;
	uint16_t j = 0;
	uint16_t k = 0;
	//int nonExistIndex = 0; //[BG025] remark.
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = 0; i <= totalNum; i++)
	{
		//顶层循环处理要找的所有sector

		for(j = startSectorNumber; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			//先找后半部分
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//只要调用该函数意味着，起始和结束index都已经找到了，这里就不判读tag有效性了
			if(fsh.index == startSectorIndex)
			{
				//找到index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//查找下一个index,还是从头找，这里有区间跨flashsector尾的情况。可以分段找，效率高；
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}


		if(j == INDEX_DATA_END_SECTOR + 1)
		{
			//这里表明，在后半部分都没有找到该index，将要从起始位置找。
			for(k = INDEX_DATA_START_SECTOR; k < startSectorNumber ;k++)
			{
				//扫描前半部分
				addr = k * pFlashInfo->sectorSize;
				FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

				if(fsh.index == startSectorIndex)
				{
					//找到index
					sectorNode->sectorIndex = startSectorIndex;
					sectorNode->sectorNumber = j;

					//查找下一个index,还是从头找，这里有区间跨flashsector尾的情况。可以分段找，效率高；
					sectorNode++;
					startSectorIndex++;
					break;
				}
			}
		}


		if((j == INDEX_DATA_END_SECTOR + 1)  && (k == startSectorNumber-1))
		{
			//表明某个sector index在整个flash中都没有。
			* counter = startSectorIndex;
			break;//就退出整个扫描过程，把前面所有的index的数据下载下来，后面的就不找了。
		}
	}
}


//该函数返回当前flash的sector index的最大值。
int SearchMaxSectorIndex(uint16_t *secnum)
{
	/***
	扫描整个flash的范围：512 sector ----2047 sector,但是当发现有非法的tag时，就不用完下扫描了，就把当前最大的
	index 当成flash的所有sector中最大的存储索引
	***/
  
	
	int maxIndex = 0;
#if 1  //BG030_FIX
	if (pFlashStorageIndicator->nextSectorIsPrepared==true)
	{
		maxIndex = pFlashStorageIndicator->index-1;
		if (secnum!=NULL)
			*secnum = getPreIndexDataSector(pFlashStorageIndicator->sector);
	}
	else
	{
	  	maxIndex = pFlashStorageIndicator->index;
		if (secnum!=NULL)
			*secnum = pFlashStorageIndicator->sector;
	}
#else
	uint16_t i = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;
	
	for(i = INDEX_DATA_START_SECTOR; i <= INDEX_DATA_END_SECTOR; i++ )
	{
		addr = i * pFlashInfo->sectorSize;
		FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

		if(fsh.tag == FLASH_SECTOR_TAG)
		{
			if(fsh.index > maxIndex)
				maxIndex = fsh.index;
		}
		else
			continue;
	}
#endif
	return maxIndex;
}

void FlashTask(void* argument)
{
	//
//	osEvent osEvt;
//	ENUM_FLASH_COMMANDS cmd;

	static FLASH_COMMAND cmdFlash;
	ENUM_FLASH_COMMANDS cmd;


	while (1)
	{
//		osEvt = osMailGet(hFlashCommandQueue, osWaitForever);
//		FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osEvt.value.p;

		xQueueReceive(hEvtQueueFlash, &cmdFlash, portMAX_DELAY);
		cmd = (ENUM_FLASH_COMMANDS) cmdFlash.cmd;

//		FLASH_COMMAND* fcmd = (FLASH_COMMAND*)cmdFlash.data.p;

//#ifdef DEBUG_MODE
//		TEST_PORT4_SET();
//#endif

		// =========================================
		// mcu fault handle
		errlocated.pspBottom = __get_PSP();
		errlocated.controlRegisterValue = __get_CONTROL();

		//
		switch (cmd)
		{
			case FLASH_CMD_DATA_GATHER_TICK: //每秒中都会发送该消息
			{
				// 发送心跳消息
				MESSAGE msg;
				msg.params.type = MESSAGE_TASK_HEARTBEAT;
				msg.params.param = 0x04;

//				osMessagePut(hMsgInterrupt, msg.id, 0);
				xQueueSend(hEvtQueueDevice, &msg.id, 0);
				//开始采集数据
				doDataGathering();
				break;
			}

			case FLASH_CMD_PREPARE_CHIP: //没有消息发送过来。
			{
				onCmdInitChip(&cmdFlash);

				//
				break;
			}

			case FLASH_CMD_PREPARE_DATA_STORAGE: //仅仅在启动时发送一次该消息
			{
//				bool totallyEarse = (fcmd->data.v == 1);
				bool blTotallyEarse = (cmdFlash.data.v == 1);
				onCmdPrepareDataStorage(&cmdFlash, blTotallyEarse);

				//
				break;
			}

			case FLASH_CMD_INIT:
			{
				onCmdFlashInit(&cmdFlash);
				//
				break;
			}

			case FLASH_CMD_PREPARE_SECTOR://就是往写的sector中写入Tag标志和新的索引号
			{
				onCmdPrepareSector(&cmdFlash);

				//
				break;
			}

			case FLASH_CMD_WRITE:
			{
				onCmdSaveDataToFlash(&cmdFlash);

				//
				break;
			}

			case FLASH_CMD_UPLOAD:
			{



				break;
			}

#ifdef DEBUG
			case FLASH_CMD_PREPARE_SECTOR_FOR_TESTING:
			{
//				if (fcmd->data.p == NULL)
//					break;

				onCmdPrepareSectorForTesting(&cmdFlash);

				break;
			}
#endif

			case FLASH_CMD_RESET_DATA_GATHER:
			{
				bool blTotallyEarse = (cmdFlash.data.v == 1);
//				// 低16位：1=全部擦除；0=仅擦除若干起始sector
//				// 高16位：1=完成后启动采集；0=完成后无动作
//				bool blTotallyEarse = ((fcmd->data.v && 0xFF) == 1);
//				bool blRestartGathering = (((fcmd->data.v >> 16) && 0xFF) == 1);

				LED_ON();
				SetFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);

				StopDataGathering(false);

//				onCmdPrepareDataStorage(NULL, blTotallyEarse, blRestartGathering);
				onCmdPrepareDataStorage(NULL, blTotallyEarse);

//				StartDataGathering();
				ClearFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);
				LED_OFF();

				break;
			}

			default:
				break;
		}

		// 释放事件存储
//		osMailFree(hFlashCommandQueue, fcmd);

//#ifdef DEBUG_MODE
//		TEST_PORT4_CLEAR();
//#endif
	}
}

BaseType_t createFlashTask()
{
	return xTaskCreate(FlashTask, FLASH_TASK_NAME, FLASH_TASK_STACK_DEPTH, 0, FLASH_TASK_PRIORITY, NULL);
}
