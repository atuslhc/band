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
#define FLASH_TASK_STACK_DEPTH	(128)	// ��ջ����
#define FLASH_TASK_PRIORITY		(0)

#define FLASH_TASK_QUEUE_LEN			5
#define FLASH_TASK_QUEUE_ITME_SIZE		sizeof(FLASH_COMMAND)

QueueHandle_t hEvtQueueFlash = 0;



//FLASH_STORAGE_INDICATOR dataStartSector;

// ���ڱ��������ϴ���λ��
// ��Ч������
//		sector:
//		offset: ��ȡ����λ�ã��Ե�ǰsector��ʼ��ַ
//		capacity: ��ǰsector��������
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
	SetFlashUsingTime(30);//��������һ��ֵ30������ֵ����0ʱ���ͻ��flash�ص���

	if (FLASH_POWER_BACK())
		vTaskDelay(1);
}

// ����flashæ
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


// blNotify���Ƿ���֪ͨ�¼�
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

// �ȴ���ʱ��������
// �ĳ� vTaskDelay ������ʱ����ֹwatchdog����
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
ÿ�����ݴ洢����ʱ�����ͷ(4�ֽ�)�����һ���ֽڵ�ָ����(1�ֽ�)��
Ȼ���ǵ�һ��ָ��Ĵ洢ͷ�����͡�����Ƶ�ʡ�samples����5�ֽڣ����Լ�����1���ֽڵ����ݣ�
��ÿ������������ 11�ֽ�
*/

/*
ɨ��һ��sector��ȷ����ʣ������
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
	long addr = sectorAddr;//���Ȼ�ȡ��sector��ʵ�ʵ�ַ���ڼ���sectorͷ��ƫ��
	addr += sizeof(FLASH_SECTOR_HEAD); // ����FLASH_SECTOR_HEAD

#ifdef FLASH_STORAGE_SCHEMA_V2

	time_t minTS = DEFAULT_TIMESTAMP;

	fsi->capacity = 0;

	for (int c = 0; c < ChunksPerSector; c++)
	{
		//���������chunk��ʱ���
		rt = FlashRead(addr + c * INDEX_DATA_CHUNK_SIZE, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // ��ȡ���ݿ�ͷ
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d, size: %d\n", addr, rt, dch.timestamp, dch.channels, dch.size); //[BG025] add rt
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success)
		{ }

		if (dch.timestamp <= 0 || dch.timestamp == DEFAULT_TIMESTAMP)
		{
			// ��ʱ�����Ч����������
			// ��ǰ��ַΪ������д���ַ
			fsi->capacity = ChunksPerSector - c; //fsi->capacity��ʾʣ�µĿ��ÿռ�
			break;
		}
		else
		{
			if (dch.timestamp < minTS)
				minTS = dch.timestamp;//��ȡ��ǰ������ʱ�䣬��������
		}
	}

	fsi->startTimestamp = minTS;//���ϴ�ɨ��õ���ʱ�����������ʼʱ�����

	return fsi->capacity; //��sector�м�ȥ�Ѿ�ɨ�����chunk������ʣ�µľͿ��������洢�µ����ݡ�

#else
	long sts = 0, ets = 0; //[BG025] move upper scope into here.

	while (addr < sectorAddr + pFlashInfo->sectorSize - LEAST_FLASH_DATA_CHUNK_SIZE)
	{
		rt = FlashRead(addr, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // ��ȡ���ݿ�ͷ
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d, size: %d\n", addr, rt, dch.timestamp, dch.channels, dch.size); //[BG025] add rt
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (dch.timestamp <= 0 || dch.timestamp == DEFAULT_TIMESTAMP || dch.size < 0)
		{
			// ��ʱ�����Ч����������
			// ��ǰ��ַΪ������д���ַ
			fsi->offset = addr - sectorAddr;
			fsi->capacity = pFlashInfo->sectorSize - fsi->offset;
			fsi->startTimestamp = sts;
			fsi->endTimestamp = ets;

			CCTRACE("\tcapacity: %d\n", fsi->capacity);
			return fsi->capacity;
		}
		else
		{
			// ��Чʱ���
			if (sts <= 0)
				sts = dch.timestamp;

			ets = dch.timestamp;

			//
//#ifdef DEBUG
//			addr += sizeof(FLASH_DATA_CHUNK_HEAD); // �������ݿ�ͷ
//
//			for (int i = 0; i < dch.channels; i++)
//			{
//				// �����ȡ����ͷ
//				rt = FlashRead(addr, (BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));
//				CCTRACE("\t\taddr: %d, type: %d, int: %d, samp: %d\n", addr, cdh.type, cdh.interval, cdh.samples);
//
//				sampleSize = INDEX_DATA_SAMPLE_SIZE[cdh.type];
//				chunkSize = sampleSize * cdh.samples;
//
//				// ������һ������
//				addr += chunkSize + sizeof(INDEX_DATA_HEAD);
//			}
//#else
			addr += dch.size;
//#endif
		}
	}

	// δ�ҵ����пռ䣬��������ֱ�ӷ���0
	return 0;

#endif
}

/*
��ʼ��flashоƬ
��������flashоƬ������ϵͳ״̬������ FLASH_CMD_INIT �����Գ�ʼ��flash�洢״̬
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

	// ����ϵͳ״̬
	systemStatus.blFlashInitialized = true;

	// ----------------------------------------------------------------------------
	// ���������ʼ��flash���ؽ�������
//	FLASH_COMMAND* cmdInit = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//	cmdInit->cmd = FLASH_CMD_INIT;
//
//	osMailPut(hFlashCommandQueue, cmdInit);


	FLASH_COMMAND cmdInit;
	cmdInit.cmd = FLASH_CMD_INIT;

	xQueueSend(hEvtQueueFlash, &cmdInit, 0);
}

/*
��ʼ�����ݴ洢��
�������� �������ݴ洢�� flash sector������ϵͳ״̬������ FLASH_CMD_INIT �����Գ�ʼ��flash�洢״̬
*/

//void onCmdPrepareDataStorage(FLASH_COMMAND* fcmd, bool blEarseAllData, bool blRestartGathering)
void onCmdPrepareDataStorage(FLASH_COMMAND* fcmd, bool blEarseAllData)
{
//	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;

	//
//	systemStatus.flashStatus = FLASH_STATUS_INITIALIZING;

	// ----------------------------------------------------------------------------
	// �������ݴ洢sector
	WakeFlashUp();

	ReturnType rt = Flash_Success;
//	ReturnType rt = FlashBulkErase();

	//���������ݴ洢��sector
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

	// ����ϵͳ״̬
	systemStatus.blFlashInitialized = true;

	// ----------------------------------------------------------------------------
	// ���������ʼ��flash���ؽ�������
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
	long addr = sectorAddr + sizeof(FLASH_SECTOR_HEAD); // ����FLASH_SECTOR_HEAD. Atus???: Why skip sector_head check? maybe it is not used.

	FLASH_DATA_CHUNK_HEAD dch;
	//INDEX_DATA_HEAD cdh; //[BG025] remark.

	long sts = 0; // ��������ʱ��¼sector�ڵ�ʱ���

//	int sampleSize = 0;
//	int chunkSize = 0;

	//
	dataUploadPointer.sector = sector;

#ifdef FLASH_STORAGE_SCHEMA_V2

	for (int c = 0; c < ChunksPerSector; c++)
	{
		rt = FlashRead(addr + c * INDEX_DATA_CHUNK_SIZE, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // ��ȡ���ݿ�ͷ
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d, size: %d\n", addr, rt, dch.timestamp, dch.channels, dch.size);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (dch.timestamp == 0 || dch.timestamp == DEFAULT_TIMESTAMP)
		{
			// ��ʱ�����Ч����������
			break;
		}
		else
		{
			// �ҵ���ȫƥ��ʱ���
			if (dch.timestamp == t)
			{
				dataUploadPointer.offset = addr - sectorAddr;

				return t;
			}

			// �ҵ�һ����С��ʱ������ȼ�¼
			if (dch.timestamp < t)
			{
				sts = dch.timestamp;
				continue;
			}

			// �ҵ�һ���ϴ��ʱ����������Сʱ�����ʼ�ϴ�
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
		rt = FlashRead(addr, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); // ��ȡ���ݿ�ͷ
		CCTRACE("\taddr: %d, rt=%d, ts: %lu, channels: %d\n", addr, rt, dch.timestamp, dch.channels);
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		if (dch.timestamp == 0 || dch.timestamp == DEFAULT_TIMESTAMP)
		{
			// ��ʱ�����Ч����������
			break;
		}
		else
		{
			// �ҵ���ȫƥ��ʱ���
			if (dch.timestamp == t)
			{
				dataUploadPointer.offset = addr - sectorAddr;
//				dataUploadPointer.capacity = getSectorDataSize(dataUploadPointer.sector);
//				dataUploadPointer.startTimestamp = t;

				return t;
			}

			// �ҵ�һ����С��ʱ������ȼ�¼
			if (dch.timestamp < t)
			{
				sts = dch.timestamp;
				goto findNextChunk;
			}

			// �ҵ�һ���ϴ��ʱ����������Сʱ�����ʼ�ϴ�
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
//			addr += sizeof(FLASH_DATA_CHUNK_HEAD); // �������ݿ�ͷ
//
//			for (int i = 0; i < dch.channels; i++)
//			{
//				// �����ȡ����ͷ
//				rt = FlashRead(addr, (BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));
//				CCTRACE("\t\taddr: %d, type: %d, int: %d, samp: %d\n", addr, cdh.type, cdh.interval, cdh.samples);
//
//				sampleSize = INDEX_DATA_SAMPLE_SIZE[cdh.type];
//				chunkSize = sampleSize * cdh.samples;
//
//				// ������һ������
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

		// �ҵ���ʵʱ���
		if (fsh.tag == FLASH_SECTOR_TAG)
		{
			if (fsh.startTimestamp == DEFAULT_TIMESTAMP || fsh.endTimestamp == DEFAULT_TIMESTAMP)
			{
				// The sector is initialized only, not have data.
				// �����ܽ������ң���Ϊ���ܺ����� sector ������Ч
				continue; //it will skip the partial data.
			}
			else
			{
				// �ҵ�����Ч���ݵ�sector(tag��Ч��ʱ�����Ч��

				// ������С��ʱ�����������ʱ���Ϊ0���쳣�����  //����������Сʱ������Ѿ�ɨ�����sector��
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
					// �ҵ�һ����С��ʱ������ȼ�¼������ʱ���Ϊ0���쳣�����   //Ҫ�ҵ�ʱ������ڸ�sector�Ľ���ʱ�������¼�¸�sector����ʼ�ͽ���ʱ�����
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
			// ������δ�õ�sector
			// ���ҽ���
			break;
		}
	}

	// �������ʱ���С������ʱ����������������ݿ�ʼ�ϴ�
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

		// �ҵ���ʵʱ���
		if (fsh.tag == FLASH_SECTOR_TAG)
		{
			if (fsh.startTimestamp == DEFAULT_TIMESTAMP || fsh.endTimestamp == DEFAULT_TIMESTAMP)
			{
				// The sector is initialized only, not have data.
				// �����ܽ������ң���Ϊ���ܺ����� sector ������Ч
				continue; //it will skip the partial data.
			}
			else
			{
				// �ҵ�����Ч���ݵ�sector(tag��Ч��ʱ�����Ч��

				// ������С��ʱ�����������ʱ���Ϊ0���쳣�����  //����������Сʱ������Ѿ�ɨ�����sector��
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
					// �ҵ�һ����С��ʱ������ȼ�¼������ʱ���Ϊ0���쳣�����   //Ҫ�ҵ�ʱ������ڸ�sector�Ľ���ʱ�������¼�¸�sector����ʼ�ͽ���ʱ�����
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
			// ������δ�õ�sector
			// ���ҽ���
			break;
		}
	}

	// �������ʱ���С������ʱ����������������ݿ�ʼ�ϴ�
	if (t < minTS && minTS != DEFAULT_TIMESTAMP)
	{ //case3: less than all(minTS)
		*pt = minTS;
		return minSector;
	}

	return 0xFFFF; //case4
#endif
}


//ÿ��ϵͳ����ʱ��Ҫɨ������flash��ȷ�������Ƿ���������󣬿��Դ�ʲô��ַд�ɼ�����
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
	ɨ������flash����ȷ����ǰ���õ�sector����洢״̬�����Ŀ�д���ַ��
	ɨ�跽���μ� ָ�����ݲɼ���洢
	��ȡÿ��sector��ʼ32���ֽڣ�FLASH_SECTOR_HEAD�����жϣ�
	1�����û��tag,���ҵ����õ�sector
	2�������Tag,�жϸ�sector��endTimeStamp�Ƿ���DEFAULT_TIMESTAMP������ǣ���#3��������ǣ���5
	3�����startTimeStamp,�������DEFAULT_TIMESTAMP�������ҵ�һ����sector������Ԥ��sector��
	4�����startTimeStamp,���������DEFAULT_TIMESTAMP����ô��ʾ��sector��û�����ֻ꣬��Ҫ�ҵ���ǰ�Ĵ洢������
	5��
	*/

	WakeFlashUp();

	//
	UINT32 sector = 0; //INDEX_DATA_START_SECTOR; //[BG030] 0 >> INDEX_DATA_START_SECTOR
	UINT32 maxidx = 0;
	FLASH_SECTOR_HEAD fsh;

	//bool findDefaultTimestamp = false; //[BG025] remark.

	// ��sector��ȡǰ8���ֽڣ�����������sector
	/* �迼�����������

	    1. Sn��ʹ��ʱ��δԤ��Sn+1����Sn+1��Ϊ��Ч�����ݣ�����Ŵ���Sn���
	       �������Sn�����Sn+1�����ܻᵼ�����ؾɵ�����

		2. Sn��ʹ��ʱ��Ԥ����Sn+1����ʱ����sector������ȷ��tag,index����ʱ�����Ϊ0xFFFFFFFF
		��ʱϵͳ���������������sector������ȷ��tag,index��
		������������Ӷ�ʱ����ļ��

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
			//���������sector�Ѿ�����ʽ����
#ifdef DEBUG0
			pFlashStorageIndicator->sector = s;
			ScanSectorCapacity((FLASH_STORAGE_INDICATOR*)pFlashStorageIndicator);
#endif

			// ����ȱʡʱ���
//			if (fsh.startTimestamp == DEFAULT_TIMESTAMP)
//			if (fsh.endTimestamp == DEFAULT_TIMESTAMP)
//			{
//				sector = s;
//				maxidx = fsh.index;
//				break;
//			}

			if (fsh.endTimestamp == DEFAULT_TIMESTAMP)
			{
				// ���������ܣ�һ���ǵ�ǰsectorû���κ����ݣ�����Ԥ��sector
				// ���������ݣ���δ����
				// ����жϣ�
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
					break;  //sector��ʱ���Ϊȱʡ����ô�ͱ�ʾ��sector���������������ݴ洢��sector�����Բ��������ˡ�

				}
			}

			if (fsh.index > maxidx)
			{
				// �����������ţ���Ҫ�����ң�
				sector = s;
				maxidx = fsh.index;
			}
			else
			{
				// ����С����ţ���ֱ��ʹ�ã����������ˣ�����
				break;
			}
		}
		else
		{
			// ������һ��δ��ʼ��sector
			break;
		}
	}

	pFlashStorageIndicator->index = maxidx;  //�����ҵ���ǰ���Դ洢����sector������ȡ�˴洢��š�
	pFlashStorageIndicator->sector = sector;     //�ҵ������ݴ洢����ʼ��ź���ʼsector��


//	if (t < minTS && minTS != DEFAULT_TIMESTAMP)
//	{
//		dataUploadPointer.sector = minSector;
//		dataUploadPointer.offset = addr - sectorAddr;
//		dataUploadPointer.startTimestamp = minTS;
//	}

	// -------------------------------------------------
	// ɨ�赱ǰsector�����ʣ������
	// ���ݴ洢���ֲμ� ָ�����ݲɼ���洢
	if (pFlashStorageIndicator->index > 0)
	{
		ScanSectorCapacity((FLASH_STORAGE_INDICATOR*)pFlashStorageIndicator); //���ص�ǰsector��ʣ�����������ǲ�û��ʹ�á�
	}

	// -------------------------------------------------
	// ��ʼ������
	WaitFlashOpeartionDone(cmd, true);
	systemStatus.flashStatus = FLASH_STATUS_IDLE;

//	CCTRACE("FLASH_CMD_INIT <<\n\n");
}


// ============================================================================
// Ԥ��һ��sector
// ��������sector��д���ض���־���µ�������
void onCmdPrepareSector(FLASH_COMMAND* fcmd)
{
	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;
	CCTRACE("FLASH_CMD_PREPARE_SECTOR, sector: %d, idx: %d >>\n", fcmd->address.sector, fcmd->data.v);

	// ����һ��sector
	// ��������һ����Ԥ����������ʹ�õ�ǰsectorʱ��Ԥ�Ȳ�����һ��sector
	//
	systemStatus.flashStatus = FLASH_STATUS_WRITING;

	// -------------------------------------------------
	// ����ָ��sector
	WakeFlashUp();

	ReturnType rt = FlashSectorErase(fcmd->address.sector);

//	WaitFlashOpeartionDone(cmd, false);

	// -------------------------------------------------
	// д���־��������
	FLASH_SECTOR_HEAD fsh; //�����ʱ���ȫ���Ĭ��ʱ����ˡ�
	memset(&fsh, 0xFF, sizeof(FLASH_SECTOR_HEAD));
	fsh.tag = FLASH_SECTOR_TAG;
	fsh.index = fcmd->data.v; //�������µ�sector index��
	fsh.chunkSize = INDEX_DATA_CHUNK_SIZE;
	fsh.schemaVersion = DATA_STORAGE_SCHEMA_VERSION;

	rt = FlashProgram(fcmd->address.sector * pFlashInfo->sectorSize,
	                  (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD)); //�Ѹ�ʽ����tagд�����sector���ʼ��

	WaitFlashOpeartionDone(cmd, true);//Ԥ����sector׼�����˷���һ��notify��ȥ��

	//
	systemStatus.flashStatus = FLASH_STATUS_IDLE;

//				CCTRACE("FLASH_CMD_PREPARE_SECTOR <<\n\n");
}

/*
�ڵ�ǰsector�ռ伴������ʱ������ָ��׼����һ��sector������ǰsector��ʣ�����һ��chunkʱ����׼����һ��sector��
*/
void prepareNextFlashSector()
{
	if (pFlashStorageIndicator->nextSectorIsPrepared)
		return;

	pFlashStorageIndicator->nextSectorIsPrepared = true;

	// ��ǰsector�ռ伴�����㣬׼����һ��sector
	int idx = pFlashStorageIndicator->index + 1; //�����sector�Ĵ洢��ż�1.
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
�л�����һ��sector
����sector��ʼts������ts��Ϣ�����ں�������
*/
void switchToNextSector(bool updateTimestamp)
{
	ReturnType rt;

//	if (updateTimestamp)
	if (pFlashStorageIndicator->index > 0 && updateTimestamp)
	{
		// ����timestamp
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
		                  (BYTE*) &fsh, sizeof(FLASH_SECTOR_HEAD)); //д��sectorͷ��Ϣ
		//Atus: Why did not check rt with error code, Flash_AddressInvalid...
		if (rt!=Flash_Success) //[BG025] add dumy check.
		{ }

		//
//		waitFlashOperaDone();
	}

	// �л�sector
	pFlashStorageIndicator->index++; //�洢��������
	pFlashStorageIndicator->sector = getNextIndexDataSector(pFlashStorageIndicator->sector);

	pFlashStorageIndicator->offset = sizeof(FLASH_SECTOR_HEAD);//�洢���ݵ���ʵλ�á�
#ifdef FLASH_STORAGE_SCHEMA_V2
	pFlashStorageIndicator->capacity = ChunksPerSector; //ÿ��һ���µ�sector����ô������8�����õ�chunk�顣
#else
	pFlashStorageIndicator->capacity = pFlashInfo->sectorSize - pFlashStorageIndicator->offset;
#endif

	//�µ�sector��Ѵ洢ʱ�䶼���ó�0.
	pFlashStorageIndicator->startTimestamp = 0;
	pFlashStorageIndicator->endTimestamp = 0;

	pFlashStorageIndicator->nextSectorIsPrepared = false;
}


//BYTE flashWritingBuffer[5];

// ���� pFlashStorageIndicator ִ��flashд�붯����
// ������ pFlashStorageIndicator ��Ӧ״̬
void writeToFlash(BYTE* buffer, UINT32 size)
{
	ReturnType rt;
	long addr = 0;

	addr = pFlashStorageIndicator->sector * pFlashInfo->sectorSize
	       + pFlashStorageIndicator->offset;//�ҵ�д�����ݵ�λ�á�

	rt = FlashProgram(addr, buffer, size);
	//Atus: Why did not check rt with error code, Flash_AddressInvalid...
	if (rt!=Flash_Success) //[BG025] add dumy check.
	{ }

	pFlashStorageIndicator->offset += size;
//#ifdef FLASH_STORAGE_SCHEMA_V2
//	pFlashStorageIndicator->capacity --;
//#else
	pFlashStorageIndicator->capacity -= size; //�������ʲô��˼����
//#endif
}


// �˺������ɼ���������д��flash
// ���������������ݲɼ�Ϊ˫������������
// �������Ϊ������ָ�룬�������а�����ָ��洢����Ϣ
void onCmdSaveDataToFlash(FLASH_COMMAND* fcmd)
{
	MESSAGE msg;

	CCASSERT(fcmd);

	ENUM_FLASH_COMMANDS cmd = (ENUM_FLASH_COMMANDS) fcmd->cmd;


	/* ��������
	 ÿ�δ洢�������¶�������
	extra = ��ͷ��С + ����ͷ��С * n
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
	// ��ǰsector�ռ䲻�㣬�л�����һ��sector
#ifdef FLASH_STORAGE_SCHEMA_V2

	if (pFlashStorageIndicator->capacity < 1)
#else
	if (pFlashStorageIndicator->capacity < dataSize)
#endif
	{
		switchToNextSector(true);

		// ��Ϊ�����л������ģ���˸��� startTimestamp
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;
	}
	else if (pFlashStorageIndicator->startTimestamp <= 0)
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;

	// ÿ��д�����ݣ������� endTimestamp
	pFlashStorageIndicator->endTimestamp = pbh->timestamp;

	CCTRACE("\nFLASH_CMD_WRITE, sector: %d, offset: %d, size: %d, sts: %lu, ets: %lu >>\n",
	        pFlashStorageIndicator->sector, pFlashStorageIndicator->offset, dataSize,
	        pFlashStorageIndicator->startTimestamp,
	        pFlashStorageIndicator->endTimestamp);
//	systemStatus.flashStatus = FLASH_STATUS_WRITING;

	//
	WakeFlashUp();

	// =============================================================
	// д��ͷ

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
	// д������
	INDEX_DATA_DEF* pDef = NULL;
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];
		pInd = &(pbh->bufferIndicator[i]);

		// =========================================================
		// д��ָ��ͷ
		dataSize = pInd->bufferOffset;;
//		CCTRACE("    totalData: %d, flashOffset: %d\n", dataSize, pFlashStorageIndicator->offset);

		INDEX_DATA_HEAD cdh = { pInd->type, pDef->sampleInterval,  pInd->samples };

		writeToFlash((BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));


		// =========================================================
		// д��ָ�����ݣ���д�������ݵ�ָ��
		if (pInd->samples > 0)
		{
			BYTE* buffer = (BYTE*) pbh;
			buffer += pInd->bufferPos;
			writeToFlash(buffer, pInd->bufferOffset);
		}
	}

	// ��������д��֪ͨ
//	WaitFlashOpeartionDone(cmd, true);
//	osMessagePut(hMsgInterrupt, (long)MESSAGE_FLASH_OPERATION_DONE + ((long)cmd << 16), 0);

	msg.params.type = MESSAGE_FLASH_OPERATION_DONE;
	msg.params.param = cmd;
	xQueueSend(hEvtQueueDevice, &msg.id, 0);
	//
	systemStatus.flashStatus = FLASH_STATUS_IDLE;

//	CCTRACE("FLASH_CMD_WRITE <<\n
}


// �˺������ɼ���������д��flash����data_gather.c����
// ���������������ݲɼ�Ϊ��������������
// ָ��洢�����pbh��
// buffer��������ֻ��ָ�����ݣ�
FLASH_SECTOR_HEAD flashSectorHead;
void saveIndexDataToFlash(DATA_GATHER_BUFFER_HEAD* pbh, BYTE* buffer)
{
	// =============================================================
	// flash�洢����v2����flash��Ϊ8��508�ֽڵ�chunk
	// ÿ��д�붼ռ��һ��chunk����˲���Ҫ���flashʣ����ÿռ䣬
	// ֻ��Ҫ��鵱ǰsector�Ƿ��п���chunk����

	// =============================================================
	// ��ǰsector�ռ䲻�㣬�л�����һ��sector
#ifdef FLASH_STORAGE_SCHEMA_V2
	if (pFlashStorageIndicator->capacity < 1)
#else
	if (pFlashStorageIndicator->capacity < dataSize)
#endif
	{
		switchToNextSector(true);

		// ��Ϊ�����л������ģ���˸��� startTimestamp
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;
	}
	else if (pFlashStorageIndicator->startTimestamp <= 0)
		pFlashStorageIndicator->startTimestamp = pbh->timestamp;


	// ÿ��д�����ݣ������� endTimestamp
	pFlashStorageIndicator->endTimestamp = pbh->timestamp;

	CCTRACE("\nFLASH_CMD_WRITE, sector: %d, offset: %d, size: %d, sts: %lu, ets: %lu >>\n",
	        pFlashStorageIndicator->sector, pFlashStorageIndicator->offset, dataSize,
	        pFlashStorageIndicator->startTimestamp,
	        pFlashStorageIndicator->endTimestamp);
//	systemStatus.flashStatus = FLASH_STATUS_WRITING;

	//
	WakeFlashUp();

	// =============================================================
	// д��ͷ
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

	if (pFlashStorageIndicator->capacity >= ChunksPerSector) //��ʾʲô������
	{
		// ��һ�� chunk��д�� start timestamp?
//		FLASH_SECTOR_HEAD fsh;
		memset(&flashSectorHead, 0xFF, sizeof(FLASH_SECTOR_HEAD));
		flashSectorHead.startTimestamp = pFlashStorageIndicator->startTimestamp;

		FlashProgram(addr, (BYTE*) &flashSectorHead, sizeof(FLASH_SECTOR_HEAD));//д��sectorͷ��Ϣ
	}

	// skip sector head
	addr += sizeof(FLASH_SECTOR_HEAD);

	// ����ָ����chunk
	addr += INDEX_DATA_CHUNK_SIZE * (ChunksPerSector - pFlashStorageIndicator->capacity);

	FlashProgram(addr, (BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD)); //дChunkͷ��Ϣ
	addr += sizeof(FLASH_DATA_CHUNK_HEAD);

	// =============================================================
	// д������
	// v2��������д������ָ��ͷ��Ȼ�������������
	INDEX_DATA_DEF* pDef = NULL;
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	short dataOffsetBase = sizeof(FLASH_DATA_CHUNK_HEAD) + sizeof(INDEX_DATA_HEAD) * GATHERABLE_INDEX_DATA_COUNT;
//chunkͷ��С������ָ��ͷ��С��������ָ�����	��Ȼ��Ż����������ݡ�

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];
		pInd = &(pbh->bufferIndicator[i]);

		// =========================================================
		// д��ָ��ͷ
		INDEX_DATA_HEAD cdh = { pInd->type, pDef->sampleInterval,  pInd->samples, pInd->bufferPos + dataOffsetBase};

		FlashProgram(addr, (BYTE*) &cdh, sizeof(INDEX_DATA_HEAD)); //����д�����ָ���ͷ��Ϣ��
		addr += sizeof(INDEX_DATA_HEAD);//����һ��ָ��һ��ָ���д�롣
	}

	// д������
	// buffer��������ֻ��ָ�����ݣ�����Ч��СΪ buffersize - FLASH_DATA_CHUNK_HEAD size - all INDEX_DATA_HEAD size
	FlashProgram(addr, buffer, INDEX_DATA_CHUNK_SIZE - sizeof(FLASH_DATA_CHUNK_HEAD) - GATHERABLE_INDEX_DATA_COUNT * sizeof(INDEX_DATA_HEAD));

	//
	pFlashStorageIndicator->capacity--; //ÿ��sectorֻ����8��chunk��ÿдһ���ͼ���һ����

	// ����д�꣬�����¼�֪ͨ
//	osMessagePut(hMsgInterrupt, (long)MESSAGE_FLASH_OPERATION_DONE + ((long)FLASH_CMD_WRITE << 16), 0);
	MESSAGE msg;
	msg.params.type = MESSAGE_FLASH_OPERATION_DONE;
	msg.params.param = FLASH_CMD_WRITE;
	xQueueSend(hEvtQueueDevice, &msg.id, 0);
#else

	writeToFlash((BYTE*) &dch, sizeof(FLASH_DATA_CHUNK_HEAD));

	// v1�����ǣ�ָ��ͷa+����a+ָ��ͷb+����b...
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pInd = &(pbh->bufferIndicator[i]);

		// =========================================================
		// д��ָ��ͷ
		dataSize = pInd->bufferOffset;;
//		CCTRACE("    totalData: %d, flashOffset: %d\n", dataSize, pFlashStorageIndicator->offset);

		INDEX_DATA_HEAD cdh = { pInd->type, INDEX_DATA_SAMPLE_INTERVAL[pInd->type],  pInd->samples };

		writeToFlash((BYTE*) &cdh, sizeof(INDEX_DATA_HEAD));


		// =========================================================
		// д��ָ�����ݣ���д�������ݵ�ָ��
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
// ɨ��flash���ݴ洢���
// ���δʹ���κ�һ��sector�������һ��sector�ռ䲻�㣬��׼����һ��sector
void onPostScanFlashStorage()
{
	CCTRACE("flash init done.\n");


	// ============================================================================
	// �������ݲɼ�
	StartDataGathering();//׼����������Ҫ�ǰ��Ѿ���ʼ�ɼ���λ


	// ============================================================================
	// ����Ҫ���� FLASH_CMD_PREPARE_SECTOR ����
	if (pFlashStorageIndicator->index == 0)
	{
		// ˵��δʹ���κ�һ��sector����ϵͳ����������
		pFlashStorageIndicator->sector = 0;
		pFlashStorageIndicator->capacity = 0; // ����sector��capacityΪ0��������´δ洢ʱ�����л�����sector
//�����ǲ���Ҫ������һ��sector���������ô�������������
		prepareNextFlashSector();
	}

#ifdef FLASH_STORAGE_SCHEMA_V2
	else if (pFlashStorageIndicator->capacity < 2)	// ��ʣһ��chunk
#else
	else if (pFlashStorageIndicator->capacity < pFlashInfo->sectorSize / FLASH_MIN_CAPACITY_FACTOR)
#endif
	{
		CCTRACE("\n** sector %d out of space, switch to next sector\n",
		        pFlashStorageIndicator->sector);

		// ��ǰsector�ռ伴�����㣬Ԥ����һ��sector
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
// sectorԤ�����
void onPostPreparedSector()
{
	CCTRACE("preparing sector done.\n");

	// set flash to power off
	ClearFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);
	PutFlashToSleep();
}

// ============================================================================
// ����д��flash���
// ����flash�洢����
// ���sector�ռ伴�����㣬��׼����һ��sector
void onPostWroteDataToFlash()
{
	CCTRACE("flash-writing done.\n");

	//
#ifdef FLASH_STORAGE_SCHEMA_V2

	if (pFlashStorageIndicator->capacity < 2)	// ��ʣһ��chunk
#else
	if (pFlashStorageIndicator->capacity < pFlashInfo->sectorSize / FLASH_MIN_CAPACITY_FACTOR)
#endif
	{
		CCTRACE("\n** sector %d running out of space, prepare next sector\n",
		        pFlashStorageIndicator->sector);

		// ��ǰsector�ռ伴�����㣬׼����һ��sector
		prepareNextFlashSector();
	}
	else
	{
		// set flash to power off
		ClearFlashBusyBit(FLASH_BUSY_BIT_DATA_GATHER);
		PutFlashToSleep();
	}
}


// ��ȡ��һ�����������ϴ��������ⲿ����flash
// ��ȡλ���� dataUploadPointer ָ��
// ��ȡ����� dataUploadPointer
//
// ����ֵ��
// true ��ʾ�������ݿ���
// false ��ʾ����ȫ������
bool readNextUploadChunk(BYTE* buffer, int size)
{
	ReturnType rt;
	int addr = dataUploadPointer.sector * pFlashInfo->sectorSize + dataUploadPointer.offset; //�ҵ���ʵ���ݵ���ʼλ��

	if (dataUploadPointer.capacity <= 0) //���capacity��д��ʱ��ʾ��sector���ж��ٿռ��ܹ����������ݣ��ڶ���ʱ��ʾ���ж�������û�ж���
		return false;//���⣺�����capacity��ʾ��ǰsector���������������ж��ٸ�chunk�飿����

	if (dataUploadPointer.offset + size <= dataUploadPointer.capacity)
	{
		// ��ǰsector��������������buffer
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

		// ��ǰsector�����ݲ���������buffer
		int sectorRemain = dataUploadPointer.capacity - dataUploadPointer.offset;

		if (sectorRemain > 0)
		{
			// ��ǰsector��������
			rt = FlashRead(addr, buffer + bufferOffset, sectorRemain);
			//Atus: Why did not check rt with error code, Flash_AddressInvalid...
			if (rt!=Flash_Success) //[BG025] add dumy check.
			{ }

			bufferOffset += sectorRemain;
			size -= sectorRemain;
			bufferHasRemain = true;
		}

		// �л�����һ��sector
		dataUploadPointer.sector = getNextIndexDataSector(dataUploadPointer.sector);
		dataUploadPointer.offset = sizeof(FLASH_SECTOR_HEAD); // �л�����sector�󣬳�ʼ��ƫ�Ʊ�ȻΪ sizeof(FLASH_SECTOR_HEAD)
//		dataUploadPointer.capacity = getSectorDataSize(dataUploadPointer.sector);

		if (dataUploadPointer.capacity > 0)
		{
			// ��sector������
			return readNextUploadChunk(buffer + bufferOffset, size);//������ֵݹ����
		}
		else
			return bufferHasRemain;
	}

//	return false;
}

#ifdef DEBUG
// �R����Н��sector
void onCmdPrepareSectorForTesting(FLASH_COMMAND* fcmd)
{
#if 0
	// ��z�v?�a��sector���҉���ދ?sector head
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
		// ދ?����s�l��
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
			// sectorԤ�����
			onPostPreparedSector();

			break;
		}

		case FLASH_CMD_WRITE:
		{
			// ����д��
			onPostWroteDataToFlash();

			break;
		}

#ifdef DEBUG

		case FLASH_CMD_PREPARE_SECTOR_FOR_TESTING:
		{
			// ����sector׼�����
			break;
		}

#endif
	}
}



//==============================================================================

BYTE SearchSectorIndex(uint32_t indexSearch, uint16_t* sectorNo, bool blStart)
{
	/****
	�㷨������
	1��������Χ�� sector number 512  �� sector number 2047�������ȡÿ��sector��ͷ��Ϣ
	2���鿴sector tag�Ƿ���Ч��
	   2.1 �����Ч���ٶ�sector index���ڽ��бȽ�
			 2.1.1 ���sector index��index һֱ���˳�����ѭ����result=05������
			 2.1.2 �����һ�£�������ȡ��һ��sector��ͷ��Ϣ
	   2.2 �����Ч���˳�ѭ����
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
				//˵���ҵ���ָ��index
				if((fsh.endTimestamp !=  DEFAULT_TIMESTAMP))
				{
					if(blStart)
					{
						result = 0x01;
					}
					else
						result = 0x03;

					*sectorNo = i;//��ȡ��ǰ��sector number;
				}
				else
				{
					if(blStart)
					{
						result = 0x05;//ָ����sector��ô�вɼ���
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

			//һ��������sector��tag����ȷ��˵����������flash��û�д洢�ɼ����ݣ��ʲ�ɨ����
			if(blStart)
				result = 0x02;
			else
				result = 0x04;

			return result;
			//��ʱ��һ��sector��ͷ��Ϣ��index=-1��ʱ�������FF��tagҲ����FFFF,����������tag��sector������ֹͣ��Ѱ�������ͻ���һ�����⣬��һ�������п��ܰ�����tag��sector
#endif
			continue;
		}
	}


	if(i == INDEX_DATA_END_SECTOR + 1)
	{
		result = 0x07;//��ǰҪ�ҵ�sector index�����ڡ�
		return result;
	}

	return result;
}


#if 0
//������Ǽ��ֲ���sector index��sector number�Եķ�����
void CombinIndexNumberOne(SECTOR_NODE* sectorNode, int startSectorIndex, int totalNum)
{
	uint16_t i = 0;
	uint16_t j = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = 0; i <= totalNum; i++)
	{
		//����ѭ������Ҫ�ҵ�����sector

		for(j = INDEX_DATA_START_SECTOR; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//ֻҪ���øú�����ζ�ţ���ʼ�ͽ���index���Ѿ��ҵ��ˣ�����Ͳ��ж�tag��Ч����
			if(fsh.index == startSectorIndex)
			{
				//�ҵ�index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//������һ��index,���Ǵ�ͷ�ң������������flashsectorβ����������Էֶ��ң�Ч�ʸߣ�
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}
	}
}

//���д���ܺ�ʱ�����һ����������϶�Ĳ�����index�Ļ���û��������һ��sector index��ֻ��ɨ��������flash��sector�������������
void CombinIndexNumberTwo(SECTOR_NODE* sectorNode, int startSectorIndex, int totalNum, int* counter)
{
	uint16_t i = 0;
	uint16_t j = 0;
	int nonExistIndex = 0;
	FLASH_SECTOR_HEAD fsh;
	long addr = 0;

	for(i = 0; i <= totalNum; i++)
	{
		//����ѭ������Ҫ�ҵ�����sector

		for(j = INDEX_DATA_START_SECTOR; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//ֻҪ���øú�����ζ�ţ���ʼ�ͽ���index���Ѿ��ҵ��ˣ�����Ͳ��ж�tag��Ч����
			if(fsh.index == startSectorIndex)
			{
				//�ҵ�index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//������һ��index,���Ǵ�ͷ�ң������������flashsectorβ����������Էֶ��ң�Ч�ʸߣ�
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}

		if(j == INDEX_DATA_END_SECTOR + 1)
		{
			//���������sector������һ��󣬻�û�����index�Ļ�����ȥ����һ��index���������Ῠ��������һֱ�ң������Ĳ��ֲ������С�
			startSectorIndex++;//�����Ľ����sectorNodeָ��ָ���������ֵ�������ҵĵ�����Щsector����������ʵ��sector��š�
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
		//����ѭ������Ҫ�ҵ�����sector

		for(j = INDEX_DATA_START_SECTOR; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//ֻҪ���øú�����ζ�ţ���ʼ�ͽ���index���Ѿ��ҵ��ˣ�����Ͳ��ж�tag��Ч����
			if(fsh.index == startSectorIndex)
			{
				//�ҵ�index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//������һ��index,���Ǵ�ͷ�ң������������flashsectorβ����������Էֶ��ң�Ч�ʸߣ�
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}

		if(j == INDEX_DATA_END_SECTOR + 1)
		{
			//����ĳ��sector index������flash�ж�û�С�
			* counter = startSectorIndex;
			break;//���˳�����ɨ����̣���ǰ�����е�index��������������������ľͲ����ˡ�
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
		//����ѭ������Ҫ�ҵ�����sector

		for(j = startSectorNumber; j <= INDEX_DATA_END_SECTOR; j++ )
		{
			//���Һ�벿��
			addr = j * pFlashInfo->sectorSize;
			FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

//ֻҪ���øú�����ζ�ţ���ʼ�ͽ���index���Ѿ��ҵ��ˣ�����Ͳ��ж�tag��Ч����
			if(fsh.index == startSectorIndex)
			{
				//�ҵ�index
				sectorNode->sectorIndex = startSectorIndex;
				sectorNode->sectorNumber = j;

				//������һ��index,���Ǵ�ͷ�ң������������flashsectorβ����������Էֶ��ң�Ч�ʸߣ�
				sectorNode++;
				startSectorIndex++;
				break;
			}
		}


		if(j == INDEX_DATA_END_SECTOR + 1)
		{
			//����������ں�벿�ֶ�û���ҵ���index����Ҫ����ʼλ���ҡ�
			for(k = INDEX_DATA_START_SECTOR; k < startSectorNumber ;k++)
			{
				//ɨ��ǰ�벿��
				addr = k * pFlashInfo->sectorSize;
				FlashRead(addr, (BYTE*)&fsh, sizeof(FLASH_SECTOR_HEAD));

				if(fsh.index == startSectorIndex)
				{
					//�ҵ�index
					sectorNode->sectorIndex = startSectorIndex;
					sectorNode->sectorNumber = j;

					//������һ��index,���Ǵ�ͷ�ң������������flashsectorβ����������Էֶ��ң�Ч�ʸߣ�
					sectorNode++;
					startSectorIndex++;
					break;
				}
			}
		}


		if((j == INDEX_DATA_END_SECTOR + 1)  && (k == startSectorNumber-1))
		{
			//����ĳ��sector index������flash�ж�û�С�
			* counter = startSectorIndex;
			break;//���˳�����ɨ����̣���ǰ�����е�index��������������������ľͲ����ˡ�
		}
	}
}


//�ú������ص�ǰflash��sector index�����ֵ��
int SearchMaxSectorIndex(uint16_t *secnum)
{
	/***
	ɨ������flash�ķ�Χ��512 sector ----2047 sector,���ǵ������зǷ���tagʱ���Ͳ�������ɨ���ˣ��Ͱѵ�ǰ����
	index ����flash������sector�����Ĵ洢����
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
			case FLASH_CMD_DATA_GATHER_TICK: //ÿ���ж��ᷢ�͸���Ϣ
			{
				// ����������Ϣ
				MESSAGE msg;
				msg.params.type = MESSAGE_TASK_HEARTBEAT;
				msg.params.param = 0x04;

//				osMessagePut(hMsgInterrupt, msg.id, 0);
				xQueueSend(hEvtQueueDevice, &msg.id, 0);
				//��ʼ�ɼ�����
				doDataGathering();
				break;
			}

			case FLASH_CMD_PREPARE_CHIP: //û����Ϣ���͹�����
			{
				onCmdInitChip(&cmdFlash);

				//
				break;
			}

			case FLASH_CMD_PREPARE_DATA_STORAGE: //����������ʱ����һ�θ���Ϣ
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

			case FLASH_CMD_PREPARE_SECTOR://������д��sector��д��Tag��־���µ�������
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
//				// ��16λ��1=ȫ��������0=������������ʼsector
//				// ��16λ��1=��ɺ������ɼ���0=��ɺ��޶���
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

		// �ͷ��¼��洢
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
