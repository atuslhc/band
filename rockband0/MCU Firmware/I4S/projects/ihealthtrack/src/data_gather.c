#include <stdlib.h>
#include <string.h>

#include "debug.h"

#include "AFE44x0.h"

#include "main.h"
#include "time.h"
#include "common_vars.h"
#include "globaldata.h"
#include "flash_task.h"
#include "data_gather.h"
#include "sys_sharing_source.h"

#define ENABLE_DATA_GATHER_DOUBLE_BUFFER	0	//[BG025] change define with define value, and #ifdef change to #if

#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
#define TOTAL_INDEX_DATA_BUFFER_SIZE	1500//(98 * 4 + 64 * 2)	//(1292)
#define INDEX_DATA_BUFFER_SIZE	(TOTAL_INDEX_DATA_BUFFER_SIZE / 2)
BYTE indexDataBuffer[2][INDEX_DATA_BUFFER_SIZE] = {0};
BYTE indexDataBufferIndicator = 0;
#else
// single buffer. One buffer store the historical index data.
DATA_GATHER_BUFFER_HEAD gatherBufferHead;
BYTE indexDataBuffer[INDEX_DATA_BUFFER_SIZE] = {0};//One buffer with a chunk size 508 bytes.
#endif

BYTE ChunksPerSector = 8;


int getGreatestCommonDivisor(int m, int n); //calculate GCD with Euclidean algorithm.

int getGreatestCommonDivisor2(int m, int n); //calculate GCD with Euclidean algorithm - recursive


/*
初始化指标缓冲区，主要工作是计算各指标独立缓冲区大小，并写入缓冲区头
仅需调用一次？？可能会在运行时允许/禁止某些指标

缓冲区最大的大小为 INDEX_DATA_BUFFER_SIZE ，这是在flash中一个数据块的大小
一个数据块包括 一个块头INDEX_DATA_HEAD，若干个（GATHERABLE_INDEX_DATA_COUNT）（实际上是8个） 指标头 INDEX_DATA_HEAD
剩下的空间才是用于数据的存储

每项指标数据分配的空间（内存/flash）通过其存储比重 INDEX_DATA_GRAVITY 计算而来
*/
void initIndexDataBuffer()
{
	INDEX_DATA_DEF* pDef = NULL;

	ChunksPerSector = (pFlashInfo->sectorSize - sizeof(FLASH_SECTOR_HEAD)) / INDEX_DATA_CHUNK_SIZE;

	// 计算需要采集的指标的比重
	// 然后计算总的比重
	// 再确定每个指标占缓冲区的大小

	// 计算采样周期的最大公约数
	int gcdInterval = 0;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];

		if (gcdInterval == 0)
			gcdInterval = pDef->sampleInterval;
		else
			gcdInterval = getGreatestCommonDivisor(gcdInterval, pDef->sampleInterval);
	}

	// 计算采样周期的最小公倍数
	int lcmInterval = 0;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];

		if (lcmInterval == 0)
			lcmInterval = pDef->sampleInterval;
		else
		{
			if ((lcmInterval % pDef->sampleInterval) != 0)
				lcmInterval *= pDef->sampleInterval / gcdInterval;
		}
	}

	// 然后用 采样周期的最小公倍数 / 采样周期 * 样本大小 取得每个指标的总样本大小（在 采样周期的最小公倍数 时间段内）
	// 然后再计算总样本大小的最大公约数，
	// 从而计算出每个样本的比重，以及总的比重
//	int sampleSize[] = new int[TOTAL_INDEXES];

	int s = 0, gcdSample = 0;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];

		pDef->weight = s = lcmInterval / pDef->sampleInterval * pDef->sampleSize;

		if (gcdSample == 0)
			gcdSample = s;
		else
			gcdSample = getGreatestCommonDivisor(gcdSample, s);
	}

	if (gcdSample > 1)
	{
		for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
		{
			pDef = GATHERABLE_INDEX_DATA[i];

			pDef->weight =  pDef->weight / gcdSample;
		}
	}

	int totalWeight = 0; //sum of all gather data weight.

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];

		totalWeight += pDef->weight;
	}


#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	int w = (INDEX_DATA_BUFFER_SIZE - sizeof(DATA_GATHER_BUFFER_HEAD)) / totalWeight;
//	float w = (float)(INDEX_DATA_BUFFER_SIZE - sizeof(DATA_GATHER_BUFFER_HEAD)) / (float)tw;

	DATA_GATHER_BUFFER_HEAD bh = {0};
	int pos = sizeof(DATA_GATHER_BUFFER_HEAD);

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		INDEX_DATA_BUFFER_INDICATOR* pInd = &(bh.bufferIndicator[i]);
		pInd->type = GATHERABLE_INDEX_DATA[i]->type;
//		pInd->bufferSize = (INT16)(w * (float) INDEX_DATA_GRAVITY[i]) / INDEX_DATA_SAMPLE_SIZE[i] * INDEX_DATA_SAMPLE_SIZE[i];
		pInd->bufferSize = w * GATHERABLE_INDEX_DATA[i]->weight;
		pInd->bufferPos = pos;
		pInd->bufferOffset = 0;

		pos += pInd->bufferSize;
	}

	//
	memcpy(indexDataBuffer[0], &bh, sizeof(DATA_GATHER_BUFFER_HEAD));
	memcpy(indexDataBuffer[1], &bh, sizeof(DATA_GATHER_BUFFER_HEAD));
#else
	// 单缓存机制

	// 缓冲区中全部为数据，其有效大小为 INDEX_DATA_BUFFER_SIZE（508bytes） - GATHERABLE_INDEX_DATA_COUNT（8） * sizeof(INDEX_DATA_HEAD)（7bytes）
	int w = (INDEX_DATA_BUFFER_SIZE - sizeof(FLASH_DATA_CHUNK_HEAD) - GATHERABLE_INDEX_DATA_COUNT * sizeof(INDEX_DATA_HEAD)) / totalWeight;
//这个w是什么意思？？？？


	int pos = 0;//sizeof(DATA_GATHER_BUFFER_HEAD);

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		INDEX_DATA_BUFFER_INDICATOR* pInd = &(gatherBufferHead.bufferIndicator[i]);
		pInd->type = GATHERABLE_INDEX_DATA[i]->type;
//		pInd->bufferSize = (INT16)(w * (float) INDEX_DATA_GRAVITY[i]) / INDEX_DATA_SAMPLE_SIZE[i] * INDEX_DATA_SAMPLE_SIZE[i];
		pInd->bufferSize = w * GATHERABLE_INDEX_DATA[i]->weight;
		pInd->bufferPos = pos;
		pInd->bufferOffset = 0;

		pos += pInd->bufferSize;//这里为下个指标的存储的起始位置。
	}

#endif
}

// 翻转数据缓冲区
// 返回缓冲区指针
//#pragma inline=forced
BYTE* flipIndexDataBuffer()
{
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	indexDataBufferIndicator ++;
	indexDataBufferIndicator %= 2;

	return indexDataBuffer[indexDataBufferIndicator];
#else
	return indexDataBuffer;
#endif
}

// 翻转并初始化指标缓冲区
// 初始化包括：写入时间戳；重置各个指标缓冲区的写入指针；重置样本数量
// 返回缓冲区指针
BYTE* flipAndInitIndexDataBuffer()
{
	BYTE* buffer = flipIndexDataBuffer();

#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
	pbh->timestamp = getCurrentTimestamp();

	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pInd = &(pbh->bufferIndicator[i]);
		pInd->bufferOffset = 0;
		pInd->samples = 0;
	}

#else
	memset(buffer, 0, INDEX_DATA_BUFFER_SIZE);

	gatherBufferHead.timestamp = getGMTTimestamp();//time(NULL);
	gatherBufferHead.timezoneOffset = systemSetting.timezoneOffset;

	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pInd = &(gatherBufferHead.bufferIndicator[i]);//在一个新的sector里各个指标数据缓存区的数据需要清零。
		pInd->bufferOffset = 0;
		pInd->samples = 0;
	}

#endif

	return buffer;
}

//#pragma inline=forced
BYTE* getActivatedIndexDataBuffer()
{
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	return indexDataBuffer[indexDataBufferIndicator];
#else
	return indexDataBuffer;
#endif
}

// 此函数检查缓冲区是否已满，若已满，则将数据写入flash并准备好buffer
// 其内部调用 checkIndexDataBuffer 用于检查缓冲区是否已满
// 准备好buffer分两种情况
//		双缓冲：发起写flash操作，并切换到下一个buffer
// 		单缓冲：直接写入flash，并切换到下一个buffer
bool checkAndPrepareBuffer(INDEX_DATA_TYPE type)
{
	if (checkIndexDataBuffer(type))
	{
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER

		// initiate a saving request here
		BYTE* buff = getActivatedIndexDataBuffer();

//		FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//		fcmd->cmd = FLASH_CMD_WRITE;
//		fcmd->data.p = buff;
//
//		osMailPut(hFlashCommandQueue, fcmd);


		FLASH_COMMAND fcmd;
		fcmd.cmd = FLASH_CMD_WRITE;
		fcmd.data.p = buff;

		xQueueSend(hEvtQueueFlash, &fcmd, 0);

#else

		saveIndexDataToFlash(&gatherBufferHead, indexDataBuffer);
		
#endif

		//
		flipAndInitIndexDataBuffer();

		return true;
	}

	return false;
}

// 此函数检查缓冲区是否已满，若已满，则将数据写入flash并准备好buffer
// 与checkAndPrepareBuffer不同之处在于此函数检查所有的index data是否都已满
// 只有都已满才认为缓冲区已满
bool checkAndPrepareBuffer2()
{
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
#else
	DATA_GATHER_BUFFER_HEAD* pbh = &gatherBufferHead;
#endif

	// 指标的存储参数
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pInd = &(pbh->bufferIndicator[i]);

		if (pInd->bufferOffset < pInd->bufferSize) // 缓冲区未满
			return false;
	}


	// 数据写入flash
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER

	// initiate a saving request here
	BYTE* buff = getActivatedIndexDataBuffer();

	FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
	fcmd->cmd = FLASH_CMD_WRITE;
	fcmd->data.p = buff;

	osMailPut(hFlashCommandQueue, fcmd);

#else
#if !BATTERY_LIFE_OPTIMIZATION

	saveIndexDataToFlash(&gatherBufferHead, indexDataBuffer);
#else
	//如果进行功耗测量，就不往flash中写入采集数据。

#endif

#endif

	//完成一次存储后翻转并初始化指标缓冲区
	flipAndInitIndexDataBuffer();

	return true;
}

// 检查缓冲区是否已满
bool checkIndexDataBuffer(INDEX_DATA_TYPE type)
{
	BYTE* buffer = getActivatedIndexDataBuffer();//indexDataBuffer[indexDataBufferIndicator];
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
#else
	DATA_GATHER_BUFFER_HEAD* pbh = &gatherBufferHead;
#endif

	// 查找到指标的存储参数
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++) //这里是检查所有的类型是否都存储满了。
	{
		pInd = &(pbh->bufferIndicator[i]);

		if (pInd->type == type)//注意要匹配类型。
		{
			if (pInd->bufferOffset >= pInd->bufferSize) //当前指标的偏移量大于buffersize，那么就是存储满了。
				return true;
			else
				return false;
		}
	}

	return false;
}

//// 检查缓冲区是否已满
//bool checkIndexDataBufferIsFull()
//{
//	BYTE* buffer = indexDataBuffer[indexDataBufferIndicator];
//	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
//
//	// 查找到指标的存储参数
//	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;
//	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
//	{
//		pInd = &(pbh->bufferIndicator[i]);
////		if (pInd->type == type)
////		{
//			if (pInd->bufferOffset >= pInd->bufferSize)
//				return true;
////		}
//	}
//
//	return false;
//}


// 将指标数据写入对应的指标缓冲区
//void putIndexData(INDEX_DATA_TYPE type, BYTE* data)
void putIndexData(BYTE idx, BYTE size, BYTE* data)
{
	CCASSERT(data != NULL);

	//
	BYTE* buffer = getActivatedIndexDataBuffer();//indexDataBuffer[indexDataBufferIndicator];
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
#else
	DATA_GATHER_BUFFER_HEAD* pbh = &gatherBufferHead;
#endif

	// 查找到指标的存储参数
//	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;
//	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
//	{
	INDEX_DATA_BUFFER_INDICATOR* pInd = &(pbh->bufferIndicator[idx]);
//		if (pInd->type == type)
//		{
	CCASSERT(pInd->bufferSize >= pInd->bufferOffset + INDEX_DATA_SAMPLE_SIZE[type]);

	// 存储数据
	// 加此条件是为了解决“有时存放的数据超出缓冲区范围”的bug
	// 此bug的产生原因是时间同步引起的
	if (pInd->bufferOffset < pInd->bufferSize)
	{
		memcpy(buffer + (pInd->bufferPos + pInd->bufferOffset), data, size);
		pInd->bufferOffset += size;
		pInd->samples++;            //每存储一次某种指标数据，那么缓存区的偏移和该指标样本数均增加
	}

//#ifdef DEBUG
//			if (pInd->bufferOffset > pInd->bufferSize)
//				while(1); // 调试用，用于检查有时存放的数据超出缓冲区范围
//#endif
//			break;
//		}
//	}
}

// 将所有指标数据置为0
// 实际起作用的是累加型数据，实时型数据始终取的是最新值
void checkResetIndexData()
{
//	iHeartRate = {0}; // 低字节为心率，高字节为可靠度
	if (pSystemTime->tm_hour == 0
	        && pSystemTime->tm_min == 0
	        && pSystemTime->tm_sec == 0)
	{
		iCalories = 0;
		iCalories_lastSaving = 0;

		iSteps = 0;
		iSteps_lastSaving = 0;

		iDistance = 0;
		iDistance_lastSaving = 0;

		active_level = 0;
		active_level_lastSaving = 0;


		// 同时重置秒表基础值、累加值
		iCaloriesBase = 0;
		iStepsBase = 0;
		iDistanceBase = 0;

		iCaloriesAccumulated = 0;
		iStepsAccumulated = 0;
		iDistanceAccumulated = 0;


		//清除掉已经达到的目标 2015年7月1日18:27:34
		blStepAccomplish = false;
		blDistanceAccomplish = false;
		blCalorieAccomplish = false;
                
                iUVexp = 0; //[BG023-1] add.

	}
}

void StartDataGathering()
{
	if (systemStatus.blEnableDataGathering == true
	        || systemStatus.blFlashOnline == false)
//			|| systemStatus.blFlashInitialized == false)
		return;

	// 处于出厂模式（锁定）时，不启动数据采集
	if (systemSetting.SystemMode == SYSTEM_MODE_RELEASED)
		return;

	// 先设置缓冲区未初始化，以便采集进程翻转缓冲区
	systemStatus.blDataGatheringInitialized = false;

	systemStatus.blEnableDataGathering = true;
}

void StopDataGathering(bool flush)
{
	if (systemStatus.blEnableDataGathering == false
	        || systemStatus.blFlashOnline == false
	        || systemStatus.blFlashInitialized == false)
		return;

	systemStatus.blEnableDataGathering = false;

	if (flush)
	{
//		// 将已有数据写入flash
//		BYTE* buff = getActivatedIndexDataBuffer();
//
//		FLASH_COMMAND* fcmd = (FLASH_COMMAND*) osMailCAlloc(hFlashCommandQueue, 0);
//		fcmd->cmd = FLASH_CMD_WRITE;
//		fcmd->data.p = buff;
//
//		osMailPut(hFlashCommandQueue, fcmd);
		saveIndexDataToFlash(&gatherBufferHead, indexDataBuffer);
	}
}

void doDataGathering()
{
	//
	if (systemStatus.blEnableDataGathering == false)
	{
		// 即使不采集数据，也需要重置指标数据，把他们置0
		checkResetIndexData();
		return;
	}

	if (!systemStatus.blDataGatheringInitialized)
	{
		//准备了新的sector才会执行？？？
		systemStatus.blDataGatheringInitialized = true;

		// 第一次翻转数据采集缓冲区，用于写入本地当前的时间戳
		flipAndInitIndexDataBuffer();
	}


	// =============================================================
	BYTE buffer[4];

#ifdef DEBUG
	// 生成模拟数据
//	iHeartRate = (rand() % 150) + 50;
//	iCalories = rand() % 20;
//	iSteps = rand() % 50;
//	iDistance = rand() % 10;
//
//	bUltraViolet = rand() % 50;
//
//	fAmbientTemperature = ((float)(rand() % 200 + 50)) / 10;
//	fSkinTemperature = ((float)(rand() % 200 + 50)) / 10;
#endif

	// 取得当前时间,
	time_t ts = time(NULL);

	// 检查缓冲区是否已满，已满则将当前缓冲区数据写入flash
	// 然后准备缓冲区
	checkAndPrepareBuffer2();

	//
	INDEX_DATA_DEF* pDef = NULL;
//	INDEX_DATA_TYPE type = DATATYPE_UNKNOWN;
	BYTE sampleSize = 0;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];
//		type = pDef->type;
		sampleSize = pDef->sampleSize;

		// 是否到达指标采样周期
//#ifdef DEBUG
//		if (ts % (pDef->sampleInterval / 2) == 0)
//#else
		if (ts % (pDef->sampleInterval) == 0)
//#endif
		{
//			// 检查缓冲区是否已满，已满则将当前缓冲区数据写入flash
//			checkAndPrepareBuffer(type);

			if (pDef->type == DATATYPE_AMBIENT_TEMPERATURE
			        || pDef->type == DATATYPE_SKIN_TEMPERATURE)
			{
				// 温度特别处理
				buffer[0] = (BYTE) * ((float*)pDef->data);
				buffer[1] = (BYTE) ((*((float*)pDef->data) - (float) buffer[0]) * 10);
				putIndexData(i, sampleSize, buffer);
			}
			else if(pDef->type == DATATYPE_HEART_RATE)
			{
				buffer[0] = iHeartRate.component.heart;
				buffer[1] = iHeartRate.component.reliability;
				putIndexData(i, sampleSize, buffer);//将采集指标写入到对应缓存区

			}
                        else if (pDef->type == DATATYPE_UV) //[BG023] add, need reset.
                        {
                                if (*(BYTE*)(pDef->data)>= (BYTE)UVexp_Threshold) //[BG023-1] add.
                                {
                                  iUVexp ++;
                                }
				putIndexData(i, sampleSize, pDef->data); //Atus: Why can not put above if?
                                *pDef->data = 0; //reset to the minimux to get the maximum.
                        }
			else
			{
				if (pDef->accumulative)
				{
					UINT data = *((UINT*)pDef->data) - *((UINT*)pDef->lastData);
					putIndexData(i, sampleSize, (BYTE*) &data); //这里只保存了增量。

					//测试用代码，最终需要去掉
//					if (pDef->type == DATATYPE_ACTIVE_LEVEL)
//					{
//						active_level_delta = data;
//					}

					// 保存最新数据（用当前数据把原来的数据覆盖）
					memcpy(pDef->lastData, pDef->data, pDef->sampleSize);
				}
				else
				{
					putIndexData(i, sampleSize, pDef->data);
				}
			}
		}
	}


	//在0点的时候把有累加属性的数据清零
	checkResetIndexData();
}


/**
 * Calculate the GCD(Greatest Common Divisor) of m,n.
 * Euclidean algorithm(欧几里得算法-辗转相除法)
 * @param m: The first input integer.
 * @param n: The second input integer.
 * The input param m and n without order sequence.
 * @return the GCD
 */
int getGreatestCommonDivisor(int m, int n)
{
	int t = 0;

	if (m < n)
	{
		t = m;
		m = n;
		n = t;
	}

	while (n != 0)
	{
		t = m % n;
		m = n;
		n = t;
	}

	return m;
}

/**
 * Calculate the GCD(Greatest Common Divisor) of m,n.
 * Euclidean algorithm - recursive(欧几里得算法-辗转相除法)
 * @param m: The first input integer.
 * @param n: The second input integer.
 * The input param m and n without order sequence.
 * @return GCD
 */
int getGreatestCommonDivisor2(int m, int n)
{
	int t = 0;

	if (m < n)
	{
		t = m;
		m = n;
		n = t;
	}

	while ((t = m % n) != 0)
		return getGreatestCommonDivisor2(n, t);

	return 0;
}
