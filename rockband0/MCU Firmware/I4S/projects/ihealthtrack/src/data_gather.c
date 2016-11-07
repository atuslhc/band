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
��ʼ��ָ�껺��������Ҫ�����Ǽ����ָ�������������С����д�뻺����ͷ
�������һ�Σ������ܻ�������ʱ����/��ֹĳЩָ��

���������Ĵ�СΪ INDEX_DATA_BUFFER_SIZE ��������flash��һ�����ݿ�Ĵ�С
һ�����ݿ���� һ����ͷINDEX_DATA_HEAD�����ɸ���GATHERABLE_INDEX_DATA_COUNT����ʵ������8���� ָ��ͷ INDEX_DATA_HEAD
ʣ�µĿռ�����������ݵĴ洢

ÿ��ָ�����ݷ���Ŀռ䣨�ڴ�/flash��ͨ����洢���� INDEX_DATA_GRAVITY �������
*/
void initIndexDataBuffer()
{
	INDEX_DATA_DEF* pDef = NULL;

	ChunksPerSector = (pFlashInfo->sectorSize - sizeof(FLASH_SECTOR_HEAD)) / INDEX_DATA_CHUNK_SIZE;

	// ������Ҫ�ɼ���ָ��ı���
	// Ȼ������ܵı���
	// ��ȷ��ÿ��ָ��ռ�������Ĵ�С

	// ����������ڵ����Լ��
	int gcdInterval = 0;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pDef = GATHERABLE_INDEX_DATA[i];

		if (gcdInterval == 0)
			gcdInterval = pDef->sampleInterval;
		else
			gcdInterval = getGreatestCommonDivisor(gcdInterval, pDef->sampleInterval);
	}

	// ����������ڵ���С������
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

	// Ȼ���� �������ڵ���С������ / �������� * ������С ȡ��ÿ��ָ�����������С���� �������ڵ���С������ ʱ����ڣ�
	// Ȼ���ټ�����������С�����Լ����
	// �Ӷ������ÿ�������ı��أ��Լ��ܵı���
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
	// ���������

	// ��������ȫ��Ϊ���ݣ�����Ч��СΪ INDEX_DATA_BUFFER_SIZE��508bytes�� - GATHERABLE_INDEX_DATA_COUNT��8�� * sizeof(INDEX_DATA_HEAD)��7bytes��
	int w = (INDEX_DATA_BUFFER_SIZE - sizeof(FLASH_DATA_CHUNK_HEAD) - GATHERABLE_INDEX_DATA_COUNT * sizeof(INDEX_DATA_HEAD)) / totalWeight;
//���w��ʲô��˼��������


	int pos = 0;//sizeof(DATA_GATHER_BUFFER_HEAD);

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		INDEX_DATA_BUFFER_INDICATOR* pInd = &(gatherBufferHead.bufferIndicator[i]);
		pInd->type = GATHERABLE_INDEX_DATA[i]->type;
//		pInd->bufferSize = (INT16)(w * (float) INDEX_DATA_GRAVITY[i]) / INDEX_DATA_SAMPLE_SIZE[i] * INDEX_DATA_SAMPLE_SIZE[i];
		pInd->bufferSize = w * GATHERABLE_INDEX_DATA[i]->weight;
		pInd->bufferPos = pos;
		pInd->bufferOffset = 0;

		pos += pInd->bufferSize;//����Ϊ�¸�ָ��Ĵ洢����ʼλ�á�
	}

#endif
}

// ��ת���ݻ�����
// ���ػ�����ָ��
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

// ��ת����ʼ��ָ�껺����
// ��ʼ��������д��ʱ��������ø���ָ�껺������д��ָ�룻������������
// ���ػ�����ָ��
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
		pInd = &(gatherBufferHead.bufferIndicator[i]);//��һ���µ�sector�����ָ�����ݻ�������������Ҫ���㡣
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

// �˺�����黺�����Ƿ���������������������д��flash��׼����buffer
// ���ڲ����� checkIndexDataBuffer ���ڼ�黺�����Ƿ�����
// ׼����buffer���������
//		˫���壺����дflash���������л�����һ��buffer
// 		�����壺ֱ��д��flash�����л�����һ��buffer
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

// �˺�����黺�����Ƿ���������������������д��flash��׼����buffer
// ��checkAndPrepareBuffer��֮ͬ�����ڴ˺���������е�index data�Ƿ�����
// ֻ�ж���������Ϊ����������
bool checkAndPrepareBuffer2()
{
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
#else
	DATA_GATHER_BUFFER_HEAD* pbh = &gatherBufferHead;
#endif

	// ָ��Ĵ洢����
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
	{
		pInd = &(pbh->bufferIndicator[i]);

		if (pInd->bufferOffset < pInd->bufferSize) // ������δ��
			return false;
	}


	// ����д��flash
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
	//������й��Ĳ������Ͳ���flash��д��ɼ����ݡ�

#endif

#endif

	//���һ�δ洢��ת����ʼ��ָ�껺����
	flipAndInitIndexDataBuffer();

	return true;
}

// ��黺�����Ƿ�����
bool checkIndexDataBuffer(INDEX_DATA_TYPE type)
{
	BYTE* buffer = getActivatedIndexDataBuffer();//indexDataBuffer[indexDataBufferIndicator];
#if ENABLE_DATA_GATHER_DOUBLE_BUFFER
	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
#else
	DATA_GATHER_BUFFER_HEAD* pbh = &gatherBufferHead;
#endif

	// ���ҵ�ָ��Ĵ洢����
	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;

	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++) //�����Ǽ�����е������Ƿ񶼴洢���ˡ�
	{
		pInd = &(pbh->bufferIndicator[i]);

		if (pInd->type == type)//ע��Ҫƥ�����͡�
		{
			if (pInd->bufferOffset >= pInd->bufferSize) //��ǰָ���ƫ��������buffersize����ô���Ǵ洢���ˡ�
				return true;
			else
				return false;
		}
	}

	return false;
}

//// ��黺�����Ƿ�����
//bool checkIndexDataBufferIsFull()
//{
//	BYTE* buffer = indexDataBuffer[indexDataBufferIndicator];
//	DATA_GATHER_BUFFER_HEAD* pbh = (DATA_GATHER_BUFFER_HEAD*) buffer;
//
//	// ���ҵ�ָ��Ĵ洢����
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


// ��ָ������д���Ӧ��ָ�껺����
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

	// ���ҵ�ָ��Ĵ洢����
//	INDEX_DATA_BUFFER_INDICATOR* pInd = NULL;
//	for (int i = 0; i < GATHERABLE_INDEX_DATA_COUNT; i++)
//	{
	INDEX_DATA_BUFFER_INDICATOR* pInd = &(pbh->bufferIndicator[idx]);
//		if (pInd->type == type)
//		{
	CCASSERT(pInd->bufferSize >= pInd->bufferOffset + INDEX_DATA_SAMPLE_SIZE[type]);

	// �洢����
	// �Ӵ�������Ϊ�˽������ʱ��ŵ����ݳ�����������Χ����bug
	// ��bug�Ĳ���ԭ����ʱ��ͬ�������
	if (pInd->bufferOffset < pInd->bufferSize)
	{
		memcpy(buffer + (pInd->bufferPos + pInd->bufferOffset), data, size);
		pInd->bufferOffset += size;
		pInd->samples++;            //ÿ�洢һ��ĳ��ָ�����ݣ���ô��������ƫ�ƺ͸�ָ��������������
	}

//#ifdef DEBUG
//			if (pInd->bufferOffset > pInd->bufferSize)
//				while(1); // �����ã����ڼ����ʱ��ŵ����ݳ�����������Χ
//#endif
//			break;
//		}
//	}
}

// ������ָ��������Ϊ0
// ʵ�������õ����ۼ������ݣ�ʵʱ������ʼ��ȡ��������ֵ
void checkResetIndexData()
{
//	iHeartRate = {0}; // ���ֽ�Ϊ���ʣ����ֽ�Ϊ�ɿ���
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


		// ͬʱ����������ֵ���ۼ�ֵ
		iCaloriesBase = 0;
		iStepsBase = 0;
		iDistanceBase = 0;

		iCaloriesAccumulated = 0;
		iStepsAccumulated = 0;
		iDistanceAccumulated = 0;


		//������Ѿ��ﵽ��Ŀ�� 2015��7��1��18:27:34
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

	// ���ڳ���ģʽ��������ʱ�����������ݲɼ�
	if (systemSetting.SystemMode == SYSTEM_MODE_RELEASED)
		return;

	// �����û�����δ��ʼ�����Ա�ɼ����̷�ת������
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
//		// ����������д��flash
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
		// ��ʹ���ɼ����ݣ�Ҳ��Ҫ����ָ�����ݣ���������0
		checkResetIndexData();
		return;
	}

	if (!systemStatus.blDataGatheringInitialized)
	{
		//׼�����µ�sector�Ż�ִ�У�����
		systemStatus.blDataGatheringInitialized = true;

		// ��һ�η�ת���ݲɼ�������������д�뱾�ص�ǰ��ʱ���
		flipAndInitIndexDataBuffer();
	}


	// =============================================================
	BYTE buffer[4];

#ifdef DEBUG
	// ����ģ������
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

	// ȡ�õ�ǰʱ��,
	time_t ts = time(NULL);

	// ��黺�����Ƿ������������򽫵�ǰ����������д��flash
	// Ȼ��׼��������
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

		// �Ƿ񵽴�ָ���������
//#ifdef DEBUG
//		if (ts % (pDef->sampleInterval / 2) == 0)
//#else
		if (ts % (pDef->sampleInterval) == 0)
//#endif
		{
//			// ��黺�����Ƿ������������򽫵�ǰ����������д��flash
//			checkAndPrepareBuffer(type);

			if (pDef->type == DATATYPE_AMBIENT_TEMPERATURE
			        || pDef->type == DATATYPE_SKIN_TEMPERATURE)
			{
				// �¶��ر���
				buffer[0] = (BYTE) * ((float*)pDef->data);
				buffer[1] = (BYTE) ((*((float*)pDef->data) - (float) buffer[0]) * 10);
				putIndexData(i, sampleSize, buffer);
			}
			else if(pDef->type == DATATYPE_HEART_RATE)
			{
				buffer[0] = iHeartRate.component.heart;
				buffer[1] = iHeartRate.component.reliability;
				putIndexData(i, sampleSize, buffer);//���ɼ�ָ��д�뵽��Ӧ������

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
					putIndexData(i, sampleSize, (BYTE*) &data); //����ֻ������������

					//�����ô��룬������Ҫȥ��
//					if (pDef->type == DATATYPE_ACTIVE_LEVEL)
//					{
//						active_level_delta = data;
//					}

					// �����������ݣ��õ�ǰ���ݰ�ԭ�������ݸ��ǣ�
					memcpy(pDef->lastData, pDef->data, pDef->sampleSize);
				}
				else
				{
					putIndexData(i, sampleSize, pDef->data);
				}
			}
		}
	}


	//��0���ʱ������ۼ����Ե���������
	checkResetIndexData();
}


/**
 * Calculate the GCD(Greatest Common Divisor) of m,n.
 * Euclidean algorithm(ŷ������㷨-շת�����)
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
 * Euclidean algorithm - recursive(ŷ������㷨-շת�����)
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
