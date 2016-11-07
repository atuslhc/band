#ifndef DATA_GATHER_H
#define DATA_GATHER_H

#include <stdbool.h>
#include "typedefs.h"

#include "common_vars.h"

// The type of data defined.
typedef enum _INDEX_DATA_TYPE
{
	DATATYPE_UNKNOWN	= 0,
	DATATYPE_HEART_RATE	= 1,
	DATATYPE_ECG,
	DATATYPE_ECG_VAL,
	DATATYPE_AMBIENT_TEMPERATURE,
	DATATYPE_SKIN_TEMPERATURE,
	DATATYPE_ACTIVE_LEVEL,
	DATATYPE_STEPS,
	DATATYPE_DISTANCE,
	DATATYPE_CALORIE_CONSUMPTION,
	DATATYPE_UV,
	DATATYPE_IR_REG,
	DATATYPE_PPG_RAW,		//[BG027] add, RESEARCH_TYPE
	DATATYPE_XYZ_RAW,		//[BG027] add, RESEARCH_TYPE
} INDEX_DATA_TYPE;

//The definition of index data.
typedef struct
{
	INDEX_DATA_TYPE type;	// the data type.
	bool accumulative;	// accumulative type data. Ex, steps, active_level.
	BYTE sampleSize;	// the data size.
	BYTE sampleInterval;	// sampling interval.
	
	BYTE* data;             // the gather data pointer
	BYTE* lastData;         // last gather data pointer(only use for accumulative type). The historical gather accumulative data is the dirrerence. i.e. *data - *lastData

	int weight;		// the weight to caculate the buffer for all index data
} INDEX_DATA_DEF;


//--------------------- The details index data definition -------------------------
static INDEX_DATA_DEF DATA_DEF_HEART_RATE =
{
	DATATYPE_HEART_RATE,
	false,
	2,
	1, // sample interval, in second(s)
	(BYTE*) &iHeartRate,
	//(BYTE*) NULL,		//save code can be skip assigned.
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_ECG =
{
	DATATYPE_ECG,
	false,
	2,
	1
};

static INDEX_DATA_DEF DATA_DEF_ECG_VAL =
{
	DATATYPE_ECG_VAL,
	false,
	2,
	1
};

static INDEX_DATA_DEF DATA_DEF_AMBIENT_TEMPERATURE =
{
	DATATYPE_AMBIENT_TEMPERATURE,
	false,
	2,
	60,
	(BYTE*) &fAmbientTemperature,
	//(BYTE*) NULL,		//save code can be skip assigned.
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_SKIN_TEMPERATURE =
{
	DATATYPE_SKIN_TEMPERATURE,
	false,
	2,
	60,
	(BYTE*) &fSkinTemperature,
	//(BYTE*) NULL,		//save code can be skip assigned.
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_ACTIVE_LEVEL =
{
	DATATYPE_ACTIVE_LEVEL,
	true,
	2,
	60,
	(BYTE*) &active_level,
	(BYTE*) &active_level_lastSaving,
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_STEPS =
{
	DATATYPE_STEPS,
	true,
	2,
	60,
	(BYTE*) &iSteps,
	(BYTE*) &iSteps_lastSaving,
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_DISTANCE =
{
	DATATYPE_DISTANCE,
	true,
	2,
	60,
	(BYTE*) &iDistance,
	(BYTE*) &iDistance_lastSaving,
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_CALORIE_CONSUMPTION =
{
	DATATYPE_CALORIE_CONSUMPTION,
	true,
	2,
	60,
	(BYTE*) &iCalories,
	(BYTE*) &iCalories_lastSaving,
	//0			//save code can be skip assigned.
};

static INDEX_DATA_DEF DATA_DEF_IR_REG =
{
	DATATYPE_IR_REG,
	false,
	2,
	2
};

static INDEX_DATA_DEF DATA_DEF_UV =
{
	DATATYPE_UV,
	false,
	1,
	60,
	(BYTE*) &bUltraVioletGather, //[BG023] bUltraViolet>>bUltraVioletGather
};

/* [BG027] add for record the raw data in RESEARCH_TYPE */
extern int16_t PPG_OUT_DATA;
static INDEX_DATA_DEF DATA_DEF_PPG_RAW =
{
	DATATYPE_PPG_RAW,
	false,
	2,		//2*32=64
	1,
	(BYTE*) &PPG_OUT_DATA,
};
extern int16_t AxieOUT[3];
static INDEX_DATA_DEF DATA_DEF_XYZ_RAW =
{
	DATATYPE_XYZ_RAW,
	false,
	6,		//6*32=192
	1,
	(BYTE*) &AxieOUT,
};
// The list of gather index data.
static INDEX_DATA_DEF* GATHERABLE_INDEX_DATA[] =
{
	&DATA_DEF_HEART_RATE,
	&DATA_DEF_STEPS,
	&DATA_DEF_DISTANCE,
	&DATA_DEF_CALORIE_CONSUMPTION,
	&DATA_DEF_ACTIVE_LEVEL,
	&DATA_DEF_AMBIENT_TEMPERATURE,
	&DATA_DEF_SKIN_TEMPERATURE,
	&DATA_DEF_UV,
#if RESEARCH_TYPE
	&DATA_DEF_PPG_RAW,
	&DATA_DEF_XYZ_RAW,
#endif
};

#define GATHERABLE_INDEX_DATA_COUNT (sizeof(GATHERABLE_INDEX_DATA) / sizeof(INDEX_DATA_DEF*))

//#pragma pack(push, 1)

// =================================================================
// 指标数据采集缓冲区头
typedef struct _INDEX_DATA_BUFFER_INDICATOR
{
	INDEX_DATA_TYPE 	type;
//	bool 	enable;
	INT16	bufferSize;	// buffer大小 （是指每个指标在整个缓存区独自所占的大小）
	INT16	bufferPos;	// buffer位置，对整个缓冲区而言
	INT16	bufferOffset;	// buffer写入偏移量，相对单个指标缓冲区（bufferPos），offset的值就等于已有数据大小
	// 冗余，可通过 samples 计算出来
	INT16	samples;	// 样本个数
} INDEX_DATA_BUFFER_INDICATOR;

/* 采集缓冲区布局
pos相对整个缓冲区
offset相对单个缓冲区

|0   |pos1               |pos2
+----+-------------------+-------------------+
|head|index data buffer 1|index data buffer 2|
+----+-------------------+-------------------+
     |0     offset1      |0    offset2

*/


typedef struct _DATA_GATHER_BUFFER_HEAD
{
	time_t timestamp;	// gmt timestamp
	int16_t timezoneOffset;	// tz offset, 以分钟计
	INDEX_DATA_BUFFER_INDICATOR bufferIndicator[GATHERABLE_INDEX_DATA_COUNT];//指标数据采集头
} DATA_GATHER_BUFFER_HEAD;

//#pragma pack(pop)

//extern bool blEnableDataGathering;

extern BYTE ChunksPerSector;

// =================================================================
/*
初始化指标缓冲区，主要工作是计算各指标独立缓冲区大小，并写入缓冲区头
仅需调用一次
*/
void initIndexDataBuffer();

// 翻转指标缓冲区
// 返回新缓冲区指针
BYTE* flipIndexDataBuffer();

// 翻转并初始化指标缓冲区
// 初始化包括：写入时间戳；重置各个指标缓冲区的写入指针
// 返回缓冲区指针
BYTE* flipAndInitIndexDataBuffer();

// 获得当前缓冲区指针
BYTE* getActivatedIndexDataBuffer();

// 检查缓冲区是否已满
bool checkIndexDataBuffer(INDEX_DATA_TYPE type);

// 将指标数据写入对应的指标缓冲区
//void putIndexData(INDEX_DATA_TYPE type, BYTE* data);
void putIndexData(BYTE idx, BYTE size, BYTE* data);

void ResetIndexData();

void StartDataGathering();
void doDataGathering();
void StopDataGathering(bool flush);

#endif
