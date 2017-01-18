#ifndef __WAVE_FROM_H
#define __WAVE_FROM_H
// Waveform Structure Type Definition

typedef struct Haptics_Waveform
{
	const unsigned char 	inputMode; 		// See input mode
	const unsigned char		length;			// Size of array in bytes
	const unsigned char* 	data;			// Pointer to waveform array data (waveform array is in (amplitude, time) pairs
} WaveForm_TypeDef ;

typedef struct
{
	unsigned char     actuatorType;
	unsigned char     openLoop;               //0 COLSELOOP 1 openLoop
	unsigned char     odClampVoltage;         //过载电压
	unsigned char     ratedVoltage;
	unsigned char     triggerType;            //触发方式
	unsigned char     aCalComp;
	unsigned char     backEMF;
	unsigned char     feedBack;
	const  WaveForm_TypeDef*     ptCWaveform;
} MotorSet_TypeDef;

//--------------------------------------------------------//
// ROM Mode
//--------------------------------------------------------//
typedef enum
{
	Index0,
	StrongClick_100,
	StrongClick_60,
	StrongClick_30,

	SharpClick_100,
	SharpClick_60,
	SharpClick_30,

	SoftBump_100,
	SoftBump_60,
	SoftBump_30,

	DoubleClick_100,
	DoubleClick_60,

	TripleClick_100,

	SoftFuzz_60,

	StrongBuzz_100,

	BuzzAlert750ms,
	BuzzAlert1000ms,
	StrongClick1_100,
	StrongClick2_80,
	StrongClick3_60,
	StrongClick4_30,
	MediumClick1_100,
	MediumClick2_80,
	MediumClick3_60,
	SharpTick1_100,
	SharpTick2_80,
	SharpTick3_60,
	ShortDoubleClickStrong1_100,
	ShortDoubleClickStrong2_80,
	ShortDoubleClickStrong3_60,
	ShortDoubleClickStrong4_30,
	ShortDoubleClickMedium1_100,
	ShortDoubleClickMedium2_80,
	ShortDoubleClickMedium3_60,
	ShortDoubleSharpTick1_100,
	ShortDoubleSharpTick2_80,
	ShortDoubleSharpTick3_60,
	LongDoubleSharpClickStrong1_100,
	LongDoubleSharpClickStrong2_80,
	LongDoubleSharpClickStrong3_60,
	LongDoubleSharpClickStrong4_30,
	LongDoubleSharpClickMedium1_100,
	LongDoubleSharpClickMedium2_80,
	LongDoubleSharpClickMedium3_60,
	LongDoubleSharpTick1_100,
	LongDoubleSharpTick2_80,
	LongDoubleSharpTick3_60,
	Buzz1_100,
	Buzz2_80,
	Buzz3_60,
	Buzz4_40,
	Buzz5_20,
	PulsingStrong1_100,
	PulsingStrong2_60,
	PulsingMedium1_100,
	PulsingMedium2_60,
	PulsingSharp1_100,
	PulsingSharp2_60,
	TransitionClick1_100,
	TransitionClick2_80,
	TransitionClick3_60,
	TransitionClick4_40,
	TransitionClick5_20,
	TransitionClick6_10,
	TransitionHum1_100,
	TransitionHum2_80,
	TransitionHum3_60,
	TransitionHum4_40,
	TransitionHum5_20,
	TransitionHum6_10,
	TransitionRampDownLongSmooth1_100to0,
	TransitionRampDownLongSmooth2_100to0,
	TransitionRampDownMediumSmooth1_100to0,
	TransitionRampDownMediumSmooth2_100to0,
	TransitionRampDownShortSmooth1_100to0,
	TransitionRampDownShortSmooth2_100to0,
	TransitionRampDownLongSharp1_100to0,
	TransitionRampDownLongSharp2_100to0,
	TransitionRampDownMediumSharp1_100to0,
	TransitionRampDownMediumSharp2_100to0,
	TransitionRampDownShortSharp1_100to0,
	TransitionRampDownShortSharp2_100to0,
	TransitionRampUpLongSmooth1_0to100,
	TransitionRampUpLongSmooth2_0to100,
	TransitionRampUpMediumSmooth1_0to100,
	TransitionRampUpMediumSmooth2_0to100,
	TransitionRampUpShortSmooth1_0to100,
	TransitionRampUpShortSmooth2_0to100,
	TransitionRampUpLongSharp1_0to100,
	TransitionRampUpLongSharp2_0to100,
	TransitionRampUpMediumSharp1_0to100,
	TransitionRampUpMediumSharp2_0to100,
	TransitionRampUpShortSharp1_0to100,
	TransitionRampUpShortSharp2_0to100,
	TransitionRampDownLongSmooth1_50to0,
	TransitionRampDownLongSmooth2_50to0,
	TransitionRampDownMediumSmooth1_50to0,
	TransitionRampDownMediumSmooth2_50to0,
	TransitionRampDownShortSmooth1_50to0,
	TransitionRampDownShortSmooth2_50to0,
	TransitionRampDownLongSharp1_50to0,
	TransitionRampDownLongSharp2_50to0,
	TransitionRampDownMediumSharp1_50to0,
	TransitionRampDownMediumSharp2_50to0,
	TransitionRampDownShortSharp1_50to0,
	TransitionRampDownShortSharp2_50to0,
	TransitionRampUpLongSmooth1_0to50,
	TransitionRampUpLongSmooth2_0to50,
	TransitionRampUpMediumSmooth1_0to50,
	TransitionRampUpMediumSmooth2_0to50,
	TransitionRampUpShortSmooth1_0to50,
	TransitionRampUpShortSmooth2_0to50,
	TransitionRampUpLongSharp1_0to50,
	TransitionRampUpLongSharp2_0to50,
	TransitionRampUpMediumSharp1_0to50,
	TransitionRampUpMediumSharp2_0to50,
	TransitionRampUpShortSharp1_0to50,
	TransitionRampUpShortSharp2_0to50,
	Longbuzzforprogrammaticstopping_100,
	SmoothHum1Nokickorbrakepulse_50,
	SmoothHum2Nokickorbrakepulse_40,
	SmoothHum3Nokickorbrakepulse_30,
	SmoothHum4Nokickorbrakepulse_20,
	SmoothHum5Nokickorbrakepulse_10
} VIBRATE_MODE;

#endif