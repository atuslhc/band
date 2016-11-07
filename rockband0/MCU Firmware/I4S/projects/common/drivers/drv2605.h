/*

** File:
**     drv2605.h
**
** Description:
**     Header file for drv2605.c
**
** =============================================================================
*/

/*
  Drv2605
  This file created by Immersion Corporation as an interim solution until
  Texas Instruments provides an official header for the DRV2605 part.
  Based on "DRV2605 Preliminary Datasheet 3-7-12 - Immersion.pdf"
*/

/*
** DRV2605 addresses
*/
#ifndef __DRV2605_H
#define __DRV2605_H

#include "stdint.h"
#include "waveForm.h"

#define DEVICE_NAME "drv2605"
#define DRIVER_VERSION "182"

/* Commands */
#define HAPTIC_CMDID_PLAY_SINGLE_EFFECT     0x01
#define HAPTIC_CMDID_PLAY_EFFECT_SEQUENCE   0x02
#define HAPTIC_CMDID_PLAY_TIMED_EFFECT      0x03
#define HAPTIC_CMDID_GET_DEV_ID             0x04
#define HAPTIC_CMDID_RUN_DIAG               0x05
#define HAPTIC_CMDID_AUDIOHAPTIC_ENABLE     0x06
#define HAPTIC_CMDID_AUDIOHAPTIC_DISABLE    0x07
#define HAPTIC_CMDID_AUDIOHAPTIC_GETSTATUS  0x08
#define HAPTIC_CMDID_STOP                   0xFF

/* Command size */
#define HAPTIC_CMDSZ_SINGLE_EFFECT     2
#define HAPTIC_CMDSZ_EFFECT_SEQUENCE   9
#define HAPTIC_CMDSZ_TIMED_EFFECT      3
#define HAPTIC_CMDSZ_STOP              1

/*
** Go
*/
#define GO_REG 0x0C
#define GO     0x01
#define STOP   0x00

/*
** Status
*/
#define STATUS_REG          0x00
#define STATUS_DEFAULT      0x00

#define DIAG_RESULT_MASK    (1 << 3)
#define AUTO_CAL_PASSED     (0 << 3)
#define AUTO_CAL_FAILED     (1 << 3)
#define DIAG_GOOD           (0 << 3)
#define DIAG_BAD            (1 << 3)

#define DEV_ID_MASK (7 << 5)
#define DRV2605 (5 << 5)
#define DRV2604 (4 << 5)

/*
** Mode
*/
#define MODE_REG            0x01
#define MODE_STANDBY        0x40

#define DRV260X_MODE_MASK           0x07
#define MODE_INTERNAL_TRIGGER       0
#define MODE_EXTERNAL_TRIGGER_EDGE  1
#define MODE_EXTERNAL_TRIGGER_LEVEL 2
#define MODE_PWM_OR_ANALOG_INPUT    3
#define MODE_AUDIOHAPTIC            4
#define MODE_REAL_TIME_PLAYBACK     5
#define MODE_DIAGNOSTICS            6
#define AUTO_CALIBRATION            7

#define MODE_STANDBY_MASK           0x40
#define MODE_READY                  1 // default
#define MODE_SOFT_STANDBY           0

#define MODE_RESET                  0x80

/*
** Real Time Playback
*/
#define REAL_TIME_PLAYBACK_REG      0x02

/*
** Library Selection
*/
#define LIBRARY_SELECTION_REG       0x03
#define LIBRARY_SELECTION_DEFAULT   0x00

#define LIBRARY_A 0x01
#define LIBRARY_B 0x02
#define LIBRARY_C 0x03
#define LIBRARY_D 0x04
#define LIBRARY_E 0x05
#define LIBRARY_F 0x06

#define LIBRARY_SELECTION_MASK              0x07
#define LIBRARY_SELECTION_LIBRARY_RAM       0 // default
#define LIBRARY_SELECTION_LIBRARY_OVERDRIVE 1
#define LIBRARY_SELECTION_LIBRARY_40_60     2
#define LIBRARY_SELECTION_LIBRARY_60_80     3
#define LIBRARY_SELECTION_LIBRARY_100_140   4
#define LIBRARY_SELECTION_LIBRARY_140_PLUS  5

#define LIBRARY_SELECTION_HIZ_MASK          0x10
#define LIBRARY_SELECTION_HIZ_EN            1
#define LIBRARY_SELECTION_HIZ_DIS           0

/*
** Waveform Sequencer
*/
#define WAVEFORM_SEQUENCER_REG1     0x04
#define WAVEFORM_SEQUENCER_REG2     0x05
#define WAVEFORM_SEQUENCER_REG3     0x06
#define WAVEFORM_SEQUENCER_REG4     0x07
#define WAVEFORM_SEQUENCER_REG5     0x08
#define WAVEFORM_SEQUENCER_REG6     0x09
#define WAVEFORM_SEQUENCER_REG7     0x0A
#define WAVEFORM_SEQUENCER_REG8     0x0B
#define WAVEFORM_SEQUENCER_MAX      8
#define WAVEFORM_SEQUENCER_DEFAULT  0x00

/*
** OverDrive Time Offset
*/
#define OVERDRIVE_TIME_OFFSET_REG  0x0D

/*
** Sustain Time Offset, postive
*/
#define SUSTAIN_TIME_OFFSET_POS_REG 0x0E

/*
** Sustain Time Offset, negative
*/
#define SUSTAIN_TIME_OFFSET_NEG_REG 0x0F

/*
** Brake Time Offset
*/
#define BRAKE_TIME_OFFSET_REG       0x10

/*
** Audio to Haptics Control
*/
#define AUDIO_HAPTICS_CONTROL_REG   0x11

#define AUDIO_HAPTICS_RECT_10MS     (0 << 2)
#define AUDIO_HAPTICS_RECT_20MS     (1 << 2)
#define AUDIO_HAPTICS_RECT_30MS     (2 << 2)
#define AUDIO_HAPTICS_RECT_40MS     (3 << 2)

#define AUDIO_HAPTICS_FILTER_100HZ  0
#define AUDIO_HAPTICS_FILTER_125HZ  1
#define AUDIO_HAPTICS_FILTER_150HZ  2
#define AUDIO_HAPTICS_FILTER_200HZ  3

/*
** Audio to Haptics Minimum Input Level
*/
#define AUDIO_HAPTICS_MIN_INPUT_REG 0x12

/*
** Audio to Haptics Maximum Input Level
*/
#define AUDIO_HAPTICS_MAX_INPUT_REG 0x13

/*
** Audio to Haptics Minimum Output Drive
*/
#define AUDIO_HAPTICS_MIN_OUTPUT_REG 0x14

/*
** Audio to Haptics Maximum Output Drive
*/
#define AUDIO_HAPTICS_MAX_OUTPUT_REG 0x15

/*
** Rated Voltage 额定电压
**只可以用在开环模式下
*/
#define RATED_VOLTAGE_REG           0x16
//ERM closed loop 
//      Vavg=[7:0]*5.44V/255
//LRA open loop 
//      Vavg_abs=[7:0]*5.28V/255

/*
** Overdrive Clamp Voltage
*/
#define OVERDRIVE_CLAMP_VOLTAGE_REG 0x17
//过驱动的输出峰值电压
//ERM open loop or LRA closed loop 
//      Vod =[7:0]*5.6V/255
//ERM closed loop 
//      Vpeak=[7:0]*5.44V/255
//LRA open loop 
//      Vpeak=[7:0]*5.44V/255

/*
** Auto Calibrationi Compensation Result
*/
#define AUTO_CALI_RESULT_REG        0x18
//自动校准补尝的
//=1+[7..0]/255

/*
** Auto Calibration Back-EMF Result
*/
#define AUTO_CALI_BACK_EMF_RESULT_REG 0x19
//自动校准反电动势
//Back-EMF(V)=[7..0]/255*4.88v/BEMF Gain
/*
** Feedback Control
*/
#define FEEDBACK_CONTROL_REG        0x1A
//反馈控制：
/*7: 0:ERM 1:LRA
6-4: 反馈B
*/
#define FEEDBACK_CONTROL_ERM ((0<<7) | (3<<4) | (1<<2) |(2<<0))
#define FEEDBACK_CONTROL_LRM ((1<<7) | (3<<4) | (1<<2) |(2<<0))

#define FEEDBACK_CONTROL_BEMF_ERM_GAIN0 0 // 0.33x
#define FEEDBACK_CONTROL_BEMF_ERM_GAIN1 1 // 1.0x
#define FEEDBACK_CONTROL_BEMF_ERM_GAIN2 2 // 1.8x
#define FEEDBACK_CONTROL_BEMF_ERM_GAIN3 3 // 4.0x

#define FEEDBACK_CONTROL_BEMF_LRA_GAIN0 0 // 5x
#define FEEDBACK_CONTROL_BEMF_LRA_GAIN1 1 // 10x
#define FEEDBACK_CONTROL_BEMF_LRA_GAIN2 2 // 20x
#define FEEDBACK_CONTROL_BEMF_LRA_GAIN3 3 // 30x

#define LOOP_RESPONSE_SLOW      (0 << 2)
#define LOOP_RESPONSE_MEDIUM    (1 << 2) // default
#define LOOP_RESPONSE_FAST      (2 << 2)
#define LOOP_RESPONSE_VERY_FAST (3 << 2)

#define FB_BRAKE_FACTOR_1X   (0 << 4) // 1x
#define FB_BRAKE_FACTOR_2X   (1 << 4) // 2x
#define FB_BRAKE_FACTOR_3X   (2 << 4) // 3x (default)
#define FB_BRAKE_FACTOR_4X   (3 << 4) // 4x
#define FB_BRAKE_FACTOR_6X   (4 << 4) // 6x
#define FB_BRAKE_FACTOR_8X   (5 << 4) // 8x
#define FB_BRAKE_FACTOR_16X  (6 << 4) // 16x
#define FB_BRAKE_DISABLED    (7 << 4)

#define FEEDBACK_CONTROL_MODE_ERM 0 // default
#define FEEDBACK_CONTROL_MODE_LRA (1 << 7)

/*
** Control1
*/
#define Control1_REG            0x1B

#define STARTUP_BOOST_ENABLED   (1 << 7)
#define STARTUP_BOOST_DISABLED  (0 << 7) // default
#define AC_COUPLE_ENABLED       (1 << 5)
#define AC_COUPLE_DISABLED      (0 << 5) // default

#define DEFAULT_DRIVE_TIME      0x17
#define AUDIOHAPTIC_DRIVE_TIME  0x13

/*
** Control2
*/
#define Control2_REG            0x1C

#define IDISS_TIME_MASK         0x03
#define IDISS_TIME_VERY_SHORT   0
#define IDISS_TIME_SHORT        1
#define IDISS_TIME_MEDIUM       2 // default
#define IDISS_TIME_LONG         3

#define BLANKING_TIME_MASK          0x0C
#define BLANKING_TIME_VERY_SHORT    (0 << 2)
#define BLANKING_TIME_SHORT         (1 << 2)
#define BLANKING_TIME_MEDIUM        (2 << 2) // default
#define BLANKING_TIME_VERY_LONG     (3 << 2)

#define AUTO_RES_GAIN_MASK         0x30
#define AUTO_RES_GAIN_VERY_LOW     (0 << 4)
#define AUTO_RES_GAIN_LOW          (1 << 4)
#define AUTO_RES_GAIN_MEDIUM       (2 << 4) // default
#define AUTO_RES_GAIN_HIGH         (3 << 4)

#define SOFT_BRAKE_MASK            0x40

#define BIDIR_INPUT_MASK           0x80
#define UNIDIRECT_INPUT            (0 << 7)
#define BIDIRECT_INPUT             (1 << 7) // default

/*
** Control3
*/
#define Control3_REG 0x1D

  
#define INPUT_PWM               (0 << 1) // default
#define INPUT_ANALOG            (1 << 1)
#define ERM_OpenLoop_Enabled    (1 << 5)
#define NG_Thresh_DISABLED      (0 << 6)
#define NG_Thresh_1             (1 << 6)
#define NG_Thresh_2             (2 << 6)
#define NG_Thresh_3             (3 << 6)

/*
** Auto Calibration Memory Interface
*/
#define AUTOCAL_MEM_INTERFACE_REG   0x1E

#define AUTOCAL_TIME_150MS          (0 << 4)
#define AUTOCAL_TIME_250MS          (1 << 4)
#define AUTOCAL_TIME_500MS          (2 << 4)
#define AUTOCAL_TIME_1000MS         (3 << 4)

#define SILICON_REVISION_REG        0x3B
#define SILICON_REVISION_MASK       0x07

#define AUDIO_HAPTICS_MIN_INPUT_VOLTAGE     0x19
#define AUDIO_HAPTICS_MAX_INPUT_VOLTAGE     0x64
#define AUDIO_HAPTICS_MIN_OUTPUT_VOLTAGE    0x19
#define AUDIO_HAPTICS_MAX_OUTPUT_VOLTAGE    0xFF

#define DEFAULT_ERM_AUTOCAL_COMPENSATION    0x14
#define DEFAULT_ERM_AUTOCAL_BACKEMF         0x72

#define DEFAULT_LRA_AUTOCAL_COMPENSATION    0x06
#define DEFAULT_LRA_AUTOCAL_BACKEMF         0xDE


#define DRV2605_I2C_ADD  0xb4
void DRV2605_Init(void);
unsigned char DRV2605_ReadReg(unsigned char Reg, unsigned char* Data);
unsigned char DRV2605_WriteReg(unsigned char WriteAddr, unsigned char Data);
void VibrateCon(VIBRATE_MODE act, uint8_t duration, uint8_t times);
void stopVibrate();

void CheckVibrateStatus(void);

/****************************************************************************/
/* Feedback Control Register */
/****************************************************************************/
#define DRV260x_FEEDBACK_CONTROL	(0x1A)
/*** Register Bits ***/
#define ERM_MODE				(0x00)
#define LRA_MODE				(0x80)

#define FBBrakeFactor_1x		(0x00)
#define FBBrakeFactor_2x		(0x10)
#define FBBrakeFactor_3x		(0x20)
#define FBBrakeFactor_4x		(0x30)
#define FBBrakeFactor_6x		(0x40)
#define FBBrakeFactor_8x		(0x50)
#define FBBrakeFactor_16x		(0x60)
#define FBBrakeFactor_Disabled	(0x70)

#define LoopResponse_Slow		(0x00)
#define LoopResponse_Medium		(0x04)
#define LoopResponse_Fast		(0x08)
#define LoopResponse_VeryFast	(0x0C)

/* The AutoCal routine will populate BEMFGain
 * Interpretation of BEMFGain changes based on ERM/LRA MODE
 */
#define BEMFGainERM_0p3x		(0x00)
#define BEMFGainERM_1x			(0x01)
#define BEMFGainERM_1p8x		(0x02)
#define BEMFGainERM_4x			(0x03)
#define BEMFGainLRA_5x			(0x00)
#define BEMFGainLRA_10x			(0x01)
#define BEMFGainLRA_20x			(0x02)
#define BEMFGainLRA_30x			(0x03)

/****************************************************************************/
/* Control1 Register */
/****************************************************************************/
#define DRV260x_CONTROL1	(0x1B)
/*** Register Bits ***/
#define StartupBoost	(0x80)
#define BypassComp		(0x40)
#define AC_Couple		(0x20)
#define DC_Couple		(0x00)

#define DriveTime_0p5m  (0x00)
#define DriveTime_0p6m  (0x01)
#define DriveTime_0p7m  (0x02)
#define DriveTime_0p8m  (0x03)
#define DriveTime_0p9m  (0x04)
#define DriveTime_1p0m  (0x05)
#define DriveTime_1p1m  (0x06)
#define DriveTime_1p2m  (0x07)
#define DriveTime_1p3m  (0x08)
#define DriveTime_1p4m  (0x09)
#define DriveTime_1p5m  (0x0A)
#define DriveTime_1p6m  (0x0B)
#define DriveTime_1p7m  (0x0C)
#define DriveTime_1p8m  (0x0D)
#define DriveTime_1p9m  (0x0E)
#define DriveTime_2p0m  (0x0F)
#define DriveTime_2p1m  (0x10)
#define DriveTime_2p2m  (0x11)
#define DriveTime_2p3m  (0x12)
#define DriveTime_2p4m  (0x13)
#define DriveTime_2p5m  (0x14)
#define DriveTime_2p6m  (0x15)
#define DriveTime_2p7m  (0x16)
#define DriveTime_2p8m  (0x17)
#define DriveTime_2p9m  (0x18)
#define DriveTime_3p0m  (0x19)
#define DriveTime_3p1m  (0x1A)
#define DriveTime_3p2m  (0x1B)
#define DriveTime_3p3m  (0x1C)
#define DriveTime_3p4m  (0x1D)
#define DriveTime_3p5m  (0x1E)
#define DriveTime_3p6m  (0x1F)

/****************************************************************************/
/* Control2 Register */
/****************************************************************************/
#define DRV260x_CONTROL2		(0x1C)
/*** Register Bits ***/
#define BiDir_Input				(0x80)
#define UniDir_Input			(0x00)

#define BrakeStabilizer			(0x40)

#define SampleTime_150us		(0x00)
#define SampleTime_200us		(0x10)
#define SampleTime_250us		(0x20)
#define SampleTime_300us		(0x30)

#define BlankingTime_VeryShort	(0x00)
#define BlankingTime_Short   	(0x04)
#define BlankingTime_Medium 	(0x08)
#define BlankingTime_Long  		(0x06)

#define IDissTime_VeryShort 	(0x00)
#define IDissTime_Short   		(0x01)
#define IDissTime_Medium   		(0x02)
#define IDissTime_Long   		(0x03)

/****************************************************************************/
/* Control3 Register */
/****************************************************************************/
#define DRV260x_CONTROL3		(0x1D)
/*** Register Bits ***/
#define NGThresh_OFF      		(0x00)
#define NGThresh_2PERCENT 		(0x40)
#define NGThresh_4PERCENT 		(0x80)
#define NGThresh_8PERCENT 		(0xC0)

#define ERM_ClosedLoop			(0x00)
#define ERM_OpenLoop			(0x20)
#define SupplyCompDis			(0x10)

#define	DataFormat_RTP_Signed	(0x00)
#define	DataFormat_RTP_Unsigned	(0x08)

#define LRADriveMode			(0x04)
#define LRADriveMode_Once		(0x00)
#define LRADriveMode_Twice		(0x04)

#define InputMode_PWM     		(0x00)
#define InputMode_ANALOG  		(0x02)

#define LRA_AutoResonance		(0x00)
#define LRA_OpenLoop			(0x01)
#define LRA_ClosedLoop			(0x00)


/****************************************************************************/
/* Rated Voltage Register */
/****************************************************************************/
#define DRV260x_RATED_VOLTAGE	(0x16)
/* Rated Voltage (V) = RatedVoltage[7:0] / 255 * 5.3 V
 * User should write this value before performing autocal
 */

// @TODO - Recaluclate Voltages

// ERM - Open Loop
#define Voltage_1p3	(0x3B)
#define Voltage_1p5	(0x44)
#define Voltage_1p8	(0x52)
#define Voltage_2p0	(0x5B)
#define Voltage_2p3	(0x69)
#define Voltage_2p5	(0x72)
#define Voltage_2p7	(0x7B)
#define Voltage_3p0	(0x89)
#define Voltage_3p3	(0x96)
#define Voltage_3p6 (0xA4)
#define Voltage_5p0	(0xE4)

// ERM Overdrive - Closed Loop
#define Voltage_ERM_OD_CL_1p3	(0x43)
#define Voltage_ERM_OD_CL_1p5	(0x4D)
#define Voltage_ERM_OD_CL_1p8	(0x5D)
#define Voltage_ERM_OD_CL_2p0	(0x67)
#define Voltage_ERM_OD_CL_2p5	(0x81)
#define Voltage_ERM_OD_CL_2p7	(0x8B)
#define Voltage_ERM_OD_CL_3p0	(0x9B)
#define Voltage_ERM_OD_CL_3p3	(0xAA)
#define Voltage_ERM_OD_CL_3p6 	(0xBA)
#define Voltage_ERM_OD_CL_5p0	(0xFF)

// ERM Rated Voltage - Closed Loop
#define Voltage_ERM_RV_CL_1p3	(0x3D)
#define Voltage_ERM_RV_CL_1p5	(0x46)
#define Voltage_ERM_RV_CL_1p8	(0x54)
#define Voltage_ERM_RV_CL_2p0	(0x5E)
#define Voltage_ERM_RV_CL_2p5	(0x75)
#define Voltage_ERM_RV_CL_2p7	(0x7F)
#define Voltage_ERM_RV_CL_3p0	(0xBD)
#define Voltage_ERM_RV_CL_3p3	(0x9B)
#define Voltage_ERM_RV_CL_3p6 	(0xA9)
#define Voltage_ERM_RV_CL_5p0	(0xEB)

// LRA - RMS Voltages for LRA Rated Voltage
#define VoltageRMS_LRA_RV_1p5	(0x3E)	// 1.5 Vrms
#define VoltageRMS_LRA_RV_2p0	(0x53)	// 2.0 Vrms

#define LRA_CTRL2 				SampleTime_300us | BlankingTime_Short | IDissTime_Short

#define DEFAULT_CTRL1	StartupBoost | DriveTime_2p4m
#define DEFAULT_CTRL2	BiDir_Input| BrakeStabilizer | SampleTime_300us | BlankingTime_Short | IDissTime_Short
#define DEFAULT_CTRL3	NGThresh_4PERCENT | ERM_ClosedLoop | DataFormat_RTP_Signed | LRADriveMode_Once | InputMode_PWM | LRA_AutoResonance


#endif
