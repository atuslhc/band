#ifndef  SI1132_H
#define  SI1132_H

#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

#include "stdint.h"

//addr
#define SIl4x_IIC_ADDR               0xc0//芯片其从地址
#define SI14x_GLOBAL_RST_CMD         0x06 
//write-read bit
#define SI14x_WRT_FLG                0x00
#define SI14x_RD_FLG                 0x01
//#define I2C_SUBA_ONEBYTE             0x01//要写几个字节

#define INDOOR  0
#define OUTDOOR  1



// I2C Registers

#define REG_PART_ID               0x00
#define REG_REV_ID                0x01
#define REG_SEQ_ID                0x02
#define REG_INT_CFG               0x03
#define REG_IRQ_ENABLE            0x04
#define REG_IRQ_MODE1             0x05
#define REG_IRQ_MODE2             0x06
#define REG_HW_KEY                0x07
#define REG_MEAS_RATE             0x08
#define REG_ALS_RATE              0x09
#define REG_PS_RATE               0x0A
#define REG_ALS_LO_TH_LSB         0x0B
#define REG_ALS_LO_TH_MSB         0x0C
#define REG_ALS_HI_TH_LSB         0x0D
#define REG_ALS_HI_TH_MSB         0x0E
#define REG_PS_LED21              0x0F
#define REG_PS_LED3               0x10
#define REG_PS1_TH_LSB            0x11
#define REG_PS1_TH_MSB            0x12
#define REG_PS2_TH_LSB            0x13
#define REG_PS2_TH_MSB            0x14
#define REG_PS3_TH_LSB            0x15
#define REG_PS3_TH_MSB            0x16
#define REG_PARAM_WR              0x17
#define REG_COMMAND               0x18
#define REG_RESPONSE              0x20
#define REG_IRQ_STATUS            0x21
#define REG_ALS_VIS_DATA0         0x22
#define REG_ALS_VIS_DATA1         0x23
#define REG_ALS_IR_DATA0          0x24
#define REG_ALS_IR_DATA1          0x25
#define REG_PS1_DATA0             0x26
#define REG_PS1_DATA1             0x27
#define REG_PS2_DATA0             0x28
#define REG_PS2_DATA1             0x29
#define REG_PS3_DATA0             0x2A
#define REG_PS3_DATA1             0x2B
#define REG_AUX_DATA0             0x2C
#define REG_AUX_DATA1             0x2D
#define REG_PARAM_OUT             0x2E
#define REG_PARAM_RD              0x2E
#define REG_CHIP_STAT             0x30

#define REG_UCOEF0                0x13  
#define REG_UCOEF1                0x14
#define REG_UCOEF2                0x15  
#define REG_UCOEF3                0x16  

#define REG_MEAS_RATE_LSB         0x08
#define REG_MEAS_RATE_MSB         0x09

// Parameter Offsets
#define PARAM_I2C_ADDR            0x00
#define PARAM_CH_LIST             0x01
#define PARAM_PSLED12_SELECT      0x02
#define PARAM_PSLED3_SELECT       0x03
#define PARAM_FILTER_EN           0x04
#define PARAM_PS_ENCODING         0x05
#define PARAM_ALS_ENCODING        0x06
#define PARAM_PS1_ADC_MUX         0x07
#define PARAM_PS2_ADC_MUX         0x08
#define PARAM_PS3_ADC_MUX         0x09
#define PARAM_PS_ADC_COUNTER      0x0A
#define PARAM_PS_ADC_CLKDIV       0x0B
#define PARAM_PS_ADC_GAIN         0x0B
#define PARAM_PS_ADC_MISC         0x0C
#define PARAM_VIS_ADC_MUX         0x0D
#define PARAM_IR_ADC_MUX          0x0E
#define PARAM_AUX_ADC_MUX         0x0F
#define PARAM_ALSVIS_ADC_COUNTER  0x10
#define PARAM_ALSVIS_ADC_CLKDIV   0x11
#define PARAM_ALSVIS_ADC_GAIN     0x11
#define PARAM_ALSVIS_ADC_MISC     0x12
#define PARAM_ALS_HYST            0x16
#define PARAM_PS_HYST             0x17
#define PARAM_PS_HISTORY          0x18
#define PARAM_ALS_HISTORY         0x19
#define PARAM_ADC_OFFSET          0x1A
#define PARAM_SLEEP_CTRL          0x1B
#define PARAM_LED_RECOVERY        0x1C
#define PARAM_ALSIR_ADC_COUNTER   0x1D
#define PARAM_ALSIR_ADC_CLKDIV    0x1E
#define PARAM_ALSIR_ADC_GAIN      0x1E
#define PARAM_ALSIR_ADC_MISC      0x1F



//在有覆盖物下的UV指数是通过y=ax+b计算得到的，这里的a为斜率，b位截距
#define SLOPE_B                     1.0
#define INTERCEPT_B                 0.4//上面这两个数据是通过多次有覆盖物和没有覆盖物试验得到数据拟合求到的

#define SLOPE_R                     0.629 
#define INTERCEPT_R                 -0.2

// REG_IRQ_CFG 
#define  ICG_INTOE                0x01
#define  ICG_INTMODE              0x02


// REG_IRQ_ENABLE
// REG_IRQ_STATUS
#define IE_NONE                   0x00

#define IE_ALS_NONE               0x00
#define IE_ALS_EVRYSAMPLE         0x01
#define IE_ALS_EXIT_WIN           0x01
#define IE_ALS_ENTER_WIN          0x02

#define IE_PS1_NONE               0x00
#define IE_PS1_EVRYSAMPLE         0x04
#define IE_PS1_CROSS_TH           0x04
#define IE_PS1_EXCEED_TH          0x04
#define IE_PS1                    0x04

#define IE_PS2_NONE               0x00
#define IE_PS2_EVRYSAMPLE         0x08
#define IE_PS2_CROSS_TH           0x08
#define IE_PS2_EXCEEED_TH         0x08
#define IE_PS2                    0x08

#define IE_PS3_NONE               0x00
#define IE_PS3_EVRYSAMPLE         0x10
#define IE_PS3_CROSS_TH           0x10
#define IE_PS3_EXCEED_TH          0x10
#define IE_PS3                    0x10

#define IE_CMD                    0x20

#define IE_ALL                    0x3F

// REG_IRQ_MODE1
#define IM1_NONE                  0x00
#define IM1_ALS_NONE              0x00
#define IM1_ALS_EVRYSAMPLE        0x00
#define IM1_ALS_VIS_EXIT          0x01
#define IM1_ALS_VIS_ENTER         0x05
#define IM1_ALS_IR_EXIT           0x03
#define IM1_ALS_IR_ENTER          0x06

#define IM1_PS1_NONE              0x00
#define IM1_PS1_EVRYSAMPLE        (0x0<<4)
#define IM1_PS1_CROSS_TH          (0x1<<4)
#define IM1_PS1_EXCEED_TH         (0x3<<4)

#define IM1_PS2_NONE              0x00
#define IM1_PS2_EVRYSAMPLE        (0x0<<6)
#define IM1_PS2_CROSS_TH          (0x1<<6)
#define IM1_PS2_EXCEED_TH         (0x3<<6)


// REG_IRQ_MODE1
#define IM2_PS3_NONE              0x00
#define IM2_PS3_EVRYSAMPLE        (0x0)
#define IM2_PS3_CROSS_TH          (0x1)
#define IM2_PS3_EXCEED_TH         (0x3)

//REG_PS_LED21                           LED2 current is upper nibble
//                                                 LED1 current is low nibble
//REG_PS_LED3                             LED3 current is low nibble

#define  LED1_000                0x00
#define  LED1_006                0x01
#define  LED1_011                0x02
#define  LED1_022                0x03
#define  LED1_045                0x04
#define  LED1_067                0x05
#define  LED1_090                0x06
#define  LED1_112                0x07
#define  LED1_135                0x08
#define  LED1_157                0x09
#define  LED1_180                0x0A
#define  LED1_202                0x0B
#define  LED1_224                0x0C
#define  LED1_269                0x0D
#define  LED1_314                0x0E
#define  LED1_359                0x0F



// PARAM_CH_LIST
#define PS1_TASK                  0x01
#define PS2_TASK                  0x02
#define PS3_TASK                  0x04
#define ALS_VIS_TASK              0x10
#define ALS_IR_TASK               0x20
#define AUX_TASK                  0x40
#define UV_TASK                   0x80



// ADC Counters
// PARAM_PS_ADC_COUNTER      
// PARAM_ALSVIS_ADC_COUNTER  
// PARAM_ALSIR_ADC_COUNTER  
//
#define RECCNT_001                0x00
#define RECCNT_007                0x10
#define RECCNT_015                0x20
#define RECCNT_031                0x30
#define RECCNT_063                0x40
#define RECCNT_127                0x50
#define RECCNT_255                0x60
#define RECCNT_511                0x70



//
// PARAM_PS_ENCODING  
// When these bits are set the corresponding measurement 
// will report the least significant bits of the
// ADC is used instead of the most significant bits
#define PS1_LSB                   0x10
#define PS2_LSB                   0x20
#define PS3_LSB                   0x40
#define PS_ENCODING_MASK          0x70



// PARAM_ALS_ENCODING 
// When these bits are set the corresponding measurement 
// will report the least significant bits of the
// ADC is used instead of the most significant bits
#define ALS_VIS_LSB               0x10
#define ALS_IR_LSB                0x20
#define AUX_LSB                   0x40
#define ALS_ENCODING_MASK         0xCF


#define NOT_PS_MEAS_MODE          0x00 
#define PS_MEAS_MODE              0x04
#define HSIG_EN                   0x20
#define RANGE_EN                  0x20

#define ALS_IR_ADC_MISC_MASK      0x20
#define ALS_VIS_ADC_MISC_MASK     0x20

// ADC Mux Settings
// PARAM_PS1_ADC_MUX    See PARAM_PS_ADC_MISC also 
// PARAM_PS2_ADC_MUX    See PARAM_PS_ADC_MISC also
// PARAM_PS3_ADC_MUX    See PARAM_PS_ADC_MISC also
//
// PARAM_VIS_ADC_MUX    MUX_ALS_VIS or MUX_NONE only
// PARAM_IR_ADC_MUX     MUX_ALS_IR, MUX_PS_IR or MUX_NONE only
// PARAM_AUX_ADC_MUX    MUX_VTEMP, MUX_LED1, MUX_LED2, MUX_INT
//                      to use anything other than MUX_VTEMP, 
//                      ANA_IN_KEY should be unlocked first.
//
#define MUX_SMALL_IR              0x00
#define MUX_VIS                   0x02
#define MUX_LARGE_IR              0x03
#define MUX_NO_PHOTO_DIODE        0x06
#define MUX_VTEMP      	          0x65
#define MUX_INT      	          0x05
#define MUX_LED1      	          0x15
#define MUX_VSS      	          0x25
#define MUX_LED2      	          0x35
#define MUX_VDD      	          0x75


// Hardware Key value
// REG_HW_KEY
#define HW_KEY_VAL0               0x17

// Sleep Control
// PARAM_SLEEP_CTRL 
#define SLEEP_DISABLED            0x01

// ANA_IN_KEY value
#define ANA_KEY_38                0x10
#define ANA_KEY_39                0x40
#define AMA_KEY_3A                0x62
#define ANA_KEY_3B                0x3b


// This structure is used to store the result of the calibration retrieval
typedef struct 
{
    float   vispd_correction;
    float   irpd_correction;
    float   adcrange_ratio;
    float   irsize_ratio;
    float   ledi_ratio;
} SI114X_CAL_S;


#define REG_UCOEF0                0x13   // ?Important
#define REG_UCOEF1                0x14   // Not used really, but here for completeness
#define REG_UCOEF2                0x15   // Not used really, but here for completeness
#define REG_UCOEF3                0x16   // Not used really, but here for completeness


// Interrupt Sample
typedef struct 
{
    unsigned int  sequence;       // sequence number
    unsigned int  timestamp;      // 16-bit Timestamp to record
    unsigned char pad;
    unsigned char irqstat;        // 8-bit irq status
    unsigned int  vis;            // VIS
    unsigned int  ir;             // IR
    unsigned int  ps1;            // PS1
    unsigned int  ps2;            // PS2
    unsigned int  ps3;            // PS3
    unsigned int  aux;            // AUX
} SI114X_IRQ_SAMPLE;



//static int WaitUntilSleep(unsigned char nonused);
//int SI14x_I2CReadNByte_GetRegData(I2C_TypeDef *i2c, uint8_t addr, uint8_t subaType, uint32_t suba, uint8_t *data, uint32_t len);
//int SI14x_WriteReg(unsigned char WriteAddr, unsigned char Data);
//int SI14x_ReadReg(unsigned char Reg, unsigned char* nonused);
//int SendCMD(unsigned char cmd);
//int Si114xBlockWrite(unsigned char  address, unsigned char  length, unsigned char  const *values);
//int Si114xBlockRead(unsigned char address, unsigned char length, unsigned char *values);
//int Si114xParamSet(unsigned char address, unsigned char value);
//int Si114xParamRead(unsigned char address);
//int si114x_set_ucoef(unsigned char ref_ucoef[], SI114X_CAL_S *si114x_cal );
//int SI14xNOP(void);
//int SI14x_PauseAll(void);
//void UV_RST(void);
//void CorrectUV(void);
//float collect(unsigned char buf[],unsigned char msb_addr,unsigned char lsb_addr,unsigned char alignment);

//int SI14xPsForce(void);
int Si114xAlsForce(void);

void UV_Init(void);

uint8_t GetUVindex(void);
uint16_t GetAmbLight(void);
void InOutdoorChange(uint8_t flg);
int Si114xPauseAll(void );

#endif
