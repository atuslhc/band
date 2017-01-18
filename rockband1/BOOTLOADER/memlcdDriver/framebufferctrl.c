/**************************************************************************//**
 * @file framebuffer_control.c
 * @brief Contains functions to manipulate and retrieve the frame buffer
 * @author Energy Micro AS
 * @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2012 Energy Micro AS, http://www.energymicro.com</b>
 ******************************************************************************
 *
 * This source code is the property of Energy Micro AS. The source and compiled
 * code may only be used on Energy Micro "EFM32" microcontrollers.
 *
 * This copyright notice may not be removed from the source code nor changed.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 *****************************************************************************/

#include "framebufferctrl.h"


///* Two frame buffers are used. While transferring one buffer to the display
// * the next frame is begin drawn to the other frame buffer. */
//uint16_t frameBuffer[BUFFER_SIZE_HALFWORDS];
//
//uint16_t backgroundBuffer[2][BUFFER_HALFWORDS_PER_LINE * 15];//192*15*2/8=24*30=720

/* Two frame buffers are used. While transferring one buffer to the display
 * the next frame is begin drawn to the other frame buffer. */
uint16_t frameBuffer[BUFFER_SIZE_HALFWORDS];
//uint16_t frameBuffer2[BUFFER_SIZE_HALFWORDS];

/* Flag indicating which frame buffer is active */
//bool useBuffer1 = true;

/* Extra buffer holding the background image. Copied to the active 
 * frame buffer before the start of drawing each frame. */
//uint16_t backgroundBuffer[BUFFER_SIZE_HALFWORDS];




/** Returns the address of the active frame buffer */
uint16_t *FB_getActiveBuffer(void)
{
//  if ( useBuffer1 ) {
    return frameBuffer;
//  } else {
//    return frameBuffer;
//  }
}


//uint8_t *FB_getFirstBuffer(void)
//{
//  return (uint8_t *)frameBuffer;
//}

//uint8_t *FB_getSecondBuffer(void)
//{
//  //return (uint8_t *)frameBuffer2;
//  return (uint8_t *)frameBuffer;
//}

//uint8_t *FB_getBackgroundBuffer(void)
//{
//  return (uint8_t *)backgroundBuffer;
//}


/** Direct emWin drawing output to the background buffer */
//void FB_activateBackgroundBuffer(void)
//{
//  LCD_SetVRAMAddrEx(0, (void *)backgroundBuffer);
//}

/** Direct emWin drawing output to the frame buffer */
//void FB_activateFirstBuffer(void)
//{
//  useBuffer1 = true;
//  LCD_SetVRAMAddrEx(0, (void *)frameBuffer);
//}
//
//void FB_activateSecondBuffer(void)
//{
//  useBuffer1 = false;
//  LCD_SetVRAMAddrEx(0, (void *)frameBuffer);
//  //LCD_SetVRAMAddrEx(0, (void *)frameBuffer2);
//}

/** Copy the entire background buffer to the frame buffer */
//void FB_copyBackgroundToFrameBuffer(void)
//{
//  if ( useBuffer1 ) {
//    //memcpy(frameBuffer, backgroundBuffer, BUFFER_SIZE_BYTES);  
//  } else {
//    //memcpy(frameBuffer2, backgroundBuffer, BUFFER_SIZE_BYTES);  
//  }
//}

//void FB_flipBuffer(void)
//{
//  if ( useBuffer1 ) {
//    FB_activateSecondBuffer();
//  } else {
//    FB_activateFirstBuffer();
//  }
//}


/** Write the SPI control signals (line adresses and dummy data) to frame buffer */
void FB_writeControlSignals(uint8_t *buffer)
{
  int i,j;
  uint8_t * t = buffer;
  
  for ( i=0; i<DISPLAY_HEIGHT; i++ )
  {
    /* Clear all pixels */
    for ( j=0; j<DISPLAY_BYTES_PER_LINE; j++ ) 
    {
      *buffer++ = 0x00;
    }
    
    /* Write dummy data and address of next line */
#if 0    
    *buffer++ = 0xff;   /* Dummy data for MEMLCD */
    *buffer++ = (i+2);  /* Address of next line */
    *buffer++ = 0xff;   /* Filler data to make virtual display 160px wide */
    *buffer++ = 0xff;   /* Filler data */
#else
    *buffer++ = 0xff;   /* Dummy data for MEMLCD */
    *buffer++ = (i+2);  /* Address of next line */
    *buffer++ = 0xff;   /* Filler data to make virtual display 160px wide */
    *buffer++ = 0xff;   /* Filler data */
    *buffer++ = 0xff;
    *buffer++ = 0xff;
#endif
  }
  
//  *t = 0x01;
//  *(t+17) = 0x80;
//  
//  *(t+24) = 0xff;
//  
//  *(t+25) = 0xff;
}

/** Clears the current buffer */
void FB_clearBuffer(void)//
{
  FB_writeControlSignals((uint8_t *)FB_getActiveBuffer());
}

typedef struct _BMP_DATA
{
    uint16_t width;
    uint16_t height;
    
    uint16_t bpp;
    
    unsigned char * data;
} BMP_DATA;

bool validBMPData(const unsigned char * bmp, BMP_DATA* pBmpData)
{
	// 以 BMd\0 为起始标志，4个字节
	// 2字节宽度 + 2字节高度
	// 2字节，格式
	// 2字节，bpp
	// 2字节，palette数量
	// 2字节，透明色索引
	// 调色板，4字节/项，可能不存在
	// 像素数据，每行按字节对齐

    if (bmp == NULL || pBmpData == NULL)
        return false;
    
    if (bmp[0] != 'B' && bmp[1] != 'M' && bmp[2] != 'd' && bmp[3] != 0)
        return false;
    
    pBmpData->width = bmp[4] + (bmp[5] << 8);
    pBmpData->height = bmp[6] + (bmp[7] << 8);
    
	uint16_t palettes = bmp[12] + (bmp[13] << 8);
    pBmpData->data = (unsigned char *) bmp + 16 + palettes * 4;
    
    return true;
}


#pragma inline=forced
static unsigned char reverse_byte( unsigned char c )
{
     c = ( c & 0x55 ) << 1 | ( c & 0xAA ) >> 1;
     c = ( c & 0x33 ) << 2 | ( c & 0xCC ) >> 2;
     c = ( c & 0x0F ) << 4 | ( c & 0xF0 ) >> 4;
     return c;
}

/*
 将位图组合至framebuffer
 destX, destY 为目标位置，按像素，左上角为(0,0)
// srcX, srcY 为源位置，按像素，左上角为(0,0)
 src为源地址
 srcWidth
 srcHeight
*/
//void CompoundImage(unsigned short x, unsigned short y, unsigned short w, unsigned short h, unsigned char style)
//void CompoundImage(unsigned short destX, unsigned short destX, BYTE* src, unsigned short srcWidth, unsigned short srcHeighth)
void DrawBMP(const unsigned char * src, unsigned short x, unsigned short y)
{
	unsigned char i,j,s,l, pixelByte, c1, c2, shift_f,shift_r;
	
	BMP_DATA bmpData;
	if (!validBMPData(src, &bmpData))
		return;
	
	// 源宽度的字节数（宽度应该对齐字节）
	unsigned char wb;
	wb = bmpData.width / 8;
	if (bmpData.width % 8 != 0)
		wb++;
	
	// 当前字节对应的X坐标
	unsigned short cx;

	unsigned char * destBuffer = (unsigned char *)frameBuffer;
	
	/*
		i - 行计数；j - 列计数
		一次处理一个字节
	*/
	for(i=0; i< bmpData.height; i++)
	{
		for (j = 0; j < wb; j++)
		{
			cx = x + j * 8; // 当前字节显示的位置（按像素）
			
			s = cx / 8;   // 当前位置按字节计算

			//2011.08.30
			//s = 1 + cx / 8  ; // add 1 , the reason is the first byte of each line is standong the line address
			
			l = cx % 8;
		
			shift_f = 0xff >> (8-l);
			shift_r = 0xff << l;
			
			// 源像素，字节
			pixelByte = reverse_byte(bmpData.data[i * wb + j]);
			
			c1 = pixelByte << l;
			c2 = destBuffer[(y+i) * BUFFER_BYTES_PER_LINE + s] & shift_f; 
			
			destBuffer[(y+i) * BUFFER_BYTES_PER_LINE + s] = c1 | c2;
	
			if (l)
			{
				c1 = pixelByte >> (8-l);
				c2 = destBuffer[(y+i) * BUFFER_BYTES_PER_LINE + s + 1] & shift_r; 
				destBuffer[(y+i) * BUFFER_BYTES_PER_LINE + s + 1] = c1 | c2; 
			}
		}
	}

}
