#ifndef TYPE_DEFS_H
#define TYPE_DEFS_H

// -------------------------------------------
// 数据类型定义
//typedef unsigned char BOOLEAN;
typedef unsigned char BYTE;
typedef unsigned char UINT8;

//typedef unsigned char UCHAR;
//typedef signed char CHAR;
typedef signed char INT8;

typedef unsigned short int UINT16;
typedef signed short int INT16;

typedef unsigned int UINT;

typedef unsigned long int UINT32;
typedef signed long int INT32;

typedef unsigned short  USHORT;


// ====================================================
// 常量
#undef      OK
#undef      ERROR
#define     OK      ((INT32)0)
#define     ERROR   ((INT32)-1)

//#undef      TRUE
//#undef      FALSE
//#define     TRUE    ((BOOLEAN)1)
//#define     FALSE   ((BOOLEAN)0)

#define ON 	1
#define OFF 0
// -------------------------------------------

#endif