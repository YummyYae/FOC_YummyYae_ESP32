#ifndef __DATATYPE_H
#define __DATATYPE_H

// 标准库头文件包含
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*------------------------- 类型重定义 -------------------------*/
// 8位整数类型重定义
typedef   signed char 			int8;       // 有符号8位整型
typedef unsigned char 			_u8;        // 无符号8位整型（不同命名风格）
typedef unsigned char 			u8;         // 无符号8位整型
typedef unsigned char 			uint8;      // 无符号8位整型
typedef unsigned char 			byte;       // 字节类型（8位）

// 16位整数类型重定义
typedef   signed short int  int16;       // 有符号16位整型
typedef unsigned short int  uint16;      // 无符号16位整型
typedef unsigned short int  _u16;        // 无符号16位整型（不同命名风格）
typedef unsigned short int  u16;         // 无符号16位整型

// 32位整数类型重定义
typedef unsigned long int 	_u32;        // 无符号32位整型
typedef unsigned long int 	u32;        // 无符号32位整型
typedef float 							fp32;
typedef double 							fp64;

/*------------------------- 常用数学宏 -------------------------*/
#define ABS(X)  (((X)>0)?(X):-(X))         // 绝对值计算
#define MAX(a,b)  ((a)>(b)?(a):(b))        // 最大值
#define MIN(a,b)  ((a)>(b)?(b):(a))        // 最小值

/*------------------------- 对象实例化 -------------------------*/

#endif


