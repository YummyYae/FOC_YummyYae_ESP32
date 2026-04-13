#ifndef __FOC_LOG_H
#define __FOC_LOG_H

#include <stdint.h>
#include <stdio.h>
#include "usart.h" // 包含UART句柄声明

/* 
 * 根据您工程的实际下发/打印串口做修改。
 * 在前面的代码中我们看到 MX_USART2_UART_Init() 和 MX_USART3_UART_Init()。
 * 此处默认使用 USART2 (huart2) 进行打印输出，如若是usart3，可自行修改 
 */
#define LOG_UART &huart3

// 自定义向上位机带换行符的UTF-8打印
void FireWaterPrintf(const char *format, ...);

// 打印校准后的完整参数列表
void PrintCalibrationResult(void);

#endif // __FOC_LOG_H




