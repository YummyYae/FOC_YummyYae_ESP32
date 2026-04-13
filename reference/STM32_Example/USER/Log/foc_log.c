#include "foc_log.h"
#include <stdarg.h>
#include <string.h>
#include "foc_calibrate.h"
#include "flash_storage.h"

// printf重定向 (Keil V5 MDK 适用) 
// 确保您的工程在C/C++魔术棒设置中勾选了 "Use MicroLIB"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

// 避免使用半主机模式，防止未勾选MicroLIB时卡死
#pragma import(__use_no_semihosting)             
struct __FILE {
    int handle;
};
FILE __stdout;
void _sys_exit(int x) {
    x = x;
}
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(LOG_UART, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// ============== 格式化日志函数 ==============
// 1. 实现自定义的FireWaterPrintf函数
// 2. 自动在末尾添加换行符以适配上位机抓取
void FireWaterPrintf(const char *format, ...)
{
    char buf[256];
    va_list args;
    
    // 初始化变参列表
    va_start(args, format);
    // 留出 3 个字节用于 "\r\n\0" 的拼接
    vsnprintf(buf, sizeof(buf) - 3, format, args);
    va_end(args);

    // 追加回车和换行符适配多数文本接收上位机
    strcat(buf, "\r\n");

    // 阻塞式发送至预设串口，若需极强实时性可后改DMA
    HAL_UART_Transmit(LOG_UART, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}

// 打印校准获取到的一整套参数，正常字符串请使用 printf
void PrintCalibrationResult(void)
{
    printf("==================================================\r\n");
    printf("         [FOC Motor Calibration Finished]         \r\n");
    printf("==================================================\r\n");
    printf("Status           : Calibrated And Saved to Flash\r\n");
    printf("----------------    MOTOR PARA    ----------------\r\n");
    printf("Pole Pairs       : %d\r\n", foc_params.pole_pairs);
    printf("Phase Resistance : %.4f Ohm\r\n", foc_params.phase_resistance);
    printf("Zero Elec Angle  : %.4f Rad\r\n", foc_params.zero_electric_angle);
    printf("Voltage Align    : %.4f V\r\n", g_motor_data.base_calibrate_data.voltage_align);
    printf("Power Supply Vol : %.2f V\r\n", foc_params.voltage_power_supply);
    printf("Sensor Direct    : %s (%ld)\r\n",
                    foc_params.sensor_direction == 1 ? "CW" : "CCW", foc_params.sensor_direction);
    printf("----------------   CONTROL PARA   ----------------\r\n");
    printf("PID Kp           : %.4f\r\n", g_motor_data.pid_kp);
    printf("PID Ki           : %.4f\r\n", g_motor_data.pid_ki);
    printf("PID Kd           : %.4f\r\n", g_motor_data.pid_kd);
    printf("Limit Max Vol    : %.2f\r\n", g_motor_data.limit_max);
    printf("Limit Min Vol    : %.2f\r\n", g_motor_data.limit_min);
    printf("Zero Pos (Mech)  : %.4f Rad\r\n", g_motor_data.zero_pos);
    printf("----------------    ADC SENSOR    ----------------\r\n");
    printf("Curr Offset U    : %.4f V\r\n", foc_params.iu_offset);
    printf("Curr Offset V    : %.4f V\r\n", foc_params.iv_offset);
    printf("Curr Offset W    : %.4f V\r\n", foc_params.iw_offset);
    printf("==================================================\r\n");
}


