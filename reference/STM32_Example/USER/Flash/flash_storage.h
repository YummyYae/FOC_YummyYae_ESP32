#ifndef __FLASH_STORAGE_H
#define __FLASH_STORAGE_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

// 存储在最后一页，以STM32G431CBT6(128KB)为例，页大小2KB，最后一页是第63页，地址0x0801F800
#define FLASH_STORAGE_ADDR     0x0801F800
#define FLASH_STORAGE_PAGE     63
#define FLASH_MAGIC_WORD       0x5D // 魔术字递增为0x5D，增加了极对数并移除了齿槽转矩表

typedef enum {
    STORAGE_STATUS_EMPTY = 0x00,
    STORAGE_STATUS_CALIBRATED = 0x01,
    STORAGE_STATUS_SAVED = 0x02
} StorageStatus_e;

typedef struct {
    float phase_resistance;
    float voltage_align;
    uint8_t encoder_direction;
    float zero_electric_angle;
    int32_t pole_pairs;
} BaseCalibrateData_t;

// 需要存储的参数结构体
typedef struct {
    uint8_t magic;
    uint8_t storage_status;
    uint8_t plug_detected;
    uint8_t reserved; // 字节对齐

    float zero_pos;
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float limit_max;
    float limit_min;

    BaseCalibrateData_t base_calibrate_data;
} MotorStorageData_t;

// 外部引用的全局参数
extern MotorStorageData_t g_motor_data;

void FlashStorage_Init(void);
uint8_t FlashStorage_ReadData(void);
uint8_t FlashStorage_SaveData(void);
void FlashStorage_Erase(void);

#endif // __FLASH_STORAGE_H
