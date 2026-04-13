#include "flash_storage.h"
#include <string.h>

MotorStorageData_t g_motor_data;

// 检查数据有效性
static uint8_t IsValidMagic(void) {
    uint8_t magic = *(volatile uint8_t*)FLASH_STORAGE_ADDR;
    return (magic == FLASH_MAGIC_WORD);
}

// 初始化Flash存储器
void FlashStorage_Init(void) {
    memset(&g_motor_data, 0, sizeof(MotorStorageData_t));
    if (IsValidMagic()) {
        FlashStorage_ReadData();
    } else {
        // 数据无效时赋予默认值
        g_motor_data.magic = FLASH_MAGIC_WORD;
        g_motor_data.storage_status = STORAGE_STATUS_EMPTY;
    }
}

// 从Flash读取参数到全局变量
uint8_t FlashStorage_ReadData(void) {
    if (!IsValidMagic()) {
        return 0; // 读取失败，数据无效
    }
    
    // 直接内存拷贝
    memcpy(&g_motor_data, (const void*)FLASH_STORAGE_ADDR, sizeof(MotorStorageData_t));
    return 1;
}

// 将全局变量保存到Flash
uint8_t FlashStorage_SaveData(void) {
    HAL_FLASH_Unlock();
    
    // 擦除页
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Banks     = FLASH_BANK_1;
    eraseInitStruct.Page      = FLASH_STORAGE_PAGE;
    eraseInitStruct.NbPages   = 1;
    
    if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0; // 擦除失败
    }
    
    // 写入魔术字（必须作为有效性标志）
    g_motor_data.magic = FLASH_MAGIC_WORD;
    
    // 按双字(64位)写入Flash
    uint32_t address = FLASH_STORAGE_ADDR;
    uint64_t* dataPtr = (uint64_t*)&g_motor_data;
    uint32_t dataCount = sizeof(MotorStorageData_t) / sizeof(uint64_t);
    
    for (uint32_t i = 0; i < dataCount; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, dataPtr[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0; // 写入失败
        }
        address += 8;
    }
    
    // 如果结构体大小不是8字节的整数倍，处理剩余部分的写入 (由于存在结构体对齐通常是不需要的，但可以按需添加)
    uint32_t remainder = sizeof(MotorStorageData_t) % sizeof(uint64_t);
    if (remainder > 0) {
        uint64_t lastDoubleWord = 0xFFFFFFFFFFFFFFFF;
        memcpy(&lastDoubleWord, ((uint8_t*)&g_motor_data) + dataCount * 8, remainder);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, lastDoubleWord);
    }
    
    HAL_FLASH_Lock();
    return 1;
}

// 手动擦除Flash
void FlashStorage_Erase(void) {
    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t pageError = 0;
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Banks = FLASH_BANK_1;
    eraseInitStruct.Page = FLASH_STORAGE_PAGE;
    eraseInitStruct.NbPages = 1;
    HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);
    HAL_FLASH_Lock();
}
