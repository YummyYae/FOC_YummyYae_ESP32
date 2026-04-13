/**
 * @brief   Encoder MT6826S Driver C Version
 */

#include "mt6826s.h"
#include "spi.h"
#include "gpio.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static uint8_t initialized = 0;
static uint8_t enabled = 0;

void MT6826S_Init(void) {
    initialized = 1;
}

void MT6826S_Enable(void) {
    if (!initialized) return;
    uint8_t txData[2] = {0xA0, 0x03};
    HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(&hspi1, txData, 1, HAL_MAX_DELAY); // 这句必须加,不然CSn片选时MOSI还是高电平
    HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, txData, 2, HAL_MAX_DELAY);
    enabled = 1;
}

void MT6826S_Disable(void) {
    if (!initialized) return;
    HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
    enabled = 0;
}

float MT6826S_GetAngle(void) {
    uint8_t rxData[4] = {0};
    if (!enabled) return 0.0f;
    HAL_SPI_Receive(&hspi1, rxData, 4, HAL_MAX_DELAY);
    return ((rxData[0] << 7) | (rxData[1] >> 1)) / 32768.0f * 2.0f * M_PI;
}
