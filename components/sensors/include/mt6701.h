#ifndef MT6701_H
#define MT6701_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MT6701_SPI_HOST SPI2_HOST
#define MT6701_MOSI_GPIO 42
#define MT6701_MISO_GPIO 41
#define MT6701_SCK_GPIO 40
#define MT6701_NSS_GPIO 39

esp_err_t MT6701_Init(void);
uint16_t MT6701_GetRawData(void);
float MT6701_GetAngleRad(void);
esp_err_t MT6701_ReadAngle(uint16_t *angle_raw, float *angle_rad, uint8_t *field_status);

#ifdef __cplusplus
}
#endif

#endif
