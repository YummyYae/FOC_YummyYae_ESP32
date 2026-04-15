#include "mt6701.h"

#include <math.h>
#include "esp_check.h"
#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MT6701_SPI_CLOCK_HZ 1000000

static const char *TAG = "mt6701";
static spi_device_handle_t s_mt6701_handle = NULL;

esp_err_t MT6701_Init(void)
{
    if (s_mt6701_handle != NULL) {
        return ESP_OK;
    }

    const spi_bus_config_t bus_config = {
        .mosi_io_num = MT6701_MOSI_GPIO,
        .miso_io_num = MT6701_MISO_GPIO,
        .sclk_io_num = MT6701_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };

    const spi_device_interface_config_t dev_config = {
        .clock_speed_hz = MT6701_SPI_CLOCK_HZ,
        .mode = 1,
        .spics_io_num = MT6701_NSS_GPIO,
        .queue_size = 1,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
    };

    esp_err_t ret = spi_bus_initialize(MT6701_SPI_HOST, &bus_config, SPI_DMA_DISABLED);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    ret = spi_bus_add_device(MT6701_SPI_HOST, &dev_config, &s_mt6701_handle);
    if (ret == ESP_ERR_INVALID_STATE) {
        return ESP_OK;
    }
    ESP_RETURN_ON_ERROR(ret, TAG, "add device failed");

    ESP_LOGI(TAG, "MT6701 initialized on SPI host %d", MT6701_SPI_HOST);
    return ESP_OK;
}

esp_err_t MT6701_ReadAngle(uint16_t *angle_raw, float *angle_rad, uint8_t *field_status)
{
    ESP_RETURN_ON_FALSE(s_mt6701_handle != NULL, ESP_ERR_INVALID_STATE, TAG, "device not initialized");

    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 24,
        .rxlength = 24,
    };
    trans.tx_data[0] = 0xFF;
    trans.tx_data[1] = 0xFF;
    trans.tx_data[2] = 0xFF;

    esp_err_t ret = spi_device_polling_transmit(s_mt6701_handle, &trans);
    ESP_RETURN_ON_ERROR(ret, TAG, "spi read failed");

    const uint8_t rx0 = trans.rx_data[0];
    const uint8_t rx1 = trans.rx_data[1];
    const uint8_t rx2 = trans.rx_data[2];
    const uint16_t raw = (uint16_t)(((uint16_t)rx0 << 6) | (rx1 >> 2));
    const uint8_t status = (uint8_t)(((rx1 & 0x03U) << 2) | (rx2 >> 6));

    if (angle_raw != NULL) {
        *angle_raw = raw & 0x3FFFU;
    }
    if (angle_rad != NULL) {
        *angle_rad = ((float)(raw & 0x3FFFU) * (2.0f * (float)M_PI)) / 16384.0f;
    }
    if (field_status != NULL) {
        *field_status = status;
    }

    return ESP_OK;
}
