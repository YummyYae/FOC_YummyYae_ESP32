#include <stdio.h>
#include "esp_log.h"
#include "esp_system.h"
#include "foc_driver_esp32.h"
#include "foc_task.h"

void app_main(void)
{
    ESP_LOGI("app_main", "Starting FOC framework");

    if (foc_driver_start(foc_fast_loop_step_isr) != ESP_OK) {
        ESP_LOGE("app_main", "FOC driver start failed");
        return;
    }
}
