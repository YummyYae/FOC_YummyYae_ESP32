#include <stdio.h>
#include "ap_task.h"
#include "control_task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "foc_debug.h"
#include "foc_driver_esp32.h"
#include "foc_task.h"
#include "led_pov_task.h"

void app_main(void)
{
    ESP_LOGI("app_main", "Starting FOC framework");

    if (ap_task_start() != ESP_OK) {
        ESP_LOGE("app_main", "AP task start failed");
        return;
    }

    // AP 端不走本地串口，所有 FireWaterPrintf 数据都通过 UDP 发给对端。
    foc_debug_set_output_writer(ap_task_send_udp_text);

    // 把快环函数绑定给驱动层。
    foc_driver_bind_fast_loop(foc_task_init, foc_fast_loop_step_isr);

    if (foc_driver_start() != ESP_OK) {
        ESP_LOGE("app_main", "FOC driver start failed");
        return;
    }

    if (control_task_start() != ESP_OK) {
        ESP_LOGE("app_main", "Control task start failed");
        return;
    }

    if (led_pov_task_start() != ESP_OK) {
        ESP_LOGE("app_main", "LED POV task start failed");
        return;
    }
}

