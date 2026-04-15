#include <stdio.h>
#include "control_task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "foc_driver_esp32.h"
#include "foc_task.h"

void app_main(void)
{
    ESP_LOGI("app_main", "Starting FOC framework");

    // 先把快环函数绑定给驱动层。
    // 这样 foc 组件不需要直接依赖 tasks 组件，也能在中断里调用快环。
    foc_driver_bind_fast_loop(foc_task_init, foc_fast_loop_step_isr);

    if (foc_driver_start() != ESP_OK) {
        ESP_LOGE("app_main", "FOC driver start failed");
        return;
    }

    if (control_task_start() != ESP_OK) {
        ESP_LOGE("app_main", "Control task start failed");
        return;
    }
}
