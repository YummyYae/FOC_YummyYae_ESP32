#include "led_pov_task.h"

#include <stdbool.h>
#include <stdint.h>
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "foc_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tlc5947.h"

#define LED_POV_CORE 1
#define LED_POV_TASK_STACK_SIZE 4096
#define LED_POV_TASK_PRIORITY 5

#define LED_POV_TIMER_RESOLUTION_HZ 1920000
#define LED_POV_UPDATE_HZ 1920
#define LED_POV_TIMER_ALARM_COUNT (LED_POV_TIMER_RESOLUTION_HZ / LED_POV_UPDATE_HZ)

static const char *TAG = "led_pov_task";
static TaskHandle_t s_led_pov_task = NULL;
static gptimer_handle_t s_led_pov_timer = NULL;

static inline uint16_t led_pov_angle_to_column(uint16_t shaft_angle_u16)
{
    return (uint16_t)(((uint32_t)shaft_angle_u16 * TLC5947_POV_COLUMNS) >> 16);
}

static bool IRAM_ATTR led_pov_timer_on_alarm(gptimer_handle_t timer,
                                             const gptimer_alarm_event_data_t *edata,
                                             void *user_ctx)
{
    (void)timer;
    (void)edata;
    TaskHandle_t task = (TaskHandle_t)user_ctx;
    if (task == NULL) {
        return false;
    }

    BaseType_t high_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &high_task_woken);
    return (high_task_woken == pdTRUE);
}

static esp_err_t led_pov_timer_start_on_core1(void)
{
    const gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = LED_POV_TIMER_RESOLUTION_HZ,
    };
    ESP_RETURN_ON_ERROR(gptimer_new_timer(&timer_cfg, &s_led_pov_timer), TAG, "new timer failed");

    const gptimer_event_callbacks_t cbs = {
        .on_alarm = led_pov_timer_on_alarm,
    };
    ESP_RETURN_ON_ERROR(gptimer_register_event_callbacks(s_led_pov_timer, &cbs, (void *)s_led_pov_task),
                        TAG, "register callbacks failed");

    const gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = LED_POV_TIMER_ALARM_COUNT,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_RETURN_ON_ERROR(gptimer_set_alarm_action(s_led_pov_timer, &alarm_cfg), TAG, "set alarm failed");
    ESP_RETURN_ON_ERROR(gptimer_enable(s_led_pov_timer), TAG, "timer enable failed");
    ESP_RETURN_ON_ERROR(gptimer_start(s_led_pov_timer), TAG, "timer start failed");

    ESP_LOGI(TAG, "POV timer started on core%d: %d Hz", LED_POV_CORE, LED_POV_UPDATE_HZ);
    return ESP_OK;
}

static void led_pov_task_entry(void *arg)
{
    (void)arg;

    ESP_ERROR_CHECK(tlc5947_init());
    tlc5947_fill_pov_test_pattern();

    ESP_ERROR_CHECK(led_pov_timer_start_on_core1());

    while (true) {
        // 由 1920Hz 硬件定时器中断唤醒，保证列刷新节拍稳定。
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        const uint16_t shaft_angle_u16 = foc_params.shaft_angle_u16;
        uint16_t column = led_pov_angle_to_column(shaft_angle_u16);
        if (column >= TLC5947_POV_COLUMNS) {
            column = (TLC5947_POV_COLUMNS - 1U);
        }
        (void)tlc5947_update_from_pov_column(column);
    }
}

esp_err_t led_pov_task_start(void)
{
    if (s_led_pov_task != NULL) {
        return ESP_OK;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(led_pov_task_entry,
                                            "led_pov_core1",
                                            LED_POV_TASK_STACK_SIZE,
                                            NULL,
                                            LED_POV_TASK_PRIORITY,
                                            &s_led_pov_task,
                                            LED_POV_CORE);

    ESP_RETURN_ON_FALSE(ok == pdPASS, ESP_FAIL, TAG, "task create failed");
    ESP_LOGI(TAG, "POV task created on core%d", LED_POV_CORE);
    return ESP_OK;
}

