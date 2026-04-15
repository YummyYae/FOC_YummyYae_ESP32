#include "control_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "foc_control.h"
#include "foc_debug.h"
#include "pid.h"

#define CONTROL_TASK_HZ 500
#define CONTROL_TASK_STACK_SIZE 4096
#define CONTROL_TASK_PRIORITY 4
#define CONTROL_SPEED_LPF_ALPHA 0.20f

// 启动积分抑制参数：
// 1. 启动后一段窗口时间内，若速度仍很低，则冻结 I 项
// 2. 速度起来后自动释放积分
#define CONTROL_PID_INTEGRAL_HOLD_MS 600
#define CONTROL_PID_RELEASE_RPM 80.0f

static const char *TAG = "control_task";
static TaskHandle_t s_control_task = NULL;
static foc_pid_t s_speed_pid;
static bool s_speed_pid_ready = false;

static float control_angle_u16_to_rad(uint16_t angle_u16)
{
    return ((float)angle_u16 * TWO_PI) / (float)FOC_ANGLE_FULL_TURN;
}

static float control_compute_rpm(uint16_t current_angle_u16, uint16_t previous_angle_u16, int64_t delta_time_us)
{
    // 用真实时间差测速，避免任务调度抖动造成系统误差。
    if (delta_time_us <= 0) {
        return 0.0f;
    }

    const int16_t delta_u16 = (int16_t)(current_angle_u16 - previous_angle_u16);
    const float delta_revolution = (float)delta_u16 / (float)FOC_ANGLE_FULL_TURN;
    const float delta_time_s = (float)delta_time_us / 1000000.0f;
    return (delta_revolution / delta_time_s) * 60.0f;
}

static void control_speed_loop_run(bool clear_integral)
{
    // 如果要求清空积分，则将 iout 归零
    if (clear_integral) {
        s_speed_pid.iout = 0.0f;
    }

    const float uq_target = pid_calc(&s_speed_pid, foc_params.mechanical_rpm, foc_params.target_mechanical_rpm);

    // 当前还没有电流环，速度环先直接输出到 Uq 电压目标。
    foc_set_voltage_target(uq_target, 0.0f);
}

static void control_position_loop_placeholder(void)
{
    // 位置环预留入口。
}

static void control_publish_realtime_frame(void)
{
    const float shaft_angle = control_angle_u16_to_rad(foc_params.shaft_angle_u16);
    const float electrical_angle = control_angle_u16_to_rad(foc_params.electrical_angle_u16);

    FireWaterPrintf("%.3f,%.3f,%.3f,%.3f,%.3f",
                    shaft_angle,
                    electrical_angle,
                    foc_params.mechanical_angle_unwrapped,
                    foc_params.mechanical_rpm,
                    foc_params.uq);
}

static void control_task_entry(void *arg)
{
    (void)arg;

    TickType_t task_period_ticks = pdMS_TO_TICKS(1000 / CONTROL_TASK_HZ);
    if (task_period_ticks == 0) {
        task_period_ticks = 1;
    }

    uint16_t previous_shaft_angle_u16 = foc_params.shaft_angle_u16;
    int64_t previous_timestamp_us = esp_timer_get_time();
    int64_t actual_startup_timestamp_us = 0;
    bool has_started = false;
    float filtered_rpm = 0.0f;
    float unwrapped_mechanical_angle = 0.0f;
    TickType_t last_wake_time = xTaskGetTickCount();
    bool integral_release_logged = false;

    foc_params.mechanical_angle_unwrapped = 0.0f;
    foc_params.mechanical_rpm = 0.0f;

    if (!s_speed_pid_ready) {
        foc_speed_pid_init(&s_speed_pid);
        s_speed_pid_ready = true;
    } else {
        pid_clear(&s_speed_pid);
    }

    // 用当前 Uq 作为 PID 初值，切入闭环更平滑。
    s_speed_pid.out = foc_params.uq;

    FireWaterPrintf("shaft_angle,electrical_angle,shaft_angle_unwrapped,rpm,uq");

    while (true) {
        const int64_t current_timestamp_us = esp_timer_get_time();
        
        // 捕获真实的闭环启动时刻 (比如校准结束刚开始能转)
        if (!has_started && foc_params.target_mechanical_rpm != 0.0f) {
            actual_startup_timestamp_us = current_timestamp_us;
            has_started = true;
        } else if (foc_params.target_mechanical_rpm == 0.0f) {
            // 如果目标速度归零，则重置启动标记
            has_started = false;
            integral_release_logged = false;
        }

        const uint16_t current_shaft_angle_u16 = foc_params.shaft_angle_u16;
        const int16_t delta_angle_u16 = (int16_t)(current_shaft_angle_u16 - previous_shaft_angle_u16);
        const int64_t delta_time_us = current_timestamp_us - previous_timestamp_us;
        const float instant_rpm = control_compute_rpm(current_shaft_angle_u16,
                                                      previous_shaft_angle_u16,
                                                      delta_time_us);

        // 展开机械角可直接反映真实累计圈数。
        unwrapped_mechanical_angle += ((float)delta_angle_u16 * TWO_PI) / (float)FOC_ANGLE_FULL_TURN;

        // 先低通再送给速度环。
        filtered_rpm += CONTROL_SPEED_LPF_ALPHA * (instant_rpm - filtered_rpm);
        foc_params.mechanical_angle_unwrapped = unwrapped_mechanical_angle;
        foc_params.mechanical_rpm = filtered_rpm;

        const int64_t elapsed_ms = has_started ? ((current_timestamp_us - actual_startup_timestamp_us) / 1000) : 0;
        // 如果刚启动不久且速度不到释放阈值，或者由于目标为0刚被复位，则清空积分，也就是“hold”
        const bool clear_integral = (!has_started) || 
                                    (elapsed_ms < CONTROL_PID_INTEGRAL_HOLD_MS && 
                                     fabsf(filtered_rpm) < CONTROL_PID_RELEASE_RPM);

        if (!clear_integral && !integral_release_logged && has_started) {
            ESP_LOGI(TAG, "Speed integral released: elapsed_ms=%ld rpm=%.1f",
                     (long)elapsed_ms, filtered_rpm);
            integral_release_logged = true;
        }

        control_speed_loop_run(clear_integral);
        control_position_loop_placeholder();
        control_publish_realtime_frame();

        previous_timestamp_us = current_timestamp_us;
        previous_shaft_angle_u16 = current_shaft_angle_u16;
        vTaskDelayUntil(&last_wake_time, task_period_ticks);
    }
}

esp_err_t control_task_start(void)
{
    if (s_control_task != NULL) {
        return ESP_OK;
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(control_task_entry,
                                                 "foc_ctrl_core0",
                                                 CONTROL_TASK_STACK_SIZE,
                                                 NULL,
                                                 CONTROL_TASK_PRIORITY,
                                                 &s_control_task,
                                                 0);

    ESP_RETURN_ON_FALSE(task_ok == pdPASS, ESP_FAIL, TAG, "Control task create failed");
    ESP_LOGI(TAG, "500Hz control task started on core0, target_rpm=%.1f", foc_params.target_mechanical_rpm);
    return ESP_OK;
}
