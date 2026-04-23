#include "foc_driver_esp32.h"

#include <math.h>
#include "driver/gptimer.h"
#include "driver/mcpwm_prelude.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "foc_control.h"
#include "foc_debug.h"
#include "foc_output.h"
#include "mt6701.h"

// 这两个函数实现在 components/tasks 组件里。
// 驱动层这里只需要调用，不把任务组件硬绑成 foc 的头文件依赖。
// 快环本体由 tasks 组件提供。
// 这里不直接链接具体实现，而是通过绑定函数指针的方式解耦组件依赖。

#if !CONFIG_MCPWM_ISR_CACHE_SAFE
#error "FOC中断路径已经切到MCPWM，请启用 CONFIG_MCPWM_ISR_CACHE_SAFE。"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FOC_CALIBRATION_NS "foc"
#define FOC_CALIBRATION_KEY "calib_v7"
#define FOC_CALIB_MAGIC 0x464F438AU
#define FOC_CALIB_VERSION 7U
#define FOC_FORCE_STARTUP_CALIBRATION 0

#define FOC_CALIB_ALIGN_VOLTAGE 3.5f
#define FOC_CALIB_ALIGN_ANGLE (1.5f * (float)M_PI)

typedef struct {
    uint32_t magic;
    uint32_t version;
    float zero_electric_angle;
    int32_t pole_pairs;
    int32_t sensor_direction;
} foc_calibration_blob_t;

static const char *TAG = "foc_driver";

// 这一层文件专门负责 ESP32-S3 平台相关的硬件初始化：
// 1. PWM 输出
// 2. GPTimer 中断
// 3. 上电校准
// 4. 标定参数存储
// 这样上层 FOC 算法文件就能尽量保持“平台无关”。
static foc_driver_config_t s_driver_config;
static bool s_driver_ready = false;
static gptimer_handle_t s_foc_timer_handle = NULL;
static TaskHandle_t s_foc_setup_task = NULL;
static int16_t s_calib_uq_q15 = 0;
static foc_fast_loop_init_fn_t s_fast_loop_init_fn = NULL;
static foc_fast_loop_step_fn_t s_fast_loop_step_fn = NULL;
static mcpwm_timer_handle_t s_pwm_timer = NULL;
static mcpwm_oper_handle_t s_pwm_operators[3];
static mcpwm_cmpr_handle_t s_pwm_comparators[3];
static mcpwm_gen_handle_t s_pwm_generators[3];

static bool IRAM_ATTR foc_pwm_on_full_cb(mcpwm_timer_handle_t timer,
                                         const mcpwm_timer_event_data_t *edata,
                                         void *user_ctx)
{
    // PWM 计数到峰值时，刚好位于中心对齐 PWM 的周期中点。
    // 这里把它作为“电流采样触发点”，让采样任务尽量在波形中段工作。
    (void)timer;
    (void)edata;
    (void)user_ctx;
    return false;
}

static bool IRAM_ATTR foc_timer_on_alarm_cb(gptimer_handle_t timer,
                                            const gptimer_alarm_event_data_t *edata,
                                            void *user_ctx)
{
    // GPTimer 中断就是整个 FOC 快环的调度器。
    // 每来一次定时中断，就执行一次“读角度 -> 算电角 -> 下发三相 PWM”。
    (void)timer;
    (void)edata;
    (void)user_ctx;
    if (s_fast_loop_step_fn != NULL) {
        s_fast_loop_step_fn();
    }
    return false;
}

static float foc_wrap_positive(float angle)
{
    while (angle >= 2.0f * (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < 0.0f) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

static float foc_average_encoder_angle(void)
{
    // 对编码器角度做单位圆平均，而不是直接做线性平均。
    // 这样可以避免角度跨过 0 / 2pi 时出现平均值错误的问题。
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    for (int i = 0; i < 100; ++i) {
        float angle = 0.0f;
        if (MT6701_ReadAngle(NULL, &angle, NULL) == ESP_OK) {
            sum_sin += sinf(angle);
            sum_cos += cosf(angle);
        }
        vTaskDelay(1);
    }

    return foc_wrap_positive(atan2f(sum_sin, sum_cos));
}

static esp_err_t foc_storage_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

static bool foc_load_calibration_from_flash(void)
{
    // 如果 flash 里已经有有效校准结果，就不必每次上电都强拖电机重新标定。
    nvs_handle_t nvs_handle;
    foc_calibration_blob_t blob = {0};
    size_t required_size = sizeof(blob);

    if (nvs_open(FOC_CALIBRATION_NS, NVS_READONLY, &nvs_handle) != ESP_OK) {
        return false;
    }

    esp_err_t ret = nvs_get_blob(nvs_handle, FOC_CALIBRATION_KEY, &blob, &required_size);
    nvs_close(nvs_handle);

    if (ret != ESP_OK || required_size != sizeof(blob)) {
        return false;
    }
    if (blob.magic != FOC_CALIB_MAGIC || blob.version != FOC_CALIB_VERSION) {
        return false;
    }

    foc_params.zero_electric_angle = blob.zero_electric_angle;
    foc_params.pole_pairs = (int)blob.pole_pairs;
    foc_params.sensor_direction = (long)blob.sensor_direction;
    foc_calibrated = 1;
    foc_started = 1;
    foc_set_voltage_target(FOC_OPEN_LOOP_UQ, FOC_OPEN_LOOP_UD);

    ESP_LOGI(TAG,
             "已加载校准参数: zero=%.4f pole_pairs=%ld direction=%ld",
             foc_params.zero_electric_angle,
             (long)foc_params.pole_pairs,
             foc_params.sensor_direction);
    foc_debug_print_calibration_result();
    return true;
}

static esp_err_t foc_save_calibration_to_flash(void)
{
    // 这里只保存重建电角所需的最小参数集：
    // 1. 电角零点
    // 2. 极对数
    // 3. 编码器方向
    // 这样结构简单，也更不容易因后续重构导致版本兼容问题。
    nvs_handle_t nvs_handle;
    const foc_calibration_blob_t blob = {
        .magic = FOC_CALIB_MAGIC,
        .version = FOC_CALIB_VERSION,
        .zero_electric_angle = foc_params.zero_electric_angle,
        .pole_pairs = foc_params.pole_pairs,
        .sensor_direction = (int32_t)foc_params.sensor_direction,
    };

    ESP_RETURN_ON_ERROR(nvs_open(FOC_CALIBRATION_NS, NVS_READWRITE, &nvs_handle), TAG, "nvs open failed");
    esp_err_t ret = nvs_set_blob(nvs_handle, FOC_CALIBRATION_KEY, &blob, sizeof(blob));
    if (ret == ESP_OK) {
        ret = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return ret;
}

static void foc_apply_calibration_vector(float electrical_angle)
{
    // 校准阶段和正常运行阶段共用同一条 q15 + SVPWM 输出路径。
    // 这样可以保证“校准时看到的输出行为”和“正式运行时的输出行为”一致，
    // 避免出现两套输出逻辑彼此不一致的问题。
    foc_set_phase_voltage_q15(s_calib_uq_q15, 0, foc_float_to_angle_u16(foc_wrap_positive(electrical_angle)));
}

static void foc_calibration_prepare(void)
{
    // 校准开始前先进入安全态：
    // 1. 暂停正式运行标志
    // 2. 清空运行目标电压
    // 3. 预先算好校准所需的固定对齐电压 q15 值
    foc_calibrated = 0;
    foc_started = 0;
    foc_set_voltage_target(0.0f, 0.0f);
    s_calib_uq_q15 = foc_voltage_to_q15(FOC_CALIB_ALIGN_VOLTAGE);
}

static float foc_calibration_lock_start(void)
{
    // 第一步：先把转子锁到一个已知电角位置上。
    // 只有先建立“已知的定子磁场方向”，后面的机械角采样才有参考意义。
    foc_apply_calibration_vector(FOC_CALIB_ALIGN_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(500) > 0 ? pdMS_TO_TICKS(500) : 50);
    return foc_average_encoder_angle();
}

static int foc_calibration_drag_cycles(float *recorded_mech_angles, float start_angle, float *total_moved_out)
{
    float prev_angle = start_angle;
    float total_moved = 0.0f;
    int cycle_count = 0;

    recorded_mech_angles[0] = start_angle;

    while (cycle_count < 99) {
        ++cycle_count;

        // 每一圈电角都拆成很多个很小的步进。
        // 这样做不是为了“算得更准”，而是为了让定子磁场平滑旋转，
        // 使转子能够被连续拖动，而不是一下跳到目标点。
        for (int step = 1; step <= 60; ++step) {
            const float electrical_angle = FOC_CALIB_ALIGN_ANGLE +
                                           (2.0f * (float)M_PI * (float)step) / 60.0f;
            foc_apply_calibration_vector(electrical_angle);
            vTaskDelay(1);
        }

        foc_apply_calibration_vector(FOC_CALIB_ALIGN_ANGLE);
        vTaskDelay(pdMS_TO_TICKS(100) > 0 ? pdMS_TO_TICKS(100) : 30);

        recorded_mech_angles[cycle_count] = foc_average_encoder_angle();
        float delta = recorded_mech_angles[cycle_count] - prev_angle;
        if (delta > (float)M_PI) delta -= 2.0f * (float)M_PI;
        if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;
        total_moved += delta;
        prev_angle = recorded_mech_angles[cycle_count];

        if (fabsf(total_moved) >= 2.0f * (float)M_PI - fabsf(delta) * 0.5f) {
            break;
        }
    }

    *total_moved_out = total_moved;
    return cycle_count;
}

static bool foc_calibration_finish(float *recorded_mech_angles, int cycle_count, float total_moved)
{
    // 校准动作结束后先撤掉输出，避免继续给电机施加锁定电流。
    foc_set_phase_voltage_q15(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200) > 0 ? pdMS_TO_TICKS(200) : 20);

    if (fabsf(total_moved) < 0.5f) {
        return false;
    }

    foc_params.sensor_direction = (total_moved > 0.0f) ? 1 : -1;
    foc_params.pole_pairs = cycle_count;

    // 利用记录下来的机械角，反推出电角零点。
    // 这里仍然采用单位圆平均，避免角度回绕带来偏差。
    float z_sin = 0.0f;
    float z_cos = 0.0f;
    for (int i = 0; i <= foc_params.pole_pairs; ++i) {
        const float zero = (float)(foc_params.sensor_direction * foc_params.pole_pairs) * recorded_mech_angles[i];
        z_sin += sinf(zero);
        z_cos += cosf(zero);
    }

    foc_params.zero_electric_angle = atan2f(z_sin, z_cos);
    if (foc_params.zero_electric_angle < 0.0f) {
        foc_params.zero_electric_angle += 2.0f * (float)M_PI;
    }

    return true;
}

static esp_err_t foc_run_startup_calibration(void)
{
    float recorded_mech_angles[100];
    float total_moved = 0.0f;
    ESP_LOGW(TAG, "Startup calibration begin: align_voltage=%.2fV", FOC_CALIB_ALIGN_VOLTAGE);

    // 上电校准流程本身并不复杂，核心就是四步：
    // 1. 先把转子锁到一个已知电角位置；
    // 2. 再让定子磁场按电角连续旋转，强制拖动转子；
    // 3. 统计“多少圈电角”对应“一圈机械角”，得到极对数；
    // 4. 用采集到的机械角反推出电角零点。
    foc_calibration_prepare();
    const float start_angle = foc_calibration_lock_start();
    const int cycle_count = foc_calibration_drag_cycles(recorded_mech_angles, start_angle, &total_moved);

    if (!foc_calibration_finish(recorded_mech_angles, cycle_count, total_moved)) {
        return ESP_FAIL;
    }

    foc_calibrated = 1;
    foc_started = 1;
    foc_set_voltage_target(FOC_OPEN_LOOP_UQ, FOC_OPEN_LOOP_UD);
    if (s_fast_loop_init_fn != NULL) {
        s_fast_loop_init_fn();
    }

    ESP_RETURN_ON_ERROR(foc_save_calibration_to_flash(), TAG, "save calibration failed");
    ESP_LOGI(TAG,
             "校准完成: zero=%.4f pole_pairs=%ld direction=%ld",
             foc_params.zero_electric_angle,
             (long)foc_params.pole_pairs,
             (long)foc_params.sensor_direction);
    foc_debug_print_calibration_result();
    return ESP_OK;
}

static esp_err_t foc_prepare_calibration(void)
{
    ESP_RETURN_ON_ERROR(foc_storage_init(), TAG, "nvs init failed");

#if FOC_FORCE_STARTUP_CALIBRATION
    ESP_LOGW(TAG, "Force startup calibration enabled, skip flash load");
    ESP_RETURN_ON_ERROR(foc_run_startup_calibration(), TAG, "startup calibration failed");
#else
    if (!foc_load_calibration_from_flash()) {
        ESP_LOGW(TAG, "未找到有效校准参数，开始执行上电校准");
        ESP_RETURN_ON_ERROR(foc_run_startup_calibration(), TAG, "startup calibration failed");
    } else {
        if (s_fast_loop_init_fn != NULL) {
            s_fast_loop_init_fn();
        }
    }
#endif

    return ESP_OK;
}

static esp_err_t foc_driver_init_pwm(const foc_driver_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG, "config is null");

    // 这里把三相 PWM 改成 MCPWM 中心对齐输出：
    // 1. 更适合电机控制；
    // 2. 更方便把采样触发点挂在 PWM 周期中点；
    // 3. 占空比更新接口支持在 ISR 内直接写比较值。
    s_driver_config = *config;
    // 这里一定要注意 ESP-IDF MCPWM 在 UP_DOWN 模式下的语义：
    // 1. 传给 `period_ticks` 的是“完整往返周期”；
    // 2. 驱动内部会再除以 2，得到真正的 `peak_ticks`；
    // 3. 比较器允许写入的上限其实是 `peak_ticks`，不是 `period_ticks`。
    // 之前这里把两者混在一起用了，所以比较值上限被算大了一倍，导致持续报越界。
    const uint32_t pwm_period_ticks = FOC_MCPWM_TIMER_RESOLUTION_HZ / s_driver_config.pwm_frequency_hz;
    const uint32_t pwm_peak_ticks = pwm_period_ticks / 2U;
    ESP_RETURN_ON_FALSE(pwm_peak_ticks > 8U, ESP_ERR_INVALID_ARG, TAG, "PWM周期太小");
    s_driver_config.pwm_max_duty = pwm_peak_ticks - 4U;

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = FOC_MCPWM_TIMER_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks = pwm_period_ticks,
        .flags.update_period_on_empty = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_config, &s_pwm_timer), TAG, "MCPWM定时器创建失败");

    mcpwm_timer_event_callbacks_t callbacks = {
        .on_full = foc_pwm_on_full_cb,
    };
    ESP_RETURN_ON_ERROR(mcpwm_timer_register_event_callbacks(s_pwm_timer, &callbacks, NULL), TAG, "PWM事件回调注册失败");

    for (int i = 0; i < 3; ++i) {
        mcpwm_operator_config_t operator_config = {
            .group_id = 0,
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&operator_config, &s_pwm_operators[i]), TAG, "MCPWM操作器创建失败");
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_pwm_operators[i], s_pwm_timer), TAG, "MCPWM操作器绑定定时器失败");

        mcpwm_comparator_config_t comparator_config = {
            .flags.update_cmp_on_tez = true,
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_pwm_operators[i], &comparator_config, &s_pwm_comparators[i]), TAG, "比较器创建失败");
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_pwm_comparators[i], pwm_peak_ticks / 2U), TAG, "比较值初始化失败");

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = (i == 0) ? s_driver_config.gpio_u : ((i == 1) ? s_driver_config.gpio_v : s_driver_config.gpio_w),
        };
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_pwm_operators[i], &generator_config, &s_pwm_generators[i]), TAG, "PWM引脚创建失败");

        // 下面这一组动作定义的是“中心对齐高脉冲”：
        // 1. 计数到 0 时先拉低；
        // 2. 上数到比较值时拉高；
        // 3. 下数回比较值时拉低。
        // 因此：
        // 比较值越小，高电平时间越长；
        // 比较值越大，高电平时间越短。
        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_actions_on_timer_event(
                s_pwm_generators[i],
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_TIMER_EVENT_ACTION_END()),
            TAG,
            "PWM定时事件动作配置失败");

        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_actions_on_compare_event(
                s_pwm_generators[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_pwm_comparators[i], MCPWM_GEN_ACTION_HIGH),
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, s_pwm_comparators[i], MCPWM_GEN_ACTION_LOW),
                MCPWM_GEN_COMPARE_EVENT_ACTION_END()),
            TAG,
            "PWM比较事件动作配置失败");
    }

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_pwm_timer), TAG, "MCPWM定时器使能失败");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_pwm_timer, MCPWM_TIMER_START_NO_STOP), TAG, "MCPWM启动失败");

    s_driver_ready = true;
    return ESP_OK;
}

static void foc_driver_core1_setup_task(void *arg)
{
    (void)arg;

    // 这里统一定义 FOC 所占用的 PWM 三相输出引脚与频率参数。
    const foc_driver_config_t driver_config = {
        .gpio_u = FOC_PHASE_U_GPIO,
        .gpio_v = FOC_PHASE_V_GPIO,
        .gpio_w = FOC_PHASE_W_GPIO,
        .pwm_frequency_hz = FOC_PWM_FREQUENCY_HZ,
        .pwm_resolution_bits = 10,
        .pwm_max_duty = 0,
    };

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    const gptimer_event_callbacks_t timer_callbacks = {.on_alarm = foc_timer_on_alarm_cb};
    const gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / FOC_CONTROL_FREQUENCY_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    // 整个 FOC 侧的硬件初始化都放到 core1 上执行：
    // 1. 初始化编码器
    // 2. 初始化 FOC 参数
    // 3. 初始化 PWM
    // 4. 读取或执行校准
    // 5. 启动 GPTimer 快环中断
    // 这样可以尽量把电机控制路径和 core0 上的其他业务隔离开。
    ESP_ERROR_CHECK(MT6701_Init());
    foc_control_init();
    ESP_ERROR_CHECK(foc_driver_init_pwm(&driver_config));

    ESP_ERROR_CHECK(foc_prepare_calibration());

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_foc_timer_handle));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_foc_timer_handle, &timer_callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_foc_timer_handle, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_foc_timer_handle));
    ESP_ERROR_CHECK(gptimer_start(s_foc_timer_handle));

    ESP_LOGI(TAG, "FOC定时中断已运行在core %d，频率=%d Hz", xPortGetCoreID(), FOC_CONTROL_FREQUENCY_HZ);

    s_foc_setup_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t foc_driver_start(void)
{
    ESP_RETURN_ON_ERROR(foc_debug_init(), TAG, "调试串口初始化失败");
    ESP_RETURN_ON_FALSE(s_fast_loop_init_fn != NULL && s_fast_loop_step_fn != NULL,
                        ESP_ERR_INVALID_STATE,
                        TAG,
                        "FOC快环尚未绑定");

    if (s_foc_timer_handle != NULL || s_foc_setup_task != NULL) {
        return ESP_OK;
    }

    BaseType_t task_ok = xTaskCreatePinnedToCore(
        foc_driver_core1_setup_task,
        "foc_drv_core1",
        8192,
        NULL,
        configMAX_PRIORITIES - 1,
        &s_foc_setup_task,
        1);

    return (task_ok == pdPASS) ? ESP_OK : ESP_FAIL;
}

void foc_driver_bind_fast_loop(foc_fast_loop_init_fn_t init_fn, foc_fast_loop_step_fn_t step_fn)
{
    s_fast_loop_init_fn = init_fn;
    s_fast_loop_step_fn = step_fn;
}

void IRAM_ATTR foc_driver_set_pwm_raw(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w)
{
    // 这一层是最终硬件写寄存器前的最后一站。
    // 这里再次做一次占空比上限保护，防止异常计算把值写爆。
    if (!s_driver_ready) {
        return;
    }

    // 再做一次边界保护，避免 ISR 里任何异常值把比较器打到非法范围。
    if (duty_u < 2U) duty_u = 2U;
    if (duty_v < 2U) duty_v = 2U;
    if (duty_w < 2U) duty_w = 2U;
    if (duty_u > s_driver_config.pwm_max_duty) duty_u = s_driver_config.pwm_max_duty;
    if (duty_v > s_driver_config.pwm_max_duty) duty_v = s_driver_config.pwm_max_duty;
    if (duty_w > s_driver_config.pwm_max_duty) duty_w = s_driver_config.pwm_max_duty;

    mcpwm_comparator_set_compare_value(s_pwm_comparators[0], duty_u);
    mcpwm_comparator_set_compare_value(s_pwm_comparators[1], duty_v);
    mcpwm_comparator_set_compare_value(s_pwm_comparators[2], duty_w);
}

const foc_driver_config_t *foc_driver_get_config(void)
{
    return s_driver_ready ? &s_driver_config : NULL;
}
