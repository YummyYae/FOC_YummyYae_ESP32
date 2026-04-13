#include "foc_driver_esp32.h"

#include <math.h>
#include <string.h>
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "BSPDriver/MT6701.h"
#include "foc_calibrate.h"
#include "foc_lib.h"
#include "foc_task.h"

#if !CONFIG_LEDC_CTRL_FUNC_IN_IRAM
#error "FOC ISR path updates LEDC in interrupt context. Please enable CONFIG_LEDC_CTRL_FUNC_IN_IRAM."
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define FOC_CALIBRATION_NS "foc"
#define FOC_CALIBRATION_KEY "calib_v4"
#define FOC_CALIB_MAGIC 0x464F4335U

#define FOC_CALIB_ALIGN_VOLTAGE 2.0f
#define FOC_CALIB_ALIGN_ANGLE (1.5f * (float)M_PI)
#define FOC_CALIB_ALIGN_SETTLE_MS 800U
#define FOC_CALIB_CYCLE_SETTLE_MS 120U
#define FOC_CALIB_ZERO_SETTLE_MS 250U
#define FOC_CALIB_STEPS_PER_ELEC_REV 256U
#define FOC_CALIB_STEP_DELAY_MS 2U
#define FOC_CALIB_MAX_POLE_PAIRS 32
#define FOC_CALIB_MIN_MECH_REV_RATIO 0.80f

typedef struct {
    uint32_t magic;
    uint32_t version;
    float zero_electric_angle;
    int32_t pole_pairs;
    int32_t sensor_direction;
} foc_calibration_blob_t;

static const char *TAG = "foc_driver";

static foc_driver_config_t s_driver_config;
static bool s_driver_ready = false;
static gptimer_handle_t s_foc_timer_handle = NULL;
static TaskHandle_t s_foc_setup_task = NULL;
static foc_isr_callback_t s_foc_isr_callback = NULL;
static int16_t s_calib_uq_q15 = 0;
static int16_t s_calib_ud_q15 = 0;

static float IRAM_ATTR foc_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t IRAM_ATTR foc_duty_to_raw(float duty)
{
    const float clamped = foc_clampf(duty, 0.0f, 1.0f);
    return (uint32_t)lroundf(clamped * (float)s_driver_config.pwm_max_duty);
}

static bool IRAM_ATTR foc_timer_on_alarm_cb(gptimer_handle_t timer,
                                            const gptimer_alarm_event_data_t *edata,
                                            void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    if (s_foc_isr_callback != NULL) {
        s_foc_isr_callback();
    }

    return false;
}

static float foc_wrap_angle_positive(float angle)
{
    while (angle >= 2.0f * (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < 0.0f) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

static float foc_wrap_angle_signed(float angle)
{
    while (angle > (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < -(float)M_PI) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

static float foc_average_encoder_angle(uint32_t samples, uint32_t delay_ms)
{
    float sum_sin = 0.0f;
    float sum_cos = 0.0f;

    for (uint32_t i = 0; i < samples; ++i) {
        const float angle = MT6701_GetAngleRad();
        sum_sin += sinf(angle);
        sum_cos += cosf(angle);
        if (delay_ms > 0U) {
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }

    return foc_wrap_angle_positive(atan2f(sum_sin, sum_cos));
}

static esp_err_t foc_log_encoder_snapshot(const char *label)
{
    float min_angle = 1000.0f;
    float max_angle = -1000.0f;
    float prev = 0.0f;
    float acc_delta = 0.0f;

    for (uint32_t i = 0; i < 32U; ++i) {
        float angle = 0.0f;
        uint16_t raw = 0U;
        uint8_t status = 0U;
        ESP_RETURN_ON_ERROR(MT6701_ReadAngle(&raw, &angle, &status), TAG, "mt6701 read failed");

        if (i > 0U) {
            acc_delta += fabsf(foc_wrap_angle_signed(angle - prev));
        }
        prev = angle;

        if (angle < min_angle) {
            min_angle = angle;
        }
        if (angle > max_angle) {
            max_angle = angle;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
        if (i == 15U) {
            ESP_LOGI(TAG,
                     "%s encoder snapshot: raw=%u angle=%.4f status=0x%02x jitter=%.5f",
                     label,
                     raw,
                     angle,
                     status,
                     acc_delta);
        }
    }

    ESP_LOGI(TAG, "%s encoder range: min=%.4f max=%.4f", label, min_angle, max_angle);
    return ESP_OK;
}

static void foc_apply_calibration_vector(float electrical_angle)
{
    // Calibration uses the same q15 -> SVPWM path as the fast FOC loop.
    // This avoids differences between "calibration output" and "runtime output".
    foc_set_phase_voltage_q15(s_calib_uq_q15,
                              s_calib_ud_q15,
                              foc_float_to_angle_u16(foc_wrap_angle_positive(electrical_angle)));
}

static void foc_release_motor_after_calibration(void)
{
    setPhaseVoltage(0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(FOC_CALIB_ZERO_SETTLE_MS));
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
    if (blob.magic != FOC_CALIB_MAGIC || blob.version != 4U) {
        return false;
    }

    foc_params.zero_electric_angle = blob.zero_electric_angle;
    foc_params.pole_pairs = (int)blob.pole_pairs;
    foc_params.sensor_direction = (long)blob.sensor_direction;
    foc_calibrated = 1;
    foc_started = 1;
    setTargetVotage(FOC_OPEN_LOOP_UQ, FOC_OPEN_LOOP_UD);

    ESP_LOGI(TAG,
             "Calibration loaded from flash: zero=%.4f pole_pairs=%ld direction=%ld",
             foc_params.zero_electric_angle,
             (long)foc_params.pole_pairs,
             foc_params.sensor_direction);
    return true;
}

static esp_err_t foc_save_calibration_to_flash(void)
{
    nvs_handle_t nvs_handle;
    const foc_calibration_blob_t blob = {
        .magic = FOC_CALIB_MAGIC,
        .version = 4U,
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

static esp_err_t foc_run_startup_calibration(void)
{
    if (!foc_enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    foc_calibrated = 0;
    foc_started = 0;
    foc_params.phase_resistance = 0.0f; // 暂时略过相电阻测量
    const float voltage_align = 1.5f;   // 固定校准电压设为 1.5V
    
    // 设置快环初始输出为0
    setTargetVotage(0.0f, 0.0f);
    
    // 强制修改底层投递的对齐电压向量 (Uq = 1.5V, Ud = 0)
    s_calib_uq_q15 = foc_voltage_to_q15(voltage_align);
    s_calib_ud_q15 = 0;

    ESP_LOGI(TAG, "Calibration: Aligning motor to 1.5*PI (270 degrees)...");

    // 1. 强制锁定初始位置 (电角度 1.5 * PI 即 _3PI_2)
    foc_apply_calibration_vector(1.5f * (float)M_PI);
    // 延时 500ms 等待电机完全稳定 (使用安全的 pdMS_TO_TICKS 兜底机制应对 100Hz 系统的 0-tick 截断)
    vTaskDelay(pdMS_TO_TICKS(500) > 0 ? pdMS_TO_TICKS(500) : 50);

    // 2. 采样当前的机械角度作为起始角度
    float sz = 0.0f, cz = 0.0f;
    for (int i = 0; i < 100; ++i) {
        float a = MT6701_GetAngleRad();
        sz += sinf(a);
        cz += cosf(a);
        vTaskDelay(1); // 强制让出 1 个 tick (如果是 100Hz 则为 10ms)，保证读数充分刷新且消除高频震动
    }
    float start_angle = atan2f(sz, cz);
    if (start_angle < 0.0f) start_angle += 2.0f * (float)M_PI;
    
    float recorded_mech_angles[100];
    recorded_mech_angles[0] = start_angle;
    
    float prev_angle = start_angle;
    float total_moved = 0.0f;
    int cycle_count = 0;

    ESP_LOGI(TAG, "Calibration: Dragging motor until 1 full mechanical rev...");

    // 3. 强制正向循环拖动电机，直到累积完成一整个机械圈的位移除此以外测定极对数！
    // 最多尝试 99 个周期作为死区保护
    while (cycle_count < 99) {
        cycle_count++;

        // 正向拖动刚好一个电周期，分 125 步，每步强制保持至少 1 调度的时槽
        for (int j = 1; j <= 125; ++j) {
            float angle = 1.5f * (float)M_PI + (2.0f * (float)M_PI * (float)j) / 125.0f;
            foc_apply_calibration_vector(angle);
            // 绝不要使用 pdMS_TO_TICKS(1) 因为在 100Hz 下会变0，直接用 1 tick 最保险！
            vTaskDelay(1);
        }

        // 步骤完成，退回保证完美锁定在这个周期的起末电角度位 (1.5 * PI)
        foc_apply_calibration_vector(1.5f * (float)M_PI);
        vTaskDelay(pdMS_TO_TICKS(300) > 0 ? pdMS_TO_TICKS(300) : 30);

        // 重新平滑采样当前机械角
        sz = 0.0f; cz = 0.0f;
        for (int i = 0; i < 100; ++i) {
            float a = MT6701_GetAngleRad();
            sz += sinf(a);
            cz += cosf(a);
            vTaskDelay(1); 
        }
        float current_angle = atan2f(sz, cz);
        if (current_angle < 0.0f) current_angle += 2.0f * (float)M_PI;

        recorded_mech_angles[cycle_count] = current_angle;

        // 计算差值 delta 以判定位移
        float delta = current_angle - prev_angle;
        if (delta > (float)M_PI) delta -= 2.0f * (float)M_PI;
        else if (delta < -(float)M_PI) delta += 2.0f * (float)M_PI;

        total_moved += delta;
        prev_angle = current_angle;

        ESP_LOGI(TAG, "calib cycle %d: mech=%.4f delta=%.4f total=%.4f", 
                 cycle_count, current_angle, delta, total_moved);

        // 检测是否达到了机械 1 圈的容宽界限 (2 * PI)
        if (fabsf(total_moved) >= 2.0f * (float)M_PI - fabsf(delta) * 0.5f) {
            break;
        }
    }

    // 释放电机 (输出电压赋0)
    setTargetVotage(0.0f, 0.0f);
    foc_set_phase_voltage_q15(0, 0, 0);
    vTaskDelay(pdMS_TO_TICKS(200) > 0 ? pdMS_TO_TICKS(200) : 20);

    if (fabsf(total_moved) < 0.5f) {
        ESP_LOGE(TAG, "Calibration Warning: Motor blocked or not moving properly!");
        return ESP_FAIL;
    } else {
        foc_params.sensor_direction = (total_moved > 0.0f) ? 1 : -1;
        foc_params.pole_pairs = cycle_count;
    }

    // 4. 计算完美的定子电角度初始偏差补偿点 (零偏)
    float z_sz = 0.0f, z_cz = 0.0f;
    for (int i = 0; i <= foc_params.pole_pairs; ++i) {
        float current_zero_offset = (float)(foc_params.sensor_direction * foc_params.pole_pairs) * recorded_mech_angles[i];
        z_sz += sinf(current_zero_offset);
        z_cz += cosf(current_zero_offset);
    }

    foc_params.zero_electric_angle = atan2f(z_sz, z_cz);
    if(foc_params.zero_electric_angle < 0.0f) {
        foc_params.zero_electric_angle += 2.0f * (float)M_PI;
    }

    foc_calibrated = 1;
    foc_started = 1;
    // 恢复默认的开环运行电压用于后续调试
    setTargetVotage(FOC_OPEN_LOOP_UQ, FOC_OPEN_LOOP_UD);
    foc_task_init();

    ESP_RETURN_ON_ERROR(foc_save_calibration_to_flash(), TAG, "save calibration failed");

    ESP_LOGI(TAG, "Calibration done: zero=%.4f pole_pairs=%ld direction=%ld",
             foc_params.zero_electric_angle,
             (long)foc_params.pole_pairs,
             (long)foc_params.sensor_direction);

    return ESP_OK;
}

static esp_err_t foc_prepare_calibration(void)
{
    ESP_RETURN_ON_ERROR(foc_storage_init(), TAG, "nvs init failed");

    if (!foc_load_calibration_from_flash()) {
        ESP_LOGW(TAG, "No valid calibration found, running startup calibration");
        ESP_RETURN_ON_ERROR(foc_run_startup_calibration(), TAG, "startup calibration failed");
    } else {
        foc_started = 1;
        foc_task_init();
    }

    return ESP_OK;
}

esp_err_t foc_driver_init_pwm(const foc_driver_config_t *config)
{
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, "foc_driver", "config is null");

    s_driver_config = *config;
    if (s_driver_config.pwm_resolution_bits == 0U) {
        s_driver_config.pwm_resolution_bits = 10U;
    }
    s_driver_config.pwm_max_duty = (1U << s_driver_config.pwm_resolution_bits) - 1U;

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)s_driver_config.pwm_resolution_bits,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = (uint32_t)s_driver_config.pwm_frequency_hz,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_config), "foc_driver", "timer init failed");

    const ledc_channel_config_t channel_configs[] = {
        {
            .gpio_num = s_driver_config.gpio_u,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_0,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = s_driver_config.gpio_v,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        },
        {
            .gpio_num = s_driver_config.gpio_w,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = LEDC_CHANNEL_2,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        },
    };

    for (size_t i = 0; i < sizeof(channel_configs) / sizeof(channel_configs[0]); ++i) {
        ESP_RETURN_ON_ERROR(ledc_channel_config(&channel_configs[i]), "foc_driver", "channel init failed");
    }

    s_driver_ready = true;
    return ESP_OK;
}

static void foc_driver_core1_setup_task(void *arg)
{
    (void)arg;

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

    const gptimer_event_callbacks_t timer_callbacks = {
        .on_alarm = foc_timer_on_alarm_cb,
    };

    const gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000000 / FOC_CONTROL_FREQUENCY_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    ESP_ERROR_CHECK(MT6701_Init());
    FOC_Calibrate_Init();
    ESP_ERROR_CHECK(foc_driver_init_pwm(&driver_config));
    ESP_ERROR_CHECK(foc_prepare_calibration());

    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &s_foc_timer_handle));
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_foc_timer_handle, &timer_callbacks, NULL));
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_foc_timer_handle, &alarm_config));
    ESP_ERROR_CHECK(gptimer_enable(s_foc_timer_handle));
    ESP_ERROR_CHECK(gptimer_start(s_foc_timer_handle));

    ESP_LOGI(TAG, "FOC timer ISR is running on core %d at %d Hz", xPortGetCoreID(), FOC_CONTROL_FREQUENCY_HZ);

    s_foc_setup_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t foc_driver_start(foc_isr_callback_t foc_isr_callback)
{
    if (s_foc_timer_handle != NULL || s_foc_setup_task != NULL) {
        return ESP_OK;
    }

    s_foc_isr_callback = foc_isr_callback;

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

void IRAM_ATTR foc_driver_set_pwm_raw(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w)
{
    if (!s_driver_ready) {
        return;
    }

    if (duty_u > s_driver_config.pwm_max_duty) duty_u = s_driver_config.pwm_max_duty;
    if (duty_v > s_driver_config.pwm_max_duty) duty_v = s_driver_config.pwm_max_duty;
    if (duty_w > s_driver_config.pwm_max_duty) duty_w = s_driver_config.pwm_max_duty;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty_u);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty_v);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, duty_w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

void IRAM_ATTR foc_driver_set_pwm(float duty_u, float duty_v, float duty_w)
{
    foc_driver_set_pwm_raw(foc_duty_to_raw(duty_u),
                           foc_duty_to_raw(duty_v),
                           foc_duty_to_raw(duty_w));
}

void IRAM_ATTR foc_driver_output_disable(void)
{
    foc_driver_set_pwm_raw(0U, 0U, 0U);
}

bool foc_driver_is_ready(void)
{
    return s_driver_ready;
}

const foc_driver_config_t *foc_driver_get_config(void)
{
    return s_driver_ready ? &s_driver_config : NULL;
}
