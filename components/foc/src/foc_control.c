#include "foc_control.h"

#include <math.h>
#include "esp_attr.h"
#include "esp_log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "foc_core";

// 这一份参数结构体是整个 FOC 运行链共享的核心状态。
// 上电初始化、启动校准、以及后续 10kHz 中断快环都会访问它。
// 目前这里只保留“真实运行路径”会用到的字段，已经删除了大量旧工程里暂时不用的成员。
FOC_Parameters foc_params;

// 下面这三个状态位专门用于控制输出安全。
// 只有在驱动准备完成、校准完成、并且显式允许输出之后，中断才会真正下发 PWM。
uint8_t foc_enabled = 1;
uint8_t foc_started = 0;
uint8_t foc_calibrated = 0;

// 正弦查表。
// 中断里如果直接调用 sinf/cosf，会明显拖慢快环速度；
// 因此这里在启动时一次性生成 LUT，运行期只查表和插值。
static int16_t s_sin_lut_q15[FOC_SIN_LUT_SIZE];
static bool s_sin_lut_ready = false;

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

static void foc_build_sin_lut(void)
{
    // 查表只在启动阶段构建一次，后面整个运行期都重复使用。
    // 这样可以把三角函数的计算开销提前挪到非实时阶段。
    if (s_sin_lut_ready) {
        return;
    }

    for (uint32_t i = 0; i < FOC_SIN_LUT_SIZE; ++i) {
        const float angle = ((float)i * TWO_PI) / (float)FOC_SIN_LUT_SIZE;
        s_sin_lut_q15[i] = (int16_t)lroundf(sinf(angle) * (float)FOC_Q15_ONE);
    }

    s_sin_lut_ready = true;
}

static void foc_voltage_vector_to_q15(float uq, float ud, int16_t *uq_q15, int16_t *ud_q15)
{
    // 这里不是分别限制 Uq 和 Ud，而是按 dq 电压矢量的总模长统一限幅。
    // 这样做的原因是：
    // 1. 能保持原本电压矢量方向不被破坏；
    // 2. 能确保输出仍位于 SVPWM 线性区内；
    // 3. 避免单轴裁剪导致实际相位偏掉。
    float uq_norm = uq / foc_params.voltage_power_supply;
    float ud_norm = ud / foc_params.voltage_power_supply;
    const float magnitude = sqrtf((uq_norm * uq_norm) + (ud_norm * ud_norm));

    if (magnitude > 0.577f && magnitude > 0.0f) {
        const float scale = 0.577f / magnitude;
        uq_norm *= scale;
        ud_norm *= scale;
    }

    *uq_q15 = (int16_t)lroundf(uq_norm * (float)FOC_Q15_ONE);
    *ud_q15 = (int16_t)lroundf(ud_norm * (float)FOC_Q15_ONE);
    // 转成 q15 之后，快环就可以主要使用整数完成后续运算，减少浮点参与频率。
}

void foc_control_init(void)
{
    // 这里只初始化当前固件确实在用的参数。
    // 之前为了兼容 STM32 示例工程保留了很多历史字段，会明显拉低可读性，
    // 现在都已经删掉了，只留下真正参与启动、校准、快环的部分。
    foc_params.shaft_angle_u16 = 0U;
    foc_params.electrical_angle_u16 = 0U;
    foc_params.mechanical_angle_unwrapped = 0.0f;
    foc_params.mechanical_rpm = 0.0f;
    foc_params.target_mechanical_rpm = 50.0f;
    foc_params.uq = 0.0f;
    foc_params.ud = 0.0f;
    foc_params.voltage_power_supply = 20.0f;
    foc_params.pole_pairs = 7;
    foc_params.sensor_direction = 1;
    foc_params.zero_electric_angle = 0.0f;
    foc_params.uq_q15 = 0;
    foc_params.ud_q15 = 0;

    foc_build_sin_lut();

    foc_enabled = 1;
    foc_started = 0;
    foc_calibrated = 0;

    ESP_LOGI(TAG, "FOC核心参数已初始化");
}

void foc_set_voltage_target(float uq, float ud)
{
    // 这里同时保留 float 和 q15 两份目标值：
    // 1. float 版本更适合做上层配置、调试和日志打印；
    // 2. q15 版本会被中断直接使用，避免快环里重复做浮点转定点。
    foc_params.uq = uq;
    foc_params.ud = ud;
    foc_voltage_vector_to_q15(uq, ud, &foc_params.uq_q15, &foc_params.ud_q15);
}

uint16_t foc_float_to_angle_u16(float angle)
{
    // 把弧度角映射到 16 位环形角度空间。
    // 这样做的好处是：
    // 1. 占用小，适合快环反复运算；
    // 2. 天然支持溢出回绕；
    // 3. 查表时可以直接拿高位做索引、低位做插值。
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.0f) {
        angle += TWO_PI;
    }
    return (uint16_t)((angle * (float)FOC_ANGLE_FULL_TURN) / TWO_PI);
}

int16_t foc_voltage_to_q15(float voltage)
{
    // 先把电压限制在线性 SVPWM 区域，再转 q15。
    // 这里的 0.577 大致对应 1/sqrt(3)，是三相线性调制区的安全范围。
    const float normalized = foc_clampf(voltage / foc_params.voltage_power_supply, -0.577f, 0.577f);
    return (int16_t)lroundf(normalized * (float)FOC_Q15_ONE);
}

int16_t IRAM_ATTR foc_sin_q15_from_u16(uint16_t angle_u16)
{
    // 高位用于 LUT 索引，低位用于线性插值。
    // 这样既能保持速度，又能减少查表量化误差。
    const uint32_t index = (angle_u16 >> FOC_SIN_LUT_SHIFT) & FOC_SIN_LUT_MASK;
    const uint32_t next_index = (index + 1U) & FOC_SIN_LUT_MASK;
    const uint32_t frac = angle_u16 & ((1U << FOC_SIN_LUT_SHIFT) - 1U);
    const int32_t y0 = s_sin_lut_q15[index];
    const int32_t y1 = s_sin_lut_q15[next_index];
    return (int16_t)(y0 + (((y1 - y0) * (int32_t)frac) >> FOC_SIN_LUT_SHIFT));
}

int16_t IRAM_ATTR foc_cos_q15_from_u16(uint16_t angle_u16)
{
    // 利用 cos(x)=sin(x+pi/2) 的关系，只保留一份正弦表即可。
    // 这样既省内存，也减少启动阶段的表生成工作量。
    return foc_sin_q15_from_u16((uint16_t)(angle_u16 + (FOC_ANGLE_FULL_TURN / 4U)));
}
