#include "foc_calibrate.h"

#include "esp_attr.h"
#include "foc_driver_esp32.h"
#include "foc_lib.h"

#define M_PI 3.14159265358979323846f

static int32_t IRAM_ATTR foc_q15_mul(int32_t a, int32_t b)
{
    return (a * b) >> FOC_Q15_SHIFT;
}

static uint32_t IRAM_ATTR foc_q15_to_pwm_raw(int32_t centered_q15)
{
    const foc_driver_config_t *config = foc_driver_get_config();

    // 这里不能直接把 centered_q15 加到 0.5 duty 上。
    // inverse Park + 逆 Clarke 得到的是三相相电压分量，映射到原始 SVPWM duty 时
    // 还需要乘一个 1/sqrt(3) 的比例，才能与之前浮点版的 setPhaseVoltage() 一致。
    // 少了这个比例会导致同样的 Uq/Ud 被放大，出现开环电流偏大、发热上升的问题。
    const int32_t duty_offset_q15 = foc_q15_mul(centered_q15, FOC_Q15_INV_SQRT3);
    int32_t duty_q15 = duty_offset_q15 + FOC_Q15_HALF;

    if (duty_q15 < 0) {
        duty_q15 = 0;
    } else if (duty_q15 > FOC_Q15_ONE) {
        duty_q15 = FOC_Q15_ONE;
    }

    return (uint32_t)((duty_q15 * (int32_t)config->pwm_max_duty) >> FOC_Q15_SHIFT);
}

void IRAM_ATTR foc_set_phase_voltage_q15(int16_t uq_q15, int16_t ud_q15, uint16_t electrical_angle_u16)
{
    // 快环输出部分单独放在本文件，专门负责 q15格式的 逆转 Park 变换和 Centered SVPWM(基于中点的空间矢量PWM调制)。
    
    // 1. 查表获取当前电角度的正余弦值，使用定点数 Q15(范围-32768~32767) 提高运算速度
    const int16_t sin_q15 = foc_sin_q15_from_u16(electrical_angle_u16);
    const int16_t cos_q15 = foc_cos_q15_from_u16(electrical_angle_u16);

    // 2. 逆 Park 变换 (Inverse Park Transform)
    // 将旋转坐标系下的 Ud (直轴，励磁方向) 和 Uq (交轴，转矩方向) 电压分量，
    // 投影到静止的定子参考坐标系 (Alpha-Beta 二维正交坐标系)
    // u_alpha = ud * cos(θ) - uq * sin(θ)
    // u_beta  = ud * sin(θ) + uq * cos(θ)
    const int32_t u_alpha_q15 = foc_q15_mul(ud_q15, cos_q15) - foc_q15_mul(uq_q15, sin_q15);
    const int32_t u_beta_q15 = foc_q15_mul(ud_q15, sin_q15) + foc_q15_mul(uq_q15, cos_q15);

    // 3. 逆 Clarke 变换 (Inverse Clarke Transform)
    // 将静止正交两相坐标系 (Alpha-Beta) 转换到 实际的电机三相绝对坐标系(U, V, W)
    // Phase U 相刚好对齐 alpha 轴，故 Phase U = u_alpha
    const int32_t phase_u_q15 = u_alpha_q15;
    // Phase V 和 W 轴则相位差 120° 和 240°，利用特定的系数（根号3相关的常数）换算
    const int32_t phase_v_q15 = foc_q15_mul(FOC_Q15_NEG_HALF, u_alpha_q15) + foc_q15_mul(FOC_Q15_SQRT3_BY_2, u_beta_q15);
    const int32_t phase_w_q15 = foc_q15_mul(FOC_Q15_NEG_HALF, u_alpha_q15) - foc_q15_mul(FOC_Q15_SQRT3_BY_2, u_beta_q15);

    // 4. SVPWM马鞍波构造 (相电压平移中心点)
    // 查找当前三相输出电压的极值点
    int32_t vmax = phase_u_q15;
    int32_t vmin = phase_u_q15;

    if (phase_v_q15 > vmax) vmax = phase_v_q15;
    if (phase_w_q15 > vmax) vmax = phase_w_q15;
    if (phase_v_q15 < vmin) vmin = phase_v_q15;
    if (phase_w_q15 < vmin) vmin = phase_w_q15;

    // 计算共模电压（三相电压极值的均值点）
    const int32_t common_mode_q15 = (vmax + vmin) >> 1;
    // 从各项相电压中减去共模电压得到马鞍状波形（注入三次谐波），以此充分利用母线供电，增加 15% 左右的利用率
    const uint32_t duty_u_raw = foc_q15_to_pwm_raw(phase_u_q15 - common_mode_q15);
    const uint32_t duty_v_raw = foc_q15_to_pwm_raw(phase_v_q15 - common_mode_q15);
    const uint32_t duty_w_raw = foc_q15_to_pwm_raw(phase_w_q15 - common_mode_q15);

    // 存储当前计算的占空比信息（范围通常为 0.0 ~ 1.0）便于外部观测/上位机调试
    const uint32_t pwm_max = foc_driver_get_config()->pwm_max_duty;
    foc_params.pwm_u = (float)duty_u_raw / (float)pwm_max;
    foc_params.pwm_v = (float)duty_v_raw / (float)pwm_max;
    foc_params.pwm_w = (float)duty_w_raw / (float)pwm_max;

    // 5. 将计算出的原始占空比投递给 ESP32 定时器（LEDC通道等）进行底层硬件配置与高频发波
    foc_driver_set_pwm_raw(duty_u_raw, duty_v_raw, duty_w_raw);
}
