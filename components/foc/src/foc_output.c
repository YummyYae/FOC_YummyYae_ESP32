#include "foc_output.h"

#include "esp_attr.h"
#include "foc_control.h"
#include "foc_driver_esp32.h"

static int32_t IRAM_ATTR foc_q15_mul(int32_t a, int32_t b)
{
    // q15 乘法。
    // 两个 q15 数相乘后结果会变成 q30，因此这里右移 15 位把量纲拉回 q15。
    return (a * b) >> FOC_Q15_SHIFT;
}

static uint32_t IRAM_ATTR foc_q15_to_pwm_raw(int32_t centered_q15)
{
    // 把“以零为中心”的相电压 q15 值，换算成硬件 PWM 比较值。
    // 这里先得到传统意义上的 0~100% 占空比，再换算成 MCPWM 的比较值。
    // 注意：
    // 1. 旧 LEDC 方案是“占空比越大，输出有效时间越长”；
    // 2. 现在 MCPWM 这里使用的是中心对齐比较模式，比较值越小，高电平脉宽反而越宽。
    // 所以最后要做一次反相映射，不能直接沿用 LEDC 的数值语义。
    const foc_driver_config_t *config = foc_driver_get_config();
    const int32_t duty_offset_q15 = foc_q15_mul(centered_q15, FOC_Q15_INV_SQRT3);
    int32_t duty_q15 = duty_offset_q15 + FOC_Q15_HALF;

    if (duty_q15 < 0) {
        duty_q15 = 0;
    } else if (duty_q15 > FOC_Q15_ONE) {
        duty_q15 = FOC_Q15_ONE;
    }

    // 先算出“直接占空比”的计数值，再转成 MCPWM 比较值。
    // `pwm_max_duty` 在当前工程里就是允许写入比较器的最大安全值。
    const int32_t compare_q15 = FOC_Q15_ONE - duty_q15;
    uint32_t compare_value = (uint32_t)((compare_q15 * (int32_t)config->pwm_max_duty) >> FOC_Q15_SHIFT);

    // MCPWM 在中心对齐模式下，比较值贴近两端边界时更容易出现时序边界问题。
    // 这里强制把输出限制在一个更保守的安全区间内，避免 0 或 peak 附近的极值。
    if (compare_value < 2U) {
        compare_value = 2U;
    } else if (compare_value > (config->pwm_max_duty - 2U)) {
        compare_value = config->pwm_max_duty - 2U;
    }

    return compare_value;
}

void IRAM_ATTR foc_set_phase_voltage_q15(int16_t uq_q15, int16_t ud_q15, uint16_t electrical_angle_u16)
{
    // 1. dq -> alpha/beta
    // 这一段就是反 Park 变换。
    // dq 坐标系里的 Uq、Ud 是跟着转子一起转的；
    // 而三相逆变输出需要的是定子静止坐标系下的 alpha/beta 电压。
    const int16_t sin_q15 = foc_sin_q15_from_u16(electrical_angle_u16);
    const int16_t cos_q15 = foc_cos_q15_from_u16(electrical_angle_u16);

    const int32_t u_alpha_q15 = foc_q15_mul(ud_q15, cos_q15) - foc_q15_mul(uq_q15, sin_q15);
    const int32_t u_beta_q15 = foc_q15_mul(ud_q15, sin_q15) + foc_q15_mul(uq_q15, cos_q15);

    // 2. alpha/beta -> 三相电压
    // 这里按标准三相对称关系，把二维静止坐标电压展开为 U/V/W 三相。
    const int32_t phase_u_q15 = u_alpha_q15;
    const int32_t phase_v_q15 = foc_q15_mul(FOC_Q15_NEG_HALF, u_alpha_q15) +
                                foc_q15_mul(FOC_Q15_SQRT3_BY_2, u_beta_q15);
    const int32_t phase_w_q15 = foc_q15_mul(FOC_Q15_NEG_HALF, u_alpha_q15) -
                                foc_q15_mul(FOC_Q15_SQRT3_BY_2, u_beta_q15);

    int32_t vmax = phase_u_q15;
    int32_t vmin = phase_u_q15;

    if (phase_v_q15 > vmax) vmax = phase_v_q15;
    if (phase_w_q15 > vmax) vmax = phase_w_q15;
    if (phase_v_q15 < vmin) vmin = phase_v_q15;
    if (phase_w_q15 < vmin) vmin = phase_w_q15;

    // 3. 注入共模电压
    // 通过减去 (vmax+vmin)/2，把三相整体居中到母线可用范围里。
    // 这一步本质上就是 centered SVPWM 的核心动作，
    // 它能在不改变线电压的前提下，尽量提高直流母线利用率。
    const int32_t common_mode_q15 = (vmax + vmin) >> 1;
    const uint32_t duty_u_raw = foc_q15_to_pwm_raw(phase_u_q15 - common_mode_q15);
    const uint32_t duty_v_raw = foc_q15_to_pwm_raw(phase_v_q15 - common_mode_q15);
    const uint32_t duty_w_raw = foc_q15_to_pwm_raw(phase_w_q15 - common_mode_q15);

    // 4. 把三路占空比真正写入硬件。
    // 到这里为止，dq 电压命令已经完整变成了三相 PWM 输出。
    foc_driver_set_pwm_raw(duty_u_raw, duty_v_raw, duty_w_raw);
}
