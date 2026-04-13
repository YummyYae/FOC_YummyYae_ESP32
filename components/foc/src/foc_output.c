#include "foc_output.h"

#include "esp_attr.h"
#include "foc_control.h"
#include "foc_driver_esp32.h"

static int32_t IRAM_ATTR foc_q15_mul(int32_t a, int32_t b)
{
    return (a * b) >> FOC_Q15_SHIFT;
}

static uint32_t IRAM_ATTR foc_q15_to_pwm_raw(int32_t centered_q15)
{
    const foc_driver_config_t *config = foc_driver_get_config();
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
    const int16_t sin_q15 = foc_sin_q15_from_u16(electrical_angle_u16);
    const int16_t cos_q15 = foc_cos_q15_from_u16(electrical_angle_u16);

    const int32_t u_alpha_q15 = foc_q15_mul(ud_q15, cos_q15) - foc_q15_mul(uq_q15, sin_q15);
    const int32_t u_beta_q15 = foc_q15_mul(ud_q15, sin_q15) + foc_q15_mul(uq_q15, cos_q15);

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

    const int32_t common_mode_q15 = (vmax + vmin) >> 1;
    const uint32_t duty_u_raw = foc_q15_to_pwm_raw(phase_u_q15 - common_mode_q15);
    const uint32_t duty_v_raw = foc_q15_to_pwm_raw(phase_v_q15 - common_mode_q15);
    const uint32_t duty_w_raw = foc_q15_to_pwm_raw(phase_w_q15 - common_mode_q15);

    const uint32_t pwm_max = foc_driver_get_config()->pwm_max_duty;
    foc_params.pwm_u = (float)duty_u_raw / (float)pwm_max;
    foc_params.pwm_v = (float)duty_v_raw / (float)pwm_max;
    foc_params.pwm_w = (float)duty_w_raw / (float)pwm_max;

    foc_driver_set_pwm_raw(duty_u_raw, duty_v_raw, duty_w_raw);
}
