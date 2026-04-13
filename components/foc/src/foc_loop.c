#include "foc_loop.h"

#include "esp_attr.h"
#include "foc_control.h"
#include "foc_output.h"
#include "mt6701.h"

static uint16_t s_zero_electric_angle_u16 = 0U;

static void foc_loop_sync_calibration_cache(void)
{
    s_zero_electric_angle_u16 = foc_float_to_angle_u16(foc_params.zero_electric_angle);
}

void foc_loop_init(void)
{
    foc_loop_sync_calibration_cache();
}

void IRAM_ATTR foc_update_current_feedback(float electrical_angle)
{
    (void)electrical_angle;

    foc_params.iu = 0.0f;
    foc_params.iv = 0.0f;
    foc_params.iw = 0.0f;
    foc_params.i_alpha = foc_params.iu;
    foc_params.i_beta = (foc_params.iu + (2.0f * foc_params.iv)) * ONE_BY_SQRT3;
    foc_params.id = 0.0f;
    foc_params.iq = 0.0f;
}

float IRAM_ATTR foc_read_shaft_angle(void)
{
    const uint16_t raw = MT6701_GetRawData();
    foc_params.shaft_angle_u16 = (uint16_t)(raw << 2);
    return foc_angle_u16_to_float(foc_params.shaft_angle_u16);
}

float IRAM_ATTR foc_electrical_angle_from_shaft(float shaft_angle)
{
    const int32_t shaft_u16 = (int32_t)foc_params.shaft_angle_u16;
    const int32_t direction = (foc_params.sensor_direction >= 0) ? 1 : -1;
    const int32_t electrical_u16 = (shaft_u16 * foc_params.pole_pairs * direction) - s_zero_electric_angle_u16;

    (void)shaft_angle;
    foc_params.electrical_angle_u16 = (uint16_t)electrical_u16;
    return foc_angle_u16_to_float(foc_params.electrical_angle_u16);
}

void IRAM_ATTR foc_fast_loop_step_isr(void)
{
    if (!foc_enabled || !foc_started || !foc_calibrated) {
        return;
    }

    foc_params.shaft_angle = foc_read_shaft_angle();
    foc_params.electrical_angle = foc_electrical_angle_from_shaft(foc_params.shaft_angle);
    foc_update_current_feedback(foc_params.electrical_angle);
    foc_set_phase_voltage_q15(foc_params.uq_q15, foc_params.ud_q15, foc_params.electrical_angle_u16);
}

void foc_fast_loop_step(void)
{
    foc_fast_loop_step_isr();
}
