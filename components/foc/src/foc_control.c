#include "foc_control.h"

#include <math.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "foc_output.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static const char *TAG = "foc_core";

FOC_Parameters foc_params;

uint8_t foc_enabled = 1;
uint8_t foc_started = 0;
uint8_t foc_calibrated = 0;

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

uint16_t foc_float_to_angle_u16(float angle)
{
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.0f) {
        angle += TWO_PI;
    }
    return (uint16_t)((angle * (float)FOC_ANGLE_FULL_TURN) / TWO_PI);
}

float foc_angle_u16_to_float(uint16_t angle_u16)
{
    return ((float)angle_u16 * TWO_PI) / (float)FOC_ANGLE_FULL_TURN;
}

int16_t foc_voltage_to_q15(float voltage)
{
    const float normalized = foc_clampf(voltage / foc_params.voltage_power_supply, -0.577f, 0.577f);
    return (int16_t)lroundf(normalized * (float)FOC_Q15_ONE);
}

static void foc_voltage_vector_to_q15(float uq, float ud, int16_t *uq_q15, int16_t *ud_q15)
{
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
}

void foc_build_sin_lut(void)
{
    if (s_sin_lut_ready) {
        return;
    }

    for (uint32_t i = 0; i < FOC_SIN_LUT_SIZE; ++i) {
        const float angle = ((float)i * TWO_PI) / (float)FOC_SIN_LUT_SIZE;
        s_sin_lut_q15[i] = (int16_t)lroundf(sinf(angle) * (float)FOC_Q15_ONE);
    }

    s_sin_lut_ready = true;
}

int16_t IRAM_ATTR foc_sin_q15_from_u16(uint16_t angle_u16)
{
    const uint32_t index = (angle_u16 >> FOC_SIN_LUT_SHIFT) & FOC_SIN_LUT_MASK;
    const uint32_t next_index = (index + 1U) & FOC_SIN_LUT_MASK;
    const uint32_t frac = angle_u16 & ((1U << FOC_SIN_LUT_SHIFT) - 1U);
    const int32_t y0 = s_sin_lut_q15[index];
    const int32_t y1 = s_sin_lut_q15[next_index];
    return (int16_t)(y0 + (((y1 - y0) * (int32_t)frac) >> FOC_SIN_LUT_SHIFT));
}

int16_t IRAM_ATTR foc_cos_q15_from_u16(uint16_t angle_u16)
{
    return foc_sin_q15_from_u16((uint16_t)(angle_u16 + (FOC_ANGLE_FULL_TURN / 4U)));
}

void FOC_Parameters_Init(FOC_Parameters *params)
{
    if (params == NULL) {
        return;
    }

    params->shaft_angle = 0.0f;
    params->electrical_angle = 0.0f;
    params->shaft_angle_u16 = 0U;
    params->electrical_angle_u16 = 0U;
    params->iu = 0.0f;
    params->iv = 0.0f;
    params->iw = 0.0f;
    params->iu_offset = 0.0f;
    params->iv_offset = 0.0f;
    params->iw_offset = 0.0f;
    params->pwm_u = 0.0f;
    params->pwm_v = 0.0f;
    params->pwm_w = 0.0f;
    params->uq = 0.0f;
    params->ud = 0.0f;
    params->iq = 0.0f;
    params->id = 0.0f;
    params->target_iq = 0.0f;
    params->target_id = 0.0f;
    params->motor_speed = 0.0f;
    params->target_speed = 0.0f;
    params->motor_position = 0.0f;
    params->target_position = 0.0f;
    params->i_alpha = 0.0f;
    params->i_beta = 0.0f;
    params->v_alpha = 0.0f;
    params->v_beta = 0.0f;
    params->voltage_power_supply = 12.0f;
    params->pole_pairs = 7;
    params->sensor_direction = 1;
    params->zero_electric_angle = 0.0f;
    params->phase_resistance = 0.0f;
    params->uq_q15 = 0;
    params->ud_q15 = 0;
}

void FOC_Calibrate_Init(void)
{
    FOC_Parameters_Init(&foc_params);
    foc_build_sin_lut();

    foc_calibrated = 0;
    foc_started = 0;
    foc_enabled = 1;
    setTargetVotage(0.0f, 0.0f);

    ESP_LOGI(TAG, "FOC base parameters initialized, waiting for flash load or startup calibration");
}

void FOC_Calibrate_Run(void)
{
    foc_calibrated = 0;
    ESP_LOGW(TAG, "FOC_Calibrate_Run skipped: calibration is handled by foc_driver_esp32");
}

void setTargetVotage(float target_q, float target_d)
{
    foc_params.uq = target_q;
    foc_params.ud = target_d;
    foc_voltage_vector_to_q15(target_q, target_d, &foc_params.uq_q15, &foc_params.ud_q15);
}

void setTargetI(float targeti_q, float targeti_d)
{
    foc_params.target_iq = targeti_q;
    foc_params.target_id = targeti_d;
}

void Clarke_Transf(float current_abc[3], float current_alpha_beta[2])
{
    const float i_alpha = (current_abc[0] - 0.5f * (current_abc[1] + current_abc[2])) * (2.0f / 3.0f);
    const float i_beta = (current_abc[1] - current_abc[2]) * SQRT3_BY_2 * (2.0f / 3.0f);

    foc_params.i_alpha = i_alpha;
    foc_params.i_beta = i_beta;

    if (current_alpha_beta != NULL) {
        current_alpha_beta[0] = i_alpha;
        current_alpha_beta[1] = i_beta;
    }
}

void Park_Transf(float current_alpha_beta[2], float angle_el, float current_dq[2])
{
    const float cos_angle = cosf(angle_el);
    const float sin_angle = sinf(angle_el);
    const float id = current_alpha_beta[0] * cos_angle + current_alpha_beta[1] * sin_angle;
    const float iq = -current_alpha_beta[0] * sin_angle + current_alpha_beta[1] * cos_angle;

    foc_params.id = id;
    foc_params.iq = iq;

    if (current_dq != NULL) {
        current_dq[0] = id;
        current_dq[1] = iq;
    }
}

void setPhaseVoltage(float uq, float ud, float angle_el)
{
    foc_set_phase_voltage_q15(foc_voltage_to_q15(uq),
                              foc_voltage_to_q15(ud),
                              foc_float_to_angle_u16(angle_el));
}
