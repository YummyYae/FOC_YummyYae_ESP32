#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SQRT3_BY_2 0.86602540378f
#define ONE_BY_SQRT3 0.57735026919f
#define TWO_PI 6.28318530718f

#define FOC_OPEN_LOOP_UQ 2.0f
#define FOC_OPEN_LOOP_UD 0.0f

// Q15 fixed-point format: 1.0 ~= 32767.
#define FOC_Q15_SHIFT 15
#define FOC_Q15_ONE 32767
#define FOC_Q15_HALF 16384
#define FOC_Q15_NEG_HALF (-16384)
#define FOC_Q15_SQRT3_BY_2 28378
#define FOC_Q15_INV_SQRT3 18919

// One full electrical turn maps to 16-bit wrapping angle space.
#define FOC_ANGLE_FULL_TURN 65536U
#define FOC_SIN_LUT_SIZE 1024U
#define FOC_SIN_LUT_MASK (FOC_SIN_LUT_SIZE - 1U)
#define FOC_SIN_LUT_SHIFT 6U

typedef struct {
    float shaft_angle;
    float electrical_angle;
    uint16_t shaft_angle_u16;
    uint16_t electrical_angle_u16;
    float iu;
    float iv;
    float iw;
    float iu_offset;
    float iv_offset;
    float iw_offset;
    float pwm_u;
    float pwm_v;
    float pwm_w;
    float uq;
    float ud;
    float iq;
    float id;
    float target_iq;
    float target_id;
    float motor_speed;
    float target_speed;
    float motor_position;
    float target_position;
    float i_alpha;
    float i_beta;
    float v_alpha;
    float v_beta;
    float voltage_power_supply;
    int pole_pairs;
    long sensor_direction;
    float zero_electric_angle;
    float phase_resistance;
    int16_t uq_q15;
    int16_t ud_q15;
} FOC_Parameters;

extern FOC_Parameters foc_params;
extern uint8_t foc_enabled;
extern uint8_t foc_started;
extern uint8_t foc_calibrated;

#ifdef __cplusplus
}
#endif

#endif
