#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TWO_PI 6.28318530718f
#define FOC_OPEN_LOOP_UQ 20.0f
#define FOC_OPEN_LOOP_UD 0.0f

#define FOC_Q15_SHIFT 15
#define FOC_Q15_ONE 32767
#define FOC_Q15_HALF 16384
#define FOC_Q15_NEG_HALF (-16384)
#define FOC_Q15_SQRT3_BY_2 28378
#define FOC_Q15_INV_SQRT3 18919

#define FOC_ANGLE_FULL_TURN 65536U
#define FOC_SIN_LUT_SIZE 1024U
#define FOC_SIN_LUT_MASK (FOC_SIN_LUT_SIZE - 1U)
#define FOC_SIN_LUT_SHIFT 6U

typedef struct {
    uint16_t shaft_angle_u16;
    uint16_t electrical_angle_u16;
    float mechanical_angle_unwrapped;
    float mechanical_rpm;
    float target_mechanical_rpm;
    float uq;
    float ud;
    float voltage_power_supply;
    int pole_pairs;
    long sensor_direction;
    float zero_electric_angle;
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
