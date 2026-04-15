#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void foc_control_init(void);
void foc_set_voltage_target(float uq, float ud);
uint16_t foc_float_to_angle_u16(float angle);
int16_t foc_voltage_to_q15(float voltage);
int16_t foc_sin_q15_from_u16(uint16_t angle_u16);
int16_t foc_cos_q15_from_u16(uint16_t angle_u16);

#ifdef __cplusplus
}
#endif

#endif
