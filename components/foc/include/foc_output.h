#ifndef FOC_OUTPUT_H
#define FOC_OUTPUT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void foc_set_phase_voltage_q15(int16_t uq_q15, int16_t ud_q15, uint16_t electrical_angle_u16);

#ifdef __cplusplus
}
#endif

#endif
