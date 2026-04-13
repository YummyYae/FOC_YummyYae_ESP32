#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void FOC_Parameters_Init(FOC_Parameters *params);
void FOC_Calibrate_Init(void);
void FOC_Calibrate_Run(void);

void setTargetVotage(float target_q, float target_d);
void setTargetI(float targeti_q, float targeti_d);
void setPhaseVoltage(float uq, float ud, float angle_el);
void Clarke_Transf(float current_abc[3], float current_alpha_beta[2]);
void Park_Transf(float current_alpha_beta[2], float angle_el, float current_dq[2]);

float foc_angle_u16_to_float(uint16_t angle_u16);
uint16_t foc_float_to_angle_u16(float angle);
int16_t foc_voltage_to_q15(float voltage);
void foc_build_sin_lut(void);
int16_t foc_sin_q15_from_u16(uint16_t angle_u16);
int16_t foc_cos_q15_from_u16(uint16_t angle_u16);

#ifdef __cplusplus
}
#endif

#endif
