#ifndef FOC_LOOP_H
#define FOC_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

void foc_loop_init(void);
void foc_update_current_feedback(float electrical_angle);
float foc_read_shaft_angle(void);
float foc_electrical_angle_from_shaft(float shaft_angle);
void foc_fast_loop_step(void);
void foc_fast_loop_step_isr(void);

#ifdef __cplusplus
}
#endif

#endif
