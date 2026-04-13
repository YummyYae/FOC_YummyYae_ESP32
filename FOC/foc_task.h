#ifndef FOC_TASK_H
#define FOC_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

void foc_task_init(void);
void update_current_sensor(float electrical_angle);
float foc_placeholder_get_shaft_angle(void);
float foc_electrical_angle_from_shaft(float shaft_angle);
void foc_fast_loop_step(void);
void foc_fast_loop_step_isr(void);

#ifdef __cplusplus
}
#endif

#endif
