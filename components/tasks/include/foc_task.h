#ifndef FOC_TASK_H
#define FOC_TASK_H

#include "esp_attr.h"

#ifdef __cplusplus
extern "C" {
#endif

void foc_task_init(void);
void foc_fast_loop_step_isr(void);

#ifdef __cplusplus
}
#endif

#endif
