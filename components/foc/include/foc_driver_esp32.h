#ifndef FOC_DRIVER_ESP32_H
#define FOC_DRIVER_ESP32_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_PHASE_U_GPIO 17
#define FOC_PHASE_V_GPIO 18
#define FOC_PHASE_W_GPIO 8

#define FOC_PWM_FREQUENCY_HZ 30000
#define FOC_CONTROL_FREQUENCY_HZ 5000
#define FOC_CURRENT_SAMPLE_FREQUENCY_HZ 2500
#define FOC_MCPWM_TIMER_RESOLUTION_HZ 80000000UL

typedef struct {
    int gpio_u;
    int gpio_v;
    int gpio_w;
    uint32_t pwm_frequency_hz;
    uint32_t pwm_resolution_bits;
    uint32_t pwm_max_duty;
} foc_driver_config_t;

typedef void (*foc_fast_loop_init_fn_t)(void);
typedef void (*foc_fast_loop_step_fn_t)(void);

void foc_driver_bind_fast_loop(foc_fast_loop_init_fn_t init_fn, foc_fast_loop_step_fn_t step_fn);
esp_err_t foc_driver_start(void);
void foc_driver_set_pwm_raw(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w);
const foc_driver_config_t *foc_driver_get_config(void);

#ifdef __cplusplus
}
#endif

#endif
