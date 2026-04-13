#ifndef FOC_DRIVER_ESP32_H
#define FOC_DRIVER_ESP32_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_PHASE_U_GPIO 17
#define FOC_PHASE_V_GPIO 18
#define FOC_PHASE_W_GPIO 8

#define FOC_PWM_FREQUENCY_HZ 20000
#define FOC_CONTROL_FREQUENCY_HZ 5000

typedef void (*foc_isr_callback_t)(void);

typedef struct {
    int gpio_u;
    int gpio_v;
    int gpio_w;
    uint32_t pwm_frequency_hz;
    uint32_t pwm_resolution_bits;
    uint32_t pwm_max_duty;
} foc_driver_config_t;

esp_err_t foc_driver_start(foc_isr_callback_t foc_isr_callback);
esp_err_t foc_driver_init_pwm(const foc_driver_config_t *config);
void foc_driver_set_pwm(float duty_u, float duty_v, float duty_w);
void foc_driver_set_pwm_raw(uint32_t duty_u, uint32_t duty_v, uint32_t duty_w);
void foc_driver_output_disable(void);
bool foc_driver_is_ready(void);
const foc_driver_config_t *foc_driver_get_config(void);

#ifdef __cplusplus
}
#endif

#endif
