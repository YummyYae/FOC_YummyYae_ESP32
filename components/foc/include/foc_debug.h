#ifndef FOC_DEBUG_H
#define FOC_DEBUG_H

#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*foc_debug_output_fn_t)(const char *data, size_t len);

esp_err_t foc_debug_init(void);
void foc_debug_set_output_writer(foc_debug_output_fn_t writer);
void FireWaterPrintf(const char *format, ...);
void foc_debug_print_calibration_result(void);

#ifdef __cplusplus
}
#endif

#endif
