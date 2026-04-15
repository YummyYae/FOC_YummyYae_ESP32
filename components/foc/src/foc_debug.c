#include "foc_debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "foc_control.h"

static foc_debug_output_fn_t s_output_writer = NULL;

esp_err_t foc_debug_init(void)
{
    // AP 端不再初始化任何串口，调试输出通过可注入 writer 走 UDP。
    return ESP_OK;
}

void foc_debug_set_output_writer(foc_debug_output_fn_t writer)
{
    s_output_writer = writer;
}

void FireWaterPrintf(const char *format, ...)
{
    if (s_output_writer == NULL) {
        return;
    }

    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf) - 3, format, args);
    va_end(args);

    strcat(buf, "\r\n");
    s_output_writer(buf, strlen(buf));
}

void foc_debug_print_calibration_result(void)
{
    FireWaterPrintf("==================================================");
    FireWaterPrintf("             [FOC Calibration Ready]              ");
    FireWaterPrintf("==================================================");
    FireWaterPrintf("pole_pairs=%.0f", (float)foc_params.pole_pairs);
    FireWaterPrintf("zero_electric_angle=%.6f", foc_params.zero_electric_angle);
    FireWaterPrintf("sensor_direction=%.0f", (float)foc_params.sensor_direction);
    FireWaterPrintf("voltage_power_supply=%.3f", foc_params.voltage_power_supply);
    FireWaterPrintf("uq=%.3f", foc_params.uq);
    FireWaterPrintf("ud=%.3f", foc_params.ud);
    FireWaterPrintf("==================================================");
}

