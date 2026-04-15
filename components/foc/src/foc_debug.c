#include "foc_debug.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_log.h"
#include "foc_control.h"

static const char *TAG = "foc_debug";
static bool s_uart_ready = false;

static esp_err_t foc_debug_uart_init(void)
{
    if (s_uart_ready) {
        return ESP_OK;
    }

    const uart_config_t uart_config = {
        .baud_rate = FOC_DEBUG_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // 当前串口只用于发送实时状态，不处理上位机回传命令。
    ESP_RETURN_ON_ERROR(uart_driver_install(FOC_DEBUG_UART_PORT, 1024, 0, 0, NULL, 0), TAG, "串口驱动安装失败");
    ESP_RETURN_ON_ERROR(uart_param_config(FOC_DEBUG_UART_PORT, &uart_config), TAG, "串口参数配置失败");
    ESP_RETURN_ON_ERROR(
        uart_set_pin(FOC_DEBUG_UART_PORT, FOC_DEBUG_UART_TX_GPIO, FOC_DEBUG_UART_RX_GPIO,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
        TAG,
        "串口引脚配置失败");

    s_uart_ready = true;
    ESP_LOGI(TAG, "调试串口已启动: uart=%d tx=%d rx=%d baud=%d",
             FOC_DEBUG_UART_PORT, FOC_DEBUG_UART_TX_GPIO, FOC_DEBUG_UART_RX_GPIO, FOC_DEBUG_UART_BAUDRATE);
    return ESP_OK;
}

void FireWaterPrintf(const char *format, ...)
{
    if (!s_uart_ready) {
        return;
    }

    // 统一在这里补上换行，方便串口助手逐行解析。
    char buf[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf) - 3, format, args);
    va_end(args);

    strcat(buf, "\r\n");
    uart_write_bytes(FOC_DEBUG_UART_PORT, buf, strlen(buf));
}

void foc_debug_print_calibration_result(void)
{
    // 上电阶段打印一次关键校准结果，便于确认参数是否正确加载。
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

esp_err_t foc_debug_init(void)
{
    // 这个模块现在只负责串口自身初始化。
    // 任务调度已经全部迁移到 task 目录下的控制任务里。
    return foc_debug_uart_init();
}
