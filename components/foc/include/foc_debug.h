#ifndef FOC_DEBUG_H
#define FOC_DEBUG_H

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FOC_DEBUG_UART_PORT UART_NUM_1
#define FOC_DEBUG_UART_TX_GPIO 35
#define FOC_DEBUG_UART_RX_GPIO 36
#define FOC_DEBUG_UART_BAUDRATE 921600

esp_err_t foc_debug_init(void);
void FireWaterPrintf(const char *format, ...);
void foc_debug_print_calibration_result(void);

#ifdef __cplusplus
}
#endif

#endif
