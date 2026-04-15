#ifndef AP_TASK_H
#define AP_TASK_H

#include "esp_err.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ap_task_start(void);
int ap_task_send_udp_text(const char *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
