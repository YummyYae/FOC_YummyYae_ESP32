#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_MODE_POSITION = 0,
    PID_MODE_DELTA = 1,
} pid_mode_t;

typedef struct {
    uint8_t mode;
    float kp;
    float ki;
    float kd;

    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float pout;
    float iout;
    float dout;
    float dbuf[3];
    float error[3];
} foc_pid_t;

void pid_init(foc_pid_t *pid, uint8_t mode, const float gains[3], float max_out, float max_iout);
float pid_calc(foc_pid_t *pid, float ref, float set);
void pid_clear(foc_pid_t *pid);
void foc_speed_pid_init(foc_pid_t *pid);

#ifdef __cplusplus
}
#endif

#endif
