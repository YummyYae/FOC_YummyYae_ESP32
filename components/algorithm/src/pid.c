#include "pid.h"

#include <stddef.h>

// Default speed-loop PID gains. Tune on hardware as needed.
#define SPEED_PID_KP 0.06f
#define SPEED_PID_KI 0.001f
#define SPEED_PID_KD 0.08f
#define SPEED_PID_MAX_OUT 24.0f
#define SPEED_PID_MAX_IOUT 14.0f

static float pid_limit_abs(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void pid_init(foc_pid_t *pid, uint8_t mode, const float gains[3], float max_out, float max_iout)
{
    if (pid == NULL || gains == NULL) {
        return;
    }

    pid->mode = mode;
    pid->kp = gains[0];
    pid->ki = gains[1];
    pid->kd = gains[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->dbuf[0] = 0.0f;
    pid->dbuf[1] = 0.0f;
    pid->dbuf[2] = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
}

float pid_calc(foc_pid_t *pid, float ref, float set)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (pid->mode == PID_MODE_POSITION) {
        pid->pout = pid->kp * pid->error[0];
        pid->iout += pid->ki * pid->error[0];
        pid->dbuf[2] = pid->dbuf[1];
        pid->dbuf[1] = pid->dbuf[0];
        pid->dbuf[0] = pid->error[0] - pid->error[1];
        pid->dout = pid->kd * pid->dbuf[0];

        pid->iout = pid_limit_abs(pid->iout, pid->max_iout);
        pid->out = pid->pout + pid->iout + pid->dout;
        pid->out = pid_limit_abs(pid->out, pid->max_out);
    } else {
        pid->pout = pid->kp * (pid->error[0] - pid->error[1]);
        pid->iout = pid->ki * pid->error[0];
        pid->dbuf[2] = pid->dbuf[1];
        pid->dbuf[1] = pid->dbuf[0];
        pid->dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
        pid->dout = pid->kd * pid->dbuf[0];

        pid->out += pid->pout + pid->iout + pid->dout;
        pid->out = pid_limit_abs(pid->out, pid->max_out);
    }

    return pid->out;
}

void pid_clear(foc_pid_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->dbuf[0] = 0.0f;
    pid->dbuf[1] = 0.0f;
    pid->dbuf[2] = 0.0f;
    pid->set = 0.0f;
    pid->fdb = 0.0f;
    pid->out = 0.0f;
    pid->pout = 0.0f;
    pid->iout = 0.0f;
    pid->dout = 0.0f;
}

void foc_speed_pid_init(foc_pid_t *pid)
{
    const float speed_pid_param[3] = {SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD};
    pid_init(pid, PID_MODE_POSITION, speed_pid_param, SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);
}
