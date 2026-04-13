/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "pid.h"

// FOC电流环PID参数宏定义
#define IQ_PID_KP     12.6f//-0.10f//0.00145f      // Iq环比例系数
#define IQ_PID_KI     2.64f//-0.18f//3.833f      // Iq环积分系数  
#define IQ_PID_KD     0.0f      // Iq环微分系数
#define IQ_PID_MAX_OUT    1000.0f     // Iq环最大输出（电压）
#define IQ_PID_MAX_IOUT   1000.0f      // Iq环最大积分输出

#define ID_PID_KP      12.6f//-0.1f//0.00145f      // Id环比例系数
#define ID_PID_KI      2.64f//-0.18f//3.833f      // Id环积分系数
#define ID_PID_KD     0.0f      // Id环微分系数  
#define ID_PID_MAX_OUT    1000.0f     // Id环最大输出（电压）
#define ID_PID_MAX_IOUT   1000.0f      // Id环最大积分输出

// FOC速度环PID参数宏定义
#define SPEED_PID_KP     0.08f//0.012f       // 速度环比例系数
#define SPEED_PID_KI     0.00f//0.00004f       // 速度环积分系数
#define SPEED_PID_KD     0.24f//0.37// 速度环微分系数
#define SPEED_PID_MAX_OUT    20.0f       // 速度环最大输出（电流A）
#define SPEED_PID_MAX_IOUT   1.0f       // 速度环最大积分输出

// FOC位置环PID宏定义
#define POSITION_PID_KP     15.0f       // 位置环比例系数 (需要根据实际慢慢调)
#define POSITION_PID_KI     0.0f       // 位置环积分系数 (通常位置环只用P，也可以加极少量I)
#define POSITION_PID_KD     0.00f      // 位置环微分系数
#define POSITION_PID_MAX_OUT    120.0f // 位置环最大输出（即最大允许目标速度 rad/s）
#define POSITION_PID_MAX_IOUT   10.0f  // 位置环最大积分输出

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

void PID_init(Pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

fp32 PID_Calc(Pid_t *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}

void PID_clear(Pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**
 * @brief FOC电流环PID初始化函数
 * @param pid_iq Iq环PID结构体指针
 * @param pid_id Id环PID结构体指针  
 * @note 使用宏定义的参数进行初始化，便于调试和修改参数
 */
void FOC_Current_PID_Init(Pid_t *pid_iq, Pid_t *pid_id)
{
    if (pid_iq == NULL || pid_id == NULL)
    {
        return;
    }
    
    // Iq环PID参数数组
    const fp32 iq_pid_param[3] = {IQ_PID_KP, IQ_PID_KI, IQ_PID_KD};
    
    // Id环PID参数数组
    const fp32 id_pid_param[3] = {ID_PID_KP, ID_PID_KI, ID_PID_KD};
    
    // 初始化Iq环PID
    PID_init(pid_iq, PID_POSITION, iq_pid_param, IQ_PID_MAX_OUT, IQ_PID_MAX_IOUT);
    
    // 初始化Id环PID
    PID_init(pid_id, PID_POSITION, id_pid_param, ID_PID_MAX_OUT, ID_PID_MAX_IOUT);
}

/**
 * @brief FOC速度环PID初始化函数
 * @param pid_speed 速度环PID结构体指针  
 * @note 使用宏定义的参数进行初始化，便于调试和修改参数
 */
void FOC_Speed_PID_Init(Pid_t *pid_speed)
{
    if (pid_speed == NULL)
    {
        return;
    }
    
    // 速度环PID参数数组
    const fp32 speed_pid_param[3] = {SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD};
    
    // 初始化速度环PID
    PID_init(pid_speed, PID_POSITION, speed_pid_param, SPEED_PID_MAX_OUT, SPEED_PID_MAX_IOUT);
}

/**
 * @brief FOC位置环PID初始化函数
 * @param pid_position 位置环PID结构体指针  
 */
void FOC_Position_PID_Init(Pid_t *pid_position)
{
    if (pid_position == NULL)
    {
        return;
    }
    
    // 位置环PID参数数组
    const fp32 position_pid_param[3] = {POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD};
    
    // 初始化位置环PID
    PID_init(pid_position, PID_POSITION, position_pid_param, POSITION_PID_MAX_OUT, POSITION_PID_MAX_IOUT);
}

#include "foc_calibrate.h"

void setTargetCurrent(float iq, float id) {
    foc_params.target_iq = iq;
    foc_params.target_id = id;
}

void FOC_Current_PID_Update(float target_id, float target_iq) {
    foc_params.ud = PID_Calc(&pid_id, foc_params.id, target_id);
    foc_params.uq = PID_Calc(&pid_iq, foc_params.iq, target_iq);
}

Pid_t pid_iq, pid_id, pid_speed, pid_position;

