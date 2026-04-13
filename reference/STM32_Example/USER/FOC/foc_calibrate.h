#ifndef __FOC_CALIBRATE_H
#define __FOC_CALIBRATE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    // 三相电流
    float shaft_angle, electrical_angle;
    // 三相电流
    float iu, iv, iw;
    float iu_offset, iv_offset, iw_offset;
    // 三相PWM占比
    float pwm_u, pwm_v, pwm_w;
    // dq轴电压
    float uq, ud;
    // dq轴电流
    float iq, id;
    // 目标电流
    float target_iq, target_id;
    float motor_speed, target_speed;
    float motor_position, target_position;
    // Clarke/Park变换中间变量
    float i_alpha, i_beta;
    float v_alpha, v_beta;
    // 电机参数
    float voltage_power_supply;
    int pole_pairs;
    long sensor_direction;
    float zero_electric_angle;
    float phase_resistance;
} FOC_Parameters;

// 统一FOC参数实例
extern FOC_Parameters foc_params;

// 构造与析构
void FOC_Parameters_Init(FOC_Parameters *params);
void FOC_Parameters_DeInit(FOC_Parameters *params);

void FOC_Calibrate_Init(void);
void FOC_Calibrate_Run(void);
void update_current_sensor(float electrical_angle);

void setTargetVotage(float target_q, float target_d);
void setTargetI (float targeti_q,float targeti_d);
void setPhaseVoltage(float Uq, float Ud, float angle_el);

void Clarke_Transf(float Current_abc_temp[3],float Current_alpha_beta_temp[2]);
void Park_Transf(float Current_alpha_beta_temp[2],float angle_el,float current_dq_temp[2]);

typedef struct {
    float q;
    float d;
} DQVoltage_s_calib; // Renaming to local or just reuse the definition

extern DQVoltage_s_calib voltage;

#endif // __FOC_CALIBRATE_H

