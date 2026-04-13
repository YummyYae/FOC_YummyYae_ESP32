/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pidʵ�ֺ�����������ʼ����PID���㺯����
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "datatype.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //������
    fp32 max_iout; //���������

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�

} Pid_t;

extern void PID_init(Pid_t *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(Pid_t *pid, fp32 ref, fp32 set);
extern void PID_clear(Pid_t *pid);
extern void FOC_Current_PID_Init(Pid_t *pid_iq, Pid_t *pid_id);
extern void FOC_Speed_PID_Init(Pid_t *pid_speed);
extern void FOC_Position_PID_Init(Pid_t *pid_position);
extern void setTargetCurrent(float iq, float id);
extern void FOC_Current_PID_Update(float target_id, float target_iq);
extern Pid_t pid_iq, pid_id, pid_speed, pid_position;
#endif
