#ifndef BspDriver_H
#define BspDriver_H
/******************************************************************************/
#include "BspDriver.h"
/******************************************************************************/
#include "main.h"


//�������˲��ṹ��
typedef struct{
	float X_last;
	float X_mid;
	float X_now;
	float P_mid;
	float P_now;
	float P_last;
	float kg;
	float A;
	float Q;
	float R;
	float H;
} Kalman;
/******************************************************************************/
// �ⲿ��������
extern float targetVotage;
extern uint8_t beepPlaying;
extern uint8_t motorID;
extern uint8_t ledBlink;
extern float speed;
extern float filteredAngle;
extern uint32_t lastRecvTime;
extern Kalman angleFilter;

// ��������
void Kalman_Init(Kalman* p, float T_Q, float T_R);
float Kalman_Filter(Kalman* p, float dat);
void CAN_Init(void);
// removed CAN
void SimpleFOC_Init(void);
void Motor_SpeedCalcProcess(void);
void uart_printf(int data);
void uart_printf2f(float f1, float f2);
void uart_printf3f(float f1, float f2, float f3);
void uart_printf6f(float f1, float f2, float f3,float f4, float f5, float f6);
// HAL CAN�ص������������������ļ�ʹ�ã�
#include "stm32g4xx_hal.h"

#endif
/******************************************************************************/


