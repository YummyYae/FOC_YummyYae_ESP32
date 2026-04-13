#include "BspDriver.h"
#include "FOC_UTILS.h"
#include "tim.h"
#include "usart.h"      // STM32 HAL���UARTͷ�ļ�
#include "stdio.h"
#include "string.h"
#include "foc_calibrate.h"

#define VOLT_SUPPLY 12 //����ĸ�ߵ�ѹ
#define MAX_VOLT 6.9f  //���ƹ����ѹ(ƽ��ֵ)�����ΪVOLT_SUPPLY/��3; 4010���:6.9V, 2804���:4V

float targetVotage = 0;    //��ǰĿ���ѹ
uint8_t beepPlaying = 0;   //��ǰ�Ƿ��ڷ���״̬
uint8_t motorID = 1;       //��ʼ���ID
uint8_t ledBlink = 1;      //led�Ƿ���������˸ģʽ
float speed = 0;           //������������������ת��
float filteredAngle = 0;   //���˲��õ���ת�ӽǶ�
uint32_t lastRecvTime = 0; //�ϴ��յ�CAN����֡��ʱ��
Kalman angleFilter; //�������˲��ṹ��

void Kalman_Init(Kalman* p,float T_Q,float T_R)
{
	p->X_last = (float)0;
	p->P_last = 0;
	p->Q = T_Q;
	p->R = T_R;
	p->A = 1;
	p->H = 1;
	p->X_mid = p->X_last;
}

float Kalman_Filter(Kalman* p,float dat)
{
	if(!p) return 0;
	p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
	p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
	p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
	p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
	p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
	p->P_last = p->P_now;
	p->X_last = p->X_now;
	return p->X_now;
}

void CAN_Init()
{
//	CAN_FilterTypeDef filter;
//	filter.FilterActivation = ENABLE;
//	filter.FilterMode = CAN_FILTERMODE_IDMASK;
//	filter.FilterMode = CAN_FILTERMODE_IDMASK;
//	filter.FilterScale = CAN_FILTERSCALE_32BIT;
//	filter.FilterIdHigh = 0x0000;
//	filter.FilterIdLow = 0x0000;
//	filter.FilterMaskIdHigh = 0x0000;
//	filter.FilterMaskIdLow = 0x0000;
//	filter.FilterBank = 0;
//	filter.FilterFIFOAssignment = CAN_RX_FIFO0;
//	HAL_CAN_ConfigFilter(&hcan, &filter);
//	HAL_CAN_Start(&hcan);
//	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//����һ����������֡
void CAN_SendState(float angle, float speed)
{
//	CAN_TxHeaderTypeDef header;
//	header.StdId = motorID + 0x100;
//	header.IDE = CAN_ID_STD;
//	header.RTR = CAN_RTR_DATA;
//	header.DLC = 8;
//	
//	uint8_t data[8];
//	memcpy(data,&(int32_t){angle*1000},4); //�Ƕ����ݷ���ǰ�ĸ��ֽ�
//	memcpy(&data[4],&(int16_t){speed*10},2); //ת�����ݷ��ڵ�5-6�ֽ�
//	
//	uint32_t mailbox;
//	HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
}

//CAN�յ����ݵ��жϻص�

//	else if(header.StdId == 0x200 && motorID > 4) //ID=5~8����0x200����֡
//	{
//		targetVotage = *(int16_t*)&rxData[(motorID-5)*2] / 1000.0f;
//		lastRecvTime = HAL_GetTick();
//	}

void SimpleFOC_Init()
{
//	MagneticSensor_Init(); //��ʼ���Ŵ�����
//	
	foc_params.voltage_power_supply=VOLT_SUPPLY; //�趨FOC�������
	voltage_limit=MAX_VOLT;
	voltage_sensor_align=2;
	targetVotage=0;
//	
	Motor_init(); //��ʼ�������Ϣ
	Motor_initFOC(); //��ʼ��FOC����
//	
//	//motorID = Flash_ReadMotorID(); //��Flash��ȡ���ID
	motorID = motorID ? motorID : 1;
}

void Motor_SpeedCalcProcess()
{
	const uint8_t speedLpfLen = 5;
	static float speedLpfBuf[speedLpfLen] = {0}; //���5��filteredAngle
	
	float angle = filteredAngle;
	float curSpeed = (angle-speedLpfBuf[0])*1000/speedLpfLen/_2PI*60;
	speed = curSpeed;
	
	for(uint8_t i=0; i<speedLpfLen-1; i++)
		speedLpfBuf[i] = speedLpfBuf[i+1];
	speedLpfBuf[speedLpfLen-1] = angle;
}

void uart_printf(int data)
{
	char temp[64];
	sprintf(temp, "%d\n", data);
	HAL_UART_Transmit(&huart3, (const unsigned char*)temp, strlen((const char*)temp), 1000);
}

// ��ӡ��������������ʽ������1,����2,����3\n
void uart_printf2f(float f1, float f2)
{
	char temp[64];
	sprintf(temp, "%.3f,%.3f\n", f1, f2);
	HAL_UART_Transmit(&huart3, (const unsigned char*)temp, strlen((const char*)temp), 1000);
}

// ��ӡ��������������ʽ������1,����2,����3\n
void uart_printf3f(float f1, float f2, float f3)
{
	char temp[64];
	sprintf(temp, "%.3f,%.3f,%.3f\n", f1, f2, f3);
	HAL_UART_Transmit(&huart3, (const unsigned char*)temp, strlen((const char*)temp), 1000);
}

void uart_printf6f(float f1, float f2, float f3,float f4, float f5, float f6)
{
	char temp[128];
	sprintf(temp, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", f1, f2, f3, f4, f5, f6);
	HAL_UART_Transmit(&huart3, (const unsigned char*)temp, strlen((const char*)temp), 1000);
}


