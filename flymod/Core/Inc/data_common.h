#ifndef __DATA_COMMON_H
#define __DATA_COMMON_H

#include <stdbool.h>
#include <math.h>
#include "stm32f1xx_hal.h"

#define NEED_TIME 1  //1��Ҫ��ʱ��0����Ҫ��ʱ

//****************************************************************************
//RTOS������Ƶ��ѡ��
//****************************************************************************
#define RATE_2_HZ		  2
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000

#define MAIN_LOOP_RATE 	RATE_1000_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK)  ((TICK % (MAIN_LOOP_RATE / RATE_HZ)) == 0)

extern float freertos_main_dt;
extern uint8_t data_to_send[50];

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

#define M_PI    3.14159265358979323846f

#define DEG2RAD 0.017453293f	//��ת���� ��/180
#define RAD2DEG 57.29578f			//����ת�� 180/��

//----------------------------------------------------------

//****************************************************************************
//-----MPU6500------
//****************************************************************************
//������ά�������飬���������ݾ���X��Y��Z����PIT��ROL��YAW��˳������

//1.���ٶȼ�----------------------------------------
typedef struct
{
//  lpf_s   lpf[3];
	int16_t raw[3];       //ԭʼ����������
  int16_t vector[3];    //����ת�������
	float   value[3];     //ת����λ���ԭʼ���ݣ���λm/s^2��
	float   lpf[3];       //�˲���m/s^2
	bool    vaild;        //��Ч��1����Ч��0
}acc_s;

//2.�¶�----------------------------------------
typedef struct
{
  int16_t raw[3];       //ԭʼ����������
  float   value[3];     //ת����λ���ԭʼ���ݣ���λ���϶�
  float   lpf[3];       //�˲��������
  bool    vaild;        //��Ч��1����Ч��0
}temp_s;

//3.������----------------------------------------
typedef struct
{
//	lpf_s   lpf[3];
	int16_t raw[3];        //ԭʼ����������
  int16_t vector[3];     //����ת�������
  
  int16_t cal[3];        //ƫ����������У׼
  int32_t total[3];      //�������ܺ�
  int64_t error_total[3];//���ƽ�����ܺ�
  int16_t mean[3];       //������ƽ��ֵ
  int32_t error_mean[3]; //���ƽ���ܺ͵�ƽ��ֵ
  
	float   value[3];      //ת����λ���ԭʼ���ݣ���λrad/s��
	float   lpf[3];        //�˲���m/s^2
	bool    vaild;         //��Ч��1����Ч��0
}gyro_s;

//mpu6500���ݻ���------------------------------------
typedef struct
{
  acc_s  acc;      //���ٶȼ�
  temp_s temp;     //�¶�
  gyro_s gyro;     //������
  
  float  dt;       //��ʱ
  bool   updata;   //���������1������ȡʹ����0
}mpu6500_t;

extern mpu6500_t mpu6500;

//****************************************************************************
//-----IMU------
//****************************************************************************

//3������------------------------------------
typedef struct 
{
  float Pitch;
	float Roll;
  float Yaw;
}Axis3f;

//��̬------------------------------------
typedef struct
{
	Axis3f deg;	     //��̬�ǣ��Ƕȡ�
	Axis3f rad;	     //��̬�ǣ����ȡ�
  
	float q[4];      //��Ԫ��
	float dcm[3][3]; //��ת����
  
  float dt;        //��ʱ
	bool  vaild;     //��Ч��1����Ч��0
}attitude_t;

extern attitude_t attitude;//��̬����

//****************************************************************************
//-----����------
//****************************************************************************
typedef struct
{
	uint8_t  Buff[32]; //���ݻ�����
  uint16_t len;      //���ݳ���
  bool     flag;     //��ɱ�־λ
  bool     Task_flag;//����ִ�б�־λ
}Usart_s;

typedef struct
{
  Usart_s Rx;
  Usart_s Tx;
}Usart_t;

extern Usart_t Usart1; //����1
extern Usart_t Usart2; //����2

//****************************************************************************
//-----ң��������-----
//****************************************************************************
typedef struct 
{
  uint16_t Thro;
  int16_t  Yaw;
	int16_t  Roll;
	int16_t  Pitch;
	
  bool  lock;  //��
}remote_t;

extern remote_t remote;

//****************************************************************************
//-----PID------
//****************************************************************************
typedef struct
{
	float kp;
	float ki;
	float kd;
	float Error;      //�������
	float PreError;   //�ϴ����
	float Deriv;      //΢��
	float Integ;      //����
	float iLimit;     //�����޷�
	float OutP;       //< proportional output (debugging)
	float OutI;       //< integral output (debugging)
	float OutD;       //< derivative output (debugging)
	float Output;     //���
  float OutputLimit;//����޷�
}PID_Typedef;

typedef struct
{
	PID_Typedef Angle;//�Ƕ�
	PID_Typedef Rate; //���ٶ�
}ahrs_pid_t;

extern ahrs_pid_t Roll;
extern ahrs_pid_t Pitch;
extern ahrs_pid_t Yaw;

#endif
