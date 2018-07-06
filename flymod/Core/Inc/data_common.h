#ifndef __DATA_COMMON_H
#define __DATA_COMMON_H

#include <stdbool.h>
#include <math.h>
#include "stm32f1xx_hal.h"

#define NEED_TIME 1  //1需要计时，0不需要计时

//****************************************************************************
//RTOS各任务频率选择
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

#define DEG2RAD 0.017453293f	//度转弧度 π/180
#define RAD2DEG 57.29578f			//弧度转度 180/π

//----------------------------------------------------------

//****************************************************************************
//-----MPU6500------
//****************************************************************************
//对于三维数据数组，数组内数据均以X、Y、Z或者PIT、ROL、YAW的顺序排列

//1.加速度计----------------------------------------
typedef struct
{
//  lpf_s   lpf[3];
	int16_t raw[3];       //原始采样的数据
  int16_t vector[3];    //轴旋转后的数据
	float   value[3];     //转换单位后的原始数据，单位m/s^2。
	float   lpf[3];       //滤波后m/s^2
	bool    vaild;        //有效置1，无效置0
}acc_s;

//2.温度----------------------------------------
typedef struct
{
  int16_t raw[3];       //原始采样的数据
  float   value[3];     //转换单位后的原始数据，单位摄氏度
  float   lpf[3];       //滤波后的数据
  bool    vaild;        //有效置1，无效置0
}temp_s;

//3.陀螺仪----------------------------------------
typedef struct
{
//	lpf_s   lpf[3];
	int16_t raw[3];        //原始采样的数据
  int16_t vector[3];     //轴旋转后的数据
  
  int16_t cal[3];        //偏移量，用于校准
  int32_t total[3];      //样本的总和
  int64_t error_total[3];//误差平方的总和
  int16_t mean[3];       //样本的平均值
  int32_t error_mean[3]; //误差平方总和的平均值
  
	float   value[3];      //转换单位后的原始数据，单位rad/s。
	float   lpf[3];        //滤波后m/s^2
	bool    vaild;         //有效置1，无效置0
}gyro_s;

//mpu6500数据汇总------------------------------------
typedef struct
{
  acc_s  acc;      //加速度计
  temp_s temp;     //温度
  gyro_s gyro;     //陀螺仪
  
  float  dt;       //耗时
  bool   updata;   //更新完成置1，被读取使用置0
}mpu6500_t;

extern mpu6500_t mpu6500;

//****************************************************************************
//-----IMU------
//****************************************************************************

//3个轴向------------------------------------
typedef struct 
{
  float Pitch;
	float Roll;
  float Yaw;
}Axis3f;

//姿态------------------------------------
typedef struct
{
	Axis3f deg;	     //姿态角，角度。
	Axis3f rad;	     //姿态角，弧度。
  
	float q[4];      //四元数
	float dcm[3][3]; //旋转矩阵
  
  float dt;        //耗时
	bool  vaild;     //有效置1，无效置0
}attitude_t;

extern attitude_t attitude;//姿态数据

//****************************************************************************
//-----串口------
//****************************************************************************
typedef struct
{
	uint8_t  Buff[32]; //数据缓存区
  uint16_t len;      //数据长度
  bool     flag;     //完成标志位
  bool     Task_flag;//任务执行标志位
}Usart_s;

typedef struct
{
  Usart_s Rx;
  Usart_s Tx;
}Usart_t;

extern Usart_t Usart1; //串口1
extern Usart_t Usart2; //串口2

//****************************************************************************
//-----遥控器杆量-----
//****************************************************************************
typedef struct 
{
  uint16_t Thro;
  int16_t  Yaw;
	int16_t  Roll;
	int16_t  Pitch;
	
  bool  lock;  //锁
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
	float Error;      //本次误差
	float PreError;   //上次误差
	float Deriv;      //微分
	float Integ;      //积分
	float iLimit;     //积分限幅
	float OutP;       //< proportional output (debugging)
	float OutI;       //< integral output (debugging)
	float OutD;       //< derivative output (debugging)
	float Output;     //输出
  float OutputLimit;//输出限幅
}PID_Typedef;

typedef struct
{
	PID_Typedef Angle;//角度
	PID_Typedef Rate; //角速度
}ahrs_pid_t;

extern ahrs_pid_t Roll;
extern ahrs_pid_t Pitch;
extern ahrs_pid_t Yaw;

#endif
