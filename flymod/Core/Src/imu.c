#include "imu.h"
#include "tim.h"

//--------------------------------------------------

//姿态数据结构体
attitude_t attitude = 
{
  .q[0] = 1.0f,
  .q[1] = 0.0f,
  .q[2] = 0.0f,
  .q[3] = 0.0f
};

float ex, ey, ez;
float exInt=0,eyInt=0,ezInt=0;

float Kp = 4.0f;	//比例增益
float Ki = 0.001f;//积分增益

//--------------------------------------------------

//******************************************************************************
//函数名:invSqrt(void)
//描述:求平方根的倒数
//该函数是经典的Carmack求平方根算法,效率极高,使用魔数0x5f375a86
//******************************************************************************
float invSqrt(float number)
{
  volatile long i;
  volatile float x, y;

  x = number * 0.5F;
  y = number;
  i = * (( long * ) &y);
  i = 0x5f375a86 - ( i >> 1 );
  y = * (( float * ) &i);
  y = y * ( 1.5f - ( x * y * y ) );
  return y;
}

//******************************************************************************
//四元数法
//姿态更新
//******************************************************************************
void IMUupdate(float mpu_acc[3],float mpu_gyro[3], float q[4], float dcm[3][3], float dt)
{
  float norm;
  float acc[3],gyro[3];
  
  //检查数据是否更新完成
  if(!mpu6500.updata) return;
  
  //转移数据再处理，防止改变原始数据，影响观察
  for(uint8_t i=0;i<3;i++)
  {
    acc[i]  = mpu_acc[i];
    
    //将角度制转换成弧度制,很重要,后面算法基于弧度计算,角度制无法计算
    gyro[i] = mpu_gyro[i] * DEG2RAD;
  }

	///////////////////////////////////////////////////////////////////////////////
	
	//使用加速度计数据对陀螺仪数据进行修正,姿态解算中的深度融合
	//第2步到第7步是一个循环
	//第2步输入数据,由第1步提供
	//第7步输出数据,提供给第8步
	if((acc[0] != 0.0f) || (acc[1] != 0.0f) || (acc[2] != 0.0f))//加这句判断是为了防止求倒时分母为0（自由落体）
  {
    //1.重力加速度归一化
    norm = invSqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);//快速求平方根倒数
    acc[0] = acc[0] * norm;
    acc[1] = acc[1] * norm;
    acc[2] = acc[2] * norm;

    //2.向量差积得出姿态误差
    ex = (acc[1]*dcm[2][2] - acc[2]*dcm[2][1]);								
    ey = (acc[2]*dcm[2][0] - acc[0]*dcm[2][2]);
    ez = (acc[0]*dcm[2][1] - acc[1]*dcm[2][0]);

    //3.对误差进行积分
    exInt = exInt + ex*Ki;
    eyInt = eyInt + ey*Ki;
    ezInt = ezInt + ez*Ki;
      
    //4.互补滤波，姿态误差补偿到角速度上，修正角速度积分飘移
    gyro[0] = gyro[0] + Kp*ex + exInt;
    gyro[1] = gyro[1] + Kp*ey + eyInt;
    gyro[2] = gyro[2] + Kp*ez + ezInt;
  }

	//以上4步为数据融合
	/////////////////////////////////////////////////////////////////////////////////

	//5.一介龙格库塔法更新四元数
	q[0] = q[0] + (-q[1]*gyro[0] - q[2]*gyro[1] - q[3]*gyro[2])*0.5f * dt;
	q[1] = q[1] + (	q[0]*gyro[0] + q[2]*gyro[2] - q[3]*gyro[1])*0.5f * dt;
	q[2] = q[2] + (	q[0]*gyro[1] - q[1]*gyro[2] + q[3]*gyro[0])*0.5f * dt;
	q[3] = q[3] + (	q[0]*gyro[2] + q[1]*gyro[1] - q[2]*gyro[0])*0.5f * dt;

	//6.四元数归一化
	norm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] = q[0] * norm;
	q[1] = q[1] * norm;
	q[2] = q[2] * norm;
	q[3] = q[3] * norm;

	//7.提取四元数的等效余弦矩阵中的重力分量
	dcm[2][0] = 2*(q[1]*q[3] - q[0]*q[2]);							        //矩阵3,1项
	dcm[2][1] = 2*(q[0]*q[1] + q[2]*q[3]);							        //矩阵3,2项
	dcm[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];	//矩阵3,3项

	dcm[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];	//矩阵1,1项
	dcm[1][0] = 2*(q[1]*q[2] + q[0]*q[3]);							        //矩阵2,1项

	//8.四元数转欧拉角
  attitude.deg.Pitch =  asinf(dcm[2][0]) * RAD2DEG;            // pitch
	attitude.deg.Roll  =  atan2f(dcm[2][1], dcm[2][2]) * RAD2DEG;// roll
	attitude.deg.Yaw	 = -atan2f(dcm[1][0], dcm[0][0]) * RAD2DEG;// yaw
  
  mpu6500.updata = false;//数据已使用
}

//***********************************************
//带计时
//***********************************************
void IMUupdate_2(float dt)
{
  #if NEED_TIME == 1
    Start_timing(&htim2);              //开始计时，单位ms
  #endif
  
  IMUupdate(mpu6500.acc.lpf, mpu6500.gyro.value, attitude.q, attitude.dcm, dt);
  
  #if NEED_TIME == 1
    Stop_timing(&htim2, &attitude.dt); //停止计时
  #endif
}
