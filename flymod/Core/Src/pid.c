#include "pid.h"

ahrs_pid_t Roll; //Roll轴PID
ahrs_pid_t Pitch;//Pitch轴PID
ahrs_pid_t Yaw;  //Yaw轴PID

//**************************************
//PID控制函数，版本1（原子版本）
//target :期望值
//measure:测量值
//dt     :单位时间
//**************************************
//void PID_Common_Update(PID_Typedef *pid, float target, float measure, float dt)
//{
//  float termI,output;
//  
//  //误差=期望-实际
//	pid->Error  = target - measure;
//  
//  //微分=(本次误差-上次误差)/周期时间
//	pid->Deriv  = (pid->Error - pid->PreError) / dt;
//  
//  pid->PreError = pid->Error;
//  
//  //积分+=误差*所耗时间
//	termI = pid->Integ + pid->Error * dt;
//  
//  //积分限幅
//  if(termI > pid->iLimit)
//    termI = pid->iLimit;
//  else if(termI < -pid->iLimit)
//    termI = -pid->iLimit;
//  
//  pid->Integ = termI;
//  
//  pid->OutP = pid->kp * pid->Error;//debug
//	pid->OutI = pid->ki * pid->Integ;//debug
//	pid->OutD = pid->kd * pid->Deriv;//debug
//	
//	output = pid->OutP + pid->OutI + pid->OutD;
//  
//  //输出限幅
//  if(pid->OutputLimit != 0)//如果需要对输出限幅
//  {
//    if(output > pid->OutputLimit)
//      output = pid->OutputLimit;
//    else if(output < -pid->OutputLimit)
//      output = -pid->OutputLimit;
//  }
//  
//  pid->Output = output;
//}

//**************************************
//PID控制函数，版本2（蒋大师版本）
//target :期望值
//measure:测量值
//dt     :单位时间
//**************************************
void PID_Common_Update(PID_Typedef *pid, float target, float measure, float dt)
{
  float termI;
  
  //误差=期望-实际
	pid->Error  = target - measure;
  
  //微分=(本次误差-上次误差)/周期时间
	pid->Deriv  = (pid->Error - pid->PreError) / dt;
  
  pid->PreError = pid->Error;
  
  pid->OutP = pid->kp * pid->Error;//debug
	pid->OutI = pid->ki * pid->Integ;//debug
	pid->OutD = pid->kd * pid->Deriv;//debug
	
	pid->Output = pid->OutP + pid->OutI + pid->OutD;//上一次的积分，有一周期的滞后
  
  //积分+=误差*所耗时间
	termI = pid->Integ + pid->Error * dt;
  
  //积分限幅
  if(termI > -pid->iLimit && termI < pid->iLimit && pid->Output > -pid->iLimit && pid->Output < pid->iLimit)
    pid->Integ = termI;
}


