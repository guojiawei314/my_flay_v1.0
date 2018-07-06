#include "pid.h"

ahrs_pid_t Roll; //Roll��PID
ahrs_pid_t Pitch;//Pitch��PID
ahrs_pid_t Yaw;  //Yaw��PID

//**************************************
//PID���ƺ������汾1��ԭ�Ӱ汾��
//target :����ֵ
//measure:����ֵ
//dt     :��λʱ��
//**************************************
//void PID_Common_Update(PID_Typedef *pid, float target, float measure, float dt)
//{
//  float termI,output;
//  
//  //���=����-ʵ��
//	pid->Error  = target - measure;
//  
//  //΢��=(�������-�ϴ����)/����ʱ��
//	pid->Deriv  = (pid->Error - pid->PreError) / dt;
//  
//  pid->PreError = pid->Error;
//  
//  //����+=���*����ʱ��
//	termI = pid->Integ + pid->Error * dt;
//  
//  //�����޷�
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
//  //����޷�
//  if(pid->OutputLimit != 0)//�����Ҫ������޷�
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
//PID���ƺ������汾2������ʦ�汾��
//target :����ֵ
//measure:����ֵ
//dt     :��λʱ��
//**************************************
void PID_Common_Update(PID_Typedef *pid, float target, float measure, float dt)
{
  float termI;
  
  //���=����-ʵ��
	pid->Error  = target - measure;
  
  //΢��=(�������-�ϴ����)/����ʱ��
	pid->Deriv  = (pid->Error - pid->PreError) / dt;
  
  pid->PreError = pid->Error;
  
  pid->OutP = pid->kp * pid->Error;//debug
	pid->OutI = pid->ki * pid->Integ;//debug
	pid->OutD = pid->kd * pid->Deriv;//debug
	
	pid->Output = pid->OutP + pid->OutI + pid->OutD;//��һ�εĻ��֣���һ���ڵ��ͺ�
  
  //����+=���*����ʱ��
	termI = pid->Integ + pid->Error * dt;
  
  //�����޷�
  if(termI > -pid->iLimit && termI < pid->iLimit && pid->Output > -pid->iLimit && pid->Output < pid->iLimit)
    pid->Integ = termI;
}


