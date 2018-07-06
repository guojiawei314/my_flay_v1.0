#include "ANO_Tech_V4.h"
#include "usart.h"

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义,在发送大于1字节的数据类型时,比如int16、float等,需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

dt_flag_t f;              //需要发送数据的标志
uint8_t data_to_send[50];	//发送数据缓存

extern DMA_HandleTypeDef hdma_usart2_tx;

//**************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------华--丽--的--分--割--线------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////
//**************************************************************************************************


/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
void ANO_DT_Data_Exchange(void)
{
	static uint8_t cnt = 0;
	static uint8_t senser_cnt 	= 10;
	static uint8_t status_cnt 	= 15;
	static uint8_t rcdata_cnt 	= 20;
	static uint8_t motopwm_cnt	= 20;
	static uint8_t power_cnt		=	50;
	
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;
	
	cnt++;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)//版本信息
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)//姿态数据
	{
		f.send_status = 0;
		ANO_DT_Send_Status(attitude.deg.Roll, attitude.deg.Pitch, attitude.deg.Yaw, 0, 0, 0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)//传感器数据
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser(mpu6500.acc.lpf[0],    mpu6500.acc.lpf[1],    mpu6500.acc.lpf[2],
											 mpu6500.gyro.value[0], mpu6500.gyro.value[1], mpu6500.gyro.value[2],
											 0, 0, 0);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(0,0,0,0,0,0,0,0,0,0);
	}
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(1,2,3,4,5,6,7,8);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(123,456);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid1)//内环PID
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1, Roll.Rate.kp,  Roll.Rate.ki,  Roll.Rate.kd,
											 Pitch.Rate.kp, Pitch.Rate.ki, Pitch.Rate.kd,
											 Yaw.Rate.kp,   Yaw.Rate.ki,   Yaw.Rate.kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid2)//外环PID
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2, Roll.Angle.kp,  Roll.Angle.ki,  Roll.Angle.kd,
											 Pitch.Angle.kp, Pitch.Angle.ki, Pitch.Angle.kd,
											 Yaw.Angle.kp,   Yaw.Angle.ki,   Yaw.Angle.kd);
	}
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_pid3)
	{
		f.send_pid3 = 0;
		ANO_DT_Send_PID(3,0,0,0,
											0,0,0,
											0,0,0);
	}
}


//**************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------华--丽--的--分--割--线------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////
//**************************************************************************************************


/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(uint8_t *dataToSend , uint8_t length)
{
  if(!Usart2.Tx.flag)
  {
    Usart2.Tx.len = length;
    USARTx_DMA_SendData(&hdma_usart2_tx, &Usart2.Tx, data_to_send);
  }
}

//***********************************************
//发送CHECK数据
//***********************************************
//static void ANO_DT_Send_Check(uint8_t head, uint8_t check_sum)
//{
//	uint8_t sum=0;
//	
//	data_to_send[0]=0xAA;
//	data_to_send[1]=0xAA;
//	data_to_send[2]=0xEF;
//	data_to_send[3]=2;
//	data_to_send[4]=head;
//	data_to_send[5]=check_sum;

//	for(uint8_t i=0;i<6;i++)
//		sum += data_to_send[i];
//	data_to_send[6]=sum;

//	//////////////////////////////////////////////////////////
//	
//  ANO_DT_Send_Data(data_to_send, 7);
//}


//**************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------华--丽--的--分--割--线------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////
//**************************************************************************************************


//解析函数待移植


//**************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////
//-----------------------------------华--丽--的--分--割--线------------------------------------------
////////////////////////////////////////////////////////////////////////////////////////////////////
//**************************************************************************************************


//***********************************************
//版本信息
//***********************************************
void ANO_DT_Send_Version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0,sum=0;
  
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
  
	//////////////////////////////////////////////////////////
  
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//姿态数据
//***********************************************
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0,sum=0;
	volatile int16_t _temp;
	volatile int32_t _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//////////////////////////////////////////////////////////
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//发送原始9轴数据
//***********************************************
void ANO_DT_Send_Senser(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z)
{
	uint8_t _cnt=0,sum=0;
	volatile int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	//////////////////////////////////////////////////////////
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//
//***********************************************
void ANO_DT_Send_RCData(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0,sum=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
  
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//////////////////////////////////////////////////////////
  
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//发送电池电压和电流
//***********************************************
void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0,sum=0;
	uint16_t temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//////////////////////////////////////////////////////////
  
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//
//***********************************************
void ANO_DT_Send_MotoPWM(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	uint8_t _cnt=0,sum=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	//////////////////////////////////////////////////////////
  
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//气压数据和超声波数据
//***********************************************
void ANO_DT_Send_Senser2(int32_t bar,uint16_t csb)
{
	uint8_t _cnt=0,sum=0,i;
	volatile int32_t _temp2 = bar;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++]=BYTE1(csb);
	data_to_send[_cnt++]=BYTE0(csb);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	//////////////////////////////////////////////////////////
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

//***********************************************
//发送PID数据
//***********************************************
void ANO_DT_Send_PID(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0,sum=0;
	volatile int16_t _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	//////////////////////////////////////////////////////////

	ANO_DT_Send_Data(data_to_send, _cnt);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//**************************************************************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////
