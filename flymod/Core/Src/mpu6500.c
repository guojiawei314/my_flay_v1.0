#include "mpu6500.h"
#include "spi.h"
#include "tim.h"

mpu6500_t mpu6500;

//8位数据合并成16位数据
#define BYTE16(Type, ByteH, ByteL)  ((Type)((((uint16_t)ByteH)<<8) | ((uint16_t)ByteL)))

//***********************************************
//初始化MPU6500
//***********************************************
bool MPU6500_Init(void)
{
  if(MPU6500_Read_Byte_SPI(MPU_WHO_AM_I)==0x70)
  {
    MPU6500_Write_Byte_SPI(PWR_MGMT_1,0x81);		    //芯片复位，并自动选择最佳时钟源。Auto selects the best available clock source
    HAL_Delay(100);
    MPU6500_Write_Byte_SPI(PWR_MGMT_2,0x00);        //打开加速度计和陀螺仪
    HAL_Delay(100);
    MPU6500_Write_Byte_SPI(SIGNAL_PATH_RESET,0x07); //陀螺仪，加速度计，温度计复位。Reset
    HAL_Delay(100);
    MPU6500_Write_Byte_SPI(USER_CTRL,0x01);			    //清除所有传感器数据。clears all the sensor registers.
    HAL_Delay(100);
    
    MPU6500_Write_Byte_SPI(SMPLRT_DIV,0x00);		    //采样频率1000/(1+0)=1000Hz（最大采样速度）。SAMPLE_RATE =INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)  where INTERNAL_SAMPLE_RATE = 1kHz.
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(CONFIG,0x03);			      //陀螺仪和温度计的数字低通滤波器，陀螺仪：Bandwidth 41Hz,delay 5.9ms，温度计：Bandwidth 42Hz,delay 4.8ms
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(GYRO_CONFIG,0x18);		    //陀螺仪测量范围+-2000dps，并启用数字低通滤波。FCHOICE_B[1:0] is 2’b00.
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(ACCEL_CONFIG,0x10);		  //加速度计测量范围+-8g
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(ACCEL_CONFIG_2,0x04);	  //加速度计数字低通滤波：Bandwidth 20Hz,delay 19.80ms
    HAL_Delay(5);
    
    return true; //成功
  }
	else 
    return false;//失败
}

//**************************************************
//读1byte数据
//**************************************************
uint8_t MPU6500_Read_Byte_SPI(uint8_t reg)
{
	uint8_t byte=0;
	
	MPU6500_CS_L;
	SPI1_ReadWrite_Byte(0x80|reg);
	byte=SPI1_ReadWrite_Byte(0xff);
	MPU6500_CS_H;
	
	return byte;
}

//**************************************************
//写1byte数据
//**************************************************
void MPU6500_Write_Byte_SPI(uint8_t reg,uint8_t data)
{
	MPU6500_CS_L;
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
	MPU6500_CS_H;
}

//**************************************************
//获取MPU65000 6个轴和温度原始数据
//**************************************************
void MPU6500_Get_Raw_Data(int16_t *acc, int16_t *temp, int16_t *gyro)
{
  uint8_t buf[14];
  
  MPU6500_CS_L;
  
	SPI1_ReadWrite_Byte(0x80|ACCEL_XOUT_H);
  SPI1_ReadWrite_nByte(0, buf, 14);       //读取传感器原始数据
  
  //高低8位数据合并
  acc[0] = BYTE16(int16_t, buf[0], buf[1]);
  acc[1] = BYTE16(int16_t, buf[2], buf[3]);
  acc[2] = BYTE16(int16_t, buf[4], buf[5]);
  
  temp[0] = BYTE16(int16_t, buf[6], buf[7]);
  
  gyro[0] = BYTE16(int16_t, buf[8], buf[9]);
  gyro[1] = BYTE16(int16_t, buf[10], buf[11]);
  gyro[2] = BYTE16(int16_t, buf[12], buf[13]);
  
  MPU6500_CS_H;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

//***********************************************
//更新Acc和Gyro数据，并进行单位转换
//***********************************************
void Sensors_Acc_Gyro_Update(void)
{
  //原始数据获取
  MPU6500_Get_Raw_Data(mpu6500.acc.raw, mpu6500.temp.raw, mpu6500.gyro.raw);
  
  //旋转，将mpu芯片的轴向旋转，和机体轴向对应
	mpu6500.acc.vector[0] = -mpu6500.acc.raw[1];
	mpu6500.acc.vector[1] =  mpu6500.acc.raw[0];
	mpu6500.acc.vector[2] =  mpu6500.acc.raw[2];
	
	mpu6500.gyro.vector[0] = -mpu6500.gyro.raw[1];
	mpu6500.gyro.vector[1] =  mpu6500.gyro.raw[0];
	mpu6500.gyro.vector[2] =  mpu6500.gyro.raw[2];
  
  MPU6500_Gyro_Cal_Update();//每当机身处于静止时，就可以更新陀螺仪偏移量用于校准
  
  //ACC值域转换（单位转换）
	mpu6500.acc.value[0] = (float)mpu6500.acc.vector[0] / MPU6500_ACC_LSB_8G;
	mpu6500.acc.value[1] = (float)mpu6500.acc.vector[1] / MPU6500_ACC_LSB_8G;
	mpu6500.acc.value[2] = (float)mpu6500.acc.vector[2] / MPU6500_ACC_LSB_8G;
  
  Filter_One(mpu6500.acc.value, mpu6500.acc.lpf); //一阶滞后滤波
	
	//GYRO值域转换（单位转换）
	mpu6500.gyro.value[0] = ((float)(mpu6500.gyro.vector[0] - mpu6500.gyro.cal[0])) / MPU6500_GYRO_LSB_2000;
	mpu6500.gyro.value[1] = ((float)(mpu6500.gyro.vector[1] - mpu6500.gyro.cal[1])) / MPU6500_GYRO_LSB_2000;
	mpu6500.gyro.value[2] = ((float)(mpu6500.gyro.vector[2] - mpu6500.gyro.cal[2])) / MPU6500_GYRO_LSB_2000;
  
  mpu6500.updata=true; //更新完成
}

//***********************************************
//带计时
//***********************************************
void Sensors_Acc_Gyro_Update_2(void)
{
  #if NEED_TIME == 1
    Start_timing(&htim2);             //开始计时，单位ms
  #endif
  
  Sensors_Acc_Gyro_Update();
  
  #if NEED_TIME == 1
    Stop_timing(&htim2, &mpu6500.dt); //停止计时
  #endif
}

//***********************************************
//更新陀螺仪偏移量，用于陀螺仪校准
//***********************************************
void MPU6500_Gyro_Cal_Update(void)
{
	static uint16_t sample=0;            //样本统计
	static int16_t  gyro_last[3]={0,0,0};//记录上一次的数据
  
	int16_t gyro_error[3]={0,0,0};       //两次相邻数据的误差

  //500个样本
	if(sample<500)
	{
		//计算样本总和
		mpu6500.gyro.total[0] += mpu6500.gyro.vector[0];
		mpu6500.gyro.total[1] += mpu6500.gyro.vector[1];
		mpu6500.gyro.total[2] += mpu6500.gyro.vector[2];

		//计算两次数据的误差
		gyro_error[0] = mpu6500.gyro.vector[0] - gyro_last[0];
		gyro_error[1] = mpu6500.gyro.vector[1] - gyro_last[1];
		gyro_error[2] = mpu6500.gyro.vector[2] - gyro_last[2];

		//记录本次数据，用于下次计算误差
		gyro_last[0] = mpu6500.gyro.vector[0];
		gyro_last[1] = mpu6500.gyro.vector[1];
		gyro_last[2] = mpu6500.gyro.vector[2];
    
		//计算误差的平方和（平方使误差均为正）
		mpu6500.gyro.error_total[0] += (gyro_error[0] * gyro_error[0]);
		mpu6500.gyro.error_total[1] += (gyro_error[1] * gyro_error[1]);
		mpu6500.gyro.error_total[2] += (gyro_error[2] * gyro_error[2]);

		sample++;
	}
	else
	{
		//样本均值
		mpu6500.gyro.mean[0] = mpu6500.gyro.total[0]/sample;
		mpu6500.gyro.mean[1] = mpu6500.gyro.total[1]/sample;
		mpu6500.gyro.mean[2] = mpu6500.gyro.total[2]/sample;

		//误差平方和的均值，用于判断样本的有效性
		mpu6500.gyro.error_mean[0] = mpu6500.gyro.error_total[0]/sample;
		mpu6500.gyro.error_mean[1] = mpu6500.gyro.error_total[1]/sample;
		mpu6500.gyro.error_mean[2] = mpu6500.gyro.error_total[2]/sample;

		//三个方向的误差平方和的均值很小时可以认为是静止状态
		if((mpu6500.gyro.error_mean[0] + mpu6500.gyro.error_mean[1] + mpu6500.gyro.error_mean[2]) < 10)
		{
			//获得有效校准偏移量
			mpu6500.gyro.cal[0]=mpu6500.gyro.mean[0];
			mpu6500.gyro.cal[1]=mpu6500.gyro.mean[1];
			mpu6500.gyro.cal[2]=mpu6500.gyro.mean[2];
		}

		//清零0，用于下次计算
		mpu6500.gyro.total[0]=0;
		mpu6500.gyro.total[1]=0;
		mpu6500.gyro.total[2]=0;
		
		mpu6500.gyro.error_total[0]=0;
		mpu6500.gyro.error_total[1]=0;
		mpu6500.gyro.error_total[2]=0;
		
		sample=0;
	}
}

//***********************************************
// 一阶滞后滤波
//***********************************************
void Filter_One(float *in, float *out)
{
  for(uint8_t i=0;i<3;i++)
    out[i] = out[i] + 0.05f * (in[i] - out[i]);
}
