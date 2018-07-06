#include "mpu6500.h"
#include "spi.h"
#include "tim.h"

mpu6500_t mpu6500;

//8λ���ݺϲ���16λ����
#define BYTE16(Type, ByteH, ByteL)  ((Type)((((uint16_t)ByteH)<<8) | ((uint16_t)ByteL)))

//***********************************************
//��ʼ��MPU6500
//***********************************************
bool MPU6500_Init(void)
{
  if(MPU6500_Read_Byte_SPI(MPU_WHO_AM_I)==0x70)
  {
    MPU6500_Write_Byte_SPI(PWR_MGMT_1,0x81);		    //оƬ��λ�����Զ�ѡ�����ʱ��Դ��Auto selects the best available clock source
    HAL_Delay(100);
    MPU6500_Write_Byte_SPI(PWR_MGMT_2,0x00);        //�򿪼��ٶȼƺ�������
    HAL_Delay(100);
    MPU6500_Write_Byte_SPI(SIGNAL_PATH_RESET,0x07); //�����ǣ����ٶȼƣ��¶ȼƸ�λ��Reset
    HAL_Delay(100);
    MPU6500_Write_Byte_SPI(USER_CTRL,0x01);			    //������д��������ݡ�clears all the sensor registers.
    HAL_Delay(100);
    
    MPU6500_Write_Byte_SPI(SMPLRT_DIV,0x00);		    //����Ƶ��1000/(1+0)=1000Hz���������ٶȣ���SAMPLE_RATE =INTERNAL_SAMPLE_RATE / (1 + SMPLRT_DIV)  where INTERNAL_SAMPLE_RATE = 1kHz.
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(CONFIG,0x03);			      //�����Ǻ��¶ȼƵ����ֵ�ͨ�˲����������ǣ�Bandwidth 41Hz,delay 5.9ms���¶ȼƣ�Bandwidth 42Hz,delay 4.8ms
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(GYRO_CONFIG,0x18);		    //�����ǲ�����Χ+-2000dps�����������ֵ�ͨ�˲���FCHOICE_B[1:0] is 2��b00.
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(ACCEL_CONFIG,0x10);		  //���ٶȼƲ�����Χ+-8g
    HAL_Delay(5);
    MPU6500_Write_Byte_SPI(ACCEL_CONFIG_2,0x04);	  //���ٶȼ����ֵ�ͨ�˲���Bandwidth 20Hz,delay 19.80ms
    HAL_Delay(5);
    
    return true; //�ɹ�
  }
	else 
    return false;//ʧ��
}

//**************************************************
//��1byte����
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
//д1byte����
//**************************************************
void MPU6500_Write_Byte_SPI(uint8_t reg,uint8_t data)
{
	MPU6500_CS_L;
	SPI1_ReadWrite_Byte(reg);
	SPI1_ReadWrite_Byte(data);
	MPU6500_CS_H;
}

//**************************************************
//��ȡMPU65000 6������¶�ԭʼ����
//**************************************************
void MPU6500_Get_Raw_Data(int16_t *acc, int16_t *temp, int16_t *gyro)
{
  uint8_t buf[14];
  
  MPU6500_CS_L;
  
	SPI1_ReadWrite_Byte(0x80|ACCEL_XOUT_H);
  SPI1_ReadWrite_nByte(0, buf, 14);       //��ȡ������ԭʼ����
  
  //�ߵ�8λ���ݺϲ�
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
//����Acc��Gyro���ݣ������е�λת��
//***********************************************
void Sensors_Acc_Gyro_Update(void)
{
  //ԭʼ���ݻ�ȡ
  MPU6500_Get_Raw_Data(mpu6500.acc.raw, mpu6500.temp.raw, mpu6500.gyro.raw);
  
  //��ת����mpuоƬ��������ת���ͻ��������Ӧ
	mpu6500.acc.vector[0] = -mpu6500.acc.raw[1];
	mpu6500.acc.vector[1] =  mpu6500.acc.raw[0];
	mpu6500.acc.vector[2] =  mpu6500.acc.raw[2];
	
	mpu6500.gyro.vector[0] = -mpu6500.gyro.raw[1];
	mpu6500.gyro.vector[1] =  mpu6500.gyro.raw[0];
	mpu6500.gyro.vector[2] =  mpu6500.gyro.raw[2];
  
  MPU6500_Gyro_Cal_Update();//ÿ�������ھ�ֹʱ���Ϳ��Ը���������ƫ��������У׼
  
  //ACCֵ��ת������λת����
	mpu6500.acc.value[0] = (float)mpu6500.acc.vector[0] / MPU6500_ACC_LSB_8G;
	mpu6500.acc.value[1] = (float)mpu6500.acc.vector[1] / MPU6500_ACC_LSB_8G;
	mpu6500.acc.value[2] = (float)mpu6500.acc.vector[2] / MPU6500_ACC_LSB_8G;
  
  Filter_One(mpu6500.acc.value, mpu6500.acc.lpf); //һ���ͺ��˲�
	
	//GYROֵ��ת������λת����
	mpu6500.gyro.value[0] = ((float)(mpu6500.gyro.vector[0] - mpu6500.gyro.cal[0])) / MPU6500_GYRO_LSB_2000;
	mpu6500.gyro.value[1] = ((float)(mpu6500.gyro.vector[1] - mpu6500.gyro.cal[1])) / MPU6500_GYRO_LSB_2000;
	mpu6500.gyro.value[2] = ((float)(mpu6500.gyro.vector[2] - mpu6500.gyro.cal[2])) / MPU6500_GYRO_LSB_2000;
  
  mpu6500.updata=true; //�������
}

//***********************************************
//����ʱ
//***********************************************
void Sensors_Acc_Gyro_Update_2(void)
{
  #if NEED_TIME == 1
    Start_timing(&htim2);             //��ʼ��ʱ����λms
  #endif
  
  Sensors_Acc_Gyro_Update();
  
  #if NEED_TIME == 1
    Stop_timing(&htim2, &mpu6500.dt); //ֹͣ��ʱ
  #endif
}

//***********************************************
//����������ƫ����������������У׼
//***********************************************
void MPU6500_Gyro_Cal_Update(void)
{
	static uint16_t sample=0;            //����ͳ��
	static int16_t  gyro_last[3]={0,0,0};//��¼��һ�ε�����
  
	int16_t gyro_error[3]={0,0,0};       //�����������ݵ����

  //500������
	if(sample<500)
	{
		//���������ܺ�
		mpu6500.gyro.total[0] += mpu6500.gyro.vector[0];
		mpu6500.gyro.total[1] += mpu6500.gyro.vector[1];
		mpu6500.gyro.total[2] += mpu6500.gyro.vector[2];

		//�����������ݵ����
		gyro_error[0] = mpu6500.gyro.vector[0] - gyro_last[0];
		gyro_error[1] = mpu6500.gyro.vector[1] - gyro_last[1];
		gyro_error[2] = mpu6500.gyro.vector[2] - gyro_last[2];

		//��¼�������ݣ������´μ������
		gyro_last[0] = mpu6500.gyro.vector[0];
		gyro_last[1] = mpu6500.gyro.vector[1];
		gyro_last[2] = mpu6500.gyro.vector[2];
    
		//��������ƽ���ͣ�ƽ��ʹ����Ϊ����
		mpu6500.gyro.error_total[0] += (gyro_error[0] * gyro_error[0]);
		mpu6500.gyro.error_total[1] += (gyro_error[1] * gyro_error[1]);
		mpu6500.gyro.error_total[2] += (gyro_error[2] * gyro_error[2]);

		sample++;
	}
	else
	{
		//������ֵ
		mpu6500.gyro.mean[0] = mpu6500.gyro.total[0]/sample;
		mpu6500.gyro.mean[1] = mpu6500.gyro.total[1]/sample;
		mpu6500.gyro.mean[2] = mpu6500.gyro.total[2]/sample;

		//���ƽ���͵ľ�ֵ�������ж���������Ч��
		mpu6500.gyro.error_mean[0] = mpu6500.gyro.error_total[0]/sample;
		mpu6500.gyro.error_mean[1] = mpu6500.gyro.error_total[1]/sample;
		mpu6500.gyro.error_mean[2] = mpu6500.gyro.error_total[2]/sample;

		//������������ƽ���͵ľ�ֵ��Сʱ������Ϊ�Ǿ�ֹ״̬
		if((mpu6500.gyro.error_mean[0] + mpu6500.gyro.error_mean[1] + mpu6500.gyro.error_mean[2]) < 10)
		{
			//�����ЧУ׼ƫ����
			mpu6500.gyro.cal[0]=mpu6500.gyro.mean[0];
			mpu6500.gyro.cal[1]=mpu6500.gyro.mean[1];
			mpu6500.gyro.cal[2]=mpu6500.gyro.mean[2];
		}

		//����0�������´μ���
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
// һ���ͺ��˲�
//***********************************************
void Filter_One(float *in, float *out)
{
  for(uint8_t i=0;i<3;i++)
    out[i] = out[i] + 0.05f * (in[i] - out[i]);
}
