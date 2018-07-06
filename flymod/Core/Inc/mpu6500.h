#ifndef __MPU6500_H
#define __MPU6500_H

#include "stm32f1xx_hal.h"
#include "data_common.h"

//-------------------------------------------------------------

#define	SELF_TEST_X_GYRO			0X00	//自检
#define	SELF_TEST_Y_GYRO			0X01
#define	SELF_TEST_Z_GYRO			0X02
#define	SELF_TEST_X_ACCEL			0X0D
#define	SELF_TEST_Y_ACCEL			0X0E
#define	SELF_TEST_Z_ACCEL			0X0F

#define SMPLRT_DIV						0X19	//采样频率
#define CONFIG								0X1A	//配置寄存器，陀螺仪低通滤波
#define GYRO_CONFIG						0X1B	//陀螺仪配置寄存器
#define ACCEL_CONFIG					0X1C	//加速度计配置寄存器
#define	ACCEL_CONFIG_2				0X1D	//加速度计低通滤波

#define FIFO_EN								0X23	//FIFO使能寄存器

#define INT_PIN_CFG						0X37	//中断/旁路设置寄存器
#define INT_ENABLE						0X38	//中断使能寄存器

#define ACCEL_XOUT_H					0X3B	//加速度值,X轴高8位寄存器
#define ACCEL_XOUT_L					0X3C	//加速度值,X轴低8位寄存器
#define ACCEL_YOUT_H					0X3D	//加速度值,Y轴高8位寄存器
#define ACCEL_YOUT_L					0X3E	//加速度值,Y轴低8位寄存器
#define ACCEL_ZOUT_H					0X3F	//加速度值,Z轴高8位寄存器
#define ACCEL_ZOUT_L					0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUT_H				0X41	//温度值高8位寄存器
#define MPU_TEMP_OUT_L				0X42	//温度值低8位寄存器

#define GYRO_XOUT_H						0X43	//陀螺仪值,X轴高8位寄存器
#define GYRO_XOUT_L						0X44	//陀螺仪值,X轴低8位寄存器
#define GYRO_YOUT_H						0X45	//陀螺仪值,Y轴高8位寄存器
#define GYRO_YOUT_L						0X46	//陀螺仪值,Y轴低8位寄存器
#define GYRO_ZOUT_H						0X47	//陀螺仪值,Z轴高8位寄存器
#define GYRO_ZOUT_L						0X48	//陀螺仪值,Z轴低8位寄存器

#define SIGNAL_PATH_RESET			0X68	//陀螺仪，加速度计，温度计复位

#define USER_CTRL							0X6A	//用户控制寄存器
#define PWR_MGMT_1						0X6B	//电源管理寄存器1
#define PWR_MGMT_2						0X6C	//电源管理寄存器2

#define MPU_WHO_AM_I					0X75	//器件ID寄存器，值0x70

//-------------------------------------------------------------
//传感器不同采样范围下单位转换的映射关系
#define MPU6500_ACC_LSB_2G  16384.f //65536/4
#define MPU6500_ACC_LSB_4G  8192.f  //65536/8
#define MPU6500_ACC_LSB_8G  4096.f  //65536/16
#define MPU6500_ACC_LSB_16G 2048.f  //65536/32

#define MPU6500_GYRO_LSB_250  131.072f //65536/500
#define MPU6500_GYRO_LSB_500  65.536f  //65536/1000
#define MPU6500_GYRO_LSB_1000 32.768f  //65536/2000
#define MPU6500_GYRO_LSB_2000 16.384f  //65536/4000

//-------------------------------------------------------------

#define MPU6500_CS_H  HAL_GPIO_WritePin(SPI1_CS_MPU6500_GPIO_Port, SPI1_CS_MPU6500_Pin, GPIO_PIN_SET)
#define MPU6500_CS_L  HAL_GPIO_WritePin(SPI1_CS_MPU6500_GPIO_Port, SPI1_CS_MPU6500_Pin, GPIO_PIN_RESET)

//-------------------------------------------------------------

bool MPU6500_Init(void);

uint8_t MPU6500_Read_Byte_SPI(uint8_t reg);
void MPU6500_Write_Byte_SPI(uint8_t reg,uint8_t data);

void MPU6500_Get_Raw_Data(int16_t *acc, int16_t *temp, int16_t *gyro);

void Sensors_Acc_Gyro_Update(void);
void Sensors_Acc_Gyro_Update_2(void);
void MPU6500_Gyro_Cal_Update(void);

void Filter_One(float *in, float *out);

#endif
