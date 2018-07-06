/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

#include "data_common.h"
#include "mpu6500.h"
#include "tim.h"
#include "imu.h"
#include "usart.h"
#include "ANO_Tech_V4.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId STABILIZERHandle;
osThreadId USART1_RXHandle;
osThreadId USART2_RXHandle;
osThreadId USART1_TXHandle;
osThreadId USART2_TXHandle;

/* USER CODE BEGIN Variables */

float main_loop_time=0;  //系统主循环周期

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void stabilizerTask(void const * argument);
void usart1RxTask(void const * argument);
void usart2RxTask(void const * argument);
void usart1TxTask(void const * argument);
void usart2TxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of STABILIZER */
  osThreadDef(STABILIZER, stabilizerTask, osPriorityNormal, 0, 128);
  STABILIZERHandle = osThreadCreate(osThread(STABILIZER), NULL);

  /* definition and creation of USART1_RX */
  osThreadDef(USART1_RX, usart1RxTask, osPriorityBelowNormal, 0, 64);
  USART1_RXHandle = osThreadCreate(osThread(USART1_RX), NULL);

  /* definition and creation of USART2_RX */
  osThreadDef(USART2_RX, usart2RxTask, osPriorityBelowNormal, 0, 64);
  USART2_RXHandle = osThreadCreate(osThread(USART2_RX), NULL);

  /* definition and creation of USART1_TX */
  osThreadDef(USART1_TX, usart1TxTask, osPriorityIdle, 0, 64);
  USART1_TXHandle = osThreadCreate(osThread(USART1_TX), NULL);

  /* definition and creation of USART2_TX */
  osThreadDef(USART2_TX, usart2TxTask, osPriorityIdle, 0, 64);
  USART2_TXHandle = osThreadCreate(osThread(USART2_TX), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  
  osThreadSuspend(USART1_RXHandle);//挂起
  Usart1.Rx.Task_flag=true;
  
  osThreadSuspend(USART2_RXHandle);//挂起
  Usart2.Rx.Task_flag=true;
  
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* stabilizerTask function */
void stabilizerTask(void const * argument)
{
  /* USER CODE BEGIN stabilizerTask */
  /* Infinite loop */
  
  uint32_t tick=0;
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, portTICK_RATE_MS);//1ms延时，1000Hz
    
    //mpu6500数据读取，imu更新（500Hz）
    if(RATE_DO_EXECUTE(RATE_500_HZ, tick))
    {
      //更新MPU6500数据
      Sensors_Acc_Gyro_Update_2();
      
      //姿态解算
      IMUupdate_2(0.002f);
      
      //主循环时间计时
      #if NEED_TIME == 1
        Stop_timing(&htim4, &main_loop_time);//单位ms
        Start_timing(&htim4);
      #endif
    }
    
    tick++;//以1000Hz的速度自加，需要49天才会溢出（4,294,967,295）
  }
  /* USER CODE END stabilizerTask */
}

/* usart1RxTask function */
void usart1RxTask(void const * argument)
{
  /* USER CODE BEGIN usart1RxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usart1RxTask */
}

/* usart2RxTask function */
void usart2RxTask(void const * argument)
{
  /* USER CODE BEGIN usart2RxTask */
  /* Infinite loop */
  for(;;)
  {
    USART_Rx_Decode(Usart2.Rx.Buff);
    
    Usart2.Rx.Task_flag=true;
    osThreadSuspend(USART2_RXHandle);//挂起
  }
  /* USER CODE END usart2RxTask */
}

/* usart1TxTask function */
void usart1TxTask(void const * argument)
{
  /* USER CODE BEGIN usart1TxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usart1TxTask */
}

/* usart2TxTask function */
void usart2TxTask(void const * argument)
{
  /* USER CODE BEGIN usart2TxTask */
  /* Infinite loop */
  for(;;)
  {
    ANO_DT_Data_Exchange();//给上位机发送数据
    
    osDelay(1);
  }
  /* USER CODE END usart2TxTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
