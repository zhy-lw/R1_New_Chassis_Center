/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task_Init.h"
#include "semphr.h"
#include "comm_stm32_hal_middle.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SemaphoreHandle_t Remote_semaphore;

SemaphoreHandle_t Radar_semaphore;

uint16_t _stack[12] = {0};
extern TaskHandle_t Remote_Analysis_Handle;
extern TaskHandle_t Uart_Tx_Handle;
extern TaskHandle_t task_handle;
extern TaskHandle_t SendDataPackTask_handle;
extern TaskHandle_t ReceiveDataPackTask_handle;
extern TaskHandle_t ACKTimeoutCheckTask_handle;
extern CommHandle_t *g_comm_handle;
extern TaskHandle_t Auto_Navigatoin_Handle;
extern TaskHandle_t Radar_Handle;
extern TaskHandle_t Arm_Task_Handle;
extern TaskHandle_t Action_Handle;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  Remote_semaphore = xSemaphoreCreateBinary();
	Radar_semaphore = xSemaphoreCreateBinary();
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
	Task_Init();
  /* Infinite loop */
  for(;;)
  {
		_stack[0]=uxTaskGetStackHighWaterMark(Uart_Tx_Handle);
		_stack[1]=uxTaskGetStackHighWaterMark(task_handle);
		_stack[2]=uxTaskGetStackHighWaterMark(SendDataPackTask_handle);
		_stack[3]=uxTaskGetStackHighWaterMark(ReceiveDataPackTask_handle);
		_stack[4]=uxTaskGetStackHighWaterMark(ACKTimeoutCheckTask_handle);
		_stack[5]=uxTaskGetStackHighWaterMark(defaultTaskHandle);
		_stack[6]=uxTaskGetStackHighWaterMark(g_comm_handle->tx_task_handle);
		_stack[7]=uxTaskGetStackHighWaterMark(Remote_Analysis_Handle);
		_stack[8]=uxTaskGetStackHighWaterMark(Auto_Navigatoin_Handle);
		_stack[9]=uxTaskGetStackHighWaterMark(Radar_Handle);
		_stack[10]=uxTaskGetStackHighWaterMark(Arm_Task_Handle);
		_stack[11]=uxTaskGetStackHighWaterMark(Action_Handle);


    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

