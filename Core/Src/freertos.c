/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "iwdg.h"
#include "plc_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osThreadId_t g_freertos_vmTaskHandle;
static const osThreadAttr_t kVmTaskAttr = {
  .name       = "vmScanTask",
  .stack_size = 1024U * 4U,
  .priority   = (osPriority_t) osPriorityAboveNormal,
};
static osThreadId_t g_freertos_commIoTaskHandle;
static const osThreadAttr_t kCommIoTaskAttr = {
  .name       = "commIoTask",
  .stack_size = 512U * 4U,
  .priority   = (osPriority_t) osPriorityBelowNormal,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void vm_scan_task(void *p_argument);
static void comm_io_task(void *p_argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
extern void MX_LWIP_Init(void);
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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  g_freertos_vmTaskHandle = osThreadNew(vm_scan_task, NULL, &kVmTaskAttr);
  g_freertos_commIoTaskHandle = osThreadNew(comm_io_task, NULL, &kCommIoTaskAttr);
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
  /* Load persisted unit/network config before LwIP initializes */
  PLC_HAL_LoadUnitConfig();

  /* init code for USB_DEVICE or LWIP depending on persisted mode */
  if (PLC_HAL_IsEthMode()) {
    MX_LWIP_Init();
  } else {
    MX_USB_DEVICE_Init();
  }
  /* USER CODE BEGIN StartDefaultTask */
  PLC_HAL_Init();

  for (;;) {
    HAL_IWDG_Refresh(&hiwdg);
    PLC_HAL_PollProtocol();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void vm_scan_task(void *p_argument)
{
  (void)p_argument;
  uint32_t tick5msCount = 0U;
  uint32_t lastWake_ms  = osKernelGetTickCount();

  for (;;) {
    PLC_HAL_StepVmTick(tick5msCount);
    tick5msCount++;
    lastWake_ms += 5U;
    osDelayUntil(lastWake_ms);
  }
}

static void comm_io_task(void *p_argument)
{
  (void)p_argument;
  for (;;) {
    PLC_HAL_StepCommIo();
    osDelay(1);
  }
}
/* USER CODE END Application */

