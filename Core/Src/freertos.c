/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "aircraft.h"
#include "ins.h"
#include "remote_keyboard.h"
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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId insTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId modeTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void StartInsTask(void const *argument);
void StartGimbalTask(void const *argument);
void StartModeTask(void const *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of insTask */
    osThreadDef(insTask, StartInsTask, osPriorityHigh, 0, 128);
    insTaskHandle = osThreadCreate(osThread(insTask), NULL);

    /* definition and creation of gimbalTask */
    osThreadDef(gimbalTask, StartGimbalTask, osPriorityAboveNormal, 0, 128);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    /* definition and creation of modeTask */
    osThreadDef(modeTask, StartModeTask, osPriorityHigh, 0, 256);
    modeTaskHandle = osThreadCreate(osThread(modeTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;) {
        osDelay(500);
    }
    /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartInsTask */
/**
 * @brief Function implementing the insTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartInsTask */
void StartInsTask(void const *argument)
{
    /* USER CODE BEGIN StartInsTask */
    INS_Init();
    /* Infinite loop */
    for (;;) {
        INS_Task();
        osDelay(1);
    }
    /* USER CODE END StartInsTask */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
 * @brief Function implementing the gimbalTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void const *argument)
{
    /* USER CODE BEGIN StartGimbalTask */
    /* Infinite loop */
    for (;;) {
        GimbalTask();
        osDelay(5);
    }
    /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartModeTask */
/**
 * @brief Function implementing the modeTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartModeTask */
void StartModeTask(void const *argument)
{
    /* USER CODE BEGIN StartModeTask */
    GimbalInit();
    /* Infinite loop */
    for (;;) {
        ModeTask();
        osDelay(1);
    }
    /* USER CODE END StartModeTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
