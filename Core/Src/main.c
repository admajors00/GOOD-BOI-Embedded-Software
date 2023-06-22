/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <math.h>
#include <semphr.h>
#include "../ECUAL/SERVO.h"
#include "../Robot/ADI_IMU/ADI_IMU.h"
#include "../Robot/LegControl/LegControl.h"
#include "../Robot/LegControl/LegControl_cfg.h"
#include "../Robot/Queue.h"
#include "../Inc/mutexs.h"
#include "../Robot/PiSerialComs/PiSerialComs.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265358979323846
#define RAD_TO_DEG 57.2958
#define DEG_TO_RAD .0175

#define STATE_WAIT 0
#define STATE_GET_TARGET_ANGLES 1
#define STATE_NORMAL_MODE 2
#define STATE_STAND_STILL 3
#define STATE_DIAGONAL_TROTT 4
#define ENABLE_PI_COMS 1
#define ENABLE_LAPTOP_COMS 0
#define numPrevAnglesToAverage 1

volatile int ENABLE_DATA_LOGGING = 0;
volatile int DONE_SENDING = 1;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for xControlLoop */
osThreadId_t xControlLoopHandle;
const osThreadAttr_t xControlLoop_attributes = {
  .name = "xControlLoop",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for yControlLoop */
osThreadId_t yControlLoopHandle;
const osThreadAttr_t yControlLoop_attributes = {
  .name = "yControlLoop",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for moveLegs */
osThreadId_t moveLegsHandle;
const osThreadAttr_t moveLegs_attributes = {
  .name = "moveLegs",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for updateServos */
osThreadId_t updateServosHandle;
const osThreadAttr_t updateServos_attributes = {
  .name = "updateServos",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for readADI_IMU */
osThreadId_t readADI_IMUHandle;
const osThreadAttr_t readADI_IMU_attributes = {
  .name = "readADI_IMU",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for applyServoOffse */
osThreadId_t applyServoOffseHandle;
const osThreadAttr_t applyServoOffse_attributes = {
  .name = "applyServoOffse",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for piComms */
osThreadId_t piCommsHandle;
const osThreadAttr_t piComms_attributes = {
  .name = "piComms",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for checkBuffer */
osThreadId_t checkBufferHandle;
const osThreadAttr_t checkBuffer_attributes = {
  .name = "checkBuffer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for serialMessages_Queue */
osMessageQueueId_t serialMessages_QueueHandle;
const osMessageQueueAttr_t serialMessages_Queue_attributes = {
  .name = "serialMessages_Queue"
};
/* Definitions for servoAngleMutex */
osMutexId_t servoAngleMutexHandle;
const osMutexAttr_t servoAngleMutex_attributes = {
  .name = "servoAngleMutex"
};
/* Definitions for servoOffsetMutex */
osMutexId_t servoOffsetMutexHandle;
const osMutexAttr_t servoOffsetMutex_attributes = {
  .name = "servoOffsetMutex"
};
/* Definitions for accGyroMutex */
osMutexId_t accGyroMutexHandle;
const osMutexAttr_t accGyroMutex_attributes = {
  .name = "accGyroMutex"
};
/* Definitions for uartBuffMutex */
osMutexId_t uartBuffMutexHandle;
const osMutexAttr_t uartBuffMutex_attributes = {
  .name = "uartBuffMutex"
};
/* Definitions for DataReady_Pin */
osEventFlagsId_t DataReady_PinHandle;
const osEventFlagsAttr_t DataReady_Pin_attributes = {
  .name = "DataReady_Pin"
};
/* Definitions for xControllLoop_EventFlag */
osEventFlagsId_t xControllLoop_EventFlagHandle;
const osEventFlagsAttr_t xControllLoop_EventFlag_attributes = {
  .name = "xControllLoop_EventFlag"
};
/* Definitions for yControllLoop_EventFlag */
osEventFlagsId_t yControllLoop_EventFlagHandle;
const osEventFlagsAttr_t yControllLoop_EventFlag_attributes = {
  .name = "yControllLoop_EventFlag"
};
/* Definitions for ControllLoopsFinished_EventFlag */
osEventFlagsId_t ControllLoopsFinished_EventFlagHandle;
const osEventFlagsAttr_t ControllLoopsFinished_EventFlag_attributes = {
  .name = "ControllLoopsFinished_EventFlag"
};
/* Definitions for piDataRx */
osEventFlagsId_t piDataRxHandle;
const osEventFlagsAttr_t piDataRx_attributes = {
  .name = "piDataRx"
};
/* Definitions for searialDataReady */
osEventFlagsId_t searialDataReadyHandle;
const osEventFlagsAttr_t searialDataReady_attributes = {
  .name = "searialDataReady"
};
/* USER CODE BEGIN PV */

enum States {cntrlSys, standStill, wait, test1, test2, layDown, walk};
volatile enum States STATE = standStill;
enum States testNum = test1;
int counter = 0;
int i;
int16_t targetValues[10];

float a = .966;
//use .011 .000015 .00045 for non stepping
const float Kpx = .015 ;//0.01645
const float Kix = .00009;
const float Kdx = .0000005;//

const float Kpy = Kpx;
const float Kiy = Kix;
const float Kdy = Kdx;

float sampleTime = .01;

volatile float currentTick = 0;

float prevTick = 0;
float deltaTicks;
float prevTick_1 = 0;
float deltaTicks_1;



float yAccAngle, yGyroRate, yGyroAngle, yCurrentAngle, yOffset, yTargetAngle;
float yError = 0;
float yErrorSum = 0;
float yPrevError = 0;
const float yErrorLimit = 10000;
float yPrevAngle = 0;
float yPrevAngles[numPrevAnglesToAverage];
float yPrevAngleSum = 0;
float yPrevAnglesAverage = 0;
int   yNumPrevAnglesAveraged = 0;
QUEUE_Queue yPrevAngles_queue;

int yPrevDataCount = 0;

float xAccAngle, xGyroRate, xGyroAngle, xCurrentAngle, xOffset, xTargetAngle;
float xError = 0;
float xErrorSum = 0;
float xPrevError = 0;
const float xErrorLimit = 10000;
float xPrevAngle = 0;
float xPrevAngles[numPrevAnglesToAverage];
float xPrevAngleSum = 0;
float xPrevAnglesAverage = 0;
int   xNumPrevAnglesAveraged = 0;

QUEUE_Queue xPrevAngles_queue;

volatile int state = STATE_WAIT;
int stateCounter = 0;

char UART2_rxBuffer[8];
char laptopStartMsg[] = {0,0,0,0,0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);
void StartTask07(void *argument);
void StartTask08(void *argument);
void StartTask09(void *argument);

/* USER CODE BEGIN PFP */
//void DMATransferComplete(DMA_HandleTypeDef hdma);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_TIM10_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uart_buf_len = sprintf(uart_buf,"<GoodBoi Starting>\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) uart_buf, uart_buf_len, 100);
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&PSC_INPUT_BUFFER[PSC_BUFFER_INDEX], 1);
	//HAL_UART_Receive_IT(&huart2,(uint8_t*) UART2_rxBuffer, 8);
  //HAL_TIM_PWM_MspInit(&htim4);]
  HAL_SPI_MspInit(&hspi3);
  LEG_CONT_initServos();
  /////IS_RTOS_USED = 0;
  LEG_CONT_setPosXYZ(L_1, 0,0,height);
  LEG_CONT_setPosXYZ(L_2, 0,0,height);
  LEG_CONT_setPosXYZ(L_3, 0,0,height);
  LEG_CONT_setPosXYZ(L_4, 0,0,height);
	for(int i=0; i<LEG_CONT_NUM_SERVOS; i++){
		SERVO_MoveTo(i, LEG_CONT_servoAngles[i]);
	}
  //IS_RTOS_USED = 1;
//  HAL_DMA_RegisterCallback(&hspi4, HAL_DMA_XFER_CPLT_CB_ID, &DMATransferComplete);
  //pull cs high

  ADI_IMU_initDevice(hspi3, GPIOG, GPIO_PIN_0, GPIOG, GPIO_PIN_1, GPIOG, GPIO_PIN_2);

  //say something

  spi_xmit_flag = 0;
  spi_recv_flag =0;
  yPrevAngles_queue.front=0;
  yPrevAngles_queue.rear=-1;
  yPrevAngles_queue.itemCount=0;
  xPrevAngles_queue.front=0;
  xPrevAngles_queue.rear=-1;
  xPrevAngles_queue.itemCount=0;

  LEG_CONT_movingForward = 0;
  /* Infinite loop */



 // HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of servoAngleMutex */
  servoAngleMutexHandle = osMutexNew(&servoAngleMutex_attributes);

  /* creation of servoOffsetMutex */
  servoOffsetMutexHandle = osMutexNew(&servoOffsetMutex_attributes);

  /* creation of accGyroMutex */
  accGyroMutexHandle = osMutexNew(&accGyroMutex_attributes);

  /* creation of uartBuffMutex */
  uartBuffMutexHandle = osMutexNew(&uartBuffMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of serialMessages_Queue */
  serialMessages_QueueHandle = osMessageQueueNew (32, sizeof(PSC_MSGQUEUE), &serialMessages_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of xControlLoop */
  xControlLoopHandle = osThreadNew(StartTask02, NULL, &xControlLoop_attributes);

  /* creation of yControlLoop */
  yControlLoopHandle = osThreadNew(StartTask03, NULL, &yControlLoop_attributes);

  /* creation of moveLegs */
  moveLegsHandle = osThreadNew(StartTask04, NULL, &moveLegs_attributes);

  /* creation of updateServos */
  updateServosHandle = osThreadNew(StartTask05, NULL, &updateServos_attributes);

  /* creation of readADI_IMU */
  readADI_IMUHandle = osThreadNew(StartTask06, NULL, &readADI_IMU_attributes);

  /* creation of applyServoOffse */
  applyServoOffseHandle = osThreadNew(StartTask07, NULL, &applyServoOffse_attributes);

  /* creation of piComms */
  piCommsHandle = osThreadNew(StartTask08, NULL, &piComms_attributes);

  /* creation of checkBuffer */
  checkBufferHandle = osThreadNew(StartTask09, NULL, &checkBuffer_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of DataReady_Pin */
  DataReady_PinHandle = osEventFlagsNew(&DataReady_Pin_attributes);

  /* creation of xControllLoop_EventFlag */
  xControllLoop_EventFlagHandle = osEventFlagsNew(&xControllLoop_EventFlag_attributes);

  /* creation of yControllLoop_EventFlag */
  yControllLoop_EventFlagHandle = osEventFlagsNew(&yControllLoop_EventFlag_attributes);

  /* creation of ControllLoopsFinished_EventFlag */
  ControllLoopsFinished_EventFlagHandle = osEventFlagsNew(&ControllLoopsFinished_EventFlag_attributes);

  /* creation of piDataRx */
  piDataRxHandle = osEventFlagsNew(&piDataRx_attributes);

  /* creation of searialDataReady */
  searialDataReadyHandle = osEventFlagsNew(&searialDataReady_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  state = STATE_GET_TARGET_ANGLES;
  //osTimerStart(xControllLoop_TimerHandle, 100u);
  //osTimerStart(yControllLoop_TimerHandle, 100u);
  HAL_TIM_Base_Start_IT(&htim14);
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 72;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 10000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ADI_IMU_device1.DRPin && HAL_SPI_GetState(&ADI_IMU_device1.HSPI) == HAL_SPI_STATE_READY && ADI_IMU_device1.State == ADI_IMU_READY)
    {
    	osEventFlagsSet(DataReady_PinHandle, 0x0001);

//
//
//    	//ADI_IMU_readRegisterScaled();
//    		//ADI_IMU_readRegister(PROD_ID, 1, spi_buf);
//    		//ADI_IMU_burstRead();
//    		//uart_buf_len = sprintf(uart_buf,"%.2X%.2X\r\n\n\n", (uint8_t)spi_buf[0],
//    		//													(uint8_t)spi_buf[1]);
//    		//HAL_UART_Transmit(&huart3, (uint8_t*) uart_buf, uart_buf_len, 100);
//
//
    }
    if(GPIO_Pin == GPIO_PIN_13){
    	stateCounter++;
    	ADI_IMU_device1.State = ADI_IMU_NOT_READY;
		HAL_GPIO_TogglePin(ADI_IMU_device1.RSTPort, ADI_IMU_device1.RSTPin);
		HAL_GPIO_TogglePin(ADI_IMU_device1.RSTPort, ADI_IMU_device1.RSTPin);
		//HAL_Delay(255);
		ADI_IMU_device1.State = ADI_IMU_READY;

//		xTargetAngle = atan2(ADI_IMU_burstReadBufScaled[5], ADI_IMU_burstReadBufScaled[7])*180/PI;
//		yTargetAngle = atan2(ADI_IMU_burstReadBufScaled[6], ADI_IMU_burstReadBufScaled[7])*180/PI;


		xAccAngle = atan2(ADI_IMU_burstReadBufScaled[5], ADI_IMU_burstReadBufScaled[7])*180/PI;
		xGyroRate = ADI_IMU_burstReadBufScaled[2];
		xGyroAngle = xGyroRate*sampleTime;
		xTargetAngle = a*(xPrevAnglesAverage+xGyroAngle)+(1-a)*(xAccAngle);

		yAccAngle = atan2(ADI_IMU_burstReadBufScaled[6], ADI_IMU_burstReadBufScaled[7])*180/PI;
		yGyroRate = ADI_IMU_burstReadBufScaled[3];
		yGyroAngle = yGyroRate*sampleTime;
		yTargetAngle = a*(yPrevAnglesAverage+yGyroAngle)+(1-a)*(yAccAngle);

		xErrorSum = 0;
		yErrorSum = 0;
		xNumPrevAnglesAveraged = 0;
		yNumPrevAnglesAveraged = 0;
		xPrevAnglesAverage = 0;
		yPrevAnglesAverage = 0;

//		if(stateCounter %2){
//			state = STATE_DIAGONAL_TROTT;
//		}else{
//			state = STATE_STAND_STILL;
//		}

//		while(!QUEUE_isEmpty(&xPrevAngles_queue)){
//			QUEUE_removeData(&xPrevAngles_queue);
//		}
//		while(!QUEUE_isEmpty(&yPrevAngles_queue)){
//			QUEUE_removeData(&yPrevAngles_queue);
//		}

    }
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi){

 }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2){
		PSC_BUFFER_INDEX++;
		if(PSC_BUFFER_INDEX>=PSC_BUFFER_SIZE){
			PSC_BUFFER_INDEX=0;
		}
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&PSC_INPUT_BUFFER[PSC_BUFFER_INDEX], 1);


	}
	if(huart == &huart3){
		 //ENABLE_DATA_LOGGING = 1;
		 STATE = testNum;

	}
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	char endchar = '>';
	if(huart == &huart2){
	}
	if(huart == &huart3){
		DONE_SENDING = 1;

	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

		  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the xControlLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
	float imuValue2, imuValue5, imuValue7;
  for(;;){
	if((STATE == cntrlSys || STATE == test2)  && STATE != wait){
		osEventFlagsWait(xControllLoop_EventFlagHandle, 0x0001,  osFlagsWaitAny, 0);

		if(ADI_IMU_device1.HSPI.State == HAL_SPI_STATE_READY)  {
			if(osMutexAcquire(accGyroMutexHandle, 0)==osOK){
				imuValue2 = ADI_IMU_burstReadBufScaled[2];
				imuValue5 = ADI_IMU_burstReadBufScaled[5];
				imuValue7 = ADI_IMU_burstReadBufScaled[7];
				osMutexRelease(accGyroMutexHandle);

				xAccAngle = atan2(imuValue5, imuValue7)*180/PI;
				xGyroRate = imuValue2;
				xGyroAngle = xGyroRate*sampleTime;
				xCurrentAngle = a*(xPrevAnglesAverage+xGyroAngle)+(1-a)*(xAccAngle);
				xError = xCurrentAngle-xTargetAngle;

				if(xErrorSum<xErrorLimit && xErrorSum > -xErrorLimit){
					xErrorSum += xError;
				}
				xOffset = Kpx*xError + Kix*xErrorSum * sampleTime - Kdx*(xGyroRate)/sampleTime;
				xPrevAnglesAverage = xCurrentAngle;
				xPrevError = xError;
			}
		}
		//osDelay(1);
	  }
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the yControlLoop thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	float imuValue3, imuValue6, imuValue7;

	for(;;) {
		if((STATE == cntrlSys || STATE == test2) && STATE != wait){
			osEventFlagsWait(yControllLoop_EventFlagHandle, 0x0001,  osFlagsWaitAny, 0);
			if(osMutexAcquire(accGyroMutexHandle, 0) == osOK){
				imuValue3 = ADI_IMU_burstReadBufScaled[3];
				imuValue6 = ADI_IMU_burstReadBufScaled[6];
				imuValue7 = ADI_IMU_burstReadBufScaled[7];
				osMutexRelease(accGyroMutexHandle);

				yAccAngle = atan2(imuValue6, imuValue7)*180/PI;
				yGyroRate = imuValue3;
				yGyroAngle = yGyroRate * sampleTime;
				yCurrentAngle = a*(yPrevAnglesAverage+yGyroAngle)+(1-a)*(yAccAngle);
				yError = yCurrentAngle-yTargetAngle;

				if(yErrorSum<yErrorLimit && yErrorSum>-yErrorLimit){
					yErrorSum += yError;
				}

				yOffset = Kpy*yError + Kiy*yErrorSum * sampleTime - Kdy*(yGyroRate)/sampleTime;
				yPrevAnglesAverage = yCurrentAngle;
				yPrevError = yError;
			}

		}
	}
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the moveLegs thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	short int temp = 0;
	float startTime;
  float time = 0;
  float max_time = 1000;
  float distance = .5;
  float l1_start = distance;
  float l2_start = distance * 3/4;
  float l3_start = distance / 2;
  float l4_start = distance /4; 
  float pos1 = 0;
  float pos1A = 0;
  float height = 1.5;
  float distance_sqrt = sqrt(distance /2);
  float percentage = 0;

  for(;;){
	  if(STATE != wait){
      if(STATE == walk){
        percentage = time/max_time;
        LEG_CONT_walkingGait_1(L_1, l1_start, distance, percentage, .5,0);
        LEG_CONT_walkingGait_1(L_2, l2_start, distance, percentage, -.5,0);
        LEG_CONT_walkingGait_1(L_3, l3_start, distance, percentage, .5,0);
        LEG_CONT_walkingGait_1(L_4, l4_start, distance, percentage, -.5,0);
        time+=1;
        if(time >= max_time){
          time = 0;
        }
      }
	  	  if(STATE == cntrlSys){
          LEG_CONT_setPosXYZ(L_1, .5-xOffset, -.5-yOffset,1.5);
          LEG_CONT_setPosXYZ(L_2, -.5-xOffset, -.5-yOffset,1.5);
          LEG_CONT_setPosXYZ(L_3, 0,0,.5);
          LEG_CONT_setPosXYZ(L_4, -.5-xOffset,  .5-yOffset,1.5);
	  	  }
	  	  if(STATE == standStill){
          LEG_CONT_setPosXYZ(L_1,  .75, -.75, 1.5);
          LEG_CONT_setPosXYZ(L_2, -.75, -.75 ,1.5);
          LEG_CONT_setPosXYZ(L_3,  .75,  .75, 1.5);
          LEG_CONT_setPosXYZ(L_4, -.75,  .75, 1.5);
	  	  }
	  	  if(STATE == layDown){
	  		  LEG_CONT_setPosXYZ(L_1,  0, -.5, 0);
			    LEG_CONT_setPosXYZ(L_2,  0, -.5, 0);
			    LEG_CONT_setPosXYZ(L_3,  0,  .5, 0);
			    LEG_CONT_setPosXYZ(L_4,  0,  .5, 0);
	  	  }
	  	  if(STATE == test1){
	  		  if(!temp){
	  			  startTime = (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq();
	  			  temp = 1;
	  		  }
	  		  if((float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq()-startTime > 5){
				  for(float i=1.5; i>1.0; i-=.001){
					  LEG_CONT_setPosXYZ(L_3,   .75,   .75,  i);

				  }
				  STATE = wait;
	  		  }

	  	  }if(STATE == test2){
	  		  if(!temp){
	  			  startTime = (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq();
	  			  temp = 1;
	  		  }
	  		  if( (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq() - startTime > 5){

	  			  LEG_CONT_setPosXYZ(L_1, .5-xOffset, 	-.5-yOffset,1.5);
	  			  LEG_CONT_setPosXYZ(L_2, -.5-xOffset, 	-.5-yOffset,1.5);
	  			  LEG_CONT_setPosXYZ(L_3, .5-xOffset,   0,  i);
				    LEG_CONT_setPosXYZ(L_4, -.5-xOffset,  .5-yOffset,1.5);
				  for(float i=0; i<.24; i+=.001){
					  LEG_CONT_setPosXYZ(L_3,   0,   0,  i);
				  }
				  STATE = wait;
	  		  }
	  	  }
	  }
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
* @brief Function implementing the updateServos thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
  /* Infinite loop */
	//float angle =0;
	for(;;){

		if(STATE != wait || STATE == test1 || STATE == test2){
			for(int i=0; i<LEG_CONT_NUM_SERVOS; i++){
				if(osMutexAcquire(servoAngleMutexHandle, 0) == osOK){
					//angle =  /* + LEG_CONT_servoOffsets[i]*/;
					SERVO_MoveTo(i, LEG_CONT_servoAngles[i]);
					osMutexRelease(servoAngleMutexHandle);
				}
			}
		}
	}
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
* @brief Function implementing the readADI_IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	for(;;){

			osEventFlagsWait(DataReady_PinHandle, 0x0001,  osFlagsWaitAny, 0);
			ADI_IMU_burstRead();
			if(osMutexAcquire(accGyroMutexHandle, 0) == osOK){
				ADI_IMU_scaleBurstRead();
				osMutexRelease(accGyroMutexHandle);
				currentTick = (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq();
				deltaTicks_1 = currentTick - prevTick_1;
				prevTick_1 = currentTick;
			}

	}
  /* USER CODE END StartTask06 */
}

/* USER CODE BEGIN Header_StartTask07 */
/**
* @brief Function implementing the applyServoOffse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask07 */
void StartTask07(void *argument)
{
  /* USER CODE BEGIN StartTask07 */
  /* Infinite loop */
  for(;;)
  {
		  osDelay(1);

  }
  /* USER CODE END StartTask07 */
}

/* USER CODE BEGIN Header_StartTask08 */
/**
* @brief Function implementing the piComms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask08 */
void StartTask08(void *argument)
{
  /* USER CODE BEGIN StartTask08 */
  /* Infinite loop */
	PSC_MSGQUEUE msg;

	osStatus_t status;
	char startChar = '<';
	char endChar = '>';
	int messageLen = 0;
	//char[100] response;
  for(;;)
  {
	  if(ENABLE_PI_COMS&&PSC_checkBuffer()){
	  //osEventFlagsWait(serialDataReady_EventFlagHandle, 0x0010,  osFlagsWaitAny, 0);
	  //if(osMutexAcquire(uartBuffMutexHandle, 0) == osOK){
		  HAL_UART_AbortReceive(&huart2);
		  HAL_UART_Receive_IT(&huart2, (uint8_t *)&PSC_INPUT_BUFFER[PSC_BUFFER_INDEX], 1);

		  for(int i=0;i<PSC_MSG_LEN;i++){
			  msg.Buf[i] =PSC_MESSAGE[i];
		  }
		  msg.Idx= PSC_MSG_LEN;
		  PSC_clearBuffer();
//	  status = osMessageQueueGet(serialMessages_QueueHandle, &msg, NULL, 10U);   // wait for message
//	  if (status == osOK) {
		  if(strncmp(msg.Buf, "brdIdRqst", msg.Idx)==0){
			  char response = '1';
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response), 150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);
		  }
		  else if(strncmp(msg.Buf, "Hello There", msg.Idx)==0){
			  char response[] = "General Kenobi";
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response), 150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);

		  }else if(strncmp(msg.Buf, "Walk", msg.Idx)==0){
			  char response[] = "walking";
        STATE = walk;
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response), 150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);

		  } else if(strncmp(msg.Buf, "imuDataRqst", msg.Idx)==0){
			  char response[100];
			  messageLen = sprintf(response, "<%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,>",
			  					ADI_IMU_burstReadBufScaled[2],
			  					ADI_IMU_burstReadBufScaled[3],
			  					ADI_IMU_burstReadBufScaled[4],
			  					ADI_IMU_burstReadBufScaled[5],
			  					ADI_IMU_burstReadBufScaled[6],
			  					ADI_IMU_burstReadBufScaled[7]);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, messageLen, 1000);

		  }else if(strncmp(msg.Buf, "performTest", msg.Idx)==0){
			  STATE = test1;
			  char response[] = "testing";
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response),150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);
		  }else if(strncmp(msg.Buf, "stand", msg.Idx)==0){
			  STATE = standStill;
			  char response[] = "standing";
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response),150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);
		  }else if(strncmp(msg.Buf, "layDown", msg.Idx)==0){
			  STATE = layDown;
			  char response[] = "laying";
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response),150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);
		  }
		  else{
			  char response[] = "Unrecognized cmd";
			  HAL_UART_Transmit(&huart2, (uint8_t*)&startChar, 1, 1);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&response, sizeof(response),150);
			  HAL_UART_Transmit(&huart2, (uint8_t*)&endChar, 1, 1);
		  }

	  }
//	  osEventFlagsWait(serialDataReady_EventFlagHandle, 0x0001,  osFlagsWaitAny, 0);
//	  if(osMutexAcquire(uartBuffMutexHandle, 0) == osOK){
//
//		  PSC_clearBuffer();
//		  PSC_NEW_DATA_FROM_BOARD = 0;
//		  osMutexRelease(uartBuffMutexHandle);
//
//
//	  }

  }

  /* USER CODE END StartTask08 */
}

/* USER CODE BEGIN Header_StartTask09 */
/**
* @brief Function implementing the checkBuffer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask09 */
void StartTask09(void *argument)
{
  /* USER CODE BEGIN StartTask09 */
  /* Infinite loop */

  /* USER CODE END StartTask09 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim == &htim14){
		osEventFlagsSet(xControllLoop_EventFlagHandle, 0x0001);
		osEventFlagsSet(yControllLoop_EventFlagHandle, 0x0001);
		currentTick = (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq();
		deltaTicks = currentTick - prevTick;
		prevTick = currentTick;

		if(ENABLE_DATA_LOGGING){
			char response[150];
			int messageLen = sprintf(response, "\n%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.3f,%0.2f,%0.2f,%0.2f\n",
											currentTick,
											ADI_IMU_burstReadBufScaled[2],
											ADI_IMU_burstReadBufScaled[3],
											ADI_IMU_burstReadBufScaled[4],
											ADI_IMU_burstReadBufScaled[5],
											ADI_IMU_burstReadBufScaled[6],
											ADI_IMU_burstReadBufScaled[7],
											LEG_CONT_legPositions[0][0],
											LEG_CONT_legPositions[0][1],
											LEG_CONT_legPositions[0][2],
											LEG_CONT_legPositions[1][0],
											LEG_CONT_legPositions[1][1],
											LEG_CONT_legPositions[1][2],
											LEG_CONT_legPositions[2][0],
											LEG_CONT_legPositions[2][1],
											LEG_CONT_legPositions[2][2],
											LEG_CONT_legPositions[3][0],
											LEG_CONT_legPositions[3][1],
											LEG_CONT_legPositions[3][2]
											);

				currentTick = (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq();
				HAL_UART_Transmit(&huart3, (uint8_t*)&response, messageLen, 10);
				deltaTicks = currentTick - (float)osKernelGetSysTimerCount()/ (float)osKernelGetSysTimerFreq();

		}
	}

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
