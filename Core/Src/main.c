/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @author         : Joshua Butler, MD, MHI
  * @date           : 2023-07-02
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * Copyright (c) 2023 Joshua Butler.
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
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "splash.h"
#include "led_strip.h"
#include "ssd1306.h"
#include "ring_buffer.h"
#include "i2c_wrapper.h"
#include "pcf8563.h"
#include "eeprom.h"
#include "ir_codes.h"
#include "led_strip_effect.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ring buffer size
#define CODE_BUFFER_SIZE 8
#define UART_RX_BUFFER_SIZE 256
#define IR_REMOTE_BUFFER_SIZE 16

// EEPROM Defines
#define EEPROM_SETTINGS_PAGE 0x00
#define EEPROM_SETTINGS_OFFSET 0x00

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
    .name = "heartbeatTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for stripShowTask */
osThreadId_t stripShowTaskHandle;
const osThreadAttr_t stripShowTask_attributes = {
    .name = "stripShowTask",
    .stack_size = 384 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for stripShowMutex */
osMutexId_t stripShowMutexHandle;
const osMutexAttr_t stripShowMutex_attributes = {
    .name = "stripShowMutex"
};
/* USER CODE BEGIN PV */

// data buffer

char lcd_buffer[64];

// LED Strip
uint8_t current_num_leds = 32;
led_strip_t led_strip;
led_strip_effect_t led_fx;
uint8_t led_fx_ready = 0;
uint8_t power_state = 1;

// IR Remote

uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
uint32_t code;
uint32_t prevCode;
uint8_t codeReady;
uint32_t codeCmd;
uint8_t startIR;
uint8_t repeatCode;
uint8_t potentialRepeatCode;
uint8_t repeatCodeCount;
RingBuffer codeBuffer;

// UART
RingBuffer uartRxBuffer;
uint8_t uartRxData[2] = { 0 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
void StartDefaultTask(void *argument);
void StartHeartbeatTask(void *argument);
void StartStripShowTask(void *argument);

/* USER CODE BEGIN PFP */

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
    MX_RTC_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM5_Init();
    /* USER CODE BEGIN 2 */

    HAL_RTC_Init(&hrtc);
    __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
    HAL_RTC_WaitForSynchro(&hrtc);
    __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

    if (ring_buffer_init(&uartRxBuffer, UART_RX_BUFFER_SIZE, sizeof(uint8_t)) == RING_BUFFER_MALLOC_FAILED) {
        Error_Handler();
    }

    if (ring_buffer_init(&codeBuffer, IR_REMOTE_BUFFER_SIZE, sizeof(uint32_t)) == RING_BUFFER_MALLOC_FAILED) {
        Error_Handler();
    }

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim5);
    
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);
    /* USER CODE END 2 */

    /* Init scheduler */
    osKernelInitialize();
    /* Create the mutex(es) */
    /* creation of stripShowMutex */
    stripShowMutexHandle = osMutexNew(&stripShowMutex_attributes);

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

    /* creation of heartbeatTask */
    heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);

    /* creation of stripShowTask */
    stripShowTaskHandle = osThreadNew(StartStripShowTask, NULL, &stripShowTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
      /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    /* USER CODE BEGIN RTC_Init 1 */

    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
    */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 84 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */

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

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 84 - 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4294967295;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */

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

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_OC_InitTypeDef sConfigOC = { 0 };

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 105 - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
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
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 8400 - 1;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4294967295;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */

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
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream4_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

      /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin | LED_HB_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, OE_Pin | EEPROM_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : IR_RECV_Pin */
    GPIO_InitStruct.Pin = IR_RECV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IR_RECV_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_STATUS_Pin LED_HB_Pin */
    GPIO_InitStruct.Pin = LED_STATUS_Pin | LED_HB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : OE_Pin EEPROM_Pin */
    GPIO_InitStruct.Pin = OE_Pin | EEPROM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : DIP3_Pin DIP2_Pin DIP1_Pin */
    GPIO_InitStruct.Pin = DIP3_Pin | DIP2_Pin | DIP1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static int counter;
    if (GPIO_Pin == IR_RECV_Pin) {

        counter = (int)__HAL_TIM_GET_COUNTER(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        // HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

        if (counter > 11000 && counter < 100000 && potentialRepeatCode && startIR && prevCode) {
            repeatCodeCount++;

            codeCmd = prevCode;
            // ignore first two repeat codes and send every other repeat code
            if (repeatCodeCount > 2 && repeatCodeCount % 2 == 0) {
                repeatCode = 1;
                ring_buffer_enqueue(&codeBuffer, (void *)&codeCmd);
            }
        }
        else if (counter > 13000) {
            tempCode = 0;
            bitIndex = 0;
            startIR = 1;
            repeatCode = 0;
            potentialRepeatCode = 1;
            repeatCodeCount = 0;
            prevCode = codeCmd;
        }
        else if (counter > 1700) {
            tempCode |= (1UL << (31 - bitIndex)); // write 1
            bitIndex++;
            potentialRepeatCode = 0;
        }
        else if (counter > 1000) {
            tempCode &= ~(1UL << (31 - bitIndex)); // write 0
            bitIndex++;
            potentialRepeatCode = 0;
        }
        if (bitIndex == 32) {
            cmdli = ~tempCode; // Logical inversion last 8 bits
            cmd = tempCode >> 8; // Second last 8 bits
            if (cmdli == cmd) {
                code = tempCode;
                codeCmd = code;
                ring_buffer_enqueue(&codeBuffer, (void *)&codeCmd);
                // HAL_GPIO_WritePin(LED_UI_GPIO_Port, LED_UI_Pin, GPIO_PIN_RESET);
            }
            else {
                codeCmd = 0xFFFFFFFF;
            }
            bitIndex = 0;
            startIR = 0;
        }
        
        // HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    ring_buffer_enqueue(&uartRxBuffer, (void *)uartRxData);
    HAL_UART_Receive_IT(&huart2, uartRxData, 1);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
    led_strip.data_sent_flag = 1;
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
    /* init code for USB_DEVICE */
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 5 */
    
    char fx_name_buffer[25] = "";
    osStatus_t os_status;
    uint8_t current_brightness = 25;
    uint8_t current_fx = 0;
    uint8_t prev_fx = 0;
    uint8_t has_fx_changed = 0;
    uint32_t remote_cmd = 0;
    uint8_t need_update = 0;
    uint8_t power_update = 0;
    struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } current_color;
    
    current_color.red = 128;
    current_color.green = 0;
    current_color.blue = 0;
    
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    
    uint8_t use_external_rtc = HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin) == GPIO_PIN_SET ? 1 : 0;
    
    led_strip_error_t led_strip_error;
    uint8_t sacrifical_led_flag = HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin) == GPIO_PIN_SET ? 1 : 0;
    
    led_strip_error = led_strip_init(&led_strip, &htim3, TIM_CHANNEL_1, htim3.Init.Period, 100, sacrifical_led_flag);
    if (led_strip_error != LED_STRIP_OK) {
        Error_Handler();
    }
    
    led_strip.data_sent_flag = 1;
    led_strip.sacrificial_led_flag = sacrifical_led_flag;
    
    // Splash screen
    splash(&led_strip);
    
    // Initialize LED Effects
    led_strip_effect_error_t led_fx_error = led_strip_effect_init(&led_fx, &led_strip, current_num_leds);
    if (led_fx_error != LED_STRIP_EFFECT_OK) {
        Error_Handler();
    }
    
    for (int i = 0; i < 5; i--)
    {
        os_status = osMutexAcquire(stripShowMutexHandle, 2000);
        if (os_status == osErrorTimeout) {
            Error_Handler();
        } else if (os_status == osOK) {
            led_fx.effect_id = 0;
            led_fx.solid_color = 0x00007F00; // Red
            led_fx.brightness = current_brightness;
            break;
        }
    }
    osMutexRelease(stripShowMutexHandle);
    
    led_fx_ready = 1;
    
    fx_get_name(fx_name_buffer, current_fx, 12);
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteStringCenteredHorizonal(fx_name_buffer, Font_11x18, White);
    ssd1306_UpdateScreen();
    
      /* Infinite loop */
    for (;;)
    {
        if (!is_ring_buffer_empty(&codeBuffer)) {
            ring_buffer_dequeue(&codeBuffer, &remote_cmd);

            switch (remote_cmd)
            {
            case IR_BPlus:
                current_brightness += 1;
                if (current_brightness > 45) current_brightness = 45;
                need_update = 1;
                break;
            case IR_BMinus:
                current_brightness -= 1;
                if (current_brightness > 45) current_brightness = 0;
                need_update = 1;
                break;
            case IR_UPR:
                current_color.red++;
                if (current_color.red > 128) current_color.red = 128;                
                need_update = 1;
                break;
            case IR_UPG:
                current_color.green++;
                if (current_color.green > 128) current_color.green = 128;               
                need_update = 1;
                break;
            case IR_UPB:
                current_color.blue++;
                if (current_color.blue > 128) current_color.blue = 128;
                need_update = 1;
                break;
            case IR_DOWNR:
                current_color.red--;
                if (current_color.red > 128) current_color.red = 128;
                else if (current_color.red < 0) current_color.red = 0;
                need_update = 1;
                break;
            case IR_DOWNG:
                current_color.green--;
                if (current_color.green > 128) current_color.green = 128;
                else if (current_color.green < 0) current_color.green = 0;
                need_update = 1;
                break;
            case IR_DOWNB:
                current_color.blue--;
                if (current_color.blue > 128) current_color.blue = 128;
                else if (current_color.blue < 0) current_color.blue = 0;
                need_update = 1;
                break;
            case IR_PLAY:
                prev_fx = current_fx;
                current_fx++;
                if (current_fx >= NBR_FX) current_fx = 0;
                has_fx_changed = 1;
                break;
            case IR_PWR:
                power_update = 1;
                power_state = !power_state;
                break;
            default:
                sprintf(lcd_buffer, "Code: %0X     ", (unsigned int)remote_cmd);
                ssd1306_SetCursor(5, 20);
                ssd1306_WriteString(lcd_buffer, Font_7x10, White);
                ssd1306_UpdateScreen();
                osDelay(3000);
                break;
            }
        }
        if (!power_state && power_update) {
            power_update = 0;
            HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
            ssd1306_SetDisplayOn(0);
            led_strip_set_brightness(&led_strip, 0);
            led_strip_WS2812_send(&led_strip);
        } else if (power_state && power_update) {
            power_update = 0;
            HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
            ssd1306_SetDisplayOn(1);
            led_strip_set_brightness(&led_strip, current_brightness);
            led_strip_WS2812_send(&led_strip);
            need_update = 1;
        } 
        if (need_update || has_fx_changed) {
            need_update = 0;            
            os_status = osMutexAcquire(stripShowMutexHandle, 2000);
            if (os_status == osErrorTimeout) {
                Error_Handler();
            }
            else if (os_status == osOK) {
                if (has_fx_changed) {
                    has_fx_changed = 0;
                    fx_get_name(fx_name_buffer, current_fx, 12);
                    ssd1306_SetCursor(0, 0);
                    ssd1306_ClearLine(0, Font_11x18);
                    ssd1306_SetCursor(0, 0);
                    ssd1306_WriteStringCenteredHorizonal(fx_name_buffer, Font_11x18, White);
                    ssd1306_UpdateScreen();
                    led_fx.effect_id = current_fx;
                    led_fx.initialized = 0;
                }
                else {
                    led_fx.solid_color = ((uint32_t)current_color.green << 16) | ((uint32_t)current_color.red << 8) | (uint32_t)current_color.blue;
                    led_fx.brightness = current_brightness;
                    led_fx.need_update = 1;
                }
            }
            osMutexRelease(stripShowMutexHandle);
        } else 
        osDelay(50);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
* @brief Function implementing the heartbeatTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
    /* USER CODE BEGIN StartHeartbeatTask */
      /* Infinite loop */
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_HB_GPIO_Port, LED_HB_Pin);
        osDelay(1000);
    }
    /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartStripShowTask */
/**
* @brief Function implementing the stripShowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStripShowTask */
void StartStripShowTask(void *argument)
{
    osStatus_t os_status;
    
    /* USER CODE BEGIN StartStripShowTask */
    while (!led_fx_ready) {
        osDelay(5);
    }
    
    /* Infinite loop */
    for (;;) {
        if (power_state) {
            if (!led_fx.initialized || led_fx.need_update || led_fx.is_loop) {
                os_status = osMutexAcquire(stripShowMutexHandle, 2000);
                fx_update(&led_fx);
                osMutexRelease(stripShowMutexHandle);
            }
            HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
            if (led_fx.is_loop) {
                osDelay(led_fx.delay_time);
            }
            else {
                osDelay(500);
            }
        } else {
            osDelay(500);
        }
    }
    /* USER CODE END StartStripShowTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM4) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

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
    
    ssd1306_Fill(Black);
    ssd1306_SetCursor(3, 7);
    ssd1306_WriteString("FATAL ERROR", Font_11x18, White);
    ssd1306_UpdateScreen();

    while (1)
    {
        int i;
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        for (i = 0; i < 2000000; i++) { __NOP(); }
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        for (i = 0; i < 2000000; i++) { __NOP(); }
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        for (i = 0; i < 2000000; i++) { __NOP(); }
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        for (i = 0; i < 8000000; i++) { __NOP(); }
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
