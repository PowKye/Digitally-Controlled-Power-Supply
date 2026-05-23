/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t raw_reading;
  uint16_t corrected_value;
} lut_point_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STABILIZER_KP_DIVISOR 16
#define STABILIZER_MAX_STEP 10
#define STABILIZER_DEADBAND 5
#define UART_RX_BUFFER_SIZE 32
#define VREF_MILV 3325
#define MAX_MILV 9000
#define ADC_DIVISOR 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint8_t flag_update_oled = 0;
volatile uint8_t flag_log_adc = 0;

volatile uint32_t adc_accumulator = 0;
volatile uint8_t adc_sample_count = 0;
volatile uint32_t adc_avg = 0;
volatile uint32_t target_voltage_mV = 0;
volatile uint16_t adc_target = 0;
uint32_t adc_corrected = 0;

volatile uint8_t dac_output = 0;
volatile int16_t previous_encoder_count = 0;
uint8_t state = 0;

// ADC Non Linear Characteristic Compensation LUT
const lut_point_t adc_correction_lut[] = {
    {0, 0},
    {6, 66},
    {72, 130},
    {160, 226},
    {244, 306},
    {357, 424},
    {440, 496},
    {564, 635},
    {644, 716},
    {770, 833},
    {930, 999},
    {1013, 1081},
    {1140, 1211},
    {1223, 1292},
    {1307, 1373},
    {1380, 1449},
    {1554, 1624},
    {1675, 1740},
    {1883, 1938},
    {1951, 2035},
    {2293, 2344},
    {2584, 2631},
    {2664, 2709},
    {2754, 2804},
    {2830, 2878},
    {2904, 2952},
    {3019, 3067},
    {3116, 3161},
    {3313, 3362},
    {3391, 3436},
    {3527, 3572},
    {3663, 3707}};
const int adc_lut_size = sizeof(adc_correction_lut) / sizeof(adc_correction_lut[0]);

// UART command processing
volatile uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
volatile uint8_t uart_rx_index = 0;
volatile uint8_t flag_uart_cmd_ready = 0;
uint8_t uart_rx_char[1]; // Buffer for HAL_UART_Receive_IT

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void LogGPIOState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void CycleRGBLED(int numCycles, int delayMs);
void LogStartupMessage(void);
void WritePortByte(GPIO_TypeDef *GPIOx, uint8_t isHighByte, uint8_t value);
uint8_t App_KillSwitch_Check(void);
void ISR_ReadADC(void);
void App_LogData(void);
void App_DigitalStabilizer(void);
uint32_t ApplyADCCorrection(uint32_t raw_value);
void App_ProcessUartCommand(void);
void App_HandleEncoderSwitch(void);
void App_HandleEncoderRotation(void);
void App_UpdateOLED(void);

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // Start 10ms time base
  HAL_TIM_Base_Start_IT(&htim2);

  // Start encoder hardware reading
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // Initializr encoder counter
  previous_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

  // Log startup message in the serial terminal
  LogStartupMessage();

  // Start listening for UART commands
  HAL_UART_Receive_IT(&huart1, uart_rx_char, 1);

  // Cycle through RGB colors at startup in a blocking manner
  CycleRGBLED(1, 300);

  // Initialize the OLED display
  ssd1306_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    App_KillSwitch_Check();
    App_ProcessUartCommand();

    App_HandleEncoderSwitch();
    App_HandleEncoderRotation();

    if (state == 1)
    {
      App_LogData();
      App_DigitalStabilizer();
    }

    if (flag_update_oled)
    {
      App_UpdateOLED();
      flag_update_oled = 0;
    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
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
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, R_LED_Pin | G_LED_Pin | Vb2_Pin | Vb3_Pin | Vb4_Pin | Vb5_Pin | Vb6_Pin | Vb7_Pin | Vb0_Pin | Vb1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(B_LED_GPIO_Port, B_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_SW_Pin */
  GPIO_InitStruct.Pin = ENC_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R_LED_Pin G_LED_Pin */
  GPIO_InitStruct.Pin = R_LED_Pin | G_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Vb2_Pin Vb3_Pin Vb4_Pin Vb5_Pin
                           Vb6_Pin Vb7_Pin Vb0_Pin Vb1_Pin */
  GPIO_InitStruct.Pin = Vb2_Pin | Vb3_Pin | Vb4_Pin | Vb5_Pin | Vb6_Pin | Vb7_Pin | Vb0_Pin | Vb1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : B_LED_Pin */
  GPIO_InitStruct.Pin = B_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(B_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/// @brief Loggs State of GPIOx_Pin through UART1
/// @param GPIOx
/// @param GPIO_Pin
void LogGPIOState(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_PinState state = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
  char *port_name;
  if (GPIOx == GPIOA)
    port_name = "GPIOA";
  else if (GPIOx == GPIOB)
    port_name = "GPIOB";
  else if (GPIOx == GPIOC)
    port_name = "GPIOC";
  else
    port_name = "UNKNOWN";
  int pin_num = 0;
  uint16_t temp = GPIO_Pin;
  while (temp > 1)
  {
    temp >>= 1;
    pin_num++;
  }
  char msg[50];
  if (state == GPIO_PIN_SET)
  {
    sprintf(msg, "%s Pin %d is ACTIVE (High)\r\n", port_name, pin_num);
  }
  else
  {
    sprintf(msg, "%s Pin %d is INACTIVE (Low)\r\n", port_name, pin_num);
  }
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/// @brief  Cycles through a blocking R-G-B sequence on the built-in RGB LED.
/// @param numCycles The number of times to cycle through Red, Green, and Blue.
/// @param delayMs The time in milliseconds to keep each color on.
void CycleRGBLED(int numCycles, int delayMs)
{
  for (int i = 0; i < numCycles; i++)
  {
    // --- RED ---
    HAL_GPIO_WritePin(GPIOB, G_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, B_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, R_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(delayMs);

    // --- GREEN ---
    HAL_GPIO_WritePin(GPIOB, R_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, G_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(delayMs);

    // --- BLUE ---
    HAL_GPIO_WritePin(GPIOB, G_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, B_LED_Pin, GPIO_PIN_SET);
    HAL_Delay(delayMs);
  }

  // Turn all LEDs off at the end of the sequence.
  HAL_GPIO_WritePin(GPIOB, R_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, G_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, B_LED_Pin, GPIO_PIN_RESET);
}

/// @brief Loggs startup message through UART1
/// @param
void LogStartupMessage(void)
{
  char *msg = "P2 v.1 running\r\n";

  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/// @brief Atomically writes an 8-bit value to either the high or low byte of a given GPIO port
/// @param GPIOx Port to write to (e.g., GPIOB)
/// @param isHighByte 1 to target pins 8-15, 0 to target pins 0-7
/// @param value 8-bit value to write
void WritePortByte(GPIO_TypeDef *GPIOx, uint8_t isHighByte, uint8_t value)
{
  if (isHighByte)
  {
    // Target pins 8-15: Set mask shifted by 8, Reset mask shifted by 24
    GPIOx->BSRR = ((uint32_t)value << 8) | ((uint32_t)(~value & 0xFF) << 24);
  }
  else
  {
    // Target pins 0-7: Set mask shifted by 0, Reset mask shifted by 16
    GPIOx->BSRR = (uint32_t)value | ((uint32_t)(~value & 0xFF) << 16);
  }
}

/// @brief Checks the state of the global kill switch (PC13).
///        If active, it sets outputs to a safe state, signals with LEDs,
///        waits for the switch to be released, and then enters a permanent stop state.
///@retval 1 if the kill switch was active and handled, 0 otherwise.
uint8_t App_KillSwitch_Check(void)
{
  // Global "Kill Switch" check
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
  {
    HAL_Delay(10); // Debounce to filter noise
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
    {
      // Set outputs to a safe state
      WritePortByte(GPIOB, 1, 0); // Set DAC output to 0
      dac_output = 0;

      ssd1306_Fill(Black);
      ssd1306_UpdateScreen();

      // Indicate stop state (red LED on)
      HAL_GPIO_WritePin(GPIOB, R_LED_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, G_LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, B_LED_Pin, GPIO_PIN_RESET);

      // Wait until the button is released
      while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
      {
      }

      // Button released - enter permanent stop state
      while (1)
      {
        ssd1306_SetCursor(27, 20);
        ssd1306_WriteString("H MODE", Font_11x18, White);
        ssd1306_UpdateScreen();
        HAL_GPIO_WritePin(GPIOB, R_LED_Pin, GPIO_PIN_SET);

        HAL_Delay(500);

        ssd1306_Fill(Black);
        ssd1306_UpdateScreen();
        HAL_GPIO_WritePin(GPIOB, R_LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(500);
      }
    }
  }
  return 0; // Kill switch not active, continue normal operation
}

/// @brief Reads ADC value and accumulates it for averaging.
/// @param
void ISR_ReadADC(void)
{
  HAL_ADC_Start(&hadc1);

  // Poll inside ISR for EOC flag
  uint32_t timeout = 1000;
  while (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) == RESET && timeout > 0)
  {
    timeout--;
  }

  if (timeout > 0) // Conversion finished successfully
  {
    uint32_t adc_raw_value = HAL_ADC_GetValue(&hadc1);

    // Accumulate data for the digital stabilizer
    adc_accumulator += adc_raw_value;
    adc_sample_count++;
  }
}

/// @brief
/// @param
void App_LogData(void)
{
  if (flag_log_adc)
  {
    char msg[90];
    uint32_t voltage_mV = (adc_corrected * VREF_MILV * ADC_DIVISOR + 2047) / 4095;
    uint32_t v_int = voltage_mV / 1000;
    uint32_t v_frac = (voltage_mV % 1000) / 10;
    int len = sprintf(msg, "ADC_Avg: %lu | ADC_Corr: %lu | V_Out: %lu.%02luV | DAC: %u | ADC_TARGET: %u\r\n", adc_avg, adc_corrected, v_int, v_frac, dac_output, adc_target);

    HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
    flag_log_adc = 0;
  }
}

/// @brief Counts adc_sample_counts and performs an average.
///        Then computes the error and the dynamic step necesarry to correct it
/// @param
void App_DigitalStabilizer(void)
{
  // Process only if 10 readings have been accumulated (100ms at 10ms rate)
  if (adc_sample_count >= 10)
  {
    // Copy and reset data atomically to prevent race conditions with the interrupt
    __disable_irq();
    uint32_t acc_copy = adc_accumulator;
    uint8_t count_copy = adc_sample_count;
    adc_accumulator = 0;
    adc_sample_count = 0;
    __enable_irq();

    // Compute the average of the accumulated ADC samples
    adc_avg = acc_copy / count_copy;

    // Apply LUT-based correction to the averaged value
    adc_corrected = ApplyADCCorrection(adc_avg);

    // Slow comparator
    int32_t error = adc_target - adc_corrected;
    int32_t step = 0;

    // Apply deadband to prevent constant small adjustments
    if (abs(error) > STABILIZER_DEADBAND)
    {
      // Calculate proportional step
      step = error / STABILIZER_KP_DIVISOR;

      // Limit the step size to prevent aggressive changes
      if (step > STABILIZER_MAX_STEP)
      {
        step = STABILIZER_MAX_STEP;
      }
      else if (step < -STABILIZER_MAX_STEP)
      {
        step = -STABILIZER_MAX_STEP;
      }
    }

    // Adjust dac_output
    int32_t new_dac_output = (int32_t)dac_output + step;

    // Clamp dac_output to valid range (0-255)
    if (new_dac_output > 255)
    {
      dac_output = 255;
    }
    else if (new_dac_output < 0)
    {
      dac_output = 0;
    }
    else
    {
      dac_output = (uint8_t)new_dac_output;
    }

    WritePortByte(GPIOB, 1, dac_output);
  }
}

/// @brief Applies correction to a raw ADC value using an interpolated Look-Up Table.
/// @param raw_value The raw ADC value to be corrected.
/// @return The corrected ADC value.
uint32_t ApplyADCCorrection(uint32_t raw_value)
{
  // If the LUT is empty or has only one point, no interpolation is possible. Return the raw value.
  if (adc_lut_size < 2)
  {
    return raw_value;
  }

  // Handle boundary conditions: if the value is outside the table's range,
  // clamp it to the nearest boundary's corrected value.
  if (raw_value <= adc_correction_lut[0].raw_reading)
  {
    return adc_correction_lut[0].corrected_value;
  }
  if (raw_value >= adc_correction_lut[adc_lut_size - 1].raw_reading)
  {
    return adc_correction_lut[adc_lut_size - 1].corrected_value;
  }

  // Find the two points in the LUT that bracket the raw_value.
  // The table is assumed to be sorted by raw_reading.
  for (int i = 0; i < adc_lut_size - 1; i++)
  {
    if (raw_value >= adc_correction_lut[i].raw_reading && raw_value <= adc_correction_lut[i + 1].raw_reading)
    {
      // Found the segment. Perform linear interpolation.
      uint32_t x0 = adc_correction_lut[i].raw_reading;
      uint32_t y0 = adc_correction_lut[i].corrected_value;
      uint32_t x1 = adc_correction_lut[i + 1].raw_reading;
      uint32_t y1 = adc_correction_lut[i + 1].corrected_value;

      // Use 64-bit integers for intermediate calculations to prevent overflow.
      int64_t numerator = (int64_t)(raw_value - x0) * (int64_t)(y1 - y0);
      int64_t denominator = (int64_t)(x1 - x0);

      // Return the interpolated value. Avoid division by zero.
      return (denominator != 0) ? (y0 + (uint32_t)(numerator / denominator)) : y0;
    }
  }

  // Should not be reached if the value is within the table range, but as a fallback:
  return raw_value;
}

/// @brief Processes commands received over UART.
///        Parses commands to update the ADC target value.
///        Supported commands: +, -
void App_ProcessUartCommand(void)
{
  // Check if a command is ready to be processed.
  if (flag_uart_cmd_ready)
  {
    // Use a local buffer to safely process the command without race conditions.
    char cmd_buffer[UART_RX_BUFFER_SIZE];

    // Create a critical section to atomically copy the command and clear the flag.
    __disable_irq();
    strcpy(cmd_buffer, (const char *)uart_rx_buffer);
    flag_uart_cmd_ready = 0; // Clear the flag immediately so the ISR can receive the next command.
    __enable_irq();

    // Now, parse the command from the safe local buffer.
    uint8_t flag_target_updated_serial = 0;

    // Check for incremental commands
    if (strcmp(cmd_buffer, "+") == 0)
    {
      target_voltage_mV += 100;
      flag_target_updated_serial = 1;
    }
    else if (strcmp(cmd_buffer, "-") == 0)
    {
      if (target_voltage_mV >= 100)
        target_voltage_mV -= 100;
      else
        target_voltage_mV = 0;
      flag_target_updated_serial = 1;
    }
    else
    {
      // Command not recognized
      char msg[40];
      sprintf(msg, "ERR: Unknown cmd. Use: + or -\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
    }

    if (flag_target_updated_serial)
    {
      // Clamp the target voltage to a valid range
      const uint32_t max_voltage_mV = MAX_MILV;
      if (target_voltage_mV > max_voltage_mV)
        target_voltage_mV = max_voltage_mV;

      // Update the integer adc_target from the float voltage
      __disable_irq();
      adc_target = (uint16_t)((target_voltage_mV * 4095) / (VREF_MILV * ADC_DIVISOR));
      __enable_irq();

      // Log confirmation message
      char msg[60];
      uint32_t v_int = target_voltage_mV / 1000;
      uint32_t v_frac = (target_voltage_mV % 1000) / 10;
      int len = sprintf(msg, "OK: Target set to %lu.%02luV (ADC: %u)\r\n", v_int, v_frac, adc_target);
      HAL_UART_Transmit(&huart1, (uint8_t *)msg, len, 100);
    }
  }
}

/// @brief Handles rotary encoder switch press
///        Cycles through states 0 - 1
void App_HandleEncoderSwitch(void)
{
  if (HAL_GPIO_ReadPin(GPIOA, ENC_SW_Pin) == GPIO_PIN_RESET)
  {
    HAL_Delay(50); // Debounce the switch contact
    if (HAL_GPIO_ReadPin(GPIOA, ENC_SW_Pin) == GPIO_PIN_RESET)
    {
      HAL_GPIO_WritePin(GPIOA, B_LED_Pin, GPIO_PIN_SET);

      // Wait for release
      while (HAL_GPIO_ReadPin(GPIOA, ENC_SW_Pin) == GPIO_PIN_RESET)
      {
      }

      HAL_Delay(50); // Debounce release
      HAL_GPIO_WritePin(GPIOA, B_LED_Pin, GPIO_PIN_RESET);

      state = (state == 0) ? 1 : 0;
    }
  }
}

/// @brief Adjusts the ADC target voltage based on encoder rotation.
///        This function reads the encoder, calculates the change, and updates
///        u_target_voltage and adc_target accordingly.
void App_HandleEncoderRotation(void)
{
  int16_t current_encoder_count = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t encoder_diff = current_encoder_count - previous_encoder_count;

  // Handle potential wrap-around for a 16-bit counter (0 to 65535).
  // Assuming the encoder won't be rotated more than half its range in one loop iteration.
  // The period is (htim3.Init.Period + 1) because it counts from 0 to Period.
  if (encoder_diff > (htim3.Init.Period / 2))
  {
    encoder_diff -= (htim3.Init.Period + 1);
  }
  else if (encoder_diff < -(htim3.Init.Period / 2))
  {
    encoder_diff += (htim3.Init.Period + 1);
  }

  // Divide by 2 because the timer registers 2 counts per physical encoder click
  int16_t clicks = encoder_diff / 2;

  if (clicks != 0)
  {
    // Scaling factor: 0.1V per physical click.
    int32_t new_target_mV = (int32_t)target_voltage_mV + (clicks * 100);

    // Clamp the target voltage to a valid range
    const int32_t max_voltage_mV = MAX_MILV;
    if (new_target_mV < 0)
      target_voltage_mV = 0;
    else if (new_target_mV > max_voltage_mV)
      target_voltage_mV = max_voltage_mV;
    else
      target_voltage_mV = (uint32_t)new_target_mV;

    // Update the integer adc_target from the float voltage
    __disable_irq();
    adc_target = (uint16_t)((target_voltage_mV * 4095) / (VREF_MILV * ADC_DIVISOR));
    __enable_irq();

    // Update previous encoder count, preserving any remainder (half-steps) for the next read
    previous_encoder_count += clicks * 2;
  }
}

/// @brief Updates the OLED display with target or real-time voltages
void App_UpdateOLED(void)
{
  // Cache the previous state and values to prevent I2C bus spam and interrupt lockups
  static uint8_t last_state = 255;
  static uint32_t last_v_int = 255;
  static uint32_t last_v_frac = 255;

  uint32_t v_int = 0;
  uint32_t v_frac = 0;
  char display_str[32];

  if (state == 0)
  {
    v_int = target_voltage_mV / 1000;
    v_frac = (target_voltage_mV % 1000) / 10;
  }
  else if (state == 1)
  {
    uint32_t voltage_mV = (adc_corrected * VREF_MILV * ADC_DIVISOR + 2047) / 4095;
    v_int = voltage_mV / 1000;
    v_frac = (voltage_mV % 1000) / 10;
  }

  // ONLY send I2C commands if the displayed text actually changes
  if (state == last_state && v_int == last_v_int && v_frac == last_v_frac)
  {
    return;
  }

  last_state = state;
  last_v_int = v_int;
  last_v_frac = v_frac;

  ssd1306_Fill(Black);

  ssd1306_SetCursor(5, 3);
  if (state == 0)
    ssd1306_WriteString("TARGET VOLTAGE:", Font_7x10, White);
  else
    ssd1306_WriteString("OUTPUT VOLTAGE:", Font_7x10, White);

  sprintf(display_str, "%lu.%02lu V", v_int, v_frac);
  ssd1306_SetCursor(30, 30);
  ssd1306_WriteString(display_str, Font_11x18, White);
  ssd1306_UpdateScreen();
}

/// @brief  Rx Transfer completed callback.
/// @param  huart: UART handle
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    uint8_t received_char = uart_rx_char[0];

    // Handle single-character commands immediately, otherwise buffer for multi-character commands.
    if (received_char == '+' || received_char == '-')
    {
      // Treat '+' and '-' as complete commands
      uart_rx_buffer[0] = received_char;
      uart_rx_buffer[1] = '\0';
      flag_uart_cmd_ready = 1;
      uart_rx_index = 0; // Reset for next command
    }
    else if (received_char == '\r' || received_char == '\n')
    {
      if (uart_rx_index > 0) // Command received
      {
        uart_rx_buffer[uart_rx_index] = '\0'; // Null-terminate the string
        flag_uart_cmd_ready = 1;              // Set flag for main loop to process
        uart_rx_index = 0;                    // Reset for next command
      }
    }
    else
    {
      // Buffer character for multi-character commands (e.g., "T=2048")
      if (uart_rx_index < (UART_RX_BUFFER_SIZE - 1))
      {
        uart_rx_buffer[uart_rx_index++] = received_char;
      }
    }

    // Re-arm the UART receive interrupt
    HAL_UART_Receive_IT(&huart1, uart_rx_char, 1);
  }
}

/// @brief
/// @param htim
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    ISR_ReadADC();

    // Tick divider from 10ms to 1000ms
    static uint8_t ticks = 0;
    ticks++;
    if (ticks >= 100) // 100 ticks * 10 ms = 1 second
    {
      flag_log_adc = 1; // Send the flag to the main function
      ticks = 0;
    }

    // Tick divider from 10ms to 100ms for OLED updates
    static uint8_t oled_ticks = 0;
    oled_ticks++;
    if (oled_ticks >= 10) // 10 ticks * 10 ms = 100 ms
    {
      flag_update_oled = 1; // Trigger OLED update
      oled_ticks = 0;
    }
  }
}

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
