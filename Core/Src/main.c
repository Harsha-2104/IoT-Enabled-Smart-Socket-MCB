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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// --- Safety Thresholds ---
#define MAX_SAFE_TEMP_C 80.0
#define OVERCURRENT_PERIOD_US 2000 // The formula f_CF1 = (48 * V1 * F_OSC) / (V_REF^2 * 512) varies by component tolerance.
// Measure the actual CF1 period at 16A on your specific PCB before deploying.

// --- Constants ---
#define NTC_SERIES_RESISTOR 10000.0
#define NTC_NOMINAL_RESISTANCE 10000.0
#define NTC_BETA_VALUE 3950.0
#define NOMINAL_TEMP_K 298.15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

volatile uint8_t flag_measure_temp = 0;
volatile uint32_t last_systick = 0;

volatile uint16_t last_time = 0;
volatile uint16_t pulse_period = 0;// NOTE: pulse_period is 16-bit. On Cortex-M0+, 16-bit reads are atomic.
// Safe to read in main loop without __disable_irq(), but do not change to 32-bit without adding critical sections.
volatile uint32_t last_pulse_tick = 0;
volatile uint8_t sensor_fault_flagged = 0; // State machine flag for zero-load
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Process_Temperature(void);
void Emergency_Cutoff(void);
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
  // Demote SysTick priority so EXTI0_1_IRQn (Priority 0) can preempt it safely.
  // This prevents jitter in the hard-real-time overcurrent detection.
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Start the background stopwatch (TIM3)
  HAL_TIM_Base_Start(&htim3);

  // Wait 500ms for power supplies and the HLW8012 oscillator to fully stabilize.
  HAL_Delay(500);

  // Calibrate ADC before use for accurate temperature readings
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Turn ON Relay initially (PA5)
  HAL_GPIO_WritePin(GPIOA, RELAY_PIN_Pin, GPIO_PIN_SET);

  // Set HLW8012 SEL Pin HIGH to monitor CURRENT on CF1 (PA2)
  HAL_GPIO_WritePin(GPIOA, HLW_SEL_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // --- EVENT SCHEDULER ---
	        uint32_t now = HAL_GetTick(); // Single snapshot for the entire loop iteration

	        if ((now - last_systick) >= 100) {
	            last_systick = now;
	            flag_measure_temp = 1;
	        }

	        // --- BACKGROUND TASKS ---
	        if (flag_measure_temp) {
	            Process_Temperature();
	            flag_measure_temp = 0;
	        }

	        // --- SENSOR WATCHDOG ---
	        // If running for >500ms, no pulse in 5 seconds, and we haven't already flagged it...
	        if ((now > 500) && ((now - last_pulse_tick) > 5000) && !sensor_fault_flagged) {
	            sensor_fault_flagged = 1; // Latch the flag so this only fires once

	            // Zero pulses could mean 0A (no load), OR a dead HLW8012 IC.
	            // Toggle a warning LED or send a UART alert. Do NOT cut off power.
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, HLW_SEL_Pin|RELAY_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : HLW_SEL_Pin RELAY_PIN_Pin */
  GPIO_InitStruct.Pin = HLW_SEL_Pin|RELAY_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// --- The Critical Path: Input Capture Interrupt ---
// This triggers the nanosecond a pulse edge arrives from the HLW8012

// --- NTC Temperature Process (PB0) ---
void Process_Temperature(void) {
    // Polling ADC here is fine because it's a fast peripheral and
    // we only call this every 100ms from the event scheduler.
    // (For pure non-blocking, you could use DMA, but it's overkill for one NTC)
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

        if (adc_value > 0 && adc_value < 4095) {
        	float r_ntc = NTC_SERIES_RESISTOR * ((float)adc_value / (4095.0 - (float)adc_value));
            float temp_k = 1.0 / ((1.0 / NOMINAL_TEMP_K) + (1.0 / NTC_BETA_VALUE) * log(r_ntc / NTC_NOMINAL_RESISTANCE));
            float temp_c = temp_k - 273.15;

            // Check for overheating
            if (temp_c > MAX_SAFE_TEMP_C) {
                Emergency_Cutoff();
            }
        }
    }
    HAL_ADC_Stop(&hadc1);
}

// --- Emergency Cutoff ---
void Emergency_Cutoff(void) {
    // Break the circuit instantly
	HAL_GPIO_WritePin(GPIOA, RELAY_PIN_Pin, GPIO_PIN_RESET);

    // Disable interrupts to freeze the system state
    __disable_irq();

    // Latch off until manual reset
    while(1) {
    	// Non-interrupt blocking delay for a fault LED (assuming PA4 is an LED)
		// HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		// for(volatile uint32_t i = 0; i < 1000000; i++) {}
    }
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) { // PA1 (CF1 Pulse)
    	uint16_t current_time = __HAL_TIM_GET_COUNTER(&htim3);

		// A pulse arrived! Clear the zero-load watchdog flag
		sensor_fault_flagged = 0;

		// Time-based lockout. Ignore all pulses during the 500ms startup settling phase.
		if (HAL_GetTick() < 500) {
			last_time = current_time;
			last_pulse_tick = HAL_GetTick();
			return;
		}

		if (current_time >= last_time) {
			pulse_period = current_time - last_time;
		} else {
			pulse_period = (0xFFFF - last_time) + current_time + 1;
		}

		last_time = current_time;
		last_pulse_tick = HAL_GetTick();

		if (pulse_period < OVERCURRENT_PERIOD_US) {
			Emergency_Cutoff();
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
