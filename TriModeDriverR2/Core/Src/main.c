/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
 uint16_t dim_filt;
 uint16_t ntc_filt;
 uint16_t dim_adc;
 uint16_t ntc_adc;
 uint16_t dim_setpoint;
 uint16_t current_index;
 uint16_t duty_cycle;
 uint16_t duty_cycle_setpoint;
 uint16_t over_temp_duty_cycle;
 uint16_t ntc;
 uint16_t MCU_temperature;
 uint16_t temp;
 uint16_t cs0;
 uint16_t cs1;
 uint16_t cs2;
 uint16_t start_derating = OTP_THRESHOLD;
 int16_t filter_out;
 uint16_t counter_filter;
//volatile uint32_t sum;
//volatile uint32_t in;
//volatile uint32_t average;
//volatile uint16_t alpha;
volatile uint32_t dim_circ_buf[N(K)]; // The circular buffer is the filter register.
volatile uint32_t ntc_circ_buf[N(K)]; // The circular buffer is the filter register.
volatile uint32_t otp_circ_buf[N(K)];
volatile uint32_t dim_sum = 0; // Sum of elements in the filter register.
volatile uint32_t ntc_sum = 0; // Sum of elements in the filter register.
volatile uint32_t otp_duty_cycle_sum = 0;
volatile uint16_t circ_buf_ptr = 0; // Index for the circular buffer.
volatile uint16_t otp_circ_buf_ptr = 0; // Index for the circular buffer.
 float ntc_temp;
 float ntc_resistance;
 float ratio;
 float T;
 float otp_duty_cycle;

 uint16_t otp_flag = 1;
 uint16_t softstart = 0;
//float start_dimming = 0.53 ; // in %
//float start_derating = 80; // in °C
//float stop_dimming = 0; // in %
//float stop_derating = 100; // in °C
// y = (start_dimming - stop_dimming) / (start_derating - stop_derating);
// x = - ( (y * start_derating) - (start_dimming) );


float percent_derating;

State_e state;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint16_t 	adcResultsDMA[ADC_BUF_LEN];

//const int 			adcChannelCount = sizeof (adcResultsDMA)/sizeof (adcResultsDMA[0]);
volatile int		adcConversionComplete = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t convCompleted = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convCompleted = 1;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	HAL_ADC_Stop_DMA(&hadc1);

//	 dim_filt = Averagingfilter (dim_adc);
//	 uint16_t Averagingfilter (uint16_t input) {
//	 dim_sum -= dim_circ_buf[circ_buf_ptr]; // Subtract the oldest value from the sum.
//	 ntc_sum -= ntc_circ_buf[circ_buf_ptr]; // Subtract the oldest value from the sum.
//	 dim_circ_buf[circ_buf_ptr] = dim_adc; // Place the input in the filter register.
//	 ntc_circ_buf[circ_buf_ptr] = ntc_adc; // Place the input in the filter register.
//	 dim_sum += dim_circ_buf[circ_buf_ptr++]; // Ad the newest value to the sum.
//	 ntc_sum += ntc_circ_buf[circ_buf_ptr++]; // Ad the newest value to the sum.
//	 circ_buf_ptr %= N(K); // Increment the buffer keeping it in the range 0 to N-1;
//	 dim_filt = dim_sum >> K;
//	 ntc_filt = ntc_sum >> K;
//	 temp = dim_filt;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

	current_index = 0.0f;
	ntc = 0;
	duty_cycle_setpoint = 0;
	duty_cycle = 1;
	MCU_temperature = 0;
	temp = 0;
	cs0 = 0;
	cs1 = 0;
	cs2 = 0;
	state = STANDBY;
	uint16_t max_current[8] = {
			SCALED_CURRENT_LEVEL_1,
			SCALED_CURRENT_LEVEL_2,
			SCALED_CURRENT_LEVEL_3,
			SCALED_CURRENT_LEVEL_4,
			SCALED_CURRENT_LEVEL_5,
			SCALED_CURRENT_LEVEL_6,
			SCALED_CURRENT_LEVEL_7,
			SCALED_CURRENT_LEVEL_8
			};
	duty_cycle = 0;

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_TIM_OC_Start(&htim14, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, ADC_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  	  HAL_ADC_Start(&hadc1);
//	  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//  	  	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, ADC_BUF_LEN);

// Voltage divider is 120k and 61.9k. Gain is 0.340, adc count at 9.7V = 4095. adc count at 1.0V is 395
	  	  dim_adc = adcResultsDMA[0];
	  	  ntc_adc = adcResultsDMA[1];
	  	  dim_sum -= dim_circ_buf[circ_buf_ptr]; // Subtract the oldest value from the sum.
	  	  ntc_sum -= ntc_circ_buf[circ_buf_ptr]; // Subtract the oldest value from the sum.
	  	  dim_circ_buf[circ_buf_ptr] = dim_adc; // Place the input in the filter register.
	  	  ntc_circ_buf[circ_buf_ptr] = ntc_adc; // Place the input in the filter register.
	  	  dim_sum += dim_circ_buf[circ_buf_ptr]; // Ad the newest value to the sum.
	  	  ntc_sum += ntc_circ_buf[circ_buf_ptr++]; // Ad the newest value to the sum.
	  	  circ_buf_ptr %= N(K); // Increment the buffer keeping it in the range 0 to N-1;
	  	  dim_filt = (uint16_t)(dim_sum >> K);
	  	  ntc_filt = (uint16_t)(ntc_sum >> K);
	  	  temp = dim_filt;

		  if (dim_filt < 870){
			  dim_filt = 0;
		  }
		  if (dim_filt != 0){
		  dim_filt = dim_filt - 870; //Offset due to 1k resistor and 0.5mA current source
		  dim_filt = dim_filt * 1.27;// set slope
		  }
		  if (dim_filt > 4095){
			  dim_filt = 4095;
		  }
		  dim_setpoint = dim_filt;

		  MCU_temperature = adcResultsDMA[2];

		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA, ADC_BUF_LEN);
//	  	  HAL_Delay(50);

		  cs0 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		  cs1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		  cs2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		  current_index = (cs2<<2) + (cs1<<1) + cs0;

//	  	  temp = max_current[current_index];

	  	  duty_cycle = max_current[current_index] * dim_setpoint; //Timer1 period is 8192

			 if ((duty_cycle_setpoint <= duty_cycle) && (ntc_temp < START_DERATING)){
				 duty_cycle_setpoint++;
				 percent_derating = 1;
				 otp_flag = 1;
			 }
			 if ((duty_cycle_setpoint > duty_cycle) && (ntc_temp < START_DERATING)){
			 	 duty_cycle_setpoint--;
			 	percent_derating = 1;
			 	otp_flag = 1;
			 }
//			 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
			 TIM1->CCR1 = duty_cycle_setpoint;   //Timer1 period is 8192

	  	  switch (state) {
	  	           case STANDBY:
	  	           if (softstart < 9){
	  	            duty_cycle_setpoint = duty_cycle;
	  	           }
	  	           if (softstart == 9){
	  	           	state = RUNNING;
	  	           }
	  	           ++softstart;

	  	            break;
	  	            case SOFTSTART:
	  	            break;
	  	            case FAULT:
	  	          	break;
	  	            case RUNNING:
	  	            	softstart = 0;
	  	}
  }

}

	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
	  /* Prevent unused argument(s) compilation warning */
		 if(htim->Instance==TIM14)
		     {
//			 if ((duty_cycle_setpoint <= duty_cycle) && (ntc_temp < START_DERATING)){
//			 				 duty_cycle_setpoint++;
//			 				 percent_derating = 1;
//			 				 otp_flag = 1;
//			 			 }
//			 			 if ((duty_cycle_setpoint > duty_cycle) && (ntc_temp < START_DERATING)){
//			 			 	 duty_cycle_setpoint--;
//			 			 	percent_derating = 1;
//			 			 	otp_flag = 1;
//			 			 }

			 //	  	  ntc_resistance = thermistorResistance(ntc_filt);
			 	  	  ratio = (float)ntc_filt/4096.0;
			 	  	  ntc_resistance = 10000 * (1-ratio)/ ratio;
			 //	  	  ntc_temp = thermistorTemperature(ntc_resistance);
			 	  	  T =  1 / ((1 / T25) + ((log(ntc_resistance / R25)) / BETA));
			 	  	  ntc_temp = T - 273.15; // Converting kelvin to celsius
			 	  	  if (ntc_temp > START_DERATING && ntc_temp < STOP_DERATING){
			 	  		  // derating slope calculation
			 	  		  if (otp_flag == 1){
			 				 over_temp_duty_cycle = duty_cycle_setpoint;
			 				 otp_flag = 0;
			 				 }
			 				 percent_derating = ntc_temp * ORDINATE + ABSCISSA;
			 				 if(percent_derating > 1)percent_derating = 1;
			 				 otp_duty_cycle = (over_temp_duty_cycle * percent_derating);

			 				otp_duty_cycle_sum -= otp_circ_buf[otp_circ_buf_ptr]; // Subtract the oldest value from the sum.

			 				otp_circ_buf[otp_circ_buf_ptr] = otp_duty_cycle; // Place the input in the filter register.

			 				otp_duty_cycle_sum += otp_circ_buf[otp_circ_buf_ptr++]; // Ad the newest value to the sum.

			 				otp_circ_buf_ptr %= N(K); // Increment the buffer keeping it in the range 0 to N-1;

			 				duty_cycle_setpoint = (uint16_t)(otp_duty_cycle_sum>>K);
			 			 }
			 			 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
//			 			 TIM1->CCR1 = duty_cycle_setpoint;   //Timer1 period is 8192
			 }

	}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_19CYCLES_5;
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

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8192;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 1024;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim14.Init.Prescaler = 2999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 999;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : cs2_Pin cs1_Pin cs0_Pin */
  GPIO_InitStruct.Pin = cs2_Pin|cs1_Pin|cs0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
