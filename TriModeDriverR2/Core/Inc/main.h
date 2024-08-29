/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define cs2_Pin GPIO_PIN_1
#define cs2_GPIO_Port GPIOA
#define cs1_Pin GPIO_PIN_2
#define cs1_GPIO_Port GPIOA
#define cs0_Pin GPIO_PIN_3
#define cs0_GPIO_Port GPIOA
#define NTC_Pin GPIO_PIN_7
#define NTC_GPIO_Port GPIOA
#define DIM_Pin GPIO_PIN_8
#define DIM_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define CURRENT_LEVEL_1  150
#define CURRENT_LEVEL_2  200
#define CURRENT_LEVEL_3  250
#define CURRENT_LEVEL_4  300
#define CURRENT_LEVEL_5  350
#define CURRENT_LEVEL_6  400
#define CURRENT_LEVEL_7  450
#define CURRENT_LEVEL_8  500

#define CURRENT_SCALE_FACTOR  0.004

#define SCALED_CURRENT_LEVEL_1  (CURRENT_LEVEL_1 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_2  (CURRENT_LEVEL_2 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_3  (CURRENT_LEVEL_3 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_4  (CURRENT_LEVEL_4 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_5  (CURRENT_LEVEL_5 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_6  (CURRENT_LEVEL_6 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_7  (CURRENT_LEVEL_7 * CURRENT_SCALE_FACTOR)
#define SCALED_CURRENT_LEVEL_8  (CURRENT_LEVEL_8 * CURRENT_SCALE_FACTOR)

#define OTP_THRESHOLD  3000 //see spreadsheet
//#define NTC_RESISTANCE(x)  (float)(10000*x/(4096-x))
#define T25  (float)(25+273.15)
#define BETA 4480
#define R25 10000
//#define NTC_TEMP(x) (float)((1/((1 / T25) + ((log(x / R25)) / BETA)))-273.15)

#define K 3 // The register length aust be a power of two to avoid multiplies.
#define N(x) ((uint16_t)(1<<x)) // Register length is 2^k.

#define START_DIMMING 0.9
#define START_DERATING 60 //80
#define STOP_DIMMING 0
#define STOP_DERATING 80 //100
//#define ORDINATE (float)((START_DIMMING - STOP_DIMMING) / (START_DERATING - STOP_DERATING))
//#define ABSCISSA  (float)(-((ORDINATE * START_DERATING) - (START_DIMMING)))
#define ORDINATE (float)-0.0265
#define ABSCISSA (float)2.65
#define RATIO X-AXIS
//state variable definition

typedef enum {
    STANDBY,
    SOFTSTART,
    RUNNING,
    FAULT,
}State_e ;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
