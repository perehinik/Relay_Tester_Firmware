/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
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
#define Out10_Pin GPIO_PIN_13
#define Out10_GPIO_Port GPIOC
#define Out12_Pin GPIO_PIN_14
#define Out12_GPIO_Port GPIOC
#define Out11_Pin GPIO_PIN_15
#define Out11_GPIO_Port GPIOC
#define Out13_Pin GPIO_PIN_0
#define Out13_GPIO_Port GPIOC
#define Out15_Pin GPIO_PIN_1
#define Out15_GPIO_Port GPIOC
#define Out14_Pin GPIO_PIN_2
#define Out14_GPIO_Port GPIOC
#define Out16_Pin GPIO_PIN_3
#define Out16_GPIO_Port GPIOC
#define RelOn7_Pin GPIO_PIN_4
#define RelOn7_GPIO_Port GPIOC
#define RelOn8_Pin GPIO_PIN_5
#define RelOn8_GPIO_Port GPIOC
#define But1_Pin GPIO_PIN_0
#define But1_GPIO_Port GPIOB
#define But1_EXTI_IRQn EXTI0_IRQn
#define But2_Pin GPIO_PIN_1
#define But2_GPIO_Port GPIOB
#define But2_EXTI_IRQn EXTI1_IRQn
#define But3_Pin GPIO_PIN_2
#define But3_GPIO_Port GPIOB
#define But3_EXTI_IRQn EXTI2_IRQn
#define RelOn5_Pin GPIO_PIN_10
#define RelOn5_GPIO_Port GPIOB
#define RelOn6_Pin GPIO_PIN_11
#define RelOn6_GPIO_Port GPIOB
#define RelOn3_Pin GPIO_PIN_12
#define RelOn3_GPIO_Port GPIOB
#define RelOn4_Pin GPIO_PIN_13
#define RelOn4_GPIO_Port GPIOB
#define RelOn1_Pin GPIO_PIN_14
#define RelOn1_GPIO_Port GPIOB
#define RelOn2_Pin GPIO_PIN_15
#define RelOn2_GPIO_Port GPIOB
#define Out1_Pin GPIO_PIN_6
#define Out1_GPIO_Port GPIOC
#define Out3_Pin GPIO_PIN_7
#define Out3_GPIO_Port GPIOC
#define Out2_Pin GPIO_PIN_8
#define Out2_GPIO_Port GPIOC
#define Out4_Pin GPIO_PIN_9
#define Out4_GPIO_Port GPIOC
#define RelOn9_Pin GPIO_PIN_8
#define RelOn9_GPIO_Port GPIOA
#define RelOn10_Pin GPIO_PIN_9
#define RelOn10_GPIO_Port GPIOA
#define Out6_Pin GPIO_PIN_10
#define Out6_GPIO_Port GPIOC
#define Out5_Pin GPIO_PIN_11
#define Out5_GPIO_Port GPIOC
#define Out7_Pin GPIO_PIN_12
#define Out7_GPIO_Port GPIOC
#define But4_Pin GPIO_PIN_4
#define But4_GPIO_Port GPIOB
#define But4_EXTI_IRQn EXTI4_IRQn
#define Out9_Pin GPIO_PIN_8
#define Out9_GPIO_Port GPIOB
#define Out8_Pin GPIO_PIN_9
#define Out8_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
