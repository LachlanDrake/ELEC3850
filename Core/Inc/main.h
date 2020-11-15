/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Echo_Pin GPIO_PIN_2
#define Echo_GPIO_Port GPIOE
#define Hall_Effect_Pin GPIO_PIN_5
#define Hall_Effect_GPIO_Port GPIOE
#define Hall_Effect_EXTI_IRQn EXTI9_5_IRQn
#define Right_blinker_Pin GPIO_PIN_6
#define Right_blinker_GPIO_Port GPIOE
#define Headlight_ADC_Pin GPIO_PIN_4
#define Headlight_ADC_GPIO_Port GPIOA
#define Battery_ADC_Pin GPIO_PIN_5
#define Battery_ADC_GPIO_Port GPIOA
#define Forward_PWM_Pin GPIO_PIN_6
#define Forward_PWM_GPIO_Port GPIOA
#define LCDTP_CS_Pin GPIO_PIN_4
#define LCDTP_CS_GPIO_Port GPIOC
#define LCDTP_IRQ_Pin GPIO_PIN_5
#define LCDTP_IRQ_GPIO_Port GPIOC
#define BL_PWM_Pin GPIO_PIN_0
#define BL_PWM_GPIO_Port GPIOB
#define Amber_light_Pin GPIO_PIN_12
#define Amber_light_GPIO_Port GPIOB
#define Headlight_PWM_Pin GPIO_PIN_12
#define Headlight_PWM_GPIO_Port GPIOD
#define Reverse_GPIO_Pin GPIO_PIN_7
#define Reverse_GPIO_GPIO_Port GPIOC
#define Forward_GPIO_Pin GPIO_PIN_9
#define Forward_GPIO_GPIO_Port GPIOC
#define Servo_PWM_Pin GPIO_PIN_8
#define Servo_PWM_GPIO_Port GPIOA
#define RED_Light_Pin GPIO_PIN_10
#define RED_Light_GPIO_Port GPIOA
#define Left_blinker_Pin GPIO_PIN_2
#define Left_blinker_GPIO_Port GPIOD
#define IMU_SCL_Pin GPIO_PIN_6
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_7
#define IMU_SDA_GPIO_Port GPIOB
#define GREEN_Light_Pin GPIO_PIN_0
#define GREEN_Light_GPIO_Port GPIOE
#define Trig_Pin GPIO_PIN_1
#define Trig_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
