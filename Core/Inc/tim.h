/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim4;

extern TIM_HandleTypeDef htim7;

extern TIM_HandleTypeDef htim9;

extern TIM_HandleTypeDef htim12;

/* USER CODE BEGIN Private defines */
#define ENC_Y_COUNT		(360)
#define ENC_X_COUNT		(360)

#define ENC_WHEEL_D		(12.0f)//in cm
#define X_ENC_WHEEL_D  (10.0f)//in cm

#define ENC_T1_COUNT (360)
#define ENC_T2_COUNT (360)
#define ENCODER_X_POSITION_CONST	(float)(ENC_X_COUNT*4.0f/(360))
#define ENCODER_Y_POSITION_CONST	(float)(ENC_Y_COUNT*4.0f/(360))
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM4_Init(void);
void MX_TIM7_Init(void);
void MX_TIM9_Init(void);
void MX_TIM12_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

