/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
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
#define LIMIT_SW_2_Pin GPIO_PIN_2
#define LIMIT_SW_2_GPIO_Port GPIOE
#define LIMIT_SW_5_Pin GPIO_PIN_3
#define LIMIT_SW_5_GPIO_Port GPIOE
#define LIMIT_SW_3_Pin GPIO_PIN_4
#define LIMIT_SW_3_GPIO_Port GPIOE
#define PE5_LM629_CS_S0_Pin GPIO_PIN_5
#define PE5_LM629_CS_S0_GPIO_Port GPIOE
#define PE6_LM629_LT1_Pin GPIO_PIN_6
#define PE6_LM629_LT1_GPIO_Port GPIOE
#define LCD_D5_Pin GPIO_PIN_13
#define LCD_D5_GPIO_Port GPIOC
#define LCD_D6_Pin GPIO_PIN_14
#define LCD_D6_GPIO_Port GPIOC
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOC
#define PHO_MOT2_A_Pin GPIO_PIN_0
#define PHO_MOT2_A_GPIO_Port GPIOH
#define PH1_MOT1_B_Pin GPIO_PIN_1
#define PH1_MOT1_B_GPIO_Port GPIOH
#define LASER_ADC1_Pin GPIO_PIN_0
#define LASER_ADC1_GPIO_Port GPIOC
#define LASER_ADC1C1_Pin GPIO_PIN_1
#define LASER_ADC1C1_GPIO_Port GPIOC
#define LASER_ADC1C2_Pin GPIO_PIN_2
#define LASER_ADC1C2_GPIO_Port GPIOC
#define LASER_ADC1C3_Pin GPIO_PIN_3
#define LASER_ADC1C3_GPIO_Port GPIOC
#define PA0_UART4_TX_Pin GPIO_PIN_0
#define PA0_UART4_TX_GPIO_Port GPIOA
#define PA1_UART4_RX_Pin GPIO_PIN_1
#define PA1_UART4_RX_GPIO_Port GPIOA
#define GLA_TIM_Pin GPIO_PIN_2
#define GLA_TIM_GPIO_Port GPIOA
#define PA3_GLA_A_Pin GPIO_PIN_3
#define PA3_GLA_A_GPIO_Port GPIOA
#define PA4_SPI1_NSS_Pin GPIO_PIN_4
#define PA4_SPI1_NSS_GPIO_Port GPIOA
#define PA5_SPI1_SCK_Pin GPIO_PIN_5
#define PA5_SPI1_SCK_GPIO_Port GPIOA
#define PA6_SPI1_MISO_Pin GPIO_PIN_6
#define PA6_SPI1_MISO_GPIO_Port GPIOA
#define PA7_SPI1_MOSI_Pin GPIO_PIN_7
#define PA7_SPI1_MOSI_GPIO_Port GPIOA
#define PC4_MOT1_A_Pin GPIO_PIN_4
#define PC4_MOT1_A_GPIO_Port GPIOC
#define PC5_MOT2_B_Pin GPIO_PIN_5
#define PC5_MOT2_B_GPIO_Port GPIOC
#define PB0_LM629_WR_Pin GPIO_PIN_0
#define PB0_LM629_WR_GPIO_Port GPIOB
#define PE1_LM629_2LT2_Pin GPIO_PIN_1
#define PE1_LM629_2LT2_GPIO_Port GPIOB
#define PB2_LM629_RD_Pin GPIO_PIN_2
#define PB2_LM629_RD_GPIO_Port GPIOB
#define PE7_LM629_2LT1_Pin GPIO_PIN_7
#define PE7_LM629_2LT1_GPIO_Port GPIOE
#define PE8_LM629_RESET_Pin GPIO_PIN_8
#define PE8_LM629_RESET_GPIO_Port GPIOE
#define PE9_STEPPER_PULSE_Pin GPIO_PIN_9
#define PE9_STEPPER_PULSE_GPIO_Port GPIOE
#define PE10_LM629_CS_S1_Pin GPIO_PIN_10
#define PE10_LM629_CS_S1_GPIO_Port GPIOE
#define LIMIT_SW_7_Pin GPIO_PIN_11
#define LIMIT_SW_7_GPIO_Port GPIOE
#define DCV1_Pin GPIO_PIN_12
#define DCV1_GPIO_Port GPIOE
#define DCV2_Pin GPIO_PIN_13
#define DCV2_GPIO_Port GPIOE
#define DCV3_Pin GPIO_PIN_14
#define DCV3_GPIO_Port GPIOE
#define DCV4_Pin GPIO_PIN_15
#define DCV4_GPIO_Port GPIOE
#define PB10_I2C2_SCL_Pin GPIO_PIN_10
#define PB10_I2C2_SCL_GPIO_Port GPIOB
#define PB11_I2C2_SDA_Pin GPIO_PIN_11
#define PB11_I2C2_SDA_GPIO_Port GPIOB
#define PB12_CAN_RX_Pin GPIO_PIN_12
#define PB12_CAN_RX_GPIO_Port GPIOB
#define PB13_CAN_TX_Pin GPIO_PIN_13
#define PB13_CAN_TX_GPIO_Port GPIOB
#define PB14_MOT2_PWM_Pin GPIO_PIN_14
#define PB14_MOT2_PWM_GPIO_Port GPIOB
#define PB15_MOT1_PWM_Pin GPIO_PIN_15
#define PB15_MOT1_PWM_GPIO_Port GPIOB
#define PD8_LM629_CS_S0_Pin GPIO_PIN_8
#define PD8_LM629_CS_S0_GPIO_Port GPIOD
#define PD9_LM629_CS_S1_Pin GPIO_PIN_9
#define PD9_LM629_CS_S1_GPIO_Port GPIOD
#define PD10_RELIMATE_1_Pin GPIO_PIN_10
#define PD10_RELIMATE_1_GPIO_Port GPIOD
#define PD11_LM629_LT2_Pin GPIO_PIN_11
#define PD11_LM629_LT2_GPIO_Port GPIOD
#define PD12_LM629_PS_Pin GPIO_PIN_12
#define PD12_LM629_PS_GPIO_Port GPIOD
#define PD13_RELIMATE_2_Pin GPIO_PIN_13
#define PD13_RELIMATE_2_GPIO_Port GPIOD
#define PD14_NANO_DATA1_Pin GPIO_PIN_14
#define PD14_NANO_DATA1_GPIO_Port GPIOD
#define PD15_NANO_DATA2_Pin GPIO_PIN_15
#define PD15_NANO_DATA2_GPIO_Port GPIOD
#define PC6_RELIMATE_2_Pin GPIO_PIN_6
#define PC6_RELIMATE_2_GPIO_Port GPIOC
#define PC7_RELIMATE_1_Pin GPIO_PIN_7
#define PC7_RELIMATE_1_GPIO_Port GPIOC
#define PC8_LM629_EN1_Pin GPIO_PIN_8
#define PC8_LM629_EN1_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOC
#define PA8_LM629_EN2_Pin GPIO_PIN_8
#define PA8_LM629_EN2_GPIO_Port GPIOA
#define PA9_USART1_TX_Pin GPIO_PIN_9
#define PA9_USART1_TX_GPIO_Port GPIOA
#define PA10_USART1_RX_Pin GPIO_PIN_10
#define PA10_USART1_RX_GPIO_Port GPIOA
#define LIMIT_SW_9_Pin GPIO_PIN_15
#define LIMIT_SW_9_GPIO_Port GPIOA
#define PC10_STEPPER_DIR_Pin GPIO_PIN_10
#define PC10_STEPPER_DIR_GPIO_Port GPIOC
#define LCD_EN_Pin GPIO_PIN_11
#define LCD_EN_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_12
#define LCD_D4_GPIO_Port GPIOC
#define PD0_LM629_D0_Pin GPIO_PIN_0
#define PD0_LM629_D0_GPIO_Port GPIOD
#define PD1_LM629_D1_Pin GPIO_PIN_1
#define PD1_LM629_D1_GPIO_Port GPIOD
#define PD2_LM629_D2_Pin GPIO_PIN_2
#define PD2_LM629_D2_GPIO_Port GPIOD
#define PD3_LM629_D3_Pin GPIO_PIN_3
#define PD3_LM629_D3_GPIO_Port GPIOD
#define PD4_LM629_D4_Pin GPIO_PIN_4
#define PD4_LM629_D4_GPIO_Port GPIOD
#define PD5_LM629_D5_Pin GPIO_PIN_5
#define PD5_LM629_D5_GPIO_Port GPIOD
#define PD6_LM629_D6_Pin GPIO_PIN_6
#define PD6_LM629_D6_GPIO_Port GPIOD
#define PD7_LM629_D7_Pin GPIO_PIN_7
#define PD7_LM629_D7_GPIO_Port GPIOD
#define PB3_RELIMATE_1_Pin GPIO_PIN_3
#define PB3_RELIMATE_1_GPIO_Port GPIOB
#define PB4_RELIMATE_2_Pin GPIO_PIN_4
#define PB4_RELIMATE_2_GPIO_Port GPIOB
#define PB5_RELIMATE_3_Pin GPIO_PIN_5
#define PB5_RELIMATE_3_GPIO_Port GPIOB
#define PB6_RELIMATE_4_Pin GPIO_PIN_6
#define PB6_RELIMATE_4_GPIO_Port GPIOB
#define PB7_LM629_CLK_Pin GPIO_PIN_7
#define PB7_LM629_CLK_GPIO_Port GPIOB
#define PB8_I2C1_SCL_Pin GPIO_PIN_8
#define PB8_I2C1_SCL_GPIO_Port GPIOB
#define PB9_I2C1_SDA_Pin GPIO_PIN_9
#define PB9_I2C1_SDA_GPIO_Port GPIOB
#define LIMIT_SW_1_Pin GPIO_PIN_0
#define LIMIT_SW_1_GPIO_Port GPIOE
#define LIMIT_SW_4_Pin GPIO_PIN_1
#define LIMIT_SW_4_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
