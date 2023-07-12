/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PE5_LM629_CS_S0_Pin|PE6_LM629_LT1_Pin|PE7_LM629_2LT1_Pin|PE8_LM629_RESET_Pin
                          |PE10_LM629_CS_S1_Pin|DCV1_Pin|DCV2_Pin|DCV3_Pin
                          |DCV4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|PC4_MOT1_A_Pin
                          |PC5_MOT2_B_Pin|PC6_RELIMATE_2_Pin|PC7_RELIMATE_1_Pin|PC8_LM629_EN1_Pin
                          |LCD_RS_Pin|PC10_STEPPER_DIR_Pin|LCD_EN_Pin|LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, PHO_MOT2_A_Pin|PH1_MOT1_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA3_GLA_A_Pin|PA4_SPI1_NSS_Pin|PA8_LM629_EN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PB0_LM629_WR_Pin|PE1_LM629_2LT2_Pin|PB2_LM629_RD_Pin|PB3_RELIMATE_1_Pin
                          |PB5_RELIMATE_3_Pin|PB6_RELIMATE_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, PD8_LM629_CS_S0_Pin|PD9_LM629_CS_S1_Pin|PD10_RELIMATE_1_Pin|PD11_LM629_LT2_Pin
                          |PD12_LM629_PS_Pin|PD13_RELIMATE_2_Pin|PD14_NANO_DATA1_Pin|PD15_NANO_DATA2_Pin
                          |PD0_LM629_D0_Pin|PD1_LM629_D1_Pin|PD2_LM629_D2_Pin|PD3_LM629_D3_Pin
                          |PD4_LM629_D4_Pin|PD5_LM629_D5_Pin|PD6_LM629_D6_Pin|PD7_LM629_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin */
  GPIO_InitStruct.Pin = LIMIT_SW_2_Pin|LIMIT_SW_5_Pin|LIMIT_SW_3_Pin|LIMIT_SW_7_Pin
                          |LIMIT_SW_1_Pin|LIMIT_SW_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin PEPin PEPin
                           PEPin */
  GPIO_InitStruct.Pin = PE5_LM629_CS_S0_Pin|PE6_LM629_LT1_Pin|PE7_LM629_2LT1_Pin|PE8_LM629_RESET_Pin
                          |PE10_LM629_CS_S1_Pin|DCV1_Pin|DCV2_Pin|DCV3_Pin
                          |DCV4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin PCPin
                           PCPin PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin|PC4_MOT1_A_Pin
                          |PC5_MOT2_B_Pin|PC6_RELIMATE_2_Pin|PC7_RELIMATE_1_Pin|PC8_LM629_EN1_Pin
                          |LCD_RS_Pin|PC10_STEPPER_DIR_Pin|LCD_EN_Pin|LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PHPin PHPin */
  GPIO_InitStruct.Pin = PHO_MOT2_A_Pin|PH1_MOT1_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = PA3_GLA_A_Pin|PA4_SPI1_NSS_Pin|PA8_LM629_EN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = PB0_LM629_WR_Pin|PE1_LM629_2LT2_Pin|PB2_LM629_RD_Pin|PB3_RELIMATE_1_Pin
                          |PB5_RELIMATE_3_Pin|PB6_RELIMATE_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin
                           PDPin PDPin PDPin PDPin */
  GPIO_InitStruct.Pin = PD8_LM629_CS_S0_Pin|PD9_LM629_CS_S1_Pin|PD10_RELIMATE_1_Pin|PD11_LM629_LT2_Pin
                          |PD12_LM629_PS_Pin|PD13_RELIMATE_2_Pin|PD14_NANO_DATA1_Pin|PD15_NANO_DATA2_Pin
                          |PD0_LM629_D0_Pin|PD1_LM629_D1_Pin|PD2_LM629_D2_Pin|PD3_LM629_D3_Pin
                          |PD4_LM629_D4_Pin|PD5_LM629_D5_Pin|PD6_LM629_D6_Pin|PD7_LM629_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LIMIT_SW_9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LIMIT_SW_9_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = PB4_RELIMATE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB4_RELIMATE_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
