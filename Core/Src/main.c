/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_CAN2_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_TIM4_Init();
	MX_TIM12_Init();
	MX_TIM1_Init();
	MX_TIM7_Init();
	MX_TIM9_Init();
	/* USER CODE BEGIN 2 */
	int accnf = 600;
	delay_init(DELAY_MODULE);
	lm629_clock_start(LM629_CLOCK_MODULE);
	motors_pwm_start(MOTOR_MODULE);
	//	encoders_init();
	adc_init();
	mux_auto();
	lcd_init();
	// CAN_filterConfig();
	// HAL_CAN_Start(&hcan2);
	//	servo_pwm_start(BLDC_TIM_MODULE);
	int q, tick, dt, prev_tick, pd;
	int ps4_mode = 0;
	int st = 0;
	int pickmode = 0;
	int mode = 0;
	int limit_value1 = 0;
	int limit_value4 = 0;
	int limit_value3 = 0;
	int limit_value2 = 0;
	char *buffer1[1];
	int ret;
	int stop = 0;
	int j = 800;
	int steppermode = 0;

	lcd_gotoxy(1, 1);
	lcd_string("Robocon 2023");
	HAL_Delay(800);
	lcd_clear();

	int status = 0;
	//		encoders_define_home();
	lcd_clear();
	lcd_gotoxy(1, 1);
	lcd_string("Homed");
	HAL_Delay(800);
	lcd_clear();
	lcd_clear();
	float x = 0;
	float y = 0;

	int prev_distance = 0;
	/////////////////////
	//	    robot_position_init(50,50);
	robot_velocity_init(accnf);
	lcd_clear();
	lcd_gotoxy(1, 1);
	lcd_string("Velocity init done");
	HAL_Delay(800);
	lcd_clear();

	while (pairing_flag == false)
	{
		lcd_gotoxy(4, 1);
		connect_ps4();
		lcd_gotoxy(1, 4);
		lcd_string("Pair Lost");
	}
	lcd_clear();

	stick_invert_flag = false;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (0)
	{
		VELOCITY_MAX_SPEED = 350;
		receive_ps4();
		lcd_print(1, 1, read_real_position(1), 5);
		lcd_print(2, 1, read_real_position(2), 5);
		lcd_print(3, 1, read_real_position(3), 5);
		lcd_print(4, 1, read_real_position(4), 5);

		lcd_print(4, 10, HAL_GPIO_ReadPin(LIMIT_SW_1_GPIO_Port, LIMIT_SW_1_Pin), 1);
		lcd_print(4, 11, HAL_GPIO_ReadPin(LIMIT_SW_2_GPIO_Port, LIMIT_SW_2_Pin), 1);
		lcd_print(4, 12, HAL_GPIO_ReadPin(LIMIT_SW_3_GPIO_Port, LIMIT_SW_3_Pin), 1);
		lcd_print(4, 13, HAL_GPIO_ReadPin(LIMIT_SW_4_GPIO_Port, LIMIT_SW_4_Pin), 1);
		lcd_print(4, 14, HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin), 1);
		lcd_print(4, 15, HAL_GPIO_ReadPin(LIMIT_SW_7_GPIO_Port, LIMIT_SW_7_Pin), 1);
		lcd_print(4, 16, HAL_GPIO_ReadPin(LIMIT_SW_9_GPIO_Port, LIMIT_SW_9_Pin), 1);
		lcd_print(3, 10, j, 4);

		//	  if (PS4_L1_PRESSED)
		//	  		{
		//		  	  	  robot_newvelocity_run(100, 0, 0, SMOOTH_STOP);
		//	  		}
		//	  if (PS4_L2_PRESSED)
		//	  		{
		//		  	  	  robot_newvelocity_run(0, 0, 0, SMOOTH_STOP);
		//	  		}
		//	  if (PS4_R1_PRESSED)
		//			{
		//				  robot_newvelocity_run(100, 90, 0, SMOOTH_STOP);
		//			}
		//	  if (PS4_R2_PRESSED)
		//			{
		//				  robot_newvelocity_run(0, 0, 45, SMOOTH_STOP);
		//			}
		//	  if (PS4_CROSS_PRESSED)
		//	  			{
		//	  				  robot_newvelocity_run(100,180, 0, SMOOTH_STOP);
		//	  			}
		robot_joystick_velocity();
		SAVE_CONSOLE;
	}
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/////////////////////////

		/////////////////////////

		receive_ps4();
		lcd_print(1, 1, read_real_position(1), 5);
		lcd_print(2, 1, read_real_position(2), 5);
		lcd_print(3, 1, read_real_position(3), 5);
		lcd_print(4, 1, read_real_position(4), 5);

		lcd_print(4, 10, HAL_GPIO_ReadPin(LIMIT_SW_1_GPIO_Port, LIMIT_SW_1_Pin), 1);
		lcd_print(4, 11, HAL_GPIO_ReadPin(LIMIT_SW_2_GPIO_Port, LIMIT_SW_2_Pin), 1);
		lcd_print(4, 12, HAL_GPIO_ReadPin(LIMIT_SW_3_GPIO_Port, LIMIT_SW_3_Pin), 1);
		lcd_print(4, 13, HAL_GPIO_ReadPin(LIMIT_SW_4_GPIO_Port, LIMIT_SW_4_Pin), 1);
		lcd_print(4, 14, HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin), 1);
		lcd_print(4, 15, HAL_GPIO_ReadPin(LIMIT_SW_7_GPIO_Port, LIMIT_SW_7_Pin), 1);
		lcd_print(4, 16, HAL_GPIO_ReadPin(LIMIT_SW_9_GPIO_Port, LIMIT_SW_9_Pin), 1);
		lcd_print(3, 10, j, 4);

		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 1)
		{
			st = 0;
		}
		if (st > 19)
		{
			st = 19;
		}
		if (st < 0)
		{
			st = 0;
		}

		if (PS4_CROSS_PRESSED)
		{
			robot_velocity_init(accnf);
			lcd_clear();
			lcd_gotoxy(1, 1);
			lcd_string("Velocity INIT Done");
			HAL_Delay(800);
			lcd_clear();
			lcd_print(1, 1, read_real_position(1), 5);
			lcd_print(2, 1, read_real_position(2), 5);
			lcd_print(3, 1, read_real_position(3), 5);
			lcd_print(4, 1, read_real_position(4), 5);

			lcd_print(4, 10, HAL_GPIO_ReadPin(LIMIT_SW_1_GPIO_Port, LIMIT_SW_1_Pin), 1);
			lcd_print(4, 11, HAL_GPIO_ReadPin(LIMIT_SW_2_GPIO_Port, LIMIT_SW_2_Pin), 1);
			lcd_print(4, 12, HAL_GPIO_ReadPin(LIMIT_SW_3_GPIO_Port, LIMIT_SW_3_Pin), 1);
			lcd_print(4, 13, HAL_GPIO_ReadPin(LIMIT_SW_4_GPIO_Port, LIMIT_SW_4_Pin), 1);
			lcd_print(4, 14, HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin), 1);
			lcd_print(3, 10, j, 4);
		}
		//////////////  BD Mode
		if (PS4_L1_PRESSED)
		{
			VELOCITY_MAX_SPEED = 100;
		}

		if (PS4_L1_RELEASED)
		{
			VELOCITY_MAX_SPEED = 400;
		}

		if (PS4_OPTIONS_PRESSED)
		{
			ps4_mode = !(ps4_mode);
			lcd_print(4, 17, ps4_mode, 1);
		}
		if (pickmode == 1)
		{

			if (HAL_GPIO_ReadPin(LIMIT_SW_7_GPIO_Port, LIMIT_SW_7_Pin) == 1)
			{
				mode = 1;
			}
			if (HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin) == 1)
			{
				mode = 0;
			}
			if (HAL_GPIO_ReadPin(LIMIT_SW_1_GPIO_Port, LIMIT_SW_1_Pin) == 1)
			{
				if (mode == 0)
				{
					limit_value1 = 100;
				}
				else
				{
					limit_value1 = 250; // 160
				}
			}
			else
			{
				limit_value1 = 10;
			}

			motor_fwd(ELBOW, limit_value1);
			if (HAL_GPIO_ReadPin(LIMIT_SW_3_GPIO_Port, LIMIT_SW_3_Pin) == 1)
			{
				limit_value4 = 80;
			}
			else
			{
				pickmode = 0;
				limit_value4 = 0;
			}
			motor_rev(SHOULDER, limit_value4);
		}

		if (ps4_mode == 0) // RING PICKING MODE
		{

			////////TURN TABLE   --  SQUARE, CIRCLE
			if (PS4_SQUARE)
			{

				if (HAL_GPIO_ReadPin(LIMIT_SW_3_GPIO_Port, LIMIT_SW_3_Pin) == 0)
				{
					limit_value3 = 100;
				}
				else
				{
					limit_value3 = 0;
				}

				motor_fwd(SHOULDER, limit_value3);
			}
			if (PS4_SQUARE_RELEASED)
			{

				motor_fwd(SHOULDER, 0);
			}

			if (PS4_CIRCLE)
			{

				if (HAL_GPIO_ReadPin(LIMIT_SW_4_GPIO_Port, LIMIT_SW_4_Pin) == 0)
				{
					limit_value4 = 120;
				}
				else
				{
					limit_value4 = 0;
				}
				motor_rev(SHOULDER, limit_value4);
			}
			if (PS4_CIRCLE_RELEASED)
			{

				motor_rev(SHOULDER, 0);
			}
			///////////////////////////////////////////////////////////////////////////
			if (PS4_L1)
			{
				// Turn Table
				if (PS4_SQUARE)
				{
					motor_fwd(SHOULDER, 100);
				}
				if (PS4_SQUARE_RELEASED)
				{
					motor_fwd(SHOULDER, 0);
				}

				if (PS4_CIRCLE)
				{
					motor_rev(SHOULDER, 120);
				}
				if (PS4_CIRCLE_RELEASED)
				{
					motor_rev(SHOULDER, 0);
				}

				// Side Tower
				if (PS4_PAD_UP)
				{
					motor_fwd(ELBOW, 150);
				}
				if (PS4_PAD_UP_RELEASED)
				{
					motor_fwd(ELBOW, 0);
				}
				if (PS4_PAD_DOWN)
				{
					motor_rev(ELBOW, 70);
				}
				if (PS4_PAD_DOWN_RELEASED)
				{
					motor_rev(ELBOW, 0);
				}
			}

			if (PS4_TOUCHPAD_PRESSED) // SIDE TOWER GRIPPER
			{
				HAL_GPIO_TogglePin(DCV2_GPIO_Port, DCV2_Pin);
				HAL_Delay(100);
			}

			if (PS4_TRIANGLE_PRESSED) // SIDE TOWER UP DOWN
			{
				HAL_GPIO_TogglePin(DCV3_GPIO_Port, DCV3_Pin);
			}
			if (PS4_PAD_LEFT_PRESSED) // SIDE TOWER UP DOWN
			{
				HAL_GPIO_TogglePin(DCV4_GPIO_Port, DCV4_Pin);
			}

			////////// SIDE TOWER  -- PAD_UP(up), PAD_DOWN(down) with limit switch

			if (PS4_PAD_UP)
			{
				//				limit_value1 = 100;

				if (HAL_GPIO_ReadPin(LIMIT_SW_1_GPIO_Port, LIMIT_SW_1_Pin) == 0)
				{
					limit_value1 = 150;
				}
				else
				{
					limit_value1 = 30;
				}

				motor_fwd(ELBOW, limit_value1);
			}
			if (PS4_PAD_UP_RELEASED)
			{

				motor_fwd(ELBOW, 0);
			}
			if (PS4_PAD_DOWN)
			{

				if (HAL_GPIO_ReadPin(LIMIT_SW_2_GPIO_Port, LIMIT_SW_2_Pin) == 1)
				{

					limit_value2 = 70;
				}
				else
				{
					limit_value2 = 0;
				}
				motor_rev(ELBOW, limit_value2);
			}
			if (PS4_PAD_DOWN_RELEASED)
			{

				motor_rev(ELBOW, 0);
			}
		}

		if (ps4_mode == 1) // RING SHOOTING MODE
		{
			if (PS4_L1)
			{
				steppermode = 1;
			}
			else
			{
				steppermode = 0;
			}

			if (steppermode == 0)
			{
				if (PS4_PAD_DOWN_PRESSED && (HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin) == 0))
				{
					st--;
					//				if(st >= 0){
					buffer1[0] = '2';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					lcd_gotoxy(1, 10);
					lcd_string("Down");
					//				}
				}
				else if (PS4_PAD_DOWN_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
				if (PS4_PAD_UP_PRESSED)
				{
					st++;
					//				if(st <= 19){
					buffer1[0] = '1';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					lcd_gotoxy(1, 10);
					lcd_string("Up");
					//				}
				}
				else if (PS4_PAD_UP_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
			}
			if (steppermode == 1)
			{
				if (PS4_PAD_DOWN_PRESSED && (HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin) == 0))
				{
					st--;
					//				if(st >= 0){
					buffer1[0] = '6';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					lcd_gotoxy(1, 10);
					lcd_string("Down");
					//				}
				}
				else if (PS4_PAD_DOWN_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
				if (PS4_PAD_UP_PRESSED)
				{
					st++;
					//				if(st <= 19){
					buffer1[0] = '5';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					lcd_gotoxy(1, 10);
					lcd_string("Up");
					//				}
				}
				else if (PS4_PAD_UP_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
			}
			/////////  GLA  --  PAD-LEFT, PAD-RIGHT
			if (PS4_PAD_RIGHT)
			{
				motor_fwd(GLA_MOTOR, 240);
			}
			if (PS4_PAD_RIGHT_RELEASED)
			{
				motor_fwd(GLA_MOTOR, 0);
			}
			if (PS4_PAD_LEFT)
			{
				motor_rev(GLA_MOTOR, 240);
			}
			if (PS4_PAD_LEFT_RELEASED)
			{
				motor_rev(GLA_MOTOR, 0);
			}

			//////// DCV   --   OPTIONS, SHARE
			if (PS4_TOUCHPAD_PRESSED) // RING PUSHING MECHANISM
			{
				HAL_GPIO_TogglePin(DCV1_GPIO_Port, DCV1_Pin);
				HAL_Delay(500);
				HAL_GPIO_TogglePin(DCV1_GPIO_Port, DCV1_Pin);
				HAL_Delay(500);
				buffer1[0] = '1';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				lcd_gotoxy(1, 10);
				lcd_string("Up");
				buffer1[0] = 'a';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				st++;
				//				if(st <= 19){
				//					buffer1[0] = '1';
				//				}
			}
			//			else if (PS4_TOUCHPAD_RELEASED){
			//				buffer1[0] = 'a';
			//			}

			///////BLDC  --  PAD-UP-&-L1, PAD-DOWN-&-L2

			if (PS4_L2)
			{
				if (PS4_R1_PRESSED)
				{
					buffer1[0] = 'd';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					j += 2;
				}
				else if (PS4_R1_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
				if (PS4_R2_PRESSED)
				{
					buffer1[0] = 'e';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					j -= 2;
				}
				else if (PS4_R2_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
			}
			else
			{
				if (PS4_R1_PRESSED)
				{
					buffer1[0] = 'b';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					j += 5;
				}
				else if (PS4_R1_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
				if (PS4_R2_PRESSED)
				{
					buffer1[0] = 'c';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
					j -= 5;
				}
				else if (PS4_R2_RELEASED)
				{
					buffer1[0] = 'a';
					ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				}
			}
			if (PS4_CIRCLE_PRESSED)
			{

				buffer1[0] = 'f';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				j = 1045;
			}
			else if (PS4_CIRCLE_RELEASED)
			{
				buffer1[0] = 'a';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
			}
			if (PS4_SQUARE_PRESSED)
			{
				buffer1[0] = 'g';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				j = 1000;
			}
			else if (PS4_SQUARE_RELEASED)
			{
				buffer1[0] = 'a';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
			}
			if (PS4_TRIANGLE_PRESSED)
			{

				buffer1[0] = 'h';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				j = 800;
			}
			else if (PS4_TRIANGLE_RELEASED)
			{
				buffer1[0] = 'a';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
			}

			////////// Stepper -- PAD_UP, PAD_DOWN, SHARE
			if (PS4_SHARE_PRESSED)
			{
				stop = 1;
				buffer1[0] = '3';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
			}
			else if (PS4_SHARE_RELEASED)
			{
				buffer1[0] = 'a';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
			}

			if (HAL_GPIO_ReadPin(LIMIT_SW_5_GPIO_Port, LIMIT_SW_5_Pin) == 1 && stop == 1)
			{
				buffer1[0] = '4';
				ret = HAL_I2C_Master_Transmit(&hi2c1, 4 << 1, (uint8_t *)buffer1, 1, 100);
				st = 0;
				stop = 0;
			}

			robot_joystick_velocity();
			SAVE_CONSOLE;
		}
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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 64;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
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
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
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
