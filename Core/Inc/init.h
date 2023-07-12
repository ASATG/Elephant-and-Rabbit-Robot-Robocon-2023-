#ifndef INIT_H_
#define INIT_H_

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
//#include "SDC_CAN.h"
//#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "prep.h"
#include "motor.h"
#include "base_drive.h"
//#include "laser.h"
#include "encoder.h"
//#include "calib.h"
#include "ps4.h"
//#include "adxrs.h"
//#include "debug.h"
//#include "imu.h"
//#include "mpu9250.h"
#include "lcd.h"
//#include "i2c_lcd.h"
//#include "imu_bno055.h"
#include "delay.h"

#define LASER_MODULE			    (&hadc1)

#define ENCODER_X_MODULE			(&htim1)
#define MOTOR_MODULE					(&htim12)
#define MOTOR_MODULE_MOTOR3      (&htim9)
#define MOTOR_MODULE_MOTOR124    (&htim12)
#define MOTOR_MODULE_MOTOR14     (&htim14)      //////ACP
#define DELAY_MODULE					(&htim7)
#define ENCODER_Y_MODULE			(&htim1)
#define ENCODER_Z_MODULE			(&htim3)
#define LM629_CLOCK_MODULE 		(&htim4)
#define LM629_CLOCK_CHANNEL		TIM_CHANNEL_2

//#define BLDC_TIM_MODULE       (&htim13)        ///////ACP

#define CONSOLE_MODULE			  (&huart4)
//#define UART3_MODULE					(&huart3)
//#define UART2_MODULE					(&huart2)
#define LCD_MODULE						(&huart1)

#define I2C_BUS						    (&hi2c1)

//#define ADXRS_MODULE 			    (&hspi1)


#define BLDC_TIM          TIM13                /////ACP
#define ENCODER_X_TIM				TIM1
#define GLA_TIM						TIM9
#define MOTOR_TIM124				TIM12
//#define MOTOR_TIM3					TIM15
#define MOTOR_TIM14         TIM14           //////ACP
#define DELAY_TIM						TIM7
#define ENCODER_Y_TIM				TIM2
#define ENCODER_Z_TIM				TIM3
#define LM629_CLOCK_TIM			TIM4
#define ENCODER_THROW_ONE_TIM				TIM1
#define ENCODER_THROW_TWO_TIM				TIM3
// modules defination

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c1;
//extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim14;            //////ACP


extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;

extern int receivednum;
//motor definitions 4 wheel Omni

#define FRONT_LEFT_MOTOR 		1
#define FRONT_RIGHT_MOTOR 	2
#define BACK_RIGHT_MOTOR 		3
#define BACK_LEFT_MOTOR 		4


#define motor_5 5
#define motor_6 6
#define motor_7 7
#define motor_8 8

#define throw_motor 0

// 3 wheel Omni

#define FRONT_MOTOR 	2
#define RIGHT_MOTOR 	1
#define LEFT_MOTOR 		3


#define LM629_GPIO_Port 		GPIOD

extern motor *suction_pump;
extern motor *KICK_MOTOR;
extern motor *THROW_MOTOR_ONE;
extern motor *THROW_MOTOR_TWO;
extern motor *YAW_MOTOR_ONE;
extern motor *YAW_MOTOR_TWO;
extern motor *SHOULDER;
extern motor *GLA_MOTOR;
extern motor *TURN_TABLE;        /////ACP
extern motor *SHOULDER;
extern motor *ELBOW;
extern motor *GRIPPER;

extern servo *BLDC;               ///////ACP

extern uint8_t global_temp8[10];
extern uint16_t global_temp16[10];
extern uint32_t global_temp32[10];
extern uint32_t adc_data_f[8];
extern uint32_t adc_data_b[6];
extern uint32_t adc_dma_counter;
extern uint8_t lm629_data[10];
extern uint16_t gla_current_pos[1];

extern bool f_gerege_up;
extern bool f_shagai_grip; 
extern bool mech_shift ;
extern bool shagai_up_flag;
extern bool task_start_flag;
extern bool restart_flag;
extern bool retry_flag;

extern bool ball_grip;


extern bool flag_pid;
extern bool flag_sample;
extern uint32_t k,j;
extern float dt,yaw;
extern float x_susp,y_susp;
extern inline void interrupt_handler_1(void);
extern float tick;

typedef enum
{
	CALB=0,
	PLAY=1,
	AUTO=2,
} bot_mode;


extern bot_mode robot_mode;
extern float gyro_data[1];

void lm629_clock_start(TIM_HandleTypeDef *htim);
void motors_pwm_start(TIM_HandleTypeDef *htim);
void delay_init(TIM_HandleTypeDef *htim);
void adc_init(void);
void servo_pwm_start(TIM_HandleTypeDef *htim);         /////wasnt here from the start



#endif /* INIT_H_ */
