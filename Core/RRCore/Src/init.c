
#include "init.h"

motor mot_1 = {PC4_MOT1_A_GPIO_Port, PC4_MOT1_A_Pin, PH1_MOT1_B_GPIO_Port, PH1_MOT1_B_Pin, &(MOTOR_TIM124->CCR2)};
motor mot_2 = {PHO_MOT2_A_GPIO_Port, PHO_MOT2_A_Pin, PC5_MOT2_B_GPIO_Port, PC5_MOT2_B_Pin, &(MOTOR_TIM124->CCR1)};
motor mot_3 = {PA3_GLA_A_GPIO_Port, PA3_GLA_A_Pin, PA3_GLA_A_GPIO_Port, PA3_GLA_A_Pin, &(GLA_TIM->CCR1)};
//motor mot_4 = {MOT4_A_GPIO_Port, MOT4_A_Pin, MOT4_B_GPIO_Port, MOT4_B_Pin, &(MOTOR_TIM14->CCR1)};      ///ACP

motor *suction_pump = &mot_1;

servo BLDCM = {&(BLDC_TIM->CCR1), 800, 4400, 90, 0, 180};         /////ACP

//motor mot_1 = {MOT1_A_GPIO_Port, MOT1_A_Pin, MOT1_B_GPIO_Port, MOT1_B_Pin, &(MOTOR_TIM->CCR3)};


servo *BLDC = &BLDCM;       ////ACP

motor *KICK_MOTOR = &mot_1;

motor *THROW_MOTOR_ONE = &mot_1;
motor *THROW_MOTOR_TWO = &mot_2;
motor *YAW_MOTOR_ONE = &mot_1;
motor *YAW_MOTOR_TWO = &mot_2;
//motor *TURN_TABLE = &mot_4;
motor *GLA_MOTOR = &mot_3;

motor *SHOULDER = &mot_1;
motor *ELBOW = &mot_2;
motor *GRIPPER = &mot_2;



float x_susp = 0,y_susp = 0;

uint8_t global_temp8[10]={0};
uint16_t global_temp16[10]={0};
uint32_t global_temp32[10]={0};
uint8_t lm629_data[10] = {0};
uint32_t adc_data_f[8]={0};
uint32_t adc_data_b[6]={0};
uint16_t gla_current_pos[1] = {0};

float gyro_data[1] = {0};
float dt,yaw ;

bool f_gerege_up = false;
bool f_shagai_grip = false;
bool mech_shift = false;
bool task_start_flag = false;
bool shagai_up_flag = false;
bool restart_flag = false;
bool retry_flag = false;

bool ball_grip = false;

bot_mode robot_mode = PLAY;


void lm629_clock_start(TIM_HandleTypeDef *htim)
{
	HAL_TIM_OC_Start(LM629_CLOCK_MODULE,LM629_CLOCK_CHANNEL);
	_delay_ms(1);
	hardware_reset();
}

void motors_pwm_start(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(MOTOR_MODULE_MOTOR3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(MOTOR_MODULE_MOTOR124,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(MOTOR_MODULE_MOTOR124,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(MOTOR_MODULE_MOTOR124,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(MOTOR_MODULE_MOTOR3,TIM_CHANNEL_1);
}

void servo_pwm_start(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Start(htim,TIM_CHANNEL_1);
}

void delay_init(TIM_HandleTypeDef *htim)
{
	HAL_TIM_Base_Start(htim);
	DELAY_TIM->CNT = 0x0000;
}
int i;



void adc_init(void)
{

	//HAL_ADCEx_Calibration_SetValue(LASER_MODULE,ADC_SINGLE_ENDED,HAL_ADCEx_Calibration_GetValue(LASER_MODULE,ADC_SINGLE_ENDED));
	HAL_ADC_Start(LASER_MODULE);
}
