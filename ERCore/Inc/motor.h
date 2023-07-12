#ifndef MOTOR_H_
#define MOTOR_H_

// Includes
#include "stdint.h"
#include "prep.h"


struct motor_t
{
	GPIO_TypeDef *const A_PORT;
	const uint16_t A_PIN;
	GPIO_TypeDef *const B_PORT;
	const uint16_t B_PIN;
	__IO uint32_t *const vel;

};

typedef struct motor_t motor;

struct servo_t
{
	__IO uint32_t *tim_ch;
	uint16_t min_pulse;
	uint16_t max_pulse;
	int angle;
	uint8_t min_angle;
	uint8_t max_angle;
	

};

typedef struct servo_t servo;


//struct BLDC_t
//{
//	__IO uint32_t *tim_ch;
//	uint16_t min_pulse;
//	uint16_t max_pulse;
//	int speed;
//};

//typedef struct BLDC_t BLDC;




typedef enum
{
	MOTOR_BREAK,
	MOTOR_STOP,
	MOTOR_FWD,
	MOTOR_REV,
}motor_status;

// Function Definations
void motor_fwd(motor *mot,__IO uint16_t velo);
void motor_rev(motor *mot,__IO uint16_t velo);
void motor_stop(motor *mot);
void motor_brake(motor *mot);
void motor_run(motor *mot,__IO int16_t velo);

void servo_set_pulse(servo *ser,uint16_t on_time);
void servo_set_angle(servo *ser, int angle);

#endif /* MOTOR_H_ */


