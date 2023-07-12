#include "init.h"
#include "motor.h"


void motor_fwd(motor *mot, __IO uint16_t velo)
{
	setPin(mot->A_PORT,mot->A_PIN);
	clrPin(mot->B_PORT,mot->B_PIN);
	*(mot->vel) = velo;
}

void motor_rev(motor *mot, uint16_t velo)
{
	clrPin(mot->A_PORT,mot->A_PIN);
	setPin(mot->B_PORT,mot->B_PIN);
	*(mot->vel) = velo;
}

void motor_stop(motor *mot)
{
	clrPin(mot->A_PORT,mot->A_PIN);
	clrPin(mot->B_PORT,mot->B_PIN);
}

void motor_brake(motor *mot)
{
	*(mot->vel) = 0xFF;
	setPin(mot->A_PORT,mot->A_PIN);
	setPin(mot->B_PORT,mot->B_PIN);
}

void motor_run(motor *mot,__IO int16_t velo)
{
	if(velo==0)
	{
		*(mot->vel) = 0xFF;
			setPin(mot->A_PORT,mot->A_PIN);
			setPin(mot->B_PORT,mot->B_PIN);
	}
	else if(velo>0)
	{
			*(mot->vel) = velo;
			setPin(mot->A_PORT,mot->A_PIN);
			clrPin(mot->B_PORT,mot->B_PIN);
	} 
	else
	{
		*(mot->vel) = -velo;
			clrPin(mot->A_PORT,mot->A_PIN);
			setPin(mot->B_PORT,mot->B_PIN);
	}	
}

void servo_set_pulse(servo *ser,uint16_t on_time)
{
	if(on_time>(ser->max_pulse))
		*(ser->tim_ch) = (ser->max_pulse);
	else if (on_time<(ser->min_pulse))
		*(ser->tim_ch) = (ser->min_pulse);
	else
		*(ser->tim_ch) = on_time;
}

void servo_set_angle(servo *ser, int angle)
{
	if(angle < ser->min_angle)
		angle = ser->min_angle;
	else if(angle > ser->max_angle)
		angle = ser->max_angle;
	
	ser->angle = angle;
	servo_set_pulse(ser,(((((ser->max_pulse) - (ser->min_pulse))*angle)/180) + ser->min_pulse));
}


//void BLDC_RUN(BLDC *bldc, int speed)
//{
//	if(speed > (bldc->max_pulse))
//     *(bldc->tim_ch) = (bldc->max_pulse);
//	else if (speed < (bldc->min_pulse))
//     *(bldc->tim_ch) = (bldc->min_pulse);
//  else
//     *(bldc->tim_ch) = speed;
//}


