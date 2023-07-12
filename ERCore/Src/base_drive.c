#include "init.h"
#include "base_drive.h"
//#include "imu_bno055.h"

int VELOCITY_MAX_SPEED = 135 ;

uint8_t fast_mode = 0;
float tick = 0;
uint32_t prev_zero_start_ticks = 0;
uint32_t prev_zero_last_ticks = 0;
bool prev_vel_zero = true;
bool y_susp_reached_flag = 0;
bool stick_invert_flag = false;


int16_t position[5] = {0};
int16_t x_centre,y_centre;
int32_t x_integral=0;
int32_t prev_x_error=0;

float kp,ki,kd,max_v,max_w;
void BD_rotate(double ang)    //input in terms of angle //not tested yet
{

		double f_posi=ang*(280.0f/360.0f);
		float err_w=0,prev_err_w=0,w=0,integral_w=0;
		float kpw=10.0f,kdw=50.0f,kiw=0.0f;
		
		max_w=220;//80
	
	while(1)
	{
		receive_ps4();
		if(PS4_SHARE_PRESSED )   
    {
		 robot_velocity_init(250);
    }
		position[1] = read_real_position(1);//right //observed from front wheel
		position[2] = read_real_position(2);//front
		position[3] = read_real_position(3);//left //observed from front wheel side
		position[4] = read_real_position(4);
		
		lcd_print(1,1,read_real_position(1),5);
		lcd_print(2,1,read_real_position(2),5);
		lcd_print(3,1,read_real_position(3),5);
		lcd_print(4,1,read_real_position(4),5);
		
		float curr_w = (position[1]+position[2]+position[3]+position[4])/4;
		
		err_w = f_posi - curr_w;
		w = kpw*err_w + kiw*integral_w + kdw*(err_w-prev_err_w);
		integral_w += err_w;
		
		if(w > max_w)
				{
				 w = max_w;
				}
				else if(w < -max_w)
				{
				 w = -max_w;
				}			
		    if(integral_w >1000)
				{
				 integral_w = 1000;
				}
				else if(integral_w < -1000)
				{
				 integral_w = -1000;
				}
//				lcd_print(4,6,w,4);
//				lcd_print(2,6,ang,4);
//				lcd_print(3,6,err_w,4);
				robot_newvelocity_run(0,0,w,SMOOTH_STOP);

				prev_err_w = err_w;

				
				if((err_w < 2) &&(err_w > -2))
				{
						robot_newvelocity_run(0,0,0,SMOOTH_STOP);
						break;
				}
		
	}
}
/**
  * @brief  Brake Both Base Drive Motor in Manual MUX
  * @param  None
  * @retval None
  */
void robot_brake(void)
{
	mux_manual();
}

void rotate_anti()
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
	
}

void rotate_clk()
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}
void forward()
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);

}
void backward()
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
	
}

void lft()
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}
void rght()
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE,1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE,1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}
void rotate_pid()
{
	
	 float err, prev_error, p, i, d, prev_integ, PID_VAL;
		int angle_ach=90;
		
		float acc = Get_Yaw();
	  lcd_print(1,12,(int)acc,4);
	
	while(1)
	{
	int angle_ach=90;
	float acc = Get_Yaw();
	lcd_print(1,12,(int)acc,4);
	err = angle_ach - acc;
	p = err;
	d = err - prev_error;
	i = i + prev_integ;
	
	
		PID_VAL = (kp*p) + (ki*i)+ (kd*d);
	if (PID_VAL>150)
	{
		PID_VAL=150;
	}
	if (PID_VAL<0)
	{
		PID_VAL=PID_VAL*(-1);
	}
	prev_error = err;
	prev_integ = i;
		
		lcd_print(1,1,(int)acc,4);
		lcd_print(1,2,(int)PID_VAL,4);
if (err>0)
{
	float acc = Get_Yaw();
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_R);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_R);
	lcd_print(1,1,read_real_position(1),5);
	  lcd_print(2,1,read_real_position(2),5);
	  lcd_print(3,1,read_real_position(3),5);
	  lcd_print(4,1,read_real_position(4),5);
	  lcd_print(1,12,(int)acc,4);
	  lcd_gotoxy(2,12);
	  lcd_string("err>0");
		
  StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}
if(err<0)
{
	float acc = Get_Yaw();
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_F);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 1.0f*PID_VAL, 1.0f*50, VEL_V_F);
	lcd_print(1,1,read_real_position(1),5);
	  lcd_print(2,1,read_real_position(2),5);
	  lcd_print(3,1,read_real_position(3),5);
	  lcd_print(4,1,read_real_position(4),5);
	  lcd_print(1,12,(int)acc,4);
	  lcd_gotoxy(2,12);
	  lcd_string("err<0");
		
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}	
}
	}
	
	



void rotate_without_pid()
{
	
	int angle_ach=90;
	float acc = Get_Yaw();
	lcd_print(1,1,(int)acc,4);
	
	if (angle_ach < 90)
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_R);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}

if (angle_ach > 90)
{
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 1.0f*80, 1.0f*50, VEL_V_F);
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
}

if (angle_ach == 90)
	{
		robot_velocity_run(0, 0, 0, ABRUPT_STOP);
	}

	
}



/**
  * @brief  Initialize Robot in Velocity Mode with given Acceleration
  * @param  accel: Acceleration (cannot be changed later) in cm/s^2 (unsigned float)
  * @retval None
  */
 void robot_velocity_init(float accel) //Tested for 4 wheel omni
{
	hardware_reset();//lm629_reset

	
//	initialise_lm629(FRONT_LEFT_MOTOR , VELOCITY_MODE);
//	initialise_lm629(BACK_RIGHT_MOTOR, VELOCITY_MODE);
//	initialise_lm629(BACK_LEFT_MOTOR, VELOCITY_MODE);
//	initialise_lm629(FRONT_RIGHT_MOTOR, VELOCITY_MODE);
	
	
	initialise_lm629(1 , VELOCITY_MODE);
	initialise_lm629(2, VELOCITY_MODE);
	initialise_lm629(3, VELOCITY_MODE);
	initialise_lm629(4, VELOCITY_MODE);
	
//  initialise_lm629 (motor_5,VELOCITY_MODE);
//	initialise_lm629 (motor_6,VELOCITY_MODE);
//	initialise_lm629 (motor_7,VELOCITY_MODE);
//	initialise_lm629 (motor_8,VELOCITY_MODE);
	
	
//	
	load_position_error_stop(1,16000);
	load_position_error_stop(2,16000);
	load_position_error_stop(3,16000);
	load_position_error_stop(4,16000);
//	load_position_error_stop(5,6000);
	//load_position_error_stop(6,6000);
	//load_position_error_stop(7,6000);
	//load_position_error_stop(8,6000);
//////	
	mux_auto();
	
	prev_zero_start_ticks = 0;
	prev_zero_last_ticks = HAL_GetTick();
	prev_vel_zero = true;
	
	Load_trajectory_velocity(FRONT_RIGHT_MOTOR,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
	Load_trajectory_velocity(FRONT_LEFT_MOTOR ,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
	Load_trajectory_velocity(BACK_RIGHT_MOTOR,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
  Load_trajectory_velocity(BACK_LEFT_MOTOR,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
//	Load_trajectory_velocity(motor_5,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
	//Load_trajectory_velocity(motor_6,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
	//Load_trajectory_velocity(motor_7,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
	//Load_trajectory_velocity(motor_8,  ABSOLUTE, 0, 1.0f*accel, VEL_VA_F);
	
 		
	StartMotion(FRONT_RIGHT_MOTOR);
	StartMotion(FRONT_LEFT_MOTOR);
	StartMotion(BACK_RIGHT_MOTOR);
	StartMotion(BACK_LEFT_MOTOR);
//	StartMotion(motor_5);
	//StartMotion(motor_6);
	//StartMotion(motor_7);
  //   StartMotion(motor_8);
	
}


void robot_position_init(float vel, float acc)
{
	hardware_reset();
	
	initialise_lm629(1,POSITION_MODE);
	initialise_lm629(2,POSITION_MODE);
	initialise_lm629(3,POSITION_MODE);
	initialise_lm629(4,POSITION_MODE);
	
	mask_interrupt(1,POSITION_MODE);
	mask_interrupt(2,POSITION_MODE);
	mask_interrupt(3,POSITION_MODE);
	mask_interrupt(4,POSITION_MODE);
	
	interrupt_reset(1);
	interrupt_reset(2);
	interrupt_reset(3);
	interrupt_reset(4);
	
	define_home(1);
	define_home(2);
	define_home(3);
	define_home(4);
	
	Load_trajectory_position(1,ABSOLUTE,0,vel,acc,POS_PVA);
	Load_trajectory_position(2,ABSOLUTE,0,vel,acc,POS_PVA);
	Load_trajectory_position(3,ABSOLUTE,0,vel,acc,POS_PVA);
	Load_trajectory_position(4,ABSOLUTE,0,vel,acc,POS_PVA);
	
	StartMotion(1);
	StartMotion(2);
	StartMotion(3);
	StartMotion(4);
	
}



void rod_curve(float w,float r,stop_type stop_method)
{
	float vf,vl,vr;
	w=toRAD(w);
	
	vf = (w*(4900 + 1225 + r*r + 70*r))/70; 
	vl = w*2*r + 140*w;
	vr = 2*w*r;
	
   	if(vf==0 && vl==0 && vr==0)
	{
		if(stop_method == MOTOR_OFF)
		{
			motor_off(FRONT_MOTOR);
			motor_off(LEFT_MOTOR);
			motor_off(RIGHT_MOTOR);
			robot_brake();
		}
		else if(stop_method == SMOOTH_STOP)
		{
			smooth_stop(FRONT_MOTOR);
			smooth_stop(LEFT_MOTOR);
			smooth_stop(RIGHT_MOTOR);
		}
		else if(stop_method == ABRUPT_STOP)
		{
			abrupt_stop(FRONT_MOTOR);
			abrupt_stop(LEFT_MOTOR);
			abrupt_stop(RIGHT_MOTOR);
		}
		else
		{
			mux_auto();
			Load_trajectory_velocity(FRONT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			Load_trajectory_velocity(LEFT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			Load_trajectory_velocity(RIGHT_MOTOR,ABSOLUTE,0,0,VEL_V_F);

			StartMotion(FRONT_MOTOR);
			StartMotion(LEFT_MOTOR);
			StartMotion(RIGHT_MOTOR);
		}
	}
	else
	{
		mux_auto();
		if(vf<0)
			Load_trajectory_velocity(FRONT_MOTOR,ABSOLUTE,-vf,0,VEL_V_R);
		else
			Load_trajectory_velocity(FRONT_MOTOR,ABSOLUTE,vf,0,VEL_V_F);


		if(vl<0)
			Load_trajectory_velocity(LEFT_MOTOR,ABSOLUTE,-vl,0,VEL_V_R);
		else
			Load_trajectory_velocity(LEFT_MOTOR,ABSOLUTE,vl,0,VEL_V_F);

		if (vr<0)
			Load_trajectory_velocity(RIGHT_MOTOR,ABSOLUTE,-vr,0,VEL_V_F);
		else
			Load_trajectory_velocity(RIGHT_MOTOR,ABSOLUTE,vr,0,VEL_V_R);

		StartMotion(FRONT_MOTOR);
		StartMotion(LEFT_MOTOR);
		StartMotion(RIGHT_MOTOR);
	}
}



void robot_newvelocity_run(float v, float phi,float w ,stop_type stop_method) 	//Tested OK for 4 wheel omni
{
	
	float vfl=0,vfr=0,vbl=0,vbr= 0;
	
	w = toRAD(w);

//	vbl = (v*((-sin(phi*M_PI/180.0f)) + cos(phi*M_PI/180.0f))  +   (ROBOT_R*w));
//	vbr = -(v*((-sin(phi*M_PI/180.0f)) - cos(phi*M_PI/180.0f))  -  (ROBOT_R*w));
//
//	vfr = -(v*((-sin(phi*M_PI/180.0f)) + cos(phi*M_PI/180.0f))  -   (ROBOT_R*w));
//	vfl =  v*((-sin(phi*M_PI/180.0f)) - cos(phi*M_PI/180.0f))  +   (ROBOT_R*w);
//
	vfl = -v - phi + (ROBOT_R*w);              	//FRONT LEFT MOTOR
	vfr = -v + phi + (ROBOT_R*w);								//FRONT RIGHT MOTOR
	vbr = v + phi + (ROBOT_R*w);								//BACK RIGHT MOTOR
	vbl = v - phi + (ROBOT_R*w);								//BACK LEFT MOTOR
//
	
	
		if(vfr==0 && vfl==0 && vbr==0 && vbl==0)     
	{
		if(stop_method == MOTOR_OFF)
		{
			motor_off(FRONT_RIGHT_MOTOR);
			motor_off(FRONT_LEFT_MOTOR);
			motor_off(BACK_RIGHT_MOTOR);
			motor_off(BACK_LEFT_MOTOR);
			robot_brake();
		}
		else if(stop_method == SMOOTH_STOP)
		{
			smooth_stop(FRONT_RIGHT_MOTOR);
			smooth_stop(FRONT_LEFT_MOTOR);
			smooth_stop(BACK_RIGHT_MOTOR);
			smooth_stop(BACK_LEFT_MOTOR);
		}
		else if(stop_method == ABRUPT_STOP)
		{
			abrupt_stop(FRONT_RIGHT_MOTOR);
			abrupt_stop(FRONT_LEFT_MOTOR);
			abrupt_stop(BACK_RIGHT_MOTOR);
			abrupt_stop(BACK_LEFT_MOTOR);
		}
		else
		{
			mux_auto();
			Load_trajectory_velocity(FRONT_LEFT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			Load_trajectory_velocity(FRONT_RIGHT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			
			Load_trajectory_velocity(BACK_LEFT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			Load_trajectory_velocity(BACK_RIGHT_MOTOR,ABSOLUTE,0,0,VEL_V_F);

		
			StartMotion(FRONT_LEFT_MOTOR);
			StartMotion(FRONT_RIGHT_MOTOR);
			
			StartMotion(BACK_LEFT_MOTOR);
			StartMotion(BACK_RIGHT_MOTOR);
		}
	}
	else
	{
		mux_auto();

		if(vfl<0)
			Load_trajectory_velocity(FRONT_LEFT_MOTOR,ABSOLUTE,-vfl,0,VEL_V_R);
		else
			Load_trajectory_velocity(FRONT_LEFT_MOTOR,ABSOLUTE,vfl,0,VEL_V_F);		
		
		if(vfr<0)
			Load_trajectory_velocity(FRONT_RIGHT_MOTOR,ABSOLUTE,-vfr,0,VEL_V_R);
		else
			Load_trajectory_velocity(FRONT_RIGHT_MOTOR,ABSOLUTE,vfr,0,VEL_V_F);

		if (vbl>0)
			Load_trajectory_velocity(BACK_LEFT_MOTOR,ABSOLUTE,vbl,0,VEL_V_F);
		else
			Load_trajectory_velocity(BACK_LEFT_MOTOR,ABSOLUTE,-vbl,0,VEL_V_R);

		
		if (vbr>0)
			Load_trajectory_velocity(BACK_RIGHT_MOTOR,ABSOLUTE,vbr,0,VEL_V_F);
		else
			Load_trajectory_velocity(BACK_RIGHT_MOTOR,ABSOLUTE,-vbr,0,VEL_V_R);
		
		for(int i=1;i<5;i++){
			StartMotion(i);
		}
	}
	/* start_motion_velocity_mode_SDC_UART(1,vf);
	 start_motion_velocity_mode_SDC_UART(4,vl);
	 start_motion_velocity_mode_SDC_UART(3,vr);*/
		
}




void robot_velocity_run(float vx, float vy, float w, stop_type stop_method) //Tested for 4 wheel omni
{
		float vf=0,vl=0,vr=0;
	w = toRAD(w);
	vf = -vx + (ROBOT_R*w);
	vl = (+0.5f*vx) - (sqrtf(3)/2.0f)*vy + (ROBOT_R*w);
	vr = (-0.5f*vx) - (sqrtf(3)/2.0f)*vy - (ROBOT_R*w);
	
	if(vf==0 && vl==0 && vr==0)
	{
		if(stop_method == MOTOR_OFF)
		{
			motor_off(FRONT_MOTOR);
			motor_off(LEFT_MOTOR);
			motor_off(RIGHT_MOTOR);
			robot_brake();
		}
		else if(stop_method == SMOOTH_STOP)
		{
			smooth_stop(FRONT_MOTOR);
			smooth_stop(LEFT_MOTOR);
			smooth_stop(RIGHT_MOTOR);
		}
		else if(stop_method == ABRUPT_STOP)
		{
			abrupt_stop(FRONT_MOTOR);
			abrupt_stop(LEFT_MOTOR);
			abrupt_stop(RIGHT_MOTOR);
		}
		else
		{
			mux_auto();
			Load_trajectory_velocity(FRONT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			Load_trajectory_velocity(LEFT_MOTOR,ABSOLUTE,0,0,VEL_V_F);
			Load_trajectory_velocity(RIGHT_MOTOR,ABSOLUTE,0,0,VEL_V_F);

			StartMotion(FRONT_MOTOR);
			StartMotion(LEFT_MOTOR);
			StartMotion(RIGHT_MOTOR);
		}
	}
	else
	{
		mux_auto();
		if(vf<0)
			Load_trajectory_velocity(FRONT_MOTOR,ABSOLUTE,-vf,0,VEL_V_R);
		else
			Load_trajectory_velocity(FRONT_MOTOR,ABSOLUTE,vf,0,VEL_V_F);


		if(vl<0)
			Load_trajectory_velocity(LEFT_MOTOR,ABSOLUTE,-vl,0,VEL_V_R);
		else
			Load_trajectory_velocity(LEFT_MOTOR,ABSOLUTE,vl,0,VEL_V_F);

		if (vr<0)
			Load_trajectory_velocity(RIGHT_MOTOR,ABSOLUTE,-vr,0,VEL_V_F);
		else
			Load_trajectory_velocity(RIGHT_MOTOR,ABSOLUTE,vr,0,VEL_V_R);

		StartMotion(FRONT_MOTOR);
		StartMotion(LEFT_MOTOR);
		StartMotion(RIGHT_MOTOR);
	}

}





void robot_joystick_velocity(void) //Tested for 4 wheel omni         
{
	float r=0;
	int theta=0;
	int lxx,lyy,rxx;
	
	bool current_vel_zero = true;
		
	uint32_t change_ticks = 0;
	
	const float vel_step[3] = {60,120,180};  
	uint32_t  step_time[3] = {400,800,1200};              
	
	if(pairing_flag == true)
	{	
			lyy = get_stick(ly);  
			lxx = get_stick(lx);   
			rxx = get_stick(rx);      
	}
	else
	{
		lyy = 0;
		lxx = 0;
		rxx = 0;
	}	
	
  if(stick_invert_flag == true)
	{		
		rxx = (rxx /127.0f) * VELOCITY_MAX_SPEED*0.75;
	  lyy = -(lyy /127.0f) * VELOCITY_MAX_SPEED;
		lxx = -(lxx /127.0f) * VELOCITY_MAX_SPEED;
	}
	else
	{
		//rxx = (rxx /127.0f) * VELOCITY_MAX_SPEED*0.75;
		rxx = (rxx /127.0f) * VELOCITY_MAX_SPEED;
		lyy = (lyy /127.0f) * VELOCITY_MAX_SPEED;
		lxx = (lxx /127.0f) * VELOCITY_MAX_SPEED;
	}
	/*lcd_i2c_print(1,1,lxx,4);
	lcd_i2c_print(1,6,lyy,4);
	lcd_i2c_print(2,1,rxx,4);*/

	r = sqrtf((float)(lxx*lxx + lyy*lyy));
	theta = (int) toDEG(atan2f(lyy,lxx));
	if(theta<0)
		theta = theta + 360;                             

	current_vel_zero = isVelZero(lxx,lyy,rxx);         
	if(current_vel_zero == true)
	{
		prev_zero_last_ticks = HAL_GetTick();
		if(prev_vel_zero == false)                                      //used in somether func to change value
			prev_zero_start_ticks = HAL_GetTick();
	}
	change_ticks = HAL_GetTick() - prev_zero_last_ticks;              //change ticks approx 0 if cuurent_vel_zero = true
	                                                            //if current_vel_zero = false, change_ticks = HAL_GetTick
	//30-60-90
	if(VELOCITY_MAX_SPEED >= 	180)                                              //also called in somether function  
	{
		switch ((int)(theta/11.25))
		{
			case 0:			//0-11
				theta = 0;
				break;

			case 1:			//11-22
				theta = 0;
				break;

			case 2:			//22-33.37
				theta = 45;
				break;
			
			case 3:			//33-45
				theta = 45;
				break;

			case 4:			//45-56.25
				theta = 45;
				break;

			case 5:			//56-67
				theta = 45;
				break;
			
			case 6:			//67.5-78.75
				theta = 90;
				break;

			case 7:			//78-90
				theta = 90;
				break;

			case 8:			//90-101
				theta = 90;
				break;
			
			case 9:			//101-112
				theta = 90;
				break;

			case 10:			//112-123
				theta = 135;
				break;
			
			case 11:			//123-135
				theta = 135;
				break;

			case 12:			//135-146
				theta = 135;
				break;

			case 13:			//146-157
				theta = 135;
				break;

			case 14:		//157-168
				theta = 180;
				break;

			case 15:		//168-180
				theta = 180;
				break;

			case 16:		//180-191.25
				theta = 180; 
				break;

			case 17:		//191.25-202
				theta = 180;
				break;

			case 18:		//202-213
				theta = 225;
				break;

			case 19:		//213-225
				theta = 225;
				break;
			
			case 20:		//225-236
				theta = 225;
				break;

			case 21:		//236-247
				theta = 225; 
				break;

			case 22:		//247-258
				theta = 270;
				break;

			case 23:		//258-270
				theta = 270;
				break;

			case 24:		//270-281
				theta = 270;
				break;
			
			case 25:		//281-292
				theta = 270;
				break;

			case 26:		//292-303
				theta = 315; 
				break;
			
			case 27:		//303-315
				theta = 315; 
				break;

			case 28:		//315-326
				theta = 315;
				break;

			case 29:		//326-337
				theta = 315;
				break;

			case 30:		//337-348
				theta = 0;
				break;
			
			case 31:		//348-360
				theta = 0;
				break;
			
		}
	}
	else
	{
		switch ((int)(theta/11.25))
		{
			case 0:			//0-11
				theta = 0;
				break;

			case 1:			//11-22
				theta = 22;
				break;

			case 2:			//22-33.37
				theta = 22;
				break;
			
			case 3:			//33-45
				theta = 45;
				break;

			case 4:			//45-56.25
				theta = 45;
				break;

			case 5:			//56-67
				theta = 67;
				break;
			
			case 6:			//67.5-78.75
				theta = 67;
				break;

			case 7:			//78-90
				theta = 90;
				break;

			case 8:			//90-101
				theta = 90;
				break;
			
			case 9:			//101-112
				theta = 112;
				break;

			case 10:			//112-123
				theta = 112;
				break;
			
			case 11:			//123-135
				theta = 135;
				break;

			case 12:			//135-146
				theta = 135;
				break;

			case 13:			//146-157
				theta = 157;
				break;

			case 14:		//157-168
				theta = 157;
				break;

			case 15:		//168-180
				theta = 180;
				break;

			case 16:		//180-191.25
				theta = 180; 
				break;

			case 17:		//191.25-202
				theta = 202;
				break;

			case 18:		//202-213
				theta = 202;
				break;

			case 19:		//213-225
				theta = 225;
				break;
			
			case 20:		//225-236
				theta = 225;
				break;

			case 21:		//236-247
				theta = 247; 
				break;

			case 22:		//247-258
				theta = 247;
				break;

			case 23:		//258-270
				theta = 270;
				break;

			case 24:		//270-281
				theta = 270;
				break;
			
			case 25:		//281-292
				theta = 292;
				break;

			case 26:		//292-303
				theta = 292; 
				break;
			
			case 27:		//303-315
				theta = 315; 
				break;

			case 28:		//315-326
				theta = 315;
				break;

			case 29:		//326-337
				theta = 337;
				break;

			case 30:		//337-348
				theta = 337;
				break;
			
			case 31:		//348-360
				theta = 0;
				break;
		}
	}
	//lcd_i2c_print(4,1,theta,4); 
	lxx = r*cosf(toRAD(theta));      //r = r/2 in fast mode
	lyy = r*sinf(toRAD(theta));
	
	if(VELOCITY_MAX_SPEED >= 	200)
	{
		if(lxx >= 0)
		{
			if((lxx >= vel_step[0]) && (change_ticks <= step_time[2]))          
			{
				if((lxx >= vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lxx >= vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lxx = vel_step[2];
					}
					else
					{
						lxx = vel_step[1];
					}
				}
				else
				{
					lxx = vel_step[0];
				}
			}
			else
			{
				lxx = lxx;
			}
		}
		else
		{
			if((lxx <= -vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lxx <= -vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lxx <= -vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lxx = -vel_step[2];
					}
					else
					{
						lxx = -vel_step[1];
					}
				}
				else
				{
					lxx = -vel_step[0];
				}
			}
			else
			{
				lxx = lxx;
			}
		}
				
		
		if(lyy >= 0)
		{	
			if((lyy >= vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lyy >= vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lyy >= vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lyy = vel_step[2];
					}
					else
					{
						lyy = vel_step[1];
					}
				}
				else
				{
					lyy = vel_step[0];
				}
			}
			else
			{
				lyy = lyy;
			}
		}
		else
		{
			if((lyy <= -vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lyy <= -vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lyy <= -vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lyy = -vel_step[2];
					}
					else
					{
						lyy = -vel_step[1];
					}
				}
				else
				{
					lyy = -vel_step[0];
				}
			}
			else
			{
				lyy = lyy;
			}
		}
	}
	
	prev_vel_zero = current_vel_zero;

	if(current_vel_zero == true)
	{
		if((HAL_GetTick() - prev_zero_start_ticks) > 5000)
		{
			motor_off(FRONT_LEFT_MOTOR);
			motor_off(FRONT_RIGHT_MOTOR);			
			motor_off(BACK_LEFT_MOTOR);
			motor_off(BACK_RIGHT_MOTOR);
			return;
		}
	}
	
  //robot_velocity_run(lxx,lyy,rxx,SMOOTH_STOP); for 3 wheel bd
	//robot_newvelocity_run	(r,theta,rxx,SMOOTH_STOP); //for 4 wheel bd             /////Omni
	robot_newvelocity_run((float)lxx,(float)lyy,rxx,SMOOTH_STOP);   //0.5,0.15f //////Mecanum
}



void robot_joystick_velocity_orientation(void) //Tested for 4 wheel omni         
{
	float r=0;
	int theta=0;
	int lxx,lyy,rxx;
	
	bool current_vel_zero = true;
		
	uint32_t change_ticks = 0;
	
	const float vel_step[3] = {60,120,180};  
	uint32_t  step_time[3] = {400,800,1200};              
	
	if(pairing_flag == true)
	{	
			lyy = get_stick(ly);  
			lxx = get_stick(lx);   
			rxx = get_stick(rx);      
	}
	else
	{
		lyy = 0;
		lxx = 0;
		rxx = 0;
	}	
	
  if(stick_invert_flag == true)
	{		
		rxx = (rxx /127.0f) * VELOCITY_MAX_SPEED;
	  lyy = -(lyy /127.0f) * VELOCITY_MAX_SPEED;
		lxx = -(lxx /127.0f) * VELOCITY_MAX_SPEED;
	}
	else
	{
		rxx = (rxx /127.0f) * VELOCITY_MAX_SPEED;
		lyy = (lyy /127.0f) * VELOCITY_MAX_SPEED;
		lxx = (lxx /127.0f) * VELOCITY_MAX_SPEED;
	}
	

	r = sqrtf((float)(lxx*lxx + lyy*lyy));
	theta = (int) toDEG(atan2f(lyy,lxx));
	if(theta<0)
		theta = theta + 360;                             

	current_vel_zero = isVelZero(lxx,lyy,rxx);         
	if(current_vel_zero == true)
	{
		prev_zero_last_ticks = HAL_GetTick();
		if(prev_vel_zero == false)                                      //used in somether func to change value
			prev_zero_start_ticks = HAL_GetTick();
	}
	change_ticks = HAL_GetTick() - prev_zero_last_ticks;              //change ticks approx 0 if cuurent_vel_zero = true
	                                                            //if current_vel_zero = false, change_ticks = HAL_GetTick
	//30-60-90
	if(VELOCITY_MAX_SPEED >= 	200)                                              //also called in somether function  
	{
		switch ((int)(theta/22.5))
		{
			case 0:			//0-15
				theta = 0;
				break;

			case 1:			//15-30
				theta = 45;
				break;

			case 2:			//30-45
				theta = 45;
				break;

			case 3:			//45-60
				theta = 90;
				break;

			case 4:			//60-75
				theta = 90;
				break;

			case 5:			//75-90
				theta = 90;
				break;

			case 6:			//90-105
				theta = 90;
				break;

			case 7:			//105-120
				theta = 135;
				break;

			case 8:			//120-135180
				theta = 135;
				break;

			case 9:			//135-150
				theta = 135;
				break;

			case 10:		//150-165
				theta = 180;
				break;

			case 11:		//165-180
				theta = 180;
				break;

			case 12:		//180-195
				theta = 180; 
				break;

			case 13:		//195-210
				theta = 225;
				break;

			case 14:		//210-225
				theta = 225;
				break;

			case 15:		//225-240
				theta = 225;
				break;

			case 16:		//240-255
				theta = 270;
				break;

			case 17:		//255-270
				theta = 270;
				break;

			case 18:		//270-285
				theta = 270;
				break;

			case 19:		//285-300
				theta = 315;
				break;

			case 20:		//300-315
				theta = 315;
				break;

			case 21:		//315-330
				theta = 315;
				break;

			case 22:		//330-345
				theta = 0;
				break;

			case 23:		//345-360
				theta = 0;
				break;
		}
	}
	else
	{
		switch ((int)(theta/15))
		{
			case 0:			//0-15
				theta = 0;
				break;

			case 1:			//15-30
				theta = 45;
				break;

			case 2:			//30-45
				theta = 45;
				break;

			case 3:			//45-60
				theta = 90;
				break;

			case 4:			//60-75
				theta = 90;
				break;

			case 5:			//75-90
				theta = 90;
				break;

			case 6:			//90-105
				theta = 90;
				break;

			case 7:			//105-120
				theta = 135;
				break;

			case 8:			//120-135180
				theta = 135;
				break;

			case 9:			//135-150
				theta = 135;
				break;

			case 10:		//150-165
				theta = 180;
				break;

			case 11:		//165-180
				theta = 180;
				break;

			case 12:		//180-195
				theta = 180; 
				break;

			case 13:		//195-210
				theta = 225;
				break;

			case 14:		//210-225
				theta = 225;
				break;

			case 15:		//225-240
				theta = 225;
				break;

			case 16:		//240-255
				theta = 270;
				break;

			case 17:		//255-270
				theta = 270;
				break;

			case 18:		//270-285
				theta = 270;
				break;

			case 19:		//285-300
				theta = 315;
				break;

			case 20:		//300-315
				theta = 315;
				break;

			case 21:		//315-330
				theta = 315;
				break;

			case 22:		//330-345
				theta = 0;
				break;

			case 23:		//345-360
				theta = 0;
				break;
		}
	}
		 
	lxx = r*cosf(toRAD(theta));      //r = r/2 in fast mode
	lyy = r*sinf(toRAD(theta));
	
	if(VELOCITY_MAX_SPEED >= 	200)
	{
		if(lxx >= 0)
		{
			if((lxx >= vel_step[0]) && (change_ticks <= step_time[2]))          
			{
				if((lxx >= vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lxx >= vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lxx = vel_step[2];
					}
					else
					{
						lxx = vel_step[1];
					}
				}
				else
				{
					lxx = vel_step[0];
				}
			}
			else
			{
				lxx = lxx;
			}
		}
		else
		{
			if((lxx <= -vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lxx <= -vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lxx <= -vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lxx = -vel_step[2];
					}
					else
					{
						lxx = -vel_step[1];
					}
				}
				else
				{
					lxx = -vel_step[0];
				}
			}
			else
			{
				lxx = lxx;
			}
		}
				
		
		if(lyy >= 0)
		{	
			if((lyy >= vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lyy >= vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lyy >= vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lyy = vel_step[2];
					}
					else
					{
						lyy = vel_step[1];
					}
				}
				else
				{
					lyy = vel_step[0];
				}
			}
			else
			{
				lyy = lyy;
			}
		}
		else
		{
			if((lyy <= -vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lyy <= -vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lyy <= -vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lyy = -vel_step[2];
					}
					else
					{
						lyy = -vel_step[1];
					}
				}
				else
				{
					lyy = -vel_step[0];
				}
			}
			else
			{
				lyy = lyy;
			}
		}
	}
	
	prev_vel_zero = current_vel_zero;

	if(current_vel_zero == true)
	{
		if((HAL_GetTick() - prev_zero_start_ticks) > 5000)
		{
			motor_off(FRONT_RIGHT_MOTOR);
			motor_off(BACK_RIGHT_MOTOR);
			motor_off(FRONT_LEFT_MOTOR);
			motor_off(BACK_LEFT_MOTOR);
			return;
		}
	}
	
	yaw = toRAD(yaw);
	robot_velocity_run(lxx*cos(yaw) + lyy*sin(-yaw),lxx*sin(yaw) + lyy*cos(-yaw), rxx, SMOOTH_STOP);
	
	
}



void robot_joystick_velocity_imu(void)          
{
	float r=0;
	int theta=0;
	int lxx,lyy,rxx;
		
	static float err_w,prev_err_w,w,kpw = 4.00f,kdw=0.6f,kiw=0.0f,integral_w,phi=0;

	bool current_vel_zero = true;
		
	uint32_t change_ticks = 0;
	
	const float vel_step[3] = {30,60,90};                
	uint32_t step_time[3] = {400,800,1200};                
	
	
	if(pairing_flag == true)
	{	
			lyy = get_stick(ly);  
			lxx = get_stick(lx);   
			rxx = get_stick(rx);      
	}
	else
	{
		lyy = 0;
		lxx = 0;
		rxx = 0;
	}

	rxx = (rxx /127.0f) * VELOCITY_MAX_SPEED;
	lyy = (lyy /127.0f) * VELOCITY_MAX_SPEED;
	lxx = (lxx /127.0f) * VELOCITY_MAX_SPEED;
	
	/*lcd_i2c_print(1,1,lxx,4);
	lcd_i2c_print(1,6,lyy,4);
	lcd_i2c_print(2,1,rxx,4);*/

	r = sqrtf((float)(lxx*lxx + lyy*lyy));
	theta = (int) toDEG(atan2f(lyy,lxx));
	if(theta<0)
		theta = theta + 360;    
//====================================================================================================
	
	//gyro_yaw(gyro_data);
	dt = HAL_GetTick() - tick;
	//imu_get_data(&dt,&yaw,0);
	if( rxx == 0)
	{
		err_w = phi - yaw ; //gyro_data[0]; 
	}
	else 
	{
		phi = yaw; //gyro_data[0]; 
		err_w = 0;
	}
	
	w = kpw*err_w + kdw*(err_w - prev_err_w) + kiw*integral_w;
	 if(w > 45)
			 w = 45;
	 else if(w < -45)
			 w = -45;
	 
	 prev_err_w = err_w;
	 
	 lcd_print(3,1,yaw /*gyro_data[0]*/ ,4);
	 /*lcd_i2c_print(3,6,phi,4);
	 lcd_i2c_print(4,1,err_w,4);
	 lcd_i2c_print(4,6,w,4);
	 */
	//=======================================================================================================       
	
/*
	current_vel_zero = isVelZero(lxx,lyy,rxx);         
	if(current_vel_zero == true)
	{
		prev_zero_last_ticks = HAL_GetTick();
		if(prev_vel_zero == false)                                      //used in somether func to change value
			prev_zero_start_ticks = HAL_GetTick();
	}
	change_ticks = HAL_GetTick() - prev_zero_last_ticks;              //change ticks approx 0 if cuurent_vel_zero = true
	*/                                                            //if current_vel_zero = false, change_ticks = HAL_GetTick
	
	//30-60-90
	if(fast_mode == true)                                              //also called in somether function  
	{
		switch ((int)(theta/22.5))
		{
			case 0:			//0-22.5
				theta = 0;
				break;

			case 1:			//22.5-45
				theta = 45;
				//r = r/2;
				break;

			case 2:			//45-67.5
				theta = 45;
				//r = r/2;
				break;

			case 3:			//67.5-90
				theta = 90;
				break;

			case 4:			//90-112.5
				theta = 90;
				break;

			case 5:			//112.5-135
				theta = 135;
				//r = r/2;
				break;

			case 6:			//135-157.5
				theta = 135;
				//r = r/2;
				break;

			case 7:			//157.5-180
				theta = 180;
				break;

			case 8:			//180-202.5
				theta = 180;
				break;

			case 9:			//202.5-225
				theta = 225;
			//	r = r/2;
				break;

			case 10:		//225-247.5
				theta = 225;
				//r = r/2;
				break;

			case 11:		//247.5-270
				theta = 270;
				break;

			case 12:		//270-292.5
				theta = 270;
				break;

			case 13:		//292.5-315
				theta = 315;
				//r = r/2;
				break;

			case 14:		//315-337.5
				theta = 315;
				//r = r/2;
				break;

			case 15:		//337.5-360
				theta = 0;
				break;
		}
	}
	else
	{
		switch ((int)(theta/15))
		{
			case 0:			//0-15
				theta = 0;
				break;

			case 1:			//15-30
				theta = 30;
				break;

			case 2:			//30-45
				theta = 30;
				break;

			case 3:			//45-60
				theta = 60;
				break;

			case 4:			//60-75
				theta = 60;
				break;

			case 5:			//75-90
				theta = 90;
				break;

			case 6:			//90-105
				theta = 90;
				break;

			case 7:			//105-120
				theta = 120;
				break;

			case 8:			//120-135
				theta = 120;
				break;

			case 9:			//135-150
				theta = 150;
				break;

			case 10:		//150-165
				theta = 150;
				break;

			case 11:		//165-180
				theta = 180;
				break;

			case 12:		//180-195
				theta = 180; 
				break;

			case 13:		//195-210
				theta = 210;
				break;

			case 14:		//210-225
				theta = 210;
				break;

			case 15:		//225-240
				theta = 240;
				break;

			case 16:		//240-255
				theta = 240;
				break;

			case 17:		//255-270
				theta = 270;
				break;

			case 18:		//270-285
				theta = 270;
				break;

			case 19:		//285-300
				theta = 300;
				break;

			case 20:		//300-315
				theta = 300;
				break;

			case 21:		//315-330
				theta = 330;
				break;

			case 22:		//330-345
				theta = 330;
				break;

			case 23:		//345-360
				theta = 0;
				break;
		}
	}
	
		 
	lxx = r*cosf(toRAD(theta));      //r = r/2 in fast mode
	lyy = r*sinf(toRAD(theta));
	
	/*
	if(fast_mode == true)
	{
		if(lxx >= 0)
		{
			
			if((lxx >= vel_step[0]) && (change_ticks <= step_time[2]))           //what the pupose of step_time
			{
				if((lxx >= vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lxx >= vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lxx = vel_step[2];
					}
					else
					{
						lxx = vel_step[1];
					}
				}
				else
				{
					lxx = vel_step[0];
				}
			}
			else
			{
				lxx = lxx;
			}
		}
		else
		{
			if((lxx <= -vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lxx <= -vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lxx <= -vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lxx = -vel_step[2];
					}
					else
					{
						lxx = -vel_step[1];
					}
				}
				else
				{
					lxx = -vel_step[0];
				}
			}
			else
			{
				lxx = lxx;
			}
		}
				
		
		if(lyy >= 0)
		{	
			if((lyy >= vel_step[0]) && (change_ticks <= step_time[2]))
 			{
				if((lyy >= vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lyy >= vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lyy = vel_step[2];
					}
					else
					{
						lyy = vel_step[1];
					}
				}
				else
				{
					lyy = vel_step[0];
				}
			}
			else
			{
				lyy = lyy;
			}
		}
		else
		{
			if((lyy <= -vel_step[0]) && (change_ticks <= step_time[2]))
			{
				if((lyy <= -vel_step[1]) && (change_ticks >= step_time[0]))
				{
					if((lyy <= -vel_step[2]) && (change_ticks >= step_time[1]))
					{
						lyy = -vel_step[2];
					}
					else
					{
						lyy = -vel_step[1];
					}
				}
				else
				{
					lyy = -vel_step[0];
				}
			}
			else
			{
				lyy = lyy;
			}
		}
	}
	*/
	/*prev_vel_zero = current_vel_zero;

	if(current_vel_zero == true)
	{
		if((HAL_GetTick() - prev_zero_start_ticks) > 5000)
		{
			motor_off(FRONT_MOTOR);
			motor_off(RIGHT_MOTOR);
			motor_off(LEFT_MOTOR);
			return;
		}
	}*/
	
		/*if(fast_mode == false)
			robot_newvelocity_run((float)r,(float)theta,0.35f*rxx,SMOOTH_STOP);
		else
			robot_newvelocity_run((float)r,(float)theta,0.45f*rxx,SMOOTH_STOP); */  

	 if( rxx != 0)
	{
		if(fast_mode == 0)             																							//slow
			robot_velocity_run(lxx,lyy,0.3f*rxx,SMOOTH_STOP);
		else if(fast_mode == 1)																											//fast
			robot_velocity_run(lxx,lyy,0.3f*rxx,SMOOTH_STOP);
		else 																			                        					//slow in x
			robot_velocity_run(0.4f*lxx,lyy,0.1f*rxx,SMOOTH_STOP); 
		
	}
	else
	{		
		if(fast_mode == 0)             																							//slow
			robot_velocity_run(lxx,lyy,w,SMOOTH_STOP);
		else if(fast_mode == 1)																											//fast
			robot_velocity_run(lxx,lyy,w,SMOOTH_STOP);
		else                      																									//slow in x
			robot_velocity_run(0.4f*lxx,lyy,w,SMOOTH_STOP); 
	}
		
}


void robot_gotox(float x_pos)	
{
  float x_temp=0,y_temp=0;
	float roll,yaw,pitch,vel_x;
	float kpx=1.0,kix=0,kdx=0;
	float err_x,prev_x,integral_x;
  while(1)
			 {			
				 if(PS4_SQUARE_PRESSED)
				 {
					 lcd_string("em stop");
					 _delay_ms(500);
				   emergency_stop();
					 break;
				 }
				//get_kick_motor_counts(&x_temp);
        err_x = x_pos - x_temp;
        lcd_print(1,1,err_x,4);				 
				vel_x=kpx*err_x + kix*integral_x + kdx*(err_x-prev_x);
				 
				if(vel_x > 150)
				{
				vel_x=150;
				}
				if(vel_x < -150)
				{
				vel_x=-150;
				}
				
				if(integral_x >5000)
				{
				integral_x=5000;
				}
				if(integral_x < -5000)
				{
				integral_x=-5000;
				}
				
				robot_velocity_run(-50,0,0,SMOOTH_STOP); 
				
				if(err_x < 2 && err_x >-2)
				{
				vel_x=0;
				robot_velocity_run(0,0,0,SMOOTH_STOP);
        break;					
				}
				 	SAVE_CONSOLE;		  
			 }
}

void robot_gotoy(float y_pos)	
{
		 static float x_temp=0,y_temp=0;
		 static float yaw=0,vel=0;
	   float kpy=3.5f,kiy=0.0f,kdy=0.0f;
		 static float err_y = 0, prev_y = 0,integral_y=0;
		// get_y_encoder_position(&y_temp);
     err_y = y_pos - y_temp;
     //lcd_i2c_print(4,1,err_y,4);				 
	   vel=kpy*err_y + kiy*integral_y + kdy*(err_y-prev_y);
				 
			if(vel > 150)
			{
				vel=150;
			}
			if(vel < -150)
			{
				vel=-150;
			}

			if(integral_y >500)
			{
				integral_y=500;
			}
			if(integral_y < -500)
			{
				integral_y=-500;
			}
				
			robot_newvelocity_run(vel,90,0,SMOOTH_STOP); 
			if(err_y < 2 && err_y > -2)
			{
					vel=0;
					robot_newvelocity_run(0,0,0,SMOOTH_STOP);				
					y_susp_reached_flag = 1;
			}
				 			  
}

void robot_gotoxtheta(float x_pos, float theta)	
{
  float x_temp=0,y_temp=0, x_res = 0, w = 0.6f;
	float roll,yaw,pitch,vel_x;
	float kpx=1.0,kix=0,kdx=0;
	
	float err_x,prev_x,integral_x;
  while(1)
			 {	
				if(PS4_SQUARE_PRESSED)
				{
				 emergency_stop();
				 break;
				}
        SAVE_CONSOLE;				
				//get_kick_motor_counts(&x_temp);
				//get_y_encoder_position(&y_temp);
				//imu_get_data(&roll,&yaw,&pitch);
				HAL_Delay(10);
				if(yaw < 90)
				{	
					x_res = x_temp - y_temp;
				}
				else if(yaw > 90)
				{
					x_res = -x_temp - y_temp;
				}
				x_res = x_temp*cos(yaw*M_PI/180) - y_temp*sin(yaw*M_PI/180);
        err_x = x_pos - x_res;
        lcd_print(1,1,err_x,4);				 
				lcd_print(2,1,x_temp,4);	
				vel_x = kpx*err_x + kix*integral_x + kdx*(err_x-prev_x);
				lcd_print(3,1,y_temp,4);	
				if(vel_x > 50)
				{
				  vel_x=50;
				}
				if(vel_x < -50)
				{
				  vel_x=-50;
				}
				
				if(integral_x >500)
				{
				 integral_x=500;
				}
				if(integral_x < -500)
				{
				 integral_x=-500;
				}
				
				robot_newvelocity_run(vel_x,-yaw*1.32 - 7,w,SMOOTH_STOP); 
				lcd_print(1,6,yaw,4);
				
				if(err_x < 2 )
				{
					vel_x=0;
					robot_newvelocity_run(0,0,0,SMOOTH_STOP);
					break;					
				}
				 			  
			 }
			 robot_newvelocity_run(0,0,0,SMOOTH_STOP);
	}

void robot_new_rotate(float theta)
{
	  static float w ,kp=5.0f,kd=8.0f,ki=0,err,prev_err=0,integral=0;
	  static float yaw=0;
		int i=0;
	float avg = 0, ang = 0;
	  
		while(1)
		{
			position[1] = read_real_position(1);
			position[2] = read_real_position(2);
			position[3] = read_real_position(3);
			position[4] = read_real_position(4);
				
			for(i = 1 ; i <= 4; i++)
			{
				avg += position[i];
			}
			
			avg = avg/4;
			ang = toRAD(theta);
		
	  err = (ROBOT_R*ang*1.4142) - avg;
	  if((err <= 2) && (err >= -2))
	  {
			w=0;
			err=0;
			prev_err=0;
		  robot_newvelocity_run(0,0,0,SMOOTH_STOP);
		  break;
	  }
	  w = kp*err + ki*integral + kd*(err - prev_err) ;
	  prev_err = err;
		 if(w > 50)
			 w=50;
		 else if(w < -50)
			 w=-50;
	  robot_newvelocity_run(0,0,w,SMOOTH_STOP);
		}
		}

		
		

void robot_rotate(float theta, float max_w) 	//Tested for 4 wheel omni
{
	  static float w, kp=4.5f, kd=1.0f, ki=0, err,prev_err = 0, integral = 0;
		int i=0;
		float avg = 0, angle = 0;
	  bot_define_home();
		while(1)
		{
			
			position[1] = read_real_position(1);
			position[2] = read_real_position(2);
			position[3] = read_real_position(3);
			position[4] = read_real_position(4);
				
			for(i = 1 ; i <= 4; i++)
			{
				avg += position[i];
			}
			
			avg = avg/4;
			
			
			angle = (theta*280)/360;
			
			lcd_gotoxy(1,6);
			lcd_string("Angle");
			lcd_print(1,1,angle,4);
			lcd_gotoxy(2,6);
			lcd_string("Theta");
			lcd_print(2,1,theta,4);
			err = angle - avg;
			if((err <= 2) && (err >= -2))
			{
				w=0;
				err=0;
				prev_err=0;
				robot_newvelocity_run(0,0,0,SMOOTH_STOP);
				break;
			}
			w = kp*err + ki*integral + kd*(err - prev_err) ;
			prev_err = err;
			 if(w > max_w)
				 w = max_w;
			 else if(w < -max_w)
				 w = -max_w;
			robot_newvelocity_run(0,0,w,SMOOTH_STOP);
		}
}		
		

	
void robot_gotoytheta(float y_pos, float theta)	
{
   float x_temp=0, y_temp=0, y_res = 0,x_res=0, w = 30.0f,dy=0,dx=0,prev_x_temp=0,prev_y_temp;
	 float yaw,vel_y;
	 float kpy=2.0f,kiy=0,kdy=0;
	 float err_y,prev_y,integral_y;
	
   while(1)
			 {
				// get_y_encoder_position(&y_temp);
				 //get_kick_motor_counts(&x_temp);
				 dy = y_temp - prev_y_temp;
				 dx = x_temp - prev_x_temp;
				 y_res += dy*cos(yaw*M_PI/180) + dx*sin(yaw*M_PI/180);
				 x_res += dx*cos(yaw*M_PI/180) - dy*sin(yaw*M_PI/180);
				 err_y = y_pos - y_res;
	       lcd_print(1,1,err_y,4);				 
				 lcd_print(1,6,yaw,4);
				 vel_y=kpy*err_y + kiy*integral_y + kdy*(err_y-prev_y);
				 lcd_print(1,12,y_res,4);				  
				 if(vel_y > 180)
				 {
					 vel_y=180;
			   }
				 if(vel_y < -180)
				 {
					 vel_y=-180;
				 }
			
				/*if(integral_y >500)
				{
				integral_y = 500;
				}
				if(integral_y < -500)
				{
				integral_y = -500;
				}*/
				
				 robot_newvelocity_run(vel_y,90 - yaw + 5, w ,SMOOTH_STOP); 	
				 prev_y_temp = y_temp;
				 prev_x_temp = x_temp;
	 	
			   if(err_y < 2 && err_y > -2)
				 {
					 vel_y=0;
					 robot_newvelocity_run(0,0,0,SMOOTH_STOP);
					 break;					
				 }			
			 } 		 		 
}


void robot_gotoxytheta(float x_pos,float y_pos,float turn_ang)
{
		 static float curr_x=0,curr_y=0,y_res =0,x_res=0,dy=0,dx=0,prev_curr_x=0,prev_curr_y=0;
		 static float yaw=0,vel_xy = 0,integral = 0,phi=0;
		 float kp = 1.0f,kd=0.0f,ki =0.0f ,w = 40;
		 static float err=0,err_x=0,err_y = 0,prev_err;
	
		 //get_kick_motor_counts(&curr_x);
		 //get_y_encoder_position(&curr_y);
		 dy = curr_y - prev_curr_y;
		 dx = curr_x - prev_curr_x;
			
		 dt = HAL_GetTick() - tick;
		 //imu_get_data(&dt,&yaw,0);
		 lcd_print(3,1,yaw,4);
		
		 y_res += dy*cos(yaw*M_PI/180) + dx*sin(yaw*M_PI/180);
		 x_res += dx*cos(yaw*M_PI/180) - dy*sin(yaw*M_PI/180);
	
		 err_y = y_pos - y_res;
		 err_x = x_pos - x_res;
		 err = sqrtf((err_x*err_x) + (err_y*err_y));
		 integral+=err;
	
		 if(integral  > 1000)
			 integral=1000;
		 else if(integral < -1000)
			 integral = -1000;
		 
		 vel_xy = kp*err + ki*integral + kd*(err - prev_err);
		 if(vel_xy > 100)
			 vel_xy = 100;
		 else if(vel_xy < -1000)
				vel_xy = -100;
		 
		 phi = atan2(err_y,err_x);
		 phi = toDEG(phi);
		 
		 robot_newvelocity_run(vel_xy,phi - yaw ,w,SMOOTH_STOP);
		 prev_err = err;
		 prev_curr_x = curr_x;
		 prev_curr_y = curr_y;
		 if((err < 2) && (err > -2))
		 {
				robot_newvelocity_run(0,0,0,SMOOTH_STOP);
				//xxxx_flag = true;
		 }
}









 	
void robot_gotoxy_pid_with_vel(float x_pos,float y_pos, float max_v)		//Tested for 4 wheel omni
{
	float err_w=0,prev_err_w=0,w=0,kpw=7.3f,kdw=2.0f,kiw=0.0f,integral_w=0;
	double curr_x,curr_y,integral,dy,dx,prev_curr_x=0,prev_curr_y=0,x_res=0,y_res=0;
  float prev_err,err,vel,phi_angle;
	float real_x = 0, real_y = 0,x1,x2,y1,y2;
	
	kp = 15.0f;
	kd = 16.0f;
	ki = 0.01f;
	//max_v = 70;
	
//	get_kick_motor_counts(&curr_x);
//	get_y_encoder_position(&curr_y);
	
	while(1)
	{
		robot_position();
		
		position[1] = read_real_position(1);
		position[2] = read_real_position(2);
		position[3] = read_real_position(3);
		position[4] = read_real_position(4);
		
		x1 = ((position[1] + position[4])/2);
		x2 = ((position[3] + position[2])/2);
		curr_x = -1.4142*(x1 - x2)/2;
		
		y1 = ((position[1] + position[2])/2);
		y2 = ((position[4] + position[3])/2);
		curr_y = -1.4142*(y1 - y2)/2;
		
		dy = curr_y - prev_curr_y;      
		dx = curr_x - prev_curr_x; 
		
		err = sqrtf((x_pos-curr_x)*(x_pos-curr_x) + (y_pos-curr_y)*(y_pos-curr_y));
	//	prev_err = sqrt(dy^2 + dx^2);
		
//		lcd_i2c_print(1,1,err,4);
//		lcd_i2c_print(1,6,max_v,3);
		
		phi_angle = atan2(y_pos-curr_y,x_pos-curr_x);
		phi_angle = toDEG(phi_angle);
		
//		lcd_i2c_print(2,1,phi_angle,4);
//		
//		lcd_i2c_print(1,10,x_pos-curr_x,4);
//		lcd_i2c_print(2,10,y_pos-curr_y,4);
	 
  	vel = kp*err + ki*integral + kd*(err - prev_err);
    integral += err;		
		 
   	    if(vel > max_v)
				{
				 vel = max_v;
				}
				else if(vel < -max_v)
				{
				 vel = -max_v;
				}
									
		    if(integral >1000)
				{
				 integral = 1000;
				}
				else if(integral < -1000)
				{
				 integral = -1000;
				}	
				
				//lcd_i2c_print(1,1,phi_angle,4);
				
				robot_newvelocity_run(vel,phi_angle - 8,0,SMOOTH_STOP);
				
				prev_err = err;
				prev_err_w = err_w;
				prev_curr_y = curr_y; 
				prev_curr_x = curr_x; 
				
				if((err < 2) &&(err > -2))
				{
						robot_newvelocity_run(0,0,0,SMOOTH_STOP);
						break;
				}
	}	
}





void robot_gotoxy_pid(float x_pos,float y_pos)		//Tested for 4 wheel omni
{
	
	float err_w=0,prev_err_w=0,w=0,kpw=7.3f,kdw=2.0f,kiw=0.0f,integral_w=0;
	double curr_x,curr_y,integral,dy,dx,prev_curr_x=0,prev_curr_y=0,x_res=0,y_res=0;
  float prev_err,err,vel,phi_angle;
	float real_x = 0, real_y = 0,x1,x2,y1,y2;
	
	kp = 4.0f;
	kd = 16.0f;
	ki = 0;
	max_v = 70;
	
//	get_kick_motor_counts(&curr_x);
//	get_y_encoder_position(&curr_y);
	
	while(1)
	{
		position[1] = read_real_position(1);
		position[2] = read_real_position(2);
		position[3] = read_real_position(3);
		position[4] = read_real_position(4);
		
		x1 = ((position[1] + position[4])/2);
		x2 = ((position[3] + position[2])/2);
		curr_x = -1.4142*(x1 - x2)/2;
		
		y1 = ((position[1] + position[2])/2);
		y2 = ((position[4] + position[3])/2);
		curr_y = -1.4142*(y1 - y2)/2;
		
		dy = curr_y - prev_curr_y;      
		dx = curr_x - prev_curr_x; 
		
		err = sqrtf((x_pos-curr_x)*(x_pos-curr_x) + (y_pos-curr_y)*(y_pos-curr_y));
	//	prev_err = sqrt(dy^2 + dx^2);
		
//		lcd_i2c_print(1,1,err,4);
//		lcd_i2c_print(1,6,max_v,3);
		
		phi_angle = atan2(y_pos-curr_y,x_pos-curr_x);
		phi_angle = toDEG(phi_angle);
		
//		lcd_i2c_print(2,1,phi_angle,4);
//		
//		lcd_i2c_print(1,10,x_pos-curr_x,4);
//		lcd_i2c_print(2,10,y_pos-curr_y,4);
	 
  	vel = kp*err + ki*integral + kd*(err - prev_err);
    integral += err;		
		 
   	    if(vel > max_v)
				{
				 vel = max_v;
				}
				else if(vel < -max_v)
				{
				 vel = -max_v;
				}
									
		    if(integral >1000)
				{
				 integral = 1000;
				}
				else if(integral < -1000)
				{
				 integral = -1000;
				}	
			
				robot_newvelocity_run(vel,phi_angle,0,SMOOTH_STOP);
				
				prev_err = err;
				prev_err_w = err_w;
				prev_curr_y = curr_y; 
				prev_curr_x = curr_x; 
				
				if((err < 2) &&(err > -2))
				{
						robot_newvelocity_run(0,0,0,SMOOTH_STOP);
						break;
				}
	}	
}





void robot_position(void)		////Tested for 4 wheel omni
{
	int16_t x1,x2;
	int16_t y1,y2;

	int resultant = 0 ,angle = 0;
	
	position[1] = read_real_position(1);
	position[2] = read_real_position(2);
	position[3] = read_real_position(3);
	position[4] = read_real_position(4);

		lcd_print(1,1,position[1],5);
		lcd_print(2,1,position[2],5);
		lcd_print(2,9,position[3],5);
		lcd_print(1,9,position[4],5);	
	
	
	x1 = ((position[1] + position[4])/2);
	x2 = ((position[3] + position[2])/2);
	x_centre = -1.4142*(x1 - x2)/2;
	
	lcd_gotoxy(3,1);
	lcd_string("x=");
	lcd_print(3,3,x_centre,4);
	
	y1 = ((position[1] + position[2])/2);
	y2 = ((position[4] + position[3])/2);
	y_centre = -1.4142*(y1 - y2)/2;
	
	lcd_gotoxy(3,9);
	lcd_string("y=");
	lcd_print(3,11,y_centre ,4);
	
	resultant =  sqrtf((float)(x_centre *x_centre  + y_centre *y_centre ));
	angle = (int) toDEG(atan2f(y_centre ,x_centre ));
	
	if(angle<0)
		angle = angle + 360;

	lcd_gotoxy(4,1);
	lcd_string("R=");
	lcd_print(4,4,resultant,4);
	
	lcd_gotoxy(4,9);
	lcd_string("A=");
	lcd_print(4,11,angle,4);
}


void robot_gotoxy_without_pid(float x_pos,float y_pos, float max_v) //Tested for 4 wheel omni
{
	float err_w=0,prev_err_w=0,w=0,kpw=7.3f,kdw=2.0f,kiw=0.0f,integral_w=0;
	double curr_x,curr_y,integral,dy,dx,prev_curr_x=0,prev_curr_y=0,x_res=0,y_res=0;
	float prev_err,err,vel,phi_angle;
	float real_x = 0, real_y = 0,x1,x2,y1,y2;

	kp = 15.0f;
	kd = 16.0f;
	ki = 0;

	robot_position();

	position[1] = read_real_position(1);
	position[2] = read_real_position(2);
	position[3] = read_real_position(3);
	position[4] = read_real_position(4);

	x1 = ((position[1] + position[4])/2);
	x2 = ((position[3] + position[2])/2);
	curr_x = -1.4142*(x1 - x2)/2;

	y1 = ((position[1] + position[2])/2);
	y2 = ((position[4] + position[3])/2);
	curr_y = -1.4142*(y1 - y2)/2;

	dy = curr_y - prev_curr_y;      
	dx = curr_x - prev_curr_x;

	err = sqrtf((x_pos-curr_x)*(x_pos-curr_x) + (y_pos-curr_y)*(y_pos-curr_y));


	phi_angle = atan2(y_pos-curr_y,x_pos-curr_x);
	phi_angle = toDEG(phi_angle);

	vel = kp*err + ki*integral + kd*(err - prev_err);
	integral += err;

	if(vel > max_v)
	{
		vel = max_v;
	}
	else if(vel < -max_v)
	{
		vel = -max_v;
	}

	if(integral >1000)
	{
		integral = 1000;
	}
	else if(integral < -1000)
	{
		integral = -1000;
	}

	robot_newvelocity_run(vel,phi_angle,0,SMOOTH_STOP);

	prev_err = err;
	prev_err_w = err_w;
	prev_curr_y = curr_y;
	prev_curr_x = curr_x;

}
void emergency_stop(void)
{
	motor_off(1);
	motor_off(2);
	motor_off(3);
	motor_off(4);

	//mux_manual();
}

__INLINE bool isVelZero(float vx, float vy, float w)
{
	if((vx==0)&&(vy==0)&&(w==0))
		return true;
	else
		return false;
}

void robot_back_wheel_init(float accel,float joy_x,int flag) //Tested for 4 wheel omni
{
	//hardware_reset();//lm629_reset
 float factor = 2;
 //joy_x = (joy_x /127.0f) * VELOCITY_MAX_SPEED;
	if(flag ==0){
	initialise_lm629(FRONT_MOTOR, VELOCITY_MODE);
	load_position_error_stop(FRONT_MOTOR,6000);
	mux_auto();
//	prev_zero_start_ticks = 0;
//	prev_zero_last_ticks = HAL_GetTick();
//	prev_vel_zero = true;
	
  Load_trajectory_velocity(FRONT_MOTOR,  ABSOLUTE,0, factor*accel, VEL_VA_F);
	StartMotion(FRONT_MOTOR);

	}
	else{
			initialise_lm629(FRONT_MOTOR, VELOCITY_MODE);
	load_position_error_stop(FRONT_MOTOR,6000);
	mux_auto();
		 Load_trajectory_velocity(FRONT_MOTOR,  ABSOLUTE,0, 1*accel, VEL_VA_F);
	StartMotion(FRONT_MOTOR);}
		
}


void robot_throw_init(float vel, float acc)
{

  //hardware_reset();
	initialise_lm629(throw_motor,POSITION_MODE);

	mask_interrupt(throw_motor,POSITION_MODE);

	interrupt_reset(throw_motor);

	define_home(throw_motor);

	Load_trajectory_position(throw_motor,ABSOLUTE,0,vel,acc,POS_PVA);

	StartMotion(throw_motor);
	
}
