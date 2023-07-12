#ifndef BASE_DRIVE_H_
#define BASE_DRIVE_H_

//Includes
#include "encoder.h"
#include "motor.h"


/************************************************************************/
/* User Defines                                                         */

#define ROBOT_R 26.69f   ///38.0f
extern int VELOCITY_MAX_SPEED;
#define JOYSTICK_DEADBAND 10


#define POSITION_UPDATE_SAMPLE_INTERVAL 0.01f
#define VELOCITY_UPDATE_SAMPLE_INTERVAL 0.1f

#define SMOOTH_STOP_ROBOT		smooth_stop(1); \ smooth_stop(2);	\ smooth_stop(3); \ smooth_stop(4);
/************************************************************************/

typedef enum
{
	XY = 0,
	X_Y = 1,
	Y_X = 2,
}trajectory_path;

typedef struct coordinate{
	float x;
	float y;
}point;

typedef enum{
	POSITIVE = '+',
	NEGATIVE = '-'
}Curve_t;

// Global Variables
extern __IO float robot_x, robot_y, robot_theta;

extern uint32_t prev_zero_start_ticks;
extern uint32_t prev_zero_last_ticks;
extern bool prev_vel_zero;
extern bool y_susp_reached_flag;
extern uint8_t fast_mode;
extern trajectory_path _trajectory;
extern bool stick_invert_flag ;



//Function Declarations
void robot_brake(void);

void robot_velocity_init(float accel);

void robot_position_init(float vel, float acc);

void robot_position_run(float pos_x,float pos_y,float vel,float acc);

void robot_velocity_run(float vx, float vy, float w, stop_type stop_method);

void robot_throw_init(float vel, float acc);

void robot_back_wheel_init(float accel,float joy_x,int flag);

void robot_joystick_velocity(void);
void robot_joystick_velocity_orientation(void);
void robot_joystick_velocity_imu(void);



void emergency_stop(void);

extern __INLINE bool isVelZero(float vx, float vy, float w);

void robot_move_lm629(float distance, float angle, float velocity, float accel);

void robot_rotate_lm629(float theta, float w, float accel);
void robot_gotoxy(float x_pos,float y_pos);
void robot_gotoy_pid(float y_pos);
void robot_gotox_pid(float x_pos);
void robot_gotoxy_without_pid(float x_pos,float y_pos, float max_v);
void robot_gotoxy_pid_with_vel(float x_pos,float y_pos, float max_v);
void robot_move(float distance, float angle, float velocity, float accel,float theta,float w,float ang_acc);
void robot_rotate(float theta, float max_w);
void robot_gotoxytheta(float x_pos,float y_pos,float turn_ang);
void robot_gotox(float x_pos);
void robot_gotoy(float y_pos);
void robot_newvelocity_run(float v, float phi,float w, stop_type stop_method);
float q_cos(float x);
float cos_360(float x);
void robot_new_rotate(float theta);
float sin_360(float x);
void robot_gotoxtheta(float x_pos, float theta);	
void robot_gotoytheta(float y_pos, float theta);
void robot_new_rotate(float theta);
void robot_position(void);
void rod_curve(float w , float r,stop_type stop_method);


void rotate_clk();
void rotate_anti();
void forward();
void backward();
void lft();
void rght();
void stop();
void rotate_pid();
void rotate_without_pid();


#endif /* BASE_DRIVE_H_ */


