#ifndef ENCODER_H_
#define ENCODER_H_

// Includes

#include "prep.h"
//#include "delay.h"
#include "motor.h"
#include "math.h"

/************************************************************************/
/* User Defines                                                         */
#define THROW_YAW_PPR 1848
#define ENC_YAW_ONE_PPR 914.0f
#define ENC_YAW_TWO_PPR 914.0f
#define ENC_COUNTS (500)
#define JUGS_ENC_COUNTS (360)
#define WHEEL_D 		15.1197f  //7.6f	     //in cm // wheels changed(19/12/19)
#define F_LM629 		8000000UL
#define JUGS_WHEEL_D 15.0f     //PREV 18.0f
/************************************************************************/

//Defines
#define RESET 						0x00
#define MASK_INTR 				0x1C
#define INTR_RST 					0x1D
#define LOAD_FILER_PARA 	0x1E
#define UPDATE_FILTER 		0x04
#define INTR_ON_ERROR 		0x1B
#define LOAD_TRAJ 				0x1F
#define START_MOTION 			0x01
#define READ_REAL_VEL 		0x0B
#define READ_SIG_REG 			0x0C
#define READ_DESI_VEL 		0x07
#define READ_REAL_POS 		0x0A
#define READ_DESI_POS 		0x08
#define SET_INDEX_POS 		0x03
#define DEFINE_HOME 			0x02
#define SET_ABS_BRKPT 		0x20
#define SET_REL_BRKPT 		0x21
#define READ_INDEX_POS 		0x09

#define POSITION_CONST (float)(ENC_COUNTS*4.0f/(M_PI*WHEEL_D))
#define VELOCITY_CONST (float)(ENC_COUNTS*4.0f*2048.0f*65536.0f/(F_LM629*M_PI*WHEEL_D))
#define ACCEL_CONST (float)(ENC_COUNTS*4.0f*(2048.0f/F_LM629)*(2048.0f/F_LM629)*(65536.0f/(M_PI*WHEEL_D)))
#define JUGS_POSITION_CONST (float )(JUGS_ENC_COUNTS*4.0f/(M_PI*JUGS_WHEEL_D))
#define JUGS_VELOCITY_CONST (float)(JUGS_ENC_COUNTS*4.0f*2048.0f*65536.0f/(F_LM629*2.0f))
#define JUGS_ACCEL_CONST (float)(28.0f*4.0f*(2048.0f/F_LM629)*(2048.0f/F_LM629)*65536.0f)
#define toDEG(value) ((value)*180.0f/M_PI)
#define toRAD(value) ((value)*M_PI/180.0f)               //value is in deg/sec

typedef enum
{
	POSITION_MODE=0,
	VELOCITY_MODE
} pid_mode;

typedef enum
{
	ABSOLUTE=0,
	RELATIVE
}abs_real_type;

typedef enum
{
	VEL_VA_F=0,
	VEL_VA_R=1,
	VEL_V_F=2,
	VEL_V_R=3,
	POS_V=4,
	POS_PV=5,
	POS_PVA
}value_load_type;

typedef	enum
{
	MOTOR_OFF=0,
	SMOOTH_STOP=1,
	ABRUPT_STOP=2,
	ZERO_VEL
}stop_type;

//Function Declaration
void mux_manual(void);
void mux_auto(void);

void select_chip(uint8_t a);
uint8_t read_cmd(void);
uint8_t read_data(void);
void write_cmd(__IO uint8_t PID_Command);
void write_data(__IO uint8_t PID_Word);
void wait(unsigned long int i);
void checkBusy( void );
void hardware_reset(void);
void software_reset(uint8_t chip_no);
void mask_interrupt(uint8_t chip_no, pid_mode lm629_mode);
void interrupt_reset(uint8_t chip_no);
void set_filter_parameters(uint8_t chip_no, pid_mode lm629_mode);
void load_position_error(uint8_t chip_no, unsigned int error);
void initialise_lm629(uint8_t chip_no, pid_mode lm629_mode);
void abrupt_stop(uint8_t chip_no);
void smooth_stop(uint8_t chip_no);
void motor_off(uint8_t chip_no);
uint8_t read_status_byte(uint8_t chip_no);
float read_real_velocity(uint8_t chip_no);
uint16_t read_signal_registers(uint8_t chip_no);
float read_desired_velocity(uint8_t chip_no);
float read_real_position(uint8_t chip_no);
float read_desired_position(uint8_t chip_no);
void set_index_position(uint8_t chip_no);
void define_home(uint8_t chip_no);
void set_absolute_breakpoint(uint8_t chip_no, float position_f);
void set_relative_breakpoint(uint8_t chip_no, float position_f);
int32_t read_index_position(uint8_t chip_no);
void StartMotion(uint8_t chip_no);
void Load_trajectory_position(uint8_t chip_no, abs_real_type rel_abs, float position_f, float velocity_f, float accn_f, value_load_type load_type);
void Load_trajectory_velocity(uint8_t chip_no, abs_real_type rel_abs, float velocity_f, float accn_f, value_load_type load_type);
void initialise_velocity_mode(float accel);
void check_sign(pid_mode lm629_mode);
void mask_interrupt_value(uint8_t chip_no, uint8_t value);
void virtual_hardware_reset(void);
void load_position_error_stop(uint8_t chip_no, uint16_t error);
void bot_define_home(void);
float read_real_jugs_velocity(uint8_t chip_no);
#endif /* ENCODER_H_ */
