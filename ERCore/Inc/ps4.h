#ifndef PS4_H_
#define PS4_H_

#include "prep.h"

#define PS4_SHARE (1<<0 & upper_byte)
#define PS4_TOUCHPAD (1<<1 & upper_byte)
#define PS4_R3 (1<<2 & upper_byte)
#define PS4_OPTIONS (1<<3 & upper_byte)
#define PS4_PAD_UP (1<<4 & upper_byte)
#define PS4_PAD_RIGHT (1<<5 & upper_byte)
#define PS4_PAD_DOWN (1<<6 & upper_byte)
#define PS4_PAD_LEFT (1<<7 & upper_byte)

#define PS4_L2 (1<<0 & lower_byte)
#define PS4_R2 (1<<1 & lower_byte)
#define PS4_L1 (1<<2 & lower_byte)
#define PS4_R1 (1<<3 & lower_byte)
#define PS4_TRIANGLE (1<<4 & lower_byte)
#define PS4_CIRCLE (1<<5 & lower_byte)
#define PS4_CROSS (1<<6 & lower_byte)
#define PS4_SQUARE (1<<7 & lower_byte)

#define PS4_LX 4-3
#define PS4_LY 5-3
#define PS4_RX 6-3
#define PS4_RY 7-3


#define prev_PS4_SHARE (1<<0 & xu)
#define prev_PS4_TOUCHPAD (1<<1 & xu)
#define prev_PS4_R3 (1<<2 & xu)
#define prev_PS4_OPTIONS (1<<3 & xu)
#define prev_PS4_PAD_UP (1<<4 & xu)
#define prev_PS4_PAD_RIGHT (1<<5 & xu)
#define prev_PS4_PAD_DOWN (1<<6 & xu)
#define prev_PS4_PAD_LEFT (1<<7 & xu)

#define prev_PS4_L2 (1<<0 & xl)
#define prev_PS4_R2 (1<<1 & xl)
#define prev_PS4_L1 (1<<2 & xl)
#define prev_PS4_R1 (1<<3 & xl)
#define prev_PS4_TRIANGLE (1<<4 & xl)
#define prev_PS4_CIRCLE (1<<5 & xl)
#define prev_PS4_CROSS (1<<6 & xl)
#define prev_PS4_SQUARE (1<<7 & xl)

#define PS4_PAD_UP_PRESSED (PS4_PAD_UP && !(prev_PS4_PAD_UP))
#define PS4_PAD_RIGHT_PRESSED (PS4_PAD_RIGHT && !(prev_PS4_PAD_RIGHT))
#define PS4_PAD_DOWN_PRESSED (PS4_PAD_DOWN && !(prev_PS4_PAD_DOWN))
#define PS4_PAD_LEFT_PRESSED (PS4_PAD_LEFT && !(prev_PS4_PAD_LEFT))
#define PS4_TRIANGLE_PRESSED (PS4_TRIANGLE && !(prev_PS4_TRIANGLE))
#define PS4_CIRCLE_PRESSED (PS4_CIRCLE && !(prev_PS4_CIRCLE))
#define PS4_CROSS_PRESSED (PS4_CROSS && !(prev_PS4_CROSS))
#define PS4_SQUARE_PRESSED (PS4_SQUARE && !(prev_PS4_SQUARE))

#define PS4_SHARE_PRESSED (PS4_SHARE && !(prev_PS4_SHARE))
#define PS4_OPTIONS_PRESSED (PS4_OPTIONS && !(prev_PS4_OPTIONS))
#define PS4_L1_PRESSED (PS4_L1 && !(prev_PS4_L1))
#define PS4_L2_PRESSED (PS4_L2 && !(prev_PS4_L2))
#define PS4_R1_PRESSED (PS4_R1 && !(prev_PS4_R1))
#define PS4_R2_PRESSED (PS4_R2 && !(prev_PS4_R2))
#define PS4_TOUCHPAD_PRESSED (PS4_TOUCHPAD && !(prev_PS4_TOUCHPAD))

#define PS4_PAD_UP_RELEASED (!PS4_PAD_UP && (prev_PS4_PAD_UP))
#define PS4_PAD_RIGHT_RELEASED (!PS4_PAD_RIGHT && (prev_PS4_PAD_RIGHT))
#define PS4_PAD_DOWN_RELEASED (!PS4_PAD_DOWN && (prev_PS4_PAD_DOWN))
#define PS4_PAD_LEFT_RELEASED (!PS4_PAD_LEFT && (prev_PS4_PAD_LEFT))
#define PS4_TRIANGLE_RELEASED (!PS4_TRIANGLE && (prev_PS4_TRIANGLE))
#define PS4_CIRCLE_RELEASED (!PS4_CIRCLE && (prev_PS4_CIRCLE))
#define PS4_CROSS_RELEASED (!PS4_CROSS && (prev_PS4_CROSS))
#define PS4_SQUARE_RELEASED (!PS4_SQUARE && (prev_PS4_SQUARE))

#define PS4_SHARE_RELEASED (!PS4_SHARE && (prev_PS4_SHARE))
#define PS4_OPTIONS_RELEASED (!PS4_OPTIONS && (prev_PS4_OPTIONS))
#define PS4_L1_RELEASED (!PS4_L1 && (prev_PS4_L1))
#define PS4_L2_RELEASED (!PS4_L2 && (prev_PS4_L2))
#define PS4_R1_RELEASED (!PS4_R1 && (prev_PS4_R1))
#define PS4_R2_RELEASED (!PS4_R2 && (prev_PS4_R2))

#define PS4_PAD_UP_STATE_CHANGE (PS4_PAD_UP ^ (prev_PS4_PAD_UP))
#define PS4_PAD_RIGHT_STATE_CHANGE (PS4_PAD_RIGHT ^ (prev_PS4_PAD_RIGHT))
#define PS4_PAD_DOWN_STATE_CHANGE (PS4_PAD_DOWN ^ (prev_PS4_PAD_DOWN))
#define PS4_PAD_LEFT_STATE_CHANGE (PS4_PAD_LEFT ^ (prev_PS4_PAD_LEFT))
#define PS4_TRIANGLE_STATE_CHANGE (PS4_TRIANGLE ^ (prev_PS4_TRIANGLE))
#define PS4_CIRCLE_STATE_CHANGE (PS4_CIRCLE ^ (prev_PS4_CIRCLE))
#define PS4_CROSS_STATE_CHANGE (PS4_CROSS ^ (prev_PS4_CROSS))
#define PS4_SQUARE_STATE_CHANGE (PS4_SQUARE ^ (prev_PS4_SQUARE))

#define PS4_SHARE_STATE_CHANGE (PS4_SHARE ^ (prev_PS4_SHARE))
#define PS4_OPTIONS_STATE_CHANGE (PS4_OPTIONS ^ (prev_PS4_OPTIONS))
#define PS4_L1_STATE_CHANGE (PS4_L1 ^ (prev_PS4_L1))
#define PS4_L2_STATE_CHANGE (PS4_L2 ^ (prev_PS4_L2))
#define PS4_R1_STATE_CHANGE (PS4_R1 ^ (prev_PS4_R1))
#define PS4_R2_STATE_CHANGE (PS4_R2 ^ (prev_PS4_R2))


#define triangle 0x10
#define circle 0x20
#define cross 0x40
#define square 0x80
#define L2 0X01
#define R2 0X02
#define L1 0X04
#define R1 0X08
#define L1_triangle 0X14
#define L2_triangle 0X11
#define R1_triangle 0X18
#define R2_triangle 0X12
#define L1_circle 0X24
#define L2_circle 0X21
#define R1_circle 0X28
#define R2_circle 0X22
#define L1_cross 0X44
#define L2_cross 0X41
#define R1_cross 0X48
#define R2_cross 0X42
#define L1_square 0X84
#define L2_square 0X81
#define R1_square 0X88
#define R2_square 0X82
#define L1_triangle 0X14
#define L2_triangle 0X11
#define R1_triangle 0X18
#define R2_triangle 0X12
#define up 0x10
#define right 0x20
#define down 0x40
#define left 0x80
#define L1_right 0X24
#define L2_right 0X21
#define R1_right 0X28
#define R2_right 0X22
#define L1_down 0X44
#define L2_down 0X41
#define R1_down 0X48
#define R2_down 0X42
#define L1_left 0X84
#define L2_left 0X81
#define R1_left 0X88
#define R2_left 0X82
#define L1_up 0X14
#define L2_up 0X11
#define R1_up 0X18
#define R2_up 0X12



#define SAVE_CONSOLE	xu = upper_byte; \
											xl = lower_byte;



typedef struct joystick_t 
{
	uint8_t* value;
	uint8_t center;
	uint8_t deadband;
	bool invert;
}joystick;

typedef enum
{
	START_R = 0,
	DATA_R = 1,
	STOP_R = 2
}
console_status;

#define N 250
typedef struct queue{
uint8_t data[N];
uint8_t front,rear,count;	
}queue;

#define START_BYTE 'c'
#define STOP_BYTE 'x'

extern joystick lx_;
extern joystick ly_;
extern joystick rx_;
extern joystick ry_;

extern joystick *lx;
extern joystick *ly;
extern joystick *rx;
extern joystick *ry;


extern uint8_t xu,xl,upper_byte,lower_byte,temp_upper_byte,temp_lower_byte,ANALOG_L2,ANALOG_R2;
extern uint8_t joy_data[10];
extern uint8_t buf[20];
extern uint8_t transmit_start_byte[1];
extern bool pairing_flag;
extern queue q_upper;
extern queue q_lower;
extern bool qufull,qlfull;
extern uint8_t temp_upper_byte,temp_lower_byte;
extern uint32_t time_upper;
extern uint8_t prev_data_upper;
extern int get_time_upper;

extern uint32_t time_lower;
extern uint8_t prev_data_lower;
extern int get_time_lower;
extern uint8_t case_q;
extern uint8_t clbk_flag; 
extern bool pairing_flag;
extern bool system_startup;
extern uint8_t transmit_start_byte[1];


void queue_init(queue* q);
uint8_t qfull(queue* q);
uint8_t qempty(queue* q);
void enqueue(queue* q,uint8_t d);
uint8_t dequeue(queue* q);
int getcount(queue *q);
void printqueue(queue* q);
void ps4_init(void);
int get_stick(joystick *stick);
void receive_max3421e(UART_HandleTypeDef *huart);
void receive_ps4(void);
void connect_ps4(void);  

#endif /* PS4_H_ */

