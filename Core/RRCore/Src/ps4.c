#include "init.h"
#include "ps4.h"
#include "stdlib.h"

uint8_t xu=0xff, xl=0xff, upper_byte=0, lower_byte=0,temp_upper_byte=0,temp_lower_byte=0,ANALOG_L2=0,ANALOG_R2=0;
uint8_t joy_data[10]={0};
uint8_t joy_data_temp[10]={0};           
uint8_t buf[20] = {0};
uint32_t time_upper;
uint8_t prev_data_upper = 0x00;
int get_time_upper = 1;
uint8_t case_q=0;
uint32_t time_lower;
uint8_t prev_data_lower = 0x00;
int get_time_lower = 1;
bool qufull=0,qlfull=0;
bool pairing_flag = false;
uint8_t count_tick = 0;
uint8_t transmit_start_byte[1] = {'a'};
queue q_upper;
queue q_lower;
bool system_startup = 0;


joystick lx_ = {&joy_data[PS4_LX],127,JOYSTICK_DEADBAND,false};
joystick ly_ = {&joy_data[PS4_LY],127,JOYSTICK_DEADBAND,true};
joystick rx_ = {&joy_data[PS4_RX],127,JOYSTICK_DEADBAND+40,true};
joystick ry_ = {&joy_data[PS4_RY],127,JOYSTICK_DEADBAND,true};

joystick* lx = &lx_;
joystick* ly = &ly_;
joystick* rx = &rx_;
joystick* ry = &ry_;


void ps4_init()
{
	HAL_UART_Transmit(CONSOLE_MODULE,transmit_start_byte,1,1000);
	HAL_UART_Receive_DMA(CONSOLE_MODULE,buf,20);		
	system_startup = 1;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	
				if(system_startup == 1)
						receive_max3421e(CONSOLE_MODULE);
				
		if(huart->Instance == UART4)
		{		
				HAL_UART_Transmit(&huart4,transmit_start_byte,1,50);
				HAL_UART_Receive_DMA(&huart4,buf,20);
		}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4){
			if(system_startup == 1)
					receive_max3421e(CONSOLE_MODULE);
	
	if(huart->Instance == UART4)
	{		
			HAL_UART_Transmit(&huart4,transmit_start_byte,1,50);
			HAL_UART_Receive_DMA(&huart4,buf,20);
	}
}
	else{
	}
}

int get_stick(joystick* stick)
{
	int value = *(stick->value);
	int prev_value;
	
	prev_value = value;
	
	if(stick->invert == true)
		value = 255 - value;	
	
	value = value - stick->center;
	
	if(abs(value) < (stick->deadband))
		value = 0;
	
	if(value>0)                                         
		return (127*value/(255-stick->center));
	else
		return (127*value/(stick->center));
}


void queue_init(queue* q)
{
	q->rear = -1;
	q->front = 0;
	q->count = 0;
}

uint8_t qfull(queue* q)
{
	if(q->count >= N)
		return 1;
	else
		return 0;
}

uint8_t qempty(queue* q)
{
	if(q->count > 0)
		return 0;
	else
		 return 1;
}
																								
void enqueue(queue* q,uint8_t d)
{
	q->rear = (q->rear+1);
	q->data[q->rear] = d;
	q->count++;
}

uint8_t dequeue(queue* q)
{
	uint8_t temp;
	temp = q->data[q->front];
	q->front = (q->front + 1);
	(q->count)--;
	lcd_print(2,11,temp,4);
	return temp;
}

int getcount(queue *q)
{
	return q->count;
}

void printqueue(queue* q)
{
	for(int i = 0;i<q->count;i++)
	{
		lcd_print(1,1,q->data[q->rear],3);
	}
}

void switch_function_lower(uint8_t temp_l)
{
	if(temp_l != R1 || temp_l != R2 || temp_l != L1 || temp_l != L2)
	{
		if((prev_data_lower == temp_l) && (time_lower != 0x00))
		{
			case_q = 1;
			enqueue(&q_lower,prev_data_lower);
			time_lower = 0;
			get_time_lower = 0;
		}
		else if((prev_data_lower!= temp_l) && (temp_l == 0x00))
		{
			case_q = 2;
			enqueue(&q_lower,prev_data_lower);
			time_lower = 0;
			get_time_lower = 0;
		}
		else if((prev_data_lower!= temp_l) && (temp_l != 0x00))
		{
			case_q = 3;
			enqueue(&q_lower,prev_data_lower | temp_l);
			time_lower = 0;
			get_time_lower = 0;
		}
		else if((HAL_GetTick() - time_lower <= 1000) && (prev_data_lower!=circle)&& (prev_data_lower!=triangle)&&(prev_data_lower!=cross)&&(prev_data_lower!=square)&&(prev_data_lower!=L1)&&(prev_data_lower!=L2)&&(prev_data_lower!=R1)&&(prev_data_lower!=R2))
		{
			case_q = 4;
			enqueue(&q_lower,prev_data_lower);
			time_lower = 0;
			get_time_lower = 0;
		}
	}
	else
	{
		if((prev_data_lower == temp_l) && (time_lower!= 0x00) && (prev_data_upper == prev_data_lower))
		{
			case_q = 5;
			enqueue(&q_lower,prev_data_lower);
			time_lower = 0;
			get_time_lower = 0;
		}
		else if(HAL_GetTick() - time_lower <= 1000 && (prev_data_lower!=temp_l) && (temp_l == 0x00))
		{
			case_q = 6;			
			enqueue(&q_lower,prev_data_lower);
			time_lower = 0;
			get_time_lower = 0;
		}
	}
}

void switch_function_upper(uint8_t temp_u)
{
	if((prev_data_upper == temp_u) && (time_upper!=0x00))
	{
			case_q = 7;		
		enqueue(&q_upper,prev_data_upper);
		time_upper = 0;
		get_time_upper = 0;
	}
	else if((HAL_GetTick() - time_upper <= 1000) && (prev_data_upper != temp_u) && (temp_u == 0x00))
	{
		case_q = 8;		
		enqueue(&q_upper,prev_data_upper);
		time_upper = 0;
		get_time_upper = 0;
	}
	else if((prev_data_upper != temp_u) && (temp_u != 0x00))
	{
		case_q = 8;		
		enqueue(&q_upper,prev_data_upper | temp_u);
		time_upper = 0;
		get_time_upper = 0;
	}
	else if((HAL_GetTick() - time_upper <= 1000) && (prev_data_upper != right) && (prev_data_upper !=up)&& (prev_data_upper!=down)&&(prev_data_upper!=left))
	{
			case_q = 9;		
		enqueue(&q_upper,prev_data_upper);
		time_upper = 0;
		get_time_upper = 0;
	}
}



void receive_max3421e(UART_HandleTypeDef *huart)
{
	int i=0,k = 0;

		while(1)
		{

			if(buf[i]!='c'){
				i++;

			}
			else{

				break;
			}
		}

				global_temp8[1] = 0;
				
				for(global_temp8[0]=i; global_temp8[0]< i+10; global_temp8[0]++)
				{

					k = global_temp8[0]%10;
					joy_data[global_temp8[1]] = buf[k];
					global_temp8[1]++;
				}

	if(joy_data[9] == 'u')
	{

		pairing_flag = false;
	/*	for(i=0;i<10;i++)
		{
			joy_data[i] = prev_joy_data[i];
		}*/
	}		
				
	if(joy_data[9] == 'p')
	{

		pairing_flag = true;
	}
	
	/*for(i=0;i<10;i++)
	{
		joy_data[i] = joy_data_temp[i];
	}*/
	
	upper_byte = joy_data[5];
	lower_byte = joy_data[6];
	ANALOG_L2 =  joy_data[7]; 						// COMPLETE VALUE PASSED
	ANALOG_R2 = joy_data[8];	// VALUE CHANGED TO 100
//		for(i=0;i<10;i++)
//		{
//			prev_joy_data[i] = joy_data[i];
//		}
}

void receive_ps4(void)
{
	HAL_UART_Transmit(CONSOLE_MODULE,transmit_start_byte,1,50);
	HAL_UART_Receive(CONSOLE_MODULE,buf,10,100);	
	
	receive_max3421e(CONSOLE_MODULE);
	if(pairing_flag == false)
	{
		emergency_stop();
		lcd_clear();
		lcd_gotoxy(1,5);
		lcd_string("pair lost");
		while(!pairing_flag)
		{
			HAL_UART_Transmit(CONSOLE_MODULE,transmit_start_byte,1,50);
			HAL_UART_Receive(CONSOLE_MODULE,buf,10,100);	
			receive_max3421e(CONSOLE_MODULE);
		}
		robot_mode = PLAY;
		f_gerege_up = false;
	  f_shagai_grip = false;
		shagai_up_flag = false;
	  mech_shift = false;
		
		lcd_clear();
		lcd_gotoxy(1,1);
		lcd_string("PLAY");
	}
	/*else if (PS4_TOUCHPAD_PRESSED)
	{
		SAVE_CONSOLE;
		retry_flag = true;
		while(retry_flag == true)
		{
			connect_ps4();
			if (PS4_TOUCHPAD_PRESSED)
			{
				retry_flag = false;
			}
			SAVE_CONSOLE;
		}
	}*/
}


void connect_ps4(void)
{

	HAL_UART_Transmit(CONSOLE_MODULE,transmit_start_byte,1,50);

	HAL_UART_Receive(CONSOLE_MODULE,buf,10,100);

	receive_max3421e(CONSOLE_MODULE);

	
}


