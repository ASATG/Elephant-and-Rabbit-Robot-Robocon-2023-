#include "init.h"
#include "encoder.h"
//#include "delay.h"



void mux_manual(void)
{
	setPin(PE6_LM629_LT1_GPIO_Port,PE6_LM629_LT1_Pin);
	clrPin(PD11_LM629_LT2_GPIO_Port,PD11_LM629_LT2_Pin);
	setPin(PE7_LM629_2LT1_GPIO_Port,PE7_LM629_2LT1_Pin);
	clrPin(PE1_LM629_2LT2_GPIO_Port,PE1_LM629_2LT2_Pin);
//	setPin(LM629_LT1_3_GPIO_Port,LM629_LT1_3_Pin);
//	clrPin(LM629_LT2_3_GPIO_Port,LM629_LT2_3_Pin);
}

void mux_auto(void)
{
	clrPin(PE6_LM629_LT1_GPIO_Port,PE6_LM629_LT1_Pin);
	setPin(PD11_LM629_LT2_GPIO_Port,PD11_LM629_LT2_Pin);
	clrPin(PE7_LM629_2LT1_GPIO_Port,PE7_LM629_2LT1_Pin);
	setPin(PE1_LM629_2LT2_GPIO_Port,PE1_LM629_2LT2_Pin);
//	clrPin(LM629_LT1_3_GPIO_Port,LM629_LT1_3_Pin);
//	setPin(LM629_LT2_3_GPIO_Port,LM629_LT2_3_Pin);
}


void select_chip(uint8_t a)
{
if(a==1||a==2||a==3||a==4)
	{
	clrPin(PC8_LM629_EN1_GPIO_Port,PC8_LM629_EN1_Pin); //MOT2 kept high(connect controller2 demux enable to MOT2_A ->PC9)
	setPin(PA8_LM629_EN2_GPIO_Port,PA8_LM629_EN2_Pin); //MOT2 kept high(connect controller2 demux enable to MOT2_A ->PC9)
//	setPin(LM629_En3_GPIO_Port,LM629_En3_Pin);//MOT2 kept low(connect controller1 demux enable to MOT2_B ->PC8)
	}
	else if(a==5 || a==6 ||a==7 || a==8)
	{
	setPin(PC8_LM629_EN1_GPIO_Port,PC8_LM629_EN1_Pin); //MOT2 kept low(connect controller2 demux enable to MOT2_A ->PC9)
	clrPin(PA8_LM629_EN2_GPIO_Port,PA8_LM629_EN2_Pin);//MOT2 kept high(connect controller1 demux enable to MOT2_B ->PC8)
//	setPin(LM629_En3_GPIO_Port,LM629_En3_Pin); //MOT2 kept high(connect controller2 demux enable to MOT2_A ->PC9)
	}
	else
	{
	clrPin(PC8_LM629_EN1_GPIO_Port,PC8_LM629_EN1_Pin);
	clrPin(PA8_LM629_EN2_GPIO_Port,PA8_LM629_EN2_Pin);
//	setPin(LM629_En3_GPIO_Port,LM629_En3_Pin);
	}
	switch(a)
	{
		case 1:
			clrPin(PD8_LM629_CS_S0_GPIO_Port,PD8_LM629_CS_S0_Pin);		//S0=0;
			clrPin(PD9_LM629_CS_S1_GPIO_Port,PD9_LM629_CS_S1_Pin);		//S1=0;
			break;

	  case 2:
			setPin(PD8_LM629_CS_S0_GPIO_Port,PD8_LM629_CS_S0_Pin);		//S0=1;
			clrPin(PD9_LM629_CS_S1_GPIO_Port,PD9_LM629_CS_S1_Pin);		//S1=0;
			break;
				
		case 3:
			clrPin(PD8_LM629_CS_S0_GPIO_Port,PD8_LM629_CS_S0_Pin);		//S0=0;
			setPin(PD9_LM629_CS_S1_GPIO_Port,PD9_LM629_CS_S1_Pin);		//S1=1;
			break;

		case 4:
			setPin(PD8_LM629_CS_S0_GPIO_Port,PD8_LM629_CS_S0_Pin);		//S0=1;
			setPin(PD9_LM629_CS_S1_GPIO_Port,PD9_LM629_CS_S1_Pin);		//S1=1;
			break;
		case 5:
			clrPin(PE5_LM629_CS_S0_GPIO_Port,PE5_LM629_CS_S0_Pin);		//S3=0;
			clrPin(PE10_LM629_CS_S1_GPIO_Port,PE10_LM629_CS_S1_Pin);		//S4=0;
		  break;
		case 6:
			setPin(PE5_LM629_CS_S0_GPIO_Port,PE5_LM629_CS_S0_Pin);		//S3=1;
			clrPin(PE10_LM629_CS_S1_GPIO_Port,PE10_LM629_CS_S1_Pin);		//S4=0;
			break;
		case 7:
			clrPin(PE5_LM629_CS_S0_GPIO_Port,PE5_LM629_CS_S0_Pin);		//S3=0;
			setPin(PE10_LM629_CS_S1_GPIO_Port,PE10_LM629_CS_S1_Pin);		//S4=1;
			break;
		case 8:
			setPin(PE5_LM629_CS_S0_GPIO_Port,PE5_LM629_CS_S0_Pin);		//S3=1;
			setPin(PE10_LM629_CS_S1_GPIO_Port,PE10_LM629_CS_S1_Pin);		//S4=1;
			break;
//		case 9:
//			clrPin(LM629_CS_S0_3_GPIO_Port,LM629_CS_S0_3_Pin);		//S5=0;
//			clrPin(LM629_CS_S1_3_GPIO_Port,LM629_CS_S1_3_Pin);		//S6=0;
//		  break;
//		case 10:
//			setPin(LM629_CS_S0_3_GPIO_Port,LM629_CS_S0_3_Pin);		//S5=1;
//			clrPin(LM629_CS_S1_3_GPIO_Port,LM629_CS_S1_3_Pin);		//S6=0;
//			break;
//		case 11:
//			clrPin(LM629_CS_S0_3_GPIO_Port,LM629_CS_S0_3_Pin);		//S5=0;
//			setPin(LM629_CS_S1_3_GPIO_Port,LM629_CS_S1_3_Pin);		//S6=1;
//			break;
//		case 12:
//			setPin(LM629_CS_S0_3_GPIO_Port,LM629_CS_S0_3_Pin);		//S5=1;
//			setPin(LM629_CS_S1_3_GPIO_Port,LM629_CS_S1_3_Pin);		//S6=1;
//			break;
      
		default:
			lcd_string("Wrong CHIP");
			//emergency_stop();
			while(1);
	}
}

uint8_t read_cmd(void)
{
	LM629_GPIO_Port->MODER &= 0xFFFF0000;				//make input
	__IO uint8_t readtemp;
	//HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	clrPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=0;
	clrPin(PB2_LM629_RD_GPIO_Port,PB2_LM629_RD_Pin);		//read=0;

	_delay_us(1);
	
	readtemp = (LM629_GPIO_Port->IDR)&0x00FF;

	setPin(PB2_LM629_RD_GPIO_Port,PB2_LM629_RD_Pin);		//read=1;
	setPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=1;
	//HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	return readtemp;
}

uint8_t read_data(void)
{
	__IO uint8_t readtemp;

	LM629_GPIO_Port->MODER &= 0xFFFF0000;	                    //make input
	//HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	setPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=1; for data
	clrPin(PB2_LM629_RD_GPIO_Port,PB2_LM629_RD_Pin);		//read=0;

	_delay_us(1);

	readtemp = (LM629_GPIO_Port->IDR)&0x00FF;

	setPin(PB2_LM629_RD_GPIO_Port,PB2_LM629_RD_Pin);		//read=1;
	setPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=1;
	//HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	return readtemp;
}


void write_cmd(__IO uint8_t PID_Command)
{
	LM629_GPIO_Port->MODER |= 0x00005555;
	LM629_GPIO_Port->ODR &= 0xFF00;
	LM629_GPIO_Port->ODR |= (PID_Command & 0x00FF);

	//HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
	clrPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=0;
	clrPin(PB0_LM629_WR_GPIO_Port,PB0_LM629_WR_Pin);		//WR=0;

	_delay_us(1);

	setPin(PB0_LM629_WR_GPIO_Port,PB0_LM629_WR_Pin);		//WR=1;
	setPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=1;

	LM629_GPIO_Port->MODER &= 0xFFFF0000;          	//make input
	//HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

void write_data(__IO uint8_t PID_Word)
{
	LM629_GPIO_Port->MODER |= 0x00005555;                //make output
	LM629_GPIO_Port->ODR &= 0xFF00;
	LM629_GPIO_Port->ODR |= (PID_Word & 0x00FF);
	
	//HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);

	setPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=1;
	clrPin(PB0_LM629_WR_GPIO_Port,PB0_LM629_WR_Pin);		//WR=0;

	_delay_us(1);

	setPin(PB0_LM629_WR_GPIO_Port,PB0_LM629_WR_Pin);		//WR=1;

	LM629_GPIO_Port->MODER &= 0xFFFF0000;                  //make input
	//HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

void wait(unsigned long int i)
{
    unsigned long int j;
    for (j=1;j<i;j++);
    return;
}

void checkBusy( void )
{
	int counter=800;
	do
    {
	}
	while((read_cmd() & 0x01) && counter--);
}

void hardware_reset(void)
{
	clrPin(PE8_LM629_RESET_GPIO_Port,PE8_LM629_RESET_Pin);		//Hardware RESET

	_delay_us(8);

	setPin(PE8_LM629_RESET_GPIO_Port,PE8_LM629_RESET_Pin);

	_delay_ms(2);
	
}

void software_reset(uint8_t chip_no)
{
	bool rst_fault_flag = true;
	select_chip(chip_no);
	lcd_gotoxy(1,8);
	lcd_string("R");
	lcd_print(1,10,chip_no,1);
	while(rst_fault_flag == true)
	{
		write_cmd(RESET);			//software reset
		checkBusy();
		_delay_ms(2);

			lcd_print(1,1,read_status_byte(chip_no),4);
		if((read_status_byte(chip_no) == 0x84) || (read_status_byte(chip_no) == 0xC4))
		{
			interrupt_reset(chip_no);
		lcd_print(2,1,read_status_byte(chip_no),4);
			if((read_status_byte(chip_no) == 0x80) || (read_status_byte(chip_no) == 0xC0))
				rst_fault_flag = false;
			lcd_print(2,7,read_status_byte(chip_no),4);
		}
	}
	lcd_gotoxy(1,8);
	lcd_string("   ");
}

void mask_interrupt(uint8_t chip_no, pid_mode lm629_mode)
{
	/*
	Bit 7 -Not Used
	Bit 6 -BreakPoint Reached Interrupt
	Bit 5 -Excessive Position Error Interrupt
	Bit 4 -Wraparound Error Interrupt
	Bit 3 -Index Pulse Interrupt
	Bit 2 -Trajectory Complete Interrupt
	Bit 1 -Command Error Interrupt
	Bit 0 -Not Used
	*/
	select_chip(chip_no);
	if(lm629_mode == VELOCITY_MODE)
	{
		write_cmd(MASK_INTR);         //Mask Interrupt
		checkBusy();

		write_data(0x00);     				//index enable
		write_data(0x04);						 //initially 0x40  44=breakpoint and trajectory complete
		checkBusy();

	}
	else
	{
		write_cmd(MASK_INTR);         //Mask Interrupt
		checkBusy();

		write_data(0x00);          //index enable
		write_data(0x04);         //initially 0x04  44=breakpoint and trajectory complete
		checkBusy();

	}
}

void mask_interrupt_value(uint8_t chip_no, uint8_t value)
{
	/*
	Bit 7 -Not Used
	Bit 6 -BreakPoint Reached Interrupt
	Bit 5 -Excessive Position Error Interrupt
	Bit 4 -Wraparound Error Interrupt
	Bit 3 -Index Pulse Interrupt
	Bit 2 -Trajectory Complete Interrupt
	Bit 1 -Command Error Interrupt
	Bit 0 -Not Used
	*/
	select_chip(chip_no);
	
	write_cmd(MASK_INTR);         //Mask Interrupt
	checkBusy();
	
	write_data(0x00);          //index enable
	write_data(value);         //initially 0x04  44=breakpoint and trajectory complete
	checkBusy();
}

//interrupt reset function to be used after every HI interrupt or any other
void interrupt_reset(uint8_t chip_no)
{
	select_chip(chip_no);
	write_cmd(INTR_RST);
	checkBusy();

	write_data(0x00);
	write_data(0x00);
	checkBusy();

 
}

void set_filter_parameters(uint8_t chip_no, pid_mode lm629_mode)
{
	
		volatile int sampling = 16;
		volatile unsigned int kp;// = 75;
		volatile unsigned int kd;// = 450;
	  volatile unsigned int ki;// = 450;v
	
		if(chip_no == 1)				//FRONT_LEFT_MOTOR
		{
			kp = 280;kd = 280;       // 300, 320
			//kp = 144;	kd = 256;		//No Load Conditions
		}
		else if(chip_no == 2)		//BACK_LEFT_MOTOR
		{ 
			kp = 280;kd = 280;      // BD== 290-200  XXprev 400,400
			//kp = 144;	k64d = 256;
		}
		else if (chip_no == 3) 	//BACK_RIGHT_MOTOR
		{
			kp = 280;	kd = 280;  //kp280,280
			//kp = 144;	kd = 256;
		}
		else if(chip_no == 4)		//FRONT_RIGHT_MOTOR
		{
			kp = 280;kd = 280;        //280,280
			//kp = 144;	kd = 256;
		}
		if(chip_no == 5)				//FRONT_LEFT_MOTOR
		{
			kp = 12;kd = 25; ki=3;													
			//kp = 144;	kd = 256;		//No Load Conditions
		}
		else if(chip_no == 6)		//BACK_LEFT_MOTOR
		{ 
			kp = 12;	kd = 25; ki=3;
			//kp = 144;	k64d = 256;
		}
		else if (chip_no == 7) 	//BACK_RIGHT_MOTOR
		{
			kp = 42;	kd = 1000; ki=3;  //kp170
			//kp = 144;	kd = 256;
		}
		else if(chip_no == 8)		//FRONT_RIGHT_MOTOR
		{
			kp = 60;	kd = 60;        //50,150                   
			//kp = 144;	kd = 256;
		}
		unsigned char kp1 = 0;
		unsigned char kp2 = 0;
		unsigned char kd1 = 0;
		unsigned char kd2 = 0;
		unsigned char ki1 = 0;
		unsigned char ki2 = 0;
			
		select_chip(chip_no);
		kd1 = (unsigned char)kd;
		kd2 = (unsigned char)(kd >> 8);
		
		kp1 = (unsigned char)kp;
		kp2 = (unsigned char)(kp >> 8);
		
		ki1 = (unsigned char)ki;
		ki2 = (unsigned char)(ki >> 8);
		

		/*
		Bit 15 to 8 -For sampling Interval
		Bit 7-4 	-Not Used
		Bit 3 -Kp will be loaded
		Bit 2 -Ki will be loaded
		Bit 1 -Kd will be loaded
		Bit 0 -IL will be loaded
		*/

		write_cmd(LOAD_FILER_PARA);          //Set Filter Params
		checkBusy();

		write_data(sampling);
		write_data(0x0F);         //PID Tuning
		checkBusy();

		//VALUE OF Kp
		write_data(kp2);
		write_data(kp1);
		checkBusy();

		//VALUE OF Ki
		write_data(ki2);
		write_data(ki1);
		checkBusy();

		//VALUE OF Kd
		write_data(kd2);
		write_data(kd1);
		checkBusy();

		//VALUE OF IL
		write_data(0x08);
		write_data(0x00);
		checkBusy();

		//UPDATE FILTER
		write_cmd(UPDATE_FILTER);
		checkBusy();
}

void load_position_error(uint8_t chip_no, unsigned int error)
{
		char i=2;
		uint8_t error_value[2];

		for(i=2;i>0;i--)
		{
			error_value[i-1] = (int8_t)(error & 0x00FF );
			error = error >> 8;
		}

		select_chip(chip_no);
		write_cmd(INTR_ON_ERROR);          // Absolute Breakpoint
		checkBusy();

		write_data(error_value[0]);
		write_data(error_value[1]);
		checkBusy();
}

void load_position_error_stop(uint8_t chip_no, uint16_t error)
{
		select_chip(chip_no);
		write_cmd(0x1A);          // Absolute Breakpoint
		checkBusy();

		write_data((error>>8)&0xff);
		write_data(error & 0xff);
		checkBusy();
}

void initialise_lm629(uint8_t chip_no, pid_mode lm629_mode)
{
		interrupt_reset(chip_no);
		lcd_gotoxy(1,9);
		lcd_string("I");
		software_reset(chip_no);

		interrupt_reset(chip_no);
		//mask_interrupt(chip_no,lm629_mode);

		set_filter_parameters(chip_no,lm629_mode);
		//load_position_error(chip_no,100);
	  lcd_gotoxy(1,9);
		lcd_string("I");
}

//To Stop motor abruptly in velocity mode
void abrupt_stop(uint8_t chip_no)
{
	select_chip(chip_no);

  write_cmd(LOAD_TRAJ);         // Set Trajectory Params
  checkBusy();

	write_data(0x02);        //Trajectory Control Word
	write_data(0x00);
	checkBusy();

	write_cmd(START_MOTION);	     //STT Motion
	checkBusy();
}

void smooth_stop(uint8_t chip_no)
{
	select_chip(chip_no);

    write_cmd(LOAD_TRAJ);         // Set Trajectory Params
    checkBusy();

	write_data(0x04);
	write_data(0x00);    //Trajectory Control Word
	checkBusy();

	write_cmd(START_MOTION);	     //STT Motion
	checkBusy();

}

void motor_off(uint8_t chip_no)
{
	select_chip(chip_no);
	write_cmd(LOAD_TRAJ);         // Set Trajectory Params
  checkBusy();

	write_data(0x01);
	write_data(0x00);        //Trajectory Control Word
	checkBusy();

	write_cmd(START_MOTION);	     //STT Motion
	checkBusy();

}

uint8_t read_status_byte(uint8_t chip_no)
{
	LM629_GPIO_Port->MODER &= 0xFFFF0000; 				//make input
	select_chip(chip_no);
	__IO uint8_t readtemp;
	clrPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=0;
	_delay_counts(1);
	clrPin(PB2_LM629_RD_GPIO_Port,PB2_LM629_RD_Pin);		//read=0;

	_delay_us(1);

	readtemp = (LM629_GPIO_Port->IDR)&0x00FF;

	setPin(PB2_LM629_RD_GPIO_Port,PB2_LM629_RD_Pin);		//read=1;
	setPin(PD12_LM629_PS_GPIO_Port,PD12_LM629_PS_Pin);		//PS=1;

	return readtemp;

}
//Function to calculate real position of shaft during motion
//Do not use continous use of this command after start motion command
//otherwise abrupt behaviour

float read_real_velocity(uint8_t chip_no)
{
	int16_t v = 0;
	uint8_t buffer[2];
	select_chip(chip_no);

   	write_cmd(READ_REAL_VEL);
	checkBusy();

	buffer[0]=read_data();
	buffer[1]=read_data();
	checkBusy();

	v = (int16_t)(((int16_t)buffer[0])<<8 | buffer[1]);

	return ((float)v/VELOCITY_CONST);
}

uint16_t read_signal_registers(uint8_t chip_no)
{
	uint8_t buffer[2];
	select_chip(chip_no);
  write_cmd(READ_SIG_REG);
	checkBusy();

	buffer[0] = read_data();
	buffer[1] = read_data();
	checkBusy();

	return ((uint16_t)(((uint16_t)buffer[0])<<8 | buffer[1]));
}

float read_desired_velocity(uint8_t chip_no)
{
	int32_t v = 0;
	uint8_t buffer[4];
	select_chip(chip_no);
    write_cmd(READ_DESI_VEL);
	checkBusy();

	buffer[0] = read_data();
	buffer[1] = read_data();
	checkBusy();

	buffer[2] = read_data();
	buffer[3] = read_data();
	checkBusy();

	v =(int32_t)((int32_t)(buffer[0])<<24 |(int32_t)(buffer[1])<<16 | (int32_t)(buffer[2])<<8 | (int32_t)(buffer[3]));
	return ((float)v/VELOCITY_CONST);
}


float read_real_position(uint8_t chip_no)
{
	int32_t d = 0;
	uint8_t buffer[4];
	select_chip(chip_no);
 
	write_cmd(READ_REAL_POS);
	checkBusy();

	buffer[0] = read_data();
	buffer[1] = read_data();
	checkBusy();

	buffer[2] = read_data();
	buffer[3] = read_data();
	checkBusy();

	d = (int32_t)((int32_t)(buffer[0])<<24 |(int32_t)(buffer[1])<<16 | (int32_t)(buffer[2])<<8 | (int32_t)(buffer[3]));

	 if(chip_no==6 || chip_no==5)
	{
		 return ((float)d/JUGS_POSITION_CONST);
	//	return ((float)d);
		
	}
		
	return ((float)d/POSITION_CONST);
}


float read_desired_position(uint8_t chip_no)
{
	int32_t d = 0;
	uint8_t buffer[4];
	select_chip(chip_no);

	write_cmd(READ_DESI_POS);
	checkBusy();

	buffer[0] = read_data();
	buffer[1] = read_data();
	checkBusy();

	buffer[2] = read_data();
	buffer[3] = read_data();
	checkBusy();

	d  = (int32_t)((int32_t)(buffer[0])<<24 |(int32_t)(buffer[1])<<16 | (int32_t)(buffer[2])<<8 | (int32_t)(buffer[3]));
	return ((float)d/POSITION_CONST);
}

void set_index_position(uint8_t chip_no)
{
	select_chip(chip_no);
	write_cmd(SET_INDEX_POS);
	checkBusy();
}

void define_home(uint8_t chip_no)
{
	select_chip(chip_no);
	write_cmd(DEFINE_HOME);
	checkBusy();
}


void set_absolute_breakpoint(uint8_t chip_no, float position_f)
{
    int32_t position=0;
	uint8_t i;

	uint8_t position_value[4];

	position = (int32_t) position_f*POSITION_CONST;
	for(i=4;i>0;i--)
	{
		position_value[i-1] = (int8_t)( position & 0x000000FF );
		position = position >> 8;
	}

	select_chip(chip_no);
	write_cmd(SET_ABS_BRKPT);          // Absolute Breakpoint
    checkBusy();

	write_data(position_value[0]);
    write_data(position_value[1]);
	checkBusy();

	write_data(position_value[2]);
    write_data(position_value[3]);
	checkBusy();
 	interrupt_reset(chip_no);
}

void set_relative_breakpoint(uint8_t chip_no, float position_f)
{
	int32_t position=0;
	uint8_t i;

	uint8_t position_value[4];

	position = (int32_t) position_f*POSITION_CONST;

	for(i=4;i>0;i--)
	{
		position_value[i-1] = (int8_t)( position & 0x000000FF );
		position = position >> 8;
	}

	select_chip(chip_no);
	write_cmd(SET_REL_BRKPT);          // Relative Breakpoint
    checkBusy();

	write_data(position_value[0]);
    write_data(position_value[1]);
	checkBusy();

	write_data(position_value[2]);
    write_data(position_value[3]);
	checkBusy();
}


int32_t read_index_position(uint8_t chip_no)
{
	uint8_t buffer[4];
	select_chip(chip_no);
	write_cmd(READ_INDEX_POS);
	checkBusy();

	buffer[0] = read_data();
	buffer[1] = read_data();
	checkBusy();

	buffer[2] = read_data();
	buffer[3] = read_data();
	checkBusy();

	return (int32_t)((int32_t)(buffer[0])<<24 |(int32_t)(buffer[1])<<16 | (int32_t)(buffer[2])<<8 | (int32_t)(buffer[3]));

}

//Trajectory Loaded ,it increases velocity with certain acceleration and then gives
//interrupt after reaching specified position

void StartMotion(uint8_t chip_no)
{
	select_chip(chip_no);
	write_cmd(START_MOTION);
    checkBusy();
}

void Load_trajectory_position(uint8_t chip_no, abs_real_type rel_abs, float position_f, float velocity_f, float accn_f, value_load_type load_type)
{

	int32_t position, velocity, accn;
	uint8_t i;
	uint8_t accn_value[4],velocity_value[4],position_value[4];

	position = (int32_t) position_f*(float)POSITION_CONST;
	velocity = (int32_t) velocity_f*(float)VELOCITY_CONST;
	accn = (int32_t) accn_f*(float)ACCEL_CONST;

	for(i=4;i>0;i--)
	{
		accn_value[i-1] = (int8_t)( accn & 0x000000FF );
		accn = accn >> 8;

		velocity_value[i-1] = (int8_t)( velocity & 0x000000FF );
		velocity = velocity >> 8;

		position_value[i-1] = (int8_t)( position & 0x000000FF );
		position = position >> 8;
	}

	select_chip(chip_no);
	 /*
	 Bit 15-13 -Not Used
	 Bit 12    -Forward Direction(Velocity Mode Only)
	 Bit 11	   -Velocity Mode
	 Bit 10	   -Stop Smoothly
	 Bit 9 	   -Stop Abruptly
	 Bit 8     -Turn off Motor
	 Bit 7-6   -Not Used
	 Bit 5     -Acceleration Will be loaded
	 Bit 4	   -Acceleration is relative
	 Bit 3     -Velocity Will be loaded
	 Bit 2	   -Velocity is relative
	 Bit 1     -Position Will be loaded
	 Bit 0	   -Position is relative
	 */

	if(load_type == POS_PVA)
	{
		write_cmd(LOAD_TRAJ);          // Set Trajectory Params
		checkBusy();

		write_data(0x00);        //Trajectory Control Word
		if(rel_abs == ABSOLUTE)
			write_data(0x2A);
		else
			write_data(0x2A | 0x15);
		checkBusy();

		//acceleration
		write_data(accn_value[0]);
		write_data(accn_value[1]);
		checkBusy();

		write_data(accn_value[2]);
		write_data(accn_value[3]);
		checkBusy();
		//velocity

		write_data(velocity_value[0]);
		write_data(velocity_value[1]);
		checkBusy();

		write_data(velocity_value[2]);
		write_data(velocity_value[3]);
		checkBusy();

		//Position

		write_data(position_value[0]);
		write_data(position_value[1]);
		checkBusy();

		write_data(position_value[2]);
		write_data(position_value[3]);
		checkBusy();

	}
	else if (load_type == POS_V)
	{
		write_cmd(LOAD_TRAJ);          // Set Trajectory Params
		checkBusy();

		write_data(0x00);        //Trajectory Control Word
		if(rel_abs == ABSOLUTE)
			write_data(0x08);
		else
			write_data(0x08|0x04);
		checkBusy();

		//velocity
		write_data(velocity_value[0]);
		write_data(velocity_value[1]);
		checkBusy();

		write_data(velocity_value[2]);
		write_data(velocity_value[3]);
		checkBusy();

	}
	else if (load_type == POS_PV)
	{
		write_cmd(LOAD_TRAJ);          // Set Trajectory Params
		checkBusy();

		write_data(0x00);        //Trajectory Control Word
		if(rel_abs == ABSOLUTE)
			write_data(0x0A);
		else
			write_data(0x0A |0x05 );
		checkBusy();

		//velocity
		write_data(velocity_value[0]);
		write_data(velocity_value[1]);
		checkBusy();

		write_data(velocity_value[2]);
		write_data(velocity_value[3]);
		checkBusy();

		//Position
		write_data(position_value[0]);
		write_data(position_value[1]);
		checkBusy();

		write_data(position_value[2]);
		write_data(position_value[3]);
		checkBusy();
	}

}

void Load_trajectory_velocity(uint8_t chip_no, abs_real_type rel_abs, float velocity_f, float accn_f, value_load_type load_type)
{
	/*
	 Bit 15-13 -Not Used
	 Bit 12    -Forward Direction(Velocity Mode Only)
	 Bit 11	   -Velocity Mode
	 Bit 10	   -Stop Smoothly
	 Bit 9 	   -Stop Abruptly
	 Bit 8     -Turn off Motor

	 Bit 7	   -Not Used
	 Bit 6	   -Not Used
	 Bit 5     -Acceleration Will be loaded
	 Bit 4	   -Acceleration is relative
	 Bit 3     -Velocity Will be loaded
	 Bit 2	   -Velocity is relative
	 Bit 1     -Position Will be loaded
	 Bit 0	   -Position is relative
	 */

	int32_t velocity, accn;
	uint8_t i;
	uint8_t accn_value[4],velocity_value[4];

	if(chip_no == 5 || chip_no == 6 )
	{
		velocity = (int32_t) velocity_f*(float)JUGS_VELOCITY_CONST;
		accn = (int32_t) accn_f*(float)JUGS_ACCEL_CONST;
	}
	
	else
	{
		velocity = (int32_t) velocity_f*(float)VELOCITY_CONST;
		accn = (int32_t) accn_f*(float)ACCEL_CONST;
	}


	for(i=4;i>0;i--)
	{
		accn_value[i-1] = (int8_t)( accn & 0x000000FF );
		accn = accn >> 8;

		velocity_value[i-1] = (int8_t)( velocity & 0x000000FF );
		velocity = velocity >> 8;
	}

	select_chip(chip_no);
	write_cmd(LOAD_TRAJ);          // Set Trajectory Params
	checkBusy();

	if(load_type == VEL_VA_F)
	{
		write_data(0x18);        //Trajectory Control Word (forward)
		if(rel_abs == ABSOLUTE)
			write_data(0x28);
		else
			write_data(0x28 | 0x14);		//velocity & accel relative
		checkBusy();
	}

	else if(load_type == VEL_VA_R)
	{
		write_data(0x08);        //Trajectory Control Word   (reverse)
		if(rel_abs == ABSOLUTE)
			write_data(0x28);
		else
			write_data(0x28 | 0x14);		//velocity & accel relative
		checkBusy();
	}


	else if(load_type == VEL_V_F)
	{

		write_data(0x18);        //Trajectory Control Word   (forward)
		if(rel_abs == ABSOLUTE)
			write_data(0x08);
		else
			write_data(0x08 | 0x14);		//velocity & accel relative
		checkBusy();
	}

	else if(load_type == VEL_V_R)
	{
		write_data(0x08);        //Trajectory Control Word   (reverse)
		if(rel_abs == ABSOLUTE)
			write_data(0x08);
		else
			write_data(0x08 | 0x14);		//velocity & accel relative
		checkBusy();
	}

	if((load_type == VEL_VA_F) || (load_type == VEL_VA_R))
	{
		//acceleration
		write_data(accn_value[0]);
		write_data(accn_value[1]);
		checkBusy();

		write_data(accn_value[2]);
		write_data(accn_value[3]);
		checkBusy();

	}

	//velocity
	write_data(velocity_value[0]);
	write_data(velocity_value[1]);
	checkBusy();

	write_data(velocity_value[2]);
	write_data(velocity_value[3]);
	checkBusy();

}


//void check_sign(pid_mode lm629_mode)
//{
//	bool flag1=false,flag2=false,flag3=false, flag_out=false;
//	while(lm629_mode == POSITION_MODE && flag_out==false)
//	{
//		if(checkPin(LM629_HI1_GPIO_Port,LM629_HI1_Pin))		//To check for HI Interrupt on pin B1
//		{
//			flag1 = true;
//		}

//		if(checkPin(LM629_HI2_GPIO_Port,LM629_HI2_Pin))		//To check for HI Interrupt on pin B1
//		{
//			flag2 = true;
//		}

//		if(checkPin(LM629_HI3_GPIO_Port,LM629_HI3_Pin))		//To check for HI Interrupt on pin B1
//		{
//			flag3 = true;
//		}

//		if(flag1==true && flag2==true && flag3==true)
//		{
//			flag_out=true;
//		}
//	}
//}

void virtual_hardware_reset(void)
{
	lcd_gotoxy(1,9);
	lcd_string("V");
	software_reset(1);
	lcd_gotoxy(1,9);
	lcd_string("V");
	software_reset(2);
	lcd_gotoxy(1,9);
	lcd_string("V");
	software_reset(3);
	lcd_gotoxy(1,9);
	lcd_string("V");
	software_reset(4);
}

void bot_define_home(void)
{
	define_home(1);
	define_home(2);
	define_home(3);
	define_home(4);
}


int time1=0;
int time2=0;
int position1=0;
int position2=0;
int posdif=0;
int timedef=0;
float jugsvelocity=0;

float read_real_jugs_velocity(uint8_t chip_no)
{
	jugsvelocity=0; 
 	time1=HAL_GetTick();
	timedef =time1-time2;
if(timedef>10)
{
	position1=read_real_position(chip_no);
	posdif =position1-position2;
	jugsvelocity=posdif;
}
//else{
	//read_real_jugs_velocity(chip_no);

	//position2=read_real_position(chip_no);
	position2=position1;
	//time2=HAL_GetTick();
	time2=time1;
	
	return(jugsvelocity);

}
